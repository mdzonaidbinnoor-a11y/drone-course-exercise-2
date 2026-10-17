#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Exercise 1.2: request gate positions and follow them in position mode.

Run only with the course simulator. Its initialization script handles arming,
 offboard mode and takeoff; this node selects POSITION control itself.
Service definitions: /request_path (RequestPath, empty request, Point[] path)
and /drone0/controller/set_control_mode (SetControlMode).
"""
import math
import time

import rclpy
from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        feedback = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.state_pose = None
        self.last_pose_time = None
        self.path = []
        self.path_idx = 0
        self.tolerance = 0.2
        self.completed = False
        self.control_mode_set = False
        self.path_request_client = self.create_client(RequestPath, '/request_path')
        self.control_mode_client = self.create_client(
            SetControlMode, '/drone0/controller/set_control_mode')
        self.pending = {'path': None, 'mode': None}
        self.started = {'path': 0.0, 'mode': 0.0}
        self.retry_after = {'path': 0.0, 'mode': 0.0}
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone0/self_localization/pose',
            self.state_pose_callback, feedback)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/drone0/motion_reference/pose', reliable)
        self.timer = self.create_timer(0.1, self.run)
        self.get_logger().info('Exercise 1.2 started. Waiting for the course services.')

    def state_pose_callback(self, msg):
        if msg.header.frame_id != 'earth':
            self.get_logger().warning(
                f'Expected earth pose frame, received {msg.header.frame_id!r}.',
                throttle_duration_sec=5.0)
            return
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        self.state_pose = msg
        self.last_pose_time = time.monotonic()

    def poll_service(self, key, client, request_factory):
        """One async request at a time. No blocking waits in timer callbacks."""
        now = time.monotonic()
        future = self.pending[key]
        if future is not None:
            if future.done():
                self.pending[key] = None
                self.retry_after[key] = now + 1.0
                try:
                    return future.result()
                except Exception as exc:
                    self.get_logger().warning(f'{key} request failed: {exc}')
                    return None
            if now - self.started[key] > 10.0:
                client.remove_pending_request(future)
                future.cancel()
                self.pending[key] = None
                self.retry_after[key] = now + 2.0
                self.get_logger().warning(f'{key} response timed out; will retry.')
            return None
        if now < self.retry_after[key]:
            return None
        if not client.service_is_ready():
            self.get_logger().warning(
                f'Waiting for {client.srv_name}. Keep the simulator running.',
                throttle_duration_sec=5.0)
            return None
        try:
            self.pending[key] = client.call_async(request_factory())
            self.started[key] = now
            self.get_logger().info(f'Sent {key} service request.')
        except Exception as exc:
            self.retry_after[key] = now + 2.0
            self.get_logger().warning(f'Could not send {key} request: {exc}')
        return None

    @staticmethod
    def mode_request():
        request = SetControlMode.Request()
        request.control_mode.control_mode = ControlMode.POSITION
        request.control_mode.yaw_mode = ControlMode.YAW_ANGLE
        request.control_mode.reference_frame = ControlMode.LOCAL_ENU_FRAME
        return request

    def run(self):
        if not self.path:
            response = self.poll_service('path', self.path_request_client, RequestPath.Request)
            if response is None:
                return
            points = list(response.path)
            if not points or not all(math.isfinite(v) for p in points for v in (p.x, p.y, p.z)):
                self.get_logger().warning('Path was empty or invalid; will retry.')
                return
            self.path = points
            self.path_idx = 0
            self.get_logger().info(f'Received path with {len(points)} targets.')

        if not self.control_mode_set:
            response = self.poll_service('mode', self.control_mode_client, self.mode_request)
            if response is None:
                return
            if not response.success:
                self.get_logger().warning('Position mode was rejected; will retry.')
                return
            self.control_mode_set = True
            self.get_logger().info('Position control mode set successfully.')

        if self.state_pose is None or time.monotonic() - self.last_pose_time > 2.0:
            self.get_logger().warning('Waiting for fresh drone position data.',
                                      throttle_duration_sec=5.0)
            return

        target = self.path[self.path_idx]
        command = PoseStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'earth'
        command.pose.position.x = float(target.x)
        command.pose.position.y = float(target.y)
        command.pose.position.z = float(target.z)
        command.pose.orientation.w = 1.0
        self.pose_pub.publish(command)
        if self.completed:
            return
        actual = self.state_pose.pose.position
        error = math.sqrt((target.x - actual.x)**2 + (target.y - actual.y)**2
                          + (target.z - actual.z)**2)
        if error < self.tolerance:
            self.get_logger().info(
                f'Target {self.path_idx + 1}/{len(self.path)} reached: '
                f'({target.x:.2f}, {target.y:.2f}, {target.z:.2f})')
            if self.path_idx + 1 < len(self.path):
                self.path_idx += 1
            else:
                self.completed = True
                self.get_logger().info('All targets reached. Holding final position. '
                                       'Verify every gate crossing visually.')


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
