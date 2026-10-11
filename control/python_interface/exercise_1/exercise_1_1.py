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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
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

"""Exercise 1.1: position commands for the course simulator.

The simulator must already be running, airborne, and in position mode.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Wait for real localization data before sending commands.
        self.state_pose = None

        # Waypoints taken from the original course template.
        self.path = [
            (4.0, 0.0, 2.1),
            (11.0, 0.0, 2.1),
            (11.0, 5.0, 2.1),
            (4.0, 5.0, 2.1)
        ]

        self.path_idx = 0
        self.tolerance = 0.2
        self.completed = False

        # Receive the drone's measured position.
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone0/self_localization/pose',
            self.state_pose_callback,
            best_effort_qos
        )

        # Send desired positions to Aerostack2.
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone0/motion_reference/pose',
            reliable_qos
        )

        # Run the waypoint logic at 10 Hz.
        self.timer = self.create_timer(0.1, self.run)

        self.get_logger().info(
            'Controller started. Waiting for drone position data.'
        )

    def state_pose_callback(self, msg):
        # The targets below use the earth coordinate frame.
        if msg.header.frame_id != 'earth':
            self.get_logger().warning(
                f'Expected earth frame, received {msg.header.frame_id!r}.',
                throttle_duration_sec=5.0
            )
            return

        self.state_pose = msg

    def run(self):
        if self.state_pose is None:
            return

        target_x, target_y, target_z = self.path[self.path_idx]

        command = PoseStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'earth'

        command.pose.position.x = target_x
        command.pose.position.y = target_y
        command.pose.position.z = target_z

        # Unit quaternion: zero target yaw.
        command.pose.orientation.w = 1.0

        # Continue publishing the final target after completion.
        self.pose_pub.publish(command)

        if self.completed:
            return

        actual = self.state_pose.pose.position
        error = math.sqrt(
            (target_x - actual.x) ** 2
            + (target_y - actual.y) ** 2
            + (target_z - actual.z) ** 2
        )

        if error < self.tolerance:
            self.get_logger().info(
                f'Target {self.path_idx + 1} reached: '
                f'({target_x}, {target_y}, {target_z})'
            )

            if self.path_idx < len(self.path) - 1:
                self.path_idx += 1
                self.get_logger().info(
                    f'Moving to target {self.path_idx + 1}.'
                )
            else:
                self.completed = True
                self.get_logger().info(
                    'All four targets reached. Holding final position.'
                )


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
