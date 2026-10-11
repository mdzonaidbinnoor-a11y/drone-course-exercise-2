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

"""Exercise 2: follow the course path using PID speed commands.

Start the simulator and prepare the simulated drone for flight first.
This node does not arm the drone or perform takeoff.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import PoseStamped, TwistStamped


class DroneController(Node):
    def __init__(self):
        super().__init__("drone_controller")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/drone0/self_localization/pose",
            self.pose_callback,
            best_effort_qos,
        )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/drone0/motion_reference/twist",
            reliable_qos,
        )

        self.path_client = self.create_client(
            RequestPath, "/request_path"
        )

        self.mode_client = self.create_client(
            SetControlMode,
            "/drone0/controller/set_control_mode",
        )

        # Starting PID gains.
        self.kp = 0.6
        self.ki = 0.02
        self.kd = 0.15

        self.max_speed = 0.8          # Total speed limit: m/s
        self.integral_limit = 2.0
        self.target_tolerance = 0.2   # Distance to target: metres

        self.position = None
        self.measured_velocity = [0.0, 0.0, 0.0]
        self.last_pose_time = None

        self.integral = [0.0, 0.0, 0.0]
        self.last_control_time = None

        self.path = []
        self.path_idx = 0
        self.finished = False

        self.path_future = None
        self.mode_future = None
        self.control_mode_set = False

        self.next_path_attempt = 0.0
        self.next_mode_attempt = 0.0

        # Async service calls avoid blocking the control loop.
        self.timer = self.create_timer(0.02, self.run)

        self.get_logger().info(
            "Controller started. Waiting for pose and course path."
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_callback(self, msg):
        new_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

        if not all(math.isfinite(v) for v in new_position):
            return

        now = self.now_seconds()

        if self.position is not None and self.last_pose_time is not None:
            dt = now - self.last_pose_time

            if 0.0001 < dt <= 0.2:
                # Filter the estimated velocity to reduce D-term noise.
                for i in range(3):
                    raw_velocity = (
                        new_position[i] - self.position[i]
                    ) / dt
                    self.measured_velocity[i] = (
                        0.7 * self.measured_velocity[i]
                        + 0.3 * raw_velocity
                    )
            elif dt <= 0.0 or dt > 0.2:
                self.measured_velocity = [0.0, 0.0, 0.0]

        self.position = new_position
        self.last_pose_time = now

    def prepare_path(self, now):
        if self.path:
            return True

        if self.path_future is not None:
            if not self.path_future.done():
                return False

            try:
                response = self.path_future.result()
                points = list(response.path)

                if not points:
                    raise ValueError("Path service returned no targets")

                if not all(
                    math.isfinite(value)
                    for point in points
                    for value in (point.x, point.y, point.z)
                ):
                    raise ValueError("Path contains invalid coordinates")

                self.path = points
                self.get_logger().info(
                    f"Received {len(self.path)} path targets."
                )
            except Exception as exc:
                self.get_logger().error(f"Path request failed: {exc}")
                self.next_path_attempt = now + 2.0
            finally:
                self.path_future = None

            return bool(self.path)

        if now < self.next_path_attempt:
            return False

        self.next_path_attempt = now + 2.0

        if not self.path_client.service_is_ready():
            self.get_logger().warn("Waiting for /request_path service.")
            return False

        self.get_logger().info("Requesting course path...")
        self.path_future = self.path_client.call_async(
            RequestPath.Request()
        )
        return False

    def prepare_control_mode(self, now):
        if self.control_mode_set:
            return True

        if self.mode_future is not None:
            if not self.mode_future.done():
                return False

            try:
                response = self.mode_future.result()
                self.control_mode_set = bool(response.success)

                if self.control_mode_set:
                    self.get_logger().info(
                        "Speed control mode set successfully."
                    )
                else:
                    self.get_logger().warn(
                        "Speed mode rejected. Retrying shortly."
                    )
            except Exception as exc:
                self.get_logger().error(f"Mode request failed: {exc}")
            finally:
                self.mode_future = None
                self.next_mode_attempt = now + 2.0

            return self.control_mode_set

        if now < self.next_mode_attempt:
            return False

        self.next_mode_attempt = now + 2.0

        if not self.mode_client.service_is_ready():
            self.get_logger().warn("Waiting for control-mode service.")
            return False

        request = SetControlMode.Request()
        request.control_mode.control_mode = ControlMode.SPEED
        request.control_mode.yaw_mode = ControlMode.YAW_SPEED
        request.control_mode.reference_frame = (
            ControlMode.LOCAL_ENU_FRAME
        )

        self.get_logger().info("Requesting speed control mode...")
        self.mode_future = self.mode_client.call_async(request)
        return False

    def publish_velocity(self, commands):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "earth"

        msg.twist.linear.x = float(commands[0])
        msg.twist.linear.y = float(commands[1])
        msg.twist.linear.z = float(commands[2])

        # Zero yaw speed.
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)

    def run(self):
        now = self.now_seconds()

        if not self.prepare_path(now):
            return

        if not self.prepare_control_mode(now):
            return

        if self.position is None:
            self.get_logger().warn(
                "Waiting for drone position.",
                throttle_duration_sec=2.0,
            )
            return

        # Stop commanding movement if position feedback stops arriving.
        pose_age = now - self.last_pose_time
        if pose_age < 0.0 or pose_age > 0.5:
            self.publish_velocity([0.0, 0.0, 0.0])
            self.last_control_time = None
            self.integral = [0.0, 0.0, 0.0]
            self.get_logger().warn(
                "Position feedback is stale. Sending zero velocity.",
                throttle_duration_sec=2.0,
            )
            return

        if self.finished:
            self.publish_velocity([0.0, 0.0, 0.0])
            return

        target = self.path[self.path_idx]
        target_position = [target.x, target.y, target.z]

        errors = [
            target_position[i] - self.position[i]
            for i in range(3)
        ]

        distance = math.sqrt(sum(e * e for e in errors))

        if distance < self.target_tolerance:
            self.get_logger().info(
                f"Reached target {self.path_idx + 1}/{len(self.path)}"
            )

            self.integral = [0.0, 0.0, 0.0]
            self.last_control_time = None

            if self.path_idx + 1 < len(self.path):
                self.path_idx += 1
                self.publish_velocity([0.0, 0.0, 0.0])
            else:
                self.finished = True
                self.publish_velocity([0.0, 0.0, 0.0])
                self.get_logger().info(
                    "All targets reached. Sending zero velocity. "
                    "Verify that every gate was crossed."
                )
            return

        dt = (
            0.0
            if self.last_control_time is None
            else now - self.last_control_time
        )
        self.last_control_time = now

        candidate_integral = self.integral.copy()

        if 0.0 < dt <= 0.2:
            for i in range(3):
                candidate_integral[i] = max(
                    -self.integral_limit,
                    min(
                        self.integral_limit,
                        self.integral[i] + errors[i] * dt,
                    ),
                )

        # P: move toward the target.
        # I: correct persistent position error.
        # D: oppose measured motion to reduce overshoot.
        # For a fixed target, error derivative = -measured velocity.
        commands = [
            self.kp * errors[i]
            + self.ki * candidate_integral[i]
            - self.kd * self.measured_velocity[i]
            for i in range(3)
        ]

        speed = math.sqrt(sum(v * v for v in commands))

        if speed > self.max_speed:
            # Preserve direction while limiting total speed.
            commands = [
                v * self.max_speed / speed for v in commands
            ]
            # Freeze integral while speed is saturated.
        else:
            self.integral = candidate_integral

        self.publish_velocity(commands)

        self.get_logger().info(
            f"Target {self.path_idx + 1}/{len(self.path)} | "
            f"error={distance:.2f} m | "
            f"velocity=({commands[0]:.2f}, "
            f"{commands[1]:.2f}, {commands[2]:.2f}) m/s",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            controller.publish_velocity([0.0, 0.0, 0.0])
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
