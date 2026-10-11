#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
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

"""Track gate crossings, measure times, and publish visualization markers in RViz."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import numpy as np
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


# Gate dimensions
GATES_HEIGHT = 1.2  # Height of the gates
GATES_WIDTH = 1.3   # Width of the gates


def load_gates_from_yaml(yaml_path: str) -> list:
    """
    Load gates configuration from a YAML file.
    
    Expected YAML format:
    gates_poses:
      gate01: [x, y, z, yaw]
      gate02: [x, y, z, yaw]
      ...
    
    :param yaml_path: Path to the YAML configuration file
    :return: List of gate dictionaries with 'name', 'xyz', and 'rpy' keys
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Gates configuration file not found: {yaml_path}")
    
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)
    
    if 'gates_poses' not in config:
        raise ValueError("YAML file must contain 'gates_poses' key")
    
    gates = []
    gates_poses = config['gates_poses']
    
    # Sort gates by name to ensure consistent ordering
    for gate_name in sorted(gates_poses.keys()):
        pose = gates_poses[gate_name]
        
        if not isinstance(pose, list) or len(pose) != 4:
            raise ValueError(
                f"Gate '{gate_name}' must have format [x, y, z, yaw]. Got: {pose}"
            )
        
        gates.append({
            'name': gate_name,
            'xyz': [float(pose[0]), float(pose[1]), float(pose[2])],
            'rpy': [0.0, 0.0, float(pose[3])]  # roll=0, pitch=0, yaw from config
        })
    
    return gates


class GateTracker:
    """Utility class to track gate crossings, measure times, and generate visualization markers."""
    
    def __init__(self, gates: list, logger, clock):
        """
        Initialize the gate tracker.
        
        :param gates: List of gate dictionaries with 'name', 'xyz', and 'rpy' keys
        :param logger: ROS2 logger instance
        :param clock: ROS2 clock instance
        """
        self.gates = gates
        self.logger = logger
        self.clock = clock
        
        # Gate tracking state
        self.gate_crossed = [False] * len(gates)
        self.gate_times = [None] * len(gates)
        self.current_gate_index = 0
        self.start_time = None
        self.drone_pose = None
        self.previous_distance_sign = [None] * len(gates)
        
        # Multi-lap tracking
        self.current_lap = 0
        self.lap_times = []  # List of lists, each containing times for each gate in that lap
        self.completed_laps = []  # List of completed lap times (from start to finish)
        
        # Detection parameters
        self.gate_detection_radius = 1.5  # Radius to consider drone is near the gate
        self.gate_width = GATES_WIDTH  # Gate width to consider valid crossing (meters)
        self.gate_height = GATES_HEIGHT  # Max height difference from gate center (meters)
        self.crossing_threshold = 0.2  # Distance threshold for plane crossing (meters) - 20cm
    
    def update_drone_pose(self, pose: PoseStamped):
        """
        Update the current drone pose and check for gate crossings.
        
        :param pose: PoseStamped message with drone position
        """
        self.drone_pose = pose
        
        # Only check if there are gates left to cross
        if self.current_gate_index < len(self.gates):
            self.check_gate_crossing()
        
        # Log tracking state periodically (every 100 messages)
        if not hasattr(self, '_pose_count'):
            self._pose_count = 0
        self._pose_count += 1
        if self._pose_count % 100 == 0:
            gate_name = self.gates[self.current_gate_index]['name'] if self.current_gate_index < len(self.gates) else 'COMPLETED'
            self.logger.info(
                f"Lap {self.current_lap + 1}, Tracking gate {self.current_gate_index} ({gate_name}), "
                f"Drone at [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}]"
            )
    
    def check_gate_crossing(self):
        """Detect if the drone has crossed the current gate."""
        if self.drone_pose is None or self.current_gate_index >= len(self.gates):
            return
        
        gate = self.gates[self.current_gate_index]
        gate_pos = np.array(gate['xyz'])
        drone_pos = np.array([
            self.drone_pose.pose.position.x,
            self.drone_pose.pose.position.y,
            self.drone_pose.pose.position.z
        ])
        
        # Calculate vector from gate to drone
        to_drone = drone_pos - gate_pos
        
        # Get gate orientation (normal vector to the gate plane)
        gate_rpy = gate['rpy']
        # Normal vector points in the local X direction of the gate (after rotation)
        cos_yaw = np.cos(gate_rpy[2])
        sin_yaw = np.sin(gate_rpy[2])
        normal = np.array([cos_yaw, sin_yaw, 0.0])
        
        # Project drone onto normal axis (perpendicular distance to plane)
        distance_along_normal = np.dot(to_drone, normal)

        # Gate local axes
        cos_yaw = np.cos(gate_rpy[2])
        sin_yaw = np.sin(gate_rpy[2])
        normal    = np.array([cos_yaw, sin_yaw, 0.0])   # gate forward (X local)
        gate_side = np.array([-sin_yaw, cos_yaw, 0.0])  # gate width axis (Y local)
        gate_up   = np.array([0.0, 0.0, 1.0])           # gate height axis (Z)

        # Signed offsets along each gate axis
        distance_along_normal = np.dot(to_drone, normal)
        distance_lateral      = np.dot(to_drone, gate_side)   # along gate width
        distance_vertical     = np.dot(to_drone, gate_up)     # along gate height

        # Check if drone is inside the rectangular gate opening
        within_width  = abs(distance_lateral)  < self.gate_width  / 2.0
        within_height = abs(distance_vertical) < self.gate_height / 2.0
        
        # Store the sign of the distance for tracking
        current_sign = np.sign(distance_along_normal) if abs(distance_along_normal) > 0.01 else 0
        prev_sign = self.previous_distance_sign[self.current_gate_index]
        
        if within_width and within_height:
            current_sign = np.sign(distance_along_normal) if abs(distance_along_normal) > 0.01 else 0
            prev_sign = self.previous_distance_sign[self.current_gate_index]

            # Debug logging when drone is close to gate
            if abs(distance_lateral) < self.gate_width and abs(distance_vertical) < self.gate_height:
                self.logger.info(
                    f"Gate {self.current_gate_index}: lateral={distance_lateral:.3f}m, "
                    f"vertical={distance_vertical:.3f}m, "
                    f"dist_normal={distance_along_normal:.3f}m, "
                    f"prev_sign={prev_sign}, curr_sign={current_sign}"
                )

            if prev_sign is not None and prev_sign < 0 and current_sign > 0 and abs(distance_along_normal) < self.crossing_threshold:
                self.logger.info(
                    f"Gate {self.current_gate_index} crossed in correct direction! "
                    f"(front to back, dist={distance_along_normal:.3f}m)"
                )
                self.register_gate_crossing()
                return

            if prev_sign is not None and prev_sign > 0 and current_sign < 0:
                self.logger.warn(
                    f"Gate {self.current_gate_index}: Wrong direction detected (back to front) - not counting"
                )

            if current_sign != 0:
                self.previous_distance_sign[self.current_gate_index] = current_sign

            # Update sign if near gate (for initial detection)
            if abs(distance_lateral) < self.gate_detection_radius and current_sign != 0:
                if self.previous_distance_sign[self.current_gate_index] is None:
                    self.previous_distance_sign[self.current_gate_index] = current_sign
                    self.logger.info(f"Gate {self.current_gate_index}: Initial sign set to {current_sign}")
    
    def register_gate_crossing(self):
        """Register that the drone has crossed the current gate."""
        current_time = self.clock.now()
        
        # If first gate, start/restart the timer
        if self.current_gate_index == 0:
            self.start_time = current_time
            if self.current_lap == 0:
                self.logger.info(
                    f"Starting circuit! Lap {self.current_lap + 1}, Gate {self.current_gate_index}: "
                    f"{self.gates[self.current_gate_index]['name']}"
                )
            else:
                self.logger.info(
                    f"Starting Lap {self.current_lap + 1}! Gate {self.current_gate_index}: "
                    f"{self.gates[self.current_gate_index]['name']}"
                )
        else:
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            self.logger.info(
                f"Lap {self.current_lap + 1}, Gate {self.current_gate_index} crossed! "
                f"{self.gates[self.current_gate_index]['name']} - Time: {elapsed:.2f}s"
            )
        
        # Mark gate as crossed and save time
        self.gate_crossed[self.current_gate_index] = True
        self.gate_times[self.current_gate_index] = current_time
        
        # Move to next gate
        self.current_gate_index += 1
        
        # If it was the last gate, complete the lap and reset for next lap
        if self.current_gate_index >= len(self.gates):
            total_time = (current_time - self.start_time).nanoseconds / 1e9
            self.logger.info(f"LAP {self.current_lap + 1} COMPLETED! Time: {total_time:.2f}s")
            
            # Save lap times
            lap_gate_times = []
            for i in range(len(self.gates)):
                if self.gate_times[i] is not None:
                    elapsed = (self.gate_times[i] - self.start_time).nanoseconds / 1e9
                    lap_gate_times.append(elapsed)
                else:
                    lap_gate_times.append(None)
            
            self.lap_times.append(lap_gate_times)
            self.completed_laps.append(total_time)
            
            # Reset for next lap
            self.current_lap += 1
            self.current_gate_index = 0
            self.gate_crossed = [False] * len(self.gates)
            self.gate_times = [None] * len(self.gates)
            self.previous_distance_sign = [None] * len(self.gates)
            
            self.logger.info(f"Ready for Lap {self.current_lap + 1}!")
    
    def create_path_line_marker(self) -> Marker:
        """Create red line marker connecting gate centers."""
        marker = Marker()
        marker.header.stamp = self.clock.now().to_msg()
        marker.header.frame_id = 'earth'
        marker.ns = 'path'
        marker.id = 1000
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add all gate points
        for gate in self.gates:
            point = Point()
            point.x = float(gate['xyz'][0])
            point.y = float(gate['xyz'][1])
            point.z = float(gate['xyz'][2])
            marker.points.append(point)
        
        # Line style
        marker.scale.x = 0.05  # Line thickness
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker
    
    def create_gate_status_marker(self, gate_index: int) -> Marker:
        """Create status rectangle marker for a gate."""
        marker = Marker()
        marker.header.stamp = self.clock.now().to_msg()
        marker.header.frame_id = self.gates[gate_index]['name']
        marker.ns = 'gate_status'
        marker.id = 2000 + gate_index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Rectangle size (slightly smaller than gate)
        marker.scale.x = 0.05  # Thickness
        marker.scale.y = self.gate_width
        marker.scale.z = self.gate_height
        
        # Position centered on gate
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Color based on state
        if self.gate_crossed[gate_index]:
            # Green: Crossed
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
        elif gate_index == self.current_gate_index:
            # Blue: Next to cross
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
        else:
            # Red: Not crossed
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
        
        return marker
    
    def create_gate_time_marker(self, gate_index: int) -> Marker:
        """Create text marker with crossing time."""
        marker = Marker()
        marker.header.stamp = self.clock.now().to_msg()
        marker.header.frame_id = self.gates[gate_index]['name']
        marker.ns = 'gate_time'
        marker.id = 3000 + gate_index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above gate
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
        
        # Build text with all lap times
        text_lines = []
        
        # Add completed lap times
        for lap_idx, lap_gate_times in enumerate(self.lap_times):
            if gate_index == 0:
                text_lines.append(f"L{lap_idx + 1}: START")
            elif gate_index < len(lap_gate_times) and lap_gate_times[gate_index] is not None:
                text_lines.append(f"L{lap_idx + 1}: {lap_gate_times[gate_index]:.2f}s")
        
        # Add current lap time
        if gate_index == 0:
            if self.gate_crossed[0]:
                text_lines.append(f"L{self.current_lap + 1}: START")
            else:
                text_lines.append(f"L{self.current_lap + 1}: ---")
        elif self.gate_crossed[gate_index] and self.gate_times[gate_index] is not None:
            elapsed = (self.gate_times[gate_index] - self.start_time).nanoseconds / 1e9
            text_lines.append(f"L{self.current_lap + 1}: {elapsed:.2f}s")
        else:
            text_lines.append(f"L{self.current_lap + 1}: ---")
        
        marker.text = "\n".join(text_lines)
        
        # Text style
        marker.scale.z = 0.4  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def get_visualization_markers(self) -> list:
        """
        Get all visualization markers (path line, status rectangles, and time texts).
        
        :return: List of Marker objects
        """
        markers = []
        
        # Add path line
        markers.append(self.create_path_line_marker())
        
        # Add status and time markers for each gate
        for i in range(len(self.gates)):
            markers.append(self.create_gate_status_marker(i))
            markers.append(self.create_gate_time_marker(i))
        
        return markers


class GatesTrackerNode(Node):
    """Node to track gate crossings and publish visualization markers."""

    def __init__(self, use_sim_time: bool, marker_freq: float = 10.0, 
                 gates_config: str = '', pose_topic: str = '/drone0/self_localization/pose'):
        super().__init__('gates_tracker_node')
        
        # Set use_sim_time parameter
        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])
        
        # Marker publisher for tracking visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'gates_tracking', 10)
        
        # QoS profile for sensor data (compatible with typical pose publishers)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Drone pose subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            sensor_qos
        )
        
        # Load gates from configuration file
        if not gates_config:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            gates_config = os.path.join(script_dir, 'gates_config.yaml')
        
        try:
            gates = load_gates_from_yaml(gates_config)
            self.get_logger().info(f'Loaded {len(gates)} gates from: {gates_config}')
        except Exception as e:
            self.get_logger().error(f'Failed to load gates configuration: {e}')
            raise
        
        # Initialize gate tracker
        self.gate_tracker = GateTracker(gates, self.get_logger(), self.get_clock())
        
        # Create timer for periodic marker publishing
        self.timer = self.create_timer(1.0 / marker_freq, self.publish_markers)
        
        self.get_logger().info('Gates Tracker Node initialized')
        self.get_logger().info(f'  Gates: {len(gates)}')
        self.get_logger().info(f'  Use sim time: {use_sim_time}')
        self.get_logger().info(f'  Marker frequency: {marker_freq} Hz')
        self.get_logger().info(f'  Pose topic: {pose_topic}')
        self.get_logger().info(f'  Config file: {gates_config}')

    def pose_callback(self, msg: PoseStamped):
        """Callback to process drone pose and detect gate crossings."""
        self.gate_tracker.update_drone_pose(msg)

    def publish_markers(self):
        """Periodically publish the tracking visualization markers."""
        tracking_markers = self.gate_tracker.get_visualization_markers()
        
        marker_array = MarkerArray()
        marker_array.markers = tracking_markers
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    """Entrypoint method."""
    parser = argparse.ArgumentParser(
        description='Track gate crossings and publish visualization markers')
    
    parser.add_argument('-s', '--use_sim_time',
                        type=lambda x: x.lower() == 'true',
                        default=False,
                        help='Use simulation time (true/false)')
    parser.add_argument('-f', '--marker_freq',
                        type=float,
                        default=10.0,
                        help='Frequency for marker publishing (Hz)')
    parser.add_argument('--gates_config',
                        type=str,
                        default='',
                        help='Path to the gates configuration YAML file')
    parser.add_argument('--pose_topic',
                        type=str,
                        default='/drone0/self_localization/pose',
                        help='Topic name for drone pose')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = GatesTrackerNode(
        parsed_args.use_sim_time,
        parsed_args.marker_freq,
        parsed_args.gates_config,
        parsed_args.pose_topic
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
