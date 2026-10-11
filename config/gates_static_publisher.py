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

"""Publish static transformations and mesh markers for gates visualization in RViz."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import os

from drone_course_msgs.msg import Point
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


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
        raise FileNotFoundError(f'Gates configuration file not found: {yaml_path}')

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
            raise ValueError(f"Gate '{gate_name}' must have format [x, y, z, yaw]. Got: {pose}")

        gates.append(
            {
                'name': gate_name,
                'xyz': [float(pose[0]), float(pose[1]), float(pose[2])],
                'rpy': [0.0, 0.0, float(pose[3])],  # roll=0, pitch=0, yaw from config
            }
        )

    return gates


class GatesStaticPublisherNode(Node):
    """Node to publish static TFs and mesh markers for gates."""

    def __init__(
        self,
        use_sim_time: bool,
        marker_freq: float = 1.0,
        dae_path: str = '',
        gates_config: str = '',
    ):
        super().__init__('gates_static_publisher_node')

        # Set use_sim_time parameter
        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        # Static TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Marker publisher for gate meshes
        self.marker_pub = self.create_publisher(MarkerArray, 'gates_static', 10)

        # Set DAE path for markers
        if not dae_path:
            self.dae_path = 'package://as2_gazebo_assets/models/gate/meshes/gate.dae'
        else:
            self.dae_path = dae_path

        # Load gates from configuration file
        if not gates_config:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            gates_config = os.path.join(script_dir, 'gates_config.yaml')

        try:
            self.gates = load_gates_from_yaml(gates_config)
            self.get_logger().info(f'Loaded {len(self.gates)} gates from: {gates_config}')
        except Exception as e:
            self.get_logger().error(f'Failed to load gates configuration: {e}')
            raise

        # Create transforms and markers
        self.transforms = []
        self.marker_array = MarkerArray()
        self.path = []  # Path will be generated after loading gates
        self.load_gates()

        # Generate path from gates
        self.generate_path()

        # Publish static TFs once
        self.publish_tfs()

        # Create timer for periodic marker publishing
        self.timer = self.create_timer(1.0 / marker_freq, self.publish_markers)

        # Create service for path requests
        self.path_service = self.create_service(
            RequestPath, 'request_path', self.path_service_callback
        )

        self.get_logger().info('Gates Static Publisher Node initialized')
        self.get_logger().info(f'  Gates: {len(self.gates)}')
        self.get_logger().info(f'  Use sim time: {use_sim_time}')
        self.get_logger().info(f'  Marker frequency: {marker_freq} Hz')
        self.get_logger().info(f'  DAE path: {self.dae_path}')
        self.get_logger().info(f'  Config file: {gates_config}')
        self.get_logger().info('Path Service is ready to receive requests.')

    def load_gates(self):
        """Load gate data and create transforms and markers."""
        if not self.gates:
            self.get_logger().error('No gates defined.')
            return

        self.get_logger().info(f'Creating transforms and markers for {len(self.gates)} gates')

        for i, gate in enumerate(self.gates):
            # Validate gate data
            if 'name' not in gate or 'xyz' not in gate or 'rpy' not in gate:
                self.get_logger().error(f'Gate {i} is missing required fields (name, xyz, rpy)')
                continue

            # Create transform
            transform = self.create_transform(gate)
            self.transforms.append(transform)

            # Create marker
            marker = self.create_gate_marker(gate['name'], i)
            self.marker_array.markers.append(marker)

            self.get_logger().info(
                f'  Gate {i}: {gate["name"]} at position [{gate["xyz"][0]:.2f}, '
                f'{gate["xyz"][1]:.2f}, {gate["xyz"][2]:.2f}]'
            )

    def create_transform(self, gate: dict) -> TransformStamped:
        """
        Create a TransformStamped object from gate data.

        :param gate: Dictionary containing 'name', 'xyz', and 'rpy' for the gate
        :return: TransformStamped object representing the gate's pose
        """
        t = TransformStamped()
        t.header.frame_id = 'earth'
        t.child_frame_id = gate['name']

        t.transform.translation.x = float(gate['xyz'][0])
        t.transform.translation.y = float(gate['xyz'][1])
        t.transform.translation.z = float(gate['xyz'][2])

        # Convert RPY to quaternion
        q = self.euler_to_quaternion(gate['rpy'][0], gate['rpy'][1], gate['rpy'][2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def publish_tfs(self):
        """Publish static transformations."""
        if not self.transforms:
            self.get_logger().warn('No transforms to publish')
            return

        current_time = self.get_clock().now().to_msg()
        for t in self.transforms:
            t.header.stamp = current_time

        self.tf_broadcaster.sendTransform(self.transforms)
        self.get_logger().info(f'Published {len(self.transforms)} static transforms')

    def create_gate_marker(self, gate_name: str, marker_id: int) -> Marker:
        """Create a gate mesh marker for visualization in RViz."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = gate_name
        marker.ns = 'gate_mesh'
        marker.id = marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = self.dae_path
        marker.mesh_use_embedded_materials = True
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose.position.z = -0.7  # Offset from gate model in SDF

        return marker

    def publish_markers(self):
        """Periodically publish the gate mesh markers."""
        if not self.marker_array.markers:
            return

        current_time = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = current_time

        self.marker_pub.publish(self.marker_array)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
            pitch / 2
        ) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
            pitch / 2
        ) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
            pitch / 2
        ) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
            pitch / 2
        ) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def generate_path(self):
        """Generate the path from gate positions (called once after loading gates)."""
        path_offsets = [-1.0, 1.0, 1.0, -1.0]
        test_crossing_offset = 2.0
        self.path = []
        for i, gate in enumerate(self.gates):
            p = Point()
            p.x = gate['xyz'][0] + path_offsets[i]
            p.y = gate['xyz'][1] 
            p.z = gate['xyz'][2]
            self.path.append(p)

        self.get_logger().info(f'Generated path with {len(self.path)} gate positions.')

    def path_service_callback(self, request, response):
        """
        Service callback function for handling path requests.

        Returns the pre-generated path of all gates in order.

        Args:
            request: The incoming service request.
            response: The service response object to be populated with the path data.

        Returns:
            The populated service response containing the gate positions as a path.
        """
        response.path = self.path
        self.get_logger().info(
            f'Path request received. Returning {len(response.path)} gate positions.'
        )
        return response


def main(args=None):
    """Entrypoint method."""
    parser = argparse.ArgumentParser(
        description='Publish static TFs and mesh markers for gates visualization'
    )

    parser.add_argument(
        '-s',
        '--use_sim_time',
        type=lambda x: x.lower() == 'true',
        default=False,
        help='Use simulation time (true/false)',
    )
    parser.add_argument(
        '-f', '--marker_freq', type=float, default=1.0, help='Frequency for marker publishing (Hz)'
    )
    parser.add_argument(
        '--dae_path',
        type=str,
        default='',
        help='Path to the DAE file (supports package:// format or file:// paths)',
    )
    parser.add_argument(
        '--gates_config', type=str, default='', help='Path to the gates configuration YAML file'
    )

    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = GatesStaticPublisherNode(
        parsed_args.use_sim_time,
        parsed_args.marker_freq,
        parsed_args.dae_path,
        parsed_args.gates_config,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
