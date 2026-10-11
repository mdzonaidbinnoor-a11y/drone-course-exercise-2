import argparse

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformStamped


class RenderCamera(Node):
    """
    RenderCamera class for managing the camera's pose in a simulation environment.

    This class subscribes to the drone's pose and publishes the camera's pose
    to follow the drone in the simulation.
    """

    def __init__(
        self,
        use_sim_time: bool = True,
        camera_resolution: list[int] = [1920, 1080],
        camera_topic='/drone0/sensor_measurements/image_raw',
        camera_info_topic='/drone0/sensor_measurements/camera_info',
        camera_frame_id='/drone0/camera',
        camera_period=0.01,
        drone_frame_id='/drone0/base_link',
    ):
        """
        Initialize the RenderCamera node.

        Sets up subscribers and publishers for managing the camera's pose.
        """
        super().__init__('render_camera')

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, value=use_sim_time
        )
        self.set_parameters([self.param_use_sim_time])

        self.cv_bridge = CvBridge()

        self.timer = self.create_timer(camera_period, self.publish_camera_image)

        self.img_publisher = self.create_publisher(Image, camera_topic, qos_profile_sensor_data)

        self.cam_info_publisher = self.create_publisher(
            CameraInfo, camera_info_topic, qos_profile_sensor_data
        )

        self.camera_frame_id = camera_frame_id
        self.drone_frame_id = drone_frame_id

        # declare parameter for camera resolution
        self.camera_resolution = camera_resolution

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.get_logger().info(
            f'RenderCamera initialized with camera resolution: {self.camera_resolution}, '
            f'camera topic: {camera_topic}, camera info topic: {camera_info_topic}, '
            f'camera frame ID: {camera_frame_id}, use_sim_time: {use_sim_time}'
            f', camera publishing period: {camera_period} seconds'
        )

        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = self.drone_frame_id
        camera_transform.child_frame_id = self.camera_frame_id
        camera_transform.transform.translation.x = 0.2
        camera_transform.transform.translation.y = 0.0
        camera_transform.transform.translation.z = -0.1
        camera_transform.transform.rotation.x = -0.5
        camera_transform.transform.rotation.y = 0.5
        camera_transform.transform.rotation.z = -0.5
        camera_transform.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(camera_transform)

        self.get_logger().info(
            f'Published camera transform from {self.drone_frame_id} to {self.camera_frame_id} '
            'with translation (0.2, 0.0, -0.1) and no rotation.'
        )

    def publish_camera_image(self):
        """
        Publish a dummy image message to simulate camera output.
        This function creates a blank image and publishes it to the specified topic.
        """

        self.get_logger().info('Publishing camera image and camera info messages...')

        # Create a transform message for the camera

        # cam_info publish
        cam_info_msg = CameraInfo()
        cam_info_msg.width = self.camera_resolution[0]
        cam_info_msg.height = self.camera_resolution[1]
        cam_info_msg.distortion_model = 'plumb_bob'
        cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        cam_info_msg.k = [
            500.0,
            0.0,
            self.camera_resolution[0] / 2.0,
            0.0,
            500.0,
            self.camera_resolution[1] / 2.0,
            0.0,
            0.0,
            1.0,
        ]  # Intrinsic camera matrix
        cam_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
        cam_info_msg.p = [
            500.0,
            0.0,
            self.camera_resolution[0] / 2.0,
            0.0,
            0.0,
            500.0,
            self.camera_resolution[1] / 2.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]  # Projection matrix

        cam_info_msg.header.frame_id = self.camera_frame_id
        self.cam_info_publisher.publish(cam_info_msg)

        self.get_logger().debug(
            'Published camera info message with resolution: '
            f'{self.camera_resolution[0]}x{self.camera_resolution[1]}'
        )

        # Create a blank image (for demonstration purposes)
        img = (
            np.ones((self.camera_resolution[1], self.camera_resolution[0], 3), dtype=np.uint8)
            * 255
        )

        # Convert the OpenCV image to a ROS Image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')

        img_msg.header.frame_id = self.camera_frame_id

        # Publish the image message
        self.img_publisher.publish(img_msg)


if __name__ == '__main__':
    import rclpy

    parser = argparse.ArgumentParser(description='Render Camera Node')
    parser.add_argument('--use_sim_time', type=str, default='false', help='Use simulation time')
    parser.add_argument(
        '--camera_resolution',
        nargs=2,
        type=int,
        default=[1920, 1080],
        help='Camera resolution (width height)',
    )
    parser.add_argument(
        '--camera_topic',
        type=str,
        default='/drone0/sensor_measurements/image_raw',
        help='Topic to publish camera images',
    )
    parser.add_argument(
        '--camera_info_topic',
        type=str,
        default='/drone0/sensor_measurements/camera_info',
        help='Topic to publish camera info',
    )

    parser.add_argument(
        '--drone_frame_id',
        type=str,
        default='/drone0/base_link',
        help='Frame ID for the drone',
    )

    parser.add_argument(
        '--camera_frame_id',
        type=str,
        default='/drone0/camera',
        help='Frame ID for the camera',
    )
    parser.add_argument(
        '--camera_period',
        type=float,
        default=0.1,
        help='Camera publishing period in seconds',
    )

    args = parser.parse_args()

    bool_sim_time = args.use_sim_time.lower() in ['true', '1', 'yes']

    rclpy.init()
    render_camera = RenderCamera(
        use_sim_time=bool_sim_time,
        camera_resolution=args.camera_resolution,
        camera_topic=args.camera_topic,
        camera_info_topic=args.camera_info_topic,
        camera_frame_id=args.camera_frame_id,
        camera_period=args.camera_period,
        drone_frame_id=args.drone_frame_id,
    )

    rclpy.spin(render_camera)
    render_camera.destroy_node()
    rclpy.shutdown()
