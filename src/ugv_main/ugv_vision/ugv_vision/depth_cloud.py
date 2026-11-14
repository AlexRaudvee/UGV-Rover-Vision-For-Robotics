"""Small metric depth cloud, transformed at acquisition time for the Vizanti 3D view."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points

from .depth_preview import depth_in_meters


def project_depth(depth, projection, step, minimum, maximum):
    """Rectified optical coordinates, in metres, with invalid depths removed."""
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[::step, ::step]
    valid = np.isfinite(z) & (z >= minimum) & (z <= maximum)
    z, u, v = z[valid], u[valid], v[valid]
    return np.column_stack(((u - projection[2]) * z / projection[0],
                            (v - projection[6]) * z / projection[5], z)).astype(np.float32)


class DepthCloud(Node):
    def __init__(self):
        super().__init__('depth_cloud')
        for key, value in {'fixed_frame': 'odom', 'rate': 5.0, 'max_points': 5000,
                           'min_depth': 0.15, 'max_depth': 6.0,
                           'max_image_age': 3.0, 'input_transport': 'raw',
                           'depth_topic': 'depth/image', 'camera_info_topic': 'depth/camera_info',
                           'output_topic': 'depth/points'}.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in
                       ('fixed_frame', 'rate', 'max_points', 'min_depth', 'max_depth',
                        'max_image_age', 'input_transport', 'depth_topic',
                        'camera_info_topic', 'output_topic')}
        if (self.config['rate'] <= 0 or self.config['max_points'] < 1 or
                not 0 < self.config['min_depth'] < self.config['max_depth'] or
                self.config['max_image_age'] <= 0 or
                self.config['input_transport'] not in ('raw', 'compressedDepth')):
            raise ValueError('Invalid cloud rate, point limit or depth range')
        self.info, self.image = None, None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, self.config['output_topic'], 1)
        self.create_subscription(CameraInfo, self.config['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        image_type = CompressedImage if self.config['input_transport'] == 'compressedDepth' else Image
        self.create_subscription(image_type, self.config['depth_topic'], self.on_image,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / self.config['rate'], self.publish)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        # Retain a candidate until TF catches up rather than chasing the newest frame.
        if self.image is None:
            self.image = msg

    def publish(self):
        msg, info = self.image, self.info
        if msg is None or info is None:
            return
        try:
            if (msg.header.frame_id != info.header.frame_id or
                    info.p[0] <= 0 or info.p[5] <= 0):
                raise ValueError('Depth and rectified CameraInfo dimensions/frame/intrinsics disagree')
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if not -0.05 <= age <= self.config['max_image_age']:
                self.image = None
                raise ValueError(f'Depth age {age:.2f}s exceeds allowed delay')
            tf = self.buffer.lookup_transform(self.config['fixed_frame'], msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp))
            depth = depth_in_meters(msg, max_width=info.width)
            if depth.shape != (info.height, info.width):
                self.image = None
                raise ValueError('Depth and CameraInfo dimensions disagree')
            step = max(1, math.ceil(math.sqrt(depth.size / self.config['max_points'])))
            points = project_depth(depth, info.p, step, self.config['min_depth'], self.config['max_depth'])
            points = transform_points(points, tf.transform).astype(np.float32)
            self.publisher.publish(create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id=self.config['fixed_frame']), points))
            self.image = None
        except (TransformException, ValueError, TypeError) as error:
            if not isinstance(error, TransformException):
                self.image = None
            self.get_logger().warning(f'Cloud unavailable: {error}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
