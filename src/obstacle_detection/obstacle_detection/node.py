"""ROS input/output adapter for black-obstacle histograms; no motor commands."""

from collections import deque
from dataclasses import fields
import json
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tf2_ros import StaticTransformBroadcaster

from .histogram import PolarHistogram
from .perception import CameraModel, DetectorSettings, detect_black_objects
from .synchronization import select_rgb, stamp_ns
from .visualization import annotated_image, histogram_image, waiting_image


def validate(settings):
    """Reject tuning values which would make the ROI or detector invalid."""
    values = [getattr(settings, field.name) for field in fields(settings)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('All detector settings must be finite')
    if not (0 < settings.value_max <= 255 and 0 <= settings.saturation_max <= 255
            and 0 < settings.dark_ratio <= 1 and 0 < settings.min_background_value <= 255
            and settings.min_area > 0 and settings.min_width > 0 and settings.min_height > 0
            and 0 < settings.min_fill <= 1 and 0 <= settings.min_bottom_fraction <= 1
            and 0 <= settings.roi_top < settings.roi_bottom <= 1
            and 0 <= settings.side_top < 0.5 and 0 <= settings.side_bottom < 0.5
            and settings.object_width_m > 0 and 0 < settings.range_min < settings.range_max
            and settings.min_depth_pixels > 0):
        raise ValueError('Invalid threshold, ROI, size, or distance setting')


class ObstacleDetector(Node):
    """Publish a measured/estimated polar scan and lightweight Vizanti previews."""

    def __init__(self):
        super().__init__('detector', namespace='obstacle_detection')
        static = {
            'rgb_topic': '/oak/rgb/image_rect',
            'camera_info_topic': '/oak/rgb/camera_info',
            'depth_topic': '/oak/stereo/image_raw',
            'use_depth': True, 'rectified': True,
            'frame_id': 'obstacle_histogram_oak', 'sensor_label': 'OAK RGB + depth',
            'bin_degrees': 5.0, 'rate_hz': 3.0, 'max_width': 640,
            'reliable_input': True,
        }
        for name, value in static.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        defaults = DetectorSettings()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        for name, value in {'stale_after_s': 1.0, 'sync_tolerance_s': 0.12,
                            'smoothing_s': 0.15, 'release_s': 0.5}.items():
            self.declare_parameter(name, value)
        validate(self.settings())
        valid_parameters = self.parameter_change([])
        if not valid_parameters.successful:
            raise ValueError(valid_parameters.reason)
        if not (0 < self.value('rate_hz') <= 10 and self.value('max_width') >= 160
                and self.value('stale_after_s') > 0 and self.value('frame_id')):
            raise ValueError('Invalid processing rate, width, timeout, or frame')
        self.add_on_set_parameters_callback(self.parameter_change)
        self.bridge = CvBridge()
        self.rgb = None
        self.rgb_received = 0.0
        self.rgb_buffer = deque(maxlen=6)
        self.camera_info = None
        self.depth = deque(maxlen=4)
        self.sequence = 0
        self.processed = -1
        self.tf_parent = None
        self.static_tf = StaticTransformBroadcaster(self)
        self.histogram = PolarHistogram(
            self.value('bin_degrees'), self.value('range_min'), self.value('range_max'),
            self.value('smoothing_s'), self.value('release_s'))
        self.scan_pub = self.create_publisher(LaserScan, 'polar_histogram', 1)
        self.values_pub = self.create_publisher(Float32MultiArray, 'histogram_values', 1)
        self.details_pub = self.create_publisher(String, 'detections', 1)
        self.plot_pub = self.create_publisher(CompressedImage, 'histogram/compressed', 1)
        self.debug_pub = self.create_publisher(CompressedImage, 'debug/compressed', 1)
        reliability = (ReliabilityPolicy.RELIABLE if self.value('reliable_input')
                       else ReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(depth=1, reliability=reliability)
        self.rgb_sub = self.create_subscription(Image, self.value('rgb_topic'), self.receive_rgb, qos)
        self.info_sub = self.create_subscription(
            CameraInfo, self.value('camera_info_topic'), self.receive_info, qos)
        self.depth_sub = None
        if self.value('use_depth'):
            self.depth_sub = self.create_subscription(
                Image, self.value('depth_topic'), self.receive_depth, qos)
        self.timer = self.create_timer(1.0 / self.value('rate_hz'), self.tick)
        self.get_logger().info(
            f"Black-obstacle detector: {self.value('rgb_topic')}; "
            f"Vizanti views: {self.plot_pub.topic_name}, {self.debug_pub.topic_name}")

    def value(self, name):
        return self.get_parameter(name).value

    def settings(self, overrides=None):
        overrides = overrides or {}
        return DetectorSettings(**{
            field.name: overrides.get(field.name, self.value(field.name))
            for field in fields(DetectorSettings)})

    def parameter_change(self, parameters):
        overrides = {parameter.name: parameter.value for parameter in parameters}
        try:
            validate(self.settings(overrides))
            for name in ('stale_after_s', 'sync_tolerance_s', 'release_s'):
                if not math.isfinite(overrides.get(name, self.value(name))) or overrides.get(
                        name, self.value(name)) <= 0:
                    raise ValueError(name + ' must be positive and finite')
            smoothing = overrides.get('smoothing_s', self.value('smoothing_s'))
            if not math.isfinite(smoothing) or smoothing < 0:
                raise ValueError('smoothing_s must be finite and nonnegative')
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def receive_rgb(self, message):
        self.rgb = message
        self.rgb_received = time.monotonic()
        self.sequence += 1
        self.rgb_buffer.append((message, self.rgb_received, self.sequence))

    def receive_info(self, message):
        self.camera_info = message

    def receive_depth(self, message):
        self.depth.append((message, time.monotonic()))

    def aligned_depth(self, rgb, shape, now):
        if not self.value('use_depth'):
            return None, 'RGB width estimates'
        usable = [(message, received) for message, received in self.depth
                  if now - received <= self.value('stale_after_s')]
        if not usable:
            return None, 'Depth unavailable; width estimates'
        message, _ = min(usable, key=lambda item: abs(stamp_ns(item[0]) - stamp_ns(rgb)))
        if abs(stamp_ns(message) - stamp_ns(rgb)) > self.value('sync_tolerance_s') * 1e9:
            return None, 'Depth not synchronized; width estimates'
        if ((message.width, message.height) != (rgb.width, rgb.height)
                or message.header.frame_id != rgb.header.frame_id):
            return None, 'Depth alignment mismatch; width estimates'
        if message.encoding not in ('16UC1', '32FC1'):
            return None, 'Unsupported depth encoding; width estimates'
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough').astype(np.float32)
        if message.encoding == '16UC1':
            depth *= 0.001
        return cv2.resize(depth, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST), 'RGB + depth'

    def publish_frame(self, parent):
        if not parent or parent == self.value('frame_id'):
            raise ValueError('RGB image must have a distinct, nonempty optical frame')
        if parent == self.tf_parent:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = self.value('frame_id')
        # Child X is optical Z (forward), child Y is optical -X (left).
        transform.transform.rotation.x = 0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = 0.5
        transform.transform.rotation.w = 0.5
        self.static_tf.sendTransform(transform)
        self.tf_parent = parent

    def tick(self):
        now = time.monotonic()
        if self.rgb is None or now - self.rgb_received > self.value('stale_after_s'):
            message = 'Waiting for RGB images' if self.rgb is None else 'RGB stream stale: histogram unknown'
            self.unavailable(message)
            return
        if self.sequence == self.processed:
            return
        selected = select_rgb(
            self.rgb_buffer, self.depth, self.processed, now,
            self.value('stale_after_s'), self.value('sync_tolerance_s'), self.value('use_depth'))
        if selected is None:
            return
        rgb, _, sequence = selected
        if self.camera_info is None:
            self.unavailable('Waiting for camera calibration')
            return
        try:
            if self.camera_info.header.frame_id != rgb.header.frame_id:
                raise ValueError('RGB and calibration frame IDs differ')
            bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            factor = min(1.0, self.value('max_width') / bgr.shape[1])
            if factor < 1:
                bgr = cv2.resize(bgr, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
            camera = CameraModel.from_info(
                self.camera_info, bgr.shape[1], bgr.shape[0], self.value('rectified'))
            depth, status = self.aligned_depth(rgb, bgr.shape, now)
            settings = self.settings()
            detections, mask, polygon, threshold = detect_black_objects(bgr, camera, settings, depth)
            self.publish_frame(rgb.header.frame_id)
            self.histogram.range_min, self.histogram.range_max = settings.range_min, settings.range_max
            self.histogram.smoothing_s = self.value('smoothing_s')
            self.histogram.release_s = self.value('release_s')
            self.histogram.update(detections, camera.field_of_view(), now)
            self.publish_data(detections, status, rgb.header.stamp, True)
            if self.debug_pub.get_subscription_count():
                debug = annotated_image(bgr, detections, mask, polygon, threshold, status)
                self.publish_jpeg(self.debug_pub, debug, rgb.header)
            self.processed = sequence
        except (ValueError, TypeError, CvBridgeError, cv2.error) as error:
            self.unavailable(str(error))
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)

    def unavailable(self, status):
        self.histogram.clear_unknown()
        self.publish_data([], status, self.get_clock().now().to_msg(), False)
        if self.debug_pub.get_subscription_count():
            self.publish_jpeg(self.debug_pub, waiting_image(status))

    def publish_data(self, detections, status, stamp, fresh):
        histogram = self.histogram
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.value('frame_id')
        scan.angle_min, scan.angle_max = float(histogram.angles[0]), float(histogram.angles[-1])
        scan.angle_increment = histogram.step
        scan.scan_time = 1.0 / self.value('rate_hz')
        scan.range_min, scan.range_max = histogram.range_min, histogram.range_max
        ranges = histogram.ranges.copy()
        ranges[np.isfinite(ranges) & (ranges > scan.range_max)] = np.inf
        ranges[np.isfinite(ranges) & (ranges < scan.range_min)] = scan.range_min
        scan.ranges = ranges.tolist()
        scan.intensities = histogram.scores.tolist()
        self.scan_pub.publish(scan)
        values = Float32MultiArray()
        values.layout.dim = [MultiArrayDimension(
            label='camera-relative angular score bins; radians; positive=left',
            size=len(histogram.angles), stride=len(histogram.angles))]
        values.data = histogram.values().tolist()
        self.values_pub.publish(values)
        details = {
            'fresh': fresh, 'status': status, 'frame_id': scan.header.frame_id,
            'sensor': self.value('sensor_label'), 'angle_min_rad': scan.angle_min,
            'angle_increment_rad': scan.angle_increment,
            'observed': histogram.observed.tolist(),
            'scores': [float(x) if np.isfinite(x) else None for x in histogram.values()],
            'ranges_m': [float(x) if np.isfinite(x) else None for x in histogram.ranges],
            'detections': [item.to_dict() for item in detections],
        }
        self.details_pub.publish(String(data=json.dumps(details, allow_nan=False)))
        if self.plot_pub.get_subscription_count():
            image = histogram_image(histogram, detections, self.value('sensor_label'), status, fresh)
            self.publish_jpeg(self.plot_pub, image, scan.header)

    def publish_jpeg(self, publisher, image, header=None):
        success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not success:
            return
        message = CompressedImage()
        if header is not None:
            message.header = header
        else:
            message.header.stamp = self.get_clock().now().to_msg()
        message.format = 'bgr8; jpeg compressed bgr8'
        message.data = encoded.tobytes()
        publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
