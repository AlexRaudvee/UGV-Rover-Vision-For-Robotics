"""Shared ROS adapter for measured RGB/depth and RGB/LiDAR detections."""

from collections import deque
from dataclasses import fields
import json
import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .image_io import annotated_image, decode_depth, depth_preview, publish_jpeg, waiting_image
from .perception import (
    CameraModel, depth_estimate_for_blob, DetectorSettings, find_black_blobs,
    lidar_estimates_for_blobs,
    measure_blob, scan_points_in_camera, validate,
)
from .ranging import TemporalRanges
from .synchronization import select_pair, stamp_ns


def transform_arrays(transform):
    """Convert a TF rigid transform to a rotation matrix and translation."""
    q = transform.rotation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('Invalid camera/LiDAR TF quaternion')
    x, y, z, w = values / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    t = transform.translation
    translation = np.array([t.x, t.y, t.z])
    if not np.isfinite(translation).all():
        raise ValueError('Invalid camera/LiDAR TF translation')
    return rotation, translation


class MeasuredDetector(Node):
    """Share segmentation and publishing while keeping range sources independent."""

    def __init__(self, mode='depth'):
        self.mode = mode
        namespace = 'object_detection' if mode == 'depth' else 'obstacle_detection'
        super().__init__('detector', namespace=namespace)
        # Avoid competing OpenCV pools on the rover's limited CPU cores.
        cv2.setNumThreads(1)
        static = {
            'rgb_topic': '/oak/rgb/image_rect/compressed',
            'rgb_transport': 'compressed',
            'camera_info_topic': '/oak/rgb/camera_info',
            'depth_topic': '/oak/stereo/image_raw/compressedDepth',
            'depth_transport': 'compressedDepth',
            'scan_topic': '/scan', 'rectified': True, 'rate_hz': 5.0, 'max_width': 640,
            'reliable_input': True,
        }
        for name, value in static.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        for field in fields(DetectorSettings):
            self.declare_parameter(field.name, getattr(DetectorSettings(), field.name))
        self.declare_parameter('stale_after_s', 0.8)
        self.declare_parameter('sync_tolerance_s', 0.12)
        result = self.parameter_change([])
        if not result.successful:
            raise ValueError(result.reason)
        if not (0 < self.value('rate_hz') <= 30 and self.value('max_width') >= 160
                and self.value('rgb_transport') in ('raw', 'compressed')
                and self.value('depth_transport') in ('raw', 'compressedDepth')):
            raise ValueError('Invalid processing rate, image width, or transport')
        self.add_on_set_parameters_callback(self.parameter_change)
        self.bridge = CvBridge()
        self.frames = deque(maxlen=12)
        self.measurements = deque(maxlen=20)
        self.camera_info = None
        self.sequence, self.processed = 0, -1
        self.last_good_received = -math.inf
        self.last_header = None
        self.range_filter = TemporalRanges(use_ema=(mode == 'depth'))
        self.range_frame = None
        self.details_pub = self.create_publisher(String, 'detections', 1)
        self.debug_pub = self.create_publisher(CompressedImage, 'debug/compressed', 1)
        self.rgb_pub = self.create_publisher(CompressedImage, 'rgb/compressed', 1)
        self.depth_pub = self.create_publisher(CompressedImage, 'depth/compressed', 1)
        rgb_type = CompressedImage if self.value('rgb_transport') == 'compressed' else Image
        image_qos = QoSProfile(depth=2, reliability=(ReliabilityPolicy.RELIABLE
                               if self.value('reliable_input') else ReliabilityPolicy.BEST_EFFORT))
        self.rgb_sub = self.create_subscription(
            rgb_type, self.value('rgb_topic'), self.receive_rgb, image_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, self.value('camera_info_topic'), self.receive_info,
            image_qos)
        if mode == 'depth':
            sensor_type = (CompressedImage if self.value('depth_transport') == 'compressedDepth'
                           else Image)
            sensor_topic = self.value('depth_topic')
        else:
            sensor_type, sensor_topic = LaserScan, self.value('scan_topic')
        self.sensor_sub = self.create_subscription(
            sensor_type, sensor_topic, self.receive_measurement,
            image_qos if mode == 'depth' else qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) if mode == 'lidar' else None
        self.timer = self.create_timer(1.0 / self.value('rate_hz'), self.tick)
        self.get_logger().info(f'{namespace}: {self.value("rgb_topic")} + {sensor_topic}')

    def value(self, name):
        """Read a ROS parameter."""
        return self.get_parameter(name).value

    def settings(self, overrides=None):
        """Collect the current detector settings."""
        overrides = overrides or {}
        return DetectorSettings(**{field.name: overrides.get(field.name, self.value(field.name))
                                   for field in fields(DetectorSettings)})

    def parameter_change(self, parameters):
        """Validate a complete proposed tuning update."""
        overrides = {parameter.name: parameter.value for parameter in parameters}
        try:
            validate(self.settings(overrides))
            stale = overrides.get('stale_after_s', self.value('stale_after_s'))
            tolerance = overrides.get('sync_tolerance_s', self.value('sync_tolerance_s'))
            if not (math.isfinite(stale) and math.isfinite(tolerance) and 0 < tolerance < stale):
                raise ValueError('Require 0 < sync_tolerance_s < stale_after_s')
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def receive_rgb(self, message):
        """Buffer RGB without doing expensive work in its callback."""
        self.sequence += 1
        self.frames.append((message, time.monotonic(), self.sequence))

    def receive_info(self, message):
        """Retain calibration; calibration can be published only once."""
        self.camera_info = message

    def receive_measurement(self, message):
        """Buffer the independent ranging sensor."""
        self.measurements.append((message, time.monotonic()))

    def check_stamp(self, message):
        """Reject old acquisitions even when their callbacks arrived recently."""
        age = (self.get_clock().now().nanoseconds - stamp_ns(message)) / 1e9
        if stamp_ns(message) <= 0 or not -self.value('sync_tolerance_s') <= age <= self.value(
                'stale_after_s'):
            raise ValueError('Input acquisition timestamp is zero, stale, or in the future')

    def tick(self):
        """Process a synchronized pair or invalidate old output."""
        now = time.monotonic()
        selected = select_pair(self.frames, self.measurements, self.processed, now,
                               self.value('stale_after_s'), self.value('sync_tolerance_s'))
        if selected is None:
            if now - self.last_good_received > self.value('stale_after_s'):
                self.unavailable('Waiting for fresh synchronized RGB + ' + self.mode)
            return
        rgb, sensor, sequence, received = selected
        # A delayed depth packet can still match an earlier buffered RGB frame.
        # Consuming the newest RGB while range is missing would discard that pair.
        if sensor is not None:
            self.processed = sequence
        try:
            self.check_stamp(rgb)
            if self.camera_info is None:
                raise ValueError('Waiting for CameraInfo')
            if not rgb.header.frame_id or self.camera_info.header.frame_id != rgb.header.frame_id:
                raise ValueError('RGB and CameraInfo must use the same optical frame')
            self.last_header = rgb.header
            if self.value('rgb_transport') == 'compressed':
                bgr = self.bridge.compressed_imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            else:
                bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            original_shape = bgr.shape[:2]
            factor = min(1.0, self.value('max_width') / bgr.shape[1])
            if factor < 1:
                bgr = cv2.resize(bgr, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA)
            if self.rgb_pub.get_subscription_count():
                publish_jpeg(self.rgb_pub, bgr, rgb.header)
            camera = CameraModel.from_info(self.camera_info, bgr.shape[1], bgr.shape[0],
                                           self.value('rectified'))
            settings = self.settings()
            blobs, _ = find_black_blobs(bgr, camera, settings)
            if sensor is None:
                raise ValueError('No synchronized ' + self.mode + '; physical width is unknown')
            self.check_stamp(sensor)
            if self.mode == 'depth':
                if sensor.header.frame_id != rgb.header.frame_id:
                    raise ValueError('Depth must be registered to the RGB optical frame')
                depth = decode_depth(sensor, self.bridge,
                                     self.value('depth_transport') == 'compressedDepth')
                if depth.shape != original_shape:
                    raise ValueError('Aligned depth and RGB resolutions differ')
                depth = cv2.resize(depth, (camera.width, camera.height),
                                   interpolation=cv2.INTER_NEAREST)
                estimates = [depth_estimate_for_blob(blob, depth, settings) for blob in blobs]
                if self.depth_pub.get_subscription_count():
                    preview = depth_preview(depth, settings.range_min, settings.range_max)
                    publish_jpeg(self.depth_pub, preview, sensor.header)
            else:
                if not sensor.header.frame_id:
                    raise ValueError('LiDAR scan has no frame')
                transform = self.tf_buffer.lookup_transform(
                    rgb.header.frame_id, sensor.header.frame_id,
                    Time.from_msg(sensor.header.stamp))
                rotation, translation = transform_arrays(transform.transform)
                points = scan_points_in_camera(sensor, rotation, translation)
                estimates = lidar_estimates_for_blobs(blobs, points, camera, settings)
            if self.range_frame != rgb.header.frame_id:
                self.range_filter.clear()
                self.range_frame = rgb.header.frame_id
            estimates = self.range_filter.update(blobs, estimates, stamp_ns(rgb) / 1e9, settings)
            measured, unknown = [], []
            for blob, estimate in zip(blobs, estimates):
                item = measure_blob(blob, estimate.axial_m, self.mode, settings, estimate)
                if item is None:
                    unknown.append(blob.interval())
                else:
                    measured.append(item)
            acquisition_age = max(0, self.get_clock().now().nanoseconds
                                  - min(stamp_ns(rgb), stamp_ns(sensor))) / 1e9
            self.last_good_received = min(received, time.monotonic() - acquisition_age)
            accepted_count = sum(item.accepted for item in measured)
            status = (f'{self.mode}: {len(blobs)} blobs; {accepted_count} accepted; '
                      f'{len(measured) - accepted_count} wrong size; '
                      f'{len(unknown)} uncertain/clipped')
            payload = self.payload(rgb.header, True, status)
            payload.update({
                'field_of_view': list(camera.field_of_view()),
                'detections': [item.to_dict() for item in measured if item.accepted],
                'blockers': [item.to_dict() for item in measured],
                'unknown_intervals': unknown,
                'blob_count': len(blobs), 'measured_count': len(measured),
                'width_band_m': [settings.width_min_m, settings.width_max_m],
                'width_tolerance_m': settings.width_tolerance_m,
            })
            if self.mode == 'depth':
                payload['depth_ema_tau_s'] = settings.depth_ema_tau_s
                payload['range_samples'] = [{
                    'bbox': blob.bbox, 'filter': estimate.filter_method,
                    'raw_distance_m': estimate.raw_axial_m / math.cos(blob.bearing),
                    'distance_m': estimate.axial_m / math.cos(blob.bearing),
                    'ema_alpha': estimate.ema_alpha,
                } for blob, estimate in zip(blobs, estimates)
                    if math.isfinite(estimate.axial_m) and math.isfinite(estimate.raw_axial_m)]
            self.details_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
            if self.debug_pub.get_subscription_count():
                debug = annotated_image(bgr, blobs, measured, status)
                publish_jpeg(self.debug_pub, debug, rgb.header)
        except (ValueError, TypeError, CvBridgeError, cv2.error, TransformException) as error:
            self.unavailable(str(error))
            self.get_logger().warn(str(error), throttle_duration_sec=5.0)

    def payload(self, header, fresh, status):
        """Build the versioned, shared JSON schema used by polar_hist."""
        settings = self.settings()
        return {
            'schema_version': 1, 'source': self.mode, 'fresh': fresh, 'status': status,
            'frame_id': header.frame_id,
            'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
            'range_min': settings.range_min, 'range_max': settings.range_max,
            'field_of_view': None, 'detections': [], 'blockers': [], 'unknown_intervals': [],
        }

    def unavailable(self, status):
        """Invalidate detections and previews immediately on a missing prerequisite."""
        self.last_good_received = -math.inf
        self.range_filter.clear()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.last_header.frame_id if self.last_header else ''
        self.details_pub.publish(String(data=json.dumps(self.payload(header, False, status))))
        publishers = ((self.debug_pub, self.depth_pub) if self.mode == 'depth'
                      else (self.debug_pub,))
        for publisher in publishers:
            if publisher.get_subscription_count():
                publish_jpeg(publisher, waiting_image(status), header)


class ObjectDetector(MeasuredDetector):
    """Detect black 23-30 cm objects using RGB and registered metric depth."""

    def __init__(self):
        super().__init__('depth')


def main(args=None):
    """Run the RGB/depth detector."""
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
