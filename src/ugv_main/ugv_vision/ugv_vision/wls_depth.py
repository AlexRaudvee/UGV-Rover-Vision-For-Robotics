"""Shared RGB-guided WLS depth for detection, projection and previews."""

import struct
import time
from copy import deepcopy
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, RegionOfInterest

from .depth_preview import colorize_depth, depth_in_meters


def filtered_camera_info(info, header, shape, stride):
    """Calibration for the exact subsampling used by depth_in_meters."""
    if shape != (math.ceil(info.height / stride), math.ceil(info.width / stride)):
        raise ValueError('WLS depth dimensions disagree with source calibration')
    result = deepcopy(info)
    result.header, (result.height, result.width) = header, shape
    for index in range(6):
        result.k[index] /= stride
    for index in range(8):
        result.p[index] /= stride
    result.roi = RegionOfInterest()
    result.binning_x = result.binning_y = 0
    return result


def smooth_depth(depth, guide, strength=8000.0, sigma_color=1.5, fill_holes=False):
    """Smooth inverse depth with normalized weights so missing pixels are not zero range."""
    if depth.shape != guide.shape[:2]:
        raise ValueError('Depth and RGB guide dimensions differ')
    valid = np.isfinite(depth) & (depth > 0)
    inverse = np.zeros(depth.shape, np.float32)
    np.divide(1.0, depth, out=inverse, where=valid)
    smoother = cv2.ximgproc.createFastGlobalSmootherFilter(guide, strength, sigma_color)
    inverse = smoother.filter(inverse)
    weight = smoother.filter(valid.astype(np.float32))
    supported = (weight > 0.1) & np.isfinite(inverse) & (inverse > 0)
    if not fill_holes:
        supported &= valid
    result = np.zeros(depth.shape, np.float32)
    np.divide(weight, inverse, out=result, where=supported)
    return result


class DepthEMA:
    """Per-pixel metric EMA using capture-time intervals, with no stale hole filling."""

    def __init__(self, tau_s, max_delta_m, max_gap_s):
        self.tau, self.max_delta, self.max_gap = tau_s, max_delta_m, max_gap_s
        self.reset()

    def reset(self):
        self.previous, self.stamp, self.key = None, None, None

    def update(self, depth, stamp, key):
        result = np.where(np.isfinite(depth) & (depth > 0), depth, 0).astype(np.float32)
        dt = None if self.stamp is None else stamp - self.stamp
        if (self.tau > 0 and self.previous is not None and self.key == key
                and self.previous.shape == result.shape and 0 < dt <= self.max_gap):
            valid = (result > 0) & (self.previous > 0)
            if self.max_delta > 0:
                valid &= np.abs(result - self.previous) <= self.max_delta
            alpha = -math.expm1(-dt / self.tau)
            result[valid] = self.previous[valid] + alpha * (result[valid] - self.previous[valid])
        self.previous, self.stamp, self.key = result.copy(), stamp, key
        return result


class WLSDepth(Node):
    """Publish metric WLS depth and two browser-compatible image transports."""

    def __init__(self):
        super().__init__('wls_filter')
        cv2.setNumThreads(1)
        defaults = {
            'depth_topic': '/oak/stereo/image_raw/compressedDepth',
            'rgb_topic': '/oak/rgb/image_rect/compressed',
            'camera_info_topic': '/oak/stereo/camera_info',
            'rate_hz': 10.0, 'max_width': 640, 'stale_after_s': 0.8,
            'lambda': 16000.0, 'sigma_color': 1.5, 'fill_holes': True,
            'near_m': 0.1, 'far_m': 15.0,
            'ema_tau_s': 0.35, 'ema_max_delta_m': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.value = lambda name: self.get_parameter(name).value
        if not (0 < self.value('rate_hz') <= 30 and self.value('max_width') >= 160
                and self.value('stale_after_s') > 0 and self.value('lambda') > 0
                and self.value('sigma_color') > 0
                and 0 <= self.value('near_m') < self.value('far_m')
                and all(math.isfinite(self.value(name)) and self.value(name) >= 0
                        for name in ('ema_tau_s', 'ema_max_delta_m'))):
            raise ValueError('Invalid WLS parameters')
        if not hasattr(cv2, 'ximgproc'):
            raise RuntimeError('WLS requires OpenCV contrib (python3-opencv on Ubuntu)')
        self.bridge = CvBridge()
        self.temporal = DepthEMA(self.value('ema_tau_s'), self.value('ema_max_delta_m'),
                                 self.value('stale_after_s'))
        self.depth = self.rgb = None
        self.camera_info = None
        self.depth_at = self.rgb_at = 0.0
        self.sequence, self.processed = 0, -1
        self.depth_sub = self.create_subscription(
            CompressedImage, self.value('depth_topic'), self.receive_depth,
            qos_profile_sensor_data)
        self.rgb_sub = self.create_subscription(
            CompressedImage, self.value('rgb_topic'), self.receive_rgb,
            qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, self.value('camera_info_topic'), self.receive_info, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/wls_filtered/camera_info', 1)
        self.raw_pub = self.create_publisher(Image, '/wls_filtered', 1)
        self.metric_pub = self.create_publisher(
            CompressedImage, '/wls_filtered/compressedDepth', 1)
        self.preview_pub = self.create_publisher(
            CompressedImage, '/wls_filtered/compressed', 1)
        self.timer = self.create_timer(1.0 / self.value('rate_hz'), self.tick)
        self.get_logger().info('WLS ready: registered OAK depth + RGB -> /wls_filtered')

    def receive_depth(self, message):
        """Retain the newest depth frame without waiting for timestamp agreement."""
        self.depth, self.depth_at = message, time.monotonic()
        self.sequence += 1

    def receive_rgb(self, message):
        """Retain the newest RGB guide."""
        self.rgb, self.rgb_at = message, time.monotonic()

    def receive_info(self, message):
        self.camera_info = message

    def tick(self):
        """Filter one fresh depth frame and retain its metric units and frame ID."""
        now = time.monotonic()
        if (self.depth is None or self.rgb is None
                or now - min(self.depth_at, self.rgb_at) > self.value('stale_after_s')):
            self.temporal.reset()
            self.get_logger().warn('WLS waiting for fresh OAK depth and RGB',
                                   throttle_duration_sec=5.0)
            return
        if self.processed == self.sequence:
            return
        try:
            if self.depth.header.frame_id != self.rgb.header.frame_id:
                raise ValueError('WLS depth must be registered to the RGB optical frame')
            depth = depth_in_meters(self.depth, self.value('max_width'))
            guide = self.bridge.compressed_imgmsg_to_cv2(self.rgb, desired_encoding='bgr8')
            guide = cv2.resize(guide, (depth.shape[1], depth.shape[0]),
                               interpolation=cv2.INTER_AREA)
            filtered = smooth_depth(depth, guide, self.value('lambda'),
                                    self.value('sigma_color'), self.value('fill_holes'))
            info = self.camera_info
            if info is not None:
                if info.header.frame_id != self.depth.header.frame_id:
                    raise ValueError('WLS calibration must use the registered depth optical frame')
                info = filtered_camera_info(info, self.depth.header, filtered.shape,
                                           max(1, math.ceil(info.width / self.value('max_width'))))
            stamp = self.depth.header.stamp
            key = (self.depth.header.frame_id, None if info is None else tuple(info.p))
            filtered = self.temporal.update(filtered, stamp.sec + stamp.nanosec / 1e9, key)
            millimeters = np.clip(np.rint(filtered * 1000), 0, 65535).astype(np.uint16)
            raw = self.bridge.cv2_to_imgmsg(millimeters, encoding='16UC1')
            raw.header = self.depth.header
            if info is not None:
                self.info_pub.publish(info)
            self.raw_pub.publish(raw)
            ok, png = cv2.imencode('.png', millimeters)
            if not ok:
                raise ValueError('Cannot encode WLS depth PNG')
            metric = CompressedImage()
            metric.header, metric.format = raw.header, '16UC1; compressedDepth png'
            metric.data = struct.pack('<iff', 0, 0., 0.) + png.tobytes()
            self.metric_pub.publish(metric)
            colors, valid = colorize_depth(filtered, self.value('near_m'), self.value('far_m'))
            picture = cv2.cvtColor(colors, cv2.COLOR_RGB2BGR)
            cv2.rectangle(picture, (0, 0), (picture.shape[1], 25), (0, 0, 0), -1)
            cv2.putText(picture, f'WLS depth | Valid {valid.mean():.0%} | Red near / Blue far',
                        (5, 17), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
            preview = self.bridge.cv2_to_compressed_imgmsg(picture, dst_format='jpg')
            preview.header = raw.header
            self.preview_pub.publish(preview)
            self.processed = self.sequence
        except (ValueError, TypeError, OSError, CvBridgeError, cv2.error) as error:
            self.temporal.reset()
            self.get_logger().error(str(error), throttle_duration_sec=5.0)


def main(args=None):
    """Run the WLS visualization publisher."""
    rclpy.init(args=args)
    node = WLSDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
