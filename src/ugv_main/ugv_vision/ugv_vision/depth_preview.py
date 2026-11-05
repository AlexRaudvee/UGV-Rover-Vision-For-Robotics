"""Publish a JPEG visualization while preserving the camera's metric depth topic."""

from io import BytesIO
import math
import time

import numpy as np
from PIL import Image as PillowImage, ImageDraw
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def depth_in_meters(message, max_width=640):
    """Decode ROS depth, including byte order and padded rows, without modifying it."""
    if message.encoding == '16UC1':
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
        scale = 0.001
    elif message.encoding == '32FC1':
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        scale = 1.0
    else:
        raise ValueError('Unsupported depth encoding: ' + message.encoding)
    if message.width <= 0 or message.height <= 0:
        raise ValueError('Depth image has no pixels')
    if message.step < message.width * dtype.itemsize:
        raise ValueError('Depth row stride is smaller than its pixel data')
    pixels = np.ndarray(
        (message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.step, dtype.itemsize))
    stride = max(1, math.ceil(message.width / max_width))
    return pixels[::stride, ::stride].astype(np.float32) * scale


def colorize_depth(depth, near, far):
    """Map near distances to red, far distances to blue, and invalid pixels to black."""
    valid = np.isfinite(depth) & (depth > 0)
    level = np.zeros_like(depth)
    level[valid] = np.clip((far - depth[valid]) / (far - near), 0, 1)
    red = np.clip(1.5 - np.abs(4 * level - 3), 0, 1)
    green = np.clip(1.5 - np.abs(4 * level - 2), 0, 1)
    blue = np.clip(1.5 - np.abs(4 * level - 1), 0, 1)
    colors = (np.stack((red, green, blue), axis=-1) * 255).astype(np.uint8)
    colors[~valid] = 0
    return colors, valid


class DepthPreview(Node):
    """Create a throttled, labeled browser preview of an existing depth stream."""

    def __init__(self):
        super().__init__('depth_preview')
        defaults = {
            'input_topic': '/oak/stereo/image_raw',
            'output_topic': '/oak/stereo/depth_preview/compressed',
            'near_m': 0.2, 'far_m': 5.0, 'rate_hz': 5.0,
            'max_width': 640, 'jpeg_quality': 75, 'stale_after_s': 1.0,
            'reliable_input': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.near, self.far = float(get('near_m')), float(get('far_m'))
        self.max_width = int(get('max_width'))
        rate, self.stale_after = float(get('rate_hz')), float(get('stale_after_s'))
        self.quality = int(get('jpeg_quality'))
        if not (0 <= self.near < self.far and 0 < rate <= 30
                and self.max_width >= 160 and self.stale_after > 0
                and 1 <= self.quality <= 95):
            raise ValueError('Invalid depth-preview range, rate, size, quality, or timeout')
        self.publisher = self.create_publisher(CompressedImage, get('output_topic'), 1)
        self.subscription = self.create_subscription(
            Image, get('input_topic'), self.receive,
            QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE
                       if get('reliable_input') else ReliabilityPolicy.BEST_EFFORT)))
        self.latest = None
        self.received_at = 0.0
        self.sequence = 0
        self.published_sequence = -1
        self.timer = self.create_timer(1.0 / rate, self.publish_preview)
        self.get_logger().info(
            f"Depth preview: {get('input_topic')} -> {get('output_topic')}")

    def receive(self, message):
        self.latest = message
        self.received_at = time.monotonic()
        self.sequence += 1

    def publish_preview(self):
        if self.publisher.get_subscription_count() == 0:
            return
        stale = self.latest is None or time.monotonic() - self.received_at > self.stale_after
        if not stale and self.sequence == self.published_sequence:
            return
        output = CompressedImage()
        if stale:
            picture = PillowImage.new('RGB', (640, 360), 'black')
            label = 'Waiting for depth frames' if self.latest is None else 'Depth stream is stale'
            ImageDraw.Draw(picture).text((12, 12), label, fill='white')
            output.header.stamp = self.get_clock().now().to_msg()
        else:
            try:
                depth = depth_in_meters(self.latest, self.max_width)
                colors, valid = colorize_depth(depth, self.near, self.far)
            except (ValueError, TypeError) as error:
                self.get_logger().error(str(error), throttle_duration_sec=5.0)
                return
            picture = PillowImage.fromarray(colors)
            draw = ImageDraw.Draw(picture)
            height, width = depth.shape
            center = depth[max(0, height // 2 - 5):height // 2 + 6,
                           max(0, width // 2 - 5):width // 2 + 6]
            center = center[np.isfinite(center) & (center > 0)]
            distance = f'{np.median(center):.2f} m' if center.size else 'unknown'
            draw.rectangle((0, 0, width, 36), fill='black')
            draw.text((6, 3), f'Near/red {self.near:g} m | Far/blue {self.far:g} m', fill='white')
            draw.text((6, 18), f'Center: {distance} | Valid: {valid.mean():.0%} | Black: unknown',
                      fill='white')
            draw.line((width // 2 - 5, height // 2, width // 2 + 5, height // 2), fill='white')
            draw.line((width // 2, height // 2 - 5, width // 2, height // 2 + 5), fill='white')
            output.header = self.latest.header
            self.published_sequence = self.sequence
        buffer = BytesIO()
        picture.save(buffer, format='JPEG', quality=self.quality)
        output.format = 'rgb8; jpeg compressed bgr8'
        output.data = buffer.getvalue()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
