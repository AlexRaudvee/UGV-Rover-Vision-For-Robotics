"""Metric depth decoding and browser-friendly RGB/depth previews."""

import struct

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def decode_depth(message, bridge, compressed=False):
    """Decode raw depth or ROS compressedDepth PNG (16UC1 or inverse 32FC1)."""
    if not compressed:
        if message.encoding not in ('16UC1', '32FC1'):
            raise ValueError('Depth requires 16UC1 millimeters or 32FC1 meters')
        depth = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough').astype(np.float32)
        if message.encoding == '16UC1':
            depth *= 0.001
    else:
        encoding = message.format.split(';')[0].strip()
        payload = bytes(message.data)
        if ('compressedDepth' not in message.format or encoding not in ('16UC1', '32FC1')
                or len(payload) <= 12 or payload[12:20] != b'\x89PNG\r\n\x1a\n'):
            raise ValueError('Expected metric compressedDepth PNG with a 12-byte ROS header')
        pixels = cv2.imdecode(np.frombuffer(payload[12:], np.uint8), cv2.IMREAD_UNCHANGED)
        if pixels is None or pixels.dtype != np.uint16 or pixels.ndim != 2:
            raise ValueError('Invalid compressedDepth PNG')
        if encoding == '16UC1':
            depth = pixels.astype(np.float32) * 0.001
        else:
            _, quant_a, quant_b = struct.unpack('<iff', payload[:12])
            if not np.isfinite([quant_a, quant_b]).all() or quant_a <= 0:
                raise ValueError('Invalid inverse-depth quantization')
            with np.errstate(divide='ignore', invalid='ignore'):
                depth = quant_a / (pixels.astype(np.float32) - quant_b)
            depth[pixels == 0] = np.nan
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def publish_jpeg(publisher, bgr, header):
    """Publish a normal JPEG understood by Vizanti's Compressed Image widget."""
    success, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if success:
        message = CompressedImage()
        message.header = header
        message.format = 'bgr8; jpeg compressed bgr8'
        message.data = encoded.tobytes()
        publisher.publish(message)


def label(image, message, xy, color=(235, 235, 235), scale=0.48):
    """Draw antialiased text."""
    cv2.putText(image, str(message), xy, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def waiting_image(status):
    """Render an explicit unavailable state."""
    image = np.full((360, 640, 3), (30, 27, 24), np.uint8)
    label(image, 'Waiting for calibrated, synchronized measurements', (15, 55))
    for row, start in enumerate(range(0, len(status), 75)):
        label(image, status[start:start + 75], (15, 100 + 25 * row), (90, 170, 255))
    return image


def annotated_image(bgr, unmeasured, measured, status):
    """Draw detections (measured distance, green) and unmeasured blobs (orange)."""
    image = bgr.copy()
    for blob in unmeasured:
        x, y, width, height = blob.bbox
        cv2.rectangle(image, (x, y), (x + width, y + height), (0, 160, 255), 2)
        label(image, 'unmeasured', (x, max(45, y - 5)), (0, 160, 255), 0.42)
    for item in measured:
        x, y, width, height = item.bbox
        cv2.rectangle(image, (x, y), (x + width, y + height), (40, 220, 40), 2)
        if item.clipped:
            message = (f'clipped >={item.width_m * 100:.0f}cm '
                       f'{item.distance_m:.2f}m {item.distance_source}')
        else:
            message = (f'{item.width_m * 100:.1f}+/-{item.width_std_m * 100:.1f}cm '
                       f'{item.distance_m:.2f}m {item.distance_source}')
        label(image, message, (x, max(45, y - 5)), (40, 220, 40), 0.42)
    cv2.rectangle(image, (0, 0), (image.shape[1], 28), (30, 27, 24), -1)
    label(image, status[:95], (6, 19), scale=0.40)
    return image


def depth_preview(depth, near, far):
    """Colorize optical Z; black pixels have no measurement."""
    valid = np.isfinite(depth) & (depth > 0)
    levels = np.zeros(depth.shape, np.uint8)
    levels[valid] = (255 * np.clip((far - depth[valid]) / (far - near), 0, 1)).astype(np.uint8)
    image = cv2.applyColorMap(levels, cv2.COLORMAP_JET)
    image[~valid] = 0
    label(image, f'Depth: near/red {near:g}m - far/blue {far:g}m; black=unknown', (6, 19),
          scale=0.40)
    return image
