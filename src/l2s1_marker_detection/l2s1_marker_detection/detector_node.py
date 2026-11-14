"""ROS 2 node: detect markers in the rover camera stream and capture test frames.

Subscribes:  image_topic (sensor_msgs/Image, or CompressedImage if it ends in /compressed)
Publishes:   ~/detections (std_msgs/String, JSON) and ~/annotated/compressed (JPEG)
Services:    ~/capture (std_srvs/Trigger) saves the latest raw frame, its annotated
             version and a JSON line with the detections of every preset.
Parameters:  preset and capture_label can be changed while running (ros2 param set).
"""

from dataclasses import replace
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera_model import CameraModel
from .detection import DEFAULT_FAMILIES, PRESETS, MarkerDetector, draw, result_dict


def _default_camera_info():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('ugv_vision'), 'config',
                            'camera_info.yaml')
    except Exception:  # noqa: BLE001 - ugv_vision not installed: undistort unavailable
        return ''


class MarkerDetectorNode(Node):

    def __init__(self):
        super().__init__('marker_detector')
        self.declare_parameter('image_topic', '/image_rect')
        self.declare_parameter('preset', 'improved')
        self.declare_parameter('families', list(DEFAULT_FAMILIES))
        self.declare_parameter('camera_info_file', _default_camera_info())
        self.declare_parameter('max_rate', 10.0)
        self.declare_parameter('capture_dir', '/home/ws/ugv_ws/runs/marker_captures')
        self.declare_parameter('capture_label', '')

        self.families = tuple(self.get_parameter('families').value)
        self.camera = self._load_camera(self.get_parameter('camera_info_file').value)
        self.detector = self._build(self.get_parameter('preset').value)
        self.capture_label = self.get_parameter('capture_label').value
        self.add_on_set_parameters_callback(self._on_parameters)

        self.latest = None          # (bgr, header) of the newest frame
        self.last_processed = 0.0
        self.capture_count = 0

        topic = self.get_parameter('image_topic').value
        if topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, topic, self._on_compressed,
                                     qos_profile_sensor_data)
        else:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(String, '~/detections', 10)
        self.annotated_pub = self.create_publisher(CompressedImage, '~/annotated/compressed', 1)
        self.create_service(Trigger, '~/capture', self._on_capture)
        self.get_logger().info(
            f'Listening on {topic}; preset {self.get_parameter("preset").value} '
            f'({self.detector.name}); families {", ".join(self.detector.config.families)}')

    # ------------------------------------------------------------ setup
    def _load_camera(self, path):
        if not path:
            return None
        try:
            return CameraModel.from_yaml(path)
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().warn(f'No camera model from {path!r} ({error}); '
                                   'presets with undistort are unavailable')
            return None

    def _build(self, preset):
        if preset not in PRESETS:
            raise ValueError(f'Unknown preset {preset!r}; choose from {", ".join(PRESETS)}')
        return MarkerDetector(replace(PRESETS[preset], families=self.families), self.camera)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            try:
                if parameter.name == 'preset':
                    self.detector = self._build(parameter.value)
                    self.get_logger().info(f'Preset {parameter.value} ({self.detector.name})')
                elif parameter.name == 'capture_label':
                    self.capture_label = str(parameter.value)
                elif parameter.name in ('image_topic', 'families', 'camera_info_file'):
                    return SetParametersResult(successful=False,
                                               reason=f'{parameter.name} needs a restart')
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ images
    def _on_image(self, message):
        self._handle(self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'), message.header)

    def _on_compressed(self, message):
        image = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self._handle(image, message.header)

    def _handle(self, bgr, header):
        self.latest = (bgr, header)
        now = time.monotonic()
        if now - self.last_processed < 1.0 / max(0.1, self.get_parameter('max_rate').value):
            return
        self.last_processed = now
        result = self.detector.detect(bgr)
        payload = result_dict(result)
        payload.update(stamp=header.stamp.sec + header.stamp.nanosec * 1e-9,
                       frame_id=header.frame_id, width=bgr.shape[1], height=bgr.shape[0],
                       preset=self.get_parameter('preset').value)
        self.detections_pub.publish(String(data=json.dumps(payload)))
        if self.annotated_pub.get_subscription_count():
            self._publish_jpeg(draw(bgr, result.detections, self._status(result)), header)

    def _status(self, result):
        labels = ' '.join(f'{d.family.replace("APRILTAG_", "")}:{d.marker_id}'
                          for d in result.detections)
        return (f'{self.get_parameter("preset").value} {result.runtime_ms:.0f}ms '
                f'n={len(result.detections)} {labels}')

    def _publish_jpeg(self, bgr, header):
        ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            message = CompressedImage(header=header, format='jpeg')
            message.data = encoded.tobytes()
            self.annotated_pub.publish(message)

    # ------------------------------------------------------------ capture
    def _on_capture(self, _request, response):
        if self.latest is None:
            response.success, response.message = False, 'No camera frame received yet'
            return response
        bgr, header = self.latest
        directory = self.get_parameter('capture_dir').value
        os.makedirs(directory, exist_ok=True)
        self.capture_count += 1
        label = ''.join(ch if ch.isalnum() or ch in '._-' else '_' for ch in self.capture_label)
        stem = time.strftime('%Y%m%d_%H%M%S') + f'_{self.capture_count:03d}' + (
            f'_{label}' if label else '')
        cv2.imwrite(os.path.join(directory, stem + '.png'), bgr)

        record = {'file': stem + '.png', 'label': self.capture_label,
                  'stamp': header.stamp.sec + header.stamp.nanosec * 1e-9, 'presets': {}}
        for name, config in PRESETS.items():
            if config.undistort and self.camera is None:
                continue
            result = MarkerDetector(replace(config, families=self.families),
                                    self.camera).detect(bgr)
            record['presets'][name] = result_dict(result)
        current = self.detector.detect(bgr)
        cv2.imwrite(os.path.join(directory, stem + '_annotated.jpg'),
                    draw(bgr, current.detections, self._status(current)))
        with open(os.path.join(directory, 'captures.jsonl'), 'a') as stream:
            stream.write(json.dumps(record) + '\n')

        found = {name: entry['count'] for name, entry in record['presets'].items()}
        response.success = True
        response.message = f'Saved {stem}.png; markers per preset: {found}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
