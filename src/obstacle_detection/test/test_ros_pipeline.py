"""Exercise moving targets, published previews, and stale input in isolated ROS domain 77."""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String

from obstacle_detection.node import ObstacleDetector


def test_moving_box_changes_histogram_and_stale_input_becomes_unknown(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '77')
    rclpy.init(args=[
        '--ros-args', '-p', 'rgb_topic:=/obstacle_test/rgb',
        '-p', 'depth_topic:=/obstacle_test/depth',
        '-p', 'camera_info_topic:=/obstacle_test/info',
        '-p', 'frame_id:=obstacle_test_histogram',
        '-p', 'rate_hz:=5.0', '-p', 'stale_after_s:=0.4',
        '--log-level', 'warn',
    ])
    detector = ObstacleDetector()
    source = Node('synthetic_obstacle_camera')
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(source)
    rgb_pub = source.create_publisher(Image, '/obstacle_test/rgb', 1)
    depth_pub = source.create_publisher(Image, '/obstacle_test/depth', 1)
    info_pub = source.create_publisher(CameraInfo, '/obstacle_test/info', 1)
    scans, details, previews = [], [], []
    subscriptions = [
        source.create_subscription(LaserScan, '/obstacle_detection/polar_histogram', scans.append, 1),
        source.create_subscription(String, '/obstacle_detection/detections',
                                   lambda msg: details.append(json.loads(msg.data)), 1),
        source.create_subscription(CompressedImage, '/obstacle_detection/histogram/compressed',
                                   previews.append, 1),
    ]
    info = CameraInfo()
    info.header.frame_id = 'obstacle_test_optical'
    info.width, info.height = 640, 480
    info.k = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=float).ravel().tolist()
    info.p = np.array([[500, 0, 320, 0], [0, 500, 240, 0], [0, 0, 1, 0]],
                      dtype=float).ravel().tolist()
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def publish_scene(center):
        bgr = np.full((480, 640, 3), (45, 150, 55), dtype=np.uint8)
        bgr[250:365, center - 34:center + 34] = 20
        depth = np.full((480, 640), 4500, dtype='<u2')
        depth[250:365, center - 34:center + 34] = 1500
        rgb = Image()
        rgb.header.frame_id = info.header.frame_id
        rgb.header.stamp = source.get_clock().now().to_msg()
        rgb.width, rgb.height, rgb.step, rgb.encoding = 640, 480, 1920, 'bgr8'
        rgb.data = bgr.tobytes()
        distance = Image()
        distance.header = rgb.header
        distance.width, distance.height, distance.step = 640, 480, 1280
        distance.encoding = '16UC1'
        distance.data = depth.tobytes()
        info_pub.publish(info)
        depth_pub.publish(distance)
        rgb_pub.publish(rgb)

    try:
        spin(0.8)
        bearings = []
        for center in (180, 320, 460):
            end = time.monotonic() + 1.3
            while time.monotonic() < end:
                publish_scene(center)
                spin(0.08)
            assert details and details[-1]['fresh']
            assert details[-1]['detections'][0]['distance_source'] == 'depth'
            scan = scans[-1]
            index = int(np.argmax(scan.intensities))
            bearings.append(math.degrees(scan.angle_min + index * scan.angle_increment))
        assert bearings[0] > 10, bearings
        assert abs(bearings[1]) <= 5, bearings
        assert bearings[2] < -10, bearings
        assert previews
        image = cv2.imdecode(np.frombuffer(previews[-1].data, np.uint8), cv2.IMREAD_COLOR)
        assert image.shape[:2] == (580, 1000)
        publishers = source.get_publisher_names_and_types_by_node('detector', '/obstacle_detection')
        assert not any('cmd_vel' in topic for topic, _ in publishers)
        rejected = detector.set_parameters([Parameter('roi_top', value=0.99)])[0]
        assert not rejected.successful
        spin(0.8)
        assert not details[-1]['fresh']
        assert np.isnan(scans[-1].ranges).all()
    finally:
        # Keep subscriptions alive throughout the test, then release all ROS resources.
        assert len(subscriptions) == 3
        executor.shutdown()
        detector.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
