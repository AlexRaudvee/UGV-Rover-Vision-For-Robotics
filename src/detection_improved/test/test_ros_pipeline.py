"""Exercise real ROS transport for both detectors, TF, plots, gaps, and stale inputs."""

import json
import struct
import time

import cv2
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from detection_improved.detector import ObjectDetector
from detection_improved.detector import ObstacleDetector
from detection_improved.histogram_node import PolarHistogramNode

HIST = '/detection_improved/polar_hist/'


@pytest.mark.parametrize('depth_transport', ['raw', 'compressedDepth'])
def test_both_range_sources_tf_width_gaps_previews_and_independent_staleness(
        monkeypatch, depth_transport):
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    rclpy.init(args=[
        '--ros-args', '-p', 'rgb_topic:=/perception_test/rgb/compressed',
        '-p', 'depth_topic:=/perception_test/depth',
        '-p', 'depth_transport:=' + depth_transport,
        '-p', 'scan_topic:=/perception_test/scan',
        '-p', 'camera_info_topic:=/perception_test/info',
        '-p', 'stale_after_s:=0.8', '--log-level', 'error',
    ])
    objects, obstacles, histograms = ObjectDetector(), ObstacleDetector(), PolarHistogramNode()
    source = Node('synthetic_perception_sensors')
    # The deployed nodes are separate processes; allow their callbacks to progress
    # independently here as well, including JPEG rendering and reliable image I/O.
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = [source, objects, obstacles, histograms]
    for node in nodes:
        executor.add_node(node)
    image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    rgb_pub = source.create_publisher(
        CompressedImage, '/perception_test/rgb/compressed', image_qos)
    depth_type = Image if depth_transport == 'raw' else CompressedImage
    depth_pub = source.create_publisher(depth_type, '/perception_test/depth', image_qos)
    scan_pub = source.create_publisher(LaserScan, '/perception_test/scan', qos_profile_sensor_data)
    info_pub = source.create_publisher(
        CameraInfo, '/perception_test/info', image_qos)
    details = {'object': [], 'obstacle': []}
    gaps, scans, plots = {'object': [], 'obstacle': []}, {}, {}
    subscriptions = []
    for key in ('object', 'obstacle'):
        namespace = 'detection_improved/' + key
        subscriptions.extend([
            source.create_subscription(String, '/' + namespace + '/detections',
                                       lambda msg, k=key: details[k].append(
                                           json.loads(msg.data)), 1),
            source.create_subscription(String, HIST + key + '/gaps',
                                       lambda msg, k=key: gaps[k].append(json.loads(msg.data)), 1),
            source.create_subscription(LaserScan, HIST + key + '/scan',
                                       lambda msg, k=key: scans.update({k: msg}), 1),
            source.create_subscription(CompressedImage,
                                       HIST + key + '/histogram/compressed',
                                       lambda msg, k=key: plots.update({k: msg}), 1),
        ])
    depth_previews = []
    subscriptions.append(source.create_subscription(
        CompressedImage, '/detection_improved/object/depth/compressed', depth_previews.append, 1))
    info = CameraInfo()
    info.header.frame_id = 'perception_test_optical'
    info.width, info.height = 640, 480
    info.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
    info.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
    info.d, info.distortion_model = [0.] * 5, 'plumb_bob'
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = info.header.frame_id, 'perception_test_laser'
    tf.transform.rotation.x, tf.transform.rotation.y = 0.5, -0.5
    tf.transform.rotation.z, tf.transform.rotation.w = 0.5, 0.5
    tf.transform.translation.x, tf.transform.translation.z = 0.03, -0.02
    broadcaster = StaticTransformBroadcaster(source)
    camera_pose = TransformStamped()
    camera_pose.header.frame_id, camera_pose.child_frame_id = 'base_link', info.header.frame_id
    camera_pose.transform.rotation.x, camera_pose.transform.rotation.y = -0.5, 0.5
    camera_pose.transform.rotation.z, camera_pose.transform.rotation.w = -0.5, 0.5

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def publish_scene(lidar=True, valid_depth=True, offset=0):
        stamp = source.get_clock().now().to_msg()
        bgr = np.full((480, 640, 3), 200, np.uint8)
        depth = np.full((480, 640), 4000, dtype='<u2')
        bounds = [(140 + offset, 230 + offset), (410 + offset, 500 + offset)]
        for low, high in bounds:
            bgr[180:340, low:high] = 20
            depth[180:340, low:high] = 1500 if valid_depth else 0
        rgb = CompressedImage()
        rgb.header.frame_id, rgb.header.stamp = info.header.frame_id, stamp
        rgb.format = 'bgr8; jpeg compressed bgr8'
        rgb.data = cv2.imencode('.jpg', bgr)[1].tobytes()
        distance = Image()
        distance.header = rgb.header
        distance.width, distance.height, distance.step = 640, 480, 1280
        distance.encoding, distance.data = '16UC1', depth.tobytes()
        if depth_transport == 'compressedDepth':
            distance = CompressedImage()
            distance.header = rgb.header
            distance.format = '16UC1; compressedDepth'
            png = cv2.imencode('.png', depth)[1].tobytes()
            distance.data = struct.pack('<iff', 0, 0., 0.) + png
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = tf.child_frame_id, stamp
        scan.angle_min, scan.angle_increment = -1.4, .002
        scan.range_min, scan.range_max = .15, 5.
        angles = scan.angle_min + np.arange(1400) * scan.angle_increment
        ranges = 1.52 / np.cos(angles)
        optical_x = -ranges * np.sin(angles) + .03
        u = 320 + 500 * optical_x / 1.5
        hit = np.zeros(len(u), bool)
        for low, high in bounds:
            hit |= (u >= low + 1) & (u < high - 1)
        ranges[~hit] = np.inf
        scan.ranges = ranges.astype(float).tolist()
        info_pub.publish(info)
        depth_pub.publish(distance)
        if lidar:
            scan_pub.publish(scan)
        rgb_pub.publish(rgb)

    def stream(seconds, **kwargs):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publish_scene(**kwargs)
            spin(.06)

    try:
        spin(.5)
        stream(.7)
        assert details['object'][-1]['fresh']
        assert not details['obstacle'][-1]['fresh']  # No camera/laser TF yet.
        tf.header.stamp = source.get_clock().now().to_msg()
        broadcaster.sendTransform([tf, camera_pose])
        spin(.3)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            stream(.1)
            outputs_fresh = all(
                details[key] and details[key][-1]['fresh']
                and gaps[key] and gaps[key][-1]['fresh'] for key in details)
            if len(plots) == 2 and outputs_fresh:
                break
        for key, expected_source in [('object', 'depth'), ('obstacle', 'lidar')]:
            payload = details[key][-1]
            assert payload['fresh'], payload
            assert len(payload['detections']) == 2, payload
            assert all(item['distance_source'] == expected_source
                       for item in payload['detections'])
            if expected_source == 'depth':
                assert payload['depth_ema_tau_s'] == .35
                assert payload['range_samples']
                assert all(item['filter'] == 'ema' for item in payload['range_samples'])
            else:
                assert 'depth_ema_tau_s' not in payload
            assert all(abs(item['width_m'] - .27) < .015 for item in payload['detections'])
            gap_payload = gaps[key][-1]
            assert gap_payload['fresh'] and len(gap_payload['gaps']) == 1, json.dumps(gap_payload)
            assert abs(gaps[key][-1]['gaps'][0]['width_m'] - .54) < .025
            image = cv2.imdecode(np.frombuffer(plots[key].data, np.uint8), cv2.IMREAD_COLOR)
            assert image.shape[:2] == (620, 1100)
            assert np.isnan(scans[key].ranges).any()  # Unseen rear remains unknown.
        assert depth_previews
        # Vizanti can connect after both cameras have already published frames.
        late_buffer = Buffer()
        late_listener = TransformListener(late_buffer, source)
        stream(.4)
        for key in ('object', 'obstacle'):
            frame = 'detection_improved_hist_' + key
            assert late_buffer.can_transform(info.header.frame_id, frame, Time())
        late_listener.unregister()
        initial = details['object'][-1]['detections'][0]['bearing']
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            stream(.1, offset=35)
            if (details['object'][-1]['detections']
                    and details['object'][-1]['detections'][0]['bearing'] < initial - .01):
                break
        assert details['object'][-1]['detections'][0]['bearing'] < initial
        assert not objects.set_parameters([Parameter('width_min_m', value=.4)])[0].successful
        stream(1.1, lidar=False)
        assert details['object'][-1]['fresh']
        assert not details['obstacle'][-1]['fresh']
        assert not gaps['obstacle'][-1]['gaps']
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            stream(.1, valid_depth=False)
            # Also wait for polar_hist to process that message; checking its gaps right
            # after the detector output changed raced the histogram node.
            if (not details['object'][-1]['detections']
                    and len(details['object'][-1]['unknown_intervals']) == 2
                    and not gaps['object'][-1]['gaps']):
                break
        assert not details['object'][-1]['detections']
        assert len(details['object'][-1]['unknown_intervals']) == 2
        assert not gaps['object'][-1]['gaps']
        spin(1.1)
        for key in details:
            assert not gaps[key][-1]['fresh']
            assert not gaps[key][-1]['gaps']
            assert np.isnan(scans[key].ranges).all()
        for node in (objects, obstacles, histograms):
            publishers = source.get_publisher_names_and_types_by_node(
                node.get_name(), node.get_namespace())
            assert not any('cmd_vel' in topic for topic, _ in publishers)
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()
