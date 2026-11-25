"""One synthetic depth detector, compatibility aliases and navigation validity.

Isolated ROS domain 89, no hardware and no velocity publishers.
"""

import json
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String

from detection_improved.detector import ObjectDetector
from test_floor import camera_pose, floor_depth, MATRIX
from test_grounded import standing_box


def test_single_depth_detector_aliases_all_blockers_and_rejects_unmeasured_coverage():
    rclpy.init(args=['--ros-args', '-p', 'alias_namespace:=detection_improved/obstacle',
                    '-p', 'depth_transport:=raw', '-p', 'ground_check:=true',
                    '-p', 'navigation_mask_topic:=/navigation_test/support',
                    '-p', 'check_acquisition_time:=false', '-p', 'synchronize_inputs:=false',
                    '-p', 'camera_pitch_down_deg:=5.0'], domain_id=89)
    node = ObjectDetector()
    client = Node('navigation_output_observer', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    for item in (node, client):
        executor.add_node(item)
    outputs, aliases = [], []
    client.create_subscription(String, '/detection_improved/object/detections',
                               lambda m: outputs.append(json.loads(m.data)), 10)
    client.create_subscription(String, '/detection_improved/obstacle/detections',
                               lambda m: aliases.append(json.loads(m.data)), 10)
    try:
        # Nominal TF is level; detector compensates the real five-degree mount.
        rotation, _ = camera_pose(.2, 0.)
        rotation = rotation.T
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_footprint', 'camera'
        tf.transform.translation.z = .2
        w = math.sqrt(1+np.trace(rotation))/2
        tf.transform.rotation.w = w
        tf.transform.rotation.x = (rotation[2, 1]-rotation[1, 2])/(4*w)
        tf.transform.rotation.y = (rotation[0, 2]-rotation[2, 0])/(4*w)
        tf.transform.rotation.z = (rotation[1, 0]-rotation[0, 1])/(4*w)
        node.tf_buffer.set_transform_static(tf, 'synthetic_test')
        bridge = CvBridge()
        depth = standing_box(floor_depth(.2, 5.), 140, 180, 1.5, 60).astype(np.float32)

        def feed(mask=True, stale=False, skew=0.):
            from rclpy.time import Time
            stamp = node.get_clock().now().nanoseconds-int((2. if stale else .05)*1e9)
            sensor = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            sensor.header.frame_id, sensor.header.stamp = 'camera', Time(nanoseconds=stamp-int(skew*1e9)).to_msg()
            rgb = bridge.cv2_to_compressed_imgmsg(np.full((240, 320, 3), 90, np.uint8))
            rgb.header.frame_id = 'camera'
            rgb.header.stamp = Time(nanoseconds=stamp).to_msg()
            info = CameraInfo(width=320, height=240)
            info.header = sensor.header
            info.k = MATRIX.ravel().tolist()
            info.p = np.column_stack((MATRIX, np.zeros(3))).ravel().tolist()
            node.receive_info(info)
            node.receive_rgb(rgb)
            node.receive_measurement(sensor)
            if mask:
                support = CompressedImage()
                support.header = sensor.header
                support.data = cv2.imencode('.png', (np.isfinite(depth)*255).astype(np.uint8))[1].tobytes()
                node.support_masks.append(support)
            node.tick()

        until = time.monotonic()+3
        while time.monotonic() < until and node.details_pub.get_subscription_count() < 2:
            executor.spin_once(timeout_sec=.02)
        feed()
        until = time.monotonic()+3
        while time.monotonic() < until and (not outputs or not aliases):
            executor.spin_once(timeout_sec=.02)
        assert outputs and aliases
        assert outputs[-1] == aliases[-1], 'aliases differ instead of sharing one inference result'
        result = outputs[-1]
        assert result['navigation']['usable'], result['navigation']
        assert result['navigation']['free_points'] and result['navigation']['occupied_points']
        assert result['navigation']['blocker_faces']
        assert result['blockers'] and any(not d['accepted'] for d in result['blockers'])
        # A fresh display does not override stricter navigation acquisition checks.
        for kwargs, reason in ((dict(stale=True), 'stale'), (dict(skew=.4), 'synchronized'),
                               (dict(mask=False), 'mask')):
            previous = len(outputs)
            feed(**kwargs)
            until = time.monotonic()+2
            while len(outputs) == previous and time.monotonic() < until:
                executor.spin_once(timeout_sec=.01)
            assert len(outputs) > previous
            assert not outputs[-1]['navigation']['usable']
            assert reason in outputs[-1]['navigation']['reason']
        # A previously fitted plane must not survive a failed current TF context.
        node.floor_heights = lambda *args: (None, 'no TF to base_footprint')
        previous = len(outputs)
        feed()
        until = time.monotonic()+2
        while len(outputs) == previous and time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert not outputs[-1]['navigation']['usable']
        assert 'floor context' in outputs[-1]['navigation']['reason']
        names = [name for name, _ in node.get_node_names_and_namespaces()]
        assert names.count('detector') == 1
        assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
        assert not node.get_subscriptions_info_by_topic('/scan')
    finally:
        executor.shutdown()
        client.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
