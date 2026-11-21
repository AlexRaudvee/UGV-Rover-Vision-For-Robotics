"""Tracker + driver + range integration on domain 94, with a fake serial port."""
import json
import math
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from l2s2_localisation.pan_tracker_node import PanTracker
from ugv_bringup.ugv_driver import UgvDriver


class SerialSink:
    def __init__(self):
        self.sent = []

    def write(self, payload):
        self.sent.append(json.loads(payload))
        return len(payload)


def head(node, pan=0):
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
    msg.position = [-math.radians(pan), 0.]
    return msg


@pytest.fixture
def tracker():
    # Integer YAML scalars must work for floating-point settings as well.
    rclpy.init(args=['--ros-args', '-p', 'detection_timeout_s:=1'], domain_id=94)
    node = PanTracker()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def detection(node, ids=(2, 3), stamp=None, offset=0):
    camera = node.cameras['rectified']
    corners = np.array([[-2, -2], [2, -2], [2, 2], [-2, 2]])
    data = dict(stamp=node.get_clock().now().nanoseconds/1e9 if stamp is None else stamp,
                width=640, height=480, image_geometry='rectified', detections=[])
    for marker, bearing in zip(ids, np.linspace(20+offset, 40+offset, len(ids))):
        center = [camera.K[0, 2]-camera.K[0, 0]*math.tan(math.radians(bearing)), 240]
        data['detections'].append(dict(id=marker, family='APRILTAG_standard41h12',
                                       corners=(corners+center).tolist(), hamming=1, decision_margin=80))
    return String(data=json.dumps(data))


def test_camera_calibration_exposure_angles_duplicate_and_stale_frames(tracker):
    node = tracker
    node.on_head(head(node, 10))
    msg = detection(node)
    node.on_detections(msg)
    assert node.control.tracks[2]['angle'] == pytest.approx(30)
    assert node.control.tracks[3]['angle'] == pytest.approx(50)
    empty = json.loads(msg.data)
    empty['detections'] = []
    node.on_detections(String(data=json.dumps(empty)))
    assert node.control.tracks[2]['misses'] == 0, 'same image counted twice'
    node.on_detections(detection(node, ids=(), stamp=node.last_stamp-2))
    assert node.control.tracks[2]['misses'] == 0, 'stale image changed tracking'
    for _ in range(3):
        node.on_detections(detection(node, ids=(2,)))
    assert node.control.tracks[3]['misses'] == 3
    node.control.step(.05, time.monotonic())
    assert node.control.mode == 'centering'


def test_tracker_commands_driver_priority_fov_and_expiry(tracker):
    node = tracker
    serial = SerialSink()
    driver = UgvDriver(serial_connection=serial)
    client = Node('pan_test_observer')
    executor = SingleThreadedExecutor()
    for item in (node, driver, client):
        executor.add_node(item)
    states = []
    client.create_subscription(String, '/marker_pan_tracker/state', lambda m: states.append(json.loads(m.data)), 10)
    try:
        # Wait for discovery without sending to any actual UART or robot domain.
        end = time.monotonic()+3
        while time.monotonic() < end and (node.pub.get_subscription_count() < 1 or driver.commanded_head_pub.get_subscription_count() < 1):
            executor.spin_once(timeout_sec=.02)
        node.tick()
        assert not serial.sent, 'tracker moved the head before receiving its starting angle'
        driver.joint_states_callback(head(node, 12))
        end = time.monotonic()+3
        while node.last_head_received is None and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        assert node.control.pan == pytest.approx(12)
        node.on_detections(detection(node))
        end = time.monotonic()+3
        while (not any(s.get('ready') for s in states) or not math.isfinite(driver.last_tracking_command)) and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        state = next(s for s in states if s.get('ready'))
        assert 115 < state['horizontal_fov_deg'] < 118
        assert state['range_m'] == 4
        assert state['mode'] == 'tracking_markers'
        assert state['tracked_ids'] == [2, 3]
        assert abs(node.control.pan-12) <= 1, 'initial takeover jumped'
        before = len(serial.sent)
        driver.joint_states_callback(head(node, 0))
        assert len(serial.sent) == before, 'baseline zero overrode tracking'
        invalid = head(node, float('nan'))
        driver.tracking_joint_callback(invalid)
        assert len(serial.sent) == before
        driver.last_tracking_command -= 2
        driver.joint_states_callback(head(node, 0))
        assert serial.sent[-1]['X'] == 0
        assert all(p['T'] == 134 for p in serial.sent), 'tracker issued vehicle commands'
        node.last_head_received -= 2
        previous = node.control.pan
        node.tick()
        assert node.control.pan == previous, 'tracker kept commanding without driver feedback'
    finally:
        executor.shutdown()
        client.destroy_node()
        driver.destroy_node()


def test_bad_decoder_quality_is_treated_as_missing(tracker):
    node = tracker
    node.on_head(head(node))
    msg = detection(node)
    data = json.loads(msg.data)
    data['detections'][0]['hamming'] = 2
    node.on_detections(String(data=json.dumps(data)))
    assert 2 not in node.control.tracks and 3 in node.control.tracks


def test_node_tracks_any_visible_subset_and_publishes_the_ids(tracker):
    node = tracker
    node.on_head(head(node))
    node.on_detections(detection(node, ids=(0, 4, 5)))
    node.tick()
    assert node.control.mode == 'tracking_markers'
    assert node.control.tracked_ids == [0, 4, 5]
    # Tags 1, 2, 3 do not need to be visible for tracking to continue.
    for _ in range(3):
        node.on_detections(detection(node, ids=(0, 5)))
    node.tick()
    assert node.control.mode == 'tracking_markers'
    assert node.control.tracked_ids == [0, 5]
