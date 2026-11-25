"""Synthetic EKF diagnostics on isolated domain 109; no hardware or motor topics."""
import json
import math
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from l2s3_curling.node import CurlingControllerNode


@pytest.fixture
def observer(monkeypatch, tmp_path, request):
    # Use a separate ROS domain and a private command topic, even for idle tests.
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    # Keep ROS logs inside pytest's temporary directory, including in restricted environments.
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    args = ['--ros-args', '-p', 'command_topic:=/avoidance_ekf_test/cmd_vel']
    # Individual tests can exercise configured limits without changing the source YAML.
    for name, value in getattr(request, 'param', {}).items():
        args.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=args, domain_id=109)
    node = CurlingControllerNode()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def pose_message(node, x=-3.2, y=0., heading=0., stamp=None):
    # Construct a field-frame pose with realistic, known planar uncertainty.
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    stamp = node.now() - .1 if stamp is None else stamp
    msg.header.stamp = Time(nanoseconds=int(stamp * 1e9)).to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z = math.sin(heading / 2)
    msg.pose.pose.orientation.w = math.cos(heading / 2)
    for i, variance in ((0, .01), (7, .04), (35, math.radians(5.)**2)):
        msg.pose.covariance[i] = variance
    return msg


def field_message(revision=1, start_x=-3.2):
    # Mirror the field server's saved-start metadata instead of hardcoding it in the node.
    return String(data=json.dumps(dict(frame_id='map', session_id='synthetic', revision=revision,
        start=dict(start_x_m=start_x, start_y_m=0., start_heading_deg=0.))))


def comparison_setup(node, msg, legacy_pose):
    # Activate only the mathematical controller; never call the start service or tick it.
    node.controller.start(np.zeros(3), time.monotonic())
    node.on_field_state(field_message())
    node.capture_comparison_reference()
    stamp = node.stamp(msg)
    node.pose = np.array(legacy_pose, dtype=float)
    node.comparison_history.extend([(stamp - .1, node.pose.copy()), (stamp + .1, node.pose.copy())])
    node.on_ekf_pose(msg)


def test_transport_reports_pose_and_uncertainty_without_publishing_commands(observer, monkeypatch):
    node = observer
    client = Node('avoidance_ekf_test_client', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    for item in (node, client):
        executor.add_node(item)
    statuses, commands = [], []
    client.create_subscription(String, '/curling_controller/status',
                               lambda msg: statuses.append(json.loads(msg.data)), 10)
    client.create_subscription(Twist, '/avoidance_ekf_test/cmd_vel', commands.append, 10)
    publisher = client.create_publisher(PoseWithCovarianceStamped, '/marker_ekf/pose', 1)
    # Fail immediately if an idle observer attempts to send any velocity command.
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Observer sent a velocity command'))
    try:
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            publisher.publish(pose_message(node))
            executor.spin_once(timeout_sec=.02)
            if statuses and statuses[-1]['ekf']['fresh']:
                break
        assert statuses and statuses[-1]['ekf']['fresh']
        status = statuses[-1]
        assert status['state'] == 'IDLE'
        assert status['localization'] == 'marker_ekf_goal_guidance'
        assert status['motion_localization'] == 'gyro_heading_and_forward_velocity'
        assert status['route_position_m'] == [0., 0.]
        assert status['ekf']['position_m'] == [-3.2, 0.]
        assert status['ekf']['sigma_x_m'] == pytest.approx(.1)
        assert status['ekf']['sigma_y_m'] == pytest.approx(.2)
        assert status['ekf']['sigma_heading_deg'] == pytest.approx(5.)
        assert not status['ekf']['comparison']['ready']
        assert node.latest_ekf_pose.pose.covariance[0] == .01
        assert commands == []
        assert node.count_publishers('/cmd_vel') == 0
    finally:
        executor.shutdown()
        client.destroy_node()


@pytest.mark.parametrize('field_pose, legacy_pose', [
    ((-3.2, 0., 0.), (0., 0., 0.)),  # Stationary at the assumed start.
    ((-2.2, 0., 0.), (1., 0., 0.)),  # One metre forward.
    ((-2.2, .5, math.pi / 2), (1., .5, math.pi / 2)),  # Left displacement and turn.
])
def test_same_frame_comparison_of_synthetic_motion(observer, field_pose, legacy_pose):
    msg = pose_message(observer, *field_pose)
    comparison_setup(observer, msg, legacy_pose)
    comparison = observer.ekf_diagnostics()['comparison']
    assert comparison['ready']
    assert comparison['distance_difference_m'] == pytest.approx(0., abs=1e-9)
    assert comparison['delta_heading_deg'] == pytest.approx(0., abs=1e-9)


def test_marker_like_correction_is_visible_without_changing_legacy_pose(observer):
    msg = pose_message(observer, x=-2.2)
    comparison_setup(observer, msg, (1., 0., 0.))
    before = observer.pose.copy()
    history = [(t, pose.copy()) for t, pose in observer.comparison_history]
    # A revised estimate at the same acquisition time represents a correction, not travel.
    corrected = pose_message(observer, x=-2.1, y=.2, heading=.1, stamp=observer.stamp(msg))
    observer.on_ekf_pose(corrected)
    result = observer.ekf_diagnostics()['comparison']
    assert result['ready']
    np.testing.assert_allclose(result['delta_position_m'], [.1, .2], atol=1e-9)
    assert result['delta_heading_deg'] == pytest.approx(math.degrees(.1))
    np.testing.assert_array_equal(observer.pose, before)
    for actual, expected in zip(observer.comparison_history, history):
        assert actual[0] == expected[0]
        np.testing.assert_array_equal(actual[1], expected[1])


def test_comparison_interpolates_at_acquisition_time_instead_of_using_latest_pose(observer):
    msg = pose_message(observer, x=-2.2)
    comparison_setup(observer, msg, (2., 0., 0.))
    stamp = observer.stamp(msg)
    observer.comparison_history.clear()
    observer.comparison_history.extend([(stamp - .1, np.array([0., 0., 0.])),
                             (stamp + .1, np.array([2., 0., 0.]))])
    result = observer.ekf_diagnostics()['comparison']
    assert result['ready']
    np.testing.assert_allclose(result['legacy_relative_pose'], [1., 0., 0.], atol=1e-6)
    assert result['distance_difference_m'] == pytest.approx(0., abs=1e-6)


def test_wheel_callback_records_acquisition_time_for_delayed_ekf_comparison(observer):
    observer.controller.start(np.zeros(3), time.monotonic())
    observer.on_field_state(field_message())
    observer.capture_comparison_reference()
    start = observer.now() - .8
    # Simulate three wheel samples while the controller remains unexecuted.
    for offset in (0., .4, .8):
        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=int((start + offset) * 1e9)).to_msg()
        msg.twist.twist.linear.x = 1.
        observer.on_wheels(msg)
        observer.on_odom(msg)
    observer.on_ekf_pose(pose_message(observer, x=-2.8, stamp=start + .4))
    result = observer.ekf_diagnostics()['comparison']
    assert result['ready']
    assert observer.pose[0] == pytest.approx(.8, abs=1e-6)
    assert result['legacy_relative_pose'][0] == pytest.approx(.4, abs=1e-6)
    assert result['distance_difference_m'] == pytest.approx(0., abs=1e-6)


@pytest.mark.parametrize('failure', ['old_capture', 'old_receipt', 'future', 'frame',
                                     'position', 'quaternion', 'covariance', 'asymmetric_covariance'])
def test_unusable_messages_are_reported_without_modifying_motion(observer, failure):
    msg = pose_message(observer)
    if failure == 'old_capture':
        msg = pose_message(observer, stamp=observer.now() - 2.)
    elif failure == 'future':
        msg = pose_message(observer, stamp=observer.now() + 1.)
    elif failure == 'frame':
        msg.header.frame_id = 'odom'
    elif failure == 'position':
        msg.pose.pose.position.x = float('nan')
    elif failure == 'quaternion':
        msg.pose.pose.orientation.w = 0.
    elif failure == 'covariance':
        msg.pose.covariance[0] = -1.
    elif failure == 'asymmetric_covariance':
        msg.pose.covariance[1] = .01
    observer.on_ekf_pose(msg)
    if failure == 'old_receipt':
        observer.ekf_received -= 2.
    report = observer.ekf_diagnostics()
    assert not (report['valid'] and report['fresh'])
    assert report['reason']
    assert not report['comparison']['ready']
    # Invalid numbers must not leak NaNs into the diagnostic JSON.
    json.dumps(report, allow_nan=False)
    np.testing.assert_array_equal(observer.pose, np.zeros(3))
    assert not observer.controller.active


@pytest.mark.parametrize('failure', ['history_missing', 'history_gap', 'field_reset',
                                     'field_stale', 'start_changed'])
def test_comparison_is_withheld_when_time_or_reference_does_not_match(observer, failure):
    msg = pose_message(observer, x=-2.2)
    comparison_setup(observer, msg, (1., 0., 0.))
    stamp = observer.stamp(msg)
    if failure == 'history_missing':
        observer.comparison_history.clear()
    elif failure == 'history_gap':
        observer.comparison_history.clear()
        observer.comparison_history.extend([(stamp - 1., np.zeros(3)), (stamp + 1., np.zeros(3))])
    elif failure == 'field_reset':
        observer.on_field_state(field_message(revision=2))
    elif failure == 'field_stale':
        observer.field_received -= 2.
    elif failure == 'start_changed':
        observer.on_field_state(field_message(start_x=-3.))
    report = observer.ekf_diagnostics()
    assert report['valid'] and report['fresh']
    assert not report['comparison']['ready']
    assert report['comparison']['reason']
