"""Synthetic confidence checks on isolated ROS domain 109; no motor commands."""
import math

import numpy as np
import pytest
import rclpy
from std_srvs.srv import Trigger

from obstacle_avoidance_adv.node import AvoidanceNode
from test_ekf_navigation import clear_view, navigation_setup
from test_ekf_observer import observer, field_message, pose_message


def with_uncertainty(msg, position_std, heading_std_deg):
    # Set an isotropic x/y covariance and a known heading standard deviation.
    msg.pose.covariance[0] = position_std**2
    msg.pose.covariance[7] = position_std**2
    msg.pose.covariance[35] = math.radians(heading_std_deg)**2
    return msg


@pytest.mark.parametrize('position_std, heading_std, accepted', [
    (.15, 5., True),
    (.30, 15., True),  # The existing assumed-start uncertainty remains acceptable.
    (.50, 20., True),  # Limits are inclusive.
    (.500001, 5., False),
    (.15, 20.0001, False),
    (.60, 25., False),
])
def test_uncertainty_limits_are_separate_from_validity_and_freshness(
        observer, position_std, heading_std, accepted):
    observer.on_field_state(field_message())
    observer.on_ekf_pose(with_uncertainty(pose_message(observer), position_std, heading_std))
    report = observer.ekf_diagnostics()
    assert report['valid'] and report['fresh']
    assert report['within_uncertainty_limits'] == accepted
    assert report['sigma_position_m'] == pytest.approx(position_std)
    assert report['sigma_heading_deg'] == pytest.approx(heading_std)
    assert report['max_position_std_m'] == .50
    assert report['max_heading_std_deg'] == 20.
    if accepted:
        assert report['uncertainty_reason'] == ''
        assert observer.require_ekf_pose()['within_uncertainty_limits']
    else:
        assert 'exceeds limit' in report['uncertainty_reason']
        with pytest.raises(ValueError, match='uncertainty .* exceeds limit'):
            observer.require_ekf_pose()


def test_correlated_position_error_is_checked_along_its_largest_direction(observer):
    msg = with_uncertainty(pose_message(observer), .4, 5.)
    # Each individual axis has only 0.4 m standard deviation, but their correlation
    # gives sqrt(0.28) = 0.529 m uncertainty along the principal diagonal.
    msg.pose.covariance[1] = msg.pose.covariance[6] = .12
    observer.on_ekf_pose(msg)
    report = observer.ekf_diagnostics()
    assert report['valid'] and report['fresh']
    assert report['sigma_x_m'] == pytest.approx(.4)
    assert report['sigma_y_m'] == pytest.approx(.4)
    assert report['sigma_position_m'] == pytest.approx(math.sqrt(.28))
    assert not report['within_uncertainty_limits']
    assert 'position uncertainty' in report['uncertainty_reason']


@pytest.mark.parametrize('observer, expected_reason', [
    ({'max_position_std_m': .10}, 'position uncertainty'),
    ({'max_heading_std_deg': 4.0}, 'heading uncertainty'),
], indirect=['observer'])
def test_parameter_overrides_change_the_acceptance_limits(observer, expected_reason):
    observer.on_field_state(field_message())
    observer.on_ekf_pose(with_uncertainty(pose_message(observer), .15, 5.))
    report = observer.ekf_diagnostics()
    assert report['max_position_std_m'] == observer.value('max_position_std_m')
    assert report['max_heading_std_deg'] == observer.value('max_heading_std_deg')
    with pytest.raises(ValueError, match=expected_reason):
        observer.require_ekf_pose()


@pytest.mark.parametrize('position_std, heading_std', [(.6, 5.), (.15, 25.), (.6, 25.)])
def test_excessive_uncertainty_rejects_start_without_commands(
        observer, monkeypatch, position_std, heading_std):
    node = observer
    node.on_field_state(field_message())
    node.on_ekf_pose(with_uncertainty(pose_message(node), position_std, heading_std))
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Rejected start sent a command'))
    response = node.start(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert 'uncertainty' in response.message and 'exceeds limit' in response.message
    assert node.controller.state == 'IDLE'


@pytest.mark.parametrize('position_std, heading_std', [(.6, 5.), (.15, 25.)])
def test_uncertainty_holds_motion_then_recovers_through_the_motion_limiter(
        observer, monkeypatch, position_std, heading_std):
    node = observer
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    fixed_goal, local_pose = node.goal_map.copy(), node.pose.copy()
    sent = []
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    uncertain = pose_message(node, x=-2.2, stamp=node.stamp(original))
    node.on_ekf_pose(with_uncertainty(uncertain, position_std, heading_std))
    node.tick()
    assert sent[-1] == (0., 0.)
    assert node.hold_since is not None
    assert not node.navigation_status['ready']
    assert node.controller.active

    node.on_ekf_pose(pose_message(node, x=-2.2, stamp=node.stamp(original)))
    node.tick()
    assert node.hold_since is None
    assert node.navigation_status['ready']
    assert sent[-1] == (0., 0.)  # Recovery still waits for continuous clearance.
    assert node.motion.clear_since is not None
    # Advance the limiter's timers without sleeping or moving any hardware.
    node.motion.clear_since -= .3
    node.motion.last_time -= .1
    node.tick()
    assert sent[-1][0] > 0.
    np.testing.assert_array_equal(node.goal_map, fixed_goal)
    np.testing.assert_array_equal(node.pose, local_pose)


def test_continuous_uncertainty_hold_faults_after_five_seconds(observer, monkeypatch):
    node = observer
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    sent = []
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    uncertain = pose_message(node, x=-2.2, stamp=node.stamp(original))
    node.on_ekf_pose(with_uncertainty(uncertain, .6, 5.))
    node.tick()
    node.hold_since -= 6.
    node.tick()
    assert node.controller.state == 'FAULT'
    assert sent[-1] == (0., 0.)
    assert 'position uncertainty' in node.controller.reason
    # A fresh, confident pose cannot automatically re-arm a faulted mission.
    node.on_ekf_pose(pose_message(node, x=-2.2, stamp=node.stamp(original)))
    node.tick()
    assert node.controller.state == 'FAULT'
    assert sent[-1] == (0., 0.)


@pytest.mark.parametrize('name', ['max_position_std_m', 'max_heading_std_deg'])
@pytest.mark.parametrize('value', [0., -.1])
def test_uncertainty_limits_must_be_positive(monkeypatch, tmp_path, name, value):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init(args=['--ros-args', '-p', f'{name}:={value}'], domain_id=109)
    try:
        with pytest.raises(ValueError, match=name+' must be finite and positive'):
            AvoidanceNode()
    finally:
        rclpy.try_shutdown()
