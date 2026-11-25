"""Synthetic confidence checks on isolated ROS domain 109; no motor commands."""
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
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
    (.54, 20.56, True),  # Previously faulted during Run 1.
    (.80, 30., True),  # Limits are inclusive.
    (.800001, 5., False),
    (.15, 30.0001, False),
    (.90, 35., False),
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
    assert report['max_position_std_m'] == .80
    assert report['max_heading_std_deg'] == 30.
    if accepted:
        assert report['uncertainty_reason'] == ''
    else:
        assert 'exceeds limit' in report['uncertainty_reason']
    assert observer.require_ekf_pose()['within_uncertainty_limits'] == accepted


def test_correlated_position_error_is_checked_along_its_largest_direction(observer):
    msg = with_uncertainty(pose_message(observer), .7, 5.)
    # Each axis is under the 0.80 m limit, but correlation raises the diagonal
    # standard deviation to sqrt(0.69), about 0.83 m.
    msg.pose.covariance[1] = msg.pose.covariance[6] = .20
    observer.on_ekf_pose(msg)
    report = observer.ekf_diagnostics()
    assert report['valid'] and report['fresh']
    assert report['sigma_x_m'] == pytest.approx(.7)
    assert report['sigma_y_m'] == pytest.approx(.7)
    assert report['sigma_position_m'] == pytest.approx(math.sqrt(.69))
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
    assert expected_reason in observer.require_ekf_pose()['uncertainty_reason']


def test_high_uncertainty_allows_start(observer, monkeypatch):
    node = observer
    node.on_field_state(field_message())
    node.on_ekf_pose(with_uncertainty(pose_message(node), .9, 35.))
    node.latest_odom = Odometry()
    node.latest_odom.pose.pose.orientation.w = 1.
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    response = node.start(Trigger.Request(), Trigger.Response())
    assert response.success, response.message
    assert node.controller.state == 'APPROACH'
    assert 'exceeds limit' in node.motion_visualization()['ekf_warning']


@pytest.mark.parametrize('position_std, heading_std', [(.9, 5.), (.15, 35.)])
def test_high_uncertainty_warns_while_motion_continues(observer, monkeypatch, position_std, heading_std):
    node = observer
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    sent = []
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    node.tick()
    node.motion.clear_since -= .3
    node.motion.last_time -= .1
    uncertain = pose_message(node, x=-2.2, stamp=node.stamp(original))
    node.on_ekf_pose(with_uncertainty(uncertain, position_std, heading_std))
    node.tick()
    assert sent[-1][0] > 0.
    assert node.hold_since is None
    assert node.navigation_status['ready']
    assert 'exceeds limit' in node.motion_visualization()['ekf_warning']

    node.on_ekf_pose(original)
    assert node.motion_visualization()['ekf_warning'] == ''


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
