"""EKF goal guidance with synthetic poses; never launches hardware or uses motor topics."""
import math
import time

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from obstacle_avoidance_adv.planner import Controller, Settings, View, ekf_goal_guidance, wrap
from test_ekf_observer import observer, comparison_setup, field_message, pose_message


class ObservedEmptySpace:
    points = np.empty((0, 2))

    def observed(self, point):
        return True


def clear_view(node):
    # All clearance checks run against synthetic, observed empty space.
    return View([ObservedEmptySpace()], node.pose, [], node.controller.cfg, 0.)


def navigation_setup(node, field_pose, motion_pose):
    msg = pose_message(node, *field_pose)
    comparison_setup(node, msg, motion_pose)
    node.goal_map = np.array([-3.2 + node.controller.cfg.goal_distance, 0.])
    node.update_navigation_goal(node.require_ekf_pose())
    return msg


@pytest.mark.parametrize('ekf, local_at_capture, local_now', [
    ((-2.2, .4, 0.), (1., 0., 0.), (1., 0., 0.)),
    ((-2.2, .4, .3), (1., 0., 0.), (1.2, .1, .1)),
    ((-.5, 1., -3.1), (2., .3, 3.1), (2.1, .2, -3.)),
])
def test_guidance_preserves_field_distance_and_heading_error(ekf, local_at_capture, local_now):
    goal_map = np.array([1.8, 0.])
    goal_local, pose_map = ekf_goal_guidance(goal_map, ekf, local_at_capture, local_now)
    local_delta = goal_local - np.array(local_now[:2])
    field_delta = goal_map - pose_map[:2]
    assert np.linalg.norm(local_delta) == pytest.approx(np.linalg.norm(field_delta))
    local_error = wrap(math.atan2(local_delta[1], local_delta[0]) - local_now[2])
    field_error = wrap(math.atan2(field_delta[1], field_delta[0]) - pose_map[2])
    assert wrap(local_error - field_error) == pytest.approx(0., abs=1e-9)


def test_delayed_pose_is_advanced_by_local_motion():
    goal, current = ekf_goal_guidance((1.8, 0.), (-2.2, 0., 0.), (1., 0., 0.), (1.4, 0., 0.))
    np.testing.assert_allclose(goal, [5., 0.])
    np.testing.assert_allclose(current, [-1.8, 0., 0.])


def test_lateral_ekf_correction_changes_goal_steering_without_changing_local_pose(observer):
    navigation_setup(observer, (-2.2, .4, 0.), (1., 0., 0.))
    before = observer.pose.copy()
    goal_map = observer.goal_map.copy()
    v, w = observer.controller.step(observer.pose, clear_view(observer), time.monotonic())
    assert v > 0. and w < 0.  # Correct back towards the field y=0 goal.
    np.testing.assert_allclose(observer.controller.goal, [observer.controller.cfg.goal_distance, -.4])
    np.testing.assert_array_equal(observer.pose, before)
    np.testing.assert_array_equal(observer.goal_map, goal_map)


def test_ekf_heading_correction_changes_the_command(observer):
    navigation_setup(observer, (-2.2, 0., .3), (1., 0., 0.))
    v, w = observer.controller.step(observer.pose, clear_view(observer), time.monotonic())
    assert v == 0. and w < 0.  # Local gyro heading alone would command straight travel.
    assert observer.pose[2] == 0.
    assert observer.navigation_status['pose_map'][2] == pytest.approx(.3)


@pytest.mark.parametrize('state', ['APPROACH', 'REJOIN'])
def test_arrival_uses_ekf_distance_even_when_wheel_distance_disagrees(observer, state):
    field_goal_x = -3.2 + observer.controller.cfg.goal_distance
    navigation_setup(observer, (field_goal_x, 0., 0.), (.7, 0., 0.))
    observer.controller.change(state, time.monotonic(), 'synthetic approach')
    v, w = observer.controller.step(observer.pose, clear_view(observer), time.monotonic())
    assert (v, w) == (0., 0.)
    assert observer.controller.state == 'DONE'
    assert observer.navigation_status['distance_to_goal_m'] == pytest.approx(0.)
    assert observer.pose[0] == .7


def test_wheel_endpoint_alone_does_not_finish_the_mission(observer):
    distance = observer.controller.cfg.goal_distance
    navigation_setup(observer, (-3.2 + distance - 1., 0., 0.), (distance, 0., 0.))
    v, w = observer.controller.step(observer.pose, clear_view(observer), time.monotonic())
    assert v > 0. and w == 0.
    assert observer.controller.state == 'APPROACH'
    assert observer.navigation_status['distance_to_goal_m'] == pytest.approx(1.)


def test_control_loop_steers_then_stops_on_ekf_arrival(observer, monkeypatch):
    node = observer
    msg = navigation_setup(node, (-2.2, .4, 0.), (1., 0., 0.))
    sent = []
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    # Acceleration limiting has separate tests; expose the controller's decision here.
    monkeypatch.setattr(node.motion, 'update', lambda v, w, now: (v, w))
    node.tick()
    assert sent[-1][0] > 0. and sent[-1][1] < 0.
    assert node.navigation_status['ready']
    fixed_goal = node.goal_map.copy()
    node.on_ekf_pose(pose_message(node, x=fixed_goal[0], stamp=node.stamp(msg)))
    node.tick()
    assert sent[-1] == (0., 0.)
    assert node.controller.state == 'DONE'
    np.testing.assert_array_equal(node.pose, [1., 0., 0.])
    np.testing.assert_array_equal(node.goal_map, fixed_goal)


def test_new_goal_representation_does_not_complete_a_physical_sidestep():
    controller = Controller(Settings(goal_distance=5.))
    controller.start(np.zeros(3), 0.)
    controller.change('SIDESTEP', .1, 'synthetic bypass')
    controller.leg_start = np.array([0., 0.])
    controller.turn_heading = math.pi / 2
    controller.side_length = .5
    controller.target = np.array([0., .5])
    # Even a large change of the final goal must not move a committed bypass target.
    controller.set_goal((100., -100.))
    pose = np.array([0., .1, math.pi / 2])
    view = View([ObservedEmptySpace()], pose, [], controller.cfg, 0.)
    v, w = controller.step(pose, view, .2)
    assert controller.state == 'SIDESTEP'
    assert v > 0.
    np.testing.assert_array_equal(controller.target, [0., .5])
    np.testing.assert_array_equal(controller.leg_start, [0., 0.])


@pytest.mark.parametrize('failure', ['missing_pose', 'stale_pose', 'missing_field'])
def test_start_rejects_missing_navigation_input_without_commands(observer, monkeypatch, failure):
    node = observer
    node.on_field_state(field_message())
    node.on_ekf_pose(pose_message(node))
    if failure == 'missing_pose':
        node.latest_ekf_pose = None
    elif failure == 'stale_pose':
        node.ekf_received -= 2.
    else:
        node.field_reference = None
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Rejected start sent a command'))
    response = node.start(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert not node.controller.active


def test_start_fixes_goal_to_assumed_start_instead_of_initial_ekf_estimate(observer, monkeypatch):
    node = observer
    node.on_field_state(field_message())
    node.on_ekf_pose(pose_message(node, x=-3.))  # Marker correction differs from assumed start.
    node.latest_odom = Odometry()
    node.latest_odom.pose.pose.orientation.w = 1.
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Start sent a command'))
    response = node.start(Trigger.Request(), Trigger.Response())
    assert response.success
    np.testing.assert_allclose(node.goal_map, [-3.2 + node.controller.cfg.goal_distance, 0.])
    assert not node.navigation_status['ready']  # Wait for a matched sample before commanding motion.


@pytest.mark.parametrize('failure', ['stale_pose', 'field_reset'])
def test_navigation_input_failure_holds_motion_and_eventually_faults(observer, monkeypatch, failure):
    node = observer
    navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    goal_before = node.goal_map.copy()
    sent = []
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    if failure == 'stale_pose':
        node.ekf_received -= 2.
    else:
        node.on_field_state(field_message(revision=2))
    node.tick()
    assert sent[-1] == (0., 0.)
    assert not node.navigation_status['ready']
    assert node.hold_since is not None
    node.hold_since -= 6.
    node.tick()
    assert node.controller.state == 'FAULT'
    assert sent[-1] == (0., 0.)
    np.testing.assert_array_equal(node.goal_map, goal_before)


def test_pending_wheel_callback_keeps_only_a_fresh_matched_alignment(observer):
    node = observer
    msg = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    old_stamp = node.navigation_anchor['stamp_s']
    # The newest EKF timestamp is just after the newest wheel sample.
    # Keep both the pending message and the original alignment in the recent past.
    node.comparison_history[-1] = (node.stamp(msg) + .02, node.pose.copy())
    pending_stamp = node.comparison_history[-1][0] + .01
    node.on_ekf_pose(pose_message(node, x=-2.1, stamp=pending_stamp))
    report = node.require_ekf_pose()
    assert not report['comparison']['ready']
    node.update_navigation_goal(report)
    assert node.navigation_status['ready']
    assert node.navigation_anchor['stamp_s'] == old_stamp
    node.comparison_history.append((pending_stamp + .02, node.pose.copy()))
    node.update_navigation_goal(node.require_ekf_pose())
    assert node.navigation_anchor['stamp_s'] == pytest.approx(pending_stamp)
    # Continuous newer messages must never keep an old alignment alive indefinitely.
    node.comparison_history.clear()
    node.navigation_anchor['stamp_s'] -= 2.
    with pytest.raises(ValueError, match='alignment is stale'):
        node.update_navigation_goal(node.require_ekf_pose())


def test_ekf_clock_regression_does_not_reuse_a_newer_alignment(observer):
    node = observer
    msg = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    node.on_ekf_pose(pose_message(node, x=-2.3, stamp=node.stamp(msg) - .05))
    with pytest.raises(ValueError, match='time moved backwards'):
        node.update_navigation_goal(node.require_ekf_pose())
