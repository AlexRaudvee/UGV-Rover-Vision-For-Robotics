"""Arrival in every active state, using synthetic FUSED poses and intercepted commands."""
import time

import numpy as np
import pytest

from l2s3_curling.planner import Controller, Settings
from test_ekf_navigation import clear_view, navigation_setup
from test_ekf_observer import observer, field_message, pose_message


ACTIVE_STATES = ['APPROACH','STOP','TURN_OUT','SIDESTEP','REPLAN',
                 'REJOIN','GAP','GAP_ALIGN','GAP_THROUGH','LOOK','LOOK_CHECK','ESCAPE']


@pytest.mark.parametrize('state', ACTIVE_STATES)
def test_fused_arrival_preempts_every_manoeuvre_without_scan_or_history(observer, monkeypatch, state):
    node = observer
    node.relocalization_result = {'outcome': 'timeout'}
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    node.controller.change(state, time.monotonic(), 'unfinished obstacle manoeuvre')
    node.pose = np.array([.4, .7, -.5])  # Deliberately unrelated wheel/gyro position.
    node.comparison_history.clear()
    node.navigation_anchor = None
    sent = []
    monkeypatch.setattr(node.command, 'publish', sent.append)
    monkeypatch.setattr(node, 'prepare', lambda: pytest.fail('Arrival attempted obstacle manoeuvres'))
    node.motion.velocity = np.array([.3, .4])
    node.last_sent = (.3, .4)
    node.on_ekf_pose(pose_message(node, x=node.goal_map[0], y=node.goal_map[1],
                                  heading=2., stamp=node.stamp(original)))
    node.tick()
    assert node.controller.state == 'DONE'
    assert len(sent) == 1
    assert sent[0].linear.x == sent[0].angular.z == 0.
    assert node.last_sent == (0., 0.)
    np.testing.assert_array_equal(node.motion.velocity, [0., 0.])
    assert node.arrival['source'] == 'FUSED'
    assert node.arrival['previous_state'] == state
    assert node.arrival['distance_m'] == pytest.approx(0.)
    assert node.arrival['pose_map'][2] == pytest.approx(2.)  # No final heading requirement.


@pytest.mark.parametrize('distance, reached', [(.079,True),(.08,True),(.0801,False)])
def test_arrival_tolerance_is_inclusive_and_configurable(distance, reached):
    controller = Controller(Settings(goal_distance=5.))
    controller.start(np.zeros(3), 0.)
    controller.change('REPLAN', .1, 'passing obstacle')
    assert controller.finish_if_at_goal(distance, .2, 'FUSED') == reached
    assert controller.state == ('DONE' if reached else 'REPLAN')
    controller = Controller(Settings(goal_distance=5., goal_tolerance_m=.15))
    controller.start(np.zeros(3), 0.)
    assert controller.finish_if_at_goal(.14, .1, 'FUSED')


def test_controller_arrival_precedes_emergency_backoff():
    controller = Controller(Settings(goal_distance=5.))
    controller.start(np.zeros(3), 0.)
    controller.change('REPLAN', .1, 'passing')
    class UnexpectedView:
        def clearance(self, *args):
            pytest.fail('Arrival evaluated emergency backoff before stopping')
    assert controller.step(np.zeros(3), UnexpectedView(), .2, arrival_distance=.02) == (0.,0.)
    assert controller.state == 'DONE'


def test_projected_local_arrival_cannot_override_a_fused_pose_outside_tolerance(observer, monkeypatch):
    node = observer
    goal_x = -3.2 + node.controller.cfg.goal_distance
    node.relocalization_result = {'outcome': 'timeout'}
    navigation_setup(node, (goal_x-.3, 0., 0.), (1., 0., 0.))
    # Local integration advances to the goal while the last published FUSED pose is still short.
    node.pose[0] += .3
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node.command, 'publish', lambda msg: None)
    node.tick()
    assert node.controller.state == 'APPROACH'
    assert node.arrival is None
    assert node.navigation_status['distance_to_goal_m'] == pytest.approx(.3)
    assert node.navigation_status['projected_distance_to_goal_m'] == pytest.approx(0., abs=1e-9)
    assert node.motion_visualization()['distance_to_goal_m'] == pytest.approx(.3)


@pytest.mark.parametrize('failure',['stale','invalid','field_reset'])
def test_unusable_fused_pose_cannot_declare_arrival(observer, monkeypatch, failure):
    node = observer
    node.relocalization_result = {'outcome': 'timeout'}
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    node.controller.change('REPLAN', time.monotonic(), 'passing obstacle')
    msg = pose_message(node, x=node.goal_map[0], y=node.goal_map[1], stamp=node.stamp(original))
    if failure == 'invalid':
        msg.pose.covariance[0] = -1.
    node.on_ekf_pose(msg)
    if failure == 'stale':
        node.ekf_received -= 2.
    elif failure == 'field_reset':
        node.on_field_state(field_message(revision=2))
    monkeypatch.setattr(node, 'prepare', lambda: pytest.fail('Invalid pose reached movement preparation'))
    sent = []
    monkeypatch.setattr(node.command, 'publish', sent.append)
    node.tick()
    assert node.controller.state == 'REPLAN'
    assert node.arrival is None
    assert sent[-1].linear.x == sent[-1].angular.z == 0.


def test_done_stays_latched_when_fused_estimate_later_moves_away(observer, monkeypatch):
    node = observer
    node.relocalization_result = {'outcome': 'timeout'}
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    node.on_ekf_pose(pose_message(node, x=node.goal_map[0], stamp=node.stamp(original)))
    sent = []
    monkeypatch.setattr(node.command, 'publish', sent.append)
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    node.tick()
    record = dict(node.arrival)
    node.on_ekf_pose(pose_message(node, x=node.goal_map[0]+.5))
    node.tick()
    assert node.controller.state == 'DONE'
    assert node.arrival == record
    assert all(msg.linear.x == msg.angular.z == 0. for msg in sent)


def test_missing_markers_do_not_prevent_fused_arrival(observer, monkeypatch):
    node = observer
    node.relocalization_result = {'outcome': 'timeout'}
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    # The controller consumes FUSED pose/covariance, not a marker-count or tracking-mode gate.
    node.on_ekf_pose(pose_message(node, x=node.goal_map[0], stamp=node.stamp(original)))
    monkeypatch.setattr(node.command, 'publish', lambda msg: None)
    node.tick()
    assert node.controller.state == 'DONE'


def test_high_uncertainty_allows_fused_arrival_and_is_recorded(observer, monkeypatch):
    node = observer
    node.relocalization_result = {'outcome': 'timeout'}
    original = navigation_setup(node, (-2.2, 0., 0.), (1., 0., 0.))
    msg = pose_message(node, x=node.goal_map[0], stamp=node.stamp(original))
    msg.pose.covariance[0] = 1.
    node.on_ekf_pose(msg)
    sent = []
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    node.tick()
    assert node.controller.state == 'DONE'
    assert sent == [(0., 0.)]
    assert node.arrival['sigma_position_m'] == pytest.approx(1.)
    assert node.arrival['sigma_heading_deg'] == pytest.approx(5.)
    assert 'exceeds limit' in node.arrival['uncertainty_warning']


