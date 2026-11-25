"""Diagnostic geometry only: no hardware or motor topics."""
import math

import numpy as np
import pytest

from obstacle_avoidance_adv.planner import Controller, Settings
from obstacle_avoidance_adv.visualization import motion_intent
from test_ekf_navigation import navigation_setup
from test_ekf_observer import observer, field_message


def controller_in(state):
    controller = Controller(Settings(goal_distance=5.))
    controller.start(np.zeros(3), 0.)
    controller.change(state, .1, 'synthetic state')
    controller.leg_start = np.zeros(2)
    controller.leg_heading = 0.
    return controller


@pytest.mark.parametrize('angle, rate', [(math.pi/2, .4), (-math.pi/2, -.4)])
def test_turn_intent_uses_a_rotation_arrow(angle, rate):
    controller = controller_in('TURN_OUT')
    controller.turn_heading = angle
    result = motion_intent(controller, np.zeros(3), (0., rate))
    assert result['kind'] == 'turn'
    assert result['remaining_deg'] == pytest.approx(90.)
    assert ('LEFT' if angle > 0 else 'RIGHT') in result['label']
    np.testing.assert_allclose(np.linalg.norm(result['path'], axis=1), .28)
    np.testing.assert_allclose(result['path'][-1], [0., math.copysign(.28, angle)], atol=1e-9)


def test_pass_preview_uses_the_sent_command_and_correct_remaining_leg():
    controller = controller_in('PASS')
    controller.target = np.array([100., -100.])  # A legacy stale target must not influence this arrow.
    result = motion_intent(controller, np.array([.3, 0., 0.]), (.2, 0.))
    assert result['remaining_m'] == pytest.approx(.7)
    np.testing.assert_allclose(result['path'][0], [.3, 0.])
    np.testing.assert_allclose(result['path'][-1], [.7, 0.])  # Two seconds at 0.2 m/s.


def test_sidestep_arrow_ends_at_the_leg_end_when_it_is_near():
    controller = controller_in('SIDESTEP')
    controller.side_length = .5
    controller.turn_heading = math.pi/2
    result = motion_intent(controller, np.array([0., .4, math.pi/2]), (.2, 0.))
    assert result['remaining_m'] == pytest.approx(.1)
    np.testing.assert_allclose(result['path'][-1], [0., .5], atol=1e-9)


def test_curved_command_preview_is_a_curve():
    controller = controller_in('APPROACH')
    result = motion_intent(controller, np.zeros(3), (.2, .5))
    np.testing.assert_allclose(result['path'][-1], [.4*math.sin(1.), .4*(1.-math.cos(1.))])
    assert result['path'][-1][1] > 0.


def test_reverse_overrides_the_current_state_and_respects_backoff_distance():
    controller = controller_in('PASS')
    controller.backoff_start = np.zeros(2)
    result = motion_intent(controller, np.array([-.1, 0., 0.]), (-.05, 0.))
    assert result['kind'] == 'reverse'
    assert result['remaining_m'] == pytest.approx(.2)
    np.testing.assert_allclose(result['path'][-1], [-.2, 0.])


def test_stopped_command_does_not_show_a_movement_arrow():
    result = motion_intent(controller_in('PASS'), np.zeros(3), (0., 0.), 'EKF input unavailable')
    assert result['kind'] == 'stop'
    assert result['path'] == []
    assert result['label'] == 'EKF input unavailable'


def test_local_drift_is_visible_but_intent_is_anchored_to_corrected_pose(observer):
    node = observer
    navigation_setup(node, (-2.1, .2, .1), (1., 0., 0.))
    node.gyro_stamp = node.now()
    node.last_sent = (.2, 0.)
    before = node.pose.copy()
    result = node.motion_visualization()
    assert result['local_fresh']
    np.testing.assert_allclose(result['local_pose'], [-2.2, 0., 0.])
    np.testing.assert_allclose(result['fused_pose'], [-2.1, .2, .1])
    np.testing.assert_allclose(result['intent']['path'][0], [-2.1, .2])
    np.testing.assert_allclose(result['intent']['path'][-1],
                               [-2.1+.4*math.cos(.1), .2+.4*math.sin(.1)])
    assert result['comparison']['distance_m'] == pytest.approx(math.hypot(.1, .2))
    np.testing.assert_array_equal(node.pose, before)


def test_field_reset_hides_old_local_pose_and_action(observer):
    node = observer
    navigation_setup(node, (-2.1, .2, .1), (1., 0., 0.))
    node.gyro_stamp = node.now()
    node.on_field_state(field_message(revision=2))
    result = node.motion_visualization()
    assert result['local_pose'] is None
    assert result['intent'] is None


def test_fresh_gyro_cannot_disguise_old_wheel_pose(observer):
    node = observer
    navigation_setup(node, (-2.1, .2, .1), (1., 0., 0.))
    node.gyro_stamp = node.now()
    node.comparison_history.clear()
    node.comparison_history.append((node.now()-2., node.pose.copy()))
    result = node.motion_visualization()
    assert not result['local_fresh']
    assert result['intent'] is None


def test_idle_visualization_does_not_claim_an_initialized_local_pose(observer):
    result = observer.motion_visualization()
    assert result['state'] == 'IDLE'
    assert result['local_pose'] is None
    assert result['intent'] is None


def test_debug_path_is_a_short_map_frame_motion_preview(observer, monkeypatch):
    paths=[]
    monkeypatch.setattr(observer.path,'publish',paths.append)
    observer.publish_motion_path(dict(intent=dict(kind='drive',path=[[1.,2.],[1.2,2.1]])))
    assert paths[-1].header.frame_id == 'map'
    assert len(paths[-1].poses) == 2
    assert paths[-1].poses[-1].pose.position.x == 1.2
    q=paths[-1].poses[0].pose.orientation
    assert 2*math.atan2(q.z,q.w) == pytest.approx(math.atan2(.1,.2))


@pytest.mark.parametrize('kind',['stop','turn'])
def test_stopping_or_rotating_clears_the_previous_debug_path(observer, monkeypatch, kind):
    paths=[]
    monkeypatch.setattr(observer.path,'publish',paths.append)
    observer.publish_motion_path(dict(intent=dict(kind=kind,path=[[0.,0.],[.1,.1]])))
    assert paths[-1].poses == []
