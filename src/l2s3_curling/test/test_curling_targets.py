"""Select and reach each assignment target using synthetic EKF poses only."""
import time
import json

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from l2s3_curling.node import CurlingControllerNode
from test_ekf_navigation import clear_view
from test_ekf_observer import observer, field_message, pose_message


# Independent expected values from assignment Table 2, in field metres.
RUNS = [
    ({'target_run': 1}, (3.20, 0.00)),
    ({'target_run': 2}, (3.90, 1.10)),
    ({'target_run': 3}, (3.90, -1.10)),
    ({'target_run': 4}, (2.85, 2.00)),
    ({'target_run': 5}, (2.85, -2.00)),
]


def start_synthetic_mission(node, monkeypatch, start_x=-3.2):
    node.on_field_state(field_message(start_x=start_x))
    node.on_ekf_pose(pose_message(node, x=start_x))
    node.latest_odom = Odometry()
    node.latest_odom.pose.pose.orientation.w = 1.
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Start sent a command'))
    response = node.start(Trigger.Request(), Trigger.Response())
    assert response.success
    return response


def matched_pose(node, field_position, local_pose=(0., 0., 0.)):
    # Supply matching local samples without running timers, sensors or motors.
    msg = pose_message(node, x=field_position[0], y=field_position[1])
    stamp = node.stamp(msg)
    node.pose = np.array(local_pose, dtype=float)
    node.comparison_history.clear()
    node.comparison_history.extend([(stamp - .05, node.pose.copy()), (stamp + .05, node.pose.copy())])
    node.on_ekf_pose(msg)


@pytest.mark.parametrize('observer, target', RUNS, indirect=['observer'])
def test_status_previews_selected_goal_while_idle_without_motion(observer, monkeypatch, target):
    node = observer
    statuses = []
    monkeypatch.setattr(node.status, 'publish', lambda msg: statuses.append(json.loads(msg.data)))
    monkeypatch.setattr(node, 'send', lambda *args: pytest.fail('Goal preview sent a command'))
    # No field alignment, sensors or EKF input is needed to preview a fixed curling target.
    node.tick()
    assert node.controller.state == 'IDLE'
    assert statuses[-1]['goal']['frame_id'] == 'map'
    assert statuses[-1]['goal']['target_run'] == node.value('target_run')
    np.testing.assert_allclose(statuses[-1]['goal']['position_m'], target)
    assert node.goal_map is None  # Preview does not start or alter a mission.


def test_distance_goal_preview_waits_for_assumed_start(observer):
    assert observer.goal_visualization() is None
    observer.on_field_state(field_message(start_x=-2.5))
    np.testing.assert_allclose(observer.goal_visualization()['position_m'],
                               [-2.5 + observer.controller.cfg.goal_distance, 0.])


@pytest.mark.parametrize('observer, target', RUNS, indirect=['observer'])
@pytest.mark.parametrize('start_x', [-3.2, -2.5])
def test_target_is_absolute_and_does_not_shift_with_assumed_start_or_ekf_correction(
        observer, monkeypatch, target, start_x):
    node = observer
    response = start_synthetic_mission(node, monkeypatch, start_x)
    np.testing.assert_allclose(node.goal_map, target)
    np.testing.assert_allclose(node.goal_visualization()['position_m'], target)
    np.testing.assert_allclose(node.controller.goal, np.array(target) - [start_x, 0.])
    assert node.target_name in response.message
    matched_pose(node, (start_x + .3, .2), (.2, 0., 0.))
    node.update_navigation_goal(node.require_ekf_pose())
    np.testing.assert_allclose(node.goal_map, target)
    np.testing.assert_allclose(node.navigation_status['goal_map_m'], target)
    assert node.navigation_status['target_run'] == node.value('target_run')
    assert node.navigation_status['target_name'] == node.target_name


@pytest.mark.parametrize('observer, target', RUNS, indirect=['observer'])
def test_approach_steers_towards_the_selected_target(observer, monkeypatch, target):
    node = observer
    start_synthetic_mission(node, monkeypatch)
    matched_pose(node, (-3.2, 0.))
    node.update_navigation_goal(node.require_ekf_pose())
    v, w = node.controller.step(node.pose, clear_view(node), time.monotonic())
    if target[1] == 0.:
        assert v > 0. and w == 0.
    else:
        assert w * target[1] > 0.  # Positive field y requires a left turn.
    assert node.controller.state == 'APPROACH'


@pytest.mark.parametrize('observer, target', RUNS, indirect=['observer'])
def test_control_loop_stops_at_each_ekf_field_target(observer, monkeypatch, target):
    node = observer
    start_synthetic_mission(node, monkeypatch)
    # Wheel/gyro displacement deliberately disagrees with travel across the field.
    matched_pose(node, target, (.4, .1, .2))
    sent = []
    monkeypatch.setattr(node, 'send', lambda v, w: sent.append((v, w)))
    node.tick()
    assert sent[-1] == (0., 0.)
    assert node.controller.state == 'DONE'
    assert node.navigation_status['distance_to_goal_m'] == pytest.approx(0.)
    np.testing.assert_allclose(node.goal_map, target)
    np.testing.assert_array_equal(node.pose, [.4, .1, .2])


@pytest.mark.parametrize('target_run', [-1, 6])
def test_invalid_run_is_rejected_before_publishing_commands(monkeypatch, tmp_path, target_run):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init(args=['--ros-args', '-p', f'target_run:={target_run}'], domain_id=109)
    try:
        with pytest.raises(ValueError, match='target_run must be'):
            CurlingControllerNode()
    finally:
        rclpy.try_shutdown()
