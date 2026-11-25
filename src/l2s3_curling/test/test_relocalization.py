"""Observe the real controller tick with synthetic FUSED poses and camera requests."""

import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import String
from std_srvs.srv import Trigger

import l2s3_curling.node as node_module
from test_ekf_navigation import clear_view, navigation_setup
from test_ekf_observer import observer, pose_message


@pytest.fixture
def recovery(observer, monkeypatch):
    node = observer
    time_s = [time.monotonic(), node.now()]
    monkeypatch.setattr(node_module, 'time', SimpleNamespace(monotonic=lambda: time_s[0]))
    monkeypatch.setattr(node, 'now', lambda: time_s[1])
    goal_x = -3.2 + node.controller.cfg.goal_distance
    navigation_setup(node, (goal_x-.7, 0., 0.), (2., 0., 0.))
    monkeypatch.setattr(node, 'prepare', lambda: clear_view(node))
    requests, commands = [], []
    monkeypatch.setattr(node.camera_search, 'get_subscription_count', lambda: 1)
    monkeypatch.setattr(node.camera_search, 'publish', lambda msg: requests.append(json.loads(msg.data)))
    monkeypatch.setattr(node.command, 'publish', lambda msg: commands.append((msg.linear.x, msg.angular.z)))

    def update(elapsed=0., distance=.7, position_std=.35, heading_std=12., age=3., stamps=None):
        time_s[0] += elapsed
        time_s[1] += elapsed
        node.field_received = time_s[0]
        stamp = node.now()-.05
        msg = pose_message(node, x=goal_x-distance, stamp=stamp)
        msg.pose.covariance[0] = position_std**2
        msg.pose.covariance[35] = math.radians(heading_std)**2
        node.comparison_history.clear()
        node.comparison_history.extend([(stamp-.02, node.pose.copy()), (stamp+.02, node.pose.copy())])
        node.on_ekf_pose(msg)
        node.on_marker_state(String(data=json.dumps(dict(initialised=True, frame_id='map',
            field_session='synthetic', field_revision=1, pose_stamp_s=stamp, seconds_since_marker=age,
            accepted_marker_stamps_s={'family:5': stamp-3.} if stamps is None else stamps))))
        node.tick()

    return node, update, requests, commands


def test_recovers_from_two_new_accepted_ids_and_lower_uncertainty(recovery):
    node, update, requests, commands = recovery
    update()
    start = node.relocalization['start_stamp_s']
    assert node.controller.state == 'RELOCALIZE'
    assert requests == [{'deadline_s': start+8., 'marker_id': 5}]
    # One new tag can temporarily put FUSED inside the arrival radius.
    update(.4, distance=.04, position_std=.19, heading_std=6., age=.1,
           stamps={'family:5': start-1., 'family:1': start+.15})
    assert node.controller.state == 'RELOCALIZE' and node.arrival is None
    assert commands == [(0., 0.), (0., 0.)]
    # A second ID alone is insufficient if the estimate is still uncertain.
    stamps = {'family:1': start+.15, 'family:2': start+.55}
    update(.4, age=.1, stamps=stamps)
    assert node.controller.state == 'RELOCALIZE'
    update(.4, position_std=.19, heading_std=6., age=.2, stamps=stamps)
    assert node.controller.state == 'APPROACH'
    assert node.relocalization_result['outcome'] == 'recovered'
    assert node.relocalization_result['accepted_tags'] == ['family:1', 'family:2']
    assert requests[-1] == {'deadline_s': 0}
    assert all(command == (0., 0.) for command in commands)


def test_eight_second_timeout_resumes_once_through_the_motion_limiter(recovery):
    node, update, requests, commands = recovery
    update()
    update(7.95)
    assert node.controller.state == 'RELOCALIZE'
    update(.055)
    assert node.controller.state == 'APPROACH'
    assert node.relocalization_result['outcome'] == 'timeout'
    assert node.relocalization_result['elapsed_s'] == pytest.approx(8.005)
    assert commands[-1] == (0., 0.)
    update(.4)
    assert node.controller.state == 'APPROACH'
    assert commands[-1][0] > 0.
    assert len(requests) == 2


@pytest.mark.parametrize('state, distance', [('PASS', .7), ('APPROACH', 1.51)])
def test_eligibility_limits_the_pause_to_final_approach(recovery, state, distance):
    node, update, requests, _ = recovery
    if state == 'PASS':
        node.controller.leg_start = np.asarray(node.pose[:2]).copy()
    node.controller.change(state, time.monotonic(), 'driving')
    update(distance=distance)
    assert node.relocalization is None and not requests


@pytest.mark.parametrize('second_tag_age, expected', [(2., True), (1., False)])
def test_recent_single_tag_can_trigger_search_earlier(recovery, second_tag_age, expected):
    node, update, requests, _ = recovery
    now = node.now()
    update(distance=1.4, position_std=.22, heading_std=6., age=.1,
           stamps={'family:5': now-.1, 'family:2': now-second_tag_age})
    assert (node.controller.state == 'RELOCALIZE') == expected
    assert bool(requests) == expected


def test_operator_stop_cancels_the_camera_lease_without_resuming(recovery):
    node, update, requests, commands = recovery
    update()
    node.stop(Trigger.Request(), Trigger.Response())
    assert node.controller.state == 'IDLE'
    assert node.relocalization_result['outcome'] == 'operator_stop'
    assert requests[-1] == {'deadline_s': 0}
    assert commands[-1] == (0., 0.)
