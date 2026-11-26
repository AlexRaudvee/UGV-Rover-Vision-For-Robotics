"""Exercise the real decision, provenance and file output without rover hardware."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage

from l2s3_curling.decision_log import DecisionLog
from l2s3_curling.planner import Controller, Settings, View
from test_ekf_navigation import navigation_setup
from test_ekf_observer import observer
from test_curling_gap import box


def scan(points, stamp):
    return SimpleNamespace(points=np.asarray(points), stamp=stamp, observed=lambda point: True)


def image(stamp, data):
    msg = CompressedImage(format='jpeg', data=data)
    msg.header.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
    return msg


@pytest.mark.parametrize('source', ['lidar_current', 'lidar_history'])
def test_stop_records_the_actual_blocking_source_and_nearby_images(observer, tmp_path, monkeypatch, source):
    node = observer
    navigation_setup(node, (-3.2, 0., 0.), (0., 0., 0.))
    stamp = node.now()
    cfg = node.controller.cfg
    historical = [[.34, 0.]] if source == 'lidar_history' else [[.70, 0.]]
    current = [[.34, 0.]] if source == 'lidar_current' else [[.80, 0.]]
    view = View([scan(historical, stamp-.20), scan(current, stamp-.06)], node.pose,
                cfg, .06)
    monkeypatch.setattr(node, 'prepare', lambda: view)
    monkeypatch.setattr(node, 'send', lambda v, w: None)
    warnings = []
    log = DecisionLog(tmp_path, warnings.append)
    node.decision_log = log
    expected_frame = b'frame near decision'
    try:
        log.on_image('oak_color', image(stamp-.35, b'frame at observation'))
        log.on_image('oak_color', image(stamp-.05, b'frame near decision'))
        node.tick()
        # Repeating a hold in the same state should not create more snapshots.
        node.log_decision('STOP', node.controller.decision, node.now())
    finally:
        log.close()

    assert not warnings
    lines = (log.directory / 'decisions.jsonl').read_text().splitlines()
    assert len(lines) == 1
    event = json.loads(lines[0])
    assert (event['previous_state'], event['state'], event['kind']) == ('APPROACH', 'STOP', 'stop')
    assert event['threshold_m'] == pytest.approx(cfg.stop_clearance+cfg.distance_tolerance)
    assert event['clearance']['measured_m'] == pytest.approx(.34-cfg.half_length)
    point, = event['clearance']['points']
    assert point['source'] == source
    assert point['age_s'] == pytest.approx(event['stamp_s']-point['stamp_s'])
    assert (log.directory / event['images']['oak_color']['path']).read_bytes() == expected_frame
    assert set(event['images']) == {'oak_color'}


def test_camera_uses_frame_near_decision_time(tmp_path):
    warnings = []
    log = DecisionLog(tmp_path, warnings.append)
    try:
        log.on_image('oak_color', image(99.9, b'frame near decision'))
        log.enqueue(dict(kind='stop', stamp_s=100.))
    finally:
        log.close()
    event = json.loads((log.directory / 'decisions.jsonl').read_text())
    assert event['images']['oak_color']['matched_to'] == 'decision'
    assert not warnings


def test_gap_records_both_sides_and_the_selected_candidate():
    cfg = Settings(goal_distance=5.)
    controller = Controller(cfg)
    pose = np.array([.1, 0., 0.])
    controller.start(pose, 0.)
    view = View([scan(np.vstack([box(1.8, 0.), box(1.8, .85)]), 99.8)], pose, cfg, .1)
    controller.step(pose, view, .1)
    record = controller.decision
    assert record['kind'] == 'gap'
    assert record['threshold_m'] == cfg.gap_lookahead
    gap = record['options']['gap']
    assert gap['selected_offset_m'] in gap['viable_offsets_m']
    assert {p['source'] for p in gap['left_points']} == {'lidar_current'}
    assert {p['source'] for p in gap['right_points']} == {'lidar_current'}


def test_emergency_stop_and_reverse_report_the_same_clearance_check():
    cfg = Settings(goal_distance=5.)
    controller = Controller(cfg)
    pose = np.zeros(3)
    controller.start(pose, 0.)
    view = View([scan([[.17, 0.]], 99.9)], pose, cfg, .1)
    assert controller.step(pose, view, .1) == (0., 0.)
    assert controller.decision['kind'] == 'backoff_stop'
    assert controller.step(pose, view, 1.2)[0] < 0.
    assert controller.decision['kind'] == 'backoff'
    assert controller.decision['threshold_m'] == cfg.emergency_clearance
    assert controller.decision['clearance']['points'][0]['source'] == 'lidar_current'


def test_sidestep_records_why_the_other_side_was_rejected():
    cfg = Settings(goal_distance=5.)
    controller = Controller(cfg)
    pose = np.zeros(3)
    controller.start(pose, 0.)
    view = View([scan([[.34, 0.], [0., -.31]], 99.9)], pose, cfg, .1)
    for now in (.1, .2, .6):
        controller.step(pose, view, now)
    record = controller.decision
    assert controller.state == 'TURN_OUT' and record['kind'] == 'sidestep'
    assert record['options']['selected_side'] == 'left'
    right = record['options']['sides']['right']
    assert right['rejected_by'] == 'obstacle'
    assert right['points'][0]['source'] == 'lidar_current'
