"""Live-range pipeline on isolated domain 95; no drivers or motion commands."""

import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from l2s2_localisation.geometry import square_object_points
from l2s2_localisation.range_node import MarkerRanges


@pytest.fixture
def pipeline(tmp_path):
    config = tmp_path / 'ranges.yaml'
    config.write_text((Path(__file__).resolve().parents[1] / 'config/marker_ranges.yaml').read_text())
    (tmp_path / 'marker_map.yaml').write_text((
        Path(__file__).resolve().parents[1] / 'config/marker_map.yaml').read_text())
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config}'], domain_id=95)
    node = MarkerRanges()
    client = Node('range_test_client', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client)
    results = []
    client.create_subscription(String, '/marker_ranges/result',
                               lambda msg: results.append(json.loads(msg.data)), 10)
    yield node, client, executor, results, config
    executor.shutdown()
    client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, condition, timeout=4):
    end = time.monotonic()+timeout
    while not condition() and time.monotonic() < end:
        executor.spin_once(timeout_sec=.02)
    assert condition(), 'Timed out waiting for range pipeline'


def field_state(revision=2):
    return String(data=json.dumps({'aligned': True, 'frame_id': 'map', 'odom_frame': 'odom',
                                   'transform': [-3.2, 0, 0], 'revision': revision}))


def image_data(node, tag_id=2):
    stamp = node.now()
    camera = node.cameras['rectified']
    # Marker at field (0,-.75,.15), rover at (-3.2,0,0). Camera is x=.04,z=.211.
    corners, _ = cv2.projectPoints(square_object_points(.14), np.array([math.pi-.1, 0, 0]),
                                   np.array([.75, .061, 3.16]), camera.K, camera.D)
    return {'stamp': stamp, 'width': 640, 'height': 480, 'image_geometry': 'rectified',
            'detections': [{'family': 'APRILTAG_standard41h12', 'id': tag_id,
                            'corners': corners.reshape(4, 2).tolist(),
                            'hamming': 0, 'decision_margin': 80}]}


def odometry(stamp):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
    message.pose.pose.orientation.w = 1.0
    return message


def test_published_native_distance_is_independent_of_landmark_map(pipeline):
    node, client, executor, results, _ = pipeline
    field_pub = client.create_publisher(String, '/football_field/state', 10)
    odom_pub = client.create_publisher(Odometry, '/odom', 10)
    detections_pub = client.create_publisher(String, '/marker_detector/detections', 10)
    spin_until(executor, lambda: all(p.get_subscription_count() for p in (field_pub, odom_pub, detections_pub)))
    field_pub.publish(field_state())
    spin_until(executor, lambda: node.field is not None)
    # Deliberately wrong reference coordinates must NOT determine the range.
    node.config['markers']['APRILTAG_standard41h12:2'] = [4, 4, 0]
    for _ in range(4):
        data = image_data(node)
        odom_pub.publish(odometry(data['stamp']))
        spin_until(executor, lambda: node.odom.samples and node.odom.samples[-1][0] >= data['stamp']-.00001)
        detections_pub.publish(String(data=json.dumps(data)))
        previous = len(node.filters.get('APRILTAG_standard41h12:2', []).samples) if 'APRILTAG_standard41h12:2' in node.filters else 0
        spin_until(executor, lambda: len(node.filters.get('APRILTAG_standard41h12:2').samples) > previous if 'APRILTAG_standard41h12:2' in node.filters else False)
    spin_until(executor, lambda: results and any(m.get('samples', 0) >= 4 for m in results[-1]['markers']))
    marker = next(m for m in results[-1]['markers'] if m['id'] == 2)
    assert marker['fresh']
    assert marker['range_m'] == pytest.approx(math.hypot(3.2, .75), abs=.002)
    assert marker['observed_position_m'] == pytest.approx([0, -.75, .15], abs=.002)
    assert marker['map_position_m'] == [4, 4, 0]
    assert marker['low_resolution']
    assert marker['camera_distance_m'] != pytest.approx(marker['range_m'], abs=.01)
    topics = dict(node.get_topic_names_and_types())
    assert '/cmd_vel' not in topics and '/tf' not in topics


def test_stale_reads_and_field_reinitialization_do_not_show_live_ranges(pipeline):
    node, _client, executor, results, _ = pipeline
    node.on_field(field_state())
    data = image_data(node)
    node.on_odom(odometry(data['stamp']))
    node.on_detections(String(data=json.dumps(data)))
    key = 'APRILTAG_standard41h12:2'
    assert node.records[key]['range_m'] > 3
    node.records[key]['stamp'] -= 2
    node.publish()
    spin_until(executor, lambda: results and any(m.get('age_s', 0) >= 2 for m in results[-1]['markers']))
    marker = next(m for m in results[-1]['markers'] if m['id'] == 2)
    assert not marker['fresh']
    node.on_field(field_state(revision=3))
    assert not node.records and not node.filters


def test_config_reload_and_rejections_preserve_valid_settings(pipeline):
    node, _client, _executor, _results, config = pipeline
    config.write_text(config.read_text().replace('marker_size_m: 0.14', 'marker_size_m: 0.12'))
    node.reload_config()
    assert node.config['marker_size_m'] == .12
    config.write_text('markers: [unfinished')
    node.reload_config()
    assert node.config_error.startswith('Range config not loaded:')
    assert node.config['marker_size_m'] == .12
    data = image_data(node)
    data['detections'][0]['decision_margin'] = 1
    node.on_detections(String(data=json.dumps(data)))
    assert 'margin' in node.rejections['APRILTAG_standard41h12:2']
    assert node.config_error


def test_image_exposure_before_head_settles_is_rejected(pipeline):
    node, _client, _executor, _results, _config = pipeline
    stamp = node.now()
    node.commands.append((stamp-.1, .5, 0, stamp-.1))
    with pytest.raises(ValueError, match='settling'):
        node.head_at(stamp)
    assert node.head_at(stamp+1) == (.5, 0)


def test_malformed_field_alignment_and_map_do_not_replace_valid_state(pipeline):
    node, _client, _executor, _results, config = pipeline
    node.on_field(field_state())
    previous = node.field
    node.on_field(String(data=json.dumps({'frame_id': 'map', 'aligned': True,
                                         'transform': [float('nan'), 0, 0]})))
    assert node.field is previous
    (config.parent / 'marker_map.yaml').write_text('[not, a, mapping]')
    node.reload_config()
    assert node.config_error.startswith('Range config not loaded:')
    assert node.config['markers']['APRILTAG_standard41h12:2'] == [0, -.75, 0]


def test_unconfigured_tag_does_not_inherit_assumed_size_or_map_location(pipeline):
    node, _client, _executor, _results, _config = pipeline
    data = image_data(node, tag_id=0)
    node.on_detections(String(data=json.dumps(data)))
    assert not node.records
    assert 'not in the configured marker map' in node.rejections['APRILTAG_standard41h12:0']
