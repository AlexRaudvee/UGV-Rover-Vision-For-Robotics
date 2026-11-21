"""Live range-to-position integration on isolated domain 92; no physical drivers."""
import json
import math
from pathlib import Path
import time

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from l2s2_localisation.geometry import rot_z, square_object_points
from l2s2_localisation.range_node import MarkerRanges
from l2s2_localisation.triangulation_node import MarkerTriangulation
from test_range_node import odometry, spin_until


@pytest.fixture
def pipeline(tmp_path):
    source = Path(__file__).resolve().parents[1]/'config'
    ranges_file, tri_file = tmp_path/'ranges.yaml', tmp_path/'tri.yaml'
    ranges_file.write_text((source/'marker_ranges.yaml').read_text())
    tri_file.write_text((source/'triangulation.yaml').read_text())
    (tmp_path/'marker_map.yaml').write_text(
        'field: {half_length: 4.5, half_width: 3, border: 0.4}\n'
        'markers:\n  APRILTAG_standard41h12:2: [0, -0.75]\n  APRILTAG_standard41h12:3: [0, 0.75]\n')
    rclpy.init(args=['--ros-args', '-p', f'config_file:={ranges_file}'], domain_id=92)
    ranges = MarkerRanges()
    tri = MarkerTriangulation(use_global_arguments=False,
                              parameter_overrides=[Parameter('config_file', value=str(tri_file))])
    observer = Node('triangulation_test_observer', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    for node in (ranges, tri, observer):
        executor.add_node(node)
    results, poses = [], []
    observer.create_subscription(String, '/marker_triangulation/result', lambda m: results.append(json.loads(m.data)), 10)
    observer.create_subscription(PoseWithCovarianceStamped, '/marker_triangulation/pose', poses.append, 10)
    spin_until(executor, lambda: ranges.observations_pub.get_subscription_count() > 0 and
               tri.pose_pub.get_subscription_count() > 0 and tri.pub.get_subscription_count() > 0)
    yield ranges, tri, executor, results, poses, tri_file
    executor.shutdown()
    for node in (ranges, tri, observer):
        node.destroy_node()
    rclpy.shutdown()


def image(ranges, ids=(2, 3), robot=(-3.2, .2, .15), pan=.4):
    stamp = ranges.now()
    ranges.commands.append((stamp-1, pan, 0, stamp-1))
    camera = ranges.cameras['rectified']
    data = dict(stamp=stamp, width=640, height=480, image_geometry='rectified', detections=[])
    for marker_id in ids:
        known = ranges.config['markers'][f'APRILTAG_standard41h12:{marker_id}']
        point_base = rot_z(-robot[2])@np.array([known[0]-robot[0], known[1]-robot[1], .15])
        tvec = ranges.mount.camera_rotation(pan, 0).T@(point_base-ranges.mount.camera_center(pan, 0))
        corners, _ = cv2.projectPoints(square_object_points(.14), np.array([math.pi-.1, 0, 0]),
                                       tvec, camera.matrix_for(data['width'], data['height']), camera.D)
        data['detections'].append(dict(id=marker_id, family='APRILTAG_standard41h12',
                                       corners=corners.reshape(4, 2).tolist(), hamming=0, decision_margin=80))
    return data


def send(ranges, tri, executor, results, data):
    ranges.on_detections(String(data=json.dumps(data)))
    spin_until(executor, lambda: tri.last_stamp == data['stamp'])
    spin_until(executor, lambda: results and results[-1].get('stamp') == data['stamp'])


def test_real_apriltag_ranges_give_independent_position_with_a_turned_camera(pipeline):
    ranges, tri, executor, results, poses, _ = pipeline
    # Deliberately incorrect odometry alignment: comparison may be wrong, but
    # the AprilTag fix must still recover the true position.
    ranges.on_field(String(data=json.dumps(dict(frame_id='map', odom_frame='odom', aligned=True,
                                                revision=7, transform=[-2, 1, 0]))))
    data = image(ranges)
    ranges.on_odom(odometry(data['stamp']))
    send(ranges, tri, executor, results, data)
    assert results[-1]['fresh'], results[-1]
    assert results[-1]['position_m'][:2] == pytest.approx([-3.2, .2], abs=.005)
    assert results[-1]['heading_deg'] == pytest.approx(math.degrees(.15), abs=.1)
    assert results[-1]['odom_position_m'] == [-2, 1, 0]
    assert results[-1]['position_delta_m'] == pytest.approx(math.hypot(1.2, .8), abs=.005)
    spin_until(executor, lambda: poses)
    assert poses[-1].header.frame_id == 'map'
    assert poses[-1].pose.pose.position.x == pytest.approx(-3.2, abs=.005)
    assert dict(tri.get_topic_names_and_types()).get('/tf') is None
    assert dict(tri.get_topic_names_and_types()).get('/cmd_vel') is None


def test_no_odometry_required_and_insufficient_or_stale_observations_clear_fix(pipeline):
    ranges, tri, executor, results, poses, _ = pipeline
    data = image(ranges)
    send(ranges, tri, executor, results, data)
    assert results[-1]['fresh'] and results[-1]['odom_position_m'] is None
    data = image(ranges, ids=(2,))
    send(ranges, tri, executor, results, data)
    assert not results[-1]['fresh'] and 'at least two' in results[-1]['error']
    count = len(poses)
    tri.on_observations(String(data=json.dumps(dict(stamp=tri.now()+.01, frame_id='map', error='stale camera image'))))
    assert not tri.result['fresh']
    assert len(poses) == count


def test_duplicates_do_not_republish_pose_and_stopped_stream_expires(pipeline):
    ranges, tri, executor, results, poses, _ = pipeline
    data = image(ranges)
    send(ranges, tri, executor, results, data)
    spin_until(executor, lambda: poses)
    before = len(poses)
    ranges.on_detections(String(data=json.dumps(data)))
    tri.received -= 2
    tri.publish()
    spin_until(executor, lambda: not results[-1]['fresh'])
    assert len(poses) == before
    assert 'stale' in results[-1]['error']


def test_config_reload_invalidates_old_fix_and_rejects_broken_edits(pipeline):
    ranges, tri, executor, results, _, path = pipeline
    send(ranges, tri, executor, results, image(ranges))
    path.write_text(path.read_text().replace('max_range_rms_m: 0.5', 'max_range_rms_m: 0.4'))
    tri.reload_config()
    assert tri.config['max_range_rms_m'] == .4 and not tri.result['fresh']
    path.write_text('max_range_rms_m: -1')
    tri.reload_config()
    assert tri.config_error and tri.config['max_range_rms_m'] == .4
