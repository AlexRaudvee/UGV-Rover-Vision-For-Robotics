"""EKF-owned field alignment on isolated ROS domain 91; no hardware or cmd_vel."""

import json
import math
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from l2s2_localisation.ekf_node import MarkerEKFNode
from l2s2_localisation.ranging import planar_pose
from ugv_bringup.football_field import FootballField


def spin(executor, condition=lambda: False, seconds=.15):
    until = time.monotonic() + seconds
    while time.monotonic() < until and not condition():
        executor.spin_once(timeout_sec=.01)


@pytest.fixture
def pipeline(tmp_path, request):
    mode = getattr(request, 'param', 'owner')
    source = Path(__file__).resolve().parents[2] / 'ugv_main/ugv_bringup/param/football_field.yaml'
    config = tmp_path / 'field.yaml'
    config.write_text(source.read_text())
    managed = mode != 'conflict'
    owner = mode != 'pose_only'
    rclpy.init(args=['--ros-args', '-p', f'football_field:config_file:={config}',
                    '-p', f'football_field:ekf_localization:={str(managed).lower()}',
                    '-p', f'marker_ekf:publish_tf:={str(owner).lower()}'], domain_id=91)
    field = FootballField()
    ekf = MarkerEKFNode()
    client = Node('ekf_tf_test_client', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    for node in (field, ekf, client):
        executor.add_node(node)
    poses, transforms, states = [], [], []
    client.create_subscription(PoseWithCovarianceStamped, '/marker_ekf/pose', poses.append, 10)
    client.create_subscription(TFMessage, '/tf', lambda msg: transforms.extend(msg.transforms),
                               qos_profile_sensor_data)
    client.create_subscription(String, '/marker_ekf/state', lambda m: states.append(json.loads(m.data)), 10)

    def feed(x=1.2, y=-.4, h=.7):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
        msg.header.stamp = client.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(h/2), math.cos(h/2)
        field.on_odom(msg)
        ekf._on_odom(msg)
        field.tick()
        spin(executor)
        return msg

    spin(executor, seconds=.3)
    yield field, ekf, client, executor, feed, poses, transforms, states
    executor.shutdown()
    for node in (client, ekf, field):
        node.destroy_node()
    rclpy.shutdown()


def observations(field, ekf, pose, stamp=None):
    markers = []
    for index, (x, y) in enumerate(((0., -.75), (0., .75), (4., 2.))):
        dx, dy = x-pose[0], y-pose[1]
        markers.append(dict(label=f'tag:{index}', known_position_m=[x, y],
                            range_m=math.hypot(dx, dy),
                            bearing_deg=math.degrees(math.atan2(dy, dx)-pose[2]), side_px=25.))
    return String(data=json.dumps(dict(stamp=ekf.latest_odom[0] if stamp is None else stamp,
        frame_id='map', field_revision=field.revision, field_session=field.session_id,
        observations=markers, odom_pose=[99., 99., 2.])))


def test_saved_start_bootstraps_without_markers_and_tf_matches_pose(pipeline):
    field, ekf, _client, executor, feed, poses, transforms, _states = pipeline
    msg = feed()
    assert ekf.ekf.initialised
    assert ekf.ekf.state == pytest.approx([-3.2, 0., 0.])
    assert field.config['publish_tf'] is False
    ekf._publish()
    spin(executor, lambda: bool(poses and transforms), seconds=3)
    assert poses and transforms
    tf, pose = transforms[-1], poses[-1]
    assert (tf.header.frame_id, tf.child_frame_id) == ('map', 'odom')
    assert tf.header.stamp == pose.header.stamp == msg.header.stamp
    t = tf.transform
    transform = (t.translation.x, t.translation.y,
                 math.atan2(2*t.rotation.w*t.rotation.z, 1-2*t.rotation.z*t.rotation.z))
    assert planar_pose(transform, ekf.latest_odom[1]) == pytest.approx([-3.2, 0., 0.])
    assert ekf.last_update is None, 'saved start incorrectly reported a marker observation'


def test_marker_correction_changes_tf_and_field_without_rewriting_odometry(pipeline):
    field, ekf, _client, executor, feed, poses, transforms, states = pipeline
    feed()
    feed()
    odom_before = ekf.latest_odom
    before = ekf.ekf.state.copy()
    ekf._on_observations(observations(field, ekf, (-3.0, .15, .08)))
    corrected = ekf.ekf.state.copy()
    assert np.linalg.norm(corrected-before) > .05
    assert ekf.latest_odom == odom_before
    feed()  # Next odometry sample publishes the new correction and pose together.
    ekf._publish()
    spin(executor, seconds=.3)
    field.tick()
    assert poses and transforms
    tf = transforms[-1].transform
    transform = (tf.translation.x, tf.translation.y,
                 math.atan2(2*tf.rotation.w*tf.rotation.z, 1-2*tf.rotation.z**2))
    assert planar_pose(transform, ekf.latest_odom[1]) == pytest.approx(corrected)
    assert poses[-1].pose.pose.position.x == pytest.approx(corrected[0])
    report = states[-1]
    assert (report['field_session'], report['field_revision']) == (field.session_id, field.revision)
    assert report['accepted_marker_stamps_s']
    assert report['accepted_marker_stamps_s'] == {label: odom_before[0] for label in ekf.last_used}
    # The field consumes the owner's transform instead of publishing another one.
    actual = field.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
    assert actual.transform.translation.x == pytest.approx(tf.translation.x)


def test_apply_start_service_resets_position_heading_and_rejects_old_images(pipeline):
    field, ekf, client, executor, feed, _poses, _transforms, _states = pipeline
    feed()
    feed(4., 1., 1.2)
    ekf._on_observations(observations(field, ekf, (-.3, .2, .4)))
    old_observation = observations(field, ekf, (-.3, .2, .4))
    revision = field.revision
    service = client.create_client(Trigger, '/football_field/apply_start')
    assert service.wait_for_service(timeout_sec=2)
    future = service.call_async(Trigger.Request())
    spin(executor, future.done, seconds=3)
    assert future.done() and future.result().success
    spin(executor, lambda: ekf.field_revision > revision, seconds=3)
    assert field.revision > revision
    assert not ekf.accepted_marker_stamps
    assert ekf.ekf.state == pytest.approx([-3.2, 0., 0.])
    ekf._on_observations(old_observation)
    assert ekf.ekf.state == pytest.approx([-3.2, 0., 0.])
    feed(4.1, 1., 1.2)
    state, _ = ekf._current()
    assert math.dist(state[:2], (-3.2, 0.)) == pytest.approx(.1)
    assert state[2] == pytest.approx(0.)


def test_fused_field_pose_is_not_fed_back_as_a_marker_or_start_measurement(pipeline):
    field, ekf, _client, _executor, feed, _poses, _transforms, _states = pipeline
    feed()
    pose, covariance = ekf.ekf.state.copy(), ekf.ekf.P.copy()
    data = dict(ekf.field, robot=[100., 100., 2.], transform=[200., 200., 2.])
    ekf._on_field(String(data=json.dumps(data)))
    assert ekf.ekf.state == pytest.approx(pose)
    assert ekf.ekf.P == pytest.approx(covariance)
    feed()
    # odom_pose deliberately lies at (99,99); only the raw observations are fused.
    ekf._on_observations(observations(field, ekf, pose))
    assert ekf.ekf.state == pytest.approx(pose)


def test_stale_odometry_stops_pose_and_tf_and_unmatched_images_do_not_update(pipeline):
    field, ekf, _client, executor, feed, poses, transforms, states = pipeline
    feed()
    feed()
    ekf.odom.samples.clear()
    before = ekf.ekf.state.copy()
    ekf._on_observations(observations(field, ekf, (-2.9, .2, .1)))
    assert ekf.ekf.state == pytest.approx(before)
    assert 'matched' in ekf.last_input_error
    spin(executor)
    counts = len(poses), len(transforms)
    ekf.last_odom_received -= 5
    ekf._publish()
    spin(executor)
    assert (len(poses), len(transforms)) == counts
    assert states[-1]['mode'] == 'odometry_stale'
    assert not ekf._on_reset(None, Trigger.Response()).success


@pytest.mark.parametrize('pipeline', ['conflict'], indirect=True)
def test_owner_waits_when_field_still_owns_tf(pipeline):
    field, ekf, _client, _executor, feed, _poses, _transforms, _states = pipeline
    feed()
    assert field.config['publish_tf']
    assert not ekf._tf_allowed()
    ekf._on_observations(observations(field, ekf, (-3.2, 0., 0.)))
    assert not ekf.ekf.initialised
    assert 'delegate' in ekf.last_input_error


@pytest.mark.parametrize('pipeline', ['pose_only'], indirect=True)
def test_pose_only_mode_does_not_publish_tf(pipeline):
    _field, ekf, _client, executor, feed, poses, transforms, _states = pipeline
    feed()
    assert ekf.broadcaster is None
    ekf._publish()
    spin(executor, lambda: bool(poses), seconds=2)
    assert poses and not transforms
