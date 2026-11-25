"""Preview mapper transport, TF, invalidation and reset on isolated domain 90."""

from copy import deepcopy
import json
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from l2s3_curling.obstacle_map_node import ObstacleMapNode


@pytest.fixture
def pipeline(tmp_path):
    config = tmp_path/'map.yaml'
    settings = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/obstacle_map.yaml').read_text())
    settings.update(max_input_age_s=2., max_pose_age_s=2.)
    config.write_text(yaml.safe_dump(settings))
    world = tmp_path/'markers.yaml'
    world.write_text('field: {half_length: 4.5, half_width: 3.0}\nmarkers:\n  tag:0: [-2, 1]\n')
    footprint = tmp_path/'footprint.yaml'
    footprint.write_text('obstacle_avoidance_adv:\n  ros__parameters: {half_length: 0.125, half_width: 0.11, side_margin: 0.06}\n')
    rclpy.init(args=[], domain_id=90)
    from rclpy.parameter import Parameter
    node = ObstacleMapNode(parameter_overrides=[Parameter('config_file', value=str(config)),
        Parameter('marker_map_file', value=str(world)), Parameter('footprint_file', value=str(footprint))])
    client = Node('mapping_test_observer')
    executor = SingleThreadedExecutor()
    for item in (node, client):
        executor.add_node(item)
    grids, states = [], []
    client.create_subscription(OccupancyGrid, '/obstacle_map/grid', grids.append, 10)
    client.create_subscription(String, '/obstacle_map/state', lambda m: states.append(json.loads(m.data)), 10)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'camera'
    tf.transform.translation.z = .2
    # optical x -> -base y, optical y -> -base z, optical z -> base x
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = -.5, .5, -.5, .5
    node.buffer.set_transform_static(tf, 'synthetic_test')
    field = dict(session_id='test', revision=1, pose_source='marker_ekf', publish_tf=False)

    def position(offset=-1.):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x = offset
        pose.pose.pose.orientation.w = 1.
        for index in (0, 7, 35):
            pose.pose.covariance[index] = .01
        node.on_pose(pose)
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = offset
        transform.transform.rotation.w = 1.
        node.buffer.set_transform(transform, 'synthetic_test')
        node.on_field(String(data=json.dumps(field)))

    def payload():
        stamp = node.get_clock().now().to_msg()
        nav = dict(schema_version=1, usable=True, reason='', frame_id='camera',
                   stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), age_s=0., pair_skew_s=0.,
                   free_points=[[-.25, .2, 1.]], occupied_points=[[0., 0., 1.5]],
                   blocker_faces=[dict(points=[[-.1, .2, 1.5], [.1, .2, 1.5]],
                                       width_std_m=.02, range_std_m=.01, target_size=False)])
        return dict(schema_version=1, fresh=True, source='depth', navigation=nav)

    def render(data=None):
        position()
        if data is not None:
            node.on_detection(String(data=json.dumps(data)))
        node.tick()
        until = time.monotonic()+.15
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        return states[-1] if states else None

    render()
    yield node, executor, grids, states, position, payload, render, field, config
    executor.shutdown()
    client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def confirm(payload, render, template=None):
    for _ in range(4):
        data = payload()
        if template is not None:
            data['navigation']['occupied_points'] = template['navigation']['occupied_points']
        state = render(data)
    return state


def value(node, grid, x, y):
    ix = int((x-node.model.origin[0])/node.model.settings.resolution)
    iy = int((y-node.model.origin[1])/node.model.settings.resolution)
    return grid.data[iy*grid.info.width+ix]


def test_output_is_in_field_frame_and_uses_all_obstacles_without_lidar_or_motors(pipeline):
    node, _executor, grids, _states, _position, payload, render, _field, _config = pipeline
    assert render(payload())['ready']
    assert value(node, grids[-1], .525, .025) == -1
    state = confirm(payload, render)
    assert state['ready'], state
    assert grids[-1].header.frame_id == 'map'
    assert value(node, grids[-1], .525, .025) == 100
    assert value(node, grids[-1], .025, .275) == 0
    assert value(node, grids[-1], -2., 1.) == 100
    assert value(node, grids[-1], 3., 2.) == -1
    topics = dict(node.get_topic_names_and_types())
    assert '/cmd_vel' not in topics and '/scan' not in topics
    assert state['dimensions_provisional']


def test_detector_faces_are_mapped_but_unassociated_depth_speckles_are_unknown(pipeline):
    node, _executor, grids, _states, _position, payload, render, _field, _config = pipeline
    data = payload()
    # This isolated hit is away from the measured face and must not become an obstacle.
    data['navigation']['occupied_points'].append([-.75, 0., 2.])
    assert confirm(payload, render, data)['ready']
    assert value(node, grids[-1], .525, .025) == 100
    assert value(node, grids[-1], 1.025, .775) == -1
    assert value(node, grids[-1], .025, .275) == 0
    assert value(node, grids[-1], -2., 1.) == 100


def test_stale_or_unmeasured_input_invalidates_free_space(pipeline):
    node, _executor, grids, _states, _position, payload, render, _field, _config = pipeline
    assert confirm(payload, render)['ready']
    data = payload()
    data['navigation'].update(usable=False, reason='No measured depth support')
    state = render(data)
    assert not state['ready']
    assert value(node, grids[-1], .025, .275) == -1
    assert value(node, grids[-1], .525, .025) == 100
    data = payload()
    data['navigation']['stamp']['sec'] -= 10
    assert not render(data)['ready']
    assert 'stale' in node.reason


def test_field_reset_clears_old_evidence_and_old_buffered_images_cannot_restore_it(pipeline):
    node, _executor, grids, _states, position, payload, render, field, _config = pipeline
    assert confirm(payload, render)['ready']
    data = payload()
    field['revision'] += 1
    position()
    assert not node.model.cells
    state = render(data)
    assert not state['ready']
    assert value(node, grids[-1], .525, .025) == -1
    assert value(node, grids[-1], -2., 1.) == 100


def test_bad_configuration_keeps_last_valid_model_and_reports_unusable(pipeline):
    node, _executor, _grids, _states, _position, payload, render, _field, config = pipeline
    assert render(payload())['ready']
    model = node.model
    config.write_text('broken: [')
    node.reload()
    assert node.model is model and node.config_error
    assert not render()['ready']
