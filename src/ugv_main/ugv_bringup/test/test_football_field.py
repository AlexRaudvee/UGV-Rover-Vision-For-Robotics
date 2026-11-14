"""Isolated ROS-domain tests. No robot drivers, camera or command publishers."""

import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from ugv_bringup.field_model import load_config
from ugv_bringup.football_field import FootballField


@pytest.fixture
def server(tmp_path, request):
    original = Path(__file__).resolve().parents[1] / 'param/football_field.yaml'
    config_file = tmp_path / 'football_field.yaml'
    config_file.write_text(original.read_text())
    if getattr(request, 'param', None) == 'external':
        config_file.write_text(config_file.read_text().replace('publish_tf: true', 'publish_tf: false'))
    rclpy.init(args=['--ros-args', '-p', f'config_file:={config_file}'], domain_id=96)
    node = FootballField()
    client = Node('field_review_client', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client)
    yield node, client, executor, config_file
    executor.shutdown()
    client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def odom(x=1.2, y=-.4, yaw=.7, stamp=10):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = stamp
    message.child_frame_id = 'base_footprint'
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.y = float(y)
    message.pose.pose.orientation.z = math.sin(yaw/2)
    message.pose.pose.orientation.w = math.cos(yaw/2)
    return message


def spin_until(executor, condition, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not condition():
        executor.spin_once(timeout_sec=.02)
    assert condition(), 'Timed out waiting for isolated ROS response'


def test_ros_service_saves_and_aligns_arbitrary_current_odometry(server):
    node, client, executor, config_file = server
    states = []
    subscription = client.create_subscription(String, '/football_field/state',
        lambda msg: states.append(json.loads(msg.data)), 10)
    publisher = client.create_publisher(Odometry, '/odom', 10)
    spin_until(executor, lambda: publisher.get_subscription_count() > 0)
    publisher.publish(odom())
    spin_until(executor, lambda: node.anchor is not None)
    service = client.create_client(SetParametersAtomically, '/football_field/set_parameters_atomically')
    assert service.wait_for_service(timeout_sec=3)
    request = SetParametersAtomically.Request(parameters=[
        Parameter('start_x_m', value=3.2).to_parameter_msg(),
        Parameter('start_y_m', value=.5).to_parameter_msg(),
        Parameter('start_heading_deg', value=180.0).to_parameter_msg()])
    future = service.call_async(request)
    spin_until(executor, future.done)
    assert future.result().result.successful
    spin_until(executor, lambda: states and states[-1]['start']['start_x_m'] == 3.2)
    assert states[-1]['robot'][:2] == pytest.approx([3.2, .5])
    assert abs(states[-1]['robot'][2]) == pytest.approx(math.pi)
    assert load_config(config_file)['start_x_m'] == 3.2
    assert '# Editable source' in config_file.read_text()
    topics = dict(node.get_topic_names_and_types())
    assert '/cmd_vel' not in topics and '/ugv/joint_states' not in topics
    client.destroy_subscription(subscription)


def test_file_edit_reanchors_and_partial_yaml_keeps_previous_fix(server):
    node, _client, _executor, config_file = server
    node.on_odom(odom())
    config_file.write_text(config_file.read_text().replace('start_x_m: -3.2', 'start_x_m: -2.5'))
    node.reload_start()
    assert node.config['start_x_m'] == -2.5
    assert node.anchor == node.latest
    previous = node.anchor
    config_file.write_text('football_field: [unfinished')
    node.reload_start()
    assert node.error.startswith('Config not loaded:')
    assert node.anchor == previous and node.config['start_x_m'] == -2.5


def test_stale_odometry_and_invalid_quaternion_cannot_be_applied(server):
    node, _client, _executor, _config = server
    node.on_odom(odom())
    node.last_received -= 3
    response = node.apply_start(None, Trigger.Response())
    assert not response.success
    previous = node.latest
    message = odom(x=8)
    message.pose.pose.orientation.z = message.pose.pose.orientation.w = 0.0
    node.on_odom(message)
    assert node.latest == previous


def test_failed_persistence_does_not_change_live_transform(server, monkeypatch):
    node, _client, _executor, _config = server
    node.on_odom(odom())
    def denied(_config):
        raise PermissionError('read-only config')
    monkeypatch.setattr(node, 'persist_start', denied)
    result = node.set_parameters_atomically([Parameter('start_x_m', value=2.0)])
    assert not result.successful
    assert node.config['start_x_m'] == -3.2


@pytest.mark.parametrize('server', ['external'], indirect=True)
def test_external_localization_is_used_without_overwriting_its_pose(server):
    node, client, executor, _config = server
    states = []
    client.create_subscription(String, '/football_field/state',
                               lambda msg: states.append(json.loads(msg.data)), 10)
    broadcaster = TransformBroadcaster(client)
    message = TransformStamped()
    message.header.frame_id = 'map'
    message.child_frame_id = 'odom'
    message.transform.translation.x = 2.0
    message.transform.translation.y = -1.0
    message.transform.rotation.z = math.sin(math.pi/4)
    message.transform.rotation.w = math.cos(math.pi/4)
    def publish():
        message.header.stamp = client.get_clock().now().to_msg()
        broadcaster.sendTransform(message)
    timer = client.create_timer(.05, publish)
    node.on_odom(odom(x=1, y=2, yaw=0))
    spin_until(executor, lambda: states and states[-1]['aligned'])
    assert states[-1]['robot'][:2] == pytest.approx([0, 0], abs=1e-9)
    assert states[-1]['robot'][2] == pytest.approx(math.pi/2)
    result = node.set_parameters_atomically([Parameter('start_x_m', value=5.0)])
    assert not result.successful
    assert not node.apply_start(None, Trigger.Response()).success
    client.destroy_timer(timer)
