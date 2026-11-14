"""Field display and explicit start-pose alignment; never sends robot commands."""

import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .field_model import (DEFAULTS, START_KEYS, alignment, field_layout, load_config,
                          map_shape, occupancy, transform_pose, validate)


class FootballField(Node):
    def __init__(self):
        super().__init__('football_field')
        default_path = str(Path(get_package_share_directory('ugv_bringup')) /
                           'param' / 'football_field.yaml')
        self.declare_parameter('config_file', default_path)
        self.config_path = Path(self.get_parameter('config_file').value).expanduser()
        self.config = load_config(self.config_path)
        for name, value in self.config.items():
            self.declare_parameter(name, value)
        self.config = {key: self.get_parameter(key).value for key in DEFAULTS}
        validate(self.config)
        self.latest = self.anchor = None
        self.last_received = None
        self.last_stamp = None
        self.revision = 0
        self.error = ''
        self.loading_file = False
        self.file_stamp = self.config_path.stat().st_mtime_ns
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.layout_pub = self.create_publisher(String, '~/layout', latched)
        self.state_pub = self.create_publisher(String, '~/state', latched)
        self.map_pub = self.create_publisher(OccupancyGrid, '/field_map', latched)
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) if not self.config['publish_tf'] else None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_service(Trigger, '~/apply_start', self.apply_start)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(.1, self.tick)
        self.create_timer(1.0, self.reload_start)
        # Also support late browser subscribers whose bridge uses volatile QoS.
        self.create_timer(2.0, lambda: self.layout_pub.publish(String(data=self.layout_json)))
        self.publish_layout()
        self.get_logger().info(f'Football field: edit start_* in {self.config_path} or in Vizanti')

    def fresh(self):
        return self.last_received is not None and (
            time.monotonic() - self.last_received <= self.config['odom_timeout_s'])

    def on_odom(self, message):
        if message.header.frame_id != self.config['odom_frame']:
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(sum(v*v for v in values[2:]))
        if norm < 1e-6:
            return
        qx, qy, qz, qw = (v / norm for v in values[2:])
        yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.anchor = None
            self.revision += 1
        self.latest, self.last_received, self.last_stamp = (p.x, p.y, yaw), time.monotonic(), stamp
        if self.anchor is None and self.config['initialize_on_first_odom']:
            self.anchor = self.latest
            self.revision += 1

    def persist_start(self, config):
        # Preserve comments and unrelated settings in the user's editable YAML.
        original_stat = self.config_path.stat()
        text = self.config_path.read_text()
        for key in START_KEYS:
            pattern = rf'(?m)^(\s*{key}:\s*)[^#\n]*(.*)$'
            text, count = re.subn(pattern, lambda m: f'{m[1]}{float(config[key])} {m[2]}'.rstrip(), text)
            if count != 1:
                raise ValueError(f'Expected exactly one {key} in {self.config_path}')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', dir=self.config_path.parent,
                                             prefix='.football_field_', delete=False) as stream:
                temporary = stream.name
                stream.write(text)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, original_stat.st_mode & 0o777)
            # The ROS container runs as root; keep the host user's file editable.
            if os.geteuid() == 0:
                os.chown(temporary, original_stat.st_uid, original_stat.st_gid)
            os.replace(temporary, self.config_path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.file_stamp = self.config_path.stat().st_mtime_ns

    def on_parameters(self, parameters):
        changes = {p.name: p.value for p in parameters if p.name in START_KEYS}
        unsupported = [p.name for p in parameters if p.name not in (*START_KEYS, 'use_sim_time')]
        if unsupported:
            return SetParametersResult(successful=False, reason='Only start_* can change live; restart for other settings')
        if not changes:
            return SetParametersResult(successful=True)
        if not self.config['publish_tf']:
            return SetParametersResult(successful=False, reason='External localization owns the field pose; edit the file for the next standalone run')
        candidate = {**self.config, **changes}
        try:
            validate(candidate)
            if not self.loading_file:
                self.persist_start(candidate)
        except (OSError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.config = candidate
        self.anchor = self.latest if self.fresh() else None
        self.revision += 1
        self.error = ''
        return SetParametersResult(successful=True)

    def apply_start(self, _request, response):
        if not self.config['publish_tf']:
            response.success, response.message = False, 'External localization owns the field pose'
        elif not self.fresh():
            response.success, response.message = False, 'No fresh /odom; place the robot and start odometry first'
        else:
            self.anchor = self.latest
            self.revision += 1
            response.success, response.message = True, 'Current robot pose aligned with the saved start; no motion commanded'
        return response

    def reload_start(self):
        try:
            stamp = self.config_path.stat().st_mtime_ns
            if stamp == self.file_stamp:
                return
            candidate = load_config(self.config_path)
            changed = [key for key in START_KEYS if candidate[key] != self.config[key]]
            self.loading_file = True
            if changed:
                result = self.set_parameters_atomically([Parameter(key, value=float(candidate[key]))
                                                         for key in changed])
                if not result.successful:
                    raise ValueError(result.reason)
            self.file_stamp = stamp
            self.error = ''
            if any(candidate[k] != self.config[k] for k in DEFAULTS if k not in START_KEYS):
                self.error = 'Field geometry or frame settings changed: restart field server to load them'
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.error = f'Config not loaded: {error}'
        finally:
            self.loading_file = False

    def publish_layout(self):
        self.layout_json = json.dumps(field_layout(self.config))
        self.layout_pub.publish(String(data=self.layout_json))
        width, height, origin = map_shape(self.config)
        message = OccupancyGrid()
        message.header.frame_id = self.config['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self.config['map_resolution_m']
        message.info.width, message.info.height = width, height
        message.info.origin.position.x, message.info.origin.position.y = origin
        message.info.origin.orientation.w = 1.0
        message.data = occupancy(self.config)
        self.map_pub.publish(message)

    def tick(self):
        start = tuple(self.config[k] for k in START_KEYS)
        transform = alignment(start, self.anchor) if self.anchor is not None else None
        if not self.config['publish_tf']:
            transform = None
            try:
                external = self.tf_buffer.lookup_transform(
                    self.config['map_frame'], self.config['odom_frame'], Time())
                p, q = external.transform.translation, external.transform.rotation
                transform = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            except TransformException:
                pass
        if transform and self.config['publish_tf']:
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.config['map_frame']
            message.child_frame_id = self.config['odom_frame']
            message.transform.translation.x, message.transform.translation.y = transform[:2]
            message.transform.rotation.z = math.sin(transform[2] / 2)
            message.transform.rotation.w = math.cos(transform[2] / 2)
            self.broadcaster.sendTransform(message)
        robot = transform_pose(transform, self.latest) if transform and self.latest else None
        state = {'frame_id': self.config['map_frame'], 'odom_frame': self.config['odom_frame'],
                 'config_file': str(self.config_path), 'start': dict(zip(START_KEYS, start)),
                 'aligned': transform is not None, 'odom_fresh': self.fresh(),
                 'transform': transform, 'robot': robot, 'revision': self.revision,
                 'publish_tf': self.config['publish_tf'], 'error': self.error}
        self.state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = FootballField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
