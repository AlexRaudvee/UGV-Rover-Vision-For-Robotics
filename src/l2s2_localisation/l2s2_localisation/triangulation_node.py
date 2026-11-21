"""Publish a separate marker-based position for comparison, without TF or fusion."""
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml
import numpy as np

from .live_triangulation import solve, validate_config
from .triangulation import Field, Landmark


def load_config(path):
    settings = yaml.safe_load(Path(path).read_text())
    if not isinstance(settings, dict):
        raise ValueError('triangulation configuration must be a mapping')
    config = validate_config(settings)
    if not isinstance(config.get('observations_topic'), str) or not config['observations_topic']:
        raise ValueError('observations_topic must be a topic name')
    return config


class MarkerTriangulation(Node):
    def __init__(self, **kwargs):
        super().__init__('marker_triangulation', **kwargs)
        default = Path(get_package_share_directory('l2s2_localisation')) / 'config/triangulation.yaml'
        self.declare_parameter('config_file', str(default))
        self.path = Path(self.get_parameter('config_file').value).expanduser()
        self.config = load_config(self.path)
        self.file_stamp = self.path.stat().st_mtime_ns
        self.config_error = ''
        self.last_stamp = self.received = None
        self.result = {'fresh': False, 'error': 'waiting for two or more marker measurements',
                       'frame_id': 'map', 'position_m': None}
        self.pub = self.create_publisher(String, '~/result', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '~/pose', 1)
        self.create_subscription(String, self.config['observations_topic'], self.on_observations, 1)
        self.create_timer(.1, self.publish)
        self.create_timer(1., self.reload_config)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_observations(self, message):
        try:
            data = json.loads(message.data)
            stamp = float(data['stamp'])
            if not math.isfinite(stamp):
                raise ValueError('invalid image timestamp')
            now = self.now()
            if self.last_stamp is not None and now < self.last_stamp-.1:
                self.last_stamp = None  # ROS time restarted, not an old buffered image.
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return
            self.last_stamp, self.received = stamp, time.monotonic()
            self.result = {key: data.get(key) for key in ('frame_id', 'field_revision', 'map_revision')}
            self.result.update(fresh=False, stamp=stamp, position_m=None,
                               odom_position_m=None, position_delta_m=None)
            if not isinstance(data.get('frame_id'), str) or not data['frame_id']:
                raise ValueError('invalid field frame')
            if not -.1 <= now-stamp <= self.config['max_image_age_s']:
                raise ValueError('marker image is stale or in the future')
            if data.get('error'):
                raise ValueError(data['error'])
            dimensions = data['field']
            if any(type(dimensions[k]) not in (int, float) or not math.isfinite(dimensions[k]) or
                   dimensions[k] < 0 or (k != 'border' and dimensions[k] == 0)
                   for k in ('half_length', 'half_width', 'border')):
                raise ValueError('invalid field dimensions')
            markers = [Landmark(item['label'], tuple(item['known_position_m'][:2]),
                                float(item['range_m']), math.radians(float(item['bearing_deg']))
                                if item.get('bearing_deg') is not None else None, item.get('side_px'))
                       for item in data['observations']]
            fix = solve(markers, Field(**dimensions), self.config)
            self.result.update(fix, fresh=True, error='')
            # Comparison only: odometry is never passed into solve().
            odom = data.get('odom_pose')
            if isinstance(odom, list) and len(odom) == 3 and all(math.isfinite(v) for v in odom):
                self.result.update(odom_position_m=[odom[0], odom[1], 0.],
                                   odom_heading_deg=math.degrees(odom[2]),
                                   position_delta_m=math.dist(fix['position_m'][:2], odom[:2]))
            if not self.config_error:
                self.publish_pose(stamp, fix)
        except (ValueError, KeyError, TypeError, ArithmeticError, np.linalg.LinAlgError) as error:
            self.result.update(fresh=False, position_m=None, error=str(error))
        self.publish()

    def publish_pose(self, stamp, fix):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.result['frame_id']
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
        msg.pose.pose.position.x, msg.pose.pose.position.y = fix['position_m'][:2]
        heading = math.radians(fix['heading_deg'] or 0.)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        covariance = [0.]*36
        xy = fix['covariance_xy']
        covariance[0], covariance[1], covariance[6], covariance[7] = xy[0][0], xy[0][1], xy[1][0], xy[1][1]
        covariance[14] = covariance[21] = covariance[28] = 1e6
        covariance[35] = math.pi**2 if fix['heading_deg'] is None else max(
            math.radians(fix['rms_bearing_error_deg'] or 0.)**2, math.radians(5)**2)
        msg.pose.covariance = covariance
        self.pose_pub.publish(msg)

    def publish(self):
        result = dict(self.result)
        age = max(0., self.now()-result['stamp']) if 'stamp' in result else None
        alive = self.received is not None and time.monotonic()-self.received <= self.config['stale_after_s']
        result.update(age_s=age, stale_after_s=self.config['stale_after_s'], config_file=str(self.path),
                      fresh=bool(result.get('fresh') and alive and age is not None and
                                 age <= self.config['stale_after_s'] and not self.config_error))
        if self.config_error:
            result['error'] = self.config_error
        elif not result['fresh'] and not result.get('error'):
            result['error'] = 'triangulation measurement is stale'
        self.pub.publish(String(data=json.dumps(result, allow_nan=False)))

    def reload_config(self):
        try:
            stamp = self.path.stat().st_mtime_ns
            if stamp == self.file_stamp:
                return
            config = load_config(self.path)
            if config['observations_topic'] != self.config['observations_topic']:
                raise ValueError('observation topic changes need a node restart')
            self.config, self.file_stamp, self.config_error = config, stamp, ''
            self.result.update(fresh=False, position_m=None, error='configuration changed; waiting for a new image')
        except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
            self.config_error = f'Triangulation config not loaded: {error}'


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTriangulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
