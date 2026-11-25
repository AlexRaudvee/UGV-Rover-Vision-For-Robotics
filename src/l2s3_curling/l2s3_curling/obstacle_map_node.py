"""Field preview map from improved depth only. Never publishes robot commands."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from detection_improved.detector import transform_arrays
from .obstacle_map import GridSettings, ObstacleMap, Stand


def load_model(config_path, marker_path, footprint_path):
    config = yaml.safe_load(Path(config_path).read_text())
    world = yaml.safe_load(Path(marker_path).read_text())
    footprint = yaml.safe_load(Path(footprint_path).read_text())['obstacle_avoidance_adv']['ros__parameters']
    field = world['field']
    settings = GridSettings(config['resolution_m'], field['half_length'], field['half_width'],
                            config['obstacle_memory_s'], config['free_memory_s'],
                            footprint['half_length'], footprint['half_width'], footprint['side_margin'])
    for name in ('max_input_age_s', 'max_pose_age_s', 'max_position_std_m', 'max_heading_std_deg'):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(name + ' must be positive and finite')
    if config['map_frame'] == config['odom_frame']:
        raise ValueError('Map and odometry frames must differ')
    stands = []
    for label, point in world['markers'].items():
        if label in config['excluded_stands']:
            continue
        width, depth = config['stand_size_overrides'].get(label, config['default_stand_size_m'])
        stands.append(Stand(label, float(point[0]), float(point[1]), float(width), float(depth),
                            math.radians(config['stand_yaw_deg'].get(label, 0.)),
                            label in config['measured_stands']))
    return config, ObstacleMap(settings, stands)


class ObstacleMapNode(Node):
    def __init__(self, **kwargs):
        super().__init__('obstacle_map', **kwargs)
        self.paths = []
        for parameter, package, relative in (
                ('config_file', 'l2s3_curling', 'config/obstacle_map_preview.yaml'),
                ('marker_map_file', 'l2s2_localisation', 'config/marker_map.yaml'),
                ('footprint_file', 'obstacle_avoidance_adv', 'config/controller.yaml')):
            default = str(Path(get_package_share_directory(package))/relative)
            self.paths.append(Path(self.declare_parameter(parameter, default).value))
        self.config, self.model = load_model(*self.paths)
        self.file_stamps = self._file_stamps()
        self.config_error = ''
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pose = self.pose_received = self.pending = None
        self.last_stamp = None
        self.last_input_received = None
        self.reason = 'Waiting for improved depth evidence'
        self.field_key = None
        self.field_owned = False
        self.field_received = None
        self.minimum_stamp = 0.
        self.last_timing = {}
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, '~/grid', retained)
        self.detected_pub = self.create_publisher(OccupancyGrid, '~/detected', retained)
        self.inflated_pub = self.create_publisher(OccupancyGrid, '~/inflated', retained)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', retained)
        self.state_pub = self.create_publisher(String, '~/state', retained)
        self.create_subscription(String, self.config['detector_topic'], self.on_detection, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/marker_ekf/pose', self.on_pose, 1)
        self.create_subscription(String, '/football_field/state', self.on_field, retained)
        self.create_timer(.2, self.tick)
        self.create_timer(1., self.reload)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def _file_stamps(self):
        return tuple(path.stat().st_mtime_ns for path in self.paths)

    def reload(self):
        try:
            stamps = self._file_stamps()
            if stamps == self.file_stamps:
                return
            config, model = load_model(*self.paths)
            if any(config[k] != self.config[k] for k in ('detector_topic', 'map_frame', 'odom_frame')):
                raise ValueError('Topic/frame changes require restarting obstacle_map')
            self.config, self.model, self.file_stamps = config, model, stamps
            self.pending = self.last_stamp = self.last_input_received = None
            self.config_error = ''
            self.reason = 'Configuration changed; waiting for new depth'
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.config_error = str(error)

    def on_pose(self, message):
        self.pose, self.pose_received = message, time.monotonic()

    def on_field(self, message):
        try:
            data = json.loads(message.data)
            key = data.get('session_id'), data['revision']
            if self.field_key is not None and key != self.field_key:
                self.model.clear()
                self.pending = self.last_stamp = self.last_input_received = None
                self.minimum_stamp = self.now()
                self.reason = 'Field start changed; old sensor map cleared'
            self.field_key = key
            self.field_owned = data.get('pose_source') == 'marker_ekf' and data.get('publish_tf') is False
            self.field_received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def on_detection(self, message):
        try:
            data = json.loads(message.data)
            nav = data.get('navigation') or {}
            if (data.get('schema_version') != 1 or data.get('source') != 'depth'
                    or not data.get('fresh') or nav.get('schema_version') != 1
                    or not nav.get('usable')):
                raise ValueError(nav.get('reason') or data.get('status') or 'No usable improved depth evidence')
            self.pending = nav
        except (ValueError, KeyError, TypeError) as error:
            self.pending = None
            self.last_input_received = None
            self.reason = str(error)

    def pose_ready(self):
        if self.pose is None or self.pose_received is None:
            return False
        stamp = self.pose.header.stamp.sec+self.pose.header.stamp.nanosec/1e9
        p = self.pose.pose.pose.position
        cov = self.pose.pose.covariance
        return (self.pose.header.frame_id == self.config['map_frame']
                and all(math.isfinite(v) for v in (p.x, p.y, cov[0], cov[7], cov[35]))
                and min(cov[0], cov[7], cov[35]) > 0
                and max(cov[0], cov[7]) <= self.config['max_position_std_m']**2
                and cov[35] <= math.radians(self.config['max_heading_std_deg'])**2
                and -.1 <= self.now()-stamp <= self.config['max_pose_age_s']
                and time.monotonic()-self.pose_received <= self.config['max_pose_age_s'])

    def ingest(self):
        nav = self.pending
        if nav is None:
            return
        stamp = nav['stamp']['sec']+nav['stamp']['nanosec']/1e9
        if not self.minimum_stamp <= stamp or not -.1 <= self.now()-stamp <= self.config['max_input_age_s']:
            raise ValueError('Depth evidence is stale, future or predates the field reset')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        header_time = Time(seconds=int(nav['stamp']['sec']), nanoseconds=int(nav['stamp']['nanosec']))
        tf = self.buffer.lookup_transform(self.config['odom_frame'], nav['frame_id'], header_time)
        rotation, translation = transform_arrays(tf.transform)

        def projected(points):
            points = np.asarray(points, float).reshape(-1, 3)
            if not np.isfinite(points).all() or len(points) > 100_000:
                raise ValueError('Invalid depth point evidence')
            return (points @ rotation.T+translation)[:, :2]

        free = projected(nav['free_points'])
        faces = []
        for face in nav['blocker_faces']:
            points = projected(face['points'])
            if len(points) != 2:
                raise ValueError('Invalid blocker face')
            uncertainty = 2*max(float(face['range_std_m']), float(face['width_std_m']))
            if not math.isfinite(uncertainty) or uncertainty < 0 or uncertainty > 2:
                raise ValueError('Invalid blocker uncertainty')
            faces.append((points, uncertainty))
        # Assume a fixed 30 cm square behind each measured detector front.
        # The front is modelled camera-parallel; box rotation is not measured.
        self.model.update(free, [], stamp)
        self.model.update_faces(faces, translation[:2], stamp)
        self.last_stamp, self.last_input_received = stamp, time.monotonic()
        self.last_timing = {k: nav.get(k) for k in ('age_s', 'pair_skew_s', 'supported_pixels', 'unknown_pixels')}
        self.reason = ''

    def publish_grid(self, publisher, data):
        message = OccupancyGrid()
        message.header.frame_id = self.config['map_frame']
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = self.model.settings.resolution
        message.info.width, message.info.height = self.model.width, self.model.height
        message.info.origin.position.x, message.info.origin.position.y = self.model.origin
        message.info.origin.orientation.w = 1.
        message.data = data.ravel().tolist()
        publisher.publish(message)

    def tick(self):
        transform = None
        try:
            if (not self.field_owned or self.field_received is None
                    or time.monotonic()-self.field_received > 1.5):
                raise ValueError('Waiting for EKF-owned field alignment')
            if not self.pose_ready():
                raise ValueError('Waiting for a fresh EKF pose with usable uncertainty')
            tf = self.buffer.lookup_transform(self.config['map_frame'], self.config['odom_frame'], Time())
            age = self.now()-tf.header.stamp.sec-tf.header.stamp.nanosec/1e9
            if not -.1 <= age <= self.config['max_pose_age_s']:
                raise ValueError('Map/odometry transform is stale')
            r, t = transform_arrays(tf.transform)
            transform = (float(t[0]), float(t[1]), math.atan2(r[1, 0], r[0, 0]))
            self.ingest()
        except (TransformException, ValueError, KeyError, TypeError) as error:
            self.reason = str(error)
        ready = bool(not self.reason and not self.config_error and transform is not None
                     and self.last_stamp is not None
                     and -.1 <= self.now()-self.last_stamp <= self.config['max_input_age_s']
                     and self.last_input_received is not None
                     and time.monotonic()-self.last_input_received <= self.config['max_input_age_s'])
        grid, inflated = self.model.render(transform, self.now(), allow_free=ready)
        self.publish_grid(self.grid_pub, grid)
        detected, _ = self.model.render(transform, self.now(), allow_free=False, include_static=False)
        self.publish_grid(self.detected_pub, detected)
        self.publish_grid(self.inflated_pub, inflated)
        self.publish_markers()
        state = dict(ready=ready, preview_only=True, source='improved_depth_only',
                     reason=self.config_error or self.reason or ('' if ready else 'Depth evidence expired'),
                     free_cells=int((grid == 0).sum()), occupied_cells=int((grid == 100).sum()),
                     unknown_cells=int((grid < 0).sum()), footprint=asdict(self.model.settings),
                     inflation_radius_m=self.model.settings.radius,
                     depth_stamp_s=self.last_stamp,
                     known_stands=[asdict(stand) for stand in self.model.stands], timing=self.last_timing,
                     dimensions_provisional=any(not stand.measured for stand in self.model.stands))
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def publish_markers(self):
        markers = [Marker(action=Marker.DELETEALL)]
        for index, stand in enumerate(self.model.stands):
            marker = Marker()
            marker.header.frame_id = self.config['map_frame']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'known_stands', index*2, Marker.CUBE
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = stand.x, stand.y, .025
            marker.pose.orientation.z, marker.pose.orientation.w = math.sin(stand.yaw/2), math.cos(stand.yaw/2)
            marker.scale.x, marker.scale.y, marker.scale.z = stand.depth, stand.width, .05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .55, .15, .65
            markers.append(marker)
            label = Marker()
            label.header = marker.header
            label.ns, label.id, label.type = marker.ns, index*2+1, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = stand.x, stand.y, .1
            label.pose.orientation.w = 1.
            label.scale.z = .12
            label.color = marker.color
            label.text = f'Stand {stand.label.rsplit(":", 1)[-1]}' + ('' if stand.measured else ' (provisional)')
            markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
