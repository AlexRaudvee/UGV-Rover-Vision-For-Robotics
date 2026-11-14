"""Stream AprilTag-measured distances to Vizanti; no robot-pose or motor control."""

from collections import deque
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from l2s1_marker_detection.camera_model import CameraModel
from l2s1_marker_detection.aruco_compat import normalize_family
from .geometry import MountModel
from .ranging import PoseHistory, SpatialFilter, base_to_map, observe_tag, planar_pose


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict):
        raise ValueError('marker range configuration must be a mapping')
    for key in ('marker_size_m', 'image_max_age_s', 'stale_after_s', 'filter_window_s',
                'min_side_px', 'low_resolution_px', 'max_reprojection_error_px',
                'min_decision_margin', 'max_range_m', 'odom_max_skew_s', 'head_settle_s'):
        if isinstance(config[key], bool) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be a positive finite number')
    if not isinstance(config['filter_samples'], int) or not 1 <= config['filter_samples'] <= 99:
        raise ValueError('filter_samples must be between 1 and 99')
    if config['image_geometry'] not in ('raw', 'rectified'):
        raise ValueError('image_geometry must be raw or rectified')
    if not isinstance(config.get('only_mapped_markers', True), bool):
        raise ValueError('only_mapped_markers must be true or false')
    if type(config['max_hamming']) is not int or not 0 <= config['max_hamming'] <= 2:
        raise ValueError('max_hamming must be an integer between 0 and 2')
    for key in ('pan_sign', 'tilt_sign', 'initial_pan_deg', 'initial_tilt_deg',
                'tilt_axis_height', 'camera_offset'):
        if not math.isfinite(config[key]):
            raise ValueError(f'{key} must be finite')
    if abs(config['pan_sign']) != 1 or abs(config['tilt_sign']) != 1:
        raise ValueError('pan_sign and tilt_sign must be -1 or +1')
    if len(config['pan_pivot_xy']) != 2 or not all(math.isfinite(v) for v in config['pan_pivot_xy']):
        raise ValueError('pan_pivot_xy must contain two finite numbers')
    def entries(values, sizes=False):
        output = {}
        if values is not None and not isinstance(values, dict):
            raise ValueError('marker positions/sizes must be a mapping keyed by FAMILY:ID')
        for label, value in (values or {}).items():
            family, _, tag_id = label.rpartition(':')
            label = f'{normalize_family(family)}:{int(tag_id)}'
            if sizes:
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{label} size must be positive')
                output[label] = float(value)
            else:
                if len(value) not in (2, 3) or not all(math.isfinite(v) for v in value):
                    raise ValueError(f'{label} must have finite field x,y[,z] coordinates')
                output[label] = [float(v) for v in value] + ([0.0] if len(value) == 2 else [])
        return output
    marker_map = config.get('marker_map_file')
    if marker_map:
        marker_map = Path(marker_map).expanduser()
        if not marker_map.is_absolute():
            marker_map = Path(path).parent / marker_map
        world = yaml.safe_load(marker_map.read_text()) or {}
        if not isinstance(world, dict):
            raise ValueError('marker map must be a YAML mapping')
        config['markers'] = entries(world.get('markers'))
        config['marker_map_file'] = str(marker_map)
    else:
        config['markers'] = entries(config.get('markers'))
    config['marker_sizes'] = entries(config.get('marker_sizes'), sizes=True)
    return config


class MarkerRanges(Node):
    def __init__(self):
        super().__init__('marker_ranges')
        default = Path(get_package_share_directory('l2s2_localisation')) / 'config/marker_ranges.yaml'
        self.declare_parameter('config_file', str(default))
        self.path = Path(self.get_parameter('config_file').value).expanduser()
        self.config = load_config(self.path)
        self.file_stamp = self.config_stamp()
        camera_file = Path(get_package_share_directory('ugv_vision')) / 'config/camera_info.yaml'
        self.cameras = {geometry: CameraModel.from_yaml(camera_file, rectified=geometry == 'rectified')
                        for geometry in ('raw', 'rectified')}
        self.mount = MountModel(tuple(self.config['pan_pivot_xy']), self.config['tilt_axis_height'],
                                self.config['camera_offset'])
        self.odom = PoseHistory(self.config['odom_max_skew_s'])
        self.commands = deque(maxlen=300)
        self.filters, self.records = {}, {}
        self.field = None
        self.field_received = self.last_received = None
        self.rejections = {}
        self.error = ''
        self.config_error = ''
        self.last_image_stamp = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(String, '~/result', latched)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.create_subscription(String, self.config['detections_topic'], self.on_detections, 10)
        self.create_subscription(CameraInfo, self.config['camera_info_topic'], self.on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/football_field/state', self.on_field, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/ugv/joint_states', self.on_joints, 10)
        self.create_timer(.1, self.publish)
        self.create_timer(1.0, self.reload_config)
        self.get_logger().info(f'AprilTag ranges from {self.config["detections_topic"]}; config {self.path}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def config_stamp(self):
        world = self.config.get('marker_map_file')
        return (self.path.stat().st_mtime_ns, Path(world).stat().st_mtime_ns if world else None)

    def on_camera(self, message):
        try:
            models = {g: CameraModel.from_camera_info(message, rectified=g == 'rectified')
                      for g in ('raw', 'rectified')}
            if any(not np.all(np.isfinite(m.K)) or m.K[0, 0] <= 0 or m.K[1, 1] <= 0
                   or m.width <= 0 or m.height <= 0 for m in models.values()):
                return
            if any(not np.allclose(self.cameras[g].K, m.K) or
                   not np.array_equal(self.cameras[g].D, m.D) for g, m in models.items()):
                self.filters.clear()
                self.records.clear()
            self.cameras = models
        except (ValueError, TypeError):
            return

    def on_field(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or not isinstance(state.get('frame_id'), str):
                return
            if state.get('aligned'):
                transform = state.get('transform')
                if not isinstance(transform, list) or len(transform) != 3 or not all(math.isfinite(v) for v in transform):
                    return
            if self.field and self.field.get('revision') != state.get('revision'):
                self.filters.clear()
                self.records.clear()
            self.field = state
            self.field_received = time.monotonic()
        except (ValueError, TypeError):
            return

    def on_odom(self, message):
        if message.header.frame_id != (self.field or {}).get('odom_frame', 'odom'):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if norm < 1e-6:
            return
        x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.odom.add(stamp, (p.x, p.y, yaw))

    def on_joints(self, message):
        try:
            pan = message.position[message.name.index('pt_base_link_to_pt_link1')] / self.config['pan_sign']
            tilt = message.position[message.name.index('pt_link1_to_pt_link2')] / self.config['tilt_sign']
            if not all(math.isfinite(v) for v in (pan, tilt)):
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            if stamp <= 0:
                stamp = self.now()
            if self.commands and stamp < self.commands[-1][0]:
                self.commands.clear()
            changed = not self.commands or abs(pan-self.commands[-1][1]) + abs(tilt-self.commands[-1][2]) > .01
            moved = stamp if changed else self.commands[-1][3]
            self.commands.append((stamp, pan, tilt, moved))
        except (ValueError, IndexError):
            return

    def head_at(self, stamp):
        for command_stamp, pan, tilt, moved in reversed(self.commands):
            if command_stamp <= stamp:
                if stamp-moved < self.config['head_settle_s']:
                    raise ValueError('camera head is settling after a pan/tilt command')
                return pan, tilt
        return math.radians(self.config['initial_pan_deg']), math.radians(self.config['initial_tilt_deg'])

    def robot_at(self, stamp):
        field = self.field
        if not field or not field.get('aligned') or self.field_received is None or time.monotonic()-self.field_received > 1.5:
            return None
        pose = self.odom.at(stamp)
        return planar_pose(field['transform'], pose) if pose is not None else None

    def on_detections(self, message):
        try:
            data = json.loads(message.data)
            stamp = float(data['stamp'])
            if not math.isfinite(stamp) or not -.1 <= self.now()-stamp <= self.config['image_max_age_s']:
                raise ValueError('detection image timestamp is stale or in the future')
            if self.last_image_stamp is not None and stamp < self.last_image_stamp:
                self.filters.clear()
                self.records.clear()
            if stamp == self.last_image_stamp:
                return
            self.last_image_stamp, self.last_received = stamp, time.monotonic()
            geometry = data.get('image_geometry', self.config['image_geometry'])
            camera = self.cameras[geometry]
            pan, tilt = self.head_at(stamp)
            robot = self.robot_at(stamp)
            self.rejections = {}
            for item in data.get('detections', []):
                label = f'{item["family"]}:{int(item["id"])}'
                try:
                    if self.config.get('only_mapped_markers', True) and label not in self.config['markers']:
                        raise ValueError('tag is not in the configured marker map')
                    size = self.config['marker_sizes'].get(label, self.config['marker_size_m'])
                    observation = observe_tag(item, size, camera, (data['width'], data['height']),
                                               self.mount, pan, tilt, self.config)
                    point = observation['position']
                    frame = 'base'
                    if robot is not None:
                        point = base_to_map(point, robot)
                        frame = f'field:{self.field.get("revision")}'
                    filtering = self.filters.setdefault(label, SpatialFilter(
                        self.config['filter_samples'], self.config['filter_window_s']))
                    filtered, points = filtering.add(stamp, point, frame)
                    if robot is not None:
                        delta = filtered[:2]-np.array(robot[:2])
                        radial = np.linalg.norm(points[:, :2]-np.array(robot[:2]), axis=1)
                        bearing = math.atan2(delta[1], delta[0])-robot[2]
                    else:
                        delta = filtered[:2]
                        radial = np.linalg.norm(points[:, :2], axis=1)
                        bearing = math.atan2(delta[1], delta[0])
                    self.records[label] = {
                        'stamp': stamp, 'range_m': float(np.linalg.norm(delta)),
                        'range_std_m': float(np.std(radial)),
                        'bearing_deg': math.degrees((bearing+math.pi) % math.tau-math.pi),
                        'camera_distance_m': observation['camera_distance_m'],
                        'reprojection_error_px': observation['reprojection_error_px'],
                        'side_px': observation['side_px'], 'samples': len(points),
                        'low_resolution': observation['side_px'] < self.config['low_resolution_px'],
                        'observed_position_m': filtered.tolist() if robot is not None else None,
                        'robot_position_m': [robot[0], robot[1], 0.0] if robot is not None else None,
                        'marker_size_m': size,
                        'quality_available': 'decision_margin' in item,
                    }
                except (ValueError, KeyError, TypeError, ImportError, cv2.error) as error:
                    self.rejections[label] = str(error)
            self.error = ''
        except (ValueError, KeyError, TypeError, ImportError, cv2.error) as error:
            self.error = str(error)

    def reload_config(self):
        try:
            stamp = self.config_stamp()
            if stamp == self.file_stamp:
                return
            candidate = load_config(self.path)
            if any(candidate[key] != self.config[key] for key in ('detections_topic', 'camera_info_topic')):
                raise ValueError('topic changes need a range-node restart')
            self.config = candidate
            self.file_stamp = self.config_stamp()
            self.mount = MountModel(tuple(candidate['pan_pivot_xy']), candidate['tilt_axis_height'], candidate['camera_offset'])
            self.odom.max_skew = candidate['odom_max_skew_s']
            self.filters.clear()
            self.records.clear()
            self.commands.clear()
            self.config_error = ''
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.config_error = f'Range config not loaded: {error}'

    def publish(self):
        now = self.now()
        items = []
        for label in sorted(set(self.config['markers']) | set(self.records)):
            family, _, tag_id = label.rpartition(':')
            record = self.records.get(label)
            item = {'family': family, 'id': int(tag_id), 'label': label,
                    'known_position_m': self.config['markers'].get(label),
                    'fresh': False, 'range_m': None}
            if record:
                age = max(0, now-record['stamp'])
                item.update(record, age_s=age, fresh=age <= self.config['stale_after_s'])
            item['map_position_m'] = item['known_position_m'] or item.get('observed_position_m')
            items.append(item)
        stream_fresh = self.last_received is not None and time.monotonic()-self.last_received < 2
        result = {'frame_id': (self.field or {}).get('frame_id', 'map'),
                  'field_revision': (self.field or {}).get('revision'), 'stamp': now,
                  'source': 'AprilTag 3 estimate_tag_pose', 'distance_kind': 'horizontal robot-centre range',
                  'config_file': str(self.path), 'marker_size_m': self.config['marker_size_m'],
                  'stream_fresh': stream_fresh, 'error': self.config_error or self.error,
                  'rejected': self.rejections, 'markers': items}
        self.result_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        self.publish_markers(result)

    def publish_markers(self, result):
        output = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        output.markers.append(clear)
        for index, item in enumerate(result['markers']):
            p = item.get('map_position_m')
            if p is None:
                continue
            marker = Marker()
            marker.header.frame_id = result['frame_id']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'apriltag_ranges', index*2, Marker.CUBE
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(p[0]), float(p[1]), float(p[2])+.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = .14
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, .9, .9, 1.0) if item['fresh'] else (.5, .5, .5, .7)
            output.markers.append(marker)
            text = Marker()
            text.header = marker.header
            text.ns, text.id, text.type = marker.ns, index*2+1, Marker.TEXT_VIEW_FACING
            text.pose.position.x, text.pose.position.y, text.pose.position.z = float(p[0]), float(p[1]), float(p[2])+.35
            text.pose.orientation.w = 1.0
            text.scale.z = .15
            text.color = marker.color
            approximate = item.get('low_resolution') or item.get('samples', 0) < 3
            distance = (f'~{item["range_m"]:.1f} m' if approximate else f'{item["range_m"]:.2f} m') if item['fresh'] else 'not visible'
            text.text = f'ID {item["id"]}: {distance}'
            output.markers.append(text)
        self.markers_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerRanges()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
