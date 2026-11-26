"""Localization-only smooth camera pan; never publishes vehicle velocity."""
import json
import math
import time
from collections import deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String

from l2s1_marker_detection.camera_model import CameraModel
from .pan_tracking import PanController
from .geometry import MountModel
from .relocalization_search import RelocalizationSearch


class PanTracker(Node):
    def __init__(self):
        super().__init__('marker_pan_tracker')
        defaults = dict(target_ids=[0, 1, 2, 3, 4, 5], target_family='APRILTAG_standard41h12', missing_frames=3,
                        detection_timeout_s=.8, max_image_age_s=1., max_speed_deg_s=8.,
                        relocalize_speed_deg_s=12., gain=.7,
                        deadband_deg=3., smoothing_alpha=.25, max_pan_deg=120., tilt_deg=0.,
                        update_rate_hz=20., pan_sign=-1., tilt_sign=1., horizontal_fov_deg=0., fov_range_m=4.,
                        max_hamming=1, min_decision_margin=20., one_marker='center',
                        search_reach_deg=36., manual_hold_s=3.)
        for name, value in defaults.items():
            # Accept YAML numbers such as `1` for second/degree settings too.
            self.declare_parameter(name, value,
                                   descriptor=ParameterDescriptor(dynamic_typing=isinstance(value, float)))
        self.p = {name: self.get_parameter(name).value for name in defaults}
        p = self.p
        for name, default in defaults.items():
            if isinstance(default, float):
                if type(p[name]) not in (int, float):
                    raise ValueError(f'{name} must be a number')
                p[name] = float(p[name])
        self.control = PanController(p['target_ids'], p['missing_frames'], p['detection_timeout_s'],
                                     p['max_speed_deg_s'], p['gain'], p['deadband_deg'],
                                     p['smoothing_alpha'], p['max_pan_deg'],
                                     p['one_marker'], p['search_reach_deg'])
        if not all(math.isfinite(p[k]) for k in ('update_rate_hz', 'tilt_deg', 'pan_sign', 'tilt_sign', 'horizontal_fov_deg', 'fov_range_m', 'max_image_age_s', 'relocalize_speed_deg_s')):
            raise ValueError('Tracker settings must be finite')
        if p['relocalize_speed_deg_s'] <= 0:
            raise ValueError('relocalize_speed_deg_s must be positive')
        if not 5 <= p['update_rate_hz'] <= 100 or p['fov_range_m'] <= 0 or p['max_image_age_s'] <= 0 or not 0 <= p['horizontal_fov_deg'] < 180:
            raise ValueError('Invalid update rate, FOV, image age or range')
        if type(p['max_hamming']) is not int or not 0 <= p['max_hamming'] <= 2 or not math.isfinite(p['min_decision_margin']) or p['min_decision_margin'] < 0:
            raise ValueError('Invalid decoder quality limits')
        if abs(p['pan_sign']) != 1 or abs(p['tilt_sign']) != 1 or not -30 <= p['tilt_deg'] <= 90:
            raise ValueError('Invalid pan/tilt sign or tilt angle')
        camera_file = Path(get_package_share_directory('ugv_vision')) / 'config/camera_info.yaml'
        self.cameras = {g: CameraModel.from_yaml(camera_file, rectified=g == 'rectified') for g in ('raw', 'rectified')}
        self.geometry = 'rectified'
        self.commands = deque(maxlen=300)
        self.last_head_received = None
        self.last_stamp = None
        self.last_tick = time.monotonic()
        # Manual camera control (e.g. a game controller): commands on this topic take
        # over the camera; tracking resumes manual_hold_s seconds after the last one.
        self.manual = None          # (monotonic time, pan_deg, tilt_deg)
        self.relocalize_until = 0.
        self.search = None
        self.search_pose = None
        self.search_pose_received = None
        self.search_markers = {}
        self.search_accepted = {}
        self.create_subscription(JointState, '/localization/pan_tilt_manual', self.on_manual, 1)
        self.create_subscription(String, '/localization/relocalize_camera', self.on_relocalize, 1)
        self.create_subscription(String, '/marker_ranges/result', self.on_marker_map, 1)
        self.create_subscription(String, '/marker_ekf/state', self.on_ekf_state, 1)
        self.pub = self.create_publisher(JointState, '/localization/pan_tilt_command', 1)
        self.state_pub = self.create_publisher(String, '~/state', 1)
        self.create_subscription(String, '/marker_detector/detections', self.on_detections, 1)
        self.create_subscription(CameraInfo, '/camera_info', self.on_camera, qos_profile_sensor_data)
        self.create_subscription(JointState, '/ugv/pan_tilt_commanded', self.on_head, 10)
        self.create_timer(1/p['update_rate_hz'], self.tick)

    def on_camera(self, message):
        try:
            models = {g: CameraModel.from_camera_info(message, rectified=g == 'rectified') for g in ('raw', 'rectified')}
            if all(m.width > 0 and m.height > 0 and m.K[0, 0] > 0 and m.K[1, 1] > 0 and
                   np.all(np.isfinite(m.K)) and np.all(np.isfinite(m.D)) for m in models.values()):
                self.cameras = models
        except (TypeError, ValueError):
            return

    def on_head(self, message):
        """Driver-selected commands, not encoder feedback or baseline zero states."""
        try:
            pan = math.degrees(message.position[message.name.index('pt_base_link_to_pt_link1')])/self.p['pan_sign']
            if not math.isfinite(pan) or abs(pan) > 180:
                return
            now = time.monotonic()
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            if self.last_head_received is None or now-self.last_head_received > 1:
                self.control.pan = pan
                self.control.tracks.clear()
                self.commands.clear()
            if self.commands and stamp < self.commands[-1][0]:
                self.commands.clear()
                self.control.tracks.clear()
            self.commands.append((stamp, pan))
            self.last_head_received = now
        except (ValueError, IndexError):
            return

    def on_manual(self, message):
        """Manual pan/tilt in joint radians, same format as /localization/pan_tilt_command."""
        try:
            pan = math.degrees(message.position[message.name.index('pt_base_link_to_pt_link1')])/self.p['pan_sign']
            tilt = math.degrees(message.position[message.name.index('pt_link1_to_pt_link2')])/self.p['tilt_sign']
        except (ValueError, IndexError):
            return
        if math.isfinite(pan) and math.isfinite(tilt) and abs(pan) <= 180 and -30 <= tilt <= 90:
            self.manual = (time.monotonic(), pan, tilt)

    def on_detections(self, message):
        try:
            data = json.loads(message.data)
            stamp = float(data['stamp'])
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if not math.isfinite(stamp) or not -.1 <= age <= self.p['max_image_age_s'] or stamp == self.last_stamp:
                return
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.control.tracks.clear()
            geometry = data.get('image_geometry', 'rectified')
            camera = self.cameras[geometry]
            if data['width'] <= 0 or data['height'] <= 0:
                return
            matrix = camera.matrix_for(data['width'], data['height'])
            exposure_pan = next((pan for t, pan in reversed(self.commands) if t <= stamp), None)
            if exposure_pan is None:
                return
            bearings = {}
            for item in data.get('detections', []):
                if item['family'] != self.p['target_family'] or item['id'] not in self.p['target_ids']:
                    continue
                try:
                    if item.get('hamming', 0) > self.p['max_hamming'] or item.get('decision_margin', 0) < self.p['min_decision_margin']:
                        continue
                    corners = np.asarray(item['corners'], dtype=float).reshape(4, 1, 2)
                    if not np.all(np.isfinite(corners)):
                        continue
                    rays = cv2.undistortPoints(corners, matrix, camera.D).reshape(4, 2)
                    bearings[item['id']] = math.degrees(math.atan(-float(rays[:, 0].mean())))
                except (TypeError, ValueError, cv2.error):
                    continue
            self.last_stamp, self.geometry = stamp, geometry
            self.control.observe(bearings, time.monotonic(), exposure_pan)
        except (ValueError, KeyError, TypeError, ZeroDivisionError):
            return

    def on_relocalize(self, message):
        try:
            request = json.loads(message.data)
            deadline = float(request['deadline_s'])
            marker_id = request.get('marker_id')
            if marker_id is not None and type(marker_id) is not int:
                return
            remaining = deadline - self.get_clock().now().nanoseconds/1e9
            if not math.isfinite(remaining) or remaining > 8.1:
                return
            if remaining <= 0.:
                self.relocalize_until = 0.
                self.search = None
                return
            now = time.monotonic()
            self.relocalize_until = now + min(remaining, 8.)
            self.control.start_relocalization(marker_id, now)
            self.search = RelocalizationSearch(deadline-min(remaining, 8.), self.p['target_family'],
                                               self.control.limit, self.control.recovery_bearing)
        except (ValueError, KeyError, TypeError):
            return

    def on_marker_map(self, message):
        try:
            data = json.loads(message.data)
            if data.get('frame_id') != 'map':
                return
            self.search_markers = {item['id']: item['known_position_m'][:2]
                for item in data['markers'] if item.get('family') == self.p['target_family']
                and type(item.get('id')) is int and isinstance(item.get('known_position_m'), list)
                and len(item['known_position_m']) >= 2
                and all(math.isfinite(float(x)) for x in item['known_position_m'][:2])}
        except (ValueError, KeyError, TypeError):
            return

    def on_ekf_state(self, message):
        try:
            data = json.loads(message.data)
            pose = (float(data['x_m']), float(data['y_m']), math.radians(float(data['heading_deg'])))
            if (data.get('frame_id') != 'map' or data.get('mode') not in ('tracking', 'odometry_only')
                    or not all(math.isfinite(x) for x in pose)):
                return
            self.search_pose = pose
            self.search_pose_received = time.monotonic()
            self.search_accepted = {k: float(v) for k, v in data.get('accepted_marker_stamps_s', {}).items()
                                    if isinstance(k, str) and math.isfinite(float(v))}
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def tick(self):
        now = time.monotonic()
        if self.last_head_received is None or now-self.last_head_received > 1:
            self.last_tick = now
            self.state_pub.publish(String(data=json.dumps({'ready': False, 'mode': 'waiting_for_driver'})))
            return
        if self.manual and now-self.manual[0] <= self.p['manual_hold_s']:
            _, pan, tilt = self.manual
            self.control.pan = pan          # tracking continues from here afterwards
            self.control.mode = 'manual'
        elif self.search is not None and now < self.relocalize_until:
            # Refresh track expiry with the normal controller before choosing a search action.
            self.control.tracked_ids = [tag for tag in self.control.ids if tag in self.control.tracks
                and self.control.tracks[tag]['misses'] < self.control.grace
                and now-self.control.tracks[tag]['time'] <= self.control.timeout]
            pose = self.search_pose if self.search_pose_received is not None and now-self.search_pose_received <= 1. else None
            target, self.control.mode = self.search.step(self.control, now, pose, self.search_markers, self.search_accepted)
            maximum = self.p['relocalize_speed_deg_s'] * min(max(0., now-self.last_tick), .1)
            pan = self.control.pan + max(-maximum, min(maximum, target-self.control.pan))
            self.control.pan = pan
            tilt = self.p['tilt_deg']
        else:
            pan = self.control.step(now-self.last_tick, now, relocalizing=now < self.relocalize_until)
            tilt = self.p['tilt_deg']
        self.last_tick = now
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
        message.position = [self.p['pan_sign']*math.radians(pan), self.p['tilt_sign']*math.radians(tilt)]
        self.pub.publish(message)
        camera = self.cameras[self.geometry]
        fx, cx = camera.K[0, 0], camera.K[0, 2]
        left, right = math.atan2(cx, fx), math.atan2(camera.width-cx, fx)
        if self.p['horizontal_fov_deg'] > 0:
            left = right = math.radians(self.p['horizontal_fov_deg']/2)
        origin = MountModel().camera_center(math.radians(pan), math.radians(tilt))
        state = dict(ready=True, pan_deg=pan, tilt_deg=tilt, mode=self.control.mode,
                     search_target_id=self.search.target_id if self.search and now < self.relocalize_until else None,
                     tracked_ids=self.control.tracked_ids,
                     left_rad=left, right_rad=right, range_m=self.p['fov_range_m'],
                     origin_base=[float(origin[0]), float(origin[1]), .01],
                     horizontal_fov_deg=math.degrees(left+right))
        self.state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = PanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
