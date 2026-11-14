"""Sweep the pan-tilt camera and report range/bearing to every marker it finds.

Needs the marker detector running (it reads /marker_detector/detections).

  ros2 service call /marker_scanner/scan std_srvs/srv/Trigger '{}'

At each pan stop the scanner waits settle_s for the servo and camera, then
collects detections for dwell_s. The result, one entry per marker relative to
the robot (range, bearing, x, y, height), is published latched on
~/result and saved as JSON under result_dir. Triangulation is not done here.

The pan-tilt has no position feedback: the commanded angle is assumed. Check
the signs once with  ros2 service call /marker_scanner/look_left ...  : the
camera must turn LEFT. If it turns right, set pan_sign to the opposite value.
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import aruco_compat as compat
from .camera_model import CameraModel
from .detector_node import _default_camera_info
from .geometry import MountModel, observe, summarize
from .localize import load_map, localize_scan


def _default_marker_map():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('marker_localization'), 'config',
                            'marker_map.yaml')
    except Exception:  # noqa: BLE001
        return ''


def parse_sizes(entries):
    """['APRILTAG_36h11:3=0.2', ...] -> {(family, id): size}."""
    sizes = {}
    for entry in entries:
        key, _, size = entry.partition('=')
        family, _, marker_id = key.rpartition(':')
        if not (family and marker_id and size):
            raise ValueError(f'marker_sizes entry {entry!r} must be FAMILY:ID=SIZE_M')
        sizes[(compat.normalize_family(family), int(marker_id))] = float(size)
    return sizes


class MarkerScannerNode(Node):

    def __init__(self):
        super().__init__('marker_scanner')
        p = self.declare_parameter
        p('detections_topic', '/marker_detector/detections')
        p('joint_topic', '/ugv/joint_states')
        p('pan_joint', 'pt_base_link_to_pt_link1')
        p('tilt_joint', 'pt_link1_to_pt_link2')
        p('pan_sign', -1.0)      # joint = pan_sign * pan(left +). URDF pan axis is -z.
        p('tilt_sign', 1.0)      # joint = tilt_sign * tilt(up +)
        p('pan_angles_deg', [-120.0, -60.0, 0.0, 60.0, 120.0, 180.0])
        p('tilt_deg', 0.0)
        p('return_pan_deg', 0.0)
        p('settle_s', 1.2)
        p('dwell_s', 1.0)
        p('marker_size_m', 0.0)  # printed black square, edge to edge; required
        p('marker_sizes', [''])  # per-marker overrides: FAMILY:ID=SIZE_M
        p('camera_info_file', _default_camera_info())
        p('pan_pivot_xy', [-0.008, 0.0])
        p('tilt_axis_height', 0.211)
        p('camera_offset', 0.048)
        p('result_dir', '/home/ws/ugv_ws/runs/marker_scans')
        p('marker_map_file', _default_marker_map())   # field positions for triangulation

        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self.value = value
        self.camera = CameraModel.from_yaml(value('camera_info_file'))
        self.mount = MountModel(tuple(value('pan_pivot_xy')), value('tilt_axis_height'),
                                value('camera_offset'))
        self._read_sizes()
        if self.default_size <= 0 and not self.sizes:
            self.get_logger().warn('marker_size_m is not set: distances cannot be computed. '
                                   'Set it to the measured marker size before scanning.')

        self.joint_pub = self.create_publisher(JointState, value('joint_topic'), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(String, '~/result', latched)
        self.create_subscription(String, value('detections_topic'), self._on_detections, 10)
        self.create_service(Trigger, '~/scan', self._on_scan)
        self.create_service(Trigger, '~/look_left', self._on_look_left)
        self.create_service(Trigger, '~/center', self._on_center)
        self.create_timer(0.05, self._tick)

        self.plan = []
        self.stop_index = -1
        self.stop_started = 0.0
        self.last_command = 0.0
        self.sightings = []
        self.skipped_sizes = set()

    def _read_sizes(self):
        """Read at start-up and before every scan, so ros2 param set takes effect."""
        self.default_size = float(self.value('marker_size_m'))
        self.sizes = parse_sizes([e for e in self.value('marker_sizes') if e])

    # ------------------------------------------------------------ pan-tilt
    def _command(self, pan, tilt):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.value('pan_joint'), self.value('tilt_joint')]
        message.position = [self.value('pan_sign') * pan, self.value('tilt_sign') * tilt]
        self.joint_pub.publish(message)
        self.last_command = time.monotonic()

    def _on_look_left(self, _request, response):
        self._command(math.radians(45.0), math.radians(self.value('tilt_deg')))
        response.success = True
        response.message = 'Commanded pan 45 deg LEFT. If the camera turned right, flip pan_sign.'
        return response

    def _on_center(self, _request, response):
        self._command(0.0, math.radians(self.value('tilt_deg')))
        response.success, response.message = True, 'Camera centred'
        return response

    # ------------------------------------------------------------ scan state machine
    @property
    def scanning(self):
        return 0 <= self.stop_index < len(self.plan)

    def _on_scan(self, _request, response):
        if self.scanning:
            response.success, response.message = False, 'A scan is already running'
            return response
        try:
            self._read_sizes()
        except ValueError as error:
            response.success, response.message = False, str(error)
            return response
        if self.default_size <= 0 and not self.sizes:
            response.success, response.message = False, 'Set marker_size_m first'
            return response
        self.plan = [math.radians(a) for a in self.value('pan_angles_deg')]
        self.sightings, self.skipped_sizes = [], set()
        self._start_stop(0)
        response.success = True
        response.message = (f'Scanning {len(self.plan)} pan stops, about '
                            f'{len(self.plan) * (self.value("settle_s") + self.value("dwell_s")):.0f}'
                            ' s; result on /marker_scanner/result')
        return response

    def _start_stop(self, index):
        self.stop_index = index
        self.stop_started = time.monotonic()
        self._command(self.plan[index], math.radians(self.value('tilt_deg')))
        self.get_logger().info(f'Pan stop {index + 1}/{len(self.plan)}: '
                               f'{math.degrees(self.plan[index]):.0f} deg')

    def _tick(self):
        if not self.scanning:
            return
        now = time.monotonic()
        if now - self.last_command > 0.5:   # resend: the first message can be lost
            self._command(self.plan[self.stop_index], math.radians(self.value('tilt_deg')))
        if now - self.stop_started >= self.value('settle_s') + self.value('dwell_s'):
            if self.stop_index + 1 < len(self.plan):
                self._start_stop(self.stop_index + 1)
            else:
                self._finish()

    def _collecting(self):
        elapsed = time.monotonic() - self.stop_started
        return self.scanning and elapsed >= self.value('settle_s')

    def _on_detections(self, message):
        if not self._collecting():
            return
        data = json.loads(message.data)
        pan, tilt = self.plan[self.stop_index], math.radians(self.value('tilt_deg'))
        for item in data.get('detections', []):
            key = (item['family'], int(item['id']))
            size = self.sizes.get(key, self.default_size)
            if size <= 0:
                self.skipped_sizes.add(key)
                continue
            try:
                self.sightings.append(observe(
                    key[0], key[1], item['corners'], size, self.camera,
                    (data['width'], data['height']), self.mount, pan, tilt, data.get('stamp', 0.0)))
            except ValueError as error:
                self.get_logger().warn(f'{key}: {error}')

    def _finish(self):
        self.stop_index = len(self.plan)
        self._command(math.radians(self.value('return_pan_deg')),
                      math.radians(self.value('tilt_deg')))
        markers = summarize(self.sightings)
        result = {'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                  'pan_angles_deg': self.value('pan_angles_deg'),
                  'tilt_deg': self.value('tilt_deg'),
                  'frames_used': len(self.sightings), 'markers': markers,
                  'skipped_no_size': [f'{f}:{i}' for f, i in sorted(self.skipped_sizes)],
                  'localization': self._localize(markers)}
        self.result_pub.publish(String(data=json.dumps(result)))
        directory = self.value('result_dir')
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, time.strftime('scan_%Y%m%d_%H%M%S.json'))
        with open(path, 'w') as stream:
            json.dump(result, stream, indent=2)

        log = self.get_logger()
        log.info(f'Scan done: {len(markers)} markers, saved {path}')
        log.info(f'{"marker":22} {"range m":>8} {"bearing":>8} {"x m":>7} {"y m":>7} '
                 f'{"height":>7} {"+/- m":>6} {"frames":>6} {"pan":>5}')
        for m in markers:
            log.info(f'{m["family"] + ":" + str(m["id"]):22} {m["range_m"]:8.3f} '
                     f'{m["bearing_deg"]:8.1f} {m["x_m"]:7.3f} {m["y_m"]:7.3f} '
                     f'{m["height_m"]:7.3f} {m["range_std_m"]:6.3f} {m["frames"]:6d} '
                     f'{m["pan_deg"]:5.0f}')
        if self.skipped_sizes:
            log.warn(f'No size for {result["skipped_no_size"]}: set marker_size_m or marker_sizes')
        fix = result['localization']
        if 'x_m' in fix:
            log.info(f'Robot at x={fix["x_m"]:.3f} m, y={fix["y_m"]:.3f} m, heading '
                     f'{fix["heading_deg"]} deg ({fix["method"]}, range RMS '
                     f'{fix["rms_range_error_m"]:.3f} m) {" ".join(fix["warnings"])}')
        else:
            log.warn(f'No position: {fix.get("error")}')
        advanced = fix.get('advanced', {})
        if 'x_m' in advanced:
            log.info(f'Advanced (range + bearing): x={advanced["x_m"]:.3f} m, '
                     f'y={advanced["y_m"]:.3f} m, heading {advanced["heading_deg"]} deg, '
                     f'+/- {advanced["sigma_x_m"]:.3f}/{advanced["sigma_y_m"]:.3f} m')

    def _localize(self, markers):
        path = self.value('marker_map_file')
        try:
            world, field_model = load_map(path)
            return localize_scan(markers, world, field_model)
        except (OSError, ValueError, KeyError) as error:
            return {'error': f'marker map {path!r}: {error}'}


def main(args=None):
    rclpy.init(args=args)
    node = MarkerScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
