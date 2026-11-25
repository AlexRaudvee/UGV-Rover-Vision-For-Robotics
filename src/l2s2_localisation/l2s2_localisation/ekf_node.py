"""Real-time localization while driving: /odom + marker observations -> EKF pose.

    ros2 run l2s2_localisation ekf                   (after colcon build)

Inputs
  /odom                          nav_msgs/Odometry   robot motion (any fixed frame)
  /marker_ranges/observations    std_msgs/String     per image: range + bearing of every
                                                     mapped marker (range_node.py)
Outputs
  ~/pose    geometry_msgs/PoseWithCovarianceStamped  field frame ('map'), 20 Hz
  ~/state   std_msgs/String (JSON)                   pose, uncertainty, mode, rejections
Services
  ~/reset   std_srvs/Trigger   restart from the saved initial alignment

The daily launch enables publish_tf: this node alone owns map -> odom. The field
server supplies a separate saved-start alignment and revision, never the fused
pose as a measurement. Applying the saved start resets the EKF at current odometry.
Standalone publish_tf defaults false, preserving pose-only use.

Odometry is applied up to each image's timestamp (interpolated), then that image's
markers are fused one by one. Between images the published pose is the filter
state moved on by the latest odometry. Pose and TF use that SAME odometry timestamp.
Stale odometry stops pose/TF publication. Never drives.
"""

import json
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .ekf import CHI2_2DOF_99_9, MarkerEKF, MotionNoise, map_to_odom, relative_motion
from .ranging import PoseHistory, planar_pose
from .triangulation import Field, Landmark
from .triangulation_advanced import NoiseModel


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MarkerEKFNode(Node):

    def __init__(self):
        super().__init__('marker_ekf')
        p = self.declare_parameter
        p('observations_topic', '/marker_ranges/observations')
        p('odom_topic', '/odom')
        p('publish_rate_hz', 20.0)
        p('publish_tf', False, descriptor=ParameterDescriptor(read_only=True))
        p('odom_timeout_s', 1.0)
        p('max_image_age_s', 1.5)
        p('sigma_px', 0.5)               # corner noise
        p('sigma_pan_deg', 1.0)          # pan servo error (no feedback)
        p('xy_per_m', 0.05)              # odometry error per metre driven
        p('h_per_rad', 0.05)             # odometry heading error per radian turned
        p('h_per_m', 0.03)               # heading drift per metre, rad
        p('gate', CHI2_2DOF_99_9)        # outlier gate (chi-square, 2 dof)
        p('recover_after', 5)            # rejections in a row before re-opening the gate
        p('init_from_start_pose', True)  # use the Vizanti field start pose if no fix yet
        p('start_sigma_m', 0.3)
        p('start_sigma_deg', 15.0)
        p('max_odom_skew_s', 0.25)
        p('tracking_timeout_s', 1.0)     # 'odometry_only' after this long without a marker
        v = lambda name: self.get_parameter(name).value  # noqa: E731
        self.v = v

        self.ekf = self._new_filter()
        self.odom = PoseHistory(max_skew=v('max_odom_skew_s'))
        self.latest_odom = None          # (stamp, (x, y, yaw))
        self.latest_odom_stamp = None    # preserve exact ROS nanoseconds for TF/pose
        self.last_odom_received = None
        self.filter_odom = None          # odometry pose the filter state belongs to
        self.filter_stamp = None
        self.last_update = None          # monotonic time of the last accepted marker
        self.last_used = []
        self.accepted_marker_stamps = {}
        self.frame_id = 'map'
        self.odom_frame = 'odom'
        self.field_revision = None
        self.field = None
        self.field_key = None
        self.last_field_received = None
        self.last_tf_stamp = None
        self.last_input_error = ''
        for name in ('publish_rate_hz', 'odom_timeout_s', 'max_image_age_s'):
            if not math.isfinite(v(name)) or v(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '~/pose', 1)
        self.state_pub = self.create_publisher(String, '~/state', 1)
        self.create_subscription(Odometry, v('odom_topic'), self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, v('observations_topic'), self._on_observations, 1)
        self.create_subscription(String, '/football_field/state', self._on_field,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.broadcaster = TransformBroadcaster(self) if v('publish_tf') else None
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_timer(1.0 / v('publish_rate_hz'), self._publish)
        self.get_logger().info(f'Marker EKF: {v("odom_topic")} + {v("observations_topic")}')

    def _new_filter(self):
        v = self.v
        return MarkerEKF(noise=NoiseModel(sigma_px=v('sigma_px'), sigma_pan_deg=v('sigma_pan_deg')),
                         motion=MotionNoise(v('xy_per_m'), v('h_per_rad'), v('h_per_m')),
                         gate=v('gate'), field_model=Field(), recover_after=v('recover_after'))

    def _reset(self, reason):
        self.ekf = self._new_filter()
        self.filter_odom = self.filter_stamp = self.last_update = None
        self.last_used = []
        self.accepted_marker_stamps.clear()
        self.get_logger().info(f'EKF reset: {reason}')

    def _on_reset(self, _request, response):
        if not self._odom_fresh():
            response.success, response.message = False, 'No fresh odometry; reset was not applied'
            return response
        self._reset('service call')
        started = self._start_from_field()
        response.success = True
        response.message = ('Filter restarted from saved initial alignment' if started else
                            'Filter reset; waiting for saved-start metadata or 2 markers')
        return response

    def _odom_fresh(self):
        if self.latest_odom is None or self.last_odom_received is None:
            return False
        age = self.get_clock().now().nanoseconds / 1e9 - self.latest_odom[0]
        return (time.monotonic() - self.last_odom_received <= self.v('odom_timeout_s')
                and -.1 <= age <= self.v('odom_timeout_s'))

    def _field_fresh(self):
        return self.field is not None and time.monotonic() - self.last_field_received <= 1.5

    def _tf_allowed(self):
        return (self._field_fresh() and self.field.get('ekf_localization') is True
                and self.field.get('publish_tf') is False)

    def _start_from_field(self):
        if (self.ekf.initialised or not self.v('init_from_start_pose')
                or not self._odom_fresh() or not self._field_fresh()
                or (self.broadcaster is not None and not self._tf_allowed())):
            return False
        transform = self.field.get('start_transform')
        if not isinstance(transform, list) or len(transform) != 3 or not all(
                isinstance(x, (int, float)) and math.isfinite(x) for x in transform):
            return False
        stamp, odom_pose = self.latest_odom
        start = planar_pose(transform, odom_pose)
        self.ekf.initialise(start, self.v('start_sigma_m'), self.v('start_sigma_deg'))
        self._started(stamp, odom_pose, 'saved field start')
        return True

    def _on_field(self, message):
        try:
            field = json.loads(message.data)
            frames = field['frame_id'], field['odom_frame']
            if (not all(isinstance(f, str) and f and not f.startswith('/') for f in frames)
                    or frames[0] == frames[1]):
                return
            key = (field.get('session_id'), field.get('revision'), *frames)
        except (ValueError, KeyError, TypeError):
            return
        if self.field_key is not None and key != self.field_key:
            self._reset('saved field start or field server changed')
        self.field, self.field_key = field, key
        self.last_field_received = time.monotonic()
        self.frame_id, self.odom_frame = frames
        self.field_revision = field.get('revision')
        self._start_from_field()

    # ------------------------------------------------------------ inputs
    def _on_odom(self, message):
        if message.header.frame_id != self.odom_frame:
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6 or stamp <= 0:
            return
        if self.latest_odom is not None and stamp < self.latest_odom[0]:
            self._reset('odometry clock moved backwards')
            self.field = self.field_key = self.last_field_received = None
            self.last_tf_stamp = None
        if self.latest_odom is not None and stamp == self.latest_odom[0]:
            return
        # Normalize before extracting heading.
        q.x, q.y, q.z, q.w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        pose = (p.x, p.y, yaw_of(q))
        self.odom.add(stamp, pose)
        self.latest_odom = (stamp, pose)
        self.latest_odom_stamp = message.header.stamp
        self.last_odom_received = time.monotonic()
        self._start_from_field()

    def _on_observations(self, message):
        try:
            data = json.loads(message.data)
            stamp = float(data['stamp'])
        except (ValueError, KeyError, TypeError):
            return
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not math.isfinite(stamp) or not -.1 <= age <= self.v('max_image_age_s'):
            self.last_input_error = 'Marker image is stale or has an invalid timestamp'
            return
        if self.broadcaster is not None:
            if not self._tf_allowed():
                self.last_input_error = 'Waiting for field server to delegate map -> odom'
                return
            if (data.get('field_revision') != self.field_revision
                    or data.get('field_session') != self.field.get('session_id')):
                self.last_input_error = 'Marker observations belong to an old field start'
                return
        if self.filter_stamp is not None and stamp <= self.filter_stamp:
            return                                         # old or repeated image
        if self.field is None and data.get('field_revision') != self.field_revision:
            if self.field_revision is not None:
                self._reset('football-field start pose changed')
            self.field_revision = data.get('field_revision')
        if self.field is not None and data.get('frame_id') != self.frame_id:
            return
        self.frame_id = data.get('frame_id') or self.frame_id
        field_data = data.get('field') or {}
        try:
            self.ekf.field = Field(**{k: float(field_data[k]) for k in
                                      ('half_length', 'half_width', 'border') if k in field_data})
        except (TypeError, ValueError):
            pass
        landmarks = []
        for item in data.get('observations') or []:
            try:
                landmarks.append(Landmark(item['label'], tuple(item['known_position_m'][:2]),
                                          float(item['range_m']), math.radians(float(item['bearing_deg'])),
                                          item.get('side_px')))
            except (KeyError, TypeError, ValueError):
                continue
        odom_at_image = self.odom.at(stamp)
        if odom_at_image is None or not self._odom_fresh():
            self.last_input_error = 'No fresh odometry matched to marker image'
            return
        self.last_input_error = ''

        if not self.ekf.initialised:
            if len(landmarks) >= 2 and self.ekf.initialise_from_markers(landmarks):
                self.accepted_marker_stamps.update({landmark.label: stamp for landmark in landmarks})
                self._started(stamp, odom_at_image, f'{len(landmarks)} markers')
                return
            # Legacy pose-only operation may use a standalone field's initial
            # alignment. The TF owner must never feed its own fused pose back in.
            start = data.get('odom_pose') if self.broadcaster is None else None
            if (self.v('init_from_start_pose') and isinstance(start, list) and len(start) == 3
                    and all(isinstance(x, (int, float)) and math.isfinite(x) for x in start)):
                self.ekf.initialise(start, self.v('start_sigma_m'), self.v('start_sigma_deg'))
                self._started(stamp, odom_at_image, 'field start pose')
            else:
                return                                     # still waiting
        else:
            if odom_at_image is not None and self.filter_odom is not None:
                self.ekf.predict(*relative_motion(self.filter_odom, odom_at_image))
            if odom_at_image is not None:
                self.filter_odom = odom_at_image
            self.filter_stamp = stamp

        used = []
        for landmark in landmarks:
            accepted, _ = self.ekf.update(landmark)
            if accepted:
                used.append(landmark.label)
                self.accepted_marker_stamps[landmark.label] = stamp
        if used:
            self.last_update = time.monotonic()
        self.last_used = used

    def _started(self, stamp, odom_at_image, source):
        self.filter_odom, self.filter_stamp = odom_at_image, stamp
        self.last_update = time.monotonic() if 'markers' in source else None
        state = self.ekf.as_dict()
        self.get_logger().info(f'EKF started from {source}: x={state["x_m"]} y={state["y_m"]} '
                               f'heading={state["heading_deg"]} deg')

    # ------------------------------------------------------------ output
    def _current(self):
        """Filter state moved on by the odometry received since the last image."""
        state, P = self.ekf.state.copy(), self.ekf.P.copy()
        if self.latest_odom is not None and self.filter_odom is not None:
            preview = MarkerEKF(motion=self.ekf.motion)
            preview.state, preview.P = state, P
            preview.predict(*relative_motion(self.filter_odom, self.latest_odom[1]))
            state, P = preview.state, preview.P
        return state, P

    def _publish(self):
        if not self.ekf.initialised:
            ownership_pending = self.broadcaster is not None and not self._tf_allowed()
            self.state_pub.publish(String(data=json.dumps(
                {'initialised': False, 'tf_published': False,
                 'mode': 'waiting_for_tf_ownership' if ownership_pending else 'waiting_for_start',
                 'hint': 'show 2 mapped markers, or set the start pose in Vizanti'})))
            return
        if not self._odom_fresh() or (self.broadcaster is not None and not self._tf_allowed()):
            reason = ('odometry_stale' if not self._odom_fresh() else 'waiting_for_tf_ownership')
            self.state_pub.publish(String(data=json.dumps(
                {**self.ekf.as_dict(), 'mode': reason, 'tf_published': False,
                 'last_input_error': self.last_input_error})))
            return
        state, P = self._current()
        stamp_ns = self.latest_odom_stamp.sec * 1_000_000_000 + self.latest_odom_stamp.nanosec
        if self.broadcaster is not None and self.last_tf_stamp is not None and stamp_ns <= self.last_tf_stamp:
            # Publish a corrected pose together with TF at the next odometry
            # sample, rather than revise a transform already cached at this time.
            return
        age = None if self.last_update is None else time.monotonic() - self.last_update
        mode = 'tracking' if age is not None and age <= self.v('tracking_timeout_s') else 'odometry_only'

        message = PoseWithCovarianceStamped()
        # _current() projects to latest_odom, not to wall time.
        message.header.stamp = self.latest_odom_stamp
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x, message.pose.pose.position.y = float(state[0]), float(state[1])
        message.pose.pose.orientation.z = math.sin(state[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(state[2] / 2.0)
        covariance = [0.0] * 36
        for i, row in enumerate((0, 1, 5)):                # x, y, yaw in the 6x6 layout
            for j, column in enumerate((0, 1, 5)):
                covariance[row * 6 + column] = float(P[i, j])
        covariance[14] = covariance[21] = covariance[28] = 1e6   # z, roll, pitch: unknown
        message.pose.covariance = covariance
        self.pose_pub.publish(message)
        if self.broadcaster is not None:
            transform = map_to_odom(state, self.latest_odom[1])
            tf = TransformStamped()
            tf.header = message.header
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x, tf.transform.translation.y = transform[:2]
            tf.transform.rotation.z = math.sin(transform[2]/2)
            tf.transform.rotation.w = math.cos(transform[2]/2)
            self.broadcaster.sendTransform(tf)
            self.last_tf_stamp = stamp_ns

        report = self.ekf.as_dict()
        sx, sy, sh = (math.sqrt(max(x, 0.0)) for x in np.diag(P))
        report.update(x_m=round(float(state[0]), 3), y_m=round(float(state[1]), 3),
                      heading_deg=round(math.degrees(state[2]), 1),
                      sigma_x_m=round(sx, 3), sigma_y_m=round(sy, 3),
                      sigma_heading_deg=round(math.degrees(sh), 2), mode=mode,
                      seconds_since_marker=None if age is None else round(age, 2),
                      markers_used_last_image=self.last_used, frame_id=self.frame_id,
                      accepted_marker_stamps_s=self.accepted_marker_stamps,
                      pose_stamp_s=stamp_ns / 1e9, field_revision=self.field_revision,
                      field_session=self.field.get('session_id') if self.field is not None else None,
                      tf_published=self.broadcaster is not None,
                      last_input_error=self.last_input_error)
        self.state_pub.publish(String(data=json.dumps(report)))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerEKFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
