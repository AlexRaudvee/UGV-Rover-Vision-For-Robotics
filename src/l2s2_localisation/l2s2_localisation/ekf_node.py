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
  ~/reset   std_srvs/Trigger   forget the pose and start again

Start: the first image with 2+ markers (absolute fix), or, if init_from_start_pose
is true, the field-aligned start pose (the Vizanti football-field start) with
start_sigma_m / start_sigma_deg uncertainty. Whichever comes first.

Odometry is applied up to each image's timestamp (interpolated), then that image's
markers are fused one by one. Between images the published pose is the filter
state moved on by the latest odometry. Never publishes TF and never drives.
"""

import json
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .ekf import CHI2_2DOF_99_9, MarkerEKF, MotionNoise, relative_motion
from .ranging import PoseHistory
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
        self.filter_odom = None          # odometry pose the filter state belongs to
        self.filter_stamp = None
        self.last_update = None          # monotonic time of the last accepted marker
        self.last_used = []
        self.frame_id = 'map'
        self.field_revision = None

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '~/pose', 1)
        self.state_pub = self.create_publisher(String, '~/state', 1)
        self.create_subscription(Odometry, v('odom_topic'), self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, v('observations_topic'), self._on_observations, 1)
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
        self.get_logger().info(f'EKF reset: {reason}')

    def _on_reset(self, _request, response):
        self._reset('service call')
        response.success, response.message = True, 'Filter reset; waiting for a start pose or 2 markers'
        return response

    # ------------------------------------------------------------ inputs
    def _on_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = (p.x, p.y, yaw_of(q))
        self.odom.add(stamp, pose)
        self.latest_odom = (stamp, pose)

    def _on_observations(self, message):
        try:
            data = json.loads(message.data)
            stamp = float(data['stamp'])
        except (ValueError, KeyError, TypeError):
            return
        if self.filter_stamp is not None and stamp <= self.filter_stamp:
            return                                         # old or repeated image
        if data.get('field_revision') != self.field_revision:
            if self.field_revision is not None:
                self._reset('football-field start pose changed')
            self.field_revision = data.get('field_revision')
        self.frame_id = data.get('frame_id') or 'map'
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

        if not self.ekf.initialised:
            if len(landmarks) >= 2 and self.ekf.initialise_from_markers(landmarks):
                self._started(stamp, odom_at_image, f'{len(landmarks)} markers')
                return
            start = data.get('odom_pose')
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
        if used:
            self.last_update = time.monotonic()
        self.last_used = used

    def _started(self, stamp, odom_at_image, source):
        self.filter_odom, self.filter_stamp = odom_at_image, stamp
        self.last_update = time.monotonic()
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
            self.state_pub.publish(String(data=json.dumps(
                {'initialised': False, 'mode': 'waiting_for_start',
                 'hint': 'show 2 mapped markers, or set the start pose in Vizanti'})))
            return
        state, P = self._current()
        age = None if self.last_update is None else time.monotonic() - self.last_update
        mode = 'tracking' if age is not None and age <= self.v('tracking_timeout_s') else 'odometry_only'

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
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

        report = self.ekf.as_dict()
        sx, sy, sh = (math.sqrt(max(x, 0.0)) for x in np.diag(P))
        report.update(x_m=round(float(state[0]), 3), y_m=round(float(state[1]), 3),
                      heading_deg=round(math.degrees(state[2]), 1),
                      sigma_x_m=round(sx, 3), sigma_y_m=round(sy, 3),
                      sigma_heading_deg=round(math.degrees(sh), 2), mode=mode,
                      seconds_since_marker=None if age is None else round(age, 2),
                      markers_used_last_image=self.last_used, frame_id=self.frame_id)
        self.state_pub.publish(String(data=json.dumps(report)))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
