"""Prepare independent velocity measurements for the planar odometry EKF."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class WheelVelocity:
    """Difference accumulated wheel distances; rebase after gaps/resets, never guess turns."""

    def __init__(self, max_speed=1.5, max_gap=0.5):
        self.previous = None
        self.max_speed = max_speed
        self.max_gap = max_gap

    def update(self, values, stamp):
        if len(values) != 2 or not all(math.isfinite(v) for v in values):
            self.previous = None
            return None
        current = (float(values[0]), float(values[1]), stamp)
        previous, self.previous = self.previous, current
        if previous is None:
            return None
        dt = stamp - previous[2]
        if not 0.001 <= dt <= self.max_gap:
            return None
        left = (current[0] - previous[0]) / dt
        right = (current[1] - previous[1]) / dt
        if max(abs(left), abs(right)) > self.max_speed:
            return None
        return (left + right) / 2.0


class OdometryInputs(Node):
    def __init__(self):
        super().__init__('odometry_inputs')
        defaults = {
            'wheel_velocity_stddev': 0.20,
            'lateral_velocity_stddev': 0.05,
            'gyro_stddev': 0.03,
            'max_wheel_speed': 1.5,
            'wheel_distance_scale': 0.87527,
            'input_timeout': 0.5,
            'base_frame': 'base_footprint',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        for key in ('wheel_velocity_stddev', 'lateral_velocity_stddev', 'gyro_stddev', 'max_wheel_speed', 'input_timeout', 'wheel_distance_scale'):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.wheels = WheelVelocity(self.config['max_wheel_speed'], self.config['input_timeout'])
        self.last_input = {}
        self.wheel_pub = self.create_publisher(Odometry, 'odom/wheels', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/estimation', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Float32MultiArray, 'odom/odom_raw', self.wheel_callback, 20)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom_rf2o', self.lidar_callback, 10)
        self.create_timer(0.5, self.diagnostics)

    def wheel_callback(self, msg):
        now = self.get_clock().now()
        velocity = self.wheels.update(msg.data, now.nanoseconds / 1e9)
        if velocity is None:
            return
        out = Odometry()
        # Legacy wheel feedback has no acquisition stamp: use receipt time.
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = self.config['base_frame']
        out.pose.pose.orientation.w = 1.0
        out.twist.twist.linear.x = (
            velocity * self.config['wheel_distance_scale']
        )
        for index in (0, 7, 14, 21, 28, 35):
            out.twist.covariance[index] = 1e6
            out.pose.covariance[index] = 1e6
        out.twist.covariance[0] = self.config['wheel_velocity_stddev'] ** 2
        # A wheeled rover does not drive sideways. Keep this soft constraint
        # active with real wheel feedback so a lost LiDAR cannot leave an old
        # lateral velocity extrapolating forever. Variance permits some slip.
        out.twist.twist.linear.y = 0.0
        out.twist.covariance[7] = self.config['lateral_velocity_stddev'] ** 2
        self.wheel_pub.publish(out)
        self.last_input['wheels'] = now.nanoseconds / 1e9

    def imu_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        if not -0.05 <= now - stamp <= self.config['input_timeout']:
            return
        if not msg.header.frame_id or not all(math.isfinite(v) for v in (
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            return
        # Preserve sensor frame and acquisition time. The EKF applies the IMU TF.
        out = Imu()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.orientation_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        for index in (0, 4, 8):
            out.angular_velocity_covariance[index] = self.config['gyro_stddev'] ** 2
        self.imu_pub.publish(out)
        self.last_input['imu'] = stamp

    def lidar_callback(self, msg):
        self.last_input['lidar'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def diagnostics(self):
        now = self.get_clock().now()
        status = DiagnosticStatus(name='odometry/input_freshness', hardware_id='rover')
        missing = []
        for source in ('wheels', 'imu', 'lidar'):
            stamp = self.last_input.get(source)
            age = None if stamp is None else now.nanoseconds / 1e9 - stamp
            fresh = age is not None and -0.05 <= age <= self.config['input_timeout']
            if not fresh:
                missing.append(source)
            status.values.append(KeyValue(key=f'{source}_age_s', value=str(age)))
        status.level = DiagnosticStatus.WARN if missing else DiagnosticStatus.OK
        status.message = ('Missing/stale: ' + ', '.join(missing) if missing
                          else 'Wheel, IMU and LiDAR measurements fresh (accuracy unverified)')
        out = DiagnosticArray()
        out.header.stamp = now.to_msg()
        out.status = [status]
        self.diag_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryInputs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
