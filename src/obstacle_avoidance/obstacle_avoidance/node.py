"""ROS node that connects the perception topics to the avoidance state machine."""

from dataclasses import fields
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from object_detection.node import transform_arrays
from object_detection.perception import scan_points_in_camera
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .controller import (
    AvoidanceController, DONE, front_blocked, parse_detections, parse_gaps, Settings,
)


def yaw_from_quaternion(q):
    """Return the yaw of a quaternion, or None before the IMU has a valid reading."""
    if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 0.5:
        return None
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AvoidanceNode(Node):
    """Run the state machine at 10 Hz and publish velocity commands."""

    def __init__(self):
        super().__init__('avoidance')
        defaults = Settings()
        for item in fields(Settings):
            self.declare_parameter(item.name, getattr(defaults, item.name))
        self.declare_parameter('detections_topic', '/obstacle_detection/detections')
        self.declare_parameter('gaps_topic', '/polar_hist/obstacle/gaps')
        self.declare_parameter('base_frame', 'base_link')
        value = lambda name: self.get_parameter(name).value
        self.settings = Settings(**{item.name: value(item.name) for item in fields(Settings)})
        self.base_frame = value('base_frame')
        self.controller = AvoidanceController(self.settings)

        self.pose, self.pose_time = None, -math.inf
        self.scan, self.scan_time = None, -math.inf
        self.perception, self.gaps = None, []
        self.scan_transform = None
        self.last_state = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(String, value('detections_topic'), self.receive_detections, 1)
        self.create_subscription(String, value('gaps_topic'), self.receive_gaps, 1)
        self.create_subscription(Odometry, '/odom', self.receive_odom, 5)
        self.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'Driving {self.settings.target_distance} m using {value("detections_topic")}')

    def receive_detections(self, message):
        """Keep the newest detections. A message that is not fresh removes them."""
        self.perception = parse_detections(json.loads(message.data), time.monotonic())

    def receive_gaps(self, message):
        """Keep the newest gaps."""
        self.gaps = parse_gaps(json.loads(message.data))

    def receive_odom(self, message):
        """Keep the newest pose."""
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if yaw is not None:
            position = message.pose.pose.position
            self.pose, self.pose_time = (position.x, position.y, yaw), time.monotonic()

    def receive_scan(self, message):
        """Keep the newest LiDAR scan."""
        self.scan, self.scan_time = message, time.monotonic()

    def scan_is_blocked(self, now):
        """Check the box in front of the robot. Returns None if the LiDAR is not usable."""
        if self.scan is None or now - self.scan_time > self.settings.data_timeout:
            return None
        if self.scan_transform is None:
            try:
                # The LiDAR is mounted rotated and does not move, so one lookup is enough.
                found = self.tf_buffer.lookup_transform(
                    self.base_frame, self.scan.header.frame_id, Time())
            except TransformException:
                return None
            self.scan_transform = transform_arrays(found.transform)
        points = scan_points_in_camera(self.scan, *self.scan_transform)
        return front_blocked(points[:, :2].tolist(), self.settings)

    def tick(self):
        """Run one control cycle."""
        now = time.monotonic()
        pose = self.pose if now - self.pose_time <= self.settings.data_timeout else None
        if self.perception is not None:
            self.perception.gaps = self.gaps
        command = self.controller.step(now, pose, self.perception, self.scan_is_blocked(now))

        twist = Twist()
        twist.linear.x, twist.angular.z = float(command.linear), float(command.angular)
        self.cmd_pub.publish(twist)
        self.status_pub.publish(String(data=json.dumps({
            'state': command.state, 'note': command.note,
            'linear': command.linear, 'angular': command.angular,
        })))
        if command.state != self.last_state:
            self.get_logger().info(f'{command.state}: {command.note}')
            self.last_state = command.state

    def done(self):
        """Check if the run has ended."""
        return self.controller.state == DONE

    def stop_robot(self):
        """Send stop commands a few times, so that at least one arrives."""
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main(args=None):
    """Run until the controller finishes or Ctrl+C is pressed, then stop the robot."""
    # Without this option, Ctrl+C closes ROS before the stop command can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AvoidanceNode()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.get_logger().info(f'Stopped: {node.controller.note}')
        node.destroy_node()
        rclpy.try_shutdown()
