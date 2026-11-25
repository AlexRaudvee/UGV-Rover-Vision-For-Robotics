"""Preview steering only; no motor topic is created."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from .path_follow import command


class PathFollowerPreview(Node):
    def __init__(self):
        super().__init__('path_follower_preview')
        self.pose = self.path = None
        self.pose_at = self.path_at = 0.
        self.planner_ready = False
        self.planner_at = 0.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/path_planner/path', self.on_path, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/marker_ekf/pose', self.on_pose, 1)
        self.create_subscription(String, '/path_planner/state', self.on_status, 1)
        self.cmd = self.create_publisher(Twist, '~/cmd_vel', 1)
        self.status = self.create_publisher(String, '~/state', 1)
        self.create_timer(.1, self.tick)

    def on_path(self, message):
        self.path, self.path_at = message, time.monotonic()

    def on_pose(self, message):
        self.pose, self.pose_at = message, time.monotonic()

    def on_status(self, message):
        try:
            self.planner_ready = json.loads(message.data).get('ready') is True
        except (ValueError, AttributeError):
            self.planner_ready = False
        self.planner_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        v, w, reason = 0., 0., 'waiting for fresh path, pose and planner'
        if (self.path is not None and self.pose is not None and self.planner_ready
                and max(now-self.path_at, now-self.pose_at, now-self.planner_at) < 1.5
                and self.path.header.frame_id == self.pose.header.frame_id == 'map'):
            clock = self.get_clock().now().nanoseconds/1e9
            stamp = lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            if all(-.1 <= clock-stamp(m) <= 1.5 for m in (self.pose, self.path)):
                p = self.pose.pose.pose
                q = p.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                points = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
                v, w, reason = command((p.position.x, p.position.y, yaw), points)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.cmd.publish(msg)
        self.status.publish(String(data=json.dumps({'mode': 'preview_only', 'reason': reason,
                                                   'linear': v, 'angular': w})))


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
