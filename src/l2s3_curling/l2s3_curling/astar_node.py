"""Obstacle-aware A* path preview. Never publishes velocity commands."""
import json
import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .astar import plan
from .path_plan import TARGETS


class AStarNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        target = int(self.declare_parameter('target_run', 1).value)
        if target not in TARGETS:
            raise ValueError('target_run must be 1 through 5')
        name, x, y = TARGETS[target]
        self.target_name, self.goal = f'Run {target}: {name}', (x, y)
        self.allow_unknown = self.declare_parameter('allow_unknown', True).value
        self.max_age = self.declare_parameter('max_input_age_s', 2.0).value
        self.grid = self.pose = None
        self.grid_at = self.pose_at = self.state_at = 0.
        self.map_ready = False
        self.path = []
        self.last_result = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/path', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', qos)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(OccupancyGrid, '/obstacle_map/inflated', self.on_grid, qos)
        self.create_subscription(String, '/obstacle_map/state', self.on_state, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/marker_ekf/pose', self.on_pose, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 1)
        self.create_timer(.5, self.tick)
        self.get_logger().info('A* path preview; unknown space allowed=%s; never drives' % self.allow_unknown)

    def on_grid(self, message):
        self.grid, self.grid_at = message, time.monotonic()

    def on_pose(self, message):
        self.pose, self.pose_at = message, time.monotonic()

    def on_state(self, message):
        try:
            data = json.loads(message.data)
            self.map_ready = data.get('ready') is True
        except (ValueError, TypeError, AttributeError):
            self.map_ready = False
        self.state_at = time.monotonic()

    def on_goal(self, message):
        if message.header.frame_id != 'map':
            self.get_logger().warning('Ignoring goal outside map frame')
            return
        p = message.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.goal, self.target_name = (p.x, p.y), 'Clicked target'
            self.path = []
            self.tick()

    def tick(self):
        now = time.monotonic()
        reason = ''
        points = []
        if not self.map_ready or now-self.state_at > self.max_age:
            reason = 'waiting for a ready obstacle map'
        elif self.grid is None or now-self.grid_at > self.max_age:
            reason = 'obstacle map is stale'
        elif self.pose is None or now-self.pose_at > self.max_age:
            reason = 'robot pose is stale'
        else:
            g, pose = self.grid, self.pose
            stamp = lambda h: h.stamp.sec + h.stamp.nanosec/1e9
            clock = self.get_clock().now().nanoseconds/1e9
            q = g.info.origin.orientation
            p = pose.pose.pose.position
            if (g.header.frame_id != 'map' or pose.header.frame_id != 'map'
                    or abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(q.w-1.) > 1e-6
                    or not all(math.isfinite(v) for v in (p.x, p.y))):
                reason = 'invalid map frame, pose or rotated grid'
            elif not (-.1 <= clock-stamp(g.header) <= self.max_age and
                      -.1 <= clock-stamp(pose.header) <= self.max_age):
                reason = 'map or robot pose timestamp is stale'
            else:
                o = g.info.origin.position
                try:
                    points, reason = plan(g.data, g.info.width, g.info.height, g.info.resolution,
                                          (o.x, o.y), (p.x, p.y), self.goal, self.allow_unknown)
                except (ValueError, OverflowError) as error:
                    reason = str(error)
        self.path = points
        self.publish_path()
        self.publish_markers()
        status = {'ready': bool(points), 'reason': reason, 'target': list(self.goal),
                  'target_name': self.target_name, 'algorithm': 'A*',
                  'preview_only': True, 'unknown_traversable': bool(self.allow_unknown)}
        self.state_pub.publish(String(data=json.dumps(status)))
        result = (bool(points), reason)
        if result != self.last_result:
            self.get_logger().info('A* preview: ' + ('path available' if points else reason))
            self.last_result = result

    def publish_path(self):
        msg = Path()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(self.path):
            nxt = self.path[min(i+1, len(self.path)-1)]
            prev = self.path[max(0, i-1)]
            angle = math.atan2(nxt[1]-prev[1], nxt[0]-prev[0])
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def publish_markers(self):
        markers = MarkerArray()
        for ident, kind in ((0, Marker.CYLINDER), (1, Marker.TEXT_VIEW_FACING)):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'path_planner', ident, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y = self.goal
            m.pose.position.z = .05
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = .25
            m.color.r, m.color.g, m.color.b, m.color.a = (1., .85, 0., .9) if ident == 0 else (1., 1., 1., 1.)
            if ident == 1:
                m.pose.position.y += .35
                m.text = self.target_name
            markers.markers.append(m)
        # Clear start labels published by the old planner in saved dashboards.
        for ident in (2, 3):
            m = Marker(ns='path_planner', id=ident, action=Marker.DELETE)
            m.header.frame_id = 'map'
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
