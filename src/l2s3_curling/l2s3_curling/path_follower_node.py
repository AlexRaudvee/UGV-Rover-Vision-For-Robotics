"""Separately launched slow follower with explicit arming and fail-closed checks."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

from std_srvs.srv import Trigger

from .path_follow import command


class PathBlocked(ValueError):
    """The current path intersects the latest inflated obstacle grid."""


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.armed = False
        self.path = self.pose = self.grid = None
        self.path_at = self.pose_at = self.grid_at = self.map_at = self.planner_at = 0.
        self.map_state = self.planner_state = {}
        self.declare_parameter('verbose', False)
        self.reason = 'disarmed'
        self.path_sequence = 0
        self.replan_started = None
        self.blocked_path_sequence = None
        self.blocked_goal = None
        self.far_blocked_since = None
        self.last_valid_path = self.last_valid_grid = None
        self.using_held_path = False
        self.publisher = None
        self.state_pub = self.create_publisher(String, '~/state', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/path_planner/path', self.on_path, qos)
        self.create_subscription(OccupancyGrid, '/obstacle_map/inflated', self.on_grid, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/marker_ekf/pose', self.on_pose, 1)
        self.create_subscription(String, '/obstacle_map/state', self.on_map, qos)
        self.create_subscription(String, '/path_planner/state', self.on_planner, 1)
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Disarmed: slow path follower (10 cm/s maximum)')

    def on_path(self, m):
        self.path, self.path_at = m, time.monotonic()
        self.path_sequence += 1
    def on_pose(self, m): self.pose, self.pose_at = m, time.monotonic()
    def on_grid(self, m): self.grid, self.grid_at = m, time.monotonic()

    def on_map(self, m):
        try:
            data = json.loads(m.data)
            self.map_state = data if isinstance(data, dict) else {}
        except (ValueError, TypeError): self.map_state = {}
        self.map_at = time.monotonic()

    def on_planner(self, m):
        try:
            data = json.loads(m.data)
            self.planner_state = data if isinstance(data, dict) else {}
        except (ValueError, TypeError): self.planner_state = {}
        self.planner_at = time.monotonic()

    def trace(self, message):
        if self.get_parameter('verbose').value:
            self.get_logger().info(message)

    def check(self):
        now = time.monotonic()
        map_available = bool(self.map_state.get('ready') and now-self.map_at <= 1.5
                             and self.grid is not None and now-self.grid_at <= 1.5)
        planner_recovering = (not self.planner_state.get('ready') and
                              self.planner_state.get('reason') == 'waiting for a ready obstacle map')
        self.using_held_path = bool(self.armed and (not map_available or planner_recovering)
                                    and self.last_valid_path is not None
                                    and self.last_valid_grid is not None)
        self.trace(f'check armed={self.armed} held_path={self.using_held_path} '
                   f'path_sequence={self.path_sequence} map_ready={map_available} '
                   f'map_reason={self.map_state.get("reason")} '
                   f'planner_ready={self.planner_state.get("ready")} '
                   f'planner_reason={self.planner_state.get("reason")} '
                   f'input_ages_s pose={now-self.pose_at:.2f} path={now-self.path_at:.2f} '
                   f'grid={now-self.grid_at:.2f} map={now-self.map_at:.2f} '
                   f'planner={now-self.planner_at:.2f} depth={self.map_state.get("timing")}')
        path = self.last_valid_path if self.using_held_path else self.path
        grid = self.grid if map_available else self.last_valid_grid if self.using_held_path else self.grid
        if path is None or self.pose is None or grid is None:
            raise ValueError('waiting for path, pose and obstacle map')
        if now-self.pose_at > 1.5:
            raise ValueError('robot pose is stale')
        if not self.using_held_path and max(now-self.path_at, now-self.grid_at,
                                            now-self.map_at, now-self.planner_at) > 1.5:
            raise ValueError('path, pose, map or status is stale')
        if not self.using_held_path:
            for source, status in (('map', self.map_state), ('planner', self.planner_state)):
                if not status.get('ready'):
                    detail = status.get('reason') or 'no reason reported'
                    raise ValueError(f'{source} unready: {detail}')
        timing = self.map_state.get('timing') or {}
        age, skew = timing.get('age_s'), timing.get('pair_skew_s')
        if map_available and (not isinstance(age, (int, float)) or not isinstance(skew, (int, float))
                or not math.isfinite(age) or not math.isfinite(skew)
                or not 0 <= age <= 4.0 or not 0 <= skew <= 3.0):
            raise ValueError('depth timing exceeds temporary limits (4 s age, 3 s RGB/depth skew)')
        clock = self.get_clock().now().nanoseconds/1e9
        depth_stamp = self.map_state.get('depth_stamp_s')
        if map_available and (not isinstance(depth_stamp, (int, float)) or not math.isfinite(depth_stamp)
                or not -.1 <= clock-depth_stamp <= 4.0):
            raise ValueError('depth acquisition is stale')
        checked_messages = [self.pose]
        if not self.using_held_path:
            checked_messages.append(path)
        if map_available:
            checked_messages.append(grid)
        for m in checked_messages:
            stamp = m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if m.header.frame_id != 'map' or not -.1 <= clock-stamp <= 1.5:
                raise ValueError('path, pose or grid has invalid frame or timestamp')
        p = self.pose.pose.pose
        q = p.orientation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not math.isfinite(norm) or abs(norm-1.) > .05:
            raise ValueError('invalid localization orientation')
        cov = self.pose.pose.covariance
        if not all(math.isfinite(cov[i]) and 0 < cov[i] <= limit for i, limit in
                   ((0, .25), (7, .25), (35, math.radians(20.)**2))):
            raise ValueError('localization uncertainty is invalid or too large')
        if path.header.frame_id != 'map' or grid.header.frame_id != 'map':
            raise ValueError('path or grid has invalid frame')
        points = [(v.pose.position.x, v.pose.position.y) for v in path.poses]
        if not points or not all(math.isfinite(z) for point in points for z in point):
            raise ValueError('path is empty or invalid')
        if not self.using_held_path and math.dist((p.position.x, p.position.y), points[0]) > .3:
            raise ValueError('path no longer starts near the rover')
        if self.using_held_path and min(math.dist((p.position.x, p.position.y), point)
                                        for point in points) > .3:
            raise ValueError('rover is too far from the retained path')
        g = grid
        qg = g.info.origin.orientation
        if (g.info.width <= 0 or g.info.height <= 0 or g.info.resolution <= 0
                or len(g.data) != g.info.width*g.info.height
                or abs(qg.x)+abs(qg.y)+abs(qg.z) > 1e-6 or abs(qg.w-1.) > 1e-6):
            raise ValueError('invalid occupancy grid')
        # Match A*: unknown cells may be traversed, but occupied cells and
        # points outside the grid still invalidate the route.
        position = (p.position.x, p.position.y)
        closest = min(range(len(points)), key=lambda i: math.dist(position, points[i]))
        # A retained path includes points already passed. Only check the
        # connection to the remaining route and its future segments.
        remaining = [position] + points[closest:]
        travelled = 0.0
        far_blocked = False
        for a, b in zip(remaining, remaining[1:]):
            steps = max(1, math.ceil(math.dist(a, b)/(g.info.resolution*.5)))
            for i in range(steps+1):
                x = a[0]+(b[0]-a[0])*i/steps
                y = a[1]+(b[1]-a[1])*i/steps
                ix = math.floor((x-g.info.origin.position.x)/g.info.resolution)
                iy = math.floor((y-g.info.origin.position.y)/g.info.resolution)
                if not (0 <= ix < g.info.width and 0 <= iy < g.info.height):
                    raise ValueError('path leaves observed field')
                if g.data[iy*g.info.width+ix] >= 50:
                    if travelled + math.dist(a, (x, y)) <= .8:
                        raise PathBlocked('near path crosses occupied space')
                    far_blocked = True
            travelled += math.dist(a, b)
        # The steering lookahead can cut across a turn. Check the direct
        # connection to the chosen target, not only the grid path itself.
        target = next((point for point in points[closest:] if math.dist(position, point) >= .45), points[-1])
        steps = max(1, math.ceil(math.dist(position, target)/(g.info.resolution*.5)))
        for i in range(steps + 1):
            x = position[0] + (target[0]-position[0])*i/steps
            y = position[1] + (target[1]-position[1])*i/steps
            ix = math.floor((x-g.info.origin.position.x)/g.info.resolution)
            iy = math.floor((y-g.info.origin.position.y)/g.info.resolution)
            if not (0 <= ix < g.info.width and 0 <= iy < g.info.height):
                raise ValueError('steering target leaves field')
            if g.data[iy*g.info.width+ix] >= 50:
                raise PathBlocked('steering lookahead crosses occupied space')
        if self.count_publishers('/cmd_vel') != (1 if self.publisher is not None else 0):
            raise ValueError('another velocity publisher is present')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v, w, reason = command((p.position.x, p.position.y, yaw), points,
                               max_speed=.10, max_turn=.25)
        self.trace(f'steering position={position} yaw_deg={math.degrees(yaw):.1f} '
                   f'target={target} goal={points[-1]} closest_index={closest} '
                   f'far_blocked={far_blocked} linear={v:.3f} angular={w:.3f} reason={reason}')
        if far_blocked and reason == 'following':
            if self.far_blocked_since is None:
                self.far_blocked_since = now
            if now - self.far_blocked_since >= 1.0:
                raise PathBlocked('distant path blocked; replan not received')
            reason = 'following'
        else:
            self.far_blocked_since = None
        if not self.using_held_path and not far_blocked and reason == 'following':
            self.last_valid_path, self.last_valid_grid = path, grid
        return v, w, reason

    def start(self, _request, response):
        if self.armed:
            response.success, response.message = False, 'already active'
            return response
        try:
            _, _, reason = self.check()
            if reason != 'following':
                raise ValueError(reason)
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
            self.replan_started = self.blocked_path_sequence = None
            self.blocked_goal = None
            self.far_blocked_since = None
            self.armed, self.reason = True, 'following'
            response.success, response.message = True, 'started at up to 0.10 m/s'
        except (ValueError, OverflowError, IndexError, TypeError) as error:
            self.reason = str(error)
            self.trace(f'Start rejected: {self.reason}')
            response.success, response.message = False, self.reason
        return response

    def stop(self, _request, response):
        self.armed = False
        self.replan_started = self.blocked_path_sequence = None
        self.blocked_goal = None
        self.far_blocked_since = None
        self.last_valid_path = self.last_valid_grid = None
        self.send(0., 0.)
        self.release_publisher()
        self.reason = 'stopped by request'
        response.success, response.message = True, self.reason
        return response

    def send(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        if self.publisher is not None:
            self.trace(f'cmd_vel linear={v:.3f} angular={w:.3f} armed={self.armed} reason={self.reason}')
            self.publisher.publish(m)

    def release_publisher(self):
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None

    def tick(self):
        if self.armed:
            try:
                v, w, self.reason = self.check()
                if self.using_held_path and self.reason == 'following':
                    self.reason = 'following last valid path; map or planner recovering'
                if self.replan_started is not None:
                    if time.monotonic() - self.replan_started >= 2.0:
                        raise ValueError('replan timed out')
                    goal = self.path.poses[-1].pose.position
                    if math.dist((goal.x, goal.y), self.blocked_goal) > .05:
                        raise ValueError('goal changed while waiting for replan')
                    if self.path_sequence <= self.blocked_path_sequence:
                        self.reason = 'waiting for a fresh replan'
                        self.send(0., 0.)
                        self.state_pub.publish(String(data=json.dumps({'armed': True, 'reason': self.reason,
                                                                        'speed_limit_mps': .10})))
                        return
                    self.replan_started = self.blocked_path_sequence = None
                    self.blocked_goal = None
                if self.reason not in ('following', 'following last valid path; map or planner recovering'):
                    self.armed = False
                    v = w = 0.
                self.send(v, w)
                if not self.armed:
                    self.release_publisher()
            except PathBlocked as error:
                self.trace(f'Route blocked: {error}')
                now = time.monotonic()
                if self.replan_started is None:
                    self.replan_started, self.blocked_path_sequence = now, self.path_sequence
                    goal = self.path.poses[-1].pose.position
                    self.blocked_goal = (goal.x, goal.y)
                self.send(0., 0.)
                if now - self.replan_started >= 2.0:
                    self.armed = False
                    self.reason = 'replan timed out: path remains occupied'
                    self.replan_started = self.blocked_path_sequence = None
                    self.blocked_goal = None
                    self.release_publisher()
                else:
                    self.reason = 'path blocked; waiting for a fresh replan'
            except (ValueError, OverflowError, IndexError, TypeError) as error:
                self.armed = False
                self.reason = str(error)
                self.get_logger().warning(f'Follower disarmed: {self.reason}')
                self.replan_started = self.blocked_path_sequence = None
                self.blocked_goal = None
                self.send(0., 0.)
                self.release_publisher()
        self.state_pub.publish(String(data=json.dumps({'armed': self.armed, 'reason': self.reason,
                                                        'speed_limit_mps': .10})))


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.armed:
            node.send(0., 0.)
        node.destroy_node()
        rclpy.try_shutdown()
