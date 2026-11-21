"""Path to a Session 3 target, shown in Vizanti. Plans only; never drives.

    ros2 run l2s3_curling path_planner --ros-args -p target_run:=1

Inputs
  pose_topic  (default /marker_ekf/pose)   robot pose from the Kalman filter
              (or /marker_triangulation/pose for the camera-only fix)
  /goal_pose  geometry_msgs/PoseStamped   optional: a target clicked in Vizanti
Outputs (frame of the pose, normally 'map')
  ~/path      nav_msgs/Path               straight line from the start to the target
  ~/markers   visualization_msgs/MarkerArray   target, line start, labels
  ~/state     std_msgs/String (JSON)      distance, sideways offset, heading error, plan number
"""
import json
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .path_plan import TARGETS, PathPlanner, line_points


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathNode(Node):

    def __init__(self):
        super().__init__('path_planner')
        p = self.declare_parameter
        p('pose_topic', '/marker_ekf/pose')
        p('goal_topic', '/goal_pose')
        p('target_run', 1)                 # 1..5, assignment Table 2
        p('target_x', float('nan'))        # set both to use any other point instead
        p('target_y', float('nan'))
        p('max_offset', 0.3)               # re-plan when this far off the line (m)
        p('max_jump', 0.3)                 # re-plan when the estimate jumps this far (m)
        p('arrive', 0.15)                  # target reached within this distance (m)
        p('spacing', 0.1)                  # distance between path points (m)
        v = lambda name: self.get_parameter(name).value  # noqa: E731
        self.v = v

        tx, ty = v('target_x'), v('target_y')
        if math.isfinite(tx) and math.isfinite(ty):
            self.target_name, goal = 'Chosen point', (tx, ty)
        else:
            name, x, y = TARGETS[int(v('target_run'))]
            self.target_name, goal = f'Run {int(v("target_run"))}: {name}', (x, y)
        self.planner = PathPlanner(goal, v('max_offset'), v('max_jump'), v('arrive'))
        self.frame = 'map'
        self.last_state = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/path', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(PoseWithCovarianceStamped, v('pose_topic'), self._on_pose, 10)
        self.create_subscription(PoseStamped, v('goal_topic'), self._on_goal, 10)
        self.create_timer(1.0, self._republish)
        self._publish_markers()
        self.get_logger().info(f'Path planner: {self.target_name} at ({goal[0]:.2f}, {goal[1]:.2f}), '
                               f'pose from {v("pose_topic")}. Plans only, never drives.')

    # ------------------------------------------------------------ inputs
    def _on_goal(self, message):
        p = message.pose.position
        self.target_name = 'Clicked target'
        self.planner.set_goal((p.x, p.y))
        self.get_logger().info(f'New target from Vizanti: ({p.x:.2f}, {p.y:.2f})')
        self._publish_markers()

    def _on_pose(self, message):
        self.frame = message.header.frame_id or 'map'
        p = message.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        status, replanned = self.planner.update(pose)
        if replanned:
            self.get_logger().info(f'Plan {status["plan_number"]}: {status["plan_reason"]} -> line from '
                                   f'({pose[0]:.2f}, {pose[1]:.2f}) to the target, '
                                   f'{status["line_length_m"]:.2f} m')
            self._publish_path()
            self._publish_markers()
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_state >= 0.2:                 # 5 Hz is plenty for watching
            self.last_state = now
            status['target_name'] = self.target_name
            self.state_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------ outputs
    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_path(self):
        plan = self.planner.plan
        if plan is None:
            return
        path = Path()
        path.header.frame_id, path.header.stamp = self.frame, self._stamp()
        heading = math.atan2(plan.goal[1] - plan.start[1], plan.goal[0] - plan.start[0])
        for x, y in line_points(plan.start, plan.goal, self.v('spacing')):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _marker(self, ident, kind, x, y, scale, colour, text=''):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame, self._stamp()
        m.ns, m.id, m.type, m.action = 'path_planner', ident, kind, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = colour
        m.text = text
        return m

    def _publish_markers(self):
        gx, gy = self.planner.goal
        markers = [
            self._marker(0, Marker.CYLINDER, gx, gy, 0.3, (1.0, 0.85, 0.0, 0.9)),
            self._marker(1, Marker.TEXT_VIEW_FACING, gx, gy + 0.35, 0.25, (1.0, 1.0, 1.0, 1.0),
                         f'{self.target_name} ({gx:.2f}, {gy:.2f})'),
        ]
        plan = self.planner.plan
        if plan is not None:
            sx, sy = plan.start
            markers += [
                self._marker(2, Marker.SPHERE, sx, sy, 0.15, (0.2, 0.6, 1.0, 0.9)),
                self._marker(3, Marker.TEXT_VIEW_FACING, sx, sy - 0.3, 0.2, (0.7, 0.85, 1.0, 1.0),
                             f'plan {plan.number} start ({sx:.2f}, {sy:.2f}): {plan.reason}'),
            ]
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _republish(self):
        self._publish_path()
        self._publish_markers()


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
