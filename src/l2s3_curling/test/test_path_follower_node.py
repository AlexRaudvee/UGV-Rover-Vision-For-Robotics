import json
import math
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from l2s3_curling.path_follower_node import PathFollower


def test_start_drive_then_fail_closed_when_map_changes():
    rclpy.init(args=[], domain_id=96)
    node = PathFollower()
    try:
        stamp = node.get_clock().now().to_msg()
        stamp_s = stamp.sec + stamp.nanosec/1e9
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.pose.position.x = .25
        pose.pose.pose.position.y = .25
        pose.pose.pose.orientation.w = 1.
        for i in (0,7,35): pose.pose.covariance[i] = .01
        path = Path()
        path.header = pose.header
        for x in (.25,.75,1.25):
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = x, .25
            path.poses.append(p)
        grid = OccupancyGrid()
        grid.header = pose.header
        grid.info.resolution = .5
        grid.info.width, grid.info.height = 4, 2
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*8
        node.on_pose(pose); node.on_path(path); node.on_grid(grid)
        node.on_map(String(data=json.dumps({'ready': True, 'depth_stamp_s': stamp_s,
                                           'timing': {'age_s': .1, 'pair_skew_s': .05}})))
        node.on_planner(String(data=json.dumps({'ready': True})))
        sent = []
        node.send = lambda v,w: sent.append((v,w))
        node.count_publishers = lambda _: 1 if node.publisher is not None else 0
        response = node.start(Trigger.Request(), Trigger.Response())
        assert response.success
        node.tick()
        assert sent[-1][0] == pytest.approx(.10)
        node.on_map(String(data=json.dumps({'ready': False, 'reason': 'missing depth'})))
        node.on_planner(String(data=json.dumps({'ready': False})))
        node.map_at -= 60.; node.grid_at -= 60.; node.path_at -= 60.; node.planner_at -= 60.
        node.tick()
        assert node.armed and sent[-1][0] > 0 and 'recovering' in node.reason
        node.on_path(path); node.on_grid(grid)
        node.on_map(String(data=json.dumps({'ready': True, 'depth_stamp_s': stamp_s,
                                           'timing': {'age_s': .1, 'pair_skew_s': .05}})))
        node.on_planner(String(data=json.dumps({'ready': False,
                                               'reason': 'waiting for a ready obstacle map'})))
        node.tick()
        assert node.armed and sent[-1][0] > 0 and 'recovering' in node.reason
        node.on_planner(String(data=json.dumps({'ready': True})))
        grid.data[1] = -1
        node.on_grid(grid)
        node.tick()
        assert node.armed and sent[-1][0] > 0
        grid.data[1] = 100
        node.on_grid(grid)
        node.tick()
        assert node.armed and sent[-1] == (0., 0.)
        assert 'replan' in node.reason
        grid.data[1] = 0
        node.on_grid(grid)
        node.tick()
        assert node.armed and sent[-1] == (0., 0.)  # old path alone cannot resume
        node.on_path(path)
        node.tick()
        assert node.armed and 0 < sent[-1][0] <= .10
        grid.data[1] = 100
        node.on_grid(grid)
        node.tick()
        node.replan_started -= 2.1
        node.tick()
        assert not node.armed and sent[-1] == (0., 0.)
        assert 'timed out' in node.reason
        grid.data[1] = 0
        node.on_grid(grid)
        node.map_state['timing']['pair_skew_s'] = 3.1
        response = node.start(Trigger.Request(), Trigger.Response())
        assert not response.success and 'timing' in response.message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_far_obstacle_allows_brief_motion_but_near_obstacle_stops():
    rclpy.init(args=[], domain_id=96)
    node = PathFollower()
    try:
        stamp = node.get_clock().now().to_msg()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.pose.position.x = pose.pose.pose.position.y = .25
        pose.pose.pose.orientation.w = 1.
        for i in (0, 7, 35): pose.pose.covariance[i] = .01
        path = Path(header=pose.header)
        for x in (.25, .75, 1.25, 1.75, 2.25):
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = x, .25
            path.poses.append(p)
        grid = OccupancyGrid(header=pose.header)
        grid.info.resolution = .5
        grid.info.width, grid.info.height = 6, 2
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*12
        node.on_pose(pose); node.on_path(path); node.on_grid(grid)
        node.on_map(String(data=json.dumps({'ready': True,
            'depth_stamp_s': stamp.sec+stamp.nanosec/1e9,
            'timing': {'age_s': .1, 'pair_skew_s': .05}})))
        node.on_planner(String(data=json.dumps({'ready': True})))
        sent = []
        node.send = lambda v, w: sent.append((v, w))
        node.count_publishers = lambda _: 1 if node.publisher is not None else 0
        assert node.start(Trigger.Request(), Trigger.Response()).success
        # A retained route can have an obstacle behind the current position.
        # It must not prevent the rover from following the remaining route.
        node.on_map(String(data=json.dumps({'ready': False})))
        pose.pose.pose.position.x = 1.25
        node.on_pose(pose)
        grid.data[0] = 100
        node.tick()
        assert node.armed and sent[-1][0] > 0
        grid.data[0] = 0
        pose.pose.pose.position.x = .25
        node.on_pose(pose); node.on_grid(grid); node.on_path(path)
        node.on_map(String(data=json.dumps({'ready': True,
            'depth_stamp_s': stamp.sec+stamp.nanosec/1e9,
            'timing': {'age_s': .1, 'pair_skew_s': .05}})))
        grid.data[3] = 100
        node.on_grid(grid); node.tick()
        assert node.armed and sent[-1][0] > 0
        grid.data[1] = 100
        node.on_grid(grid); node.tick()
        assert node.armed and sent[-1] == (0., 0.)
    finally:
        node.destroy_node()
        rclpy.shutdown()
