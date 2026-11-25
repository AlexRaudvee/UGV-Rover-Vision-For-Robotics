"""ROS preview publishes a clear path and withdraws stale/blocked paths."""
import json
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from l2s3_curling.astar_node import AStarNode


def test_node_replans_and_clears_path_when_map_unready():
    rclpy.init(args=[], domain_id=93)
    node = AStarNode()
    try:
        node.goal = (3.5, 1.5)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x, pose.pose.pose.position.y = .5, 1.5
        pose.pose.pose.orientation.w = 1.
        grid = OccupancyGrid()
        grid.header = pose.header
        grid.info.resolution = 1.
        grid.info.width, grid.info.height = 5, 4
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*20
        grid.data[1*5+2] = 100
        node.on_pose(pose)
        node.on_grid(grid)
        node.on_state(String(data=json.dumps({'ready': True})))
        node.tick()
        assert node.path and any(y != 1.5 for x, y in node.path)
        grid.data[1*5+3] = 100
        node.on_grid(grid)
        node.tick()
        assert not node.path
        grid.data[1*5+3] = 0
        node.on_grid(grid)
        node.on_state(String(data=json.dumps({'ready': False})))
        node.tick()
        assert not node.path
        assert not any(name == '/cmd_vel' for name, _ in node.get_topic_names_and_types())
    finally:
        node.destroy_node()
        rclpy.shutdown()
