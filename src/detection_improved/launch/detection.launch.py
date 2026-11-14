"""Start WLS depth detectors (or optional LiDAR obstacles) and histograms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def config(name):
    return os.path.join(get_package_share_directory('detection_improved'), 'config', name)


def launch_nodes(context):
    camera = LaunchConfiguration('camera').perform(context)
    depth_obstacles = LaunchConfiguration('obstacle_range_source').perform(context) == 'depth'
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    return [
        Node(package='detection_improved', executable='depth_detector', name='detector',
             namespace='detection_improved/object', output='screen',
             parameters=[config('depth.yaml'), sim_time],
             condition=IfCondition(LaunchConfiguration('depth')),
             respawn=True, respawn_delay=3.0),
        Node(package='detection_improved',
             executable='depth_detector' if depth_obstacles else 'lidar_detector', name='detector',
             namespace='detection_improved/obstacle', output='screen',
             parameters=([config('depth.yaml'), config('lidar.yaml'), sim_time] if depth_obstacles
                         else [config('lidar.yaml'), config(camera + '.yaml'), sim_time]),
             condition=IfCondition(LaunchConfiguration('lidar')),
             respawn=True, respawn_delay=3.0),
        Node(package='detection_improved', executable='histogram', name='histogram',
             namespace='detection_improved/polar_hist', output='screen',
             parameters=[config('histogram.yaml'), sim_time],
             condition=IfCondition(LaunchConfiguration('histogram'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Start obstacle branch using the selected range source'),
        DeclareLaunchArgument('histogram', default_value='true'),
        DeclareLaunchArgument('camera', default_value='oak', choices=['oak', 'usb'],
                              description='Camera for the LiDAR detector'),
        DeclareLaunchArgument('obstacle_range_source', default_value='depth', choices=['depth', 'lidar'],
                              description='Use WLS depth or independent LiDAR ranges for obstacles'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
