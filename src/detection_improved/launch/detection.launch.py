"""Start the improved depth and LiDAR detectors and their histogram/gap node."""

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
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    return [
        Node(package='detection_improved', executable='depth_detector', name='detector',
             namespace='detection_improved/object', output='screen',
             parameters=[config('depth.yaml'), sim_time],
             condition=IfCondition(LaunchConfiguration('depth')),
             respawn=True, respawn_delay=3.0),
        Node(package='detection_improved', executable='lidar_detector', name='detector',
             namespace='detection_improved/obstacle', output='screen',
             parameters=[config('lidar.yaml'), config(camera + '.yaml'), sim_time],
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
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('histogram', default_value='true'),
        DeclareLaunchArgument('camera', default_value='oak', choices=['oak', 'usb'],
                              description='Camera for the LiDAR detector'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
