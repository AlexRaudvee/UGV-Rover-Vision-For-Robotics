"""Start perception and previews using existing camera drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_node(context):
    share = get_package_share_directory('obstacle_detection')
    camera = LaunchConfiguration('camera').perform(context)
    return [Node(
        package='obstacle_detection', executable='detector', name='detector',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            os.path.join(share, 'config', camera + '.yaml'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
        respawn=True, respawn_delay=3.0,
    )]


def generate_launch_description():
    share = get_package_share_directory('obstacle_detection')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='oak', choices=['oak', 'usb']),
        DeclareLaunchArgument('namespace', default_value='obstacle_detection'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'detector.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_node),
    ])
