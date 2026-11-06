"""Start the avoidance node. The robot starts driving as soon as its data is ready."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('obstacle_avoidance')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'avoidance.yaml')),
        DeclareLaunchArgument('target_distance', default_value='2.0'),
        DeclareLaunchArgument('preferred_side', default_value='auto',
                              choices=['auto', 'left', 'right']),
        Node(package='obstacle_avoidance', executable='avoidance', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'target_distance': ParameterValue(LaunchConfiguration('target_distance'),
                                                   value_type=float),
                 'preferred_side': LaunchConfiguration('preferred_side'),
             }]),
    ])
