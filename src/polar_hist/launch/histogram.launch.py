"""Start independent measured histograms from both detection streams."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('polar_hist')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'histogram.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='polar_hist', executable='histogram', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
             }]),
    ])
