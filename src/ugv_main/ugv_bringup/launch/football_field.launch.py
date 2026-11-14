"""Field visualization/alignment only: no hardware drivers or motor commands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('ugv_bringup'), 'param',
                                  'football_field.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('field_config', default_value=default_config),
        Node(package='ugv_bringup', executable='football_field', name='football_field',
             parameters=[{'config_file': LaunchConfiguration('field_config')}], output='screen'),
    ])
