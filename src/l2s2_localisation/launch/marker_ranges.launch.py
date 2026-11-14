"""Read existing marker detections; start no camera, scanner or motor driver."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('l2s2_localisation')) / 'config/marker_ranges.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('marker_range_config', default_value=config),
        Node(package='l2s2_localisation', executable='ranges', name='marker_ranges', output='screen',
             parameters=[{'config_file': LaunchConfiguration('marker_range_config')}]),
    ])
