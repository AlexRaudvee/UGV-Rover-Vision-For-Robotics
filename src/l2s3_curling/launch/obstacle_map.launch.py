"""Launch only the preview map; no detector, driver, planner or motor output."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = []
    for parameter, package, relative in (
            ('config_file', 'l2s3_curling', 'config/obstacle_map_preview.yaml'),
            ('marker_map_file', 'l2s2_localisation', 'config/marker_map.yaml'),
            ('footprint_file', 'obstacle_avoidance_adv', 'config/controller.yaml')):
        arguments.append(DeclareLaunchArgument(parameter, default_value=str(
            Path(get_package_share_directory(package))/relative)))
    arguments.append(Node(package='l2s3_curling', executable='obstacle_map', name='obstacle_map',
                          output='screen', parameters=[{
                              name: LaunchConfiguration(name) for name in
                              ('config_file', 'marker_map_file', 'footprint_file')}]))
    return LaunchDescription(arguments)
