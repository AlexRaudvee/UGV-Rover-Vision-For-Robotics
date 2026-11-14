"""Marker detector + pan-tilt scanner, on an already running rover (start_rover.sh)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('l2s2_localisation')
    config = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('preset', default_value='improved'),
        DeclareLaunchArgument('marker_size_m', default_value='0.0'),
        Node(package='l2s1_marker_detection', executable='detector', name='marker_detector',
             output='screen',
             parameters=[os.path.join(get_package_share_directory('l2s1_marker_detection'), 'config', 'detector.yaml'), {
                 'image_topic': LaunchConfiguration('image_topic'),
                 'preset': LaunchConfiguration('preset')}]),
        Node(package='l2s2_localisation', executable='scanner', name='marker_scanner',
             output='screen',
             parameters=[config('scanner.yaml'), {
                 'marker_size_m': ParameterValue(LaunchConfiguration('marker_size_m'),
                                                 value_type=float)}]),
    ])
