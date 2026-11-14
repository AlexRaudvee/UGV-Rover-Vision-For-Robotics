"""Start the marker detector on an already running camera (start_rover.sh)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('l2s1_marker_detection')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'detector.yaml')),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('preset', default_value='improved'),
        Node(package='l2s1_marker_detection', executable='detector', name='marker_detector',
             output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'image_topic': LaunchConfiguration('image_topic'),
                 'preset': LaunchConfiguration('preset'),
             }]),
    ])
