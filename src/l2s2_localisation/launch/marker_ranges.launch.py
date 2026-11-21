"""Read existing detections and smoothly track visible tags with an existing driver."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('l2s2_localisation')) / 'config/marker_ranges.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('marker_range_config', default_value=config),
        DeclareLaunchArgument('marker_ekf', default_value='false'),
        DeclareLaunchArgument('ekf_publish_tf', default_value='false'),
        DeclareLaunchArgument('triangulation', default_value='true'),
        DeclareLaunchArgument('triangulation_config', default_value=str(
            Path(get_package_share_directory('l2s2_localisation')) / 'config/triangulation.yaml')),
        DeclareLaunchArgument('pan_tracking', default_value='true'),
        DeclareLaunchArgument('pan_tracking_config', default_value=str(
            Path(get_package_share_directory('l2s2_localisation')) / 'config/pan_tracking.yaml')),
        Node(package='l2s2_localisation', executable='ranges', name='marker_ranges', output='screen',
             parameters=[{'config_file': LaunchConfiguration('marker_range_config')}]),
        Node(package='l2s2_localisation', executable='ekf', name='marker_ekf', output='screen',
             condition=IfCondition(LaunchConfiguration('marker_ekf')),
             parameters=[{'publish_tf': ParameterValue(LaunchConfiguration('ekf_publish_tf'), value_type=bool)}]),
        Node(package='l2s2_localisation', executable='triangulation', name='marker_triangulation', output='screen',
             condition=IfCondition(LaunchConfiguration('triangulation')),
             parameters=[{'config_file': LaunchConfiguration('triangulation_config')}]),
        Node(package='l2s2_localisation', executable='pan_tracker', name='marker_pan_tracker', output='screen',
             condition=IfCondition(LaunchConfiguration('pan_tracking')),
             parameters=[LaunchConfiguration('pan_tracking_config')]),
    ])
