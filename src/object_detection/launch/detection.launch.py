"""Start the RGB/depth width detector with existing camera drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('object_detection')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='object_detection'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'detector.yaml')),
        DeclareLaunchArgument('depth_topic',
                              default_value='/oak/stereo/image_raw/compressedDepth'),
        DeclareLaunchArgument('depth_transport', default_value='compressedDepth',
                              choices=['raw', 'compressedDepth']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='object_detection', executable='detector', name='detector',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'depth_topic': LaunchConfiguration('depth_topic'),
                 'depth_transport': LaunchConfiguration('depth_transport'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool),
             }], respawn=True, respawn_delay=3.0),
    ])
