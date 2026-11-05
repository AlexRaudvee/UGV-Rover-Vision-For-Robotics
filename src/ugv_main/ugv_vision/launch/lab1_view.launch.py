"""Start Lab 1 cameras, depth preview, and fixed robot geometry transforms."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision = get_package_share_directory('ugv_vision')
    model = os.environ.get('UGV_MODEL', 'ugv_rover')
    urdf = Path(get_package_share_directory('ugv_description')) / 'urdf' / (model + '.urdf')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(vision, 'launch', 'camera.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(vision, 'launch', 'oak_d_lite.launch.py'))),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='ugv',
            parameters=[{'robot_description': ParameterValue(urdf.read_text(), value_type=str)}],
            output='screen', respawn=True, respawn_delay=3.0,
        ),
    ])
