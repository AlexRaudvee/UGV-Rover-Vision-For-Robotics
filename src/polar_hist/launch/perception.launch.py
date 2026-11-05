"""Run both detectors and histogram previews using existing sensor drivers."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    actions = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    for package, filename in [('object_detection', 'detection.launch.py'),
                              ('obstacle_detection', 'detection.launch.py'),
                              ('polar_hist', 'histogram.launch.py')]:
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', filename)),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        )]))
    return LaunchDescription(actions)
