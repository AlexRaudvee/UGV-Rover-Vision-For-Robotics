"""Launch the physical rover, cameras, Vizanti, and obstacle perception."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import os


def include(package, filename, enabled, **arguments):
    # Camera and detector launches both declare params_file and namespace.
    # Scope each include so their defaults cannot overwrite one another.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        condition=IfCondition(LaunchConfiguration(enabled)),
        launch_arguments=arguments.items(),
    )])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base', default_value='true',
                              description='Start motor interface, sensors, LiDAR, and robot transforms'),
        DeclareLaunchArgument('usb_camera', default_value='true'),
        DeclareLaunchArgument('oak_camera', default_value='true'),
        DeclareLaunchArgument('vizanti', default_value='true'),
        DeclareLaunchArgument('obstacles', default_value='true'),
        DeclareLaunchArgument('objects', default_value='true'),
        DeclareLaunchArgument('polar_hist', default_value='true'),
        DeclareLaunchArgument('obstacle_camera', default_value='oak', choices=['oak', 'usb']),
        SetEnvironmentVariable('UGV_MODEL', EnvironmentVariable(
            'UGV_MODEL', default_value='ugv_rover')),
        SetEnvironmentVariable('LDLIDAR_MODEL', EnvironmentVariable(
            'LDLIDAR_MODEL', default_value='ld19')),
        include('ugv_bringup', 'bringup_imu_origin.launch.py', 'base', use_rviz='false'),
        # The base launch already publishes the robot model. Include camera drivers
        # individually instead of lab1_view, which would duplicate that publisher.
        include('ugv_vision', 'camera.launch.py', 'usb_camera'),
        include('ugv_vision', 'oak_d_lite.launch.py', 'oak_camera'),
        include('vizanti_server', 'vizanti_server.launch.py', 'vizanti',
                default_widget_config=os.path.join(get_package_share_directory('polar_hist'),
                                                  'config', 'vizanti_layout.json')),
        include('obstacle_detection', 'detection.launch.py', 'obstacles',
                camera=LaunchConfiguration('obstacle_camera')),
        include('object_detection', 'detection.launch.py', 'objects'),
        include('polar_hist', 'histogram.launch.py', 'polar_hist'),
    ])
