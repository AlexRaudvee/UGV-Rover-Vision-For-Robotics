"""Launch the physical rover, cameras, Vizanti, and obstacle perception."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
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
    vision_config = os.path.join(get_package_share_directory('ugv_vision'), 'config')
    wls_config = os.path.join(vision_config, 'wls_depth.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('avoidance', default_value='false',
                              description='Launch advanced obstacle avoidance; stays IDLE until start service'),
        DeclareLaunchArgument('avoidance_drive', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('goal_distance', default_value='8.0'),
        DeclareLaunchArgument('lidar_crop', default_value=PythonExpression([
            "'false' if '", LaunchConfiguration('avoidance'), "'.lower() == 'true' else 'true'"])),
        DeclareLaunchArgument('base', default_value='true',
                              description='Start motor interface, sensors, LiDAR, and robot transforms'),
        DeclareLaunchArgument('odometry', default_value='lidar', choices=['lidar', 'legacy'],
                              description='Select LiDAR/wheel/IMU fusion or original odometry'),
        DeclareLaunchArgument('usb_camera', default_value='true'),
        DeclareLaunchArgument('oak_camera', default_value='true'),
        DeclareLaunchArgument('vizanti', default_value='true'),
        DeclareLaunchArgument('depth_3d', default_value='true',
                              description='Project OAK metric depth into odom for the 3D view'),
        DeclareLaunchArgument('wls', default_value='true',
                              description='Publish shared RGB-guided WLS depth for detectors and 3D'),
        DeclareLaunchArgument('obstacles', default_value='false'),
        DeclareLaunchArgument('objects', default_value='false'),
        DeclareLaunchArgument('polar_hist', default_value='false'),
        DeclareLaunchArgument('improved_detection', default_value='true',
                              description='Run improved depth/LiDAR detection and histograms'),
        DeclareLaunchArgument('obstacle_range_source', default_value='depth', choices=['depth', 'lidar'],
                              description='Range source for the obstacle detector; depth uses WLS'),
        DeclareLaunchArgument('obstacle_camera', default_value='oak', choices=['oak', 'usb']),
        SetEnvironmentVariable('UGV_MODEL', EnvironmentVariable(
            'UGV_MODEL', default_value='ugv_rover')),
        SetEnvironmentVariable('LDLIDAR_MODEL', EnvironmentVariable(
            'LDLIDAR_MODEL', default_value='ld19')),
        include('ugv_bringup', 'bringup_imu_origin.launch.py', 'base', use_rviz='false',
                odometry=LaunchConfiguration('odometry'),
                lidar_crop=LaunchConfiguration('lidar_crop')),
        # The base launch already publishes the robot model. Include camera drivers
        # individually instead of lab1_view, which would duplicate that publisher.
        include('ugv_vision', 'camera.launch.py', 'usb_camera'),
        include('ugv_vision', 'oak_d_lite.launch.py', 'oak_camera'),
        Node(package='ugv_vision', executable='depth_cloud', name='depth_cloud',
             condition=IfCondition(LaunchConfiguration('depth_3d')),
             parameters=[os.path.join(vision_config, 'depth_cloud.yaml')],
             output='screen'),
        Node(package='ugv_vision', executable='wls_depth', name='wls_filter',
             condition=IfCondition(LaunchConfiguration('wls')),
             parameters=[wls_config],
             output='screen', respawn=True, respawn_delay=3.0),
        Node(package='ugv_vision', executable='depth_preview', name='wls_depth_preview',
             condition=IfCondition(LaunchConfiguration('vizanti')),
             parameters=[wls_config],
             output='screen', respawn=True, respawn_delay=3.0),
        include('vizanti_server', 'vizanti_server.launch.py', 'vizanti',
                default_widget_config=os.path.join(get_package_share_directory('polar_hist'),
                                                  'config', 'vizanti_layout.json')),
        include('obstacle_detection', 'detection.launch.py', 'obstacles',
                camera=LaunchConfiguration('obstacle_camera')),
        include('object_detection', 'detection.launch.py', 'objects'),
        include('polar_hist', 'histogram.launch.py', 'polar_hist'),
        include('obstacle_avoidance_adv', 'avoidance.launch.py', 'avoidance',
                drive=LaunchConfiguration('avoidance_drive'),
                goal_distance=LaunchConfiguration('goal_distance')),
        include('detection_improved', 'detection.launch.py', 'improved_detection',
                camera=LaunchConfiguration('obstacle_camera'),
                obstacle_range_source=LaunchConfiguration('obstacle_range_source')),
    ])
