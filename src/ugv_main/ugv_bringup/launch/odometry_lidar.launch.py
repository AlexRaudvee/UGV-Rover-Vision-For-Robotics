"""Estimation only: consumes existing sensors; never starts a motor driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def local_ekf(context, config, clock):
    parameters = [config, clock]
    if LaunchConfiguration('fuse_lidar').perform(context) == 'true':
        parameters.append({
            'odom1': '/odom_rf2o',
            'odom1_config': [False, False, False,
                             False, False, False,
                             True, True, False,
                             False, False, True,
                             False, False, False],
            'odom1_queue_size': 5,
            'odom1_twist_rejection_threshold': 4.0,
        })
    return [Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                 parameters=parameters, output='screen',
                 remappings=[('/odometry/filtered', '/odom')])]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ugv_bringup'), 'param', 'ekf_lidar.yaml')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fuse_lidar', default_value='true', choices=['true', 'false']),
        Node(package='ugv_bringup', executable='odometry_inputs',
             name='odometry_inputs', parameters=[config, clock], output='screen'),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen', parameters=[clock, {
                 'laser_scan_topic': '/scan', 'odom_topic': '/odom_rf2o',
                 'publish_tf': False, 'base_frame_id': 'base_footprint',
                 'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 20.0,
                 'linear_velocity_stddev': 0.05, 'angular_velocity_stddev': 0.05,
                 'max_scan_gap': 0.5, 'max_linear_speed': 1.5,
                 'max_angular_speed': 3.0,
             }]),
        OpaqueFunction(function=local_ekf, args=[config, clock]),
    ])
