"""Opt-in controller; never starts sensors or motor drivers and never auto-arms."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obstacle_avoidance_adv'),'config','controller.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('goal_distance',default_value='8.0'),
        DeclareLaunchArgument('params_file',default_value=config),
        Node(package='obstacle_avoidance_adv',executable='controller',name='obstacle_avoidance_adv',
             output='screen',parameters=[LaunchConfiguration('params_file'),{
                 'goal_distance':ParameterValue(LaunchConfiguration('goal_distance'),value_type=float),
                 'command_topic':PythonExpression(["'/cmd_vel' if '",LaunchConfiguration('drive'),
                    "' == 'true' else '/obstacle_avoidance_adv/cmd_vel'"]),
             }]),
    ])
