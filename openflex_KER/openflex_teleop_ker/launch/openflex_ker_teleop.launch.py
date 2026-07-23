#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('openflex_teleop_ker')
    default_config = os.path.join(package_share, 'config', 'openflex_ker.yaml')
    config = LaunchConfiguration('config_file')
    transport = LaunchConfiguration('transport')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'transport', default_value='usb', choices=['usb', 'serial', 'wifi']),
        Node(
            package='openflex_teleop_ker',
            executable='ker_driver_node',
            name='openflex_ker_driver',
            output='screen',
            parameters=[config, {'transport': ParameterValue(transport, value_type=str)}],
        ),
        Node(
            package='openflex_teleop_ker',
            executable='ker_arm_bridge_node',
            name='openflex_ker_arm_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
