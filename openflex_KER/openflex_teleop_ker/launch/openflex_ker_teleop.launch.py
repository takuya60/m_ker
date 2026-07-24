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
    wifi_host = LaunchConfiguration('wifi_host')
    wifi_port = LaunchConfiguration('wifi_port')
    drop_on_error = LaunchConfiguration('drop_command_on_sensor_error')
    use_low_pass = LaunchConfiguration('use_low_pass_filter')
    low_pass_alpha = LaunchConfiguration('low_pass_alpha')
    max_joint_velocity = LaunchConfiguration('max_joint_velocity_rad_s')
    max_gripper_velocity = LaunchConfiguration('max_gripper_velocity_m_s')
    target_timeout = LaunchConfiguration('target_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'transport', default_value='usb', choices=['usb', 'serial', 'wifi']),
        DeclareLaunchArgument('wifi_host', default_value='openarm-ker.local'),
        DeclareLaunchArgument('wifi_port', default_value='19090'),
        DeclareLaunchArgument(
            'drop_command_on_sensor_error', default_value='true'),
        DeclareLaunchArgument('use_low_pass_filter', default_value='true'),
        DeclareLaunchArgument('low_pass_alpha', default_value='0.2'),
        DeclareLaunchArgument('max_joint_velocity_rad_s', default_value='3.0'),
        DeclareLaunchArgument('max_gripper_velocity_m_s', default_value='0.3'),
        DeclareLaunchArgument('target_timeout_s', default_value='0.25'),
        Node(
            package='openflex_teleop_ker',
            executable='ker_driver_node',
            name='openflex_ker_driver',
            output='screen',
            parameters=[config, {
                'transport': ParameterValue(transport, value_type=str),
                'wifi_host': ParameterValue(wifi_host, value_type=str),
                'wifi_port': ParameterValue(wifi_port, value_type=int),
                'drop_command_on_sensor_error': ParameterValue(
                    drop_on_error, value_type=bool),
                'use_low_pass_filter': ParameterValue(use_low_pass, value_type=bool),
                'low_pass_alpha': ParameterValue(low_pass_alpha, value_type=float),
            }],
        ),
        Node(
            package='openflex_teleop_ker',
            executable='ker_arm_bridge_node',
            name='openflex_ker_arm_bridge',
            output='screen',
            parameters=[config, {
                'max_joint_velocity_rad_s': ParameterValue(
                    max_joint_velocity, value_type=float),
                'max_gripper_velocity_m_s': ParameterValue(
                    max_gripper_velocity, value_type=float),
                'target_timeout_s': ParameterValue(target_timeout, value_type=float),
            }],
        ),
    ])
