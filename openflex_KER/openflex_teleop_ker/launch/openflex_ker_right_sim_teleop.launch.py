#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Default KER transport configuration. Launch arguments can override these values.
DEFAULT_TRANSPORT = 'wifi'
DEFAULT_WIFI_HOST = '192.168.3.112'
DEFAULT_WIFI_PORT = '19090'
DEFAULT_BRIDGE_DELAY_S = 1.0


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

    ker_driver = Node(
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
    )

    right_arm_bridge = Node(
        package='openflex_teleop_ker',
        executable='ker_arm_bridge_node',
        name='openflex_ker_arm_bridge',
        output='screen',
        parameters=[config, {
            'left_controller_topic': '/ker/sim/disabled_left_controller/commands',
            'log_joint_changes': True,
            'joint_log_rate_hz': 1.0,
            'joint_log_min_change_rad': 0.005,
            'max_joint_velocity_rad_s': ParameterValue(
                max_joint_velocity, value_type=float),
            'max_gripper_velocity_m_s': ParameterValue(
                max_gripper_velocity, value_type=float),
            'target_timeout_s': ParameterValue(target_timeout, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'transport', default_value=DEFAULT_TRANSPORT,
            choices=['usb', 'serial', 'wifi']),
        DeclareLaunchArgument('wifi_host', default_value=DEFAULT_WIFI_HOST),
        DeclareLaunchArgument('wifi_port', default_value=DEFAULT_WIFI_PORT),
        DeclareLaunchArgument(
            'drop_command_on_sensor_error', default_value='false'),
        DeclareLaunchArgument('use_low_pass_filter', default_value='true'),
        DeclareLaunchArgument('low_pass_alpha', default_value='0.2'),
        DeclareLaunchArgument('max_joint_velocity_rad_s', default_value='3.0'),
        DeclareLaunchArgument('max_gripper_velocity_m_s', default_value='0.3'),
        DeclareLaunchArgument('target_timeout_s', default_value='0.25'),
        ker_driver,
        TimerAction(period=DEFAULT_BRIDGE_DELAY_S, actions=[right_arm_bridge]),
    ])
