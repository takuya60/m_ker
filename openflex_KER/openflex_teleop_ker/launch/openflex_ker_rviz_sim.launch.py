#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Default KER transport configuration. Launch arguments can override these values.
DEFAULT_TRANSPORT = 'wifi'
DEFAULT_WIFI_HOST = '192.168.3.114'
DEFAULT_WIFI_PORT = '19090'
DEFAULT_ENABLE_SAFETY_CHECK = 'false'


def generate_launch_description():
    package_share = get_package_share_directory('openflex_teleop_ker')
    bringup_share = get_package_share_directory('openarmx_bringup')
    default_config = os.path.join(package_share, 'config', 'openflex_ker.yaml')
    config = LaunchConfiguration('config_file')
    transport = LaunchConfiguration('transport')
    wifi_host = LaunchConfiguration('wifi_host')
    wifi_port = LaunchConfiguration('wifi_port')
    enable_safety = LaunchConfiguration('enable_safety_check')

    openarmx_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'openarmx.bimanual.launch.py')
        ),
        launch_arguments={
            'use_fake_hardware': 'true',
            'robot_controller': 'forward_position_controller',
            'enable_forward_effort': 'false',
        }.items(),
    )

    ker_driver = Node(
        package='openflex_teleop_ker',
        executable='ker_driver_node',
        name='openflex_ker_driver',
        output='screen',
        parameters=[config, {
            'transport': ParameterValue(transport, value_type=str),
            'wifi_host': ParameterValue(wifi_host, value_type=str),
            'wifi_port': ParameterValue(wifi_port, value_type=int),
            'drop_command_on_sensor_error': False,
        }],
    )

    right_arm_bridge = Node(
        package='openflex_teleop_ker',
        executable='ker_arm_bridge_node',
        name='openflex_ker_arm_bridge',
        output='screen',
        parameters=[config, {
            'enable_safety_check': ParameterValue(enable_safety, value_type=bool),
            'left_controller_topic': '/ker/sim/disabled_left_controller/commands',
            'log_joint_changes': True,
            'joint_log_rate_hz': 1.0,
            'joint_log_min_change_rad': 0.005,
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
            'enable_safety_check', default_value=DEFAULT_ENABLE_SAFETY_CHECK),
        openarmx_bringup,
        ker_driver,
        TimerAction(period=4.0, actions=[right_arm_bridge]),
    ])
