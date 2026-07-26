#!/usr/bin/env python3
"""
ROS 2 Launch File for CAN Motor Driver 8030D

Launches the can_driver_8030 node with parameters loaded from
config/can_driver_params.yaml. All parameters can be overridden
via command-line arguments.

Usage:
  # Default launch (loads config/can_driver_params.yaml)
  ros2 launch can_driver can_driver_8030.launch.py

  # Override parameters from command line
  ros2 launch can_driver can_driver_8030.launch.py motor_speed_max:=300

  # Use a custom config file
  ros2 launch can_driver can_driver_8030.launch.py config_file:=/path/to/custom.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for can_driver_8030 node."""

    # Package share directory
    pkg_share = get_package_share_directory('can_driver')

    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'can_driver_params.yaml')

    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML parameter configuration file'
    )

    # -------------------------------------------------------------------------
    # Parameters that can be overridden from command line
    # -------------------------------------------------------------------------
    can_device_type_arg = DeclareLaunchArgument(
        'can_device_type', default_value='4',
        description='CAN device type: 3=VCI_USBCAN1, 4=VCI_USBCAN2')

    can_device_index_arg = DeclareLaunchArgument(
        'can_device_index', default_value='0',
        description='CAN device index (0 = first device)')

    can_channel_arg = DeclareLaunchArgument(
        'can_channel', default_value='0',
        description='CAN channel: 0=channel1, 1=channel2')

    motor_speed_max_arg = DeclareLaunchArgument(
        'motor_speed_max', default_value='256',
        description='Maximum motor speed in RPM')

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable_on_start', default_value='true',
        description='Auto-enable motors on node start (true/false)')

    # -------------------------------------------------------------------------
    # Node Definition
    # -------------------------------------------------------------------------
    can_driver_node = Node(
        package='can_driver',
        executable='can_driver_8030',
        name='can_driver_8030',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'can_device_type': LaunchConfiguration('can_device_type'),
                'can_device_index': LaunchConfiguration('can_device_index'),
                'can_channel': LaunchConfiguration('can_channel'),
                'motor_speed_max': LaunchConfiguration('motor_speed_max'),
                'auto_enable_on_start': LaunchConfiguration('auto_enable_on_start'),
            },
        ],
        # Topic remapping (uncomment to customize):
        # remappings=[
        #     ('motor_speed', '/custom_motor_speed'),
        #     ('current_speed', '/custom_current_speed'),
        # ],
    )

    # -------------------------------------------------------------------------
    # Launch Description
    # -------------------------------------------------------------------------
    return LaunchDescription([
        config_file_arg,
        can_device_type_arg,
        can_device_index_arg,
        can_channel_arg,
        motor_speed_max_arg,
        auto_enable_arg,
        can_driver_node,
    ])
