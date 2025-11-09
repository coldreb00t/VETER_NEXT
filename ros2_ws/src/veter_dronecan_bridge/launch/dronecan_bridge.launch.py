#!/usr/bin/env python3
"""
Launch file for DroneCAN Bridge node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for DroneCAN Bridge"""

    # Get package directory
    pkg_dir = get_package_share_directory('veter_dronecan_bridge')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'dronecan_params.yaml'),
        description='Path to config file'
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )

    # Create node
    dronecan_bridge_node = Node(
        package='veter_dronecan_bridge',
        executable='dronecan_bridge',
        name='dronecan_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        config_file_arg,
        can_interface_arg,
        dronecan_bridge_node
    ])
