#!/usr/bin/env python3
"""
MAVROS Launch File for Mini Pixhawk Integration

Launches MAVROS node for communication with ArduRover flight controller.
Provides GPS, IMU, and mission planning interface.

Usage:
    ros2 launch veter_bringup mavros.launch.py
    ros2 launch veter_bringup mavros.launch.py fcu_url:=/dev/ttyTHS0:921600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate MAVROS launch description"""

    # Declare launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS0:921600',
        description='FCU connection URL (serial:///dev/ttyTHS0:921600 for Jetson UART)'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@',
        description='GCS connection URL'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='MAVLink target system ID'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='MAVLink target component ID'
    )

    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('veter_bringup'),
        'config',
        'mavros_config.yaml'
    ])

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            config_file,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
            }
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node,
    ])
