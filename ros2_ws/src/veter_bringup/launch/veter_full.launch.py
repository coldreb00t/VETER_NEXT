#!/usr/bin/env python3
"""
Full VETER_NEXT Launch File

Launches complete robot system:
- DroneCAN Bridge (ESP32 communication)
- Channel Manager (multi-channel failover)
- MAVROS (Mini Pixhawk GPS/IMU)
- Future: Navigation2, Vision pipeline

Usage:
    ros2 launch veter_bringup veter_full.launch.py
    ros2 launch veter_bringup veter_full.launch.py channel_config:=channels_long_range.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate full system launch description"""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )

    channel_config_arg = DeclareLaunchArgument(
        'channel_config',
        default_value='channels_default.yaml',
        description='Channel manager configuration file'
    )

    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros',
        default_value='true',
        description='Enable MAVROS (Mini Pixhawk)'
    )

    # DroneCAN Bridge launch
    dronecan_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('veter_dronecan_bridge'),
                'launch',
                'dronecan_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'can_interface': LaunchConfiguration('can_interface'),
        }.items()
    )

    # Channel Manager launch
    channel_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('veter_channel_manager'),
                'launch',
                'channel_manager.launch.py'
            ])
        ]),
        launch_arguments={
            'config': LaunchConfiguration('channel_config'),
        }.items()
    )

    # MAVROS launch (conditional, when Pixhawk connected)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('veter_bringup'),
                'launch',
                'mavros.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_mavros'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        can_interface_arg,
        channel_config_arg,
        enable_mavros_arg,
        dronecan_bridge_launch,
        channel_manager_launch,
        mavros_launch,
    ])
