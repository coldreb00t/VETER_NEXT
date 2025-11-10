#!/usr/bin/env python3
"""
Launch file for VETER_NEXT Channel Manager

Examples:
    # Default configuration (all channels)
    ros2 launch veter_channel_manager channel_manager.launch.py

    # RC only mode
    ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_rc_only.yaml

    # Long range mode
    ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_long_range.yaml

    # Voice control mode
    ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_voice.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for channel manager"""

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='channels_default.yaml',
        description='Configuration file name (channels_default.yaml, channels_rc_only.yaml, '
                    'channels_long_range.yaml, channels_voice.yaml)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Health monitoring update rate in Hz'
    )

    # Channel manager node
    channel_manager_node = Node(
        package='veter_channel_manager',
        executable='channel_manager_node',
        name='channel_manager',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config'),
            'update_rate': LaunchConfiguration('update_rate'),
        }],
        remappings=[
            # Main output topic
            ('/cmd_vel', '/cmd_vel'),

            # Status topics
            ('/channel_manager/status', '/channel_manager/status'),
            ('/channel_manager/active_channel', '/channel_manager/active_channel'),

            # Input topics from each channel
            ('/cmd_vel_fiber', '/cmd_vel_fiber'),
            ('/cmd_vel_starlink', '/cmd_vel_starlink'),
            ('/cmd_vel_4g', '/cmd_vel_4g'),
            ('/cmd_vel_wifi', '/cmd_vel_wifi'),
            ('/cmd_vel_dmr', '/cmd_vel_dmr'),
            ('/cmd_vel_expresslrs', '/cmd_vel_expresslrs'),
        ]
    )

    return LaunchDescription([
        config_arg,
        update_rate_arg,
        channel_manager_node,
    ])
