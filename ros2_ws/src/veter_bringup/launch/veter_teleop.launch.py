#!/usr/bin/env python3
"""
Teleop VETER_NEXT Launch File

Launches robot with keyboard teleop for testing:
- DroneCAN Bridge
- teleop_twist_keyboard (control robot from keyboard)

Usage:
    ros2 launch veter_bringup veter_teleop.launch.py
    
    Then control with keyboard:
    - i/k: forward/backward
    - j/l: turn left/right
    - u/o/m/,: diagonal movement
    - q/z: increase/decrease speed
    - space: stop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate teleop launch description"""

    # DroneCAN Bridge launch
    dronecan_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('veter_dronecan_bridge'),
                'launch',
                'dronecan_bridge.launch.py'
            ])
        ])
    )

    # Teleop twist keyboard node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal
        remappings=[
            ('cmd_vel', 'cmd_vel')
        ]
    )

    return LaunchDescription([
        dronecan_bridge_launch,
        teleop_node,
    ])
