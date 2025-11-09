#!/usr/bin/env python3
"""
Minimal launch file for VETER_NEXT - just MAVROS and DroneCAN bridge
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate minimal launch description"""

    # Get package directories
    bringup_dir = get_package_share_directory('veter_bringup')
    dronecan_dir = get_package_share_directory('veter_dronecan_bridge')

    # Include MAVROS launch
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'mavros.launch.py')
        )
    )

    # Include DroneCAN bridge launch
    dronecan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dronecan_dir, 'launch', 'dronecan_bridge.launch.py')
        )
    )

    return LaunchDescription([
        mavros_launch,
        dronecan_launch
    ])
