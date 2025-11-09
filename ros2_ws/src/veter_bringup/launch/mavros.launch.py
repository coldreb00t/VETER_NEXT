#!/usr/bin/env python3
"""
Launch file for MAVROS connection to Mini Pixhawk (ArduRover)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MAVROS"""

    # Get package directory
    pkg_dir = get_package_share_directory('veter_bringup')

    # Declare launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyUSB0:921600',
        description='FCU connection URL (serial device:baudrate)'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@',
        description='GCS connection URL (for telemetry)'
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

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'fcu_protocol': 'v2.0',
                'system_id': 255,
                'component_id': 240,
                # Plugin configuration
                'plugin_allowlist': ['sys_status', 'imu', 'gps', 'global_position',
                                    'setpoint_position', 'setpoint_velocity',
                                    'local_position', 'mission', 'command', 'param',
                                    'rc_io', 'waypoint'],
                'conn': {
                    'timeout': 10.0,
                },
                'sys': {
                    'disable_diag': False,
                },
                'imu': {
                    'frame_id': 'imu_link',
                    'linear_acceleration_stdev': 0.0003,
                    'angular_velocity_stdev': 0.0003490659,
                    'orientation_stdev': 0.0,
                    'magnetic_stdev': 0.0,
                },
                'gps': {
                    'frame_id': 'gps_link',
                },
                'global_position': {
                    'frame_id': 'map',
                    'child_frame_id': 'base_link',
                    'rot_covariance': 99999.0,
                    'use_relative_alt': True,
                },
                'local_position': {
                    'frame_id': 'map',
                    'tf_send': True,
                    'tf_frame_id': 'map',
                    'tf_child_frame_id': 'base_link',
                },
            }
        ],
        remappings=[
            ('/mavros/imu/data', '/imu/data'),
            ('/mavros/global_position/global', '/gps/fix'),
            ('/mavros/global_position/local', '/gps/local'),
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node
    ])
