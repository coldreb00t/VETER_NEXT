#!/usr/bin/env python3
"""
Launch file for VETER IMX477 Camera using gscam
Sony IMX477 12MP camera on CSI CAM0 port
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for IMX477 camera with gscam"""

    # Launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Camera image width (1920 or 3840)'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Camera image height (1080 or 2160)'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate (30 or 60 fps for 1080p, 30 fps for 4K)'
    )

    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id',
        default_value='0',
        description='Camera sensor ID (0 for CAM0, 1 for CAM1)'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name (used for topics /camera_name/image_raw)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='TF frame_id for camera'
    )

    # Get parameter values
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    framerate = LaunchConfiguration('framerate')
    sensor_id = LaunchConfiguration('sensor_id')
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')

    # gscam node with GStreamer pipeline in parameters
    # Format: nvarguscamerasrc → NVMM memory → nvvidconv → BGRx → videoconvert

    # gscam node
    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_publisher',
        namespace=camera_name,
        output='screen',
        parameters=[{
            'gscam_config': [
                'nvarguscamerasrc sensor-id=', sensor_id,
                ' ! video/x-raw(memory:NVMM),',
                'width=', width, ',',
                'height=', height, ',',
                'format=NV12,',
                'framerate=', framerate, '/1',
                ' ! nvvidconv',
                ' ! video/x-raw,format=BGRx',
                ' ! videoconvert'
            ],
            'camera_name': camera_name,
            'frame_id': frame_id,
            'sync_sink': True,
        }],
        remappings=[
            ('camera/image_raw', 'image_raw'),
            ('camera/camera_info', 'camera_info'),
        ]
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        framerate_arg,
        sensor_id_arg,
        camera_name_arg,
        frame_id_arg,
        gscam_node,
    ])
