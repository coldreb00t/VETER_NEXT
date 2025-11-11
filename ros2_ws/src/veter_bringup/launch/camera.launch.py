#!/usr/bin/env python3
"""
Simple launch file for VETER IMX477 Camera using gscam
Sony IMX477 12MP camera on CSI CAM0 port
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for IMX477 camera with gscam"""

    # GStreamer pipeline for IMX477 camera
    # 1920x1080 @ 30fps (can be changed to 3840x2160@30 or 1920x1080@60)
    gstreamer_pipeline = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! '
        'nvvidconv ! '
        'video/x-raw,format=BGRx ! '
        'videoconvert'
    )

    # Set GSCAM_CONFIG environment variable
    set_gscam_config = SetEnvironmentVariable(
        'GSCAM_CONFIG',
        gstreamer_pipeline
    )

    # gscam node
    gscam_node = Node(
        package='gscam',
        executable='gscam_node',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            'frame_id': 'camera_link',
            'sync_sink': True,
        }]
    )

    return LaunchDescription([
        set_gscam_config,
        gscam_node,
    ])
