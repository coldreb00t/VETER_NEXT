#!/usr/bin/env python3
"""
Launch file for VETER Camera Publisher
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for camera publisher"""

    # Launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Camera image width'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Camera image height'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate (fps)'
    )

    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id',
        default_value='0',
        description='Camera sensor ID (0 for CAM0)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='ROS2 topic publish rate (Hz)'
    )

    # Camera publisher node
    camera_node = Node(
        package='veter_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'framerate': LaunchConfiguration('framerate'),
            'sensor_id': LaunchConfiguration('sensor_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        framerate_arg,
        sensor_id_arg,
        publish_rate_arg,
        camera_node,
    ])
