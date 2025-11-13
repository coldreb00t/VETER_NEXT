#!/usr/bin/env python3
"""
Launch file for YOLOv8 object detection system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('veter_perception').find('veter_perception')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'yolo_params.yaml'])

    # Launch arguments
    use_tracker_arg = DeclareLaunchArgument(
        'use_tracker',
        default_value='true',
        description='Whether to use object tracker'
    )

    publish_annotated_arg = DeclareLaunchArgument(
        'publish_annotated',
        default_value='true',
        description='Whether to publish annotated images'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8n.pt',
        description='YOLOv8 model to use (n, s, m, l, x)'
    )

    # YOLO detector node
    yolo_detector_node = Node(
        package='veter_perception',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'model_path': LaunchConfiguration('model'),
                'publish_annotated': LaunchConfiguration('publish_annotated')
            }
        ],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/detections', '/detections'),
            ('/detections/image', '/detections/image')
        ]
    )

    # Object tracker node (optional)
    object_tracker_node = Node(
        package='veter_perception',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[config_file],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_tracker'))
    )

    return LaunchDescription([
        use_tracker_arg,
        publish_annotated_arg,
        model_arg,
        yolo_detector_node,
        object_tracker_node
    ])
