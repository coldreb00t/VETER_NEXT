#!/usr/bin/env python3
"""
Web Teleop Launch File
Запускает веб-интерфейс управления роботом через браузер

Компоненты:
- rosbridge_websocket: WebSocket сервер для браузера (порт 9090)
- channel_manager: Управление приоритетом каналов связи
- HTTP server: Раздает веб-интерфейс на порту 8000

Использование:
    ros2 launch veter_bringup web_teleop.launch.py

После запуска:
    - Веб-интерфейс: http://localhost:8000
    - WebSocket: ws://localhost:9090
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='9090',
        description='Порт WebSocket сервера'
    )

    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8000',
        description='Порт HTTP сервера для веб-интерфейса'
    )

    # rosbridge WebSocket server
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('websocket_port'),
            'address': '0.0.0.0',  # Listen on all interfaces
            'retry_startup_delay': 2.0,
            'fragment_timeout': 600,
            'delay_between_messages': 0,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
        }],
        output='screen'
    )

    # Channel Manager
    channel_manager_node = Node(
        package='veter_channel_manager',
        executable='channel_manager_node',
        name='channel_manager',
        output='screen'
    )

    # HTTP server для веб-интерфейса
    # Используем простой Python HTTP server
    web_interface_dir = '/home/jetson/jetson-robot-project/web_interface'

    http_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'http.server', LaunchConfiguration('http_port'),
            '--directory', web_interface_dir
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        websocket_port_arg,
        http_port_arg,
        rosbridge_node,
        channel_manager_node,
        http_server,
    ])
