#!/usr/bin/env python3
"""
Launch file for VETER Robot Sensor Fusion
Launches MAVROS, robot_localization (EKF + navsat_transform), and static transforms
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for sensor fusion system"""

    # Get package directories
    veter_bringup_dir = get_package_share_directory('veter_bringup')
    ekf_config_file = os.path.join(veter_bringup_dir, 'config', 'ekf.yaml')
    mavros_config_file = os.path.join(veter_bringup_dir, 'config', 'mavros_config.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:115200',
        description='FCU connection URL (Crossflight via USB)'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@',
        description='Ground Control Station URL'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    # ================= MAVROS Node =================
    # Provides GPS and IMU data from Crossflight
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='/',
        output='screen',
        parameters=[
            mavros_config_file,
            {
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # ================= Robot Localization EKF Node =================
    # Fuses IMU and GPS for accurate pose estimation
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ]
    )

    # ================= Navsat Transform Node =================
    # Converts GPS (lat/lon) to odometry (XY in meters)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('/imu/data', '/mavros/imu/data'),
            ('/gps/fix', '/mavros/global_position/global'),
            ('/odometry/gps', '/odometry/gps'),
            ('/odometry/filtered', '/odometry/local')
        ]
    )

    # ================= Static Transform Publishers =================
    # Publish static transforms between frames

    # Transform: base_link -> imu_link
    # IMU is mounted at robot center, aligned with base_link
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # Transform: base_link -> gps_link
    # GPS antenna typically mounted above robot center
    # Adjust these values based on your actual GPS mounting position
    # Format: x y z yaw pitch roll parent child
    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_broadcaster',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'gps_link']
        # 0.15m above base_link (adjust to your GPS antenna height)
    )

    # Transform: odom -> map (identity transform for now)
    # This will be updated by navigation stack later
    tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # ================= Launch Description =================
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        fcu_url_arg,
        gcs_url_arg,

        # Nodes
        mavros_node,
        ekf_node,
        navsat_transform_node,

        # Static transforms
        tf_base_to_imu,
        tf_base_to_gps,
        tf_odom_to_map,
    ])
