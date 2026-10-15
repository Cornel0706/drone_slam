"""
Launch File: slam_full.launch.py
=================================
Production launch file — starts the ENTIRE stack with a single command:
  1. OAK-S2 Camera
  2. SLAM stack (robot_state_publisher + EKF + RTAB-Map odom + RTAB-Map SLAM)
  3. Vision → MAVROS bridge (vision_to_mavros.py)
  4. MAVROS (communication with Pixhawk)

Usage:
  On the real drone (RPi5):
    ros2 launch drone_slam slam_full.launch.py

  With USB instead of UART:
    ros2 launch drone_slam slam_full.launch.py fcu_url:=/dev/ttyACM0:57600

  Localization mode (map already built):
    ros2 launch drone_slam slam_full.launch.py localization_mode:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('drone_slam')

    # ── Launch arguments ─────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyAMA0:921600',
        description='Pixhawk connection port (UART or USB)'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@192.168.1.100:14550',
        description='QGroundControl IP on ground station laptop'
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='Localization mode (true) or mapping mode (false)'
    )

    # ── 1. Camera — starts first (must be active before SLAM) ────────
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'camera.launch.py')
        )
    )

    # ── 2. SLAM — starts 3 seconds after camera ──────────────────────
    # The 3-second delay gives the camera time to initialize
    # and begin publishing all topics (RGB + depth + IMU).
    slam = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'localization_mode': LaunchConfiguration('localization_mode')
                }.items()
            )
        ]
    )

    # ── 3. Vision → MAVROS bridge — starts at 5 seconds ─────────────
    # Waits for SLAM to be running and publishing its first frames
    vision_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_slam',
                executable='vision_to_mavros.py',
                name='vision_to_mavros',
                output='screen',
            )
        ]
    )

    # ── 4. MAVROS — starts at 5 seconds ─────────────────────────────
    mavros = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'mavros.launch.py')
                ),
                launch_arguments={
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                }.items()
            )
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        localization_mode_arg,
        camera,
        slam,
        vision_bridge,
        mavros,
    ])
