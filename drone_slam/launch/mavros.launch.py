"""
Launch File: mavros.launch.py
==============================
Starts MAVROS2 — the communication bridge between ROS2 and the Pixhawk 6C Mini.

What MAVROS does:
  - Translates ROS2 messages into MAVLink protocol (the native PX4 protocol)
  - Allows ROS2 to read the drone state (arming status, flight mode, GPS position)
  - Allows ROS2 to send commands to the Pixhawk (position, velocity, setpoints)
  - Forwards visual odometry (/odometry/filtered) to PX4's internal EKF2

Key topics:
  INPUT (ROS2 → Pixhawk):
    /mavros/vision_pose/pose                   — visually estimated pose (used by PX4 EKF2)
    /mavros/setpoint_velocity/cmd_vel_unstamped — velocity command
    /mavros/setpoint_position/local             — local position command

  OUTPUT (Pixhawk → ROS2):
    /mavros/state                  — status: connected, armed, flight mode
    /mavros/local_position/pose    — position estimated by PX4 (internal EKF2)
    /mavros/imu/data               — Pixhawk IMU (accelerometer + gyroscope)
    /mavros/battery                — battery level

Physical connection:
  Option A (recommended): RPi5 GPIO UART → Pixhawk TELEM2
    fcu_url: /dev/ttyAMA0:921600
  Option B (quick test): direct USB
    fcu_url: /dev/ttyACM0:57600

How to run:
  ros2 launch drone_slam mavros.launch.py
  ros2 launch drone_slam mavros.launch.py fcu_url:=/dev/ttyACM0:57600
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Ensure GeographicLib finds datasets in user directory if not in /usr/share
    geo_data_env = SetEnvironmentVariable(
        name='GEOGRAPHICLIB_DATA',
        value=os.path.expanduser('~/.local/share/GeographicLib')
    )

    # ── Configurable arguments ───────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        # Default: GPIO UART (recommended for production)
        # Change to /dev/ttyACM0:57600 for USB testing
        default_value='/dev/ttyAMA0:921600',
        description='Flight Controller connection URL: port:baudrate'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        # Forwards telemetry to QGroundControl on the ground station laptop
        # Replace the IP address with your laptop's actual IP on the network
        default_value='udp://@192.168.1.100:14550',
        description='Ground Control Station URL for QGroundControl'
    )

    # ── Launch MAVROS with the PX4 configuration ────────────────────
    # MAVROS in ROS 2 Jazzy ships px4.launch (XML).
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url':       LaunchConfiguration('fcu_url'),
            'gcs_url':       LaunchConfiguration('gcs_url'),
            'tgt_system':    '1',   # Pixhawk System ID (default 1)
            'tgt_component': '1',   # Component ID (default 1 = autopilot)
        }.items()
    )

    return LaunchDescription([
        geo_data_env,
        fcu_url_arg,
        gcs_url_arg,
        mavros_launch,
    ])
