"""
Launch File: camera.launch.py
==============================
Starts the OAK-S2 camera driver (depthai-ros v3).

What this launch file does:
  1. Loads parameters from oak_s2_params.yaml
  2. Starts the depthai_ros_driver node which communicates with the camera via USB
  3. Publishes RGB, Depth, IMU and Camera Info topics

Published topics:
  /oak/rgb/image_raw          - color image 720P @ 20fps
  /oak/rgb/camera_info        - RGB camera intrinsic parameters
  /oak/stereo/image_raw       - depth map @ 20fps
  /oak/stereo/camera_info     - stereo camera parameters
  /oak/imu/data               - IMU data (accel + gyro) @ 200Hz

How to run:
  ros2 launch drone_slam camera.launch.py
  ros2 launch drone_slam camera.launch.py params_file:=/custom/path.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package directory ────────────────────────────────────────────
    pkg_drone_slam = get_package_share_directory('drone_slam')

    # ── Default path to the parameters file ─────────────────────────
    default_params = os.path.join(
        pkg_drone_slam, 'config', 'oak_s2_params.yaml'
    )

    # ── Argument: allows overriding the parameters file at launch ────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the OAK-S2 parameters YAML file'
    )

    # ── Include the launch file from depthai_ros_driver ─────────────
    # depthai_ros_driver is the Luxonis package that knows how to start
    # any OAK camera. We simply pass our custom parameters to it.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        camera_launch,
    ])
