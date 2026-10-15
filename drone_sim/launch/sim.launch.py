"""
Launch File: sim.launch.py
===========================
Launches Gazebo Harmonic with the custom Visual SLAM world
and starts the ros_gz_bridge.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_drone_sim = get_package_share_directory('drone_sim')
    default_world = os.path.join(pkg_drone_sim, 'worlds', 'drone_slam_world.sdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the Gazebo SDF world file'
    )

    # ── 1. Gazebo Harmonic Simulation ──────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    # ── 2. ROS-Gazebo Bridge ────────────────────────────────────────
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_sim, 'launch', 'bridge.launch.py')
        )
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        bridge,
    ])
