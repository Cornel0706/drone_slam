"""
Launch File: bridge.launch.py
==============================
Starts ros_gz_bridge to connect Gazebo Harmonic topics with ROS 2 Jazzy.
Translates camera images, camera info, IMU, and clock into ROS 2 topics.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_drone_sim = get_package_share_directory('drone_sim')
    bridge_config = os.path.join(pkg_drone_sim, 'config', 'ros_gz_bridge.yaml')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
        }],
    )

    return LaunchDescription([
        bridge_node,
    ])
