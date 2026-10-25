"""
Launch File: bringup.launch.py
==============================
Alias / convenience wrapper for sim_slam.launch.py.
Launches the entire simulation SLAM stack:
  1. ROS-Gazebo topic bridges (camera, depth, IMU)
  2. Static TF tree for simulated sensor frames
  3. Visual SLAM stack (robot_state_publisher + EKF + RTAB-Map)
  4. Vision to MAVROS bridge (vision_to_mavros.py)
  5. MAVROS for PX4 SITL (UDP port 14540/14557)
  6. RViz2 visualization

Usage:
  ros2 launch drone_sim bringup.launch.py
  (or identically: ros2 launch drone_sim sim_slam.launch.py)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_drone_sim = get_package_share_directory('drone_sim')
    sim_slam_path = os.path.join(pkg_drone_sim, 'launch', 'sim_slam.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_slam_path)
        )
    ])
