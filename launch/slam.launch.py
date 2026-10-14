"""
Launch File: slam.launch.py
============================
Starts the complete Visual SLAM stack:
  1. Robot State Publisher   — publishes the TF tree (camera position on drone)
  2. EKF (robot_localization) — fuses IMU + visual odometry
  3. RTAB-Map Odometry       — fast visual odometry (SLAM frontend)
  4. RTAB-Map SLAM           — map building + loop closure (SLAM backend)

Data flow:
  OAK-S2 /rgb + /depth + /imu
       |
       v
  RGBDOdometry → /rtabmap/odom (10-20 Hz, local odometry)
       |
       v                         /oak/imu/data (200 Hz)
  EKF Filter ←──────────────────────────────────────────
       |
       v
  /odometry/filtered (50 Hz, fused odometry)
       |
       +──────────────────────────────────→ MAVROS → Pixhawk
       v
  RTAB-Map SLAM → /rtabmap/map (global map)
                  /rtabmap/mapData
                  /rtabmap/cloud_map (3D point cloud)

How to run:
  ros2 launch drone_slam slam.launch.py
  ros2 launch drone_slam slam.launch.py localization_mode:=true  (reload saved map)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_slam')

    # ── Configurable arguments ───────────────────────────────────────
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='true = localization mode (existing map), false = mapping mode'
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────
    # Reads the URDF file and publishes static TF transforms:
    #   base_link → oak_camera_frame → oak_imu_frame
    # Without this node, RTAB-Map does not know where the camera is
    # relative to the drone body.
    urdf_file = os.path.join(pkg, 'urdf', 'drone.urdf.xacro')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            # Publish TF at 50 Hz (sufficient for drone dynamics)
            'publish_frequency': 50.0,
        }]
    )

    # ── 2. EKF — IMU + Visual Odometry Fusion ───────────────────────
    # robot_localization implements a standard EKF (Extended Kalman Filter).
    # It receives two sensor streams:
    #   - /rtabmap/odom  : visually estimated pose (6DOF, 10-20 Hz)
    #   - /oak/imu/data  : acceleration + angular velocity (200 Hz)
    # It publishes:
    #   - /odometry/filtered : fused state estimate (50 Hz)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'ekf_params.yaml')
        ]
    )

    # ── 3. RTAB-Map Odometry (Frontend — fast) ───────────────────────
    # RGBDOdometry is a visual odometry node using a "frame-to-map" strategy.
    # For each new frame:
    #   1. Detects keypoints in the current image (GFTT/Harris)
    #   2. Matches them against keypoints from the previous frame
    #   3. Estimates 3D motion (rotation + translation) using a PnP solver
    #   4. Publishes the result as /rtabmap/odom
    # Uses the IMU for orientation initialization (wait_imu_to_init)
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id':                 'base_link',
            'odom_frame_id':            'odom',
            'approx_sync':              True,
            'approx_sync_max_interval': 0.02,
            'wait_imu_to_init':         True,
            # Frame-to-Map strategy: more robust than Frame-to-Frame
            'Odom/Strategy':            '0',
            # Require at least 10 common points to validate motion estimate
            'Vis/MinInliers':           '10',
            'Vis/InlierDistance':       '0.1',
            # Local map of up to 1000 keypoints
            'OdomF2M/MaxSize':          '1000',
            # GFTT detector (Good Features to Track)
            'Vis/FeatureType':          '6',
            'GFTT/MinDistance':         '5',
            'GFTT/QualityLevel':        '0.001',
        }],
        remappings=[
            # Input — topics from OAK-S2 camera
            ('rgb/image',       '/oak/rgb/image_raw'),
            ('depth/image',     '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('imu',             '/oak/imu/data'),
            # Output — visual odometry
            ('odom',            '/rtabmap/odom'),
        ]
    )

    # ── 4. RTAB-Map SLAM (Backend — global map) ──────────────────────
    # Rtabmap is the SLAM backend. It runs at a lower frequency than
    # odometry but maintains the global map and detects loop closures.
    # Loop closure = recognizing a previously visited area → reduces drift.
    # Inputs:
    #   - RGB + Depth images from the camera
    #   - Filtered odometry from EKF (/odometry/filtered)
    # Outputs:
    #   - /rtabmap/map         : 2D occupancy map (for Nav2)
    #   - /rtabmap/cloud_map   : 3D point cloud
    #   - /rtabmap/mapData     : full SLAM data (nodes + links)
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id':              'base_link',
            'subscribe_rgb':         True,
            'subscribe_depth':       True,
            'subscribe_odom_info':   True,
            'approx_sync':           True,
            'wait_imu_to_init':      True,
            # Mem/IncrementalMemory controls the operating mode:
            #   'true'  = mapping mode  (builds a new map)
            #   'false' = localization mode (uses existing map)
            # The value is taken from the 'localization_mode' launch argument.
            'Mem/IncrementalMemory': LaunchConfiguration('localization_mode'),
            # Spatial loop closure (not only visual)
            'RGBD/ProximityBySpace': 'true',
            # Update map only on significant motion (saves CPU)
            'RGBD/LinearUpdate':     '0.01',
            'RGBD/AngularUpdate':    '0.01',
            # Max 700ms per SLAM backend iteration
            'Rtabmap/TimeThr':       '700',
            # 400 visual features per frame
            'Kp/MaxFeatures':        '400',
        }],
        remappings=[
            # Input — camera images
            ('rgb/image',       '/oak/rgb/image_raw'),
            ('depth/image',     '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            # Input — filtered odometry (from EKF, not raw odometry)
            ('odom',            '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        localization_mode_arg,
        robot_state_publisher,
        ekf_node,
        rtabmap_odom_node,
        rtabmap_slam_node,
    ])
