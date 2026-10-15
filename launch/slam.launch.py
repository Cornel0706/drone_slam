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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('drone_slam')
    rtabmap_params_file = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    ekf_params_file = os.path.join(pkg, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'drone.urdf.xacro')

    # ── Configurable arguments ───────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='true = localization mode (existing map), false = mapping mode'
    )

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # Mem/IncrementalMemory: 'true' when localization_mode is false (mapping),
    #                        'false' when localization_mode is true (localization).
    # NOTE: Must be value_type=str because RTAB-Map parameters are std::string.
    incremental_memory = ParameterValue(
        PythonExpression([
            "'false' if '", LaunchConfiguration('localization_mode'), "'.lower() in ['true', '1'] else 'true'"
        ]),
        value_type=str
    )

    init_wm_with_all_nodes = ParameterValue(
        PythonExpression([
            "'true' if '", LaunchConfiguration('localization_mode'), "'.lower() in ['true', '1'] else 'false'"
        ]),
        value_type=str
    )

    # In mapping mode, wipe any stale database (~/.ros/rtabmap.db) from previous sessions
    delete_db_on_start = ParameterValue(
        PythonExpression([
            "'false' if '", LaunchConfiguration('localization_mode'), "'.lower() in ['true', '1'] else 'true'"
        ]),
        value_type=bool
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────
    # Reads the URDF/xacro file and publishes static TF transforms:
    #   base_link → oak_camera_frame → oak_rgb_camera_optical_frame
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': use_sim_time,
        }]
    )

    # ── 2. EKF — IMU + Visual Odometry Fusion ───────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # ── 3. RTAB-Map Odometry (Frontend — fast) ───────────────────────
    # RGBDOdometry estimates 6DOF motion from stereo images.
    # CRITICAL: publish_tf MUST be False because robot_localization (EKF)
    # is responsible for publishing the odom -> base_link transform.
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'publish_tf': False,
                'use_sim_time': use_sim_time,
                'always_process_most_recent_frame': False,
            }
        ],
        remappings=[
            ('rgb/image',       '/oak/rgb/image_raw'),
            ('depth/image',     '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('imu',             '/oak/imu/data'),
            ('odom',            '/rtabmap/odom'),
            ('odom_info',       '/rtabmap/odom_info'),
        ]
    )

    # ── 4. RTAB-Map SLAM (Backend — global map) ──────────────────────
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'Mem/IncrementalMemory': incremental_memory,
                'Mem/InitWMWithAllNodes': init_wm_with_all_nodes,
                'use_sim_time': use_sim_time,
                'map_always_update': True,
                'delete_db_on_start': delete_db_on_start,
            }
        ],
        remappings=[
            ('rgb/image',       '/oak/rgb/image_raw'),
            ('depth/image',     '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('odom',            '/odometry/filtered'),
            ('mapData',         '/rtabmap/mapData'),
            ('cloud_map',       '/rtabmap/cloud_map'),
            ('map',             '/rtabmap/grid_map'),
            ('grid_map',        '/rtabmap/grid_map'),
            ('mapPath',         '/rtabmap/mapPath'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_mode_arg,
        robot_state_publisher,
        ekf_node,
        rtabmap_odom_node,
        rtabmap_slam_node,
    ])
