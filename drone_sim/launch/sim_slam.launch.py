"""
Launch File: sim_slam.launch.py
================================
Starts the complete Visual SLAM pipeline in simulation:
  1. ROS-Gazebo bridge (bridges camera + IMU from Gazebo to ROS2)
  2. MAVROS connected to PX4 SITL via UDP (port 14540/14557)
  3. Visual SLAM stack (robot_state_publisher + EKF + RTAB-Map)
  4. Vision to MAVROS bridge (vision_to_mavros.py)

Usage:
  Terminal 1: Start PX4 SITL with Gazebo:
    cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth

  Terminal 2: Start SLAM and bridges:
    ros2 launch drone_sim sim_slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_drone_sim = get_package_share_directory('drone_sim')
    pkg_drone_slam = get_package_share_directory('drone_slam')

    # ── Arguments ────────────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        # Default UDP port for PX4 SITL
        default_value='udp://:14540@localhost:14557',
        description='MAVLink UDP connection to PX4 SITL'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550',
        description='QGroundControl UDP connection'
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='true = localization mode, false = mapping mode'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='true = automatically open rtabmap_viz 3D map GUI'
    )

    # ── 1. ROS-Gazebo Topic Bridge ──────────────────────────────────
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drone_sim, 'launch', 'bridge.launch.py')
        )
    )

    # ── 1b. Static Transforms for Gazebo Simulation Model Frames ────
    # In Gazebo SITL, x500_depth frames have model-scoped names:
    #   - x500_depth_0/OakD-Lite/base_link/IMX214 (RGB optical frame)
    #   - x500_depth_0/OakD-Lite/base_link/StereoOV7251 (Depth optical frame)
    #   - x500_depth_0/base_link/imu_sensor (IMU sensor frame)
    # These static publishers anchor them directly to base_link with REP-103 optical rotation.
    tf_camera_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_tf_camera_rgb',
        arguments=[
            '--x', '0.13233', '--y', '0.0', '--z', '0.26078',
            '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
            '--frame-id', 'base_link',
            '--child-frame-id', 'x500_depth_0/OakD-Lite/base_link/IMX214'
        ],
        parameters=[{'use_sim_time': True}]
    )

    tf_camera_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_tf_camera_depth',
        arguments=[
            '--x', '0.13233', '--y', '0.0', '--z', '0.26078',
            '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
            '--frame-id', 'base_link',
            '--child-frame-id', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'
        ],
        parameters=[{'use_sim_time': True}]
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_tf_imu',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'x500_depth_0/base_link/imu_sensor'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # ── 2. SLAM Stack (starts at 2s to allow bridge to connect) ─────
    slam = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drone_slam, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'localization_mode': LaunchConfiguration('localization_mode'),
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )

    # ── 3. Vision to MAVROS bridge (starts at 4s) ───────────────────
    vision_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drone_slam',
                executable='vision_to_mavros.py',
                name='vision_to_mavros',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # ── 4. MAVROS for SITL (starts at 3s) ───────────────────────────
    mavros = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drone_slam, 'launch', 'mavros.launch.py')
                ),
                launch_arguments={
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                }.items()
            )
        ]
    )

    # ── 5. Classical 3D Map Visualization (RViz2) ───────────────────
    rviz_config_file = os.path.join(pkg_drone_sim, 'config', 'drone_slam.rviz')
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('gui'))
            )
        ]
    )

    # ── 6. Optional RTAB-Map Diagnostic GUI (rtabmap_viz) ───────────
    rtabmap_viz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                output='screen',
                parameters=[{
                    'frame_id': 'base_link',
                    'subscribe_depth': True,
                    'subscribe_odom_info': False,
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.05,
                    'use_sim_time': True,
                }],
                remappings=[
                    ('rgb/image', '/oak/rgb/image_raw'),
                    ('depth/image', '/oak/stereo/image_raw'),
                    ('rgb/camera_info', '/oak/rgb/camera_info'),
                    ('odom', '/odometry/filtered'),
                ],
                condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
            )
        ]
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        localization_mode_arg,
        gui_arg,
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch rtabmap_viz diagnostic GUI (in addition to RViz2)'
        ),
        bridge,
        tf_camera_rgb,
        tf_camera_depth,
        tf_imu,
        slam,
        vision_bridge,
        mavros,
        rviz,
        rtabmap_viz,
    ])
