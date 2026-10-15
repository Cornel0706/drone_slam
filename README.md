# Autonomous Drone Navigation System with Visual SLAM

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot_v1.14+-red.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous navigation and real-time 3D mapping system for quadrotor drones operating in **GPS-denied environments**. The system utilizes stereo RGB-D Visual SLAM (RTAB-Map), extended Kalman filter state estimation (`robot_localization`), MAVROS telemetry bridging, and PX4 Autopilot for stable offboard flight control.

---

## Repository Structure

This repository is structured as a multi-package ROS 2 workspace:

```
├── drone_slam/                 # Production drone hardware stack
│   ├── config/                 # RTAB-Map, EKF & camera tuning profiles
│   ├── launch/                 # Camera, SLAM, and MAVROS launch files
│   ├── scripts/                # Vision-to-MAVROS bridge & telemetry checks
│   ├── urdf/                   # Robot model and sensor frame transforms
│   ├── CMakeLists.txt
│   └── package.xml
│
├── drone_sim/                  # Gazebo simulation & autonomous missions
│   ├── config/                 # RViz2 displays & ROS-GZ bridge mappings
│   ├── launch/                 # Simulation bringup & integrated SLAM pipeline
│   ├── scripts/                # Autonomous exploration & teleoperation scripts
│   ├── worlds/                 # 3D textured indoor world & obstacles
│   ├── CMakeLists.txt
│   └── package.xml
│
├── .gitignore
└── README.md
```

* **`drone_slam`**: Production package configured for the physical HolyBro S500 drone, Luxonis OAK-S2 stereo depth camera, and Pixhawk 6C Mini flight controller.
* **`drone_sim`**: Simulation package for testing the complete pipeline in Gazebo Harmonic with PX4 SITL and simulated RGB-D camera sensors.

---

## Architecture Overview

```
                      ┌───────────────────────────────────────┐
                      │    RGB-D Sensor / Gazebo Harmonic     │
                      │   (Color Image + Depth + Stereo IMU)  │
                      └──────────────────┬────────────────────┘
                                         │
                                         ▼
                      ┌───────────────────────────────────────┐
                      │          RTAB-Map Visual SLAM         │
                      │  - Visual Odometry (RGB-D Frontend)   │
                      │  - Loop Closure Detection & 3D Map    │
                      └──────────────────┬────────────────────┘
                                         │ /rtabmap/odom
                                         ▼
                      ┌───────────────────────────────────────┐
                      │     robot_localization (EKF Node)     │
                      │   Fused Odometry (Visual + IMU Data)  │
                      └──────────────────┬────────────────────┘
                                         │ /odometry/filtered
                                         ▼
                      ┌───────────────────────────────────────┐
                      │        vision_to_mavros Bridge        │
                      │ Converts Odometry to Vision Pose ENU  │
                      └──────────────────┬────────────────────┘
                                         │ /mavros/vision_pose/pose
                                         ▼
                      ┌───────────────────────────────────────┐
                      │       Pixhawk / PX4 EKF2 Fusion       │
                      │  Position Hold & Autonomous Offboard  │
                      └───────────────────────────────────────┘
```

---

## Quick Start / Simulation Tutorial

Follow these three steps to run the complete autonomous mapping mission in simulation.

### 1. Build the Workspace

```bash
cd ~/drone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 2. Launch Simulation (3 Terminals)

#### Terminal 1 — PX4 SITL & Gazebo World
Starts the Gazebo Harmonic physics simulator with the 3D textured indoor environment and spawns the quadrotor drone:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=drone_slam_world make px4_sitl gz_x500_depth
```

#### Terminal 2 — ROS-Gazebo Bridge, SLAM Stack & RViz
Launches the topic bridges, RTAB-Map SLAM, EKF sensor fusion, MAVROS, and opens RViz2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 launch drone_sim sim_slam.launch.py
```

> [!IMPORTANT]
> **Takeoff Step (Work in Progress):**  
> While autonomous takeoff directly from Python is currently being refined, you must manually trigger takeoff from **Terminal 1** (the PX4 SITL console) right after starting Terminal 2:
> ```bash
> commander takeoff
> ```
> Wait a few seconds until the drone completes its ascent and stabilizes in hover (~2.5 m) before proceeding to Terminal 3.

#### Terminal 3 — Autonomous Mapping Routine
Executes the automated mapping sequence (`auto_mapper.py`):
1. Detects the airborne drone at hover altitude.
2. Streams continuous zero-velocity setpoints and smoothly transitions into `OFFBOARD` mode.
3. Performs a discretized 360° panoramic scan (step-and-pause) to build feature correspondences.
4. Executes an ultra-smooth survey flight with real-time odometry quality feedback.
5. Returns to center and holds position in stable hover.

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 run drone_sim auto_mapper.py
```

> **Manual Teleoperation (Alternative to Terminal 3):**
> If you prefer manual flight control using keyboard:
> ```bash
> ros2 run drone_sim drone_teleop.py
> ```

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | Rectified RGB camera feed |
| `/oak/stereo/image_raw` | `sensor_msgs/msg/Image` | Depth image aligned to RGB |
| `/rtabmap/odom` | `nav_msgs/msg/Odometry` | Visual odometry estimate |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF fused state estimate |
| `/mavros/vision_pose/pose` | `geometry_msgs/msg/PoseStamped` | Odometry feed forwarded to PX4 EKF2 |
| `/rtabmap/grid_map` | `nav_msgs/msg/OccupancyGrid` | Generated 2D occupancy grid |
| `/rtabmap/cloud_map` | `sensor_msgs/msg/PointCloud2` | Real-time dense 3D point cloud map |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Offboard velocity setpoint commands |

---

## Hardware Deployment

For deploying on physical hardware (HolyBro S500 + Raspberry Pi 5 + Pixhawk 6C Mini + Luxonis OAK-S2):

```bash
# Run automated setup on Raspberry Pi 5
cd ~/drone_ws/src/drone_slam
./install.sh

# Launch full hardware stack
ros2 launch drone_slam slam_full.launch.py
```

---

