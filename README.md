# Autonomous Drone Navigation System using Visual SLAM

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-purple)](https://px4.io/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

## Overview

This repository contains the full software implementation of an autonomous drone navigation system using **Visual SLAM** (Simultaneous Localization and Mapping).

The system enables a quadrotor drone to:
- **Localize itself in 3D space without GPS**, using stereo visual-inertial odometry
- **Build a 3D map** of the environment in real-time using RGB-D data
- **Correct accumulated drift** through loop closure detection
- **Fuse multiple sensor streams** (camera + IMU) via an Extended Kalman Filter
- **Send localization estimates** to the PX4 flight controller for autonomous flight

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| **Frame** | HolyBro S500 V2 | Quadrotor mechanical structure |
| **Flight Controller** | Pixhawk 6C Mini (PX4) | Low-level flight control, internal EKF2 |
| **Companion Computer** | Raspberry Pi 5 | Vision processing, SLAM, ROS2 |
| **Camera** | Luxonis OAK-S2 | Stereo RGB-D + integrated IMU (BMI270) |
| **OS** | Ubuntu 24.04 LTS (aarch64) | Operating system on RPi5 |
| **Middleware** | ROS2 Jazzy Jalisco | Robotics communication framework |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Raspberry Pi 5                               │
│                                                                     │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐  │
│  │  OAK-S2      │    │  RTAB-Map        │    │ robot_locali-    │  │
│  │  depthai-ros │───►│  RGBDOdometry    │───►│ zation EKF       │  │
│  │  v3 driver   │    │  (visual front.) │    │                  │  │
│  │              │    └──────────────────┘    │ Inputs:          │  │
│  │ /rgb/image   │             │              │ /rtabmap/odom    │  │
│  │ /depth/image │             │              │ /oak/imu/data    │  │
│  │ /imu/data    │    ┌────────▼─────────┐    │                  │  │
│  └──────────────┘    │  RTAB-Map SLAM   │    │ Output:          │  │
│          │           │  (mapping back.) │    │ /odom/filtered   │  │
│          └──────────►│  loop closure    │    └────────┬─────────┘  │
│                      └──────────────────┘             │            │
│                                            ┌──────────▼─────────┐  │
│                                            │ vision_to_mavros   │  │
│                                            │ Odometry→PoseStamp │  │
│                                            └──────────┬─────────┘  │
│  ┌──────────────────────────────────────────────────  │            │
│  │                      MAVROS2                       │            │
│  │  /mavros/vision_pose/pose ◄────────────────────────┘            │
│  └────────────────────────────┬────────────────────────────────────┘
└───────────────────────────────┼─────────────────────────────────────┘
                                │ UART 921600 baud (GPIO / USB)
                 ┌──────────────▼──────────────┐
                 │    Pixhawk 6C Mini (PX4)    │
                 │                              │
                 │  EKF2 sensor fusion:         │
                 │  ✓ Vision odometry (ext.)    │
                 │  ✓ Internal IMU (ICM-42688)  │
                 │  ✓ Barometer                 │
                 │  ✓ GPS (optional)            │
                 └──────────────────────────────┘
```

## Software Stack

| Package | Version | Purpose |
|---------|---------|---------|
| [depthai-ros](https://github.com/luxonis/depthai-ros) | v3 (Jazzy branch) | OAK-S2 camera driver |
| [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros) | ros-jazzy | Visual SLAM (odometry + mapping + loop closure) |
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | ros-jazzy | Extended Kalman Filter (IMU + visual fusion) |
| [MAVROS](https://github.com/mavlink/mavros) | ros-jazzy | ROS2 ↔ MAVLink bridge for PX4 |

## Data Flow

```
OAK-S2 Camera
  ├── /oak/rgb/image_raw         (720P @ 20 fps)
  ├── /oak/stereo/image_raw      (depth map @ 20 fps)
  └── /oak/imu/data              (accel + gyro @ 200 Hz)
            │
            ▼
  RTAB-Map RGBDOdometry          ← visual frontend
  └── /rtabmap/odom              (6DOF pose @ ~15 Hz)
            │
            ├──────────────────────────────────┐
            ▼                                  ▼
  EKF Filter (/oak/imu/data)         RTAB-Map Rtabmap
  └── /odometry/filtered (50 Hz)     ├── /rtabmap/map
            │                        └── /rtabmap/cloud_map
            ▼
  vision_to_mavros.py
  └── /mavros/vision_pose/pose
            │
            ▼
  Pixhawk 6C Mini EKF2
  └── Local position estimate (PX4 internal)
```

## Repository Structure

```
drone_slam/
├── launch/
│   ├── camera.launch.py        # OAK-S2 camera driver startup
│   ├── slam.launch.py          # Full SLAM stack (EKF + RTAB-Map)
│   ├── mavros.launch.py        # MAVROS / Pixhawk communication
│   └── slam_full.launch.py     # Complete system (one command)
├── config/
│   ├── oak_s2_params.yaml      # Camera pipeline parameters
│   ├── ekf_params.yaml         # EKF fusion configuration
│   └── rtabmap_params.yaml     # RTAB-Map SLAM parameters
├── urdf/
│   └── drone.urdf.xacro        # Robot kinematic description (TF tree)
├── scripts/
│   ├── vision_to_mavros.py     # Odometry → MAVROS bridge node
│   └── check_topics.py         # Pre-flight system diagnostic tool
├── worlds/                     # Gazebo simulation worlds
├── rviz/                       # RViz2 visualization configs
├── install.sh                  # One-shot RPi5 installation script
├── CMakeLists.txt
└── package.xml
```

## Installation

### Development Machine (x86_64, for testing)

```bash
# 1. Clone the repository
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
git clone https://github.com/YOUR_USERNAME/drone_slam.git

# 2. Install ROS2 dependencies
cd ~/drone_ws
sudo apt install -y \
  ros-jazzy-rtabmap ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-slam ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-viz \
  ros-jazzy-robot-localization \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro

rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install
source install/setup.bash
```

### Raspberry Pi 5 (aarch64, production deployment)

```bash
# 1. Clone the repository
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
git clone https://github.com/YOUR_USERNAME/drone_slam.git

# 2. Run the automated installer (installs everything)
cd drone_slam
bash install.sh

# 3. Reboot to activate GPIO UART for Pixhawk
sudo reboot
```

## Usage

### One-command startup (production)

```bash
ros2 launch drone_slam slam_full.launch.py
```

### Modular startup (development / debugging)

```bash
# Terminal 1 — Camera driver
ros2 launch drone_slam camera.launch.py

# Terminal 2 — SLAM stack + EKF
ros2 launch drone_slam slam.launch.py

# Terminal 3 — Pixhawk communication
ros2 launch drone_slam mavros.launch.py

# Terminal 4 — Vision odometry bridge
ros2 run drone_slam vision_to_mavros.py
```

### Pre-flight system check

```bash
# Verifies all topics are publishing at expected frequencies
ros2 run drone_slam check_topics.py
```

### Map visualization (on ground station laptop)

```bash
ros2 launch rtabmap_viz rtabmap_viz.launch.py
```

### Localization mode (reuse saved map)

```bash
ros2 launch drone_slam slam_full.launch.py localization_mode:=true
```

## Pixhawk 6C Mini Configuration (QGroundControl)

Parameters required for visual odometry integration:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EKF2_EV_CTRL` | `15` | Enable external vision (x, y, z + yaw) |
| `EKF2_HGT_REF` | `3` | Height reference: Vision |
| `EKF2_EV_DELAY` | `0` | Estimated vision pipeline latency (ms) |
| `SER_TEL2_BAUD` | `921600` | TELEM2 UART baud rate |
| `MAV_1_CONFIG` | `TELEM2` | Companion computer port |
| `MAV_1_MODE` | `Onboard` | Companion computer stream mode |

## ⚠️ Physical Configuration Required

Before deploying, measure the **exact camera position** relative to the drone's center of mass and update `urdf/drone.urdf.xacro`:

```xml
<!-- Adjust xyz (meters) and rpy (radians) to match your physical mount -->
<origin xyz="0.05 0.0 0.02" rpy="0.0 0.0 0.0"/>
<!--          ↑x   ↑y   ↑z    ↑roll ↑pitch ↑yaw  -->
<!--      forward left  up                         -->
```

Also update the GCS IP in `launch/mavros.launch.py`:
```python
default_value='udp://@YOUR_LAPTOP_IP:14550'
```

## Key Concepts

### Visual SLAM
SLAM (Simultaneous Localization and Mapping) allows a robot to build a map of an unknown environment while simultaneously tracking its own position within that map. Visual SLAM uses camera images instead of LiDAR.

### RTAB-Map
Real-Time Appearance-Based Mapping uses a Bag-of-Words visual vocabulary for loop closure detection. When the drone revisits a previously seen area, RTAB-Map recognizes the visual signature and corrects accumulated drift.

### Extended Kalman Filter (EKF)
The EKF fuses visual odometry (low frequency, high accuracy position) with IMU data (high frequency, short-term accuracy) into a single, robust state estimate. Each sensor's uncertainty is modeled via covariance matrices.

### MAVLink / MAVROS
MAVLink is the lightweight messaging protocol used by PX4. MAVROS is the ROS2 bridge that translates between ROS2 messages and MAVLink, enabling the companion computer to send localization data to the Pixhawk.

## Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|-------------|---------|
| Camera not detected | USB 2.0 cable or port | Use the blue USB 3.0 port on RPi5 |
| `/oak/imu/data` missing | YAML config | Check `i_enable_imu: true` |
| MAVROS disconnected | Wrong port/baud | Check `fcu_url` in mavros.launch.py |
| Fast odometry drift | Motor vibrations | Add vibration dampening under camera |
| PX4 ignores vision | EKF2 misconfigured | Set `EKF2_EV_CTRL=15` in QGroundControl |
| `colcon build` OOM | RPi5 RAM | Add `--parallel-workers 1` flag |
| RTAB-Map too slow | CPU overload | Reduce camera FPS to 15 in YAML config |

## License

MIT License — see [LICENSE](LICENSE) for details.

---
