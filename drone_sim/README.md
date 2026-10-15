# Drone Sim — Gazebo Harmonic + PX4 SITL Simulation Package

This package provides the complete simulation environment for autonomous Visual SLAM with **PX4 Autopilot**, **Gazebo Harmonic**, and **ROS 2 Jazzy**.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       Gazebo Harmonic                       │
│                                                             │
│   ┌──────────────────┐               ┌──────────────────┐   │
│   │  drone_slam_     │               │  x500_depth      │   │
│   │  world.sdf       │               │  (HolyBro S500 + │   │
│   │  (3D Obstacles + │               │   OakD-Lite RGBD)│   │
│   │   Landmarks)     │               └────────┬─────────┘   │
│   └──────────────────┘                        │             │
└───────────────────────────────────────────────┼─────────────┘
                                                │ Gazebo Transport Topics
                                                │ (/camera, /depth_camera, /imu)
                                                ▼
┌─────────────────────────────────────────────────────────────┐
│                      ros_gz_bridge                          │
│                                                             │
│   /camera              ──► /oak/rgb/image_raw               │
│   /camera_info         ──► /oak/rgb/camera_info             │
│   /depth_camera        ──► /oak/stereo/image_raw            │
│   /depth_camera/info   ──► /oak/stereo/camera_info          │
│   /imu                 ──► /oak/imu/data                    │
└───────────────────────────────┬─────────────────────────────┘
                                │ ROS 2 Topics
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                 drone_slam Stack (Production)               │
│                                                             │
│   ┌──────────────────┐               ┌──────────────────┐   │
│   │ RTAB-Map SLAM    │               │ robot_locali-    │   │
│   │ (Visual Odom +   │──────────────►│ zation EKF       │   │
│   │  Loop Closure)   │               │                  │   │
│   └──────────────────┘               └────────┬─────────┘   │
│                                               │             │
│                                      ┌────────▼─────────┐   │
│                                      │ vision_to_mavros │   │
│                                      └────────┬─────────┘   │
│                                               │             │
│                                               ▼             │
│                                      MAVROS (UDP 14540)     │
└───────────────────────────────────────────────┼─────────────┘
                                                │ MAVLink UDP
                                                ▼
                               ┌──────────────────────────────┐
                               │     PX4 Autopilot SITL       │
                               │     (Software-In-The-Loop)   │
                               └──────────────────────────────┘
```

---

## Package Structure

```
drone_sim/
├── config/
│   └── ros_gz_bridge.yaml      # Gazebo Harmonic ↔ ROS 2 Jazzy topic mapping
├── worlds/
│   └── drone_slam_world.sdf    # Rich 3D environment for SLAM feature tracking
├── launch/
│   ├── bridge.launch.py        # Starts ros_gz_bridge with YAML configuration
│   ├── sim.launch.py           # Launches Gazebo world + ros_gz_bridge
│   └── sim_slam.launch.py      # Starts complete simulation pipeline (Bridge + MAVROS SITL + SLAM)
├── CMakeLists.txt
└── package.xml
```

---

## How to Run the Simulation

### Option 1: Modular Startup (Recommended for Development)

**Terminal 1 — PX4 SITL + Gazebo Harmonic:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```

**Terminal 2 — SLAM Stack + MAVROS + Bridge (Single Command):**
```bash
cd ~/drone_ws
source install/setup.bash
ros2 launch drone_sim sim_slam.launch.py
```

**Terminal 3 — Map & Point Cloud Visualization (RViz2 or RTAB-Map Viz):**
```bash
ros2 launch rtabmap_viz rtabmap_viz.launch.py
```

---

### Option 2: Standalone World Inspection (Without PX4)

To inspect the 3D world and test camera topic bridging without starting the autopilot:
```bash
ros2 launch drone_sim sim.launch.py
```
Check bridged topics:
```bash
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
```
