#!/usr/bin/env python3
"""
Node: auto_mapper.py (drone_sim)
=================================
Map-Agnostic Autonomous Drone Exploration using Real-Time SLAM & Frontier Detection.

Key Architectural Components:
------------------------------
1. Finite State Machine (FSM):
   - INIT: FCU connection, odometry health verification, parameter tuning.
   - TAKEOFF: Autonomous arming and climb to hover altitude.
   - OFFBOARD_SYNC: Dynamic altitude capture, 50 Hz setpoint pre-streaming, OFFBOARD lock.
   - PANORAMIC_SCAN: 360° discretized scan to seed initial 3D/2D occupancy grid around spawn.
   - FRONTIER_EXPLORATION: Real-time map-based frontier detection, clustering, scoring, and navigation.
   - RECOVERY: Active odometry monitoring; safe hover and micro-yaw on feature loss.
   - RETURN_TO_HOME: Autonomous return to takeoff coordinates once exploration finishes.
   - LAND: Controlled autonomous landing and disarm.
   - DONE: Mission complete.

2. Frontier Explorer (FrontierDetector):
   - Subscribes to RTAB-Map OccupancyGrid (/rtabmap/grid_map).
   - Generates obstacle inflation safety layer (scipy.ndimage.distance_transform_edt).
   - Identifies boundaries between safe free space and unknown regions.
   - Clusters contiguous frontiers (8-connectivity) and computes information gain, distance,
     and heading alignment scores.
   - Line-of-sight collision checking avoids straight-line obstacle collisions.
   - Blacklisting mechanism prevents stagnation on unreachable or occluded targets.

3. Hybrid Flight Control (PositionTarget):
   - /mavros/setpoint_raw/local with type_mask=1507 (0x05E3):
     * Controls horizontal velocities (vx, vy in ENU).
     * Rigidly locks vertical position (z) to dynamic takeoff altitude (PX4 position PID).
     * Controls yaw rate (yaw_rate in rad/s).
   - Continuous 50 Hz heartbeat timer.
"""

import math
import time
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict

import numpy as np
from scipy.ndimage import binary_dilation, label, distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from rtabmap_msgs.msg import OdomInfo
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.srv import CommandBool, CommandTOL, CommandLong, SetMode, ParamSet
from mavros_msgs.msg import State, PositionTarget


# ── FSM States ─────────────────────────────────────────────────────────────
class DroneState(Enum):
    INIT = auto()
    TAKEOFF = auto()
    OFFBOARD_SYNC = auto()
    PANORAMIC_SCAN = auto()
    FRONTIER_EXPLORATION = auto()
    RECOVERY = auto()
    RETURN_TO_HOME = auto()
    LAND = auto()
    DONE = auto()


# ── Frontier Candidate ─────────────────────────────────────────────────────
@dataclass
class FrontierCandidate:
    x: float            # Metric world X (meters)
    y: float            # Metric world Y (meters)
    size: int           # Cluster cell count (information gain metric)
    score: float = 0.0  # Overall selection score
    cluster_id: int = 0


# ── Frontier Detection & Path Safety Engine ────────────────────────────────
class FrontierDetector:
    """
    Processes 2D OccupancyGrids to extract obstacle-safe frontier clusters
    and ranks them according to information gain, travel distance, and heading alignment.
    """
    def __init__(self,
                 obstacle_inflation_m: float = 0.40,
                 min_cluster_size: int = 4,
                 max_range_m: float = 7.5,
                 min_range_m: float = 0.38):
        self.obstacle_inflation_m = obstacle_inflation_m
        self.min_cluster_size = min_cluster_size
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m

    def detect_frontiers(self,
                         grid_msg: OccupancyGrid,
                         robot_x: float,
                         robot_y: float,
                         robot_yaw: float,
                         blacklist: Dict[Tuple[float, float], float]) -> List[FrontierCandidate]:
        """
        Detects, clusters, and ranks frontier waypoints from the latest OccupancyGrid.
        """
        width = grid_msg.info.width
        height = grid_msg.info.height
        res = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        if width < 5 or height < 5 or res <= 0.0:
            return []

        # Convert to 2D numpy array: shape (height, width)
        data_2d = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))

        # 1. Classification
        unknown_mask = (data_2d == -1)
        free_mask = (data_2d >= 0) & (data_2d < 25)
        occupied_mask = (data_2d >= 50)

        if not np.any(free_mask) or not np.any(unknown_mask):
            return []

        # 2. Obstacle inflation (Safety clearance)
        if np.any(occupied_mask):
            dist_to_obs = distance_transform_edt(~occupied_mask) * res
            safe_free_mask = free_mask & (dist_to_obs >= self.obstacle_inflation_m)
        else:
            dist_to_obs = np.full((height, width), 999.0)
            safe_free_mask = free_mask

        # 3. Frontier cell detection: safe free cells adjacent to unknown cells
        unknown_adjacent = binary_dilation(unknown_mask, structure=np.ones((3, 3)))
        raw_frontiers = safe_free_mask & unknown_adjacent

        if not np.any(raw_frontiers):
            return []

        # 4. Connected-component clustering (8-connectivity)
        labeled_frontiers, num_features = label(raw_frontiers, structure=np.ones((3, 3)))
        if num_features == 0:
            return []

        now = time.time()
        candidates: List[FrontierCandidate] = []

        for cid in range(1, num_features + 1):
            cluster_mask = (labeled_frontiers == cid)
            cluster_size = int(np.sum(cluster_mask))
            if cluster_size < self.min_cluster_size:
                continue

            ys, xs = np.where(cluster_mask)
            # Choose cell within cluster having greatest clearance from obstacles
            best_idx = int(np.argmax(dist_to_obs[ys, xs]))
            cy, cx = ys[best_idx], xs[best_idx]

            wx = float(origin_x + (cx + 0.5) * res)
            wy = float(origin_y + (cy + 0.5) * res)

            # Euclidean distance to robot
            dist = math.hypot(wx - robot_x, wy - robot_y)
            if dist < self.min_range_m or dist > self.max_range_m:
                continue

            # Check blacklist (cooldown for unreachable/stagnant targets)
            is_blacklisted = False
            for (bx, by), expire_time in list(blacklist.items()):
                if now < expire_time:
                    if math.hypot(wx - bx, wy - by) < 0.5:
                        is_blacklisted = True
                        break
                else:
                    blacklist.pop((bx, by), None)

            if is_blacklisted:
                continue

            # Line of sight check: reject if line directly hits an occupied obstacle
            if not self.is_path_clear(data_2d, origin_x, origin_y, res, robot_x, robot_y, wx, wy):
                continue

            # Heading alignment (camera points forward)
            bearing = math.atan2(wy - robot_y, wx - robot_x)
            heading_err = abs(math.atan2(math.sin(bearing - robot_yaw), math.cos(bearing - robot_yaw)))

            # Frontier utility score:
            # High information gain (size), close distance, small heading deviation
            score = (cluster_size ** 0.8) / (1.0 + 0.7 * dist + 0.3 * heading_err)

            candidates.append(FrontierCandidate(
                x=wx,
                y=wy,
                size=cluster_size,
                score=score,
                cluster_id=cid
            ))

        # Sort descending by score
        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates

    @staticmethod
    def is_path_clear(grid: np.ndarray, origin_x: float, origin_y: float, res: float,
                      x1: float, y1: float, x2: float, y2: float) -> bool:
        """Checks if straight ray between (x1,y1) and (x2,y2) is obstacle-free."""
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / (res * 0.5)), 2)
        xs = np.linspace(x1, x2, steps)
        ys = np.linspace(y1, y2, steps)
        c_indices = np.floor((xs - origin_x) / res).astype(int)
        r_indices = np.floor((ys - origin_y) / res).astype(int)

        H, W = grid.shape
        valid = (c_indices >= 0) & (c_indices < W) & (r_indices >= 0) & (r_indices < H)
        if not np.all(valid):
            return False

        # Any cell with probability >= 50 is an obstacle
        return not np.any(grid[r_indices, c_indices] >= 50)


# ── Main Autonomous Node ───────────────────────────────────────────────────
class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        # Simulation Time Configuration
        if self.has_parameter('use_sim_time'):
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        else:
            self.declare_parameter('use_sim_time', True)

        # Operational Parameters
        self.cruising_speed = 0.18        # m/s (safe speed to eliminate motion blur in RTAB-Map)
        self.target_altitude = 1.5        # Cruising altitude (dynamically updated on takeoff)
        self.max_exploration_time = 360.0  # Max exploration duration before RTH (seconds)
        self.max_frontier_duration = 22.0  # Stagnation timeout on a single frontier (seconds)

        # Instantiate Frontier Detector
        self.frontier_detector = FrontierDetector(
            obstacle_inflation_m=0.40,
            min_cluster_size=4,
            max_range_m=7.5,
            min_range_m=0.38
        )

        # ── Publishers ─────────────────────────────────────────────
        # 1. Primary Hybrid PositionTarget (Horizontal Velocity + Locked Z Position + Yaw Rate)
        self.raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # 2. Legacy Unstamped Twist
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # 3. Canonical Stamped Twist
        self.vel_stamped_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        # 4. RViz Markers for Live Frontier Visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/exploration/frontiers',
            10
        )
        self.target_marker_pub = self.create_publisher(
            Marker,
            '/exploration/active_target',
            10
        )

        # ── Subscribers ────────────────────────────────────────────
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_sensor
        )

        self.odom_info_sub = self.create_subscription(
            OdomInfo,
            '/rtabmap/odom_info',
            self.odom_info_cb,
            qos_sensor
        )

        self.rtab_odom_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.rtab_odom_cb,
            qos_sensor
        )

        # Map Subscriptions (Reliable + Transient Local QoS for RTAB-Map)
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/grid_map',
            self.grid_map_cb,
            qos_map
        )
        self.fallback_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.grid_map_cb,
            qos_map
        )

        # ── Service Clients ────────────────────────────────────────
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.cmd_long_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(ParamSet, '/mavros/param/set')

        # ── State Machine & Tracking Variables ─────────────────────
        self.state = DroneState.INIT
        self.pre_recovery_state = DroneState.FRONTIER_EXPLORATION
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.latest_grid: Optional[OccupancyGrid] = None

        # Takeoff / Home coordinates
        self.home_pose: Tuple[float, float] = (0.0, 0.0)

        # Exploration tracking
        self.active_frontier: Optional[FrontierCandidate] = None
        self.frontier_start_time = 0.0
        self.blacklist: Dict[Tuple[float, float], float] = {}
        self.mission_start_time = 0.0
        self.consecutive_empty_frontiers = 0

        # Odometry Health Monitoring
        self.odom_received = False
        self.odom_lost = False
        self.odom_inliers = 0
        self.odom_matches = 0
        self.odom_has_nan = False
        self.last_odom_time = 0.0

        # Dedicated MAVROS PositionTarget Masks:
        # 1. Yaw Rate Control (Used during forward navigation):
        # Controls: PZ, VX, VY, YAW_RATE. Ignores: PX, PY, VZ, AFX, AFY, AFZ, YAW.
        self.MASK_YAW_RATE = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )  # Value: 1507 (0x05E3)

        # 2. Yaw Position Control (Used during Panoramic Scan & Discrete Yaw Steps):
        # Controls: PZ, VX, VY, YAW. Ignores: PX, PY, VZ, AFX, AFY, AFZ, YAW_RATE.
        self.MASK_YAW_POSITION = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )  # Value: 2531 (0x09E3)

        # Setpoint message templates
        self.cmd_twist = Twist()
        self.cmd_raw = PositionTarget()
        self.cmd_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd_raw.type_mask = self.MASK_YAW_RATE
        self.cmd_raw.position.x = 0.0
        self.cmd_raw.position.y = 0.0
        self.cmd_raw.position.z = float(self.target_altitude)
        self.cmd_raw.velocity.x = 0.0
        self.cmd_raw.velocity.y = 0.0
        self.cmd_raw.velocity.z = 0.0
        self.cmd_raw.yaw = 0.0
        self.cmd_raw.yaw_rate = 0.0

        # 50 Hz Heartbeat streaming timer
        self.timer = self.create_timer(0.02, self.timer_cb)

        self.get_logger().info(
            '>>> AutoMapper (Frontier Exploration) Node Initialized. '
            'Connecting to FCU and awaiting SLAM occupancy grid...'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────
    def state_cb(self, msg: State):
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def odom_info_cb(self, msg: OdomInfo):
        self.odom_received = True
        self.last_odom_time = time.time()
        self.odom_lost = bool(msg.lost)
        self.odom_inliers = int(msg.inliers)
        self.odom_matches = int(msg.matches)

    def rtab_odom_cb(self, msg: Odometry):
        self.odom_received = True
        self.last_odom_time = time.time()
        p = msg.pose.pose.position
        self.odom_has_nan = not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z))

    def grid_map_cb(self, msg: OccupancyGrid):
        self.latest_grid = msg

    # ── Odometry Health Checker ────────────────────────────────────────────
    def is_odometry_healthy(self) -> bool:
        """Checks visual odometry tracking quality (bypassed on ground to allow liftoff)."""
        alt = self.current_pose.pose.position.z
        if alt < 1.0:
            return True

        if not self.odom_received:
            return True

        if time.time() - self.last_odom_time > 1.2:
            return False

        if self.odom_lost:
            return False

        if self.odom_inliers < 8:
            return False

        if self.odom_has_nan:
            return False

        return True

    def get_current_yaw(self) -> float:
        """Extracts yaw angle (radians) from FCU orientation quaternion."""
        q = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        if not (math.isfinite(siny_cosp) and math.isfinite(cosy_cosp)):
            return 0.0
        return math.atan2(siny_cosp, cosy_cosp)

    # ── 50 Hz Heartbeat Streaming ──────────────────────────────────────────
    def timer_cb(self):
        """Dispatches continuous 50 Hz hybrid setpoints to satisfy PX4 watchdog."""
        now = self.get_clock().now().to_msg()

        # 1. Primary Hybrid PositionTarget
        self.cmd_raw.header.stamp = now
        self.cmd_raw.header.frame_id = 'odom'
        self.raw_pub.publish(self.cmd_raw)

        # 2. Unstamped Twist
        self.vel_pub.publish(self.cmd_twist)

        # 3. Stamped Twist
        stamped = TwistStamped()
        stamped.header.stamp = now
        stamped.header.frame_id = 'base_link'
        stamped.twist = self.cmd_twist
        self.vel_stamped_pub.publish(stamped)

    @staticmethod
    def wrap_pi(angle: float) -> float:
        """Wraps any angle into the [-pi, pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def set_vel(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, target_z=None):
        """Sets linear and angular velocity commands with locked altitude."""
        self.cmd_twist.linear.x = float(vx)
        self.cmd_twist.linear.y = float(vy)
        self.cmd_twist.linear.z = float(vz)
        self.cmd_twist.angular.z = float(yaw_rate)

        if target_z is not None:
            self.target_altitude = float(target_z)

        self.cmd_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd_raw.type_mask = self.MASK_YAW_RATE
        self.cmd_raw.position.x = 0.0
        self.cmd_raw.position.y = 0.0
        self.cmd_raw.position.z = float(self.target_altitude)
        self.cmd_raw.velocity.x = float(vx)
        self.cmd_raw.velocity.y = float(vy)
        self.cmd_raw.velocity.z = 0.0
        self.cmd_raw.yaw = 0.0
        self.cmd_raw.yaw_rate = float(yaw_rate)

    def set_yaw_position_setpoint(self, target_yaw: float, target_z: Optional[float] = None):
        """
        Configures PositionTarget for Yaw Position Control (used in static rotation steps):
        - type_mask = 2531 (IGNORE_YAW = 0, IGNORE_YAW_RATE = 2048)
        - position.z = self.target_altitude (rigid closed-loop altitude hold)
        - yaw = target_yaw wrapped in [-pi, pi]
        - velocity x, y, z = 0.0
        """
        if target_z is not None:
            self.target_altitude = float(target_z)

        wrapped_yaw = float(self.wrap_pi(target_yaw))

        self.cmd_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd_raw.type_mask = self.MASK_YAW_POSITION
        self.cmd_raw.position.x = 0.0
        self.cmd_raw.position.y = 0.0
        self.cmd_raw.position.z = float(self.target_altitude)
        self.cmd_raw.velocity.x = 0.0
        self.cmd_raw.velocity.y = 0.0
        self.cmd_raw.velocity.z = 0.0
        self.cmd_raw.yaw = wrapped_yaw
        self.cmd_raw.yaw_rate = 0.0

        self.cmd_twist.linear.x = 0.0
        self.cmd_twist.linear.y = 0.0
        self.cmd_twist.linear.z = 0.0
        self.cmd_twist.angular.z = 0.0

    def publish_zero_setpoint(self, target_z=None):
        """Halts all horizontal and rotational motion while maintaining altitude lock."""
        self.set_vel(0.0, 0.0, 0.0, 0.0, target_z=target_z)
        self.timer_cb()

    def sleep_and_stream(self, duration_sec: float):
        """Maintains 50 Hz setpoint streaming without blocking ROS executor."""
        end_time = time.time() + duration_sec
        while rclpy.ok() and time.time() < end_time:
            self.timer_cb()
            remaining = max(0.001, min(0.02, end_time - time.time()))
            rclpy.spin_once(self, timeout_sec=remaining)

    # ── RViz Visualization Markers ─────────────────────────────────────────
    def publish_frontier_markers(self, candidates: List[FrontierCandidate], active: Optional[FrontierCandidate]):
        """Publishes detected frontiers and active target waypoint to RViz."""
        now = self.get_clock().now().to_msg()

        # 1. Cluster Markers
        marker_array = MarkerArray()
        # Delete old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        for i, c in enumerate(candidates):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c.x
            m.pose.position.y = c.y
            m.pose.position.z = self.target_altitude
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            # Cyan color for frontiers
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.8
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

        # 2. Active Target Marker
        if active is not None:
            tm = Marker()
            tm.header.frame_id = 'map'
            tm.header.stamp = now
            tm.ns = 'active_target'
            tm.id = 999
            tm.type = Marker.CYLINDER
            tm.action = Marker.ADD
            tm.pose.position.x = active.x
            tm.pose.position.y = active.y
            tm.pose.position.z = self.target_altitude
            tm.scale.x = 0.35
            tm.scale.y = 0.35
            tm.scale.z = 0.60
            # Gold / Yellow-Orange color for active target
            tm.color.r = 1.0
            tm.color.g = 0.65
            tm.color.b = 0.0
            tm.color.a = 0.95
            self.target_marker_pub.publish(tm)

    # ── Takeoff & Mode Switching Routines ──────────────────────────────────
    def perform_takeoff(self, target_alt: float = 1.5) -> bool:
        """Executes armored automated takeoff and dynamic altitude synchronization."""
        alt = self.current_pose.pose.position.z
        if alt >= 1.0 and self.current_state.armed:
            self.target_altitude = float(alt)
            self.cmd_raw.position.z = float(self.target_altitude)
            self.get_logger().info(
                f'Vehicle already airborne at {alt:.2f}m. '
                f'Dynamically adopting target_altitude = {self.target_altitude:.2f}m.'
            )
            return True

        self.get_logger().info(f'Initiating automatic takeoff to {target_alt:.1f}m...')
        self.target_altitude = float(target_alt)
        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(1.0)

        for attempt in range(1, 6):
            alt = self.current_pose.pose.position.z
            if alt >= 1.2:
                self.get_logger().info(f'Takeoff detected! Alt: {alt:.2f}m')
                break

            self.get_logger().info(f'Takeoff attempt {attempt}/5...')

            # A. Send MAV_CMD_NAV_TAKEOFF (CommandLong 22)
            if self.cmd_long_client.service_is_ready():
                cl_req = CommandLong.Request()
                cl_req.broadcast = False
                cl_req.command = 22  # MAV_CMD_NAV_TAKEOFF
                cl_req.confirmation = 0
                cl_req.param1 = 0.0
                cl_req.param4 = float('nan')
                cl_req.param5 = float('nan')
                cl_req.param6 = float('nan')
                cl_req.param7 = float('nan')
                self.cmd_long_client.call_async(cl_req)

            # B. Send CommandTOL
            if self.takeoff_client.service_is_ready():
                t_req = CommandTOL.Request()
                t_req.min_pitch = 0.0
                t_req.yaw = float('nan')
                t_req.latitude = float('nan')
                t_req.longitude = float('nan')
                t_req.altitude = float(target_alt)
                self.takeoff_client.call_async(t_req)

            # C. Request AUTO.TAKEOFF
            if self.mode_client.service_is_ready():
                m_req = SetMode.Request()
                m_req.custom_mode = 'AUTO.TAKEOFF'
                self.mode_client.call_async(m_req)

            # D. Arming
            if not self.current_state.armed and self.arming_client.service_is_ready():
                a_req = CommandBool.Request()
                a_req.value = True
                self.arming_client.call_async(a_req)

            self.sleep_and_stream(1.0)

        # Monitor climb
        t_start = time.time()
        step = 0
        while rclpy.ok() and (time.time() - t_start < 15.0):
            alt = self.current_pose.pose.position.z
            step += 1
            if step % 5 == 0:
                self.get_logger().info(f'Climbing... Alt: {alt:.2f}m / Target: {target_alt:.1f}m')
            if alt >= 1.3:
                self.get_logger().info(f'Target altitude reached ({alt:.2f}m)!')
                break
            self.sleep_and_stream(0.2)

        # Settle and dynamically adopt the actual reached takeoff altitude
        self.sleep_and_stream(1.5)
        current_alt = self.current_pose.pose.position.z
        if current_alt >= 0.8:
            self.target_altitude = float(current_alt)
            self.cmd_raw.position.z = float(self.target_altitude)
            self.get_logger().info(f'Takeoff settled. Dynamically set target_altitude = {self.target_altitude:.2f}m')
        else:
            self.get_logger().warn(
                f'Takeoff altitude low ({current_alt:.2f}m). '
                'Tip: enter "commander takeoff" in PX4 shell if FCU requires manual authorization.'
            )
            return False

        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(1.0)
        return True

    def switch_to_offboard(self, pre_stream_sec: float = 2.0, max_wait_sec: float = 8.0) -> bool:
        """Transitions cleanly to OFFBOARD mode with dynamic altitude lock."""
        actual_z = self.current_pose.pose.position.z
        if actual_z >= 0.8:
            self.target_altitude = float(actual_z)
            self.cmd_raw.position.z = float(self.target_altitude)

        self.get_logger().info(
            f'>>> Step 1/3: Pre-streaming hybrid setpoints for {pre_stream_sec:.1f}s '
            f'(vx=0, vy=0, locked z={self.target_altitude:.2f}m)...'
        )
        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(pre_stream_sec)

        if self.current_state.mode == 'OFFBOARD':
            actual_alt = self.current_pose.pose.position.z
            if actual_alt >= 0.8:
                self.target_altitude = float(actual_alt)
                self.cmd_raw.position.z = float(self.target_altitude)
            self.get_logger().info(f'Vehicle already in OFFBOARD mode at altitude {self.target_altitude:.2f}m.')
            return True

        self.get_logger().info('>>> Step 2/3: Sending OFFBOARD mode switch request...')
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        if self.mode_client.service_is_ready():
            self.mode_client.call_async(req)

        t_start = time.time()
        last_req_time = time.time()

        while rclpy.ok() and (time.time() - t_start < max_wait_sec):
            self.publish_zero_setpoint(target_z=self.target_altitude)
            rclpy.spin_once(self, timeout_sec=0.02)

            if self.current_state.mode == 'OFFBOARD':
                confirmed_alt = self.current_pose.pose.position.z
                if confirmed_alt >= 0.8:
                    self.target_altitude = float(confirmed_alt)
                    self.cmd_raw.position.z = float(self.target_altitude)
                self.get_logger().info(
                    f'>>> OFFBOARD Mode Confirmed! Captured Current Altitude: {self.current_pose.pose.position.z:.2f}m '
                    f'as permanent locked target_altitude!'
                )
                self.publish_zero_setpoint(target_z=self.target_altitude)
                break

            if time.time() - last_req_time >= 1.0:
                if self.mode_client.service_is_ready():
                    self.mode_client.call_async(req)
                last_req_time = time.time()

        if self.current_state.mode != 'OFFBOARD':
            self.get_logger().error(f'Failed to switch to OFFBOARD mode! Current mode: {self.current_state.mode}')
            return False

        self.get_logger().info(
            f'>>> Step 3/3: Holding stable OFFBOARD hover for 2.0s at locked z={self.target_altitude:.2f}m...'
        )
        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(2.0)
        return True

    # ── Recovery Behavior on Tracking Degradation ──────────────────────────
    def handle_recovery(self):
        """Safely recovers visual tracking without erratic flight movements."""
        self.get_logger().warn(
            f'[RECOVERY] Visual tracking degraded (lost={self.odom_lost}, inliers={self.odom_inliers})! '
            'Halting into hover...'
        )
        self.publish_zero_setpoint(target_z=self.target_altitude)

        # 1. Settle in hover for up to 3.0 seconds
        t_start = time.time()
        recovered = False
        while rclpy.ok() and time.time() - t_start < 3.0:
            self.sleep_and_stream(0.05)
            if self.is_odometry_healthy():
                recovered = True
                break

        # 2. If still lost, perform gentle micro-yaw in place (+0.12 rad/s)
        if not recovered:
            self.get_logger().info(
                '[RECOVERY] Odometry still lost. Performing gentle micro-yaw in place to re-acquire visual keyframes...'
            )
            yaw_start = time.time()
            while rclpy.ok() and time.time() - yaw_start < 4.0:
                self.set_vel(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.12, target_z=self.target_altitude)
                self.sleep_and_stream(0.05)
                if self.is_odometry_healthy():
                    self.get_logger().info('[RECOVERY] Visual landmarks re-acquired!')
                    break

        # Settle in hover
        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(1.0)
        self.get_logger().info('[RECOVERY] Resuming exploration.')

    # ── Frontier Navigation Controller ─────────────────────────────────────
    def navigate_towards(self, target_x: float, target_y: float) -> Tuple[bool, float]:
        """
        Commands proportional velocity towards (target_x, target_y) while turning
        the forward-facing camera to face the frontier.
        Returns: (arrived: bool, remaining_distance: float)
        """
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        robot_yaw = self.get_current_yaw()

        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)

        # Arrival threshold
        if dist < 0.38:
            return True, dist

        # Heading calculation
        target_bearing = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_bearing - robot_yaw), math.cos(target_bearing - robot_yaw))

        # Proportional yaw rate control (clamped between -0.22 and 0.22 rad/s)
        yaw_rate = float(np.clip(1.0 * yaw_err, -0.22, 0.22))

        # Velocity scaling: if pointing away (>50°), prioritize turning before translating
        if abs(yaw_err) > math.radians(50):
            speed = 0.04
        else:
            speed = float(np.clip(0.40 * dist, 0.06, self.cruising_speed))

        # Velocity vector in local ENU frame
        vx_enu = speed * math.cos(target_bearing)
        vy_enu = speed * math.sin(target_bearing)

        # Dispatch hybrid command
        self.set_vel(vx=vx_enu, vy=vy_enu, vz=0.0, yaw_rate=yaw_rate, target_z=self.target_altitude)
        return False, dist

    # ── Non-Blocking Panoramic Scan ────────────────────────────────────────
    def execute_panoramic_scan(self, total_steps: int = 12, pause_duration: float = 0.8, rot_timeout: float = 2.0):
        """
        Executes a non-blocking, asynchronous 360° step-and-settle scan:
        1. Non-blocking: Completely eliminates time.sleep(). timer_cb publishes
           uninterrupted at 50 Hz on /mavros/setpoint_raw/local during rotation and settle pauses.
        2. Dedicated type_mask for Yaw Position:
           - IGNORE_YAW = 0 (active yaw position control)
           - IGNORE_YAW_RATE = 2048 (yaw rate ignored)
           - IGNORE_PZ = 0 (active Z closed-loop position PID hold)
           - IGNORE_VX = 0, IGNORE_VY = 0 (velocity locked to 0.0)
           - type_mask = 2531 (0x09E3)
        3. cmd_raw.yaw is computed in radians in [-pi, pi] using wrap_pi.
        4. cmd_raw.position.z is permanently synchronized to self.target_altitude.
        """
        self.state = DroneState.PANORAMIC_SCAN
        self.get_logger().info(
            f'=== [FSM: PANORAMIC_SCAN] Executing Non-Blocking 360° Yaw-Position Scan '
            f'({total_steps} steps of 30°, locked z={self.target_altitude:.2f}m) ==='
        )

        start_yaw = self.get_current_yaw()
        step_angle = (2.0 * math.pi) / total_steps

        for step in range(1, total_steps + 1):
            # 1. Target yaw in radians strictly wrapped to [-pi, pi]
            target_yaw = self.wrap_pi(start_yaw + step * step_angle)
            target_deg = math.degrees(target_yaw)

            # 2. Configure dedicated Yaw Position type_mask (2531) with locked Z altitude and 0 velocity
            self.set_yaw_position_setpoint(target_yaw=target_yaw, target_z=self.target_altitude)

            # ── Phase A: Non-blocking Rotation Step ────────────────────
            # Continually publish at 50 Hz while FCU rotates towards target_yaw
            t_rot_start = time.time()
            while rclpy.ok() and (time.time() - t_rot_start < rot_timeout):
                if not self.is_odometry_healthy():
                    self.handle_recovery()
                    # Re-enforce yaw position target upon recovery
                    self.set_yaw_position_setpoint(target_yaw=target_yaw, target_z=self.target_altitude)

                # Maintain 50 Hz publishing without time.sleep()
                self.timer_cb()
                rclpy.spin_once(self, timeout_sec=0.02)

                # Check if FCU has reached target yaw within tolerance
                yaw_err = abs(self.wrap_pi(target_yaw - self.get_current_yaw()))
                if yaw_err < math.radians(3.5):
                    break

            # ── Phase B: Non-blocking Settle Pause ──────────────────────
            # Hold exact target yaw position and locked altitude for pause_duration
            t_settle_start = time.time()
            while rclpy.ok() and (time.time() - t_settle_start < pause_duration):
                if not self.is_odometry_healthy():
                    self.handle_recovery()
                    self.set_yaw_position_setpoint(target_yaw=target_yaw, target_z=self.target_altitude)

                # Continuous 50 Hz setpoint publication without time.sleep()
                self.timer_cb()
                rclpy.spin_once(self, timeout_sec=0.02)

            self.get_logger().info(
                f'Panoramic Scan: Step {step}/{total_steps} settled at {target_deg:.1f}° '
                f'(alt: {self.current_pose.pose.position.z:.2f}m / target: {self.target_altitude:.2f}m)'
            )

        self.get_logger().info(
            f'>>> Panoramic Map Seeding Complete! Holding steady hover at locked z={self.target_altitude:.2f}m...'
        )
        t_final = time.time()
        while rclpy.ok() and (time.time() - t_final < 1.5):
            self.timer_cb()
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── Autonomous Landing Routine ─────────────────────────────────────────
    def execute_landing(self):
        """Safely commands autonomous descent and monitors touchdown."""
        self.get_logger().info('>>> Initiating safe autonomous landing...')
        self.publish_zero_setpoint(target_z=self.target_altitude)
        self.sleep_and_stream(1.5)

        # Request AUTO.LAND mode
        if self.mode_client.service_is_ready():
            m_req = SetMode.Request()
            m_req.custom_mode = 'AUTO.LAND'
            self.mode_client.call_async(m_req)

        # Also trigger land service as backup
        if self.land_client.service_is_ready():
            l_req = CommandTOL.Request()
            l_req.min_pitch = 0.0
            l_req.yaw = float('nan')
            l_req.latitude = float('nan')
            l_req.longitude = float('nan')
            l_req.altitude = 0.0
            self.land_client.call_async(l_req)

        t_land = time.time()
        while rclpy.ok() and (time.time() - t_land < 25.0):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            alt = self.current_pose.pose.position.z
            if alt < 0.25 or not self.current_state.armed:
                self.get_logger().info(f'Touchdown confirmed at {alt:.2f}m. Vehicle disarmed.')
                break

    # ── Main FSM Mission Loop ──────────────────────────────────────────────
    def run_exploration_mission(self):
        """Central Finite State Machine overseeing the entire autonomous routine."""
        # 1. State: INIT
        self.state = DroneState.INIT
        self.get_logger().info('=== [FSM: INIT] Waiting for FCU Connection ===')
        while rclpy.ok() and not self.current_state.connected:
            self.get_logger().info('Connecting to FCU...')
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)

        self.get_logger().info('FCU Connected! Waiting for services...')
        self.mode_client.wait_for_service(timeout_sec=3.0)
        self.arming_client.wait_for_service(timeout_sec=3.0)
        self.takeoff_client.wait_for_service(timeout_sec=3.0)
        self.land_client.wait_for_service(timeout_sec=3.0)
        self.cmd_long_client.wait_for_service(timeout_sec=3.0)

        if self.param_set_client.wait_for_service(timeout_sec=2.0):
            p_req = ParamSet.Request()
            p_req.param_id = 'COM_ARM_EKF_YAW'
            p_req.value.real = 5.0
            self.param_set_client.call_async(p_req)

        self.sleep_and_stream(0.5)

        # 2. State: TAKEOFF
        self.state = DroneState.TAKEOFF
        self.get_logger().info('=== [FSM: TAKEOFF] Initiating Autonomous Liftoff ===')
        takeoff_ok = self.perform_takeoff(target_alt=self.target_altitude)
        if not takeoff_ok:
            if self.current_pose.pose.position.z >= 1.0:
                self.get_logger().info('Manual takeoff detected! Proceeding.')
            else:
                self.get_logger().error('Could not achieve takeoff. Aborting.')
                return

        # 3. State: OFFBOARD_SYNC
        self.state = DroneState.OFFBOARD_SYNC
        self.get_logger().info('=== [FSM: OFFBOARD_SYNC] Synchronizing Dynamic Altitude & Engaging OFFBOARD ===')
        if not self.switch_to_offboard(pre_stream_sec=2.0):
            self.get_logger().error('Could not engage OFFBOARD mode. Aborting.')
            return

        # Record Home Pose for RTH
        self.home_pose = (
            float(self.current_pose.pose.position.x),
            float(self.current_pose.pose.position.y)
        )
        self.get_logger().info(
            f'Home coordinates locked: X={self.home_pose[0]:.2f}, Y={self.home_pose[1]:.2f}, '
            f'Altitude={self.target_altitude:.2f}m'
        )

        # 4. State: PANORAMIC_SCAN
        self.execute_panoramic_scan(total_steps=12, pause_duration=0.8, rot_timeout=2.0)

        # Wait for the first valid occupancy grid from RTAB-Map
        self.get_logger().info('Waiting for initial RTAB-Map occupancy grid...')
        t_map_wait = time.time()
        while rclpy.ok() and self.latest_grid is None and (time.time() - t_map_wait < 10.0):
            self.sleep_and_stream(0.2)

        if self.latest_grid is not None:
            self.get_logger().info(
                f'Occupancy Grid received ({self.latest_grid.info.width}x{self.latest_grid.info.height} @ '
                f'{self.latest_grid.info.resolution:.2f}m/cell). Starting Frontier Exploration!'
            )
        else:
            self.get_logger().warn('No OccupancyGrid received yet. Exploration will proceed as soon as map arrives.')

        # 5. State: FRONTIER_EXPLORATION
        self.state = DroneState.FRONTIER_EXPLORATION
        self.get_logger().info('=== [FSM: FRONTIER_EXPLORATION] Entering Dynamic Map-Agnostic Exploration ===')
        self.mission_start_time = time.time()

        while rclpy.ok():
            # Check maximum mission duration
            elapsed_mission = time.time() - self.mission_start_time
            if elapsed_mission > self.max_exploration_time:
                self.get_logger().info(
                    f'>>> Max exploration time reached ({elapsed_mission:.1f}s / {self.max_exploration_time:.1f}s). '
                    'Transitioning to Return-To-Home.'
                )
                break

            # Active Odometry Quality Guard
            if not self.is_odometry_healthy():
                self.handle_recovery()
                continue

            # Need valid occupancy grid to plan
            if self.latest_grid is None:
                self.publish_zero_setpoint(target_z=self.target_altitude)
                self.sleep_and_stream(0.2)
                continue

            robot_x = self.current_pose.pose.position.x
            robot_y = self.current_pose.pose.position.y
            robot_yaw = self.get_current_yaw()

            # If no active frontier, extract candidates and choose best
            if self.active_frontier is None:
                candidates = self.frontier_detector.detect_frontiers(
                    grid_msg=self.latest_grid,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    blacklist=self.blacklist
                )

                self.publish_frontier_markers(candidates, active=None)

                if candidates:
                    self.active_frontier = candidates[0]
                    self.frontier_start_time = time.time()
                    self.consecutive_empty_frontiers = 0
                    self.get_logger().info(
                        f'-> Selected Frontier: (X: {self.active_frontier.x:.2f}, Y: {self.active_frontier.y:.2f}) '
                        f'| Cluster Size: {self.active_frontier.size} cells | Score: {self.active_frontier.score:.1f}'
                    )
                    self.publish_frontier_markers(candidates, active=self.active_frontier)
                else:
                    self.consecutive_empty_frontiers += 1
                    if self.consecutive_empty_frontiers <= 2:
                        self.get_logger().info(
                            f'No frontiers found in current view ({self.consecutive_empty_frontiers}/2). '
                            'Performing 90° exploratory pivot...'
                        )
                        # Perform small exploratory pivot in place
                        t_piv = time.time()
                        while rclpy.ok() and time.time() - t_piv < 2.0:
                            if not self.is_odometry_healthy():
                                self.handle_recovery()
                            self.set_vel(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.22, target_z=self.target_altitude)
                            self.sleep_and_stream(0.02)
                        self.publish_zero_setpoint(target_z=self.target_altitude)
                        self.sleep_and_stream(1.0)
                        continue
                    else:
                        self.get_logger().info(
                            '>>> No reachable frontiers remaining! Entire environment mapped. '
                            'Transitioning to Return-To-Home.'
                        )
                        break

            # If active frontier is set, navigate towards it
            if self.active_frontier is not None:
                # 1. Stagnation timeout check
                if time.time() - self.frontier_start_time > self.max_frontier_duration:
                    self.get_logger().warn(
                        f'Frontier at ({self.active_frontier.x:.2f}, {self.active_frontier.y:.2f}) timed out '
                        '(unreachable or blocked). Blacklisting for 60s.'
                    )
                    b_key = (round(self.active_frontier.x, 1), round(self.active_frontier.y, 1))
                    self.blacklist[b_key] = time.time() + 60.0
                    self.active_frontier = None
                    self.publish_zero_setpoint(target_z=self.target_altitude)
                    self.sleep_and_stream(1.0)
                    continue

                # 2. Dynamic path collision check
                data_2d = np.array(self.latest_grid.data, dtype=np.int8).reshape(
                    (self.latest_grid.info.height, self.latest_grid.info.width)
                )
                path_clear = self.frontier_detector.is_path_clear(
                    grid=data_2d,
                    origin_x=self.latest_grid.info.origin.position.x,
                    origin_y=self.latest_grid.info.origin.position.y,
                    res=self.latest_grid.info.resolution,
                    x1=robot_x, y1=robot_y,
                    x2=self.active_frontier.x, y2=self.active_frontier.y
                )

                if not path_clear:
                    self.get_logger().warn(
                        f'Direct path to frontier ({self.active_frontier.x:.2f}, {self.active_frontier.y:.2f}) '
                        'now blocked by obstacle! Blacklisting for 45s.'
                    )
                    b_key = (round(self.active_frontier.x, 1), round(self.active_frontier.y, 1))
                    self.blacklist[b_key] = time.time() + 45.0
                    self.active_frontier = None
                    self.publish_zero_setpoint(target_z=self.target_altitude)
                    self.sleep_and_stream(1.0)
                    continue

                # 3. Step motion towards frontier
                arrived, dist = self.navigate_towards(self.active_frontier.x, self.active_frontier.y)

                if arrived:
                    self.get_logger().info(
                        f'>>> Frontier at ({self.active_frontier.x:.2f}, {self.active_frontier.y:.2f}) reached! '
                        'Pausing 1.5s in hover to integrate new SLAM keyframes...'
                    )
                    self.publish_zero_setpoint(target_z=self.target_altitude)
                    self.sleep_and_stream(1.5)
                    self.active_frontier = None
                    self.consecutive_empty_frontiers = 0
                else:
                    self.sleep_and_stream(0.04)

        # 6. State: RETURN_TO_HOME
        self.state = DroneState.RETURN_TO_HOME
        self.get_logger().info('=== [FSM: RETURN_TO_HOME] Returning to Takeoff Position ===')
        home_x, home_y = self.home_pose

        t_rth_start = time.time()
        while rclpy.ok() and (time.time() - t_rth_start < 45.0):
            if not self.is_odometry_healthy():
                self.handle_recovery()
                continue

            arrived, dist = self.navigate_towards(home_x, home_y)
            if arrived or dist < 0.35:
                self.get_logger().info(
                    f'>>> Returned to Home Position (dist={dist:.2f}m)! Stabilizing for 2.0s...'
                )
                self.publish_zero_setpoint(target_z=self.target_altitude)
                self.sleep_and_stream(2.0)
                break
            self.sleep_and_stream(0.04)

        # 7. State: LAND
        self.state = DroneState.LAND
        self.get_logger().info('=== [FSM: LAND] Executing Autonomous Landing ===')
        self.execute_landing()

        # 8. State: DONE
        self.state = DroneState.DONE
        self.get_logger().info(
            '=== [FSM: DONE] Autonomous Frontier Exploration Mission Successfully Completed! ==='
        )


def main():
    rclpy.init()
    node = AutoMapper()
    try:
        node.run_exploration_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user. Stopping drone.')
        node.publish_zero_setpoint()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
