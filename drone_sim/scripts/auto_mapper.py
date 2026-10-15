#!/usr/bin/env python3
"""
Node: auto_mapper.py (drone_sim)
=================================
Autonomous SLAM Mapping Routine with Visual Odometry Feedback & Robust OFFBOARD Handover:
  1. Robust, Armored Takeoff:
     - Handles AUTO.TAKEOFF, CommandTOL, and Offboard climb fallback.
     - Detects manual takeoff instantly if user executed "commander takeoff".
  2. Safe & Seamless OFFBOARD Transition:
     - Streams continuous zero-velocity setpoints (vx=0.0, vy=0.0, vz=0.0, wz=0.0)
       for 2.5 seconds BEFORE requesting OFFBOARD mode to prime PX4's signal buffer.
     - Maintains uninterrupted 50 Hz setpoint stream during mode switch request.
     - Actively verifies FCU mode confirmation and retries safely if needed.
     - Stabilizes for 2.0 seconds in zero-velocity hover so PX4's Altitude Hold
       controller cleanly locks onto current altitude without thrust cut or drop.
  3. Phase 1: Discretized 360° Panoramic Scan (Step-and-Pause):
     - 16 gentle incremental rotation steps (~22.5° each at 0.18 rad/s).
     - 0.8s stationary pause between steps for blur-free feature extraction.
  4. Phase 2: Ultra-smooth Survey Displacements:
     - Gentle translation at 0.12 m/s (zero tilt, clean depth maps).
     - Vertical velocity vz is strictly 0.0 so PX4 holds altitude locked.
  5. Real-time Closed-Loop Odometry Feedback:
     - Subscribes to /rtabmap/odom_info and /rtabmap/odom.
     - Monitors tracking status (lost flag, inliers count, NaNs).
     - If tracking degrades (inliers < 12 or lost), IMMEDIATELY halts motion into hover.
     - Waits for visual tracking to recover before resuming trajectory.
  6. Returns to center and hovers stably.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet
from mavros_msgs.msg import State


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        # Ensure node uses simulation time from Gazebo
        if self.has_parameter('use_sim_time'):
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        else:
            self.declare_parameter('use_sim_time', True)

        # ── Publishers ─────────────────────────────────────────────
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
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

        # Visual Odometry Quality Feedback Subscribers
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

        # ── Service Clients ────────────────────────────────────────
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_client = self.create_client(ParamSet, '/mavros/param/set')

        # ── State Variables ────────────────────────────────────────
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Explicitly initialize Twist with clean 0.0 floats
        self.cmd_twist = Twist()
        self.cmd_twist.linear.x = 0.0
        self.cmd_twist.linear.y = 0.0
        self.cmd_twist.linear.z = 0.0
        self.cmd_twist.angular.x = 0.0
        self.cmd_twist.angular.y = 0.0
        self.cmd_twist.angular.z = 0.0

        # Odometry Health Monitoring
        self.odom_received = False
        self.odom_lost = False
        self.odom_inliers = 0
        self.odom_matches = 0
        self.odom_has_nan = False
        self.last_odom_time = 0.0
        self.target_altitude = 2.0

        # Heartbeat timer: 50 Hz continuous velocity stream
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info('AutoMapper initialized. Waiting for FCU connection...')

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
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            self.odom_has_nan = True
        else:
            self.odom_has_nan = False

    def timer_cb(self):
        self.vel_pub.publish(self.cmd_twist)

    def set_vel(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """Sets linear and angular velocity commands with explicit float precision."""
        self.cmd_twist.linear.x = float(vx)
        self.cmd_twist.linear.y = float(vy)
        self.cmd_twist.linear.z = float(vz)
        self.cmd_twist.angular.x = 0.0
        self.cmd_twist.angular.y = 0.0
        self.cmd_twist.angular.z = float(yaw_rate)

    def publish_zero_setpoint(self):
        """Ensures all axes are strictly 0.0 and immediately dispatches to FCU."""
        self.set_vel(0.0, 0.0, 0.0, 0.0)
        self.vel_pub.publish(self.cmd_twist)

    def is_odometry_healthy(self) -> bool:
        """Checks visual odometry tracking quality."""
        if not self.odom_received:
            return True

        # Stale data (>1.0s)
        if time.time() - self.last_odom_time > 1.0:
            return False

        # Lost tracking flag from RTAB-Map
        if self.odom_lost:
            return False

        # Inlier threshold (< 12 is degraded)
        if self.odom_inliers < 12:
            return False

        # Coordinates corrupted by NaN or Inf
        if self.odom_has_nan:
            return False

        return True

    def sleep_and_stream(self, duration_sec):
        """Streams current setpoints at 50Hz without blocking the ROS executor."""
        end_time = time.time() + duration_sec
        while rclpy.ok() and time.time() < end_time:
            self.vel_pub.publish(self.cmd_twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def safe_motion(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, duration_sec=1.0, desc=""):
        """
        Executes a motion step with active visual odometry health monitoring.
        vz is maintained strictly as requested (0.0 for horizontal motion),
        allowing PX4's internal altitude hold to keep height rigidly fixed.
        """
        if desc:
            self.get_logger().info(f'-> Motion: {desc} (dur: {duration_sec:.1f}s)')

        time_moved = 0.0
        dt = 0.02

        while rclpy.ok() and time_moved < duration_sec:
            # Feedback quality check:
            if not self.is_odometry_healthy():
                self.get_logger().warn(
                    f'[ODOM FEEDBACK] Tracking degraded (lost={self.odom_lost}, inliers={self.odom_inliers})! '
                    'Halting into hover until camera settles...'
                )
                # Command instant hover (vz=0.0 holds altitude)
                self.publish_zero_setpoint()

                # Hold hover until odometry stays healthy continuously for 1.0s
                stable_duration = 0.0
                while rclpy.ok() and stable_duration < 1.0:
                    self.sleep_and_stream(dt)
                    if self.is_odometry_healthy():
                        stable_duration += dt
                    else:
                        stable_duration = 0.0

                self.get_logger().info(
                    f'[ODOM FEEDBACK] Tracking recovered (inliers={self.odom_inliers})! Resuming motion.'
                )

            # Set requested velocity — no artificial Z offsets that cause runaway climbing
            self.set_vel(vx, vy, vz, yaw_rate)
            self.sleep_and_stream(dt)
            time_moved += dt

        # Stop velocity at end of step
        self.publish_zero_setpoint()

    def perform_takeoff(self, target_alt=2.0) -> bool:
        """
        Armored takeoff routine:
        - If already flying (e.g. manual commander takeoff), detects it and proceeds immediately.
        - Sets AUTO.TAKEOFF, sends CommandTOL and Arming command simultaneously.
        - Waits until target altitude is reached, then levels off cleanly.
        """
        # 1. Check if already airborne
        alt = self.current_pose.pose.position.z
        if alt >= 1.0 and self.current_state.armed:
            self.get_logger().info(f'Vehicle already airborne at {alt:.2f}m. Proceeding directly.')
            return True

        self.get_logger().info(f'Initiating automatic takeoff to {target_alt:.1f}m...')

        # Ensure setpoint stream is active (required for MAVROS/PX4 offboard)
        self.publish_zero_setpoint()
        self.sleep_and_stream(0.5)

        # 2. Stage 1: Try AUTO.TAKEOFF (up to 5 attempts)
        for attempt in range(1, 6):
            alt = self.current_pose.pose.position.z
            if alt >= 1.2:
                self.get_logger().info(f'Takeoff detected! Alt: {alt:.2f}m')
                break

            self.get_logger().info(f'Takeoff attempt {attempt}/5 (AUTO.TAKEOFF)...')

            # Request AUTO.TAKEOFF flight mode
            if self.mode_client.service_is_ready():
                m_req = SetMode.Request()
                m_req.custom_mode = 'AUTO.TAKEOFF'
                self.mode_client.call_async(m_req)

            # Request Takeoff to target altitude
            if self.takeoff_client.service_is_ready():
                t_req = CommandTOL.Request()
                t_req.altitude = float(target_alt)
                self.takeoff_client.call_async(t_req)

            # Request Arming
            if not self.current_state.armed:
                if self.arming_client.service_is_ready():
                    a_req = CommandBool.Request()
                    a_req.value = True
                    self.arming_client.call_async(a_req)

            self.sleep_and_stream(1.0)

        # 3. Stage 2: If still on ground, try Offboard climb fallback
        if self.current_pose.pose.position.z < 0.5:
            self.get_logger().info('Switching to OFFBOARD climb takeoff fallback...')
            self.set_vel(0.0, 0.0, 0.6, 0.0)
            self.sleep_and_stream(1.5)

            if self.mode_client.service_is_ready():
                m_req = SetMode.Request()
                m_req.custom_mode = 'OFFBOARD'
                self.mode_client.call_async(m_req)

            if not self.current_state.armed and self.arming_client.service_is_ready():
                a_req = CommandBool.Request()
                a_req.value = True
                self.arming_client.call_async(a_req)

        # 4. Wait for vehicle to reach target hover altitude
        t_start = time.time()
        step = 0
        while rclpy.ok() and (time.time() - t_start < 15.0):
            alt = self.current_pose.pose.position.z
            step += 1
            if step % 10 == 0:
                self.get_logger().info(f'Climbing... Alt: {alt:.2f}m / Target: {target_alt:.1f}m')
            if alt >= 1.5:
                self.get_logger().info(f'Target altitude reached ({alt:.2f}m)!')
                break
            self.sleep_and_stream(0.2)

        # Level off into pure hover
        self.publish_zero_setpoint()
        self.sleep_and_stream(1.5)

        if self.current_pose.pose.position.z < 0.8:
            self.get_logger().warn(
                f'Takeoff altitude low ({self.current_pose.pose.position.z:.2f}m). '
                'Tip: you can enter "commander takeoff" in Terminal 1 at any time.'
            )
            return False

        return True

    def switch_to_offboard(self, pre_stream_sec=2.5, max_wait_sec=8.0) -> bool:
        """
        Transitions cleanly and securely to OFFBOARD mode:
        1. Explicitly sets zero velocity setpoint (vx=0.0, vy=0.0, vz=0.0, wz=0.0).
        2. Pre-streams zero velocity setpoints continuously for 2.5s at 50 Hz
           to thoroughly prime PX4's offboard signal monitor.
        3. Sends SetMode('OFFBOARD') request while continuously maintaining the 50 Hz stream.
        4. Verifies actual switch to OFFBOARD in self.current_state.mode, retrying if needed.
        5. Holds zero-velocity hover for another 2.0s to allow PX4's altitude hold / position
           controller to fully engage without any thrust cuts or altitude drop.
        """
        self.get_logger().info(
            f'>>> Step 1/3: Pre-streaming zero-velocity setpoints for {pre_stream_sec:.1f}s '
            'to prime PX4 offboard buffer...'
        )
        self.publish_zero_setpoint()
        self.sleep_and_stream(pre_stream_sec)

        if self.current_state.mode == 'OFFBOARD':
            self.get_logger().info('Vehicle already in OFFBOARD mode.')
            return True

        self.get_logger().info('>>> Step 2/3: Sending OFFBOARD mode switch request while maintaining 50Hz stream...')
        
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        if self.mode_client.service_is_ready():
            self.mode_client.call_async(req)

        t_start = time.time()
        last_req_time = time.time()

        while rclpy.ok() and (time.time() - t_start < max_wait_sec):
            # Maintain uninterrupted 50Hz zero setpoint stream
            self.publish_zero_setpoint()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

            if self.current_state.mode == 'OFFBOARD':
                self.get_logger().info(
                    f'>>> OFFBOARD Mode Confirmed! Current Altitude: {self.current_pose.pose.position.z:.2f}m'
                )
                break

            # Retry request every 1.0s if mode hasn't switched yet
            if time.time() - last_req_time >= 1.0:
                self.get_logger().info(f'Retrying OFFBOARD request... (current mode: {self.current_state.mode})')
                if self.mode_client.service_is_ready():
                    self.mode_client.call_async(req)
                last_req_time = time.time()

        if self.current_state.mode != 'OFFBOARD':
            self.get_logger().error(f'Failed to switch to OFFBOARD mode! Current mode: {self.current_state.mode}')
            return False

        # Settle in OFFBOARD hover to let PX4 altitude hold engage cleanly
        self.get_logger().info(
            '>>> Step 3/3: Holding stable OFFBOARD hover (zero velocity) for 2.0s to lock altitude...'
        )
        self.publish_zero_setpoint()
        self.sleep_and_stream(2.0)
        return True

    def run_mapping_mission(self):
        # 1. Wait for MAVROS FCU connection
        while rclpy.ok() and not self.current_state.connected:
            self.get_logger().info('Connecting to FCU...')
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)

        self.get_logger().info('FCU Connected!')

        # Wait for essential services ahead of time to avoid blocking during flight
        self.mode_client.wait_for_service(timeout_sec=3.0)
        self.arming_client.wait_for_service(timeout_sec=3.0)
        self.takeoff_client.wait_for_service(timeout_sec=3.0)

        # Set arming yaw check tolerance on PX4
        if self.param_set_client.wait_for_service(timeout_sec=2.0):
            p_req2 = ParamSet.Request()
            p_req2.param_id = 'COM_ARM_EKF_YAW'
            p_req2.value.real = 5.0
            self.param_set_client.call_async(p_req2)

        self.sleep_and_stream(0.5)

        # 2. Takeoff
        takeoff_ok = self.perform_takeoff(target_alt=self.target_altitude)
        if not takeoff_ok:
            # Check once more if user took off manually
            if self.current_pose.pose.position.z >= 1.0:
                self.get_logger().info('Manual takeoff detected! Proceeding with mission.')
            else:
                self.get_logger().error('Could not achieve takeoff. Aborting mission.')
                return

        # 3. Enter OFFBOARD mode with pre-stream and lock
        if not self.switch_to_offboard(pre_stream_sec=2.5):
            self.get_logger().error('Could not engage OFFBOARD mode. Aborting mission.')
            return

        # Settle in hover to acquire initial visual features
        self.get_logger().info('Holding initial hover to lock visual odometry and feature tracking...')
        self.publish_zero_setpoint()
        self.sleep_and_stream(2.0)
        if self.odom_received:
            self.get_logger().info(f'Odometry locked! Initial inliers: {self.odom_inliers}')

        # 4. Phase 1: Discretized 360° Panoramic Scan (Step-and-Pause)
        # 16 steps of ~22.5°: rotate gently at 0.18 rad/s for 2.18s, then pause 0.8s
        total_steps = 16
        step_angle_deg = 360.0 / total_steps
        yaw_speed = 0.18       # rad/s (~10.3 deg/s) - very gentle, zero motion blur
        step_duration = (math.radians(step_angle_deg)) / yaw_speed  # ~2.18s
        pause_duration = 0.8   # seconds of zero motion to allow SLAM keyframe registration

        self.get_logger().info(
            f'>>> PHASE 1: Discretized 360° Panoramic Scan ({total_steps} steps of {step_angle_deg:.1f}°)...'
        )

        for step in range(1, total_steps + 1):
            deg = step * step_angle_deg
            # Incremental rotation step with active tracking feedback and vz=0.0
            self.safe_motion(
                vx=0.0, vy=0.0, vz=0.0, yaw_rate=yaw_speed,
                duration_sec=step_duration,
                desc=f'Step {step}/{total_steps} ({deg:.0f}°)'
            )
            # Settle pause for camera stabilization and keyframe extraction
            self.publish_zero_setpoint()
            self.sleep_and_stream(pause_duration)

        self.get_logger().info('>>> 360° Panoramic Scan Complete! Settle hover for 2s...')
        self.publish_zero_setpoint()
        self.sleep_and_stream(2.0)

        # 5. Phase 2: Ultra-Smooth Survey Displacements (0.12 m/s, vz=0.0)
        # Gentle translation at 0.12 m/s with strictly zero vertical velocity
        # Each leg divided into 2 mini-segments with a 1.0s settle pause
        legs = [
            ("Forward towards North Wall",       0.12,  0.0,  0.0, 5.0),
            ("Strafe Right towards East Wall",   0.0, -0.12,  0.0, 5.0),
            ("Backward towards South Wall",     -0.12,  0.0,  0.0, 5.0),
            ("Strafe Left towards Center",       0.0,  0.12,  0.0, 5.0),
        ]

        self.get_logger().info('>>> PHASE 2: Ultra-Smooth Survey Flight (gentle translation @ 0.12 m/s, vz=0.0)...')
        for desc, vx, vy, vz, duration in legs:
            # Segment 1
            self.safe_motion(
                vx=vx, vy=vy, vz=0.0, yaw_rate=0.0,
                duration_sec=duration, desc=f'{desc} [Part 1]'
            )
            # Intermediate settle pause
            self.publish_zero_setpoint()
            self.sleep_and_stream(1.0)

            # Segment 2
            self.safe_motion(
                vx=vx, vy=vy, vz=0.0, yaw_rate=0.0,
                duration_sec=duration, desc=f'{desc} [Part 2]'
            )
            # Intermediate settle pause
            self.publish_zero_setpoint()
            self.sleep_and_stream(1.0)

        # 6. Complete and Hold Center Position
        self.publish_zero_setpoint()
        self.get_logger().info(
            '>>> Autonomous Mapping Mission Complete! 3D Map and 2D Grid are populated. '
            'Drone holding stable hover at center.'
        )
        while rclpy.ok():
            self.sleep_and_stream(0.5)


def main():
    rclpy.init()
    node = AutoMapper()
    try:
        node.run_mapping_mission()
    except KeyboardInterrupt:
        node.publish_zero_setpoint()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
