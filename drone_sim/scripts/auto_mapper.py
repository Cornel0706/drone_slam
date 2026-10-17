#!/usr/bin/env python3
"""
Node: auto_mapper.py (drone_sim)
=================================
Autonomous SLAM Mapping Routine with Ground-Decoupled Takeoff, Dual Setpoint Streaming & Body-Frame Control:
  1. Armored Autonomous Takeoff:
     - Sends MAV_CMD_NAV_TAKEOFF (CommandLong 22) + CommandTOL with NaNs for local in-place ascent.
     - Automatically arms and commands AUTO.TAKEOFF without requiring terminal intervention.
     - Retains instant fallback detection if user manually executes "commander takeoff".
  2. Ground-Decoupled Odometry Feedback:
     - Visual odometry health check is strictly BYPASSED on the ground (alt < 1.0m),
       preventing stereo disparity blind zones (<30cm from floor) from paralyzing liftoff.
     - Once airborne (alt >= 1.0m), active tracking feedback protects against motion blur.
  3. Dual-Stream Setpoint Velocity & Body-to-ENU Transformation:
     - Publishes simultaneously to /mavros/setpoint_velocity/cmd_vel (TwistStamped with sim clock)
       and /mavros/setpoint_velocity/cmd_vel_unstamped (Twist) to eliminate topic/format mismatches.
     - Automatically rotates body-relative commands (Forward/Lateral) into world ENU coordinates
       using current drone yaw, ensuring commands physically move the drone in the intended direction.
  4. Smooth OFFBOARD Handover:
     - Streams zero-velocity setpoints for 2.0s at 50 Hz before mode switch to prime PX4 watchdog.
     - Verifies OFFBOARD confirmation and stabilizes hover for 2.0s.
  5. Phase 1: Discretized 360° Panoramic Scan (Step-and-Pause):
     - 16 gentle incremental rotation steps (~22.5° each at 0.22 rad/s).
     - 0.8s stationary pause between steps for keyframe matching.
  6. Phase 2: Survey Displacements in Body Frame (0.30 m/s):
     - Visibly moves the drone across the room to map walls and obstacles.
     - vz is held strictly 0.0 to lock altitude.
  7. Returns to center and holds position in stable hover.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from mavros_msgs.srv import CommandBool, CommandTOL, CommandLong, SetMode, ParamSet
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
        # 1. Unstamped Twist (standard MAVROS topic)
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        # 2. Stamped Twist (canonical topic with explicit simulation timestamp)
        self.vel_stamped_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
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
        self.cmd_long_client = self.create_client(CommandLong, '/mavros/cmd/command')
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

    def get_current_yaw(self) -> float:
        """Extracts current yaw angle (radians) from FCU pose orientation quaternion."""
        q = self.current_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        if not (math.isfinite(siny_cosp) and math.isfinite(cosy_cosp)):
            return 0.0
        return math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        """Dispatches velocity commands to both stamped and unstamped MAVROS topics."""
        # 1. Unstamped
        self.vel_pub.publish(self.cmd_twist)
        
        # 2. Stamped with current Gazebo simulation timestamp
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = self.cmd_twist
        self.vel_stamped_pub.publish(stamped)

    def set_vel(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """Sets linear and angular velocity commands in local ENU frame."""
        self.cmd_twist.linear.x = float(vx)
        self.cmd_twist.linear.y = float(vy)
        self.cmd_twist.linear.z = float(vz)
        self.cmd_twist.angular.x = 0.0
        self.cmd_twist.angular.y = 0.0
        self.cmd_twist.angular.z = float(yaw_rate)

    def set_body_vel(self, v_forward=0.0, v_strafe_left=0.0, vz=0.0, yaw_rate=0.0):
        """
        Converts Body-frame speeds (Forward, Strafe Left, Up, Yaw rate)
        into Local ENU velocities using the drone's actual current yaw heading.
        This guarantees that commanding +Forward physically moves in the direction the camera faces.
        """
        yaw = self.get_current_yaw()
        vx_enu = float(v_forward) * math.cos(yaw) - float(v_strafe_left) * math.sin(yaw)
        vy_enu = float(v_forward) * math.sin(yaw) + float(v_strafe_left) * math.cos(yaw)
        self.set_vel(vx_enu, vy_enu, vz, yaw_rate)

    def publish_zero_setpoint(self):
        """Ensures all axes are strictly 0.0 and immediately dispatches to FCU."""
        self.set_vel(0.0, 0.0, 0.0, 0.0)
        self.timer_cb()

    def is_odometry_healthy(self) -> bool:
        """
        Checks visual odometry tracking quality.
        CRITICAL: Bypassed when on ground (alt < 1.0m) to avoid blocking liftoff!
        """
        alt = self.current_pose.pose.position.z
        if alt < 1.0:
            # On the ground or during initial climb, camera is too close to floor (<30cm)
            # for stereo disparity. DO NOT block motion!
            return True

        if not self.odom_received:
            return True

        # Stale data (>1.0s)
        if time.time() - self.last_odom_time > 1.0:
            return False

        # Lost tracking flag from RTAB-Map
        if self.odom_lost:
            return False

        # Inlier threshold (< 10 is degraded in flight)
        if self.odom_inliers < 10:
            return False

        # Coordinates corrupted by NaN or Inf
        if self.odom_has_nan:
            return False

        return True

    def sleep_and_stream(self, duration_sec):
        """Streams current setpoints at 50Hz without blocking the ROS executor."""
        end_time = time.time() + duration_sec
        while rclpy.ok() and time.time() < end_time:
            self.timer_cb()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def safe_motion(self, v_forward=0.0, v_strafe_left=0.0, vz=0.0, yaw_rate=0.0, duration_sec=1.0, desc=""):
        """
        Executes a motion step in drone body frame with active odometry health feedback.
        vz is maintained strictly as requested (0.0 for horizontal motion) to lock altitude.
        """
        if desc:
            self.get_logger().info(f'-> Motion: {desc} (dur: {duration_sec:.1f}s)')

        time_moved = 0.0
        dt = 0.02

        while rclpy.ok() and time_moved < duration_sec:
            # Feedback quality check (active only when airborne):
            if not self.is_odometry_healthy():
                self.get_logger().warn(
                    f'[ODOM FEEDBACK] Tracking degraded (lost={self.odom_lost}, inliers={self.odom_inliers})! '
                    'Halting into hover until camera settles...'
                )
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

            # Set requested body velocity converted to world ENU
            self.set_body_vel(v_forward, v_strafe_left, vz, yaw_rate)
            self.sleep_and_stream(dt)
            time_moved += dt

        # Stop velocity at end of step
        self.publish_zero_setpoint()

    def perform_takeoff(self, target_alt=2.0) -> bool:
        """
        Armored autonomous takeoff routine:
        - If already airborne (e.g. manual commander takeoff), detects it and proceeds immediately.
        - Sends MAV_CMD_NAV_TAKEOFF (CommandLong command 22) with NaNs for in-place vertical liftoff.
        - Commands CommandTOL with NaNs and requests AUTO.TAKEOFF flight mode.
        - Arms the vehicle and monitors climb until target hover altitude (>=1.5m) is reached.
        """
        # 1. Check if already airborne
        alt = self.current_pose.pose.position.z
        if alt >= 1.0 and self.current_state.armed:
            self.get_logger().info(f'Vehicle already airborne at {alt:.2f}m. Proceeding directly.')
            return True

        self.get_logger().info(f'Initiating automatic takeoff to {target_alt:.1f}m...')

        # Ensure setpoint stream is active (required for MAVROS/PX4 offboard watchdog)
        self.publish_zero_setpoint()
        self.sleep_and_stream(1.0)

        # 2. Multi-stage takeoff commands:
        for attempt in range(1, 6):
            alt = self.current_pose.pose.position.z
            if alt >= 1.2:
                self.get_logger().info(f'Takeoff detected! Alt: {alt:.2f}m')
                break

            self.get_logger().info(f'Takeoff attempt {attempt}/5...')

            # A. Send MAV_CMD_NAV_TAKEOFF via CommandLong (exact equivalent of PX4 "commander takeoff")
            if self.cmd_long_client.service_is_ready():
                cl_req = CommandLong.Request()
                cl_req.broadcast = False
                cl_req.command = 22  # MAV_CMD_NAV_TAKEOFF
                cl_req.confirmation = 0
                cl_req.param1 = 0.0          # pitch
                cl_req.param4 = float('nan') # yaw
                cl_req.param5 = float('nan') # lat (NaN = takeoff from current position)
                cl_req.param6 = float('nan') # lon (NaN = takeoff from current position)
                cl_req.param7 = float('nan') # alt (NaN = use MIS_TAKEOFF_ALT)
                self.cmd_long_client.call_async(cl_req)

            # B. Send CommandTOL with NaNs for local in-place takeoff
            if self.takeoff_client.service_is_ready():
                t_req = CommandTOL.Request()
                t_req.min_pitch = 0.0
                t_req.yaw = float('nan')
                t_req.latitude = float('nan')
                t_req.longitude = float('nan')
                t_req.altitude = float(target_alt)
                self.takeoff_client.call_async(t_req)

            # C. Request AUTO.TAKEOFF mode
            if self.mode_client.service_is_ready():
                m_req = SetMode.Request()
                m_req.custom_mode = 'AUTO.TAKEOFF'
                self.mode_client.call_async(m_req)

            # D. Request Arming
            if not self.current_state.armed and self.arming_client.service_is_ready():
                a_req = CommandBool.Request()
                a_req.value = True
                self.arming_client.call_async(a_req)

            self.sleep_and_stream(1.0)

        # 3. Wait for vehicle to climb to target hover altitude
        t_start = time.time()
        step = 0
        while rclpy.ok() and (time.time() - t_start < 15.0):
            alt = self.current_pose.pose.position.z
            step += 1
            if step % 5 == 0:
                self.get_logger().info(f'Climbing... Alt: {alt:.2f}m / Target: {target_alt:.1f}m')
            if alt >= 1.5:
                self.get_logger().info(f'Target altitude reached ({alt:.2f}m)!')
                break
            self.sleep_and_stream(0.2)

        # Level off into pure hover
        self.publish_zero_setpoint()
        self.sleep_and_stream(1.5)

        alt = self.current_pose.pose.position.z
        if alt < 0.8:
            self.get_logger().warn(
                f'Takeoff altitude low ({alt:.2f}m). '
                'Tip: you can enter "commander takeoff" in Terminal 1 if FCU requires manual authorization.'
            )
            return False

        return True

    def switch_to_offboard(self, pre_stream_sec=2.0, max_wait_sec=8.0) -> bool:
        """
        Transitions cleanly and securely to OFFBOARD mode:
        1. Explicitly sets zero velocity setpoint (vx=0.0, vy=0.0, vz=0.0, wz=0.0).
        2. Pre-streams zero velocity setpoints continuously for 2.0s at 50 Hz
           to thoroughly prime PX4's offboard signal monitor.
        3. Sends SetMode('OFFBOARD') request while continuously maintaining the 50 Hz stream.
        4. Verifies actual switch to OFFBOARD in self.current_state.mode, retrying if needed.
        5. Holds zero-velocity hover for another 2.0s to allow PX4's altitude hold
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
        self.cmd_long_client.wait_for_service(timeout_sec=3.0)

        # Set arming yaw check tolerance on PX4
        if self.param_set_client.wait_for_service(timeout_sec=2.0):
            p_req2 = ParamSet.Request()
            p_req2.param_id = 'COM_ARM_EKF_YAW'
            p_req2.value.real = 5.0
            self.param_set_client.call_async(p_req2)

        self.sleep_and_stream(0.5)

        # 2. Automated Takeoff
        takeoff_ok = self.perform_takeoff(target_alt=self.target_altitude)
        if not takeoff_ok:
            if self.current_pose.pose.position.z >= 1.0:
                self.get_logger().info('Manual takeoff detected! Proceeding with mission.')
            else:
                self.get_logger().error('Could not achieve takeoff. Aborting mission.')
                return

        # 3. Enter OFFBOARD mode with pre-stream and lock
        if not self.switch_to_offboard(pre_stream_sec=2.0):
            self.get_logger().error('Could not engage OFFBOARD mode. Aborting mission.')
            return

        # Settle in hover to acquire initial visual features
        self.get_logger().info('Holding initial hover to lock visual odometry and feature tracking...')
        self.publish_zero_setpoint()
        self.sleep_and_stream(2.0)
        if self.odom_received:
            self.get_logger().info(f'Odometry locked! Initial inliers: {self.odom_inliers}')

        # 4. Phase 1: Discretized 360° Panoramic Scan (Step-and-Pause)
        # 16 steps of ~22.5°: rotate gently at 0.22 rad/s for 1.78s, then pause 0.8s
        total_steps = 16
        step_angle_deg = 360.0 / total_steps
        yaw_speed = 0.22       # rad/s (~12.6 deg/s) - smooth and gentle
        step_duration = (math.radians(step_angle_deg)) / yaw_speed  # ~1.78s
        pause_duration = 0.8   # seconds of zero motion to allow SLAM keyframe registration

        self.get_logger().info(
            f'>>> PHASE 1: Discretized 360° Panoramic Scan ({total_steps} steps of {step_angle_deg:.1f}°)...'
        )

        for step in range(1, total_steps + 1):
            deg = step * step_angle_deg
            # Incremental rotation step with active tracking feedback and vz=0.0
            self.safe_motion(
                v_forward=0.0, v_strafe_left=0.0, vz=0.0, yaw_rate=yaw_speed,
                duration_sec=step_duration,
                desc=f'Step {step}/{total_steps} ({deg:.0f}°)'
            )
            # Settle pause for camera stabilization and keyframe extraction
            self.publish_zero_setpoint()
            self.sleep_and_stream(pause_duration)

        self.get_logger().info('>>> 360° Panoramic Scan Complete! Settle hover for 2s...')
        self.publish_zero_setpoint()
        self.sleep_and_stream(2.0)

        # 5. Phase 2: Ultra-Smooth Survey Flight in Body Coordinates
        # Speeds set to 0.30 m/s for clean, visible physical motion in Gazebo
        # v_forward and v_strafe_left automatically transform to world ENU based on current yaw
        legs = [
            ("Forward towards North Wall",       0.30,  0.0,  0.0, 4.0),
            ("Strafe Right towards East Wall",   0.0, -0.30,  0.0, 4.0),
            ("Backward towards South Wall",     -0.30,  0.0,  0.0, 4.0),
            ("Strafe Left towards Center",       0.0,  0.30,  0.0, 4.0),
        ]

        self.get_logger().info('>>> PHASE 2: Survey Flight (body-frame translation @ 0.30 m/s, vz=0.0)...')
        for desc, v_fwd, v_lat, vz, duration in legs:
            # Segment 1
            self.safe_motion(
                v_forward=v_fwd, v_strafe_left=v_lat, vz=0.0, yaw_rate=0.0,
                duration_sec=duration, desc=f'{desc} [Part 1]'
            )
            # Intermediate settle pause
            self.publish_zero_setpoint()
            self.sleep_and_stream(1.0)

            # Segment 2
            self.safe_motion(
                v_forward=v_fwd, v_strafe_left=v_lat, vz=0.0, yaw_rate=0.0,
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
