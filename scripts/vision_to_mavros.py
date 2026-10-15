#!/usr/bin/env python3
"""
ROS2 Node: vision_to_mavros.py
================================
This node bridges visual odometry (local EKF or raw visual odometry) to MAVROS.

CRITICAL ROBUSTNESS & FLIGHT SAFETY FEATURES:
  1. Local Odometry Exclusivity:
     - Subscribes exclusively to local continuous odometry (/odometry/filtered or /rtabmap/odom).
     - NEVER subscribes to global map topics (/rtabmap/map, /rtabmap/mapData, /rtabmap/localization_pose).
       This prevents violent position shocks and discontinuities caused by SLAM loop closures / graph optimizations.
  2. Universal QoS Policy:
     - Uses BEST_EFFORT subscription with depth 10, which connects seamlessly to both
       RELIABLE publishers (e.g. robot_localization, rtabmap) and BEST_EFFORT publishers without DDS mismatch.
     - Publishes with RELIABLE QoS (depth 10) to match MAVROS's vision_pose plugin.
  3. Strict Quaternion Validation & Normalization:
     - Verifies all 4 quaternion components are finite (rejects NaNs and Infs).
     - Checks quaternion norm: rejects degenerate / all-zero quaternions before they can reach PX4.
     - Strictly normalizes quaternion before publishing, preventing EKF2 covariance explosion.
     - Converts to Euler angles (roll, pitch) and verifies physical tilt limits (|tilt| < 45°).
       If visual feature tracking temporarily produces an inverted or extreme tilt, it is discarded.
  4. Jump & Velocity Outlier Rejection:
     - Rejects instantaneous position jumps that exceed physically possible speeds (> 5.0 m/s).
     - Bounded workspace check (rejects coordinate explosion).
     - Covariance check (rejects frames where visual tracking covariance indicates lost tracking).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VisionToMavros(Node):
    """
    Converts local nav_msgs/Odometry → geometry_msgs/PoseStamped
    and publishes safely to /mavros/vision_pose/pose.
    """

    def __init__(self):
        super().__init__('vision_to_mavros')

        # ── Parameters ─────────────────────────────────────────────
        self.declare_parameter('target_rate_hz', 30.0)
        self.declare_parameter('target_frame_id', '')               # empty = retain incoming frame (typically 'odom')
        self.declare_parameter('odom_topic', '/odometry/filtered')  # strictly local continuous odometry
        self.declare_parameter('max_velocity_mps', 5.0)             # reject position jumps exceeding 5 m/s
        self.declare_parameter('max_tilt_deg', 45.0)                # reject wild tilt angles > 45 degrees
        self.declare_parameter('max_pos_bound_m', 25.0)             # workspace limit

        self.target_rate = float(self.get_parameter('target_rate_hz').value)
        self.target_frame_id = str(self.get_parameter('target_frame_id').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self.max_tilt_rad = math.radians(float(self.get_parameter('max_tilt_deg').value))
        self.max_bound = float(self.get_parameter('max_pos_bound_m').value)

        self.min_interval = 1.0 / self.target_rate if self.target_rate > 0 else 0.0
        self.last_pub_time = 0.0

        # State tracking for jump outlier rejection
        self.last_valid_pos = None
        self.last_valid_time = None
        self.published_count = 0
        self.rejected_count = 0

        # ── Universal Subscription QoS Profile ─────────────────────
        # BEST_EFFORT can receive from both RELIABLE and BEST_EFFORT publishers in ROS 2.
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscriber: Strictly Local Odometry ────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_sub
        )

        # ── Publisher: Visual Pose to MAVROS/PX4 ───────────────────
        # Default QoS (Reliable, Volatile, depth 10) matches MAVROS expectation
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        self.get_logger().info(
            f'VisionToMavros started (Target rate: {self.target_rate:.1f} Hz).\n'
            f'  -> Subscribing to LOCAL odometry: {self.odom_topic} (QoS: BEST_EFFORT)\n'
            f'  -> Publishing to MAVROS: /mavros/vision_pose/pose (QoS: RELIABLE)\n'
            f'  -> Safety bounds: Max Vel={self.max_velocity:.1f} m/s, Max Tilt={math.degrees(self.max_tilt_rad):.0f}°, Max Bound=±{self.max_bound:.0f}m'
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback invoked on every Odometry message.
        Filters, sanitizes, and forwards local pose to MAVROS.
        """
        # 1. Rate throttling
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pub_time > 0.0 and (current_time - self.last_pub_time) < self.min_interval:
            return

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        cov = msg.pose.covariance

        # 2. Check for NaN or Inf in position
        if not (math.isfinite(pos.x) and math.isfinite(pos.y) and math.isfinite(pos.z)):
            self._reject_sample('Position contains NaN or Inf')
            return

        # 3. Check workspace physical bounds
        if abs(pos.x) > self.max_bound or abs(pos.y) > self.max_bound or abs(pos.z) > 15.0:
            self._reject_sample(f'Position outside workspace bounds (x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f})')
            return

        # 4. Check covariance (if tracking is lost, RTAB-Map sets high variance e.g. >= 9999)
        if len(cov) >= 15:
            var_x, var_y, var_z = cov[0], cov[7], cov[14]
            if var_x > 10.0 or var_y > 10.0 or var_z > 10.0 or var_x < 0.0:
                self._reject_sample(f'High position variance in odometry (var_x={var_x:.1f})')
                return

        # 5. Position jump / velocity outlier check
        if self.last_valid_pos is not None and self.last_valid_time is not None:
            dt = current_time - self.last_valid_time
            if dt > 0.001:
                dx = pos.x - self.last_valid_pos[0]
                dy = pos.y - self.last_valid_pos[1]
                dz = pos.z - self.last_valid_pos[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                apparent_speed = dist / dt
                if apparent_speed > self.max_velocity:
                    self._reject_sample(
                        f'Position jump outlier rejected: apparent speed {apparent_speed:.1f} m/s '
                        f'(exceeds max {self.max_velocity:.1f} m/s)'
                    )
                    return

        # 6. Check for NaN or Inf in orientation quaternion
        if not (math.isfinite(ori.x) and math.isfinite(ori.y) and math.isfinite(ori.z) and math.isfinite(ori.w)):
            self._reject_sample('Orientation quaternion contains NaN or Inf')
            return

        # 7. Check quaternion norm & normalize
        norm_sq = ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w
        if norm_sq < 1e-6 or not math.isfinite(norm_sq):
            self._reject_sample('Degenerate quaternion (norm ~ 0)')
            return

        norm = math.sqrt(norm_sq)
        norm_qx = ori.x / norm
        norm_qy = ori.y / norm
        norm_qz = ori.z / norm
        norm_qw = ori.w / norm

        # 8. Tilt Sanity Check (Convert to Euler roll and pitch)
        # Prevents violent attitude reaction in PX4 if visual tracking produces wild orientations
        sinr_cosp = 2.0 * (norm_qw * norm_qx + norm_qy * norm_qz)
        cosr_cosp = 1.0 - 2.0 * (norm_qx * norm_qx + norm_qy * norm_qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (norm_qw * norm_qy - norm_qz * norm_qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        if abs(roll) > self.max_tilt_rad or abs(pitch) > self.max_tilt_rad:
            self._reject_sample(
                f'Excessive attitude tilt in visual estimate '
                f'(roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}° > {math.degrees(self.max_tilt_rad):.0f}°)'
            )
            return

        # 9. Build and publish sanitized PoseStamped
        pose_stamped = PoseStamped()

        # Timestamp: use msg timestamp, or current time if uninitialized
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
        else:
            pose_stamped.header.stamp = msg.header.stamp

        # Frame: typically 'odom'
        if self.target_frame_id:
            pose_stamped.header.frame_id = self.target_frame_id
        else:
            pose_stamped.header.frame_id = msg.header.frame_id or 'odom'

        pose_stamped.pose.position.x = float(pos.x)
        pose_stamped.pose.position.y = float(pos.y)
        pose_stamped.pose.position.z = float(pos.z)

        pose_stamped.pose.orientation.x = float(norm_qx)
        pose_stamped.pose.orientation.y = float(norm_qy)
        pose_stamped.pose.orientation.z = float(norm_qz)
        pose_stamped.pose.orientation.w = float(norm_qw)

        # Update valid tracking state
        self.last_valid_pos = (pos.x, pos.y, pos.z)
        self.last_valid_time = current_time
        self.last_pub_time = current_time
        self.published_count += 1

        self.vision_pub.publish(pose_stamped)

    def _reject_sample(self, reason: str):
        self.rejected_count += 1
        if self.rejected_count % 30 == 1:
            self.get_logger().warn(f'[SAMPLE REJECTED] {reason} (Total rejected: {self.rejected_count})')


def main(args=None):
    rclpy.init(args=args)

    node = VisionToMavros()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
