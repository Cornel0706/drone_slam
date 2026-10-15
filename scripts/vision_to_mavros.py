#!/usr/bin/env python3
"""
ROS2 Node: vision_to_mavros.py
================================
This node bridges visual odometry (EKF output) to MAVROS.

PROBLEM IT SOLVES:
  MAVROS expects the visual position estimate on the topic
  /mavros/vision_pose/pose with type geometry_msgs/PoseStamped.
  Our EKF publishes on /odometry/filtered with type nav_msgs/Odometry.
  These are different message types, so a conversion node is needed.

WHY THIS IS IMPORTANT:
  The Pixhawk 6C Mini runs an internal EKF2 (Extended Kalman Filter v2).
  When we send it the visual position, PX4 fuses it with:
    - Barometer (altitude)
    - Internal IMU (accelerometer + gyroscope)
    - GPS (if available)
  The result is a position estimate far more robust than any single sensor.

  Relevant EKF2 parameters to set in QGroundControl:
    EKF2_EV_CTRL = 15  (enable visual source for x, y, z, yaw)
    EKF2_HGT_REF = 3   (altitude comes from vision, not barometer)

PUBLISH RATE:
  Publishes at 30 Hz — sufficient for PX4, without overloading the serial link.

How to start:
  ros2 run drone_slam vision_to_mavros.py
  (or automatically via slam_full.launch.py)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VisionToMavros(Node):
    """
    Converts nav_msgs/Odometry → geometry_msgs/PoseStamped
    and publishes to /mavros/vision_pose/pose.
    """

    def __init__(self):
        super().__init__('vision_to_mavros')

        # ── QoS Profile ────────────────────────────────────────────
        # BEST_EFFORT: do not block if a message is lost (real-time priority)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscriber: filtered odometry from EKF ─────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',       # EKF output topic
            self.odom_callback,
            qos_best_effort
        )

        # ── Publisher: visual pose to MAVROS/Pixhawk ────────────────
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose', # MAVROS listens on this topic
            10
        )

        self.get_logger().info(
            'VisionToMavros node started. '
            'Subscribing to /odometry/filtered → publishing /mavros/vision_pose/pose'
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback invoked on every Odometry message from the EKF.
        Extracts position and orientation and publishes them as PoseStamped.
        """
        pose_stamped = PoseStamped()

        # Preserve the original timestamp (critical for PX4!)
        # PX4 uses the timestamp to estimate the sensor pipeline latency
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'map'  # global reference frame

        # Copy position (x, y, z in meters)
        pose_stamped.pose.position = msg.pose.pose.position

        # Copy orientation (quaternion: x, y, z, w)
        pose_stamped.pose.orientation = msg.pose.pose.orientation

        # Publish to MAVROS
        self.vision_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)

    node = VisionToMavros()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
