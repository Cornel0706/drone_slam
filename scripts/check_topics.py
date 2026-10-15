#!/usr/bin/env python3
"""
Script: check_topics.py
========================
Diagnostic utility — quickly verifies that all required topics
are active and publishing at the expected frequency.

Run this before every flight to confirm the system is fully operational.

How to run:
  ros2 run drone_slam check_topics.py

Example output:
  ✅ /oak/rgb/image_raw          →  20.1 Hz  (expected: ~20)
  ✅ /oak/imu/data               → 200.3 Hz  (expected: ~200)
  ✅ /oak/stereo/image_raw       →  20.0 Hz  (expected: ~20)
  ✅ /rtabmap/odom               →  15.2 Hz  (expected: >10)
  ✅ /odometry/filtered          →  50.0 Hz  (expected: ~50)
  ✅ /mavros/state connected     →  True
  ⚠️  /mavros/local_position/pose →   0.0 Hz  (expected: ~30)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

import time
from collections import defaultdict


# Definition of topics to check
# (topic, message_type, minimum_frequency_Hz, description)
TOPICS_TO_CHECK = [
    ('/oak/rgb/image_raw',          Image,       15.0, 'RGB Camera'),
    ('/oak/imu/data',               Imu,        150.0, 'OAK-S2 IMU'),
    ('/oak/stereo/image_raw',       Image,       15.0, 'Depth Camera'),
    ('/rtabmap/odom',               Odometry,     8.0, 'Visual Odometry'),
    ('/odometry/filtered',          Odometry,    40.0, 'EKF Odometry'),
    ('/mavros/local_position/pose', PoseStamped, 20.0, 'PX4 Local Position'),
]


class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')

        self.counts = defaultdict(int)
        self.start_time = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create a subscriber for each topic to monitor
        self.subscribers = []
        for topic, msg_type, _, _ in TOPICS_TO_CHECK:
            sub = self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self.callback(t),
                qos
            )
            self.subscribers.append(sub)

        # Special subscriber for MAVROS state (boolean check, not frequency)
        self.mavros_connected = False
        self.mavros_sub = self.create_subscription(
            State, '/mavros/state',
            self.mavros_callback, 10
        )

        # Timer: print report after 5 seconds of monitoring
        self.timer = self.create_timer(5.0, self.print_report)
        self.get_logger().info('Monitoring topics for 5 seconds...')

    def callback(self, topic_name: str):
        self.counts[topic_name] += 1

    def mavros_callback(self, msg: State):
        self.mavros_connected = msg.connected

    def print_report(self):
        elapsed = time.time() - self.start_time
        print('\n' + '═' * 65)
        print('  VISUAL SLAM SYSTEM CHECK REPORT')
        print('═' * 65)

        all_ok = True
        for topic, _, min_hz, desc in TOPICS_TO_CHECK:
            count = self.counts[topic]
            actual_hz = count / elapsed if elapsed > 0 else 0
            ok = actual_hz >= min_hz

            if not ok:
                all_ok = False

            icon = '✅' if ok else '⚠️ '
            print(f'  {icon}  {desc:<25} {actual_hz:6.1f} Hz'
                  f'  (min: {min_hz:.0f} Hz)  [{topic}]')

        # Special MAVROS connection check
        mavros_icon = '✅' if self.mavros_connected else '❌'
        print(f'  {mavros_icon}  {"MAVROS connected":<25} '
              f'{"Yes" if self.mavros_connected else "No":<10}'
              f'  [/mavros/state]')

        print('═' * 65)
        if all_ok and self.mavros_connected:
            print('  ✅ SYSTEM OK — ready for flight')
        else:
            print('  ⚠️  ISSUES DETECTED — do not fly yet!')
        print('═' * 65 + '\n')

        # Shut down the node after printing the report
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = TopicChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
