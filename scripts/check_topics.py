#!/usr/bin/env python3
"""
Script: check_topics.py
========================
Utilitar de diagnosticare — verifica rapid ca toate topic-urile
necesare sunt active si au frecventa corecta.

Ruleaza inainte de orice zbor pentru a confirma ca sistemul e functional.

Cum se ruleaza:
  ros2 run drone_slam check_topics.py

Output exemplu:
  ✅ /oak/rgb/image_raw          →  20.1 Hz  (asteptat: ~20)
  ✅ /oak/imu/data               → 200.3 Hz  (asteptat: ~200)
  ✅ /oak/stereo/image_raw       →  20.0 Hz  (asteptat: ~20)
  ✅ /rtabmap/odom               →  15.2 Hz  (asteptat: >10)
  ✅ /odometry/filtered          →  50.0 Hz  (asteptat: ~50)
  ✅ /mavros/state connected     →  True
  ⚠️  /mavros/local_position/pose →   0.0 Hz  (asteptat: ~30)
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


# Definitia topic-urilor de verificat
# (topic, tip_mesaj, frecventa_minima_Hz, descriere)
TOPICS_TO_CHECK = [
    ('/oak/rgb/image_raw',          Image,       15.0, 'Camera RGB'),
    ('/oak/imu/data',               Imu,        150.0, 'IMU OAK-S2'),
    ('/oak/stereo/image_raw',       Image,       15.0, 'Camera Depth'),
    ('/rtabmap/odom',               Odometry,     8.0, 'Odometrie vizuala'),
    ('/odometry/filtered',          Odometry,    40.0, 'Odometrie EKF'),
    ('/mavros/local_position/pose', PoseStamped, 20.0, 'Pozitie locala PX4'),
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

        # Creeaza subscriber pentru fiecare topic
        self.subscribers = []
        for topic, msg_type, _, _ in TOPICS_TO_CHECK:
            sub = self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self.callback(t),
                qos
            )
            self.subscribers.append(sub)

        # Subscriber special pentru starea MAVROS (nu frecventa, ci boolean)
        self.mavros_connected = False
        self.mavros_sub = self.create_subscription(
            State, '/mavros/state',
            self.mavros_callback, 10
        )

        # Timer: afiseaza raport dupa 5 secunde
        self.timer = self.create_timer(5.0, self.print_report)
        self.get_logger().info('Verific topic-urile timp de 5 secunde...')

    def callback(self, topic_name: str):
        self.counts[topic_name] += 1

    def mavros_callback(self, msg: State):
        self.mavros_connected = msg.connected

    def print_report(self):
        elapsed = time.time() - self.start_time
        print('\n' + '═' * 65)
        print('  RAPORT VERIFICARE SISTEM VISUAL SLAM')
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

        # Verificare speciala MAVROS
        mavros_icon = '✅' if self.mavros_connected else '❌'
        print(f'  {mavros_icon}  {"MAVROS conectat":<25} '
              f'{"Da" if self.mavros_connected else "Nu":<10}'
              f'  [/mavros/state]')

        print('═' * 65)
        if all_ok and self.mavros_connected:
            print('  ✅ SISTEM OK — gata de zbor')
        else:
            print('  ⚠️  PROBLEME DETECTATE — nu zbura inca!')
        print('═' * 65 + '\n')

        # Opreste nodul dupa raport
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
