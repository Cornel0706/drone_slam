#!/usr/bin/env python3
"""
Nod ROS2: vision_to_mavros.py
================================
Acest nod face bridge-ul intre odometria vizuala (EKF output) si MAVROS.

PROBLEMA PE CARE O REZOLVA:
  MAVROS asteapta pozitia vizuala pe topic-ul /mavros/vision_pose/pose
  de tip geometry_msgs/PoseStamped.
  EKF-ul nostru publica pe /odometry/filtered de tip nav_msgs/Odometry.
  Cele doua tipuri sunt diferite, deci avem nevoie de un nod de conversie.

DE CE E IMPORTANT:
  Pixhawk 6C Mini ruleaza intern un EKF2 (Extended Kalman Filter v2).
  Cand ii trimitem pozitia vizuala, PX4 o fuzioneza cu:
    - Barometrul (altitudine)
    - IMU-ul intern (accelerometru + giroscop)
    - GPS (daca e disponibil)
  Rezultatul este o estimare de pozitie mult mai robusta decat oricare
  senzor singur.

  Parametrii EKF2 relevanti in QGroundControl:
    EKF2_EV_CTRL = 15  (activeaza sursa vizuala pentru x,y,z,yaw)
    EKF2_HGT_REF = 3   (altitudinea vine din vision, nu din baro)

FRECVENTA:
  Trimitem la 30Hz — suficient pentru PX4, fara a supraincarca serialul.

Cum se porneste:
  ros2 run drone_slam vision_to_mavros.py
  (sau automat din slam_full.launch.py)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VisionToMavros(Node):
    """
    Converteste nav_msgs/Odometry → geometry_msgs/PoseStamped
    si publica pe /mavros/vision_pose/pose.
    """

    def __init__(self):
        super().__init__('vision_to_mavros')

        # ── QoS Profile ────────────────────────────────────────────
        # BEST_EFFORT: nu blocam daca un mesaj se pierde (real-time priority)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscriber: odometria filtrata de la EKF ───────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',       # output-ul EKF
            self.odom_callback,
            qos_best_effort
        )

        # ── Publisher: pozitia vizuala catre MAVROS/Pixhawk ─────────
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose', # MAVROS asculta pe acest topic
            10
        )

        self.get_logger().info(
            'VisionToMavros pornit. '
            'Ascult /odometry/filtered → public /mavros/vision_pose/pose'
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback apelat la fiecare mesaj Odometry de la EKF.
        Extrage pozitia si orientarea si le publica ca PoseStamped.
        """
        pose_stamped = PoseStamped()

        # Pastreaza timestamp-ul original (important pentru PX4!)
        # PX4 foloseste timestamp-ul pentru a estima latenta senzorului
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'map'  # frame global

        # Copiaza pozitia (x, y, z in metri)
        pose_stamped.pose.position = msg.pose.pose.position

        # Copiaza orientarea (quaternion: x, y, z, w)
        pose_stamped.pose.orientation = msg.pose.pose.orientation

        # Publica catre MAVROS
        self.vision_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)

    node = VisionToMavros()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nod oprit de utilizator.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
