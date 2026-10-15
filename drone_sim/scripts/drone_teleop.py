#!/usr/bin/env python3
"""
Node: drone_teleop.py (drone_sim)
==================================
Keyboard teleoperation for drone simulation:
  - T: Arm & Takeoff to 2.0m
  - L: Land
  - W / S: Move Forward / Backward
  - A / D: Move Left / Right
  - R / F: Ascend / Descend
  - Q / E: Rotate (Yaw) Left / Right (pan camera across room)
  - Space: Stop / Hover in place
  - Ctrl+C: Exit
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


INSTRUCTIONS = """
======================================================
           DRONE KEYBOARD TELEOPERATION
======================================================
  [T] : Takeoff (2.0 m)       [L] : Land
  
  [W] : Forward               [S] : Backward
  [A] : Strafe Left           [D] : Strafe Right
  [R] : Ascend (Up)           [F] : Descend (Down)
  
  [Q] : Yaw Left (Turn)       [E] : Yaw Right (Turn)
  
  [Space] : Stop / Hover in Place
  [Ctrl+C] : Quit
======================================================
"""


class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')

        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # Clients for MAVROS commands
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.linear_speed = 0.6
        self.vertical_speed = 0.5
        self.yaw_speed = 0.6

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_linear_z = 0.0
        self.target_angular_z = 0.0

        # Periodic timer to publish setpoints continuously (required by PX4 offboard)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('DroneTeleop initialized.')

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = float(self.target_linear_x)
        twist.linear.y = float(self.target_linear_y)
        twist.linear.z = float(self.target_linear_z)
        twist.angular.z = float(self.target_angular_z)
        self.vel_pub.publish(twist)

    def trigger_takeoff(self):
        self.get_logger().info('Arming and Taking off to 2.0m...')
        # Arm
        if self.arming_client.wait_for_service(timeout_sec=2.0):
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
        # Takeoff
        if self.takeoff_client.wait_for_service(timeout_sec=2.0):
            req = CommandTOL.Request()
            req.altitude = 2.0
            self.takeoff_client.call_async(req)

    def trigger_land(self):
        self.get_logger().info('Landing...')
        if self.land_client.wait_for_service(timeout_sec=2.0):
            req = CommandTOL.Request()
            self.land_client.call_async(req)

    def trigger_offboard(self):
        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.mode_client.call_async(req)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = DroneTeleop()

    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)

            if key == '\x03':  # Ctrl+C
                break
            elif key.lower() == 't':
                node.trigger_takeoff()
            elif key.lower() == 'l':
                node.trigger_land()
            elif key.lower() == 'o':
                node.trigger_offboard()
            elif key.lower() == 'w':
                node.trigger_offboard()
                node.target_linear_x = node.linear_speed
                print(f"Moving Forward ({node.target_linear_x:.1f} m/s)")
            elif key.lower() == 's':
                node.trigger_offboard()
                node.target_linear_x = -node.linear_speed
                print(f"Moving Backward ({node.target_linear_x:.1f} m/s)")
            elif key.lower() == 'a':
                node.trigger_offboard()
                node.target_linear_y = node.linear_speed
                print(f"Strafing Left ({node.target_linear_y:.1f} m/s)")
            elif key.lower() == 'd':
                node.trigger_offboard()
                node.target_linear_y = -node.linear_speed
                print(f"Strafing Right ({node.target_linear_y:.1f} m/s)")
            elif key.lower() == 'r':
                node.trigger_offboard()
                node.target_linear_z = node.vertical_speed
                print(f"Ascending (+{node.target_linear_z:.1f} m/s)")
            elif key.lower() == 'f':
                node.trigger_offboard()
                node.target_linear_z = -node.vertical_speed
                print(f"Descending ({node.target_linear_z:.1f} m/s)")
            elif key.lower() == 'q':
                node.trigger_offboard()
                node.target_angular_z = node.yaw_speed
                print(f"Turning Left (+{node.target_angular_z:.1f} rad/s) - Scanning Room")
            elif key.lower() == 'e':
                node.trigger_offboard()
                node.target_angular_z = -node.yaw_speed
                print(f"Turning Right ({node.target_angular_z:.1f} rad/s) - Scanning Room")
            elif key == ' ':
                node.target_linear_x = 0.0
                node.target_linear_y = 0.0
                node.target_linear_z = 0.0
                node.target_angular_z = 0.0
                print("Hovering / Stopped.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.target_linear_x = 0.0
        node.target_linear_y = 0.0
        node.target_linear_z = 0.0
        node.target_angular_z = 0.0
        node.timer_callback()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
