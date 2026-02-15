#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Robot teleop node started. Use WASD keys to move. Press 'q' to stop.")

    def get_key(self):
        """Reads a single key press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def control_loop(self):
        """Main loop to read keyboard inputs and send velocity commands."""
        twist = Twist()
        speed = 0.5
        turn = 1.0

        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Forward
                twist.linear.x = speed
                twist.angular.z = 0.0
            elif key == 's':  # Backward
                twist.linear.x = -speed
                twist.angular.z = 0.0
            elif key == 'a':  # Left
                twist.linear.x = 0.0
                twist.angular.z = turn
            elif key == 'd':  # Right
                twist.linear.x = 0.0
                twist.angular.z = -turn
            elif key == 'q':  # Stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '':  # CTRL+C
                break
            self.publisher_.publish(twist)
            self.get_logger().info(f"Velocity: linear={twist.linear.x}, angular={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleop()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()