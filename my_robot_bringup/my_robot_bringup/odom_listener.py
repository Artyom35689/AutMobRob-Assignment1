#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Odom Listener Node Started, waiting for messages...")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(f"Position -> x: {position.x:.2f}, y: {position.y:.2f}, z: {position.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
