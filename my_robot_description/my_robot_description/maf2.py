import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Subscribe to the LiDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS history depth
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('safe_distance', 2.5)  # Safe threshold in meters
        self.safe_distance = self.get_parameter('safe_distance').value

    def scan_callback(self, scan: LaserScan):
        ranges = scan.ranges
        num_points = len(ranges)
        if num_points == 0:
            return  # No data

        # Determine indices for front-left and front-right sectors
        mid_index = num_points // 2
        sector_width = int((60.0/360.0) * num_points)  # 60° sector on each side
        left_start = mid_index
        left_end = min(num_points, mid_index + sector_width)
        right_end = mid_index
        right_start = max(0, mid_index - sector_width)

        # Extract ranges for each sector
        left_sector = [r for r in ranges[left_start:left_end] if not math.isinf(r)]
        right_sector = [r for r in ranges[right_start:right_end] if not math.isinf(r)]

        # Find minimum distances in each sector
        min_left = min(left_sector) if left_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')

        # Prepare a Twist message
        cmd = Twist()
        forward_speed = 0.2  # m/s
        turn_speed = 0.5     # rad/s

        # Check if obstacles are within the safe distance
        if min_left < self.safe_distance or min_right < self.safe_distance:
            cmd.linear.x = 0.0  # Stop moving forward
            if min_left < min_right:
                cmd.angular.z = -turn_speed  # Turn right
                self.get_logger().info(f"Obstacle on left at {min_left:.2f} m -> turning right")
            elif min_right < min_left:
                cmd.angular.z = turn_speed  # Turn left
                self.get_logger().info(f"Obstacle on right at {min_right:.2f} m -> turning left")
            else:
                cmd.angular.z = turn_speed  # Default turn left
                self.get_logger().info(f"Obstacle ahead -> turning left")
        else:
            cmd.linear.x = forward_speed  # Move forward
            cmd.angular.z = 0.0
            self.get_logger().info("Path clear -> moving forward")

        # Publish the command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()