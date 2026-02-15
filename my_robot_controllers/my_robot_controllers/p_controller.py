from __future__ import annotations

import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# helper function to parse waypoints from flat list of parameters
def parse_waypoints_flat(arr: List[float]) -> List[Tuple[float, float]]:
    if len(arr) < 2 or (len(arr) % 2) != 0:
        raise ValueError("waypoints_flat must be [x1,y1,x2,y2,...] with even length >= 2")
    pts = []
    for i in range(0, len(arr), 2):
        pts.append((float(arr[i]), float(arr[i + 1])))
    return pts

# wrap angle to [-pi, pi]
def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# clip a value to a specified range
def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))

# helper function to get yaw angle from quaternion
def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)

class PControllerNode(Node):
    def __init__(self):
        super().__init__("p_waypoint_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================
        self.k_rho = float(self.declare_parameter("k_rho", 1.0).value)              # speed vs distance
        self.k_alpha = float(self.declare_parameter("k_alpha", 1.0).value)          # turn vs heading error
        self.slow_angle = float(self.declare_parameter("slow_angle", 0.9).value)    # rad

        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.1).value)      # distance to waypoint to consider it reached  

        # =====================================
        # COMMON PARAMETERS FOR ALL CONTROLLERS
        # =====================================

        # ROS IO
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # path to follow, as flat list of [x1,y1,x2,y2,...]
        default_path = [3.0, 0.0,
                        6.0, 4.0,
                        3.0, 4.0,
                        3.0, 1.0,
                        0.0, 3.0]
        
        self.waypoints_flat = list(self.declare_parameter("waypoints_flat", default_path).value)
        self.waypoints = parse_waypoints_flat(self.waypoints_flat)

        # limits
        self.v_max = float(self.declare_parameter("v_max", 1.0).value)
        self.w_max = float(self.declare_parameter("w_max", 1.5).value)

        # state
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.target_idx = 0

        # timer + control loop
        self.control_rate = self.declare_parameter("control_rate", 20.0).value
        self.timer = self.create_timer(1.0 / self.control_rate, self.on_timer)

        # simple logs
        self.get_logger().info(f"odom_topic={self.odom_topic} cmd_vel_topic={self.cmd_vel_topic}")
        self.get_logger().info(f"waypoints={self.waypoints}")
    
    # parse odometry messages to get current pose
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
    
    def on_timer(self):
        if self.x is None or self.y is None or self.yaw is None:
            return  # no odom received yet
        
        if self.target_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            return
        
        # measure position error
        tx,ty = self.waypoints[self.target_idx]
        dx = tx - self.x
        dy = ty - self.y
        pos_error = math.sqrt(dx*dx + dy*dy)

        if pos_error <= self.goal_tolerance:
            self.target_idx += 1
            self.get_logger().info(f"Waypoint {self.target_idx} reached.")
            return
        
        # measure angele error
        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.yaw
        angle_error = wrap_to_pi(angle_error)

        # compute control inputs
        v = self.k_rho * pos_error
        w = self.k_alpha * angle_error

        # clamp control inputs
        v = clamp(v, 0.0, self.v_max)
        w = clamp(w, -self.w_max, self.w_max)

        # publish control inputs
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()