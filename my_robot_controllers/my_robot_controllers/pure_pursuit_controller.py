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


class PurePursuitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================

        # lookahead distance (Ld) used to pick target point on path
        self.lookahead = float(self.declare_parameter("lookahead", 0.8).value)

        # commanded forward speed (can be constant for pure pursuit)
        self.v_cmd = float(self.declare_parameter("v_cmd", 0.35).value)

        # goal reached threshold (distance to final point)
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.1).value)

        # =====================================
        # COMMON PARAMETERS FOR ALL CONTROLLERS
        # =====================================

        # ROS IO
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # path to follow, as flat list of [x1,y1,x2,y2,...]
        default_path = [
            3.0, 0.0,
            6.0, 4.0,
            3.0, 4.0,
            3.0, 1.0,
            0.0, 3.0,
        ]
        self.waypoints_flat = list(self.declare_parameter("waypoints_flat", default_path).value)
        self.path = parse_waypoints_flat(self.waypoints_flat)

        # limits
        self.v_max = float(self.declare_parameter("v_max", 1.0).value)
        self.w_max = float(self.declare_parameter("w_max", 1.5).value)

        # state
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        self.progress_idx = 0  # progress along the path

        # timer + control loop
        self.control_rate = float(self.declare_parameter("control_rate", 20.0).value)
        self.timer = self.create_timer(1.0 / self.control_rate, self.on_timer)

        # simple logs
        self.get_logger().info(f"odom_topic={self.odom_topic} cmd_vel_topic={self.cmd_vel_topic}")
        self.get_logger().info(f"path points={len(self.path)} lookahead={self.lookahead} v_cmd={self.v_cmd}")

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
    
    # parse odometry messages to get current pose
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
    
    def find_lookahead_point(self) -> Tuple[float, float]:
        assert self.x is not None and self.y is not None

        # move progress index forward while we are closer to the next point than to the current one
        while self.progress_idx + 1 < len(self.path):
            px, py = self.path[self.progress_idx]
            nx, ny = self.path[self.progress_idx + 1]

            d0 = math.hypot(px - self.x, py - self.y)
            d1 = math.hypot(nx - self.x, ny - self.y)

            if d1 < d0:
                self.progress_idx += 1
            else:
                break

        # look ahead from the current progress index until we find a point that is at least Ld away
        ld = self.lookahead
        for i in range(self.progress_idx, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= ld:
                self.progress_idx = i
                return px, py

        # if there is no such point, return the last one (goal)
        return self.path[-1]

    
    def on_timer(self) -> None:
        # get odom
        if self.x is None or self.y is None or self.yaw is None:
            return

        # empty path check
        if not self.path:
            self.stop()
            return

        # check if goal reached
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - self.x, gy - self.y)
        if dist_goal <= self.goal_tolerance:
            self.get_logger().info("Goal reached. Stopping.")
            self.stop()
            return

        # pick lookahead point on path
        lx, ly = self.find_lookahead_point()

        # calc errors
        dx = lx - self.x
        dy = ly - self.y
        desired_yaw = math.atan2(dy, dx)
        alpha = wrap_to_pi(desired_yaw - self.yaw)

        # Pure Pursuit control law
        Ld = max(self.lookahead, 1e-5) # avoid division by zero
        kappa = 2.0 * math.sin(alpha) / Ld
        v = clamp(self.v_cmd, 0.0, self.v_max)
        w = clamp(v * kappa, -self.w_max, self.w_max)

        # publish
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()