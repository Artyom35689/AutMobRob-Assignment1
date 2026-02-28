from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from my_robot_controllers.controllers_common import (
    parse_waypoints_flat,
    wrap_to_pi,
    clamp,
    yaw_from_quat,
    make_twist,
)


class PurePursuitControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================
        self.lookahead = float(self.declare_parameter("lookahead", 1.0).value)   # Ld
        self.v_cmd = float(self.declare_parameter("v_cmd", 0.9).value)          # constant speed

        # =====================================
        # COMMON PARAMETERS FOR ALL CONTROLLERS
        # =====================================
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.2).value)

        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value

        default_path = [
            3.0, 0.0,
            6.0, 4.0,
            3.0, 4.0,
            3.0, 1.0,
            0.0, 3.0,
        ]
        self.waypoints_flat = list(self.declare_parameter("waypoints_flat", default_path).value)
        self.path = parse_waypoints_flat(self.waypoints_flat)

        self.v_max = float(self.declare_parameter("v_max", 1.0).value)
        self.w_max = float(self.declare_parameter("w_max", 1.5).value)

        self.control_rate = float(self.declare_parameter("control_rate", 20.0).value)
        self.dt = 1.0 / max(self.control_rate, 1e-6)

        # State
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None
        self.target_idx = 0

        # ROS
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(f"odom={self.odom_topic} cmd_vel={self.cmd_vel_topic} points={len(self.path)}")

    def publish_cmd(self, v: float, w: float) -> None:
        v = clamp(v, 0.0, self.v_max)
        w = clamp(w, -self.w_max, self.w_max)
        self.pub.publish(make_twist(v, w))

    def stop(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def find_lookahead_point(self, from_idx: int) -> Tuple[float, float]:
        """
        Choose the first path point at distance >= lookahead from current position,
        searching only forward starting from `from_idx`.
        This function does NOT modify progress.
        """
        assert self.x is not None and self.y is not None

        ld = max(self.lookahead, 1e-3)

        # Search forward only
        for i in range(max(from_idx, 0), len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= ld:
                return px, py

        # If none found, fall back to the final waypoint
        return self.path[-1]

    def on_timer(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        # If all waypoints completed -> stop
        if self.target_idx >= len(self.path):
            self.get_logger().info("Path completed. Stopping.")
            self.stop()
            return

        # Current mandatory waypoint
        tx, ty = self.path[self.target_idx]
        dist_t = math.hypot(tx - self.x, ty - self.y)

        # HARD GATE:
        # Until we are within goal_tolerance of the current waypoint,
        # we aim at it directly (do NOT jump to lookahead / next waypoint).
        if dist_t > self.goal_tolerance:
            lx, ly = tx, ty
        else:
            # reached current waypoint -> advance
            self.target_idx += 1
            if self.target_idx >= len(self.path):
                self.get_logger().info("Goal reached. Stopping.")
                self.stop()
                return
            lx, ly = self.find_lookahead_point(self.target_idx)

        dx = lx - self.x
        dy = ly - self.y
        desired_yaw = math.atan2(dy, dx)
        alpha = wrap_to_pi(desired_yaw - self.yaw)

        # Pure Pursuit curvature
        Ld = max(self.lookahead, 1e-3)
        kappa = 2.0 * math.sin(alpha) / Ld

        v = clamp(self.v_cmd, 0.0, self.v_max)
        w = v * kappa

        self.publish_cmd(v, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
