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
        self.lookahead = float(self.declare_parameter("lookahead", 0.8).value)   # Ld
        self.v_cmd = float(self.declare_parameter("v_cmd", 0.35).value)          # constant speed

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
        self.progress_idx = 0

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

    def find_lookahead_point(self) -> Tuple[float, float]:
        """
        Choose a path point at distance >= lookahead from current position.
        We move progress_idx forward so we do not keep aiming behind us.
        """
        assert self.x is not None and self.y is not None

        # advance progress to avoid "going back"
        while self.progress_idx + 1 < len(self.path):
            x0, y0 = self.path[self.progress_idx]
            x1, y1 = self.path[self.progress_idx + 1]
            d0 = math.hypot(x0 - self.x, y0 - self.y)
            d1 = math.hypot(x1 - self.x, y1 - self.y)
            if d1 < d0:
                self.progress_idx += 1
            else:
                break

        # find first point that is far enough
        ld = max(self.lookahead, 1e-3)
        for i in range(self.progress_idx, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= ld:
                self.progress_idx = i
                return px, py

        return self.path[-1]

    def on_timer(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) <= self.goal_tolerance:
            self.get_logger().info("Goal reached. Stopping.")
            self.stop()
            return

        lx, ly = self.find_lookahead_point()

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
