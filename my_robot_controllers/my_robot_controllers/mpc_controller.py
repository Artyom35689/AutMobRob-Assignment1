from __future__ import annotations

import math
from typing import List, Optional

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


class MPCControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================
        self.horizon = int(self.declare_parameter("horizon", 12).value)
        self.lookahead_points = int(self.declare_parameter("lookahead_points", 2).value)

        self.nv = int(self.declare_parameter("nv", 5).value)
        self.nw = int(self.declare_parameter("nw", 11).value)

        self.w_track = float(self.declare_parameter("w_track", 5.0).value)
        self.w_terminal = float(self.declare_parameter("w_terminal", 20.0).value)
        self.w_u = float(self.declare_parameter("w_u", 0.05).value)

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

        self.v_min = float(self.declare_parameter("v_min", 0.0).value)
        self.v_max = float(self.declare_parameter("v_max", 1.0).value)
        self.w_max = float(self.declare_parameter("w_max", 1.5).value)

        self.control_rate = float(self.declare_parameter("control_rate", 10.0).value)
        self.dt = 1.0 / max(self.control_rate, 1e-6)

        # State
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        # Candidate grids
        self.v_grid = self._linspace(self.v_min, self.v_max, self.nv)
        self.w_grid = self._linspace(-self.w_max, self.w_max, self.nw)

        # ROS
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"odom={self.odom_topic} cmd_vel={self.cmd_vel_topic} points={len(self.path)} "
            f"N={self.horizon} dt={self.dt:.3f} nv={self.nv} nw={self.nw}"
        )

    @staticmethod
    def _linspace(a: float, b: float, n: int) -> List[float]:
        if n <= 1:
            return [a]
        step = (b - a) / (n - 1)
        return [a + i * step for i in range(n)]

    def publish_cmd(self, v: float, w: float) -> None:
        v = clamp(v, self.v_min, self.v_max)
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

    def closest_waypoint_idx(self) -> int:
        """Nearest waypoint index (simple association)."""
        assert self.x is not None and self.y is not None
        best_i = 0
        best_d2 = None
        for i, (wx, wy) in enumerate(self.path):
            dx = wx - self.x
            dy = wy - self.y
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def rollout_cost(self, v: float, w: float, tx: float, ty: float) -> float:
        """Rollout unicycle model with constant (v,w) for horizon steps."""
        assert self.x is not None and self.y is not None and self.yaw is not None

        x = self.x
        y = self.y
        yaw = self.yaw

        J = 0.0
        for _ in range(self.horizon):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw = wrap_to_pi(yaw + w * self.dt)

            dx = x - tx
            dy = y - ty
            J += self.w_track * (dx * dx + dy * dy)

        dxT = x - tx
        dyT = y - ty
        J += self.w_terminal * (dxT * dxT + dyT * dyT)

        J += self.w_u * (v * v + 0.5 * w * w)
        return J

    def on_timer(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) <= self.goal_tolerance:
            self.get_logger().info("Goal reached. Stopping.")
            self.stop()
            return

        i0 = self.closest_waypoint_idx()
        it = min(i0 + self.lookahead_points, len(self.path) - 1)
        tx, ty = self.path[it]

        best_J = None
        best_v = 0.0
        best_w = 0.0

        for v in self.v_grid:
            for w in self.w_grid:
                J = self.rollout_cost(v, w, tx, ty)
                if best_J is None or J < best_J:
                    best_J = J
                    best_v = v
                    best_w = w

        self.publish_cmd(best_v, best_w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
