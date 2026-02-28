from __future__ import annotations

import math
from typing import List, Optional, Tuple

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


def project_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float]:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return ax, ay

    t = (wx * vx + wy * vy) / vv
    t = clamp(t, 0.0, 1.0)
    return ax + t * vx, ay + t * vy


class MPCControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_controller")

        # =======================================
        # SPECIFIC PARAMETERS
        # =======================================
        self.horizon = int(self.declare_parameter("horizon", 15).value)

        self.nv = int(self.declare_parameter("nv", 5).value)
        self.nw = int(self.declare_parameter("nw", 11).value)

        self.w_track = float(self.declare_parameter("w_track", 20.0).value)
        self.w_terminal = float(self.declare_parameter("w_terminal", 60.0).value)
        self.w_u = float(self.declare_parameter("w_u", 0.05).value)

        # =====================================
        # COMMON PARAMETERS
        # =====================================
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.3).value)

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
        self.target_idx = 0

        # Candidate grids
        self.v_grid = self._linspace(self.v_min, self.v_max, self.nv)
        self.w_grid = self._linspace(-self.w_max, self.w_max, self.nw)

        # ROS
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

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

    def rollout_cost(self, v: float, w: float) -> float:
        assert self.x is not None and self.y is not None and self.yaw is not None

        x = self.x
        y = self.y
        yaw = self.yaw

        J = 0.0

        # current segment
        i = clamp(self.target_idx, 0, len(self.path) - 2)
        i = int(i)
        ax, ay = self.path[i]
        bx, by = self.path[i + 1]

        for _ in range(self.horizon):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw = wrap_to_pi(yaw + w * self.dt)

            proj_x, proj_y = project_point_to_segment(x, y, ax, ay, bx, by)
            dx = x - proj_x
            dy = y - proj_y

            J += self.w_track * (dx * dx + dy * dy)

        # terminal cost to next waypoint
        tx, ty = self.path[min(self.target_idx + 1, len(self.path) - 1)]
        dxT = x - tx
        dyT = y - ty
        J += self.w_terminal * (dxT * dxT + dyT * dyT)

        J += self.w_u * (v * v + 0.5 * w * w)
        return J

    def on_timer(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        # Final goal check
        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) <= self.goal_tolerance:
            self.stop()
            return

        # Progress gate
        nx, ny = self.path[min(self.target_idx + 1, len(self.path) - 1)]
        if math.hypot(nx - self.x, ny - self.y) <= self.goal_tolerance:
            self.target_idx += 1
            if self.target_idx >= len(self.path) - 1:
                return

        best_J = None
        best_v = 0.0
        best_w = 0.0

        for v in self.v_grid:
            for w in self.w_grid:
                J = self.rollout_cost(v, w)
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