from __future__ import annotations

import math
from dataclasses import dataclass
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


@dataclass
class SegmentMatch:
    seg_idx: int
    proj_x: float
    proj_y: float
    dist: float
    cte_signed: float
    path_yaw: float


def project_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float]:
    """Project P onto segment AB, return projection point."""
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


class StanleyControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("stanley_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================
        self.k_cte = float(self.declare_parameter("k_cte", 1.0).value)
        self.softening = float(self.declare_parameter("softening", 0.3).value)
        self.wheelbase = float(self.declare_parameter("wheelbase", 0.5).value)

        self.v_cmd = float(self.declare_parameter("v_cmd", 0.25).value)

        # protect tan(delta) on diff-drive mapping
        self.delta_max = float(self.declare_parameter("delta_max", 0.7).value)

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

    def find_closest_segment(self) -> SegmentMatch:
        """
        Classic Stanley prerequisite:
        find the closest point on the path (global search over segments).
        """
        assert self.x is not None and self.y is not None

        best: Optional[SegmentMatch] = None
        for i in range(0, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]

            proj_x, proj_y = project_point_to_segment(self.x, self.y, ax, ay, bx, by)
            dist = math.hypot(self.x - proj_x, self.y - proj_y)

            seg_dx = bx - ax
            seg_dy = by - ay
            path_yaw = math.atan2(seg_dy, seg_dx)

            # Signed cross-track error using cross product sign.
            # cross > 0 => robot is left of segment direction (A->B).
            cross = seg_dx * (self.y - proj_y) - seg_dy * (self.x - proj_x)
            sign = 1.0 if cross > 0.0 else -1.0

            # For cmd_vel (w > 0 is left/CCW), we choose CTE sign so that:
            # left of path -> negative CTE -> steering correction turns right.
            cte_signed = -sign * dist

            cand = SegmentMatch(
                seg_idx=i,
                proj_x=proj_x,
                proj_y=proj_y,
                dist=dist,
                cte_signed=cte_signed,
                path_yaw=path_yaw,
            )
            if best is None or cand.dist < best.dist:
                best = cand

        assert best is not None
        return best

    def on_timer(self) -> None:
        if self.x is None or self.y is None or self.yaw is None:
            return

        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) <= self.goal_tolerance:
            self.get_logger().info("Goal reached. Stopping.")
            self.stop()
            return

        m = self.find_closest_segment()

        # Heading error
        heading_err = wrap_to_pi(m.path_yaw - self.yaw)

        # Stanley steering law
        v = clamp(self.v_cmd, 0.0, self.v_max)
        delta = heading_err + math.atan2(self.k_cte * m.cte_signed, v + self.softening)
        delta = wrap_to_pi(delta)
        delta = clamp(delta, -self.delta_max, self.delta_max)

        # Diff-drive mapping: w = v/L * tan(delta)
        L = max(self.wheelbase, 1e-3)
        w = (v / L) * math.tan(delta)

        self.publish_cmd(v, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
