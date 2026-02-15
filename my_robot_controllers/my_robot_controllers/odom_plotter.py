from __future__ import annotations

import os
import math
from typing import List, Tuple, Optional
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_waypoints_flat(arr: List[float]) -> List[Tuple[float, float]]:
    if len(arr) < 4 or (len(arr) % 2) != 0:
        raise ValueError("waypoints_flat must be [x1,y1,x2,y2,...] with even length >= 4")
    pts: List[Tuple[float, float]] = []
    for i in range(0, len(arr), 2):
        pts.append((float(arr[i]), float(arr[i + 1])))
    return pts


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def unwrap_yaw(prev_unwrapped: float, new_wrapped: float, prev_wrapped: float) -> float:
    dy = wrap_to_pi(new_wrapped - prev_wrapped)
    return prev_unwrapped + dy


class OdomPlotterNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_plotter")

        # =========================
        # INPUTS
        # =========================
        self.odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)

        default_path = [
            3.0, 0.0,
            6.0, 4.0,
            3.0, 4.0,
            3.0, 1.0,
            0.0, 3.0,
        ]
        self.waypoints_flat = list(self.declare_parameter("waypoints_flat", default_path).value)
        self.waypoints = parse_waypoints_flat(self.waypoints_flat)

        # Radius of "acceptance zone" for every waypoint (draw circles)
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.2).value)

        # =========================
        # SAMPLING (OPTIONAL)
        # =========================
        # If > 0: keep samples at ~sample_rate (useful to match controller Hz on plots)
        # If == 0: keep every odom message
        self.sample_rate = float(self.declare_parameter("sample_rate", 0.0).value)

        # =========================
        # STOP CONDITIONS
        # =========================
        self.stop_lin_eps = float(self.declare_parameter("stop_lin_eps", 0.02).value)    # m/s
        self.stop_ang_eps = float(self.declare_parameter("stop_ang_eps", 0.05).value)   # rad/s
        self.settle_time = float(self.declare_parameter("settle_time", 1.0).value)      # s near goal while stopped
        self.max_duration = float(self.declare_parameter("max_duration", 180.0).value)  # safety timeout

        # =========================
        # OUTPUT
        # =========================
        self.plot_name = str(self.declare_parameter("plot_name", "run").value)
        self.file_prefix = str(self.declare_parameter("file_prefix", "run").value)

        # Default: "plots" relative to current working dir (typically /home/fabian/ros2_ws)
        self.output_dir = str(self.declare_parameter("output_dir", "/home/fabian/ros2_ws/src/plots").value)
        # How often we check stop conditions (NOT used for time axis)
        self.monitor_rate = float(self.declare_parameter("monitor_rate", 10.0).value)

        # =========================
        # DATA BUFFERS
        # =========================
        self.t0: Optional[float] = None           # absolute seconds from odom stamp
        self.last_rel_t: float = 0.0
        self.last_moving_t: float = 0.0
        self.last_dist_goal: float = float("inf")

        self.last_sample_t: Optional[float] = None

        self.ts: List[float] = []
        self.xs: List[float] = []
        self.ys: List[float] = []
        self.yaws_wrapped: List[float] = []
        self.yaws_unwrapped: List[float] = []

        self.saved = False

        # ROS
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.timer = self.create_timer(1.0 / max(self.monitor_rate, 1e-6), self.on_timer)

        self.get_logger().info(
            f"odom_plotter: odom={self.odom_topic} points={len(self.waypoints)} "
            f"tol={self.goal_tolerance} out={self.output_dir} name={self.plot_name} "
            f"sample_rate={self.sample_rate}"
        )

    def _stamp_to_sec(self, msg: Odometry) -> float:
        # Use ROS time stamp (sim time in Gazebo if use_sim_time:=True)
        return Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def _resolve_output_dir(self) -> Path:
        p = Path(self.output_dir).expanduser()
        if not p.is_absolute():
            p = Path.cwd() / p
        return p

    def on_odom(self, msg: Odometry) -> None:
        # -------- time --------
        t_abs = self._stamp_to_sec(msg)
        if self.t0 is None:
            self.t0 = t_abs
        t_rel = t_abs - self.t0
        self.last_rel_t = t_rel

        # optional downsample to ~sample_rate
        if self.sample_rate > 0.0:
            min_dt = 1.0 / max(self.sample_rate, 1e-6)
            if self.last_sample_t is not None and (t_rel - self.last_sample_t) < min_dt:
                return
            self.last_sample_t = t_rel

        # -------- pose --------
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x = float(p.x)
        y = float(p.y)
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        # -------- speeds (stopped detection) --------
        v_lin = float(msg.twist.twist.linear.x)
        w_ang = float(msg.twist.twist.angular.z)
        moving = (abs(v_lin) > self.stop_lin_eps) or (abs(w_ang) > self.stop_ang_eps)
        if moving:
            self.last_moving_t = t_rel

        # -------- goal distance --------
        gx, gy = self.waypoints[-1]
        self.last_dist_goal = math.hypot(gx - x, gy - y)

        # -------- store series --------
        self.ts.append(t_rel)
        self.xs.append(x)
        self.ys.append(y)
        self.yaws_wrapped.append(yaw)

        if not self.yaws_unwrapped:
            self.yaws_unwrapped.append(yaw)
        else:
            prev_unwrapped = self.yaws_unwrapped[-1]
            prev_wrapped = self.yaws_wrapped[-2]
            self.yaws_unwrapped.append(unwrap_yaw(prev_unwrapped, yaw, prev_wrapped))

    def on_timer(self) -> None:
        if self.saved:
            return
        if self.t0 is None:
            return

        # safety timeout
        if self.last_rel_t >= self.max_duration:
            self.get_logger().warn("Plotter: max_duration reached. Saving plots.")
            self.save_plots(reason="timeout")
            rclpy.shutdown()
            return

        # finished: near final goal + stopped for settle_time
        if self.last_dist_goal <= self.goal_tolerance:
            if (self.last_rel_t - self.last_moving_t) >= self.settle_time:
                self.get_logger().info("Plotter: goal reached + settled. Saving plots.")
                self.save_plots(reason="done")
                rclpy.shutdown()
                return

    def save_plots(self, reason: str) -> None:
        if self.saved:
            return
        self.saved = True

        out_dir = self._resolve_output_dir()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not self.ts:
            self.get_logger().warn("Plotter: no data collected, nothing to save.")
            return

        fig, axs = plt.subplots(2, 2, figsize=(12, 9))
        fig.suptitle(self.plot_name, fontsize=14)
        ax_xy = axs[0, 0]
        ax_x = axs[0, 1]
        ax_y = axs[1, 0]
        ax_phi = axs[1, 1]

        # expected polyline from waypoints
        wx = [p[0] for p in self.waypoints]
        wy = [p[1] for p in self.waypoints]

        # 1) XY
        ax_xy.plot(self.xs, self.ys, label="actual")
        ax_xy.plot(wx, wy, "--", label="expected (polyline)")
        ax_xy.scatter(wx, wy, marker="o", label="waypoints")
        for (cx, cy) in self.waypoints:
            ax_xy.add_patch(plt.Circle((cx, cy), self.goal_tolerance, fill=False))

        ax_xy.set_title("XY Trajectory")
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xy.axis("equal")
        ax_xy.grid(True)
        ax_xy.legend()

        # 2) x(t)
        ax_x.plot(self.ts, self.xs)
        ax_x.set_title("x vs time")
        ax_x.set_xlabel("t [s]")
        ax_x.set_ylabel("x [m]")
        ax_x.grid(True)

        # 3) y(t)
        ax_y.plot(self.ts, self.ys)
        ax_y.set_title("y vs time")
        ax_y.set_xlabel("t [s]")
        ax_y.set_ylabel("y [m]")
        ax_y.grid(True)

        # 4) phi(t)
        ax_phi.plot(self.ts, self.yaws_unwrapped)
        ax_phi.set_title("phi (heading) vs time")
        ax_phi.set_xlabel("t [s]")
        ax_phi.set_ylabel("phi [rad]")
        ax_phi.grid(True)

        fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"{self.file_prefix}_{self.plot_name}_{reason}_{stamp}.png"
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        self.get_logger().info(f"Saved plot: {out_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Plotter interrupted. Saving plots.")
        node.save_plots(reason="interrupted")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
