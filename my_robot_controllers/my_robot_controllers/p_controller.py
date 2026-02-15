from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from my_robot_controllers.controllers_common import (
    parse_waypoints_flat,
    wrap_to_pi,
    clamp,
    yaw_from_quat,
    make_twist,
)


class PControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("p_controller")

        # =======================================
        # SPECIFIC PARAMETERS FOR THIS CONTROLLER
        # =======================================
        self.k_rho = float(self.declare_parameter("k_rho", 1.0).value)          # speed vs distance
        self.k_alpha = float(self.declare_parameter("k_alpha", 1.0).value)      # turn vs heading error
        self.slow_angle = float(self.declare_parameter("slow_angle", 0.9).value)

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
        self.pub = self.create_publisher(type(make_twist(0.0, 0.0)), self.cmd_vel_topic, 10)  # type: ignore
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

    def on_timer(self) -> None:
        # wait for odom
        if self.x is None or self.y is None or self.yaw is None:
            return

        # done
        if self.target_idx >= len(self.path):
            self.stop()
            return

        # current target
        tx, ty = self.path[self.target_idx]
        dx = tx - self.x
        dy = ty - self.y

        dist = math.hypot(dx, dy)
        if dist <= self.goal_tolerance:
            self.target_idx += 1
            if self.target_idx >= len(self.path):
                self.get_logger().info("Done. Stopping.")
                self.stop()
            return

        desired_yaw = math.atan2(dy, dx)
        heading_err = wrap_to_pi(desired_yaw - self.yaw)

        # P-law
        w = self.k_alpha * heading_err
        v = self.k_rho * dist

        # slow down if robot is facing away
        if abs(heading_err) > self.slow_angle:
            v *= 0.2

        self.publish_cmd(v, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
