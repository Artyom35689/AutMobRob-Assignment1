from __future__ import annotations

import math
from typing import List, Tuple

from geometry_msgs.msg import Twist


def parse_waypoints_flat(arr: List[float]) -> List[Tuple[float, float]]:
    """
    Parse [x1,y1,x2,y2,...] -> [(x1,y1),(x2,y2),...]
    Requires at least 2 points.
    """
    if len(arr) < 4 or (len(arr) % 2) != 0:
        raise ValueError("waypoints_flat must be [x1,y1,x2,y2,...] with even length >= 4")

    pts: List[Tuple[float, float]] = []
    for i in range(0, len(arr), 2):
        pts.append((float(arr[i]), float(arr[i + 1])))
    return pts


def wrap_to_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, v_min: float, v_max: float) -> float:
    """Clamp v into [v_min, v_max]."""
    return max(v_min, min(v_max, v))


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion -> yaw (ROS convention, yaw around Z)."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def make_twist(v: float, w: float) -> Twist:
    """Create Twist from (v,w)."""
    t = Twist()
    t.linear.x = float(v)
    t.angular.z = float(w)
    return t
