"""Reference solution for 12.05 — A pure pursuit path follower.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float]
Pose = tuple[float, float, float]


def to_robot_frame(pose: Pose, point: Point) -> Point:
    """World point -> base_link coordinates (x forward, y left) of a robot at ``pose``."""
    x, y, theta = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(theta), math.sin(theta)
    return c * dx + s * dy, -s * dx + c * dy


def curvature(x_r: float, y_r: float) -> float:
    """Pure pursuit: curvature of the arc from base_link (tangent to x) through (x_r, y_r)."""
    d2 = x_r * x_r + y_r * y_r
    return 0.0 if d2 == 0.0 else 2.0 * y_r / d2


def twist_to_wheel_speeds(v: float, w: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]:
    """(v [m/s], omega [rad/s]) -> (left, right) wheel speeds [rad/s]."""
    return (v - w * wheel_separation / 2.0) / wheel_radius, (v + w * wheel_separation / 2.0) / wheel_radius


def lookahead_point(path: Sequence[Point], position: Point, lookahead: float, start_index: int = 0) -> tuple[Point, int]:
    """First point where the path, walked from segment ``start_index`` on, leaves the circle of radius
    ``lookahead`` around ``position``. Returns (point, segment index); (last point, len(path) - 2) if the
    whole rest of the path is inside the circle."""
    px, py = position
    for i in range(start_index, len(path) - 1):
        (ax, ay), (bx, by) = path[i], path[i + 1]
        if math.hypot(bx - px, by - py) < lookahead:
            continue
        dx, dy = bx - ax, by - ay
        fx, fy = ax - px, ay - py
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - lookahead * lookahead
        if a == 0.0:
            return (bx, by), i
        t = (-b + math.sqrt(max(b * b - 4.0 * a * c, 0.0))) / (2.0 * a)
        t = min(max(t, 0.0), 1.0)
        return (ax + t * dx, ay + t * dy), i
    return tuple(path[-1]), len(path) - 2  # type: ignore[return-value]


class PurePursuitController:
    def __init__(
        self,
        path: Sequence[Point],
        lookahead_m: float = 0.3,
        speed_m_s: float = 0.2,
        goal_tolerance_m: float = 0.05,
        approach_dist_m: float = 0.4,
        min_speed_m_s: float = 0.05,
        rotate_threshold_rad: float = math.pi / 4,
        rotate_speed_rad_s: float = 1.0,
        max_angular_rad_s: float = 1.2,
    ) -> None:
        self.path = [tuple(p) for p in path]
        self.lookahead_m = lookahead_m
        self.speed_m_s = speed_m_s
        self.goal_tolerance_m = goal_tolerance_m
        self.approach_dist_m = approach_dist_m
        self.min_speed_m_s = min_speed_m_s
        self.rotate_threshold_rad = rotate_threshold_rad
        self.rotate_speed_rad_s = rotate_speed_rad_s
        self.max_angular_rad_s = max_angular_rad_s
        self.done = False
        self.index = 0  # segment the robot is on; only moves forward

    def compute(self, pose: Pose) -> tuple[float, float]:
        x, y, _ = pose
        gx, gy = self.path[-1]
        if self.done or math.hypot(gx - x, gy - y) <= self.goal_tolerance_m:
            self.done = True
            return 0.0, 0.0
        # progress: the closest path vertex from the current segment on (a short window ahead)
        window = range(self.index, min(self.index + 50, len(self.path) - 1))
        self.index = min(window, key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y))
        carrot, _ = lookahead_point(self.path, (x, y), self.lookahead_m, self.index)
        x_r, y_r = to_robot_frame(pose, carrot)
        angle = math.atan2(y_r, x_r)
        if abs(angle) > self.rotate_threshold_rad:
            return 0.0, math.copysign(self.rotate_speed_rad_s, angle)
        v = self.speed_m_s
        remaining = math.hypot(gx - x, gy - y)
        if remaining < self.approach_dist_m:
            v = max(v * remaining / self.approach_dist_m, self.min_speed_m_s)
        w = v * curvature(x_r, y_r)
        if abs(w) > self.max_angular_rad_s:
            v *= self.max_angular_rad_s / abs(w)
            w = math.copysign(self.max_angular_rad_s, w)
        return v, w
