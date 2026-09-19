"""12.05 — A pure pursuit path follower.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.05``.
Only the standard library (``math``) is needed.

Conventions: meters, radians, REP-103 (x forward, y left, counter-clockwise positive).
A pose is ``(x, y, theta)`` in the map frame; a path is a list of ``(x, y)`` points about 5 cm apart,
as a global planner produces it; the last point is the goal.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float]
Pose = tuple[float, float, float]


def to_robot_frame(pose: Pose, point: Point) -> Point:
    """Express a map-frame ``point`` in base_link of a robot at ``pose``.

    Example: robot at (1, 2) facing +y (theta = pi/2); the point (1, 3) is 1 m straight ahead -> (1, 0).
    """
    # TODO(student): translate by -position, then rotate by -theta.
    raise NotImplementedError("to_robot_frame")


def curvature(x_r: float, y_r: float) -> float:
    """Curvature (1/m, + = turn left) of the circular arc that starts at base_link heading along +x and
    passes through (x_r, y_r):  kappa = 2 * y_r / (x_r^2 + y_r^2).  Return 0 for the point (0, 0)."""
    # TODO(student)
    raise NotImplementedError("curvature")


def twist_to_wheel_speeds(v: float, w: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]:
    """(v [m/s], omega [rad/s]) -> (left, right) wheel angular speeds [rad/s] (lesson 09.02)."""
    # TODO(student)
    raise NotImplementedError("twist_to_wheel_speeds")


def lookahead_point(path: Sequence[Point], position: Point, lookahead: float, start_index: int = 0) -> tuple[Point, int]:
    """The "carrot": walk the path from segment ``start_index`` (segment i joins path[i] and path[i+1]).

    * Skip every segment whose END point is closer than ``lookahead`` to ``position``.
    * On the first segment whose end is at least ``lookahead`` away, return the point where the circle of
      radius ``lookahead`` around ``position`` crosses the segment (the crossing nearest the segment's end;
      if the segment starts outside the circle, clamp to its start) and the segment index ``i``.
    * If every remaining end point is inside the circle, return ``(path[-1], len(path) - 2)``.

    Circle-segment crossing: with d = b - a and f = a - position, solve |f + t d|^2 = lookahead^2, i.e.
    (d.d) t^2 + 2 (f.d) t + (f.f - lookahead^2) = 0, take the larger root, clamp t to [0, 1].
    """
    # TODO(student)
    raise NotImplementedError("lookahead_point")


class PurePursuitController:
    """Follows ``path`` and stops at its last point.

    ``compute(pose)`` returns ``(v, w)`` and is called at 50 Hz. Rules, in this order:

    1. Goal: if the robot is within ``goal_tolerance_m`` of the last path point (or already ``done``),
       set ``self.done = True`` and return (0, 0) — from then on, always (0, 0).
    2. Progress: keep ``self.index``, the path vertex closest to the robot, searching only FORWARD from
       the previous index (a window of about 50 vertices), so the robot never "jumps back" along the path.
    3. Carrot: ``lookahead_point(path, (x, y), lookahead_m, self.index)``, expressed in base_link.
    4. Rotate in place: if the carrot's bearing ``atan2(y_r, x_r)`` is larger than ``rotate_threshold_rad``
       in magnitude, return (0, +-rotate_speed_rad_s) toward it.
    5. Speed: v = speed_m_s; within ``approach_dist_m`` of the goal (straight-line distance) scale it by
       distance / approach_dist_m, but never below ``min_speed_m_s``.
    6. Steering: w = v * curvature(x_r, y_r). If |w| > max_angular_rad_s, clamp w and scale v down by the
       same factor so the robot still drives the same arc.
    """

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
        self.index = 0

    def compute(self, pose: Pose) -> tuple[float, float]:
        # TODO(student): rules 1-6 of the class docstring.
        raise NotImplementedError("PurePursuitController.compute")
