"""12.10 — The collision monitor and the keepout filter, exactly like Nav2.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.10``.
Only the standard library and numpy are needed.

Two independent safety mechanisms:

* ``nav2_collision_monitor`` sits between ``cmd_vel_smoothed`` and ``/cmd_vel`` and overrides the
  command from raw sensor points. Four action types: ``stop``, ``slowdown``, ``limit`` and
  ``approach``. You implement the geometry and the four actions.
* ``nav2_costmap_2d::KeepoutFilter`` merges a second map (a "filter mask") into a costmap so the
  planner never routes through a marked zone. You implement the conversion and the merge.

Cost values are Nav2's: 0 FREE_SPACE, 253 INSCRIBED_INFLATED_OBSTACLE, 254 LETHAL_OBSTACLE,
255 NO_INFORMATION.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from numpy.typing import NDArray

FREE_SPACE = 0
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Velocity:
    """A planar twist: ``v`` m/s along x, ``w`` rad/s about z."""

    v: float = 0.0
    w: float = 0.0

    def scaled(self, ratio: float) -> Velocity:
        return Velocity(self.v * ratio, self.w * ratio)

    @property
    def magnitude(self) -> float:
        return math.hypot(self.v, self.w)

    def is_safer_than(self, other: Velocity) -> bool:
        return self.magnitude < other.magnitude


class ActionType(str, Enum):
    NONE = "none"
    STOP = "stop"
    SLOWDOWN = "slowdown"
    LIMIT = "limit"
    APPROACH = "approach"


@dataclass
class MonitorAction:
    velocity: Velocity
    action_type: ActionType = ActionType.NONE
    polygon_name: str = ""


def project_state(dt: float, pose: tuple[float, float, float], vel: Velocity) -> tuple[float, float, float]:
    """Given: move ``pose`` by ``vel`` for ``dt`` — Nav2 steps along the old heading, then rotates."""
    x, y, theta = pose
    return (x + vel.v * math.cos(theta) * dt,
            y + vel.v * math.sin(theta) * dt,
            theta + vel.w * dt)


def transform_points(pose: tuple[float, float, float], points: NDArray[np.floating]) -> NDArray[np.floating]:
    """Given: express ``points`` (in the robot's start frame) in the frame of the robot at ``pose``."""
    x, y, theta = pose
    c, s = math.cos(-theta), math.sin(-theta)
    shifted = points - np.array([x, y])
    return np.column_stack([shifted[:, 0] * c - shifted[:, 1] * s,
                            shifted[:, 0] * s + shifted[:, 1] * c])


# ==================================================================================================
#  Your turn
# ==================================================================================================
def points_in_polygon(polygon: NDArray[np.floating], points: NDArray[np.floating]) -> int:
    """How many ``points`` lie inside the closed polygon.

    ``polygon`` is ``(n, 2)`` vertices in order, the last joined back to the first; ``points`` is
    ``(m, 2)``. Ray casting: a point is inside when a ray from it crosses the edges an odd number
    of times. For each edge ``(x1, y1) -> (x2, y2)`` and a point ``(x, y)``:

        crosses = (y1 > y) != (y2 > y)
        x_at_y  = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        toggle the point's "inside" flag when ``crosses and x < x_at_y``

    An empty ``points`` array must return 0. Points exactly on an edge may go either way.
    """
    # TODO(student)
    raise NotImplementedError("points_in_polygon")


@dataclass
class SafetyPolygon:
    """One zone of the collision monitor. Parameter names are Nav2's."""

    name: str
    points: NDArray[np.floating]
    action_type: ActionType
    min_points: int = 4
    slowdown_ratio: float = 0.5
    linear_limit: float = 0.4
    angular_limit: float = 0.5
    time_before_collision: float = 1.2
    simulation_time_step: float = 0.1
    enabled: bool = True

    def points_inside(self, points: NDArray[np.floating]) -> int:
        return points_in_polygon(self.points, points)

    def collision_time(self, points: NDArray[np.floating], vel: Velocity) -> float:
        """Seconds until this polygon would touch ``min_points`` of ``points``, or -1.0 if never.

        Transcription of ``Polygon::getCollisionTime``:

        1. If ``min_points`` or more points are already inside, return 0.0.
        2. Start at ``pose = (0, 0, 0)`` and ``time = 0.0``.
        3. While ``time <= time_before_collision``:
             a. ``pose = project_state(simulation_time_step, pose, vel)``  — move FIRST,
             b. if ``points_inside(transform_points(pose, points)) >= min_points``: return ``time``,
             c. ``time += simulation_time_step``.
        4. Return -1.0.

        Note the order in step 3: the pose is advanced before the check, so the returned time is
        one simulation step behind the pose. That is what Nav2 does, and the checker expects it.
        """
        # TODO(student)
        raise NotImplementedError("SafetyPolygon.collision_time")


@dataclass
class CollisionMonitor:
    """``cmd_vel_smoothed`` in, ``/cmd_vel`` out."""

    polygons: list[SafetyPolygon] = field(default_factory=list)

    def process(self, cmd_vel: Velocity, points: NDArray[np.floating]) -> MonitorAction:
        """Apply every enabled polygon and return the SLOWEST requirement.

        Start from ``MonitorAction(cmd_vel)`` (action NONE, empty polygon name). For each enabled
        polygon work out a candidate velocity:

        * ``APPROACH``: ``t = polygon.collision_time(points, cmd_vel)``; if ``t >= 0`` the candidate
          is ``cmd_vel.scaled(t / polygon.time_before_collision)``. (No collision -> no candidate.)
        * otherwise, only when ``polygon.points_inside(points) >= polygon.min_points``:
            - ``STOP``      -> ``Velocity(0.0, 0.0)``
            - ``SLOWDOWN``  -> ``cmd_vel.scaled(polygon.slowdown_ratio)``
            - ``LIMIT``     -> each component clamped to ``linear_limit`` / ``angular_limit``,
              **keeping its sign** (``math.copysign``)

        Keep a candidate only when ``candidate.is_safer_than(current action's velocity)``, and
        record the polygon's name and action type with it.
        """
        # TODO(student)
        raise NotImplementedError("CollisionMonitor.process")


def mask_cost(value: int, base: float = 0.0, multiplier: float = 1.0) -> int:
    """One filter-mask cell (an OccupancyGrid value: -1, or 0..100) as a costmap cost.

    * ``value < 0`` (NO_INFORMATION in an OccupancyGrid) -> ``NO_INFORMATION`` (255).
    * otherwise the filter-space value is ``base + multiplier * value``, clamped to 0..100, and
      scaled linearly onto 0..``LETHAL_OBSTACLE``. Round to the nearest integer.

    So a keepout mask (``base: 0.0``, ``multiplier: 1.0``) turns 100 into 254 and 50 into 127.
    """
    # TODO(student)
    raise NotImplementedError("mask_cost")


def apply_keepout(master: NDArray[np.uint8], mask: NDArray[np.integer],
                  base: float = 0.0, multiplier: float = 1.0) -> NDArray[np.uint8]:
    """A NEW costmap with the keepout filter applied (``KeepoutFilter::process``).

    Same shape for ``master`` and ``mask``. Cell by cell:

    * mask value < 0: leave the master cell exactly as it is.
    * otherwise compute ``cost = mask_cost(value, base, multiplier)`` and write it into the new
      costmap when ``cost > master`` **or** ``master == NO_INFORMATION``.

    That second condition is why this is not a plain maximum: a filter may turn unexplored space
    (255) into a lower, known cost. Do not modify ``master``.
    """
    # TODO(student)
    raise NotImplementedError("apply_keepout")
