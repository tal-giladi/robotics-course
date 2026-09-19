"""12.10 — The safety layers below the planner: collision monitor, keepout zones, speed zones.

    python 12-navigation/code/safety.py            # the four actions, a wall approach, a keepout map
    python 12-navigation/code/safety.py --no-plot

Three independent mechanisms, in the order a command meets them:

1. ``velocity_smoother``  — acceleration and speed limits (12.05/12.08).
2. ``collision_monitor``  — a *reflex* between ``cmd_vel_smoothed`` and ``/cmd_vel``: it watches
   raw sensor points and stops, slows or limits the robot no matter what the planner wanted.
   Modelled here to follow ``nav2_collision_monitor`` 1.3.x (``Polygon::getCollisionTime`` and
   ``CollisionMonitor::processApproach``).
3. costmap **filters** — ``KeepoutFilter`` and ``SpeedFilter``: a second map ("filter mask") that
   marks places the robot must not enter or must go slowly through. They change the *plan*, which
   is prevention, not reflex.

None of the three is a functional-safety e-stop. See the lesson.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np
from numpy.typing import NDArray

import nav_common

FREE_SPACE = 0
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


# --- velocities and poses -------------------------------------------------------------------------
@dataclass(frozen=True)
class Velocity:
    """A planar twist. ``v`` in m/s along x, ``w`` in rad/s about z."""

    v: float = 0.0
    w: float = 0.0

    def scaled(self, ratio: float) -> Velocity:
        return Velocity(self.v * ratio, self.w * ratio)

    @property
    def magnitude(self) -> float:
        return math.hypot(self.v, self.w)

    def is_safer_than(self, other: Velocity) -> bool:
        """Nav2 keeps the slowest requirement of all polygons."""
        return self.magnitude < other.magnitude


def project_state(dt: float, pose: tuple[float, float, float], vel: Velocity) -> tuple[float, float, float]:
    """Move ``pose`` by ``vel`` for ``dt`` (Nav2's ``projectState``: straight step, then rotate)."""
    x, y, theta = pose
    return (x + vel.v * math.cos(theta) * dt,
            y + vel.v * math.sin(theta) * dt,
            theta + vel.w * dt)


def transform_points(pose: tuple[float, float, float], points: NDArray[np.floating]) -> NDArray[np.floating]:
    """Express ``points`` (in the robot's start frame) in the frame of the robot now at ``pose``."""
    x, y, theta = pose
    c, s = math.cos(-theta), math.sin(-theta)
    shifted = points - np.array([x, y])
    return np.column_stack([shifted[:, 0] * c - shifted[:, 1] * s,
                            shifted[:, 0] * s + shifted[:, 1] * c])


# --- the collision monitor ---------------------------------------------------------------------
class ActionType(str, Enum):
    NONE = "none"
    STOP = "stop"
    SLOWDOWN = "slowdown"
    LIMIT = "limit"
    APPROACH = "approach"


def points_in_polygon(polygon: NDArray[np.floating], points: NDArray[np.floating]) -> int:
    """How many ``points`` lie inside the closed ``polygon`` (ray casting, as Nav2 does)."""
    if len(points) == 0:
        return 0
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at_y = (x2 - x1) * (y - y1) / np.where(y2 - y1 == 0, np.nan, y2 - y1) + x1
        inside ^= crosses & (x < x_at_y)
    return int(inside.sum())


@dataclass
class SafetyPolygon:
    """One zone of ``nav2_collision_monitor``, with the parameter names from nav2_params.yaml."""

    name: str
    points: NDArray[np.floating]
    action_type: ActionType
    min_points: int = 4
    slowdown_ratio: float = 0.5           # action_type: slowdown
    linear_limit: float = 0.4             # action_type: limit
    angular_limit: float = 0.5            # action_type: limit
    time_before_collision: float = 1.2    # action_type: approach
    simulation_time_step: float = 0.1
    enabled: bool = True

    @staticmethod
    def rectangle(name: str, action_type: ActionType, length: float, width: float,
                  **kwargs: Any) -> SafetyPolygon:
        half_l, half_w = length / 2, width / 2
        points = np.array([[half_l, half_w], [half_l, -half_w], [-half_l, -half_w], [-half_l, half_w]])
        return SafetyPolygon(name, points, action_type, **kwargs)

    @staticmethod
    def circle(name: str, action_type: ActionType, radius: float, segments: int = 24,
               **kwargs: Any) -> SafetyPolygon:
        angles = np.linspace(0, 2 * math.pi, segments, endpoint=False)
        return SafetyPolygon(name, np.column_stack([radius * np.cos(angles), radius * np.sin(angles)]),
                             action_type, **kwargs)

    def points_inside(self, points: NDArray[np.floating]) -> int:
        return points_in_polygon(self.points, points)

    def collision_time(self, points: NDArray[np.floating], vel: Velocity) -> float:
        """Seconds until the footprint would touch a point, or -1.0 if it would not.

        Transcription of ``Polygon::getCollisionTime``: the robot is projected forward one
        ``simulation_time_step`` at a time, the points are re-expressed in the robot's new frame,
        and the first step with at least ``min_points`` inside gives the answer.
        """
        if self.points_inside(points) >= self.min_points:
            return 0.0
        pose = (0.0, 0.0, 0.0)
        time = 0.0
        while time <= self.time_before_collision + 1e-9:
            pose = project_state(self.simulation_time_step, pose, vel)
            if self.points_inside(transform_points(pose, points)) >= self.min_points:
                return time
            time += self.simulation_time_step
        return -1.0


@dataclass
class MonitorAction:
    velocity: Velocity
    action_type: ActionType = ActionType.NONE
    polygon_name: str = ""


@dataclass
class CollisionMonitor:
    """``cmd_vel_smoothed`` in, ``/cmd_vel`` out. The slowest requirement of all polygons wins."""

    polygons: list[SafetyPolygon] = field(default_factory=list)

    def process(self, cmd_vel: Velocity, points: NDArray[np.floating]) -> MonitorAction:
        action = MonitorAction(cmd_vel)
        for polygon in self.polygons:
            if not polygon.enabled:
                continue
            candidate: Velocity | None = None
            if polygon.action_type is ActionType.APPROACH:
                collision_time = polygon.collision_time(points, cmd_vel)
                if collision_time >= 0.0:
                    candidate = cmd_vel.scaled(collision_time / polygon.time_before_collision)
            elif polygon.points_inside(points) >= polygon.min_points:
                if polygon.action_type is ActionType.STOP:
                    candidate = Velocity(0.0, 0.0)
                elif polygon.action_type is ActionType.SLOWDOWN:
                    candidate = cmd_vel.scaled(polygon.slowdown_ratio)
                elif polygon.action_type is ActionType.LIMIT:
                    candidate = Velocity(
                        math.copysign(min(abs(cmd_vel.v), polygon.linear_limit), cmd_vel.v),
                        math.copysign(min(abs(cmd_vel.w), polygon.angular_limit), cmd_vel.w))
            if candidate is not None and candidate.is_safer_than(action.velocity):
                action = MonitorAction(candidate, polygon.action_type, polygon.name)
        return action


def karmel_footprint_approach(inflate: float = 0.01) -> SafetyPolygon:
    """karmel's shipped ``FootprintApproach`` polygon: the costmap footprint, approach action."""
    from robotlab.config import load_config

    cfg = load_config()
    length = cfg.chassis.length_m + 2 * inflate
    width = cfg.drive.wheel_separation_m + cfg.drive.wheel_width_m + 2 * inflate
    return SafetyPolygon.rectangle("FootprintApproach", ActionType.APPROACH, length, width,
                                   min_points=6, time_before_collision=1.2, simulation_time_step=0.1)


def wall_points(distance_m: float, half_width: float = 0.6, count: int = 60) -> NDArray[np.floating]:
    """A wall across the robot's path, ``distance_m`` ahead, as LiDAR returns in ``base_link``."""
    ys = np.linspace(-half_width, half_width, count)
    return np.column_stack([np.full(count, distance_m), ys])


# --- costmap filters ------------------------------------------------------------------------------
def mask_cost(value: int, base: float = 0.0, multiplier: float = 1.0) -> int:
    """One filter-mask cell (an OccupancyGrid value: -1, or 0..100) as a costmap cost.

    ``filter space = base + multiplier * value`` is Nav2's conversion (CostmapFilterInfo); for a
    keepout filter it is used with ``base: 0.0`` and ``multiplier: 1.0``, so the mask value *is*
    the occupancy percentage, and 100 % means "lethal, never plan through here".
    """
    if value < 0:
        return NO_INFORMATION
    space = base + multiplier * value
    return int(round(min(max(space, 0.0), 100.0) * LETHAL_OBSTACLE / 100.0))


def apply_keepout(master: NDArray[np.uint8], mask: NDArray[np.integer],
                  base: float = 0.0, multiplier: float = 1.0) -> NDArray[np.uint8]:
    """A new costmap with the keepout filter applied (``KeepoutFilter::process``).

    A mask cell of -1 (NO_INFORMATION) leaves the master cell alone. Otherwise the converted cost
    replaces the master cell when it is greater than what is there, or when the master cell is
    unknown. Note it is **not** a plain maximum: the filter can turn unknown space into known.
    """
    out = master.copy()
    costs = np.array([mask_cost(int(v), base, multiplier) for v in np.ravel(mask)],
                     dtype=np.uint8).reshape(mask.shape)
    valid = np.asarray(mask) >= 0
    replace = valid & ((costs > master) | (master == NO_INFORMATION))
    out[replace] = costs[replace]
    return out


def speed_limit(value: int, base: float = 0.0, multiplier: float = 1.0,
                percentage: bool = True, max_speed_m_s: float = 0.3) -> float:
    """``SpeedFilter``: a mask cell as a speed limit in m/s. 0 % (or value 0) means "no limit"."""
    if value <= 0:
        return max_speed_m_s
    space = base + multiplier * value
    return max_speed_m_s * space / 100.0 if percentage else min(space, max_speed_m_s)


def rasterize(grid: Any, polygons: list[NDArray[np.floating]], value: int = 100) -> NDArray[np.int8]:
    """Draw world-frame polygons into a filter mask the size of ``grid`` (an ``OccupancyGrid``)."""
    mask = np.zeros(grid.data.shape, dtype=np.int8)
    rows, cols = np.indices(grid.data.shape)
    xs, ys = grid.cell_to_world(rows, cols)
    centres = np.column_stack([np.ravel(xs), np.ravel(ys)])
    for polygon in polygons:
        inside = np.zeros(len(centres), dtype=bool)
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            crosses = (y1 > centres[:, 1]) != (y2 > centres[:, 1])
            with np.errstate(divide="ignore", invalid="ignore"):
                x_at_y = (x2 - x1) * (centres[:, 1] - y1) / np.where(y2 - y1 == 0, np.nan, y2 - y1) + x1
            inside ^= crosses & (centres[:, 0] < x_at_y)
        mask[inside.reshape(grid.data.shape)] = value
    return mask


def rectangle(x_min: float, y_min: float, x_max: float, y_max: float) -> NDArray[np.floating]:
    return np.array([[x_min, y_min], [x_max, y_min], [x_max, y_max], [x_min, y_max]])


# --- demo ------------------------------------------------------------------------------------------
#: "the floor is wet in front of the study door" — a zone the planner may not route through.
KEEPOUT_ZONES = [rectangle(1.0, 2.2, 2.4, 3.0)]
#: a zone accidentally drawn over the goal itself: the classic "why is my goal rejected?"
KEEPOUT_OVER_THE_GOAL = [rectangle(0.4, 3.4, 1.6, 4.2)]


def approach_table(polygon: SafetyPolygon, cmd: Velocity, distances: list[float]) -> list[tuple[float, float, float]]:
    """(distance, collision time, output speed) for a wall straight ahead."""
    monitor = CollisionMonitor([polygon])
    rows = []
    for distance in distances:
        points = wall_points(distance)
        rows.append((distance, polygon.collision_time(points, cmd), monitor.process(cmd, points).velocity.v))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Collision monitor and costmap filters for karmel.")
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    import costmap as cm
    import grid_planning as gp
    from robotlab.config import load_config
    from robotlab.sim import World

    cfg = load_config()
    cmd = Velocity(args.speed, 0.0)
    approach = karmel_footprint_approach()
    print(f"karmel FootprintApproach: {cfg.chassis.length_m + 0.02:.2f} x "
          f"{cfg.drive.wheel_separation_m + cfg.drive.wheel_width_m + 0.02:.2f} m, "
          f"min_points 6, time_before_collision {approach.time_before_collision} s, "
          f"simulation_time_step {approach.simulation_time_step} s\n")

    print(f"approach action, wall straight ahead, commanded {cmd.v:.2f} m/s")
    print("  distance   time to collision   /cmd_vel out")
    for distance, collision_time, out in approach_table(approach, cmd, [1.5, 0.8, 0.5, 0.4, 0.3, 0.2, 0.15, 0.125]):
        pretty = "never" if collision_time < 0 else f"{collision_time:.1f} s"
        print(f"   {distance:5.3f} m   {pretty:>15}   {out:.3f} m/s")

    print("\nthe other three actions, same wall at 0.30 m")
    others = [
        SafetyPolygon.rectangle("StopZone", ActionType.STOP, 0.55, 0.45, min_points=4),
        SafetyPolygon.rectangle("SlowZone", ActionType.SLOWDOWN, 0.9, 0.6, min_points=4, slowdown_ratio=0.4),
        SafetyPolygon.circle("LimitZone", ActionType.LIMIT, 0.5, min_points=4,
                             linear_limit=0.1, angular_limit=0.4),
    ]
    points = wall_points(0.30)
    for polygon in others:
        action = CollisionMonitor([polygon]).process(Velocity(0.25, 0.8), points)
        print(f"  {polygon.name:<10} {polygon.action_type.value:<9} {polygon.points_inside(points):3d} points inside"
              f"  -> v {action.velocity.v:.3f} m/s, w {action.velocity.w:.3f} rad/s")

    print("\nall four together (the slowest requirement wins)")
    monitor = CollisionMonitor([approach, *others])
    for distance in (1.0, 0.6, 0.4, 0.25):
        action = monitor.process(Velocity(0.25, 0.8), wall_points(distance))
        print(f"  wall at {distance:.2f} m -> {action.action_type.value:<9} by {action.polygon_name or '-':<16}"
              f" v {action.velocity.v:.3f} m/s, w {action.velocity.w:.3f} rad/s")

    # --- keepout zone ------------------------------------------------------------------------------
    world = World.apartment()
    grid = world.to_occupancy_grid(nav_common.GRID_RESOLUTION)
    costs = cm.inflate(cm.static_layer(grid), grid.resolution, cfg.chassis.footprint_radius_m, 3.0, 0.8)
    start, goal = nav_common.START_XY, nav_common.STUDY_XY

    def plan_on(costmap: NDArray[np.uint8]) -> tuple[float, NDArray[np.floating] | None]:
        blocked, extra = cm.cost_to_planner_penalty(costmap)
        result = gp.astar(blocked, grid.world_to_cell(*start), grid.world_to_cell(*goal), cell_cost=extra)
        if not result.path:
            return math.inf, None
        path = gp.densify(gp.cells_to_world(grid, gp.shortcut(costmap >= 120, result.path)), 0.05)
        return gp.path_cost(result.path) * grid.resolution, path

    mask = rasterize(grid, KEEPOUT_ZONES)
    with_keepout = apply_keepout(costs, mask)
    print(f"\nkeepout filter: {int((mask == 100).sum())} mask cells "
          f"({(mask == 100).sum() * grid.resolution ** 2:.2f} m2) become cost 254 in the costmap")
    plans = {}
    for name, costmap in (("without keepout", costs), ("with keepout", with_keepout)):
        length, path = plan_on(costmap)
        plans[name] = path
        print(f"  living room -> study, {name:<15} {length:5.2f} m")
    length, _ = plan_on(apply_keepout(costs, rasterize(grid, KEEPOUT_OVER_THE_GOAL)))
    print("  with a keepout drawn over the goal itself:   "
          + ("NO VALID PATH (Nav2: error 206, GOAL_OCCUPIED)" if length == math.inf else f"{length:.2f} m"))

    print("\nspeed zones (SpeedFilter in percentage mode, karmel's 0.3 m/s ceiling)")
    for value in (0, 20, 30, 50, 100):
        note = "   (0 and 100 both mean 'full speed here')" if value in (0, 100) else ""
        print(f"  mask value {value:3d} % -> {speed_limit(value):.2f} m/s{note}")

    if not args.no_plot:
        import matplotlib.pyplot as plt
        from robotlab.sim import viz

        fig, ax = viz.new_axes(world, title="12.10 keepout zone reroutes the global plan")
        viz.draw_world(ax, world, show_landmarks=False)
        ax.imshow(np.where(mask > 0, 1.0, np.nan), origin="lower", extent=grid.extent,
                  cmap="autumn", alpha=0.45, interpolation="nearest")
        for name, path in plans.items():
            ax.plot(path[:, 0], path[:, 1], lw=2.0, label=name)
        ax.plot(*start, "ko", ms=7)
        ax.plot(*goal, "k*", ms=14)
        ax.legend(loc="lower right", fontsize=8)
        nav_common.save(fig, "12.10_keepout")
        plt.close("all")


if __name__ == "__main__":
    main()
