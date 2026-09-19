"""12.04 — Costmaps: static, obstacle and inflation layers the way Nav2's costmap_2d builds them.

    python 12-navigation/code/costmap.py        # footprint radii, inflation table, layered costmap PNGs

Cost values are Nav2's (nav2_costmap_2d/cost_values.hpp):
    0 FREE_SPACE, 1..252 "some risk", 253 INSCRIBED_INFLATED_OBSTACLE, 254 LETHAL_OBSTACLE, 255 NO_INFORMATION.

Inflation (nav2_costmap_2d InflationLayer::computeCost, distance d in meters from the cell center
to the nearest lethal cell center):
    d == 0                     -> 254
    d <= inscribed_radius      -> 253
    otherwise                  -> floor(252 * exp(-cost_scaling_factor * (d - inscribed_radius)))
and only cells within ceil(inflation_radius / resolution) cells of a lethal cell are inflated.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import distance_transform_edt

import nav_common
from robotlab.geometry import SE2
from robotlab.sim import LaserScan, OccupancyGrid

FREE_SPACE = 0
MAX_NON_OBSTACLE = 252
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


# --- footprint ------------------------------------------------------------------------------------
def pad_footprint(footprint: Sequence[tuple[float, float]], padding: float) -> list[tuple[float, float]]:
    """Nav2's padFootprint: push every vertex ``padding`` further out along x and y."""
    return [(x + math.copysign(padding, x), y + math.copysign(padding, y)) for x, y in footprint]


def _point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def footprint_radii(footprint: Sequence[tuple[float, float]]) -> tuple[float, float]:
    """(inscribed, circumscribed) radius of a polygon around base_link, as Nav2 computes them:
    the smallest / largest distance from the origin to any vertex or edge."""
    lo, hi = math.inf, 0.0
    n = len(footprint)
    for i in range(n):
        (ax, ay), (bx, by) = footprint[i], footprint[(i + 1) % n]
        vertex = math.hypot(ax, ay)
        edge = _point_segment_distance(0.0, 0.0, ax, ay, bx, by)
        lo, hi = min(lo, vertex, edge), max(hi, vertex, edge)
    return lo, hi


# --- the inflation cost function --------------------------------------------------------------------
def inflation_cost(distance_m: float, inscribed_radius_m: float, cost_scaling_factor: float) -> int:
    """Nav2 InflationLayer::computeCost for one distance (no inflation-radius cut-off here)."""
    if distance_m == 0.0:
        return LETHAL_OBSTACLE
    if distance_m <= inscribed_radius_m:
        return INSCRIBED_INFLATED_OBSTACLE
    return int(MAX_NON_OBSTACLE * math.exp(-cost_scaling_factor * (distance_m - inscribed_radius_m)))


def distance_for_cost(cost: float, inscribed_radius_m: float, cost_scaling_factor: float) -> float:
    """Inverse of the decay: the distance at which the cost falls to ``cost`` (1..252)."""
    return inscribed_radius_m + math.log(MAX_NON_OBSTACLE / cost) / cost_scaling_factor


def inflate(
    master: NDArray[np.uint8],
    resolution: float,
    inscribed_radius_m: float,
    cost_scaling_factor: float,
    inflation_radius_m: float,
    inflate_unknown: bool = False,
) -> NDArray[np.uint8]:
    """Apply the inflation layer to a master costmap (a copy is returned).

    Distances are Euclidean between cell centers to the nearest LETHAL cell (an exact distance
    transform; Nav2 propagates a brushfire, which gives the same distances except in rare
    tie cases). Combination rule, as in Nav2: a cell that is NO_INFORMATION only takes the new cost
    if that cost is >= 253 (or > 0 with ``inflate_unknown``); every other cell keeps max(old, new).
    """
    out = master.copy()
    lethal = master == LETHAL_OBSTACLE
    if not lethal.any():
        return out
    dist_cells = distance_transform_edt(~lethal)
    cell_radius = max(0.0, math.ceil(inflation_radius_m / resolution))  # Costmap2D::cellDistance
    d = dist_cells * resolution
    cost = np.floor(MAX_NON_OBSTACLE * np.exp(-cost_scaling_factor * (d - inscribed_radius_m)))
    cost = np.where(d <= inscribed_radius_m, INSCRIBED_INFLATED_OBSTACLE, cost)
    cost = np.where(dist_cells == 0, LETHAL_OBSTACLE, cost)
    cost = np.where(dist_cells <= cell_radius, cost, FREE_SPACE).astype(np.uint8)
    unknown = master == NO_INFORMATION
    takes = cost > FREE_SPACE if inflate_unknown else cost >= INSCRIBED_INFLATED_OBSTACLE
    out = np.where(unknown, np.where(takes, cost, out), np.maximum(out, cost)).astype(np.uint8)
    return out


# --- layers ---------------------------------------------------------------------------------------
def static_layer(grid: OccupancyGrid, track_unknown_space: bool = True, lethal_cost_threshold: int = 100) -> NDArray[np.uint8]:
    """The map from map_server as costs (trinary interpretation, Nav2 defaults)."""
    out = np.full(grid.data.shape, FREE_SPACE, dtype=np.uint8)
    out[grid.data >= lethal_cost_threshold] = LETHAL_OBSTACLE
    if track_unknown_space:
        out[grid.data < 0] = NO_INFORMATION
    return out


@dataclass
class ObservationParams:
    """Per-source ObstacleLayer parameters (karmel's nav2_params.yaml values)."""

    obstacle_max_range: float = 3.5  # mark hits closer than this
    raytrace_max_range: float = 4.0  # clear free space along rays up to this
    marking: bool = True
    clearing: bool = True


def obstacle_layer(grid: OccupancyGrid, sensor_pose: SE2, scan: LaserScan, params: ObservationParams = ObservationParams()) -> NDArray[np.uint8]:
    """One LiDAR scan as its own layer: FREE along each ray (clearing), LETHAL at each hit (marking),
    NO_INFORMATION everywhere the scan says nothing."""
    layer = np.full(grid.data.shape, NO_INFORMATION, dtype=np.uint8)
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = np.asarray(scan.angles, dtype=float) + sensor_pose.theta
    ox, oy = sensor_pose.x, sensor_pose.y
    hit = np.isfinite(ranges) & (ranges > 0)
    no_return = np.isposinf(ranges)  # nothing within range: the whole raytrace range is free
    if params.clearing:
        clear_len = np.where(hit, np.minimum(ranges, params.raytrace_max_range), np.where(no_return, params.raytrace_max_range, 0.0))
        step = grid.resolution / 2
        for a, length in zip(angles, clear_len):
            if length <= 0:
                continue
            s = np.arange(0.0, length - grid.resolution * 0.5, step)
            r, c = grid.world_to_cell(ox + s * math.cos(a), oy + s * math.sin(a))
            ok = grid.in_bounds(r, c)
            layer[r[ok], c[ok]] = FREE_SPACE
    if params.marking:
        mark = hit & (ranges <= params.obstacle_max_range)
        r, c = grid.world_to_cell(ox + ranges[mark] * np.cos(angles[mark]), oy + ranges[mark] * np.sin(angles[mark]))
        ok = grid.in_bounds(r, c)
        layer[r[ok], c[ok]] = LETHAL_OBSTACLE
    return layer


def combine_max(master: NDArray[np.uint8], layer: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """CostmapLayer::updateWithMax: skip the layer's unknown cells; overwrite the master's unknown cells."""
    known = layer != NO_INFORMATION
    master_unknown = master == NO_INFORMATION
    out = master.copy()
    out[known & master_unknown] = layer[known & master_unknown]
    both = known & ~master_unknown
    out[both] = np.maximum(master[both], layer[both])
    return out


def cost_to_planner_penalty(costmap: NDArray[np.uint8], weight: float = 3.0) -> tuple[NDArray[np.bool_], NDArray[np.floating]]:
    """Turn a costmap into what grid_planning.astar wants: ``blocked`` (>= 253 or unknown) and an
    extra per-cell cost ``weight * cost / 252`` (so a step next to a wall costs up to 1 + weight)."""
    blocked = costmap >= INSCRIBED_INFLATED_OBSTACLE
    extra = np.where(blocked, 0.0, weight * costmap.astype(float) / MAX_NON_OBSTACLE)
    return blocked, extra


# --- demo -----------------------------------------------------------------------------------------
def main() -> None:
    import matplotlib.pyplot as plt

    import grid_planning as gp
    from robotlab.config import load_config
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World, viz

    cfg = load_config()
    r_sim = cfg.chassis.footprint_radius_m
    half_l, half_w = cfg.chassis.length_m / 2, cfg.chassis.width_m / 2
    chassis = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    nav2_fp = [(0.125, 0.105), (0.125, -0.105), (-0.125, -0.105), (-0.125, 0.105)]  # nav2_params.yaml
    print("Footprints (inscribed, circumscribed) [m]:")
    print(f"  karmel.yaml chassis {cfg.chassis.length_m} x {cfg.chassis.width_m} m: "
          "%.3f, %.3f  (footprint_radius_m = %.3f)" % (*footprint_radii(chassis), r_sim))
    print("  nav2_params.yaml footprint 0.25 x 0.21 m: %.3f, %.3f" % footprint_radii(nav2_fp))
    print("  ... with footprint_padding 0.01:            %.3f, %.3f" % footprint_radii(pad_footprint(nav2_fp, 0.01)))

    inscribed = footprint_radii(pad_footprint(nav2_fp, 0.01))[0]
    print(f"\nInflation cost vs distance (inscribed {inscribed:.3f} m):")
    print("  d [m]   csf=10 (Nav2 default)  csf=5 (karmel)  csf=2")
    for d in (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.45, 0.55, 0.70):
        print(f"  {d:4.2f}   {inflation_cost(d, inscribed, 10):>12d}          {inflation_cost(d, inscribed, 5):>6d}       "
              f"{inflation_cost(d, inscribed, 2):>6d}")
    for csf in (10.0, 5.0, 2.0):
        print(f"  csf={csf:4.1f}: cost drops below 128 at {distance_for_cost(128, inscribed, csf):.3f} m, "
              f"below 10 at {distance_for_cost(10, inscribed, csf):.3f} m")

    # A layered costmap of the apartment, with a box that is NOT in the map but seen by the LiDAR.
    world = World.apartment()
    grid = world.to_occupancy_grid(nav_common.GRID_RESOLUTION, margin=0.5)
    grid.data[grid.data == 0] = -1  # pretend SLAM only mapped the apartment's interior ...
    inside = np.zeros_like(grid.data, dtype=bool)
    rr, cc = np.mgrid[0 : grid.height, 0 : grid.width]
    xs, ys = grid.cell_to_world(rr, cc)
    inside[(xs > 0) & (xs < 6) & (ys > 0) & (ys < 5)] = True
    grid.data[inside & (grid.data == -1)] = 0  # ... everything outside the walls stays unknown
    static = static_layer(grid)
    clutter = World.from_segments(world.segments, np.vstack([world.circles, [[2.2, 1.5, 0.15]]]))  # a laundry basket
    sim = DiffDriveSim(clutter, DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), pose=nav_common.START_POSE, seed=0)
    scan = sim.lidar_scan()
    obstacles = obstacle_layer(grid, sim.lidar_pose, scan)
    master = combine_max(static, obstacles)
    for name, csf, radius in (("karmel", 5.0, 0.35), ("smooth", 3.0, 0.8)):
        costmap = inflate(master, grid.resolution, inscribed, csf, radius)
        blocked, extra = cost_to_planner_penalty(costmap)
        start, goal = grid.world_to_cell(*nav_common.START_XY), grid.world_to_cell(*nav_common.KITCHEN_XY)
        plain = gp.astar(costmap >= INSCRIBED_INFLATED_OBSTACLE, start, goal)
        aware = gp.astar(blocked, start, goal, cell_cost=extra)
        clear_plain = world_clearance(clutter, grid, plain.path)
        clear_aware = world_clearance(clutter, grid, aware.path)
        print(f"\n{name}: cost_scaling_factor={csf}, inflation_radius={radius} m")
        print(f"  shortest path in C-space : {gp.path_cost(plain.path) * grid.resolution:.2f} m, "
              f"min clearance {clear_plain:.3f} m (robot radius {r_sim:.3f})")
        print(f"  cost-aware path          : {gp.path_cost(aware.path) * grid.resolution:.2f} m, min clearance {clear_aware:.3f} m")

        fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
        for ax, data, title in ((axes[0], master, "static + obstacle layer (255 unknown = gray)"),
                                (axes[1], costmap, f"after inflation (csf={csf}, radius={radius} m)")):
            shown = np.ma.masked_where(data == NO_INFORMATION, data)
            ax.imshow(np.where(data == NO_INFORMATION, 1, np.nan), origin="lower", extent=grid.extent, cmap="Greys", vmin=0, vmax=3)
            im = ax.imshow(shown, origin="lower", extent=grid.extent, cmap="inferno_r", vmin=0, vmax=255, interpolation="nearest")
            ax.set_aspect("equal")
            ax.set_title(title)
            ax.plot(*nav_common.START_XY, "go", ms=8)
            ax.plot(*nav_common.KITCHEN_XY, "c*", ms=14)
        for res, color, label in ((plain, "tab:blue", "shortest (hugs 253)"), (aware, "lime", "cost-aware")):
            p = gp.cells_to_world(grid, res.path)
            axes[1].plot(p[:, 0], p[:, 1], color=color, lw=2, label=label)
        viz.draw_scan(axes[0], sim.lidar_pose, scan, size=2, color="tab:cyan")
        axes[1].legend(loc="upper right")
        fig.colorbar(im, ax=axes[1], shrink=0.8, label="cost")
        nav_common.save(fig, f"12.04_costmap_{name}")

    fig, ax = plt.subplots(figsize=(7, 4))
    d = np.linspace(0, 0.9, 400)
    for csf, radius in ((10.0, 0.55), (5.0, 0.35), (3.0, 0.8)):
        cost = [inflation_cost(x, inscribed, csf) if x <= radius else 0 for x in d]
        ax.plot(d, cost, label=f"cost_scaling_factor={csf}, inflation_radius={radius}")
    ax.axvline(inscribed, color="k", ls=":", label=f"inscribed radius {inscribed:.3f} m")
    ax.set_xlabel("distance from the obstacle cell [m]")
    ax.set_ylabel("cost")
    ax.legend(fontsize=8)
    nav_common.save(fig, "12.04_inflation_curves")


def world_clearance(world, grid: OccupancyGrid, path) -> float:
    """Smallest true distance from the path (cell centers) to any obstacle in ``world``."""
    import grid_planning as gp

    return float(world.distance_to_obstacles(gp.cells_to_world(grid, path)).min())


if __name__ == "__main__":
    main()
