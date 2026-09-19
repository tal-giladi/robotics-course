"""11.02 — Occupancy grid mapping with known poses: inverse sensor model, log-odds, Bresenham rays.

Library:
    bresenham(x0, y0, x1, y1)          integer cells on the line, both ends included
    bresenham_many(...)                 the same cells for many rays at once (numpy), for speed
    LogOddsGrid                         integrate_scan(sensor_pose, scan) / probabilities() / to_occupancy_grid()

Demo (plots and the saved map go to 11-slam/code/out/):
    python 11-slam/code/occupancy_grid_mapping.py              map the apartment with TRUE poses, save PGM + YAML
    python 11-slam/code/occupancy_grid_mapping.py --every 10   use every 10th scan only
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from slam_common import OUT, SE2, headless_pyplot, load_tour
from robotlab.sim import FREE, OCCUPIED, UNKNOWN, LaserScan, OccupancyGrid, World


def logit(p: float) -> float:
    """Probability -> log-odds: l = ln(p / (1 - p))."""
    return math.log(p / (1.0 - p))


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """All grid cells on the segment from (x0, y0) to (x1, y1), both included, in order.

    Integer-only: step along the axis with the larger change; the error term decides when to take
    a step along the other axis too. Works in all eight octants.
    """
    cells = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            return cells
        e2 = 2 * err
        if e2 >= dy:  # step in x
            err += dy
            x += sx
        if e2 <= dx:  # step in y
            err += dx
            y += sy


def bresenham_many(x0: int, y0: int, x1: np.ndarray, y1: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized ray cells from one start cell to many end cells (numpy, no Python loop per cell).

    Samples each ray once per cell along its major axis and rounds the minor axis — the same cells
    as Bresenham except, occasionally, which of two equally close cells is picked. Returns
    ``(xs, ys, ray_index)`` for every cell of every ray, end cells included.
    """
    x1, y1 = np.asarray(x1, dtype=np.int64), np.asarray(y1, dtype=np.int64)
    dx, dy = x1 - x0, y1 - y0
    n = np.maximum(np.abs(dx), np.abs(dy))  # cells per ray minus one
    ray = np.repeat(np.arange(len(n)), n + 1)
    starts = np.cumsum(n + 1) - (n + 1)
    k = np.arange(ray.size) - starts[ray]  # 0..n along each ray
    frac = np.where(n[ray] > 0, k / np.maximum(n[ray], 1), 0.0)

    def round_half_away(v: np.ndarray) -> np.ndarray:  # ties away from the start, like Bresenham mostly does
        return (np.sign(v) * np.floor(np.abs(v) + 0.5)).astype(np.int64)

    return x0 + round_half_away(frac * dx[ray]), y0 + round_half_away(frac * dy[ray]), ray


@dataclass
class LogOddsGrid:
    """A 2D occupancy grid stored as log-odds, with the same layout as ``OccupancyGrid``:
    ``logodds[row, col]``, row = y index (row 0 at the bottom), ``origin`` = world (x, y) of the
    bottom-left corner of cell (0, 0)."""

    rows: int
    cols: int
    resolution: float = 0.05
    origin: tuple[float, float] = (0.0, 0.0)
    p_hit: float = 0.70  # inverse sensor model: P(occupied | beam ended in this cell)
    p_miss: float = 0.35  # P(occupied | beam passed through this cell)
    l_min: float = -4.0  # clamp: p = 0.018 ... 0.982, so the map can still change its mind
    l_max: float = 4.0
    logodds: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.logodds = np.zeros((self.rows, self.cols))  # l = 0  <=>  p = 0.5, unknown
        self.l_hit, self.l_miss = logit(self.p_hit), logit(self.p_miss)

    @classmethod
    def like(cls, grid: OccupancyGrid, **kwargs: float) -> LogOddsGrid:
        """An empty log-odds grid with the size and placement of ``grid``."""
        return cls(grid.height, grid.width, grid.resolution, grid.origin, **kwargs)

    def world_to_cell(self, x: np.ndarray | float, y: np.ndarray | float) -> tuple[np.ndarray, np.ndarray]:
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int64)
        return col, row

    def integrate_scan(self, sensor_pose: SE2, scan: LaserScan, max_free_range: float | None = None) -> None:
        """Add one scan taken from ``sensor_pose`` (the LiDAR's pose in the map frame).

        * finite range  -> cells before the end: miss; end cell: hit
        * +inf (no return within range_max) -> cells up to ``max_free_range``: miss, no hit
        * nan / -inf    -> ignored (a dropout says nothing about the world)
        A cell gets at most one update per scan, and a hit beats a miss in the same scan.
        """
        r = scan.ranges
        with np.errstate(invalid="ignore"):
            hit = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
            no_return = np.isposinf(r)
        cap = scan.range_max if max_free_range is None else max_free_range
        use = hit | no_return
        dist = np.where(hit, r, cap)[use]
        world_angles = scan.angles[use] + sensor_pose.theta
        ex = sensor_pose.x + dist * np.cos(world_angles)
        ey = sensor_pose.y + dist * np.sin(world_angles)
        c0, r0 = self.world_to_cell(sensor_pose.x, sensor_pose.y)
        c1, r1 = self.world_to_cell(ex, ey)
        xs, ys, ray = bresenham_many(int(c0), int(r0), c1, r1)
        is_end = np.r_[ray[1:] != ray[:-1], True]  # last cell of each ray
        inside = (xs >= 0) & (xs < self.cols) & (ys >= 0) & (ys < self.rows)
        flat = ys * self.cols + xs
        hit_cells = np.unique(flat[inside & is_end & hit[use][ray]])
        miss_cells = np.setdiff1d(np.unique(flat[inside & ~is_end]), hit_cells)
        l = self.logodds.reshape(-1)  # a view: updates go into self.logodds
        l[miss_cells] += self.l_miss
        l[hit_cells] += self.l_hit
        np.clip(l, self.l_min, self.l_max, out=l)

    def probabilities(self) -> np.ndarray:
        """p = 1 - 1 / (1 + e^l) for every cell."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    def to_occupancy_grid(self, free_thresh: float = 0.25, occupied_thresh: float = 0.65) -> OccupancyGrid:
        """Trinary map: p >= occupied_thresh -> 100, p <= free_thresh -> 0, otherwise -1 (unknown).
        The defaults are ``map_saver_cli``'s (--occ 0.65 --free 0.25)."""
        p = self.probabilities()
        data = np.full(p.shape, UNKNOWN, dtype=np.int8)
        data[p <= free_thresh] = FREE
        data[p >= occupied_thresh] = OCCUPIED
        return OccupancyGrid(data, self.resolution, self.origin)


def compare_with_truth(mapped: OccupancyGrid, truth: OccupancyGrid) -> dict[str, float]:
    """Cell-by-cell quality of a map built on the same grid as ``truth`` (``World.to_occupancy_grid``)."""
    from scipy.ndimage import binary_dilation

    occ_truth = truth.data == OCCUPIED
    near_wall = binary_dilation(occ_truth, iterations=1)  # allow one cell of wall-thickness slack
    occ_map, free_map = mapped.data == OCCUPIED, mapped.data == FREE
    return {
        "occupied_cells": int(occ_map.sum()),
        "occupied_on_a_wall": float((occ_map & near_wall).sum() / max(occ_map.sum(), 1)),
        "free_really_free": float((free_map & ~occ_truth).sum() / max(free_map.sum(), 1)),
        "known_fraction": float((mapped.data != UNKNOWN).mean()),
    }


def build_map(poses: list[SE2], scans: list[LaserScan], world: World, lidar_offset: SE2 = SE2(), every: int = 1) -> tuple[LogOddsGrid, OccupancyGrid]:
    """Integrate every ``every``-th scan at the given base_link poses; grid = the world's grid."""
    truth_grid = world.to_occupancy_grid(resolution=0.05, margin=0.5)
    grid = LogOddsGrid.like(truth_grid)
    for pose, scan in list(zip(poses, scans))[::every]:
        grid.integrate_scan(pose @ lidar_offset, scan)
    return grid, truth_grid


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--every", type=int, default=1, help="use every N-th scan")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    log = load_tour(realistic=True, seed=args.seed)
    grid, truth = build_map(log.truth, log.scans, log.world, log.lidar_offset, args.every)
    occ = grid.to_occupancy_grid()
    print(f"{len(log.scans[::args.every])} scans, grid {grid.cols} x {grid.rows} cells at {grid.resolution} m")
    for key, value in compare_with_truth(occ, truth).items():
        print(f"  {key:20s} {value:.3f}" if isinstance(value, float) else f"  {key:20s} {value}")

    OUT.mkdir(exist_ok=True)
    yaml_path = occ.save(OUT / "apartment_known_poses.yaml")
    reloaded = OccupancyGrid.load(yaml_path)
    print(f"saved {yaml_path.name} + {yaml_path.with_suffix('.pgm').name}; reload identical: {np.array_equal(reloaded.data, occ.data)}")
    print(yaml_path.read_text(encoding="utf-8").strip())

    plt = headless_pyplot()
    from robotlab.sim import viz

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    im = axes[0].imshow(grid.logodds, origin="lower", extent=occ.extent, cmap="RdBu_r", vmin=-4, vmax=4)
    fig.colorbar(im, ax=axes[0], label="log-odds")
    axes[0].set_title("log-odds")
    viz.draw_occupancy_grid(axes[1], occ)
    axes[1].set_title("trinary map (free <= 0.25 < unknown < 0.65 <= occupied)")
    viz.draw_occupancy_grid(axes[2], occ)
    viz.draw_world(axes[2], log.world, color="tab:red", show_landmarks=False)
    viz.draw_trajectory(axes[2], np.array([p.as_tuple() for p in log.truth]), color="tab:blue", linewidth=1)
    axes[2].set_title("map + true walls (red) + path")
    for ax in axes:
        ax.set_aspect("equal")
    fig.savefig(OUT / "occupancy_known_poses.png", dpi=110, bbox_inches="tight")
    print("wrote", OUT / "occupancy_known_poses.png")


if __name__ == "__main__":
    main()
