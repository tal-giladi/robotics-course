#!/usr/bin/env python3
"""Score a saved map against the ground truth, instead of squinting at it (lessons 11.06, 11.09).

`map_saver_cli` gives you a .pgm you can look at. Looking is a real diagnostic — lesson 11.09 is
built on it — but "is this map better than the one I made yesterday?" needs a number. This script
compares two ROS `map_server` maps (yaml + pgm) in **world coordinates** and reports:

* wall accuracy  — what fraction of the map's occupied cells sit on a real wall
* wall coverage  — what fraction of the real walls the map found
* free-space purity — what fraction of the cells it calls free really are free
* explored area  — how much of the world it actually saw

    # the simulated apartment: the map frame's origin is the robot's spawn pose
    python 11-slam/code/compare_maps.py out/apartment_slam.yaml \
        --truth labs/ros2_ws/src/karmel_bringup/maps/apartment.yaml --map-origin -1.5 -0.5

    python 11-slam/code/compare_maps.py a.yaml --truth b.yaml --plot out/compare.png

`--map-origin X Y [YAW]` is where the *map frame* sits in the truth map's frame. In simulation
that is exactly the pose the robot was spawned at (`robot.launch.py x:= y:=`), because
slam_toolbox puts the map origin on the first scan's pose. Get it wrong and every number is
meaningless — which is itself the first thing to check when the scores look impossible.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = HERE / "out"
if str(ROOT / "labs" / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.sim import OccupancyGrid  # noqa: E402


def cells_in_world(grid: OccupancyGrid, value: int,
                   transform: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """World (x, y) of every cell equal to `value`, after applying (dx, dy, dyaw)."""
    rows, cols = np.nonzero(grid.data == value)
    x, y = grid.cell_to_world(rows, cols)
    dx, dy, dyaw = transform
    if dyaw:
        c, s = math.cos(dyaw), math.sin(dyaw)
        x, y = c * x - s * y, s * x + c * y
    return np.column_stack([x + dx, y + dy])


def nearest_distance(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest reference point (brute force; maps are small)."""
    if len(points) == 0 or len(reference) == 0:
        return np.full(len(points), np.inf)
    out = np.empty(len(points))
    for start in range(0, len(points), 2000):            # chunked so memory stays small
        chunk = points[start:start + 2000]
        d = np.linalg.norm(chunk[:, None, :] - reference[None, :, :], axis=2)
        out[start:start + len(chunk)] = d.min(axis=1)
    return out


def compare(map_path: Path, truth_path: Path, transform: tuple[float, float, float],
            tolerance: float) -> dict:
    """Score `map_path` against `truth_path`. Returns a dict of the printed numbers."""
    made = OccupancyGrid.load(map_path)
    truth = OccupancyGrid.load(truth_path)

    made_occ = cells_in_world(made, OccupancyGrid.OCCUPIED, transform)
    made_free = cells_in_world(made, OccupancyGrid.FREE, transform)
    truth_occ = cells_in_world(truth, OccupancyGrid.OCCUPIED)
    truth_free = cells_in_world(truth, OccupancyGrid.FREE)

    d_occ = nearest_distance(made_occ, truth_occ)
    accuracy = float((d_occ <= tolerance).mean()) if len(made_occ) else 0.0

    # coverage: a true wall cell is "found" if some mapped occupied cell is within tolerance
    d_cov = nearest_distance(truth_occ, made_occ) if len(made_occ) else np.full(len(truth_occ), np.inf)
    coverage = float((d_cov <= tolerance).mean()) if len(truth_occ) else 0.0

    # free-space purity: cells the map calls free that are really free floor
    d_free = nearest_distance(made_free, truth_occ) if len(made_free) else np.array([])
    purity = float((d_free > tolerance).mean()) if len(made_free) else 0.0

    cell_area = made.resolution ** 2
    stats = {
        'made_cells': (made.height, made.width),
        'resolution': made.resolution,
        'occupied': len(made_occ),
        'free': len(made_free),
        'unknown': int((made.data == OccupancyGrid.UNKNOWN).sum()),
        'wall_accuracy': accuracy,
        'wall_coverage': coverage,
        'free_purity': purity,
        'median_wall_error_m': float(np.median(d_occ)) if len(d_occ) else float('nan'),
        'p90_wall_error_m': float(np.percentile(d_occ, 90)) if len(d_occ) else float('nan'),
        'explored_m2': len(made_free) * cell_area,
        'truth_free_m2': len(truth_free) * truth.resolution ** 2,
    }
    return stats, (made, truth, made_occ, truth_occ)


def plot(path: Path, made: OccupancyGrid, truth: OccupancyGrid,
         made_occ: np.ndarray, truth_occ: np.ndarray) -> None:
    """Save an overlay: true walls in grey, mapped walls in red."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(truth_occ[:, 0], truth_occ[:, 1], s=4, c="0.7", marker="s", label="true walls")
    ax.scatter(made_occ[:, 0], made_occ[:, 1], s=4, c="tab:red", marker="s", label="mapped walls")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (truth frame)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"wrote {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map", help="the map to score (.yaml written by map_saver_cli)")
    parser.add_argument("--truth", required=True, help="ground-truth map (.yaml)")
    parser.add_argument("--map-origin", nargs="+", type=float, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y"),
                        help="pose of the map frame in the truth frame (default 0 0 0)")
    parser.add_argument("--tolerance", type=float, default=0.10,
                        help="how far a mapped wall may be from a real one, in m (default 0.10)")
    parser.add_argument("--plot", help="also write an overlay PNG here")
    args = parser.parse_args()

    origin = list(args.map_origin) + [0.0] * (3 - len(args.map_origin))
    stats, artifacts = compare(Path(args.map), Path(args.truth), tuple(origin[:3]), args.tolerance)

    print(f"map   : {args.map}")
    print(f"truth : {args.truth}")
    print(f"map frame at ({origin[0]:+.2f}, {origin[1]:+.2f}, {math.degrees(origin[2]):+.1f} deg) "
          f"in the truth frame; tolerance {args.tolerance * 100:.0f} cm")
    print(f"  grid            {stats['made_cells'][0]} x {stats['made_cells'][1]} cells "
          f"@ {stats['resolution']} m  "
          f"({stats['occupied']} occupied, {stats['free']} free, {stats['unknown']} unknown)")
    print(f"  wall accuracy   {stats['wall_accuracy']:6.1%}   "
          f"(mapped walls that sit on a real wall)")
    print(f"  wall coverage   {stats['wall_coverage']:6.1%}   (real walls the map found)")
    print(f"  free purity     {stats['free_purity']:6.1%}   (cells called free that really are)")
    print(f"  wall error      median {stats['median_wall_error_m'] * 100:.1f} cm, "
          f"90th pct {stats['p90_wall_error_m'] * 100:.1f} cm")
    print(f"  explored        {stats['explored_m2']:.1f} m2 of "
          f"{stats['truth_free_m2']:.1f} m2 of real free floor")

    if args.plot:
        made, truth, made_occ, truth_occ = artifacts
        plot(Path(args.plot), made, truth, made_occ, truth_occ)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
