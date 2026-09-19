"""11.01 — The same apartment as five kinds of map, and what each costs in memory.

    python 11-slam/code/map_representations.py                 memory table + out/map_representations.png
    python 11-slam/code/map_representations.py --area 12 10 2.6 --resolution 0.05

The memory numbers are for a dense array per representation (no compression), so you can see
how they scale: a 2D grid grows with 1/resolution², a 3D voxel grid with 1/resolution³.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from slam_common import OUT, SE2, World, headless_pyplot


def human(nbytes: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if nbytes < 1000 or unit == "GB":
            return f"{nbytes:,.0f} {unit}" if unit == "B" else f"{nbytes:,.1f} {unit}"
        nbytes /= 1000
    return ""


def memory_table(width_m: float, depth_m: float, height_m: float, resolution: float) -> list[tuple[str, str, float]]:
    cols, rows = math.ceil(width_m / resolution), math.ceil(depth_m / resolution)
    layers = math.ceil(height_m / resolution)
    cells2d, cells3d = cols * rows, cols * rows * layers
    lidar_points = 360 * 10 * 600  # 360 points, 10 Hz, 10 minutes
    return [
        (f"2D occupancy grid, int8 ({cols}x{rows} cells)", "1 byte/cell", cells2d * 1),
        ("2D occupancy grid, float32 log-odds", "4 bytes/cell", cells2d * 4),
        (f"3D voxel grid, int8 ({cols}x{rows}x{layers})", "1 byte/voxel", cells3d * 1),
        ("3D voxel grid, float32 log-odds", "4 bytes/voxel", cells3d * 4),
        ("raw 2D LiDAR points, 10 min at 10 Hz", "2 x float32/point", lidar_points * 8),
        ("one depth-camera frame, 640x480 points", "3 x float32/point", 640 * 480 * 12),
        ("landmark map, 50 tags (id, x, y, 2x2 cov)", "28 bytes/landmark", 50 * 28),
        ("topological graph, 10 places + 12 doors", "~32 bytes/element", 22 * 32),
    ]


def draw_panels(world: World) -> None:
    plt = headless_pyplot()
    from robotlab.sim import viz

    fig, axes = plt.subplots(1, 4, figsize=(22, 5))
    grid = world.to_occupancy_grid(resolution=0.05, margin=0.2)
    viz.draw_occupancy_grid(axes[0], grid)
    axes[0].set_title(f"occupancy grid: {grid.width}x{grid.height} cells at 5 cm = {grid.data.nbytes / 1000:.1f} KB")

    viz.draw_world(axes[1], world, color="0.85", show_landmarks=False)
    axes[1].plot(world.landmarks[:, 0], world.landmarks[:, 1], "*", color="goldenrod", markersize=14)
    for (x, y), lid in zip(world.landmarks, world.landmark_ids):
        axes[1].annotate(f"tag {lid}", (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8)
    axes[1].set_title(f"landmark (feature) map: {len(world.landmarks)} points, walls unknown")

    angles = -math.pi + np.arange(360) * (2 * math.pi / 360)
    cloud = []
    for x, y in [(1.0, 1.3), (2.0, 2.5), (4.3, 2.2), (4.0, 3.4), (2.5, 4.0), (4.9, 1.0)]:
        r = world.raycast((x, y), angles, 12.0)
        ok = np.isfinite(r)
        cloud.append(SE2(x, y, 0.0).apply(np.column_stack([r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])])))
    pts = np.vstack(cloud)
    axes[2].scatter(pts[:, 0], pts[:, 1], s=1.5, color="tab:red")
    axes[2].set_title(f"point cloud: {len(pts)} points from 6 scans")

    viz.draw_world(axes[3], world, color="0.85", show_landmarks=False)
    places = {"living room": (1.7, 1.8), "kitchen": (4.9, 2.2), "bedroom": (4.1, 3.3), "study": (2.3, 4.2)}
    doors = {"door A": (3.5, 1.65), "door B": (4.9, 2.8), "door C": (3.5, 4.0), "door D": (2.05, 3.2)}
    links = [("living room", "door A"), ("door A", "kitchen"), ("kitchen", "door B"), ("door B", "bedroom"),
             ("bedroom", "door C"), ("door C", "study"), ("study", "door D"), ("door D", "living room")]
    nodes = places | doors
    for a, b in links:
        axes[3].plot(*zip(nodes[a], nodes[b]), color="tab:blue", linewidth=2)
    for name, (x, y) in places.items():
        axes[3].plot(x, y, "o", color="tab:blue", markersize=18)
        axes[3].annotate(name, (x, y), textcoords="offset points", xytext=(0, -22), ha="center", fontsize=9)
    for x, y in doors.values():
        axes[3].plot(x, y, "s", color="tab:green", markersize=9)
    objects = {"sofa": (0.9, 0.5), "desk": (0.8, 4.55), "bed": (5.1, 4.2), "counter": (5.6, 1.1),
               "coffee table": (2.7, 0.8), "kitchen table": (4.4, 1.2), "plant": (3.2, 4.75)}
    for name, (x, y) in objects.items():
        axes[3].annotate(name, (x, y), ha="center", fontsize=8, color="tab:purple", style="italic")
    axes[3].set_title("topological graph (blue/green) + semantic labels (purple)")

    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlim(-0.3, 6.3)
        ax.set_ylim(-0.3, 5.3)
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "map_representations.png", dpi=100, bbox_inches="tight")
    print("wrote", OUT / "map_representations.png")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--area", type=float, nargs=3, default=[12.0, 10.0, 2.6], metavar=("W", "D", "H"),
                        help="bounding box of the home in meters (default 12 x 10 x 2.6)")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    w, d, h = args.area
    print(f"Home bounding box {w} x {d} x {h} m at {args.resolution * 100:.0f} cm resolution")
    for name, unit, nbytes in memory_table(w, d, h, args.resolution):
        print(f"  {name:48s} {unit:20s} {human(nbytes):>10s}")
    if not args.no_plot:
        draw_panels(World.apartment())


if __name__ == "__main__":
    main()
