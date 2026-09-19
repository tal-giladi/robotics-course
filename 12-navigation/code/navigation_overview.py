"""12.01 — The navigation problem in one picture: global plan on the map, local window around the robot.

    python 12-navigation/code/navigation_overview.py                 # rate/horizon numbers + nav_out/12.01_overview.png
    python 12-navigation/code/navigation_overview.py --window 1.0    # a 1 x 1 m local costmap instead of 2 x 2 m
    python 12-navigation/code/navigation_overview.py --along 1.0     # put the robot 1.0 m along the path (default 0.6)

Left: what the GLOBAL planner knows — the saved map of the whole apartment, and the path to the kitchen.
Right: what the LOCAL planner knows — a 2 x 2 m window around the robot, rebuilt from the LiDAR
several times a second, where the laundry basket that is not in the map shows up.
"""

from __future__ import annotations

import numpy as np

import costmap as cm
import grid_planning as gp
import local_planning
import nav_common
from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, OccupancyGrid, SensorParams, World, viz


def rate_table(speed_m_s: float) -> None:
    print(f"At {speed_m_s} m/s, how far karmel moves between two runs of each loop:")
    for name, hz in (("global planner (Nav2 BT: RateController 1 Hz)", 1.0),
                     ("global costmap update (karmel: 1 Hz)", 1.0),
                     ("local costmap update (karmel: 5 Hz)", 5.0),
                     ("LiDAR scan (karmel.yaml: 10 Hz)", 10.0),
                     ("controller (karmel: controller_frequency 20 Hz)", 20.0),
                     ("wheel velocity PID on the Pico (100 Hz)", 100.0)):
        print(f"  {name:50s} every {1000 / hz:6.0f} ms -> {speed_m_s / hz * 100:5.2f} cm")


def main() -> None:
    import argparse

    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", type=float, default=2.0, help="local costmap width = height [m] (karmel: 2)")
    ap.add_argument("--along", type=float, default=0.6, help="robot position along the global path [m]")
    args = ap.parse_args()
    cfg = load_config()
    rate_table(cfg.drive.max_linear_speed_m_s / 2)  # 0.25 m/s: karmel's cruise speed in nav2_params.yaml

    apartment = World.apartment()
    grid = apartment.to_occupancy_grid(nav_common.GRID_RESOLUTION)
    static = cm.static_layer(grid)
    global_costs = cm.inflate(static, grid.resolution, cfg.chassis.footprint_radius_m, 3.0, 0.8)
    blocked, extra = cm.cost_to_planner_penalty(global_costs)
    plan = gp.astar(blocked, grid.world_to_cell(*nav_common.START_XY), grid.world_to_cell(*nav_common.KITCHEN_XY),
                    cell_cost=extra)
    path = local_planning.global_path()  # the same A* plan, shortcut and resampled every 5 cm
    s = local_planning.path_distances(path)
    print(f"\nglobal plan: A* expanded {plan.expanded} of the map's {grid.width * grid.height} cells in "
          f"{plan.seconds * 1000:.0f} ms (Python); {s[-1]:.2f} m after smoothing")

    # The real apartment has a laundry basket on the path, 1.3 m along it (same as 12.05).
    basket = local_planning.point_along(path, 1.3)
    world = World.from_segments(apartment.segments, np.vstack([apartment.circles, [[basket[0], basket[1], 0.15]]]))
    robot_xy = path[int(np.searchsorted(s, args.along))]
    heading = float(np.arctan2(*(path[int(np.searchsorted(s, args.along + 0.2))] - robot_xy)[::-1]))
    sim = DiffDriveSim(world, DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), pose=(*robot_xy, heading), seed=0)
    scan = sim.lidar_scan()

    # local costmap: a rolling 2 x 2 m window centered on the robot, built from the scan only
    half = args.window / 2
    cells = int(round(args.window / 0.05))
    local = OccupancyGrid(np.full((cells, cells), -1, dtype=np.int8), 0.05, (robot_xy[0] - half, robot_xy[1] - half))
    layer = cm.obstacle_layer(local, sim.lidar_pose, scan)
    rows, cols = np.nonzero(layer == cm.LETHAL_OBSTACLE)
    lx, ly = local.cell_to_world(rows, cols)
    basket_lethal = bool(np.any(np.hypot(np.asarray(lx) - basket[0], np.asarray(ly) - basket[1]) < 0.25))
    print(f"robot {args.along:.2f} m along the path, basket {float(np.linalg.norm(basket - robot_xy)):.2f} m ahead (center); "
          f"{args.window:.1f} x {args.window:.1f} m local costmap -> basket in the local costmap: {basket_lethal}")
    local_costs = cm.inflate(cm.combine_max(np.zeros(local.data.shape, np.uint8), layer), local.resolution,
                             cfg.chassis.footprint_radius_m, 5.0, 0.35)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    ax = axes[0]
    ax.set_title("global: the saved map (no basket), whole apartment, ~1 Hz")
    ax.imshow(global_costs, origin="lower", extent=grid.extent, cmap="inferno_r", vmin=0, vmax=255)
    ax.plot(path[:, 0], path[:, 1], color="lime", lw=2, label="global path")
    ax.add_patch(plt.Rectangle((robot_xy[0] - half, robot_xy[1] - half), args.window, args.window, fill=False, ec="tab:cyan", lw=2, label="local window"))
    viz.draw_robot(ax, sim.pose, radius=cfg.chassis.footprint_radius_m, color="tab:blue")
    ax.plot(*nav_common.KITCHEN_XY, "c*", ms=14)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax = axes[1]
    ax.set_title(f"local: {args.window:.1f} x {args.window:.1f} m from the LiDAR, 5 Hz")
    ax.imshow(local_costs, origin="lower", extent=local.extent, cmap="inferno_r", vmin=0, vmax=255)
    viz.draw_scan(ax, sim.lidar_pose, scan, size=3, color="tab:cyan")
    ax.plot(path[:, 0], path[:, 1], color="lime", lw=2)
    viz.draw_robot(ax, sim.pose, radius=cfg.chassis.footprint_radius_m, color="tab:blue")
    ax.set_xlim(local.extent[0], local.extent[1])
    ax.set_ylim(local.extent[2], local.extent[3])
    ax.set_aspect("equal")
    nav_common.save(fig, "12.01_overview")


if __name__ == "__main__":
    main()
