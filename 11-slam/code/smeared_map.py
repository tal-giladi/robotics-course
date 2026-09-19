"""11.03 — Why SLAM exists: the same scans mapped with true poses vs. with wheel odometry.

    python 11-slam/code/smeared_map.py            realistic robot, seed 0
    python 11-slam/code/smeared_map.py --seed 2   another run of the same robot

Prints how the odometry error grows along the tour and how many "occupied" cells land on no
real wall, and writes out/smeared_map.png (truth map | odometry map | error over time).
"""

from __future__ import annotations

import argparse

import numpy as np

from occupancy_grid_mapping import build_map, compare_with_truth
from slam_common import OUT, angle_diff, headless_pyplot, load_tour, trajectory_array


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    log = load_tour(realistic=True, seed=args.seed)

    distance = np.r_[0.0, np.cumsum([log.truth[k].distance_to(log.truth[k + 1]) for k in range(len(log.truth) - 1)])]
    pos_err = np.array([t.distance_to(o) for t, o in zip(log.truth, log.odom)])
    head_err = np.degrees(np.abs([angle_diff(o.theta, t.theta) for t, o in zip(log.truth, log.odom)]))
    print("travelled  position error  heading error")
    for target in (1, 2, 4, 6, 8, 10, distance[-1]):
        k = int(np.searchsorted(distance, target - 1e-9))
        k = min(k, len(distance) - 1)
        print(f"{distance[k]:7.2f} m   {pos_err[k]:10.3f} m   {head_err[k]:9.1f} deg")

    maps = {}
    for name, poses in (("true poses", log.truth), ("wheel odometry", log.odom)):
        grid, truth_grid = build_map(poses, log.scans, log.world, log.lidar_offset)
        occ = grid.to_occupancy_grid()
        maps[name] = occ
        q = compare_with_truth(occ, truth_grid)
        print(f"{name:15s}: {q['occupied_cells']:5d} occupied cells, {q['occupied_on_a_wall']:.0%} of them on a real wall, "
              f"{q['free_really_free']:.1%} of free cells really free")

    plt = headless_pyplot()
    from robotlab.sim import viz

    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    for ax, (name, occ), poses in zip(axes, maps.items(), (log.truth, log.odom)):
        viz.draw_occupancy_grid(ax, occ)
        viz.draw_trajectory(ax, trajectory_array(poses), color="tab:blue", linewidth=1)
        ax.set_aspect("equal")
        ax.set_title(f"mapped with {name}")
    axes[2].plot(distance, pos_err, label="position error [m]")
    axes[2].plot(distance, head_err / 100.0, label="heading error [deg / 100]")
    axes[2].set_xlabel("distance travelled [m]")
    axes[2].set_title("wheel odometry error along the tour")
    axes[2].legend()
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "smeared_map.png", dpi=110, bbox_inches="tight")
    print("wrote", OUT / "smeared_map.png")
    print(f"final odometry error: {pos_err[-1]:.2f} m, {head_err[-1]:.1f} deg")


if __name__ == "__main__":
    main()
