"""10.07 / 10.09 — export a simulated apartment tour so ROS 2 nodes can replay it.

    python 10-localization/code/export_replay.py --out localization_out/replay

Writes into ``--out``:

* ``apartment.yaml`` + ``apartment.pgm`` — the ground-truth map in ROS ``map_server`` format,
  straight out of ``World.apartment().to_occupancy_grid()``. This is what ``map_server`` loads.
* ``replay.npz`` — the tour: truth poses, dead-reckoned odometry, LiDAR scans, encoder ticks, gyro.

Then, inside a ROS 2 Jazzy container (see ``ros2/README.md``):

    python3 10-localization/code/ros2/replay_scan.py       # feeds AMCL
    python3 10-localization/code/ros2/replay_odom_imu.py   # feeds robot_localization

Nothing here imports rclpy, so it runs anywhere numpy does.
"""

from __future__ import annotations

import argparse

import numpy as np

import loc_common as lc
from robotlab.sim import World


def export(out_dir, seed: int = 3, scan_every: int = 10, resolution: float = 0.05) -> dict:
    """Build the map and the replay log. Returns the summary numbers the lesson quotes."""
    out = lc.out_dir(out_dir)
    grid = World.apartment().to_occupancy_grid(resolution=resolution, margin=0.3)
    grid.save(out / "apartment.yaml")

    log = lc.drive_tour(seed=seed, observe_every=10, scan_every=scan_every)
    dr = lc.dead_reckon(log.ticks, tuple(log.truth[0]))
    steps = np.array([k for k, _ in log.scans], dtype=int)
    ranges = np.array([np.asarray(s.ranges, dtype=np.float32) for _, s in log.scans])
    angles = np.asarray(log.scans[0][1].angles, dtype=np.float32)
    np.savez_compressed(
        out / "replay.npz",
        t=log.t[steps], truth=log.truth[steps], odom=dr[steps], ranges=ranges, angles=angles,
        t_all=log.t, truth_all=log.truth, odom_all=dr, ticks=log.ticks, gyro=log.gyro, dt=log.dt,
    )
    err = np.hypot(dr[:, 0] - log.truth[:, 0], dr[:, 1] - log.truth[:, 1])
    return {
        "map_cells": (grid.width, grid.height), "origin": grid.origin, "resolution": grid.resolution,
        "occupied": int((grid.data == grid.OCCUPIED).sum()), "free": int((grid.data == grid.FREE).sum()),
        "seconds": float(log.t[-1]), "steps": int(len(log.t)), "scans": int(len(log.scans)),
        "beams": int(ranges.shape[1]),
        "dr_rmse": float(np.sqrt(np.mean(err**2))), "dr_final": float(err[-1]),
        "start": [round(float(v), 3) for v in log.truth[0]], "end": [round(float(v), 3) for v in log.truth[-1]],
        "odom_end": [round(float(v), 3) for v in dr[-1]], "out": out,
    }


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="localization_out/replay")
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--scan-every", type=int, default=10, help="dt = 0.02 s, so 10 -> 5 Hz")
    args = parser.parse_args(argv)
    r = export(args.out, seed=args.seed, scan_every=args.scan_every)
    print(f"map   : {r['map_cells'][0]} x {r['map_cells'][1]} cells at {r['resolution']} m, "
          f"origin {r['origin']}, {r['occupied']} occupied / {r['free']} free")
    print(f"tour  : {r['seconds']:.1f} s, {r['steps']} steps, {r['scans']} scans of {r['beams']} beams")
    print(f"truth : start {r['start']} -> end {r['end']}")
    print(f"odom  : end {r['odom_end']}  (dead reckoning RMSE {r['dr_rmse'] * 100:.1f} cm, "
          f"final {r['dr_final'] * 100:.1f} cm)")
    print(f"wrote {r['out'] / 'apartment.yaml'}, {r['out'] / 'apartment.pgm'} and {r['out'] / 'replay.npz'}")
    return r


if __name__ == "__main__":
    main()
