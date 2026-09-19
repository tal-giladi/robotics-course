#!/usr/bin/env python3
"""Fit a first-order motor model to a wheel-velocity step, and compare two targets (lesson 06.10).

    py 06-simulation/code/sysid_fit.py sim_step.csv
    py 06-simulation/code/sysid_fit.py sim_step.csv real_step.csv     # side by side

The CSVs come from `06-simulation/code/ros2/step_response.py`, which commands one step on
/cmd_vel and logs, per /joint_states message:

    t_s, cmd_rad_s, meas_rad_s, pos_rad

This is *system identification* in its smallest useful form. The model is

$$\\dot{\\omega}(t) = \\frac{K\\,u(t - L) - \\omega(t)}{\\tau}$$

three numbers: a steady-state gain $K$ (is the wheel as fast as you asked?), a dead time $L$
(how long before anything happens?) and a time constant $\\tau$ (how sluggish is it?). Measure
them on the robot, put them in the simulator, and the simulation stops lying about acceleration.

The script reports the same three numbers for every file you give it, so "the gap" stops being
a feeling and becomes a table.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Fit:
    name: str
    step_time: float
    target: float
    steady: float
    gain: float
    dead_time_s: float
    tau_s: float
    rms_rad_s: float
    t50: float
    t90: float
    t99: float
    travel_m: float


def load(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    with open(path, newline="", encoding="utf-8") as handle:
        rows = [r for r in csv.DictReader(handle)]
    t = np.array([float(r["t_s"]) for r in rows])
    cmd = np.array([float(r["cmd_rad_s"]) for r in rows])
    meas = np.array([float(r["meas_rad_s"]) for r in rows])
    pos = np.array([float(r["pos_rad"]) for r in rows])
    order = np.argsort(t)
    return t[order], cmd[order], meas[order], pos[order]


def simulate(t: np.ndarray, command: np.ndarray, gain: float, dead_time: float,
             tau: float) -> np.ndarray:
    """First-order lag with dead time, integrated exactly on the (uneven) sample grid."""
    delayed = np.interp(t - dead_time, t, command, left=command[0])
    out = np.zeros_like(t)
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        alpha = 1.0 - math.exp(-dt / tau) if tau > 1e-6 else 1.0
        out[i] = out[i - 1] + (gain * delayed[i - 1] - out[i - 1]) * alpha
    return out


def fit(path: Path, wheel_radius: float = 0.045) -> Fit:
    t, cmd, meas, pos = load(path)
    active = np.flatnonzero(cmd > 0)
    if active.size == 0:
        raise SystemExit(f"{path}: the command never leaves zero — nothing to identify")
    step_time = t[active[0]]
    target = float(cmd[active[0]])
    hold_end = t[active[-1]]

    # Steady state: the last 40 % of the held step, where any transient is over.
    window = (t > step_time + 0.6 * (hold_end - step_time)) & (t <= hold_end)
    steady = float(meas[window].mean())
    gain = steady / target

    def crossing(fraction: float) -> float:
        level = fraction * steady
        for time, value in zip(t, meas):
            if time >= step_time and value >= level:
                return float(time - step_time)
        return float("nan")

    # Grid search over dead time and tau against the *rate-limited* command the controller sent.
    segment = (t >= step_time - 0.1) & (t <= hold_end)
    ts, cs, ms = t[segment], cmd[segment], meas[segment]
    best = (float("inf"), 0.0, 0.05)
    for dead_time in np.arange(0.0, 0.201, 0.005):
        for tau in np.concatenate([np.arange(0.005, 0.20, 0.005), np.arange(0.20, 1.01, 0.02)]):
            rms = float(np.sqrt(np.mean((simulate(ts, cs, gain, dead_time, tau) - ms) ** 2)))
            if rms < best[0]:
                best = (rms, float(dead_time), float(tau))
    rms, dead_time, tau = best

    travel = float(pos[-1] - pos[0]) * wheel_radius
    return Fit(path.name, step_time, target, steady, gain, dead_time, tau, rms,
               crossing(0.5), crossing(0.9), crossing(0.99), travel)


def sparkline(path: Path, width: int = 60, height: int = 12) -> list[str]:
    """A crude ASCII plot, so the shape is visible without matplotlib."""
    t, cmd, meas, _ = load(path)
    active = np.flatnonzero(cmd > 0)
    t0, t1 = t[active[0]] - 0.1, min(t[-1], t[active[-1]] + 0.1)
    window = (t >= t0) & (t <= t1)
    ts, ms, cs = t[window], meas[window], cmd[window]
    top = max(cs.max(), ms.max()) * 1.05 or 1.0
    grid = [[" "] * width for _ in range(height)]
    for time, value, command in zip(ts, ms, cs):
        col = min(width - 1, int((time - t0) / (t1 - t0) * (width - 1)))
        grid[height - 1 - min(height - 1, int(value / top * (height - 1)))][col] = "*"
        row = height - 1 - min(height - 1, int(command / top * (height - 1)))
        if grid[row][col] == " ":
            grid[row][col] = "."
    return ["|" + "".join(row) + "|" for row in grid] + \
           ["+" + "-" * width + f"+  0 .. {top:.1f} rad/s over {t1 - t0:.1f} s  ('.' command, '*' measured)"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csvs", nargs="+", type=Path)
    parser.add_argument("--plot", action="store_true", help="print an ASCII plot per file")
    args = parser.parse_args()

    fits = [fit(path) for path in args.csvs]
    header = f"{'file':<16}{'K':>7}{'L (ms)':>9}{'tau (ms)':>10}{'rms':>8}" \
             f"{'t50 (s)':>9}{'t90 (s)':>9}{'t99 (s)':>9}{'travel (m)':>12}"
    print(header)
    print("-" * len(header))
    for f in fits:
        print(f"{f.name:<16}{f.gain:>7.3f}{f.dead_time_s * 1000:>9.0f}{f.tau_s * 1000:>10.0f}"
              f"{f.rms_rad_s:>8.3f}{f.t50:>9.3f}{f.t90:>9.3f}{f.t99:>9.3f}{f.travel_m:>12.3f}")
    if len(fits) == 2:
        a, b = fits
        print(f"\ngap: t99 differs by {abs(b.t99 - a.t99) * 1000:.0f} ms, "
              f"tau by {abs(b.tau_s - a.tau_s) * 1000:.0f} ms, "
              f"travel by {abs(b.travel_m - a.travel_m) * 1000:.0f} mm "
              f"({abs(b.travel_m - a.travel_m) / max(a.travel_m, 1e-9):.1%})")
    if args.plot:
        for path in args.csvs:
            print(f"\n{path.name}")
            print("\n".join(sparkline(path)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
