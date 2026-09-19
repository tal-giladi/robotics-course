"""Reference solution for 09.05 — Calibrating odometry from logged runs.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

Pose = tuple[float, float, float]
Ticks = Sequence[Sequence[int]]  # [[left, right], ...] cumulative counts, first sample = start


def fit_wheel_radius(wheel_angles_rad: Sequence[float], distances_m: Sequence[float]) -> float:
    """Least-squares r in distance = r * wheel_angle (a line through the origin)."""
    phi = np.asarray(wheel_angles_rad, dtype=float)
    d = np.asarray(distances_m, dtype=float)
    return float(phi @ d / (phi @ phi))


def fit_wheel_separation(
    left_travel_m: Sequence[float], right_travel_m: Sequence[float], heading_changes_rad: Sequence[float]
) -> float:
    """Least-squares b in (right_travel - left_travel) = b * heading_change."""
    s = np.asarray(right_travel_m, dtype=float) - np.asarray(left_travel_m, dtype=float)
    dtheta = np.asarray(heading_changes_rad, dtype=float)
    return float(s @ dtheta / (dtheta @ dtheta))


@dataclass(frozen=True)
class UmbmarkResult:
    cg_cw: tuple[float, float]  # center of gravity of the cw return-position errors (m)
    cg_ccw: tuple[float, float]
    e_max_syst: float  # max distance of the two centers from the origin (m)


def umbmark_summary(
    cw_errors: Sequence[tuple[float, float]], ccw_errors: Sequence[tuple[float, float]]
) -> UmbmarkResult:
    """Centers of gravity of the cw and ccw error clusters and Borenstein's E_max,syst."""
    cw = np.asarray(cw_errors, dtype=float).mean(axis=0)
    ccw = np.asarray(ccw_errors, dtype=float).mean(axis=0)
    return UmbmarkResult(
        (float(cw[0]), float(cw[1])),
        (float(ccw[0]), float(ccw[1])),
        float(max(math.hypot(*cw), math.hypot(*ccw))),
    )


def borenstein_correction(
    x_cg_cw: float, x_cg_ccw: float, side_m: float, wheel_separation_m: float
) -> tuple[float, float, float]:
    """Borenstein & Feng (1995) correction factors (c_left, c_right, e_b) from UMBmark x errors."""
    alpha = (x_cg_cw + x_cg_ccw) / (-4.0 * side_m)  # rad, wheelbase-type error per corner
    beta = (x_cg_cw - x_cg_ccw) / (-4.0 * side_m)  # rad, wheel-diameter-type error per leg
    if abs(beta) < 1e-12:
        e_d = 1.0
    else:
        radius = (side_m / 2.0) / math.sin(beta / 2.0)
        e_d = (radius + wheel_separation_m / 2.0) / (radius - wheel_separation_m / 2.0)
    e_b = (math.pi / 2.0) / (math.pi / 2.0 - alpha)
    c_left = 2.0 / (e_d + 1.0)
    c_right = 2.0 / (1.0 / e_d + 1.0)
    return c_left, c_right, e_b


def replay_odometry(
    ticks: Ticks,
    radius_left_m: float,
    radius_right_m: float,
    separation_m: float,
    ticks_per_rev: int,
    start: Pose = (0.0, 0.0, 0.0),
) -> Pose:
    """Run exact-arc odometry over a logged tick sequence with the given parameters."""
    counts = np.asarray(ticks, dtype=float)
    steps = np.diff(counts, axis=0) * (2.0 * math.pi / ticks_per_rev)
    x, y, theta = start
    for dl_angle, dr_angle in steps.tolist():
        dl, dr = dl_angle * radius_left_m, dr_angle * radius_right_m
        ds, dtheta = (dl + dr) / 2.0, (dr - dl) / separation_m
        if abs(dtheta) < 1e-9:
            x += ds * math.cos(theta)
            y += ds * math.sin(theta)
        else:
            radius = ds / dtheta
            x += radius * (math.sin(theta + dtheta) - math.sin(theta))
            y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
        theta += dtheta
    return (x, y, math.atan2(math.sin(theta), math.cos(theta)))


def pose_residuals(
    params: Sequence[float],
    runs: Sequence[tuple[Ticks, Pose]],
    ticks_per_rev: int,
    heading_weight_m: float = 1.0,
) -> np.ndarray:
    """Stack [x_err, y_err, heading_weight * heading_err] of every run (odometry minus measured)."""
    out = []
    for ticks, measured in runs:
        x, y, theta = replay_odometry(ticks, params[0], params[1], params[2], ticks_per_rev)
        dtheta = math.remainder(theta - measured[2], 2.0 * math.pi)
        out.extend([x - measured[0], y - measured[1], heading_weight_m * dtheta])
    return np.asarray(out)


def calibrate_least_squares(
    runs: Sequence[tuple[Ticks, Pose]],
    ticks_per_rev: int,
    initial: tuple[float, float, float],
    iterations: int = 10,
    heading_weight_m: float = 1.0,
) -> tuple[float, float, float]:
    """Gauss-Newton: find (radius_left, radius_right, separation) that best explain the final poses."""
    params = np.asarray(initial, dtype=float)
    for _ in range(iterations):
        residual = pose_residuals(params, runs, ticks_per_rev, heading_weight_m)
        jacobian = np.empty((residual.size, 3))
        for j in range(3):
            h = 1e-6 * params[j]
            bumped = params.copy()
            bumped[j] += h
            jacobian[:, j] = (pose_residuals(bumped, runs, ticks_per_rev, heading_weight_m) - residual) / h
        delta, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
        params = params + delta
        if np.all(np.abs(delta) < 1e-9 * np.abs(params)):
            break
    return float(params[0]), float(params[1]), float(params[2])
