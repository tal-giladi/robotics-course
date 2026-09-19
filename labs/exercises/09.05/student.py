"""09.05 — Calibrating odometry from logged runs.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 09.05``.
You may use ``math`` and ``numpy``.

The data you calibrate from is ``logged_runs.json`` (next to this file): encoder counts of
UMBmark squares, straight runs and spins, plus the final pose measured by hand. See README.md.

Conventions: meters, radians; x forward, y left, counter-clockwise positive. A pose is a tuple
``(x, y, theta)``. ``ticks`` is a sequence of ``[left, right]`` cumulative encoder counts; the
first sample is the start of the run.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

Pose = tuple[float, float, float]
Ticks = Sequence[Sequence[int]]


def fit_wheel_radius(wheel_angles_rad: Sequence[float], distances_m: Sequence[float]) -> float:
    """Least-squares wheel radius from straight runs.

    Model: distance_i = r * wheel_angle_i (wheel_angle = mean rotation of the two wheels, rad).
    Minimizing sum (distance_i - r * angle_i)^2 gives r = sum(angle*distance) / sum(angle^2).
    """
    # TODO(student): one line with numpy dot products (or a loop).
    raise NotImplementedError("fit_wheel_radius")


def fit_wheel_separation(
    left_travel_m: Sequence[float], right_travel_m: Sequence[float], heading_changes_rad: Sequence[float]
) -> float:
    """Least-squares wheel separation from spins in place.

    Model: (right_travel_i - left_travel_i) = b * heading_change_i. Same algebra as above.
    """
    # TODO(student): form s_i = right - left, then b = sum(s*dtheta) / sum(dtheta^2).
    raise NotImplementedError("fit_wheel_separation")


@dataclass(frozen=True)
class UmbmarkResult:
    cg_cw: tuple[float, float]  # center of gravity (mean) of the cw return-position errors (m)
    cg_ccw: tuple[float, float]  # same for ccw
    e_max_syst: float  # the larger distance of the two centers from the origin (m)


def umbmark_summary(
    cw_errors: Sequence[tuple[float, float]], ccw_errors: Sequence[tuple[float, float]]
) -> UmbmarkResult:
    """Borenstein's UMBmark numbers from return-position errors (eps_x, eps_y) = measured - odometry."""
    # TODO(student): mean of each cluster; E_max,syst = max(|cg_cw|, |cg_ccw|). Return plain floats.
    raise NotImplementedError("umbmark_summary")


def borenstein_correction(
    x_cg_cw: float, x_cg_ccw: float, side_m: float, wheel_separation_m: float
) -> tuple[float, float, float]:
    """Borenstein & Feng's closed-form correction from the x components of the two centers.

    For a square started at the origin facing +x (first leg along +x), in radians:
        alpha = (x_cg_cw + x_cg_ccw) / (-4 * side)      turn error per corner (wheelbase)
        beta  = (x_cg_cw - x_cg_ccw) / (-4 * side)      heading gained per leg (wheel diameters)
        R     = (side / 2) / sin(beta / 2)              radius of the curved "straight" leg
        E_d   = (R + b/2) / (R - b/2)                   = D_right / D_left   (E_d = 1 if beta == 0)
        E_b   = (pi/2) / (pi/2 - alpha)                 = b_actual / b_nominal
        c_left = 2 / (E_d + 1),   c_right = 2 / (1/E_d + 1)
    Return (c_left, c_right, E_b). Multiply the radii by c_left/c_right and the separation by E_b.
    """
    # TODO(student): transcribe the equations; guard beta == 0.
    raise NotImplementedError("borenstein_correction")


def replay_odometry(
    ticks: Ticks,
    radius_left_m: float,
    radius_right_m: float,
    separation_m: float,
    ticks_per_rev: int,
    start: Pose = (0.0, 0.0, 0.0),
) -> Pose:
    """Run exact-arc odometry (as in 09.04, but with a separate radius per wheel) over a log.

    For each pair of consecutive samples: wheel travel = tick delta * 2*pi/ticks_per_rev * radius,
    then the exact-arc update. Return the final pose with theta wrapped to (-pi, pi].
    """
    # TODO(student): loop over consecutive samples (numpy.diff helps).
    raise NotImplementedError("replay_odometry")


def pose_residuals(
    params: Sequence[float],
    runs: Sequence[tuple[Ticks, Pose]],
    ticks_per_rev: int,
    heading_weight_m: float = 1.0,
) -> np.ndarray:
    """Residual vector for least squares.

    ``params`` = (radius_left, radius_right, separation). For every run (ticks, measured_pose)
    replay the odometry and append three numbers:
        x_odom - x_measured,  y_odom - y_measured,  heading_weight_m * wrap(theta_odom - theta_measured)
    The weight converts radians to "meters' worth" so position and heading can be summed.
    """
    # TODO(student): build a list, return np.asarray(list). math.remainder(a, 2*pi) wraps.
    raise NotImplementedError("pose_residuals")


def calibrate_least_squares(
    runs: Sequence[tuple[Ticks, Pose]],
    ticks_per_rev: int,
    initial: tuple[float, float, float],
    iterations: int = 10,
    heading_weight_m: float = 1.0,
) -> tuple[float, float, float]:
    """Gauss-Newton on (radius_left, radius_right, separation).

    Repeat ``iterations`` times (stop early once the step is negligible):
      1. r = pose_residuals(params)
      2. numerical Jacobian J (N x 3): column j = (residuals(params with params[j] += h) - r) / h,
         with h = 1e-6 * params[j]
      3. solve J @ delta = -r in the least-squares sense: np.linalg.lstsq(J, -r, rcond=None)
      4. params += delta
    """
    # TODO(student): implement the loop above.
    raise NotImplementedError("calibrate_least_squares")
