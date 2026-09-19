"""Reference solution for 07.05 — IMU fundamentals: gyro bias and heading drift.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_drift_deg(bias_rad_s: float, seconds: float) -> float:
    return math.degrees(bias_rad_s * seconds)


def seconds_until_error(bias_rad_s: float, max_error_deg: float) -> float:
    if bias_rad_s == 0.0:
        return math.inf
    return math.radians(max_error_deg) / abs(bias_rad_s)


def estimate_gyro_bias(rates: list[float]) -> float:
    if not rates:
        raise ValueError("need at least one sample")
    return sum(rates) / len(rates)


def detect_still(rates: list[float], window: int, threshold_rad_s: float) -> list[bool]:
    half = window // 2
    n = len(rates)
    still = []
    for i in range(n):
        chunk = rates[max(0, i - half): min(n, i + half + 1)]
        still.append(max(chunk) - min(chunk) < threshold_rad_s)
    return still


def bias_from_still(rates: list[float], still: list[bool]) -> float:
    if len(rates) != len(still):
        raise ValueError("rates and still must have the same length")
    chosen = [r for r, s in zip(rates, still) if s]
    if not chosen:
        raise ValueError("no still samples: cannot estimate the bias")
    return estimate_gyro_bias(chosen)


def integrate_heading(times: list[float], rates: list[float], bias: float = 0.0,
                      initial_heading: float = 0.0) -> list[float]:
    if len(times) != len(rates):
        raise ValueError("times and rates must have the same length")
    if not times:
        return []
    headings = [wrap_angle(initial_heading)]
    heading = initial_heading
    for k in range(1, len(times)):
        dt = times[k] - times[k - 1]
        heading += 0.5 * ((rates[k] - bias) + (rates[k - 1] - bias)) * dt
        headings.append(wrap_angle(heading))
    return headings
