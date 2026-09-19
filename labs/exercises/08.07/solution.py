"""08.07 — Reference solution: step-response metrics from a log, and PI gains from the motor model.

Don't read this before you have tried ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class StepMetrics:
    rise_time: float  # s, 10 % -> 90 % of the step (inf if the response never gets there)
    peak_time: float  # s after the step, when the response is farthest in the step's direction
    overshoot_percent: float  # how far past the setpoint the peak went, in % of the step (>= 0)
    settling_time: float  # s after the step until the response stays within the band for good (inf: never)
    steady_state_error: float  # setpoint - mean of the last `final_window_s` seconds
    final_value: float  # that mean


def _crossing_time(t: Sequence[float], n: Sequence[float], level: float) -> float:
    """First time the normalised response ``n`` reaches ``level``, interpolated between samples."""
    for k in range(len(n)):
        if n[k] >= level:
            if k == 0 or n[k] == n[k - 1]:
                return t[k]
            fraction = (level - n[k - 1]) / (n[k] - n[k - 1])
            return t[k - 1] + fraction * (t[k] - t[k - 1])
    return math.inf


def step_metrics(
    t: Sequence[float],
    y: Sequence[float],
    t_step: float,
    y_start: float,
    setpoint: float,
    settle_band: float = 0.02,
    final_window_s: float = 0.5,
) -> StepMetrics:
    step = setpoint - y_start
    if step == 0:
        raise ValueError("setpoint == y_start: there is no step to measure")
    after = [k for k in range(len(t)) if t[k] >= t_step - 1e-9]
    if not after:
        raise ValueError("no samples after the step")
    tt = [t[k] for k in after]
    yy = [y[k] for k in after]
    n = [(v - y_start) / step for v in yy]  # 0 at the start, 1 at the setpoint, for both step directions

    rise = _crossing_time(tt, n, 0.9) - _crossing_time(tt, n, 0.1)
    rise = rise if math.isfinite(rise) else math.inf

    k_peak = max(range(len(n)), key=lambda k: n[k])
    overshoot = max(0.0, (n[k_peak] - 1.0) * 100.0)
    peak_time = tt[k_peak] - t_step

    band = settle_band * abs(step)
    outside = [k for k in range(len(yy)) if abs(yy[k] - setpoint) > band]
    if not outside:
        settling = 0.0
    elif outside[-1] == len(yy) - 1:
        settling = math.inf
    else:
        settling = tt[outside[-1] + 1] - t_step

    tail = [v for time, v in zip(t, y) if time >= t[-1] - final_window_s + 1e-9]
    final = sum(tail) / len(tail)
    return StepMetrics(rise, peak_time, overshoot, settling, setpoint - final, final)


def pi_gains_pole_placement(K: float, tau: float, zeta: float, omega_n: float) -> tuple[float, float]:
    """PI on K/(tau s + 1): tau s^2 + (1 + K kp) s + K ki = 0  <=>  s^2 + 2 zeta omega_n s + omega_n^2."""
    kp = (2.0 * zeta * omega_n * tau - 1.0) / K
    ki = omega_n**2 * tau / K
    return kp, ki


def pi_gains_lambda(K: float, tau: float, tau_cl: float) -> tuple[float, float]:
    """Cancel the motor pole with the PI zero (ki/kp = 1/tau): closed loop = 1/(tau_cl s + 1)."""
    kp = tau / (K * tau_cl)
    return kp, kp / tau
