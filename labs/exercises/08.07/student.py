"""08.07 — Step-response metrics from a logged series, and PI gains from the first-order motor model.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.07``.
Only the standard library is needed (lists or numpy arrays both work as input).

Definitions (lesson 08.07, Level 3). Only samples with ``t >= t_step`` count. Normalise the response
so it runs from 0 (``y_start``) to 1 (``setpoint``): ``n = (y - y_start) / (setpoint - y_start)``.
That makes every definition work for downward steps too.

* rise_time          time from the first n >= 0.1 to the first n >= 0.9, each crossing linearly
                     interpolated between the two samples around it; inf if n never reaches 0.9
* peak_time          time of the largest n, measured from t_step
* overshoot_percent  max(0, (largest n - 1) * 100)
* settling_time      the response is "outside" at a sample if |y - setpoint| > settle_band * |step|.
                     Settling time = (time of the sample right after the LAST outside sample) - t_step;
                     0 if no sample is outside; inf if the last sample is still outside.
                     Note: the band is around the SETPOINT, so a loop with steady-state error never settles.
* steady_state_error setpoint - final_value, final_value = mean of y over the last final_window_s
                     seconds of the whole log (samples with t >= t[-1] - final_window_s)
"""

from __future__ import annotations

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


def step_metrics(
    t: Sequence[float],
    y: Sequence[float],
    t_step: float,
    y_start: float,
    setpoint: float,
    settle_band: float = 0.02,
    final_window_s: float = 0.5,
) -> StepMetrics:
    """Measure a logged step response. Raise ValueError if setpoint == y_start."""
    # TODO(student): implement the definitions in the module docstring.
    raise NotImplementedError("step_metrics")


def pi_gains_pole_placement(K: float, tau: float, zeta: float, omega_n: float) -> tuple[float, float]:
    """(kp, ki) that put the closed-loop poles of PI + K/(tau s + 1) at damping zeta, natural frequency omega_n.

    Closed-loop characteristic polynomial:  tau s^2 + (1 + K kp) s + K ki = 0.
    Match it with s^2 + 2 zeta omega_n s + omega_n^2 (divide by tau first).
    """
    # TODO(student)
    raise NotImplementedError("pi_gains_pole_placement")


def pi_gains_lambda(K: float, tau: float, tau_cl: float) -> tuple[float, float]:
    """(kp, ki) that cancel the motor pole with the PI zero (ki / kp = 1 / tau) and give a first-order
    closed loop with time constant tau_cl."""
    # TODO(student)
    raise NotImplementedError("pi_gains_lambda")
