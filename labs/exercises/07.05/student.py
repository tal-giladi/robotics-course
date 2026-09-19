"""07.05 — IMU fundamentals: estimate a gyro's bias and integrate heading without drifting away.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 07.05``.
Standard library only (``math``).

Conventions: rad, rad/s, seconds; yaw rate counter-clockwise positive (REP-103); headings wrapped to
(-pi, pi]. A gyro sample = true rate + bias + noise. The bias is (nearly) constant for minutes, so
you can measure it whenever the robot is not rotating, and subtract it before integrating.
"""

from __future__ import annotations

import math  # noqa: F401  (you will need it)


def wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]. Already complete."""
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_drift_deg(bias_rad_s: float, seconds: float) -> float:
    """Heading error in DEGREES after integrating an uncorrected constant bias for ``seconds``."""
    # TODO(student): the integral of a constant.
    raise NotImplementedError("heading_drift_deg")


def seconds_until_error(bias_rad_s: float, max_error_deg: float) -> float:
    """How long until an uncorrected bias makes the heading ``max_error_deg`` wrong (math.inf if bias is 0).

    Use the magnitude of the bias: a negative bias drifts just as fast.
    """
    # TODO(student)
    raise NotImplementedError("seconds_until_error")


def estimate_gyro_bias(rates: list[float]) -> float:
    """Bias = mean of samples taken while the robot is NOT rotating. ValueError on an empty list."""
    # TODO(student)
    raise NotImplementedError("estimate_gyro_bias")


def detect_still(rates: list[float], window: int, threshold_rad_s: float) -> list[bool]:
    """Flag samples taken while the robot was still, from the gyro alone.

    Sample i is still if, in the window of samples from i - window//2 to i + window//2 (inclusive,
    clipped at both ends of the list), max(rate) - min(rate) < threshold_rad_s.
    Why a range test and not |rate| < threshold? Because the bias itself may be bigger than the
    threshold; a still gyro reads a CONSTANT (bias + small noise), not zero.
    """
    # TODO(student)
    raise NotImplementedError("detect_still")


def bias_from_still(rates: list[float], still: list[bool]) -> float:
    """Mean of the samples flagged still.

    ValueError if the lists differ in length or no sample is still.
    """
    # TODO(student)
    raise NotImplementedError("bias_from_still")


def integrate_heading(times: list[float], rates: list[float], bias: float = 0.0,
                      initial_heading: float = 0.0) -> list[float]:
    """Heading at every sample time, from yaw-rate samples, with the bias subtracted.

    Trapezoidal rule between consecutive samples (the time steps need not be equal):
        heading_k = heading_{k-1} + 0.5 * ((rate_k - bias) + (rate_{k-1} - bias)) * (t_k - t_{k-1})
    The first output is ``initial_heading`` (wrapped). Accumulate UNWRAPPED, wrap each output value.
    ValueError if the lists differ in length; an empty input gives an empty list.
    """
    # TODO(student)
    raise NotImplementedError("integrate_heading")
