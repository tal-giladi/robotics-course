"""Checker for 08.06 — Derivative control: damping, noise and derivative kick.

Run: ``python course.py check 08.06`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import random
import statistics
from collections import deque

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx
WIDE = {"output_min": -1e9, "output_max": 1e9}  # no saturation in the hand-computed cases


# --- P and I still behave like 08.05 --------------------------------------------------------------
def test_pi_part_unchanged(impl):
    c = impl.PIDController(kp=0.1, ki=2.0, kd=0.0)
    assert c.update(8.0, 5.0, 0.02) == approx(0.42)  # 0.3 + 2.0*3*0.02
    assert c.integral == approx(0.12)


def test_anti_windup_still_there(impl):
    c = impl.PIDController(kp=0.1, ki=2.0, kd=0.0)
    for _ in range(50):
        c.update(20.0, 0.0, 0.02)
    assert c.integral == approx(0.0)
    assert c.output == approx(1.0)


# --- the derivative -------------------------------------------------------------------------------
def test_first_update_has_no_derivative(impl):
    c = impl.PIDController(kp=0.0, ki=0.0, kd=1.0, **WIDE)
    assert c.update(0.0, 5.0, 0.01) == approx(0.0), "no previous measurement yet: d_term must be 0"
    assert c.d_term == approx(0.0)


def test_derivative_opposes_a_rising_measurement(impl):
    c = impl.PIDController(kp=0.0, ki=0.0, kd=0.1, **WIDE)
    c.update(0.0, 0.0, 0.01)
    # measured rises 1 rad/s in 10 ms: rate 100 rad/s^2, d_term = -0.1 * 100 = -10
    assert c.update(0.0, 1.0, 0.01) == approx(-10.0)
    assert c.d_term == approx(-10.0)


def test_filter_formula(impl):
    c = impl.PIDController(kp=0.0, ki=0.0, kd=0.1, derivative_filter_tau_s=0.01, **WIDE)
    c.update(0.0, 0.0, 0.01)
    # alpha = 0.01 / (0.01 + 0.01) = 0.5: rate = 0 + 0.5 * (100 - 0) = 50 -> d_term = -5
    assert c.update(0.0, 1.0, 0.01) == approx(-5.0)
    # measurement stops changing: raw rate 0, rate = 50 + 0.5 * (0 - 50) = 25 -> d_term = -2.5
    assert c.update(0.0, 1.0, 0.01) == approx(-2.5)


def test_no_derivative_kick_on_setpoint_step(impl):
    c = impl.PIDController(kp=0.1, ki=1.0, kd=0.05, derivative_filter_tau_s=0.0)
    for _ in range(20):
        c.update(4.0, 4.0, 0.02)  # steady: measurement constant, error 0
    before = c.output
    after = c.update(8.0, 4.0, 0.02)  # setpoint jumps by 4, measurement doesn't move
    # only P and I react: 0.1 * 4 + 1.0 * 4 * 0.02 = 0.48. D on the error would add 0.05 * 4 / 0.02 = 10.
    assert after - before == approx(0.48), "derivative kick: compute D from the measurement, not the error"
    assert c.d_term == approx(0.0)


def test_filtered_derivative_tracks_a_ramp(impl):
    c = impl.PIDController(kp=0.0, ki=0.0, kd=0.05, derivative_filter_tau_s=0.05, **WIDE)
    dt = 0.01
    for k in range(200):  # 2 s of measurement rising at 2 rad/s per second: 40 filter time constants
        c.update(0.0, 2.0 * k * dt, dt)
    assert c.d_term == approx(-0.1, rel=0.01), "after settling, the filtered rate equals the true rate"


def test_reset_forgets_the_last_measurement(impl):
    c = impl.PIDController(kp=0.0, ki=0.0, kd=1.0, derivative_filter_tau_s=0.02, **WIDE)
    c.update(0.0, 0.0, 0.01)
    c.update(0.0, 3.0, 0.01)
    c.reset()
    assert c.update(0.0, 10.0, 0.01) == approx(0.0), "after reset() the next update is a first update"
    assert (c.integral, c.d_term) == (0.0, 0.0)


def derivative_noise(impl, tau: float) -> float:
    """Std of the D output for a constant speed plus white measurement noise (sigma 0.1 rad/s)."""
    rng = random.Random(7)
    c = impl.PIDController(kp=0.0, ki=0.0, kd=0.002, derivative_filter_tau_s=tau, **WIDE)
    out = [c.update(8.0, 8.0 + rng.gauss(0.0, 0.1), 0.02) for _ in range(2000)]
    return statistics.pstdev(out[100:])


def test_filter_attenuates_noise(impl):
    raw = derivative_noise(impl, 0.0)
    filtered = derivative_noise(impl, 0.04)
    # unfiltered: 0.002 * sqrt(2) * 0.1 / 0.02 = 0.0141
    assert raw == approx(0.0141, rel=0.1)
    assert filtered < 0.5 * raw, f"a 40 ms filter should at least halve the D noise ({filtered:.4f} vs {raw:.4f})"


# --- against the simulator: damping a loop with extra delay ---------------------------------------
def overshoot(impl, kd: float, tau: float) -> tuple[float, float]:
    """Step 0 -> 8 rad/s on the realistic simulator, PI(0.1, 2.0) + D, one extra 20 ms of delay.
    Returns (peak speed, mean over the last second)."""
    cfg = load_config()
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=3), dt=0.01)
    c = impl.PIDController(0.1, 2.0, kd, tau)
    pending = deque([0.0])  # the command sent now reaches the motor one period later
    state = base.read()
    speeds = []
    for _ in range(150):
        pending.append(c.update(8.0, state.left_rad_s, 0.02))
        base.set_wheel_duty(pending.popleft(), 0.0)
        state = base.read()
        state = base.read()
        speeds.append(state.left_rad_s)
    return max(speeds), sum(speeds[-50:]) / 50


def test_derivative_damps_the_overshoot(impl):
    pi_peak, _ = overshoot(impl, kd=0.0, tau=0.0)
    pid_peak, pid_final = overshoot(impl, kd=0.002, tau=0.01)
    assert pi_peak > 9.6, f"sanity: PI with extra delay should overshoot past 9.6 rad/s (got {pi_peak:.2f})"
    assert pid_peak < 9.2, f"kd=0.002 should cut the peak below 9.2 rad/s (got {pid_peak:.2f})"
    assert pid_final == approx(8.0, abs=0.1)
