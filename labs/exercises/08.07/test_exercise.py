"""Checker for 08.07 — PID mathematically: step-response metrics and model-based PI gains.

Run: ``python course.py check 08.07`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx


# --- metrics on hand-made series ------------------------------------------------------------------
def test_hand_computed_series(impl):
    t = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    y = [0.0, 0.0, 5.0, 11.0, 10.0, 10.0]
    m = impl.step_metrics(t, y, t_step=0.0, y_start=0.0, setpoint=10.0, final_window_s=1.5)
    # 10 %: between t=1 (n=0) and t=2 (n=0.5) -> 1 + 0.1/0.5 = 1.2
    # 90 %: between t=2 (n=0.5) and t=3 (n=1.1) -> 2 + 0.4/0.6 = 2.667  => rise 1.467
    assert m.rise_time == approx(1.4667, abs=1e-3), "interpolate both crossings between samples"
    assert m.overshoot_percent == approx(10.0)
    assert m.peak_time == approx(3.0)
    # band 0.02 * 10 = 0.2: last sample outside is t=3 -> settled from the next sample, t=4
    assert m.settling_time == approx(4.0)
    assert m.final_value == approx(10.0) and m.steady_state_error == approx(0.0)


def test_first_order_response_matches_theory(impl):
    tau, dt = 0.08, 0.001
    t = [k * dt for k in range(1500)]
    y = [0.0 if s < 0.2 else 8.0 * (1.0 - math.exp(-(s - 0.2) / tau)) for s in t]
    m = impl.step_metrics(t, y, t_step=0.2, y_start=0.0, setpoint=8.0)
    assert m.rise_time == approx(tau * math.log(9.0), abs=0.002), "10-90 % rise of a first-order lag = tau ln 9"
    assert m.overshoot_percent == approx(0.0)
    assert m.settling_time == approx(tau * math.log(50.0), abs=0.002), "2 % settling = tau ln 50"
    assert m.steady_state_error == approx(0.0, abs=1e-3)


def test_second_order_overshoot_and_peak_time(impl):
    zeta, wn, dt = 0.5, 20.0, 0.0005
    wd = wn * math.sqrt(1 - zeta**2)
    t = [k * dt for k in range(4000)]
    y = [1.0 - math.exp(-zeta * wn * s) * (math.cos(wd * s) + zeta / math.sqrt(1 - zeta**2) * math.sin(wd * s))
         for s in t]
    m = impl.step_metrics(t, y, t_step=0.0, y_start=0.0, setpoint=1.0)
    assert m.overshoot_percent == approx(100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2)), abs=0.1)  # 16.3 %
    assert m.peak_time == approx(math.pi / wd, abs=0.001)


def test_downward_step(impl):
    t = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    y = [8.0, 8.0, 5.0, 1.5, 2.0, 2.0, 2.0]
    m = impl.step_metrics(t, y, t_step=0.1, y_start=8.0, setpoint=2.0, final_window_s=0.25)
    # step -6: n = 0, 0, 0.5, 1.083, 1, 1, 1 -> overshoot 8.3 %, peak at 0.3 s = 0.2 s after the step
    assert m.overshoot_percent == approx(8.333, abs=0.01), "normalise by the step so downward steps work"
    assert m.peak_time == approx(0.2)
    assert m.steady_state_error == approx(0.0)


def test_p_only_never_settles_on_the_setpoint(impl):
    tau, dt = 0.08, 0.01
    t = [k * dt for k in range(300)]
    y = [4.65 * (1.0 - math.exp(-s / tau)) for s in t]
    m = impl.step_metrics(t, y, t_step=0.0, y_start=0.0, setpoint=8.0)
    assert m.steady_state_error == approx(3.35, abs=0.01)
    assert m.settling_time == math.inf, "the band is around the setpoint: a response stuck at 4.65 never settles"
    assert m.rise_time == math.inf, "never reaches 90 % of the step"


def test_no_step_is_an_error(impl):
    with pytest.raises(ValueError):
        impl.step_metrics([0.0, 1.0], [1.0, 1.0], 0.0, 1.0, 1.0)


# --- gains from the model -------------------------------------------------------------------------
def test_pole_placement_by_hand(impl):
    # K 20, tau 0.1, zeta 1, wn 20: kp = (2*1*20*0.1 - 1)/20 = 0.15; ki = 400*0.1/20 = 2.0
    kp, ki = impl.pi_gains_pole_placement(20.0, 0.1, 1.0, 20.0)
    assert (kp, ki) == (approx(0.15), approx(2.0))


def test_pole_placement_puts_the_poles_there(impl):
    K, tau, zeta, wn = 19.3, 0.08, 0.7, 30.0
    kp, ki = impl.pi_gains_pole_placement(K, tau, zeta, wn)
    a, b, c = tau, 1 + K * kp, K * ki  # tau s^2 + (1 + K kp) s + K ki
    real = -b / (2 * a)
    imag = math.sqrt(4 * a * c - b * b) / (2 * a)
    assert real == approx(-zeta * wn) and imag == approx(wn * math.sqrt(1 - zeta**2))


def test_lambda_tuning(impl):
    kp, ki = impl.pi_gains_lambda(20.0, 0.1, 0.05)  # kp = 0.1/(20*0.05) = 0.1, ki = kp/tau = 1.0
    assert (kp, ki) == (approx(0.1), approx(1.0))


# --- end to end: design on the model, measure on the simulator -------------------------------------
def simulate_step(kp: float, ki: float, seed: int = 1) -> tuple[list[float], list[float]]:
    """PI (firmware form) at 50 Hz on the realistic simulator, deadband compensated; step 0 -> 8 rad/s at 0.2 s."""
    cfg = load_config()
    d = cfg.drive
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=seed),
                   dt=0.01)
    integral, t, y = 0.0, [], []
    state = base.read()
    for k in range(100):
        now = k * 0.02
        setpoint = 8.0 if now >= 0.2 - 1e-9 else 0.0
        error = setpoint - state.left_rad_s
        others = kp * error
        candidate = integral + ki * error * 0.02
        if others + candidate > 1.0 and error > 0:
            integral = max(integral, 1.0 - others)
        elif others + candidate < -1.0 and error < 0:
            integral = min(integral, -1.0 - others)
        else:
            integral = candidate
        integral = min(max(integral, -1.0), 1.0)
        u = min(max(others + integral, -1.0), 1.0)
        duty = 0.0 if u == 0 else math.copysign(d.duty_deadband + abs(u) * (1 - d.duty_deadband), u)
        base.set_wheel_duty(duty, 0.0)
        t.append(now)
        y.append(state.left_rad_s)
        base.read()
        state = base.read()
    return t, y


def test_lambda_design_on_the_simulator(impl):
    cfg = load_config()
    d = cfg.drive
    # deadband compensated plant: speed = top speed at this battery * u
    K = d.max_wheel_speed_rad_s * 12.1 / cfg.battery.nominal_v
    kp, ki = impl.pi_gains_lambda(K, d.motor_time_constant_s, 0.1)
    t, y = simulate_step(kp, ki)
    m = impl.step_metrics(t, y, 0.2, 0.0, 8.0, settle_band=0.03)
    assert m.steady_state_error == approx(0.0, abs=0.1), "PI: zero steady-state error"
    assert m.overshoot_percent < 10.0, f"a 100 ms lambda design barely overshoots (got {m.overshoot_percent:.1f} %)"
    assert 0.1 < m.rise_time < 0.4, f"rise time near 2.2 * tau_cl = 0.22 s (got {m.rise_time:.3f} s)"
    assert m.settling_time < 0.8
