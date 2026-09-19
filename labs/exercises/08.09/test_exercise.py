"""Checker for 08.09 — feedforward + PI: drive at exactly 0.5 m/s.

Run: ``python course.py check 08.09`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx

TARGET_M_S = 0.5
KP, KI = 0.05, 1.0
LOOP_DT = 0.02  # the Pi-side loop of lessons 08.04-08.09


# --- the geometry ---------------------------------------------------------------------------------
def test_wheel_speed_by_hand(impl):
    # 0.5 m/s with 45 mm wheels: 0.5 / 0.045 = 11.111 rad/s
    assert impl.wheel_speed_for(0.5, 0.045) == approx(11.1111, abs=1e-3)
    assert impl.wheel_speed_for(0.0, 0.045) == 0.0
    assert impl.wheel_speed_for(-0.25, 0.05) == approx(-5.0), "reverse is just a negative speed"


def test_wheel_speed_matches_the_robot_config(impl):
    cfg = load_config()
    w = impl.wheel_speed_for(TARGET_M_S, cfg.drive.wheel_radius_m)
    assert w * cfg.drive.wheel_radius_m == approx(TARGET_M_S)
    assert w < cfg.drive.max_wheel_speed_rad_s, "0.5 m/s must be inside the motor's range"


# --- the inverted model ---------------------------------------------------------------------------
@pytest.fixture
def model(impl):
    """karmel's numbers: 17 rad/s at duty 1.0 and 10.8 V, deadband 0.12."""
    return impl.FeedforwardModel(max_wheel_speed_rad_s=17.0, deadband=0.12, nominal_v=10.8)


def test_duty_by_hand(model):
    # fraction = 11.111 / 17 = 0.6536; duty = 0.12 + 0.6536 * 0.88 = 0.6952
    assert model.duty(11.1111) == approx(0.6952, abs=1e-3)
    assert model.duty(0.0) == 0.0, "a zero setpoint must send zero duty, not the deadband"
    assert model.duty(-11.1111) == approx(-0.6952, abs=1e-3), "the model is symmetric"
    assert model.duty(17.0) == approx(1.0), "top speed is duty 1.0"
    assert model.duty(40.0) == approx(1.0), "clamp: never ask for more than the motor has"


def test_deadband_is_the_duty_at_the_smallest_speed(model):
    assert model.duty(1e-6) == approx(0.12, abs=1e-4), "just above zero, the motor needs the deadband"


def test_battery_compensation(model):
    fresh = model.duty(11.1111, 12.6)  # top speed 17*12.6/10.8 = 19.83 rad/s
    nominal = model.duty(11.1111, 10.8)
    tired = model.duty(11.1111, 9.9)  # top speed 15.58 rad/s
    assert fresh == approx(0.6130, abs=1e-3)
    assert nominal == approx(0.6952, abs=1e-3)
    assert tired == approx(0.7475, abs=1e-3)
    assert fresh < nominal < tired, "an emptier pack needs MORE duty for the same speed"


def test_without_battery_compensation_the_voltage_is_ignored(model):
    plain = dataclasses.replace(model, use_battery=False)
    assert plain.duty(11.1111, 12.6) == approx(plain.duty(11.1111, 9.9))
    assert plain.duty(11.1111, 12.6) == approx(model.duty(11.1111, 10.8))
    assert model.duty(11.1111, None) == approx(model.duty(11.1111, 10.8)), "no reading -> assume nominal"


# --- the controller -------------------------------------------------------------------------------
def test_output_is_feedforward_plus_p_plus_i(impl, model):
    """Close to the setpoint (error 1.111 rad/s), nothing saturates, so every term is visible."""
    c = impl.FeedforwardPI(model, KP, KI)
    out = c.update(11.1111, 10.0, LOOP_DT, 10.8)
    ff = model.duty(11.1111, 10.8)  # 0.6952
    assert c.feedforward == approx(ff), "keep the feedforward for logging: it is most of the output"
    assert c.integral == approx(KI * 1.1111 * LOOP_DT), "integral in duty units: ki * e * dt"
    assert out == approx(ff + KP * 1.1111 + c.integral)
    assert out == approx(0.7730, abs=1e-3)  # 0.6952 + 0.0556 + 0.0222


def test_the_feedforward_does_almost_all_the_work(impl, model):
    """At the step itself P saturates the output — and that is fine, the model is already there."""
    c = impl.FeedforwardPI(model, KP, KI)
    out = c.update(11.1111, 0.0, LOOP_DT, 10.8)
    assert out == approx(1.0), "0.695 of feedforward + 0.556 of P: clamped"
    assert c.integral == approx(0.0, abs=1e-9), "saturated in the error's direction: no headroom to fill"


def test_at_the_setpoint_the_feedback_disappears(impl, model):
    c = impl.FeedforwardPI(model, KP, KI)
    for _ in range(50):
        out = c.update(11.1111, 11.1111, LOOP_DT, 10.8)
    assert c.integral == approx(0.0, abs=1e-9), "zero error: nothing to integrate"
    assert out == approx(model.duty(11.1111, 10.8)), "on target, the model alone drives the motor"


def test_anti_windup_fills_the_headroom_and_counts_the_feedforward(impl, model):
    """Asking for more than the motor can do: the output sits at 1.0 and the integral stays tiny."""
    c = impl.FeedforwardPI(model, KP, KI)
    for _ in range(200):  # 4 s of an impossible request
        out = c.update(25.0, 16.0, LOOP_DT, 10.8)
    assert out == approx(1.0)
    assert c.integral <= 0.3, f"the integral must stop at the headroom, got {c.integral:.2f}"
    # released: the measurement overshoots -> the duty must come off the limit at once
    assert c.update(25.0, 26.0, LOOP_DT, 10.8) < 1.0


def test_reset_clears_the_integral(impl, model):
    c = impl.FeedforwardPI(model, KP, KI)
    c.update(11.0, 0.0, LOOP_DT, 10.8)
    c.reset()
    assert c.integral == 0.0 and c.output == 0.0


# --- end to end on the simulator -------------------------------------------------------------------
def drive(impl, controller, seconds: float = 2.0, soc: float = 1.0, seed: int = 1, side: str = "right"):
    """Run a 50 Hz Pi-side loop on the realistic simulator; returns (times, wheel speeds, setpoint).

    The default is the RIGHT wheel: the realistic preset's right motor is 6 % weaker than the model,
    which is what a real motor does to a feedforward fitted on its twin.
    """
    cfg = load_config()
    params = dataclasses.replace(DiffDriveParams.realistic(cfg), battery_initial_soc=soc)
    sim = DiffDriveSim(World(), params, SensorParams.realistic(cfg), seed=seed)
    base = SimBase(sim, dt=0.01)
    setpoint = impl.wheel_speed_for(TARGET_M_S, cfg.drive.wheel_radius_m)
    t, speeds = [], []
    state = base.read()
    for k in range(round(seconds / LOOP_DT)):
        measured = state.right_rad_s if side == "right" else state.left_rad_s
        duty = controller.update(setpoint, measured, LOOP_DT, state.battery_v)
        base.set_wheel_duty(duty, duty)
        t.append(k * LOOP_DT)
        speeds.append(measured)
        base.read()
        state = base.read()
    base.close()
    return t, speeds, setpoint


def settled(values, n: int = 25) -> float:
    return sum(values[-n:]) / n


def test_feedforward_and_pi_reach_the_target_speed(impl, model):
    """The headline of the lesson: 0.5 m/s within 2 %, and quickly."""
    cfg = load_config()
    ff = impl.FeedforwardModel(cfg.drive.max_wheel_speed_rad_s, cfg.drive.duty_deadband,
                               cfg.battery.nominal_v)
    c = impl.FeedforwardPI(ff, KP, KI)
    t, speeds, setpoint = drive(impl, c)
    final = settled(speeds) * cfg.drive.wheel_radius_m
    assert final == approx(TARGET_M_S, rel=0.02), f"settled at {final:.3f} m/s, wanted {TARGET_M_S}"
    # within 5 % of the setpoint after 0.4 s: the feedforward jumps straight to almost the right duty
    early = [w for time, w in zip(t, speeds) if time >= 0.4]
    assert min(early) > 0.95 * setpoint, "with feedforward the wheel is at speed in well under half a second"


def test_feedforward_alone_is_close_but_not_exact(impl):
    """No feedback: the model gets the 6 %-weak right motor close, and that is all it can do."""
    cfg = load_config()
    ff = impl.FeedforwardModel(cfg.drive.max_wheel_speed_rad_s, cfg.drive.duty_deadband,
                               cfg.battery.nominal_v)
    c = impl.FeedforwardPI(ff, 0.0, 0.0)  # feedforward only
    _, speeds, setpoint = drive(impl, c)
    error = abs(settled(speeds) - setpoint) / setpoint
    assert 0.01 < error < 0.20, f"feedforward alone should be close but wrong, got {error * 100:.1f} %"


def test_the_tired_battery_needs_either_the_voltage_or_the_integral(impl):
    """Same command, a nearly empty pack: only the models that adapt still hold 0.5 m/s."""
    cfg = load_config()
    full = impl.FeedforwardModel(cfg.drive.max_wheel_speed_rad_s, cfg.drive.duty_deadband,
                                 cfg.battery.nominal_v)
    blind = dataclasses.replace(full, use_battery=False)

    _, blind_open, setpoint = drive(impl, impl.FeedforwardPI(blind, 0.0, 0.0), soc=0.15)
    _, smart_open, _ = drive(impl, impl.FeedforwardPI(full, 0.0, 0.0), soc=0.15)
    _, closed, _ = drive(impl, impl.FeedforwardPI(blind, KP, KI), soc=0.15)

    assert settled(blind_open) < 0.93 * setpoint, "ignoring the voltage must lose speed on a tired pack"
    assert abs(settled(smart_open) - setpoint) < abs(settled(blind_open) - setpoint), \
        "using the measured voltage must get closer than assuming the nominal one"
    assert settled(closed) == approx(setpoint, rel=0.02), "the integral removes what the model missed"


def test_zero_setpoint_sends_zero_duty(impl, model):
    c = impl.FeedforwardPI(model, KP, KI)
    assert c.update(0.0, 0.0, LOOP_DT, 10.8) == 0.0, "stop means stop: no deadband creep"
    assert math.isfinite(c.integral)
