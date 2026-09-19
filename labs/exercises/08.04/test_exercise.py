"""Checker for 08.04 — Proportional control: hold a wheel speed.

Run: ``python course.py check 08.04`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx
LOOP_DT = 0.02  # the Pi-side loop runs at 50 Hz; the simulated firmware at 100 Hz


# --- steady_state_speed -------------------------------------------------------------------------
def test_no_deadband_is_the_classic_formula(impl):
    # slope 20, kp 0.1 -> loop gain L = 2; w = r * L / (1 + L) = 9 * 2/3 = 6
    assert impl.steady_state_speed(9.0, 0.1, 20.0, 0.0) == approx(6.0)


def test_deadband_costs_extra_speed(impl):
    # w = 20 * (0.1*9 - 0.12) / (1 + 2) = 20 * 0.78 / 3 = 5.2
    assert impl.steady_state_speed(9.0, 0.1, 20.0, 0.12) == approx(5.2)


def test_command_inside_deadband_never_starts(impl):
    # kp * r = 0.02 * 5 = 0.10 < 0.12: the motor never gets past the deadband
    assert impl.steady_state_speed(5.0, 0.02, 20.0, 0.12) == 0.0


def test_negative_setpoint_mirrors(impl):
    assert impl.steady_state_speed(-9.0, 0.1, 20.0, 0.12) == approx(-5.2)


def test_saturation_limits_the_speed(impl):
    # kp huge: formula gives ~ r, but duty 1.0 only reaches 20 * (1 - 0.12) = 17.6
    assert impl.steady_state_speed(30.0, 5.0, 20.0, 0.12) == approx(17.6)


def test_zero_setpoint(impl):
    assert impl.steady_state_speed(0.0, 0.3, 20.0, 0.12) == 0.0


# --- PController --------------------------------------------------------------------------------
def test_output_is_kp_times_error(impl):
    c = impl.PController(kp=0.1)
    assert c.update(8.0, 5.0, LOOP_DT) == approx(0.3)
    assert c.error == approx(3.0)
    assert c.output == approx(0.3)


def test_negative_error_gives_negative_output(impl):
    c = impl.PController(kp=0.05)
    assert c.update(2.0, 6.0, LOOP_DT) == approx(-0.2)


def test_output_is_clamped(impl):
    c = impl.PController(kp=0.1, output_limit=0.8)
    assert c.update(8.0, -20.0, LOOP_DT) == approx(0.8), "0.1 * 28 = 2.8 must be clamped to +0.8"
    assert c.update(-20.0, 8.0, LOOP_DT) == approx(-0.8)


def test_no_memory_between_steps(impl):
    c = impl.PController(kp=0.1)
    c.update(8.0, 0.0, LOOP_DT)
    assert c.update(8.0, 8.0, LOOP_DT) == approx(0.0), "P has no memory: zero error -> zero output"


def test_reset(impl):
    c = impl.PController(kp=0.1)
    c.update(8.0, 0.0, LOOP_DT)
    c.reset()
    assert (c.error, c.output) == (0.0, 0.0)


# --- against the simulator ----------------------------------------------------------------------
def hold_speed(impl, kp: float, setpoint: float, seconds: float = 3.0) -> tuple[float, float, float]:
    """Run P on both wheels of the realistic simulator; return (left, right, battery V) averaged over the last second."""
    cfg = load_config()
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=1), dt=0.01)
    left, right = impl.PController(kp), impl.PController(kp)
    state = base.read()
    samples = []
    for k in range(round(seconds / LOOP_DT)):
        base.set_wheel_duty(left.update(setpoint, state.left_rad_s, LOOP_DT),
                            right.update(setpoint, state.right_rad_s, LOOP_DT))
        state = base.read()
        state = base.read()  # two 10 ms firmware steps per 20 ms loop period
        if k * LOOP_DT >= seconds - 1.0:
            samples.append((state.left_rad_s, state.right_rad_s, state.battery_v))
    n = len(samples)
    return tuple(sum(s[i] for s in samples) / n for i in range(3))  # type: ignore[return-value]


@pytest.mark.parametrize("kp", [0.05, 0.1, 0.2])
def test_p_loop_settles_where_the_formula_says(impl, kp):
    cfg = load_config()
    d = cfg.drive
    setpoint = 8.0
    left, right, volts = hold_speed(impl, kp, setpoint)
    # the slope you would measure in 08.02: max speed at nominal voltage, scaled by today's voltage
    slope = d.max_wheel_speed_rad_s / (1.0 - d.duty_deadband) * volts / cfg.battery.nominal_v
    predicted = impl.steady_state_speed(setpoint, kp, slope, d.duty_deadband)
    assert left < setpoint - 0.5, "a P-only loop must leave a steady-state error"
    assert left == approx(predicted, abs=0.15), (
        f"kp={kp}: left wheel settled at {left:.2f} rad/s, the formula predicts {predicted:.2f}")
    # the right motor is 6% weaker in the realistic preset: a bit more error, same idea
    assert right < left
