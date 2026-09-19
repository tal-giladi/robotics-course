"""Checker for 08.05 — Integral control: killing steady-state error (and integral windup).

Run: ``python course.py check 08.05`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx
DT = 0.02
KP, KI = 0.1, 2.0  # the gains lesson 08.05 settles on


# --- hand-computed steps --------------------------------------------------------------------------
def test_first_step_is_p_plus_one_integral_slice(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    # error 3: P = 0.3, integral = 2.0 * 3 * 0.02 = 0.12 -> output 0.42
    assert c.update(8.0, 5.0, DT) == approx(0.42)
    assert c.integral == approx(0.12)
    assert c.output == approx(0.42)


def test_integral_accumulates_while_error_persists(impl):
    c = impl.PIController(kp=0.0, ki=1.0)
    for _ in range(10):
        c.update(1.0, 0.0, 0.1)  # integral += 1.0 * 1.0 * 0.1, ten times
    assert c.integral == approx(1.0)


def test_integral_holds_the_command_when_error_is_zero(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    c.update(8.0, 5.0, DT)
    assert c.update(8.0, 8.0, DT) == approx(0.12), "zero error: the integral alone keeps the motor driven"


def test_changing_ki_does_not_jump_the_output(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    c.update(8.0, 5.0, DT)
    c.ki = 10.0
    assert c.update(8.0, 8.0, DT) == approx(0.12), "store ki*error*dt in the integral, not the raw error sum"


def test_output_is_clamped(impl):
    c = impl.PIController(kp=0.1, ki=0.0, output_min=-0.5, output_max=0.5)
    assert c.update(20.0, 0.0, DT) == approx(0.5)
    assert c.update(-20.0, 0.0, DT) == approx(-0.5)


def test_reset(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    c.update(8.0, 0.0, DT)
    c.reset()
    assert (c.integral, c.output) == (0.0, 0.0)


# --- anti-windup ----------------------------------------------------------------------------------
def test_no_integration_into_saturation(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    c.update(20.0, 0.0, DT)  # P alone = 2.0: already saturated at +1
    for _ in range(100):
        c.update(20.0, 0.0, DT)
    assert c.integral == approx(0.0), "output pinned at +1 with positive error: the integral must not grow"


def test_integral_never_leaves_the_output_range(impl):
    c = impl.PIController(kp=0.0, ki=5.0)
    for _ in range(100):
        c.update(4.0, 3.5, DT)  # without protection: 100 * 5 * 0.5 * 0.02 = 5.0
    assert c.integral <= 1.0 + 1e-9


def test_integral_is_clamped(impl):
    c = impl.PIController(kp=0.1, ki=2.0)
    c.integral = 3.0  # e.g. left over from a larger output range
    c.update(8.0, 8.0, DT)
    assert c.integral == approx(1.0), "step 2 of anti-windup: clamp the integral to [output_min, output_max]"
    c.integral = -3.0
    c.update(8.0, 8.0, DT)
    assert c.integral == approx(-1.0)


def test_opposite_error_unwinds_even_while_saturated(impl):
    c = impl.PIController(kp=0.0, ki=1.0)
    c.integral = 0.8
    c.update(0.0, 10.0, DT)  # error -10: integral += 1.0 * -10 * 0.02 = -0.2
    assert c.integral == approx(0.6)


def test_without_anti_windup_the_integral_runs_away(impl):
    c = impl.PIController(kp=0.1, ki=2.0, anti_windup=False)
    for _ in range(100):
        c.update(20.0, 0.0, DT)  # 2.0 * 20 * 0.02 = 0.8 per step
    assert c.integral == approx(80.0)
    assert c.output == approx(1.0)


# --- against the simulator ----------------------------------------------------------------------
def run(impl, profile, seconds: float, anti_windup: bool = True):
    """PI on the left wheel of the realistic simulator at 50 Hz; returns [(t, setpoint, measured)]."""
    cfg = load_config()
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=2), dt=0.01)
    c = impl.PIController(KP, KI, anti_windup=anti_windup)
    state = base.read()
    rows = []
    for k in range(round(seconds / DT)):
        t = k * DT
        setpoint = profile(t)
        duty = c.update(setpoint, state.left_rad_s, DT)
        base.set_wheel_duty(duty, 0.0)
        state = base.read()
        state = base.read()
        rows.append((t + DT, setpoint, state.left_rad_s))
    return rows


def recovery_time(rows, t_event: float, band: float = 0.5) -> float:
    """Seconds after t_event until the speed stays within ±band of the setpoint."""
    last_outside = t_event
    for t, sp, w in rows:
        if t > t_event and abs(w - sp) > band:
            last_outside = t
    return last_outside - t_event


def test_pi_removes_steady_state_error(impl):
    rows = run(impl, lambda t: 8.0, 3.0)
    late = [w for t, _, w in rows if t > 2.0]
    mean = sum(late) / len(late)
    assert mean == approx(8.0, abs=0.1), f"PI should hold 8.0 rad/s, got {mean:.2f} (P alone gives ~4.6)"


def windup_profile(t: float) -> float:
    return 25.0 if t < 2.0 else 6.0  # 25 rad/s is faster than the motor can go: the duty saturates


def test_anti_windup_recovers_quickly_from_saturation(impl):
    rows = run(impl, windup_profile, 4.0, anti_windup=True)
    recovery = recovery_time(rows, 2.0)
    assert recovery < 0.6, f"with anti-windup the wheel should settle at 6 rad/s within 0.6 s, took {recovery:.2f} s"


def test_without_anti_windup_recovery_is_slow(impl):
    slow = recovery_time(run(impl, windup_profile, 4.0, anti_windup=False), 2.0)
    fast = recovery_time(run(impl, windup_profile, 4.0, anti_windup=True), 2.0)
    assert slow > 2 * fast, f"windup should make recovery much slower ({slow:.2f} s vs {fast:.2f} s)"
