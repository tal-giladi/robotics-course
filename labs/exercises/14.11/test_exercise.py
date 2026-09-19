"""Checker for 14.11 — the safety supervisor.

Run: ``python course.py check 14.11`` (or ``--solution`` to see the reference pass).
No hardware, no numpy: a fake clock makes the watchdog tests instant and deterministic.
"""

from __future__ import annotations

import math

import pytest

STALL_TORQUE_NM = 16.5 / 10.197
MAX_REACH_M = 0.546


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def supervisor(impl, **kwargs):
    limits = {j: impl.JointLimits(-40.0, 40.0)
              for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")}
    limits["gripper"] = impl.JointLimits(5.0, 60.0)
    kwargs.setdefault("clock", FakeClock())
    return impl.SafetySupervisor(limits=limits, **kwargs)


# --- the numbers ----------------------------------------------------------------------------------
def test_tip_speed(impl):
    assert impl.tip_speed(300.0) == pytest.approx(MAX_REACH_M * math.radians(300.0))
    assert impl.tip_speed(300.0) == pytest.approx(2.859, abs=1e-3)
    assert impl.tip_speed(30.0) == pytest.approx(0.286, abs=1e-3)


def test_tip_speed_uses_radians(impl):
    """A degrees-for-radians slip gives 163.8 m/s, which is the tell."""
    assert impl.tip_speed(300.0) < 5.0


def test_kinetic_energy_is_quadratic(impl):
    assert impl.kinetic_energy(300.0) == pytest.approx(1.226, abs=1e-3)
    assert impl.kinetic_energy(30.0) == pytest.approx(0.01226, abs=1e-4)
    assert impl.kinetic_energy(300.0) / impl.kinetic_energy(30.0) == pytest.approx(100.0, rel=1e-6)


def test_pinch_force(impl):
    assert impl.pinch_force(MAX_REACH_M) == pytest.approx(2.96, abs=0.01)
    assert impl.pinch_force(0.01) == pytest.approx(161.8, abs=0.1)
    assert impl.pinch_force(0.01, 30.0) == pytest.approx(48.5, abs=0.1)


def test_pinch_force_rejects_a_nonpositive_lever(impl):
    with pytest.raises(ValueError):
        impl.pinch_force(0.0)


# --- the state machine ------------------------------------------------------------------------------
def test_starts_disabled(impl):
    assert supervisor(impl).state is impl.ArmState.DISABLED


def test_enable_goes_to_enabled(impl):
    s = supervisor(impl)
    s.enable()
    assert s.state is impl.ArmState.ENABLED


def test_stop_latches_the_reason(impl):
    s = supervisor(impl)
    s.enable()
    s.stop("elbow_flex overloaded")
    assert s.state is impl.ArmState.STOPPED
    assert "elbow_flex" in s.fault


def test_enable_is_refused_while_stopped(impl):
    s = supervisor(impl)
    s.enable()
    s.stop("something")
    with pytest.raises(impl.SafetyError):
        s.enable()


def test_clear_fault_goes_to_disabled_not_enabled(impl):
    """Recovery is two deliberate acts. One call that resumes motion is the bug this prevents."""
    s = supervisor(impl)
    s.enable()
    s.stop("something")
    s.clear_fault()
    assert s.state is impl.ArmState.DISABLED
    assert s.fault == ""
    s.enable()
    assert s.state is impl.ArmState.ENABLED


# --- the watchdog ------------------------------------------------------------------------------------
def test_watchdog_expires_after_the_timeout(impl):
    clock = FakeClock()
    s = supervisor(impl, clock=clock, watchdog_s=0.5)
    s.enable()
    assert not s.watchdog_expired
    clock.advance(0.5)
    assert not s.watchdog_expired, "expiry is STRICTLY more than watchdog_s"
    clock.advance(0.01)
    assert s.watchdog_expired


def test_a_heartbeat_resets_it(impl):
    clock = FakeClock()
    s = supervisor(impl, clock=clock, watchdog_s=0.5)
    s.enable()
    clock.advance(2.0)
    assert s.watchdog_expired
    s.heartbeat()
    assert not s.watchdog_expired


def test_enable_beats_the_watchdog(impl):
    clock = FakeClock()
    s = supervisor(impl, clock=clock, watchdog_s=0.5)
    clock.advance(10.0)
    s.enable()
    assert not s.watchdog_expired


def test_a_stale_watchdog_refuses_a_move(impl):
    clock = FakeClock()
    s = supervisor(impl, clock=clock, watchdog_s=0.5)
    s.enable()
    clock.advance(3.0)
    problems = s.check_move({"wrist_flex": 10.0})
    assert len(problems) == 1 and "watchdog" in problems[0]


# --- the gate ------------------------------------------------------------------------------------------
def test_a_legal_move_from_enabled_has_no_problems(impl):
    s = supervisor(impl)
    s.enable()
    assert s.check_move({"wrist_flex": 10.0}, tool_point=[0.25, 0.0, 0.12],
                        present_deg={"wrist_flex": 0.0}) == []


def test_refuses_while_disabled(impl):
    s = supervisor(impl)
    problems = s.check_move({"wrist_flex": 10.0})
    assert any("disabled" in p for p in problems)


def test_refuses_while_stopped_and_names_the_fault(impl):
    s = supervisor(impl)
    s.enable()
    s.stop("elbow_flex overloaded")
    problems = s.check_move({"wrist_flex": 10.0})
    assert any("stopped" in p and "elbow_flex" in p for p in problems)


def test_refuses_a_goal_outside_the_joint_limits(impl):
    s = supervisor(impl)
    s.enable()
    problems = s.check_move({"wrist_flex": 80.0})
    assert len(problems) == 1 and "wrist_flex" in problems[0]


def test_a_joint_with_no_limits_is_a_problem_not_a_free_pass(impl):
    s = supervisor(impl)
    s.enable()
    problems = s.check_move({"elbow_twist": 5.0})
    assert len(problems) == 1 and "elbow_twist" in problems[0]


def test_refuses_a_tool_point_outside_the_box(impl):
    s = supervisor(impl)
    s.enable()
    problems = s.check_move({"wrist_flex": 10.0}, tool_point=[0.25, 0.0, -0.05])
    assert len(problems) == 1 and "workspace box" in problems[0] and "z" in problems[0]


def test_refuses_a_step_that_is_too_large(impl):
    s = supervisor(impl, max_joint_step_deg=30.0)
    s.enable()
    problems = s.check_move({"wrist_flex": 35.0}, present_deg={"wrist_flex": 0.0})
    assert len(problems) == 1 and "step" in problems[0]


def test_the_step_check_is_skipped_when_the_present_pose_is_unknown(impl):
    s = supervisor(impl, max_joint_step_deg=1.0)
    s.enable()
    assert s.check_move({"wrist_flex": 35.0}) == []


def test_every_problem_is_reported_not_just_the_first(impl):
    """A caller that fixes one problem and retries should not find the next one by moving."""
    clock = FakeClock()
    s = supervisor(impl, clock=clock)
    s.enable()
    clock.advance(5.0)
    problems = s.check_move({"wrist_flex": 80.0, "elbow_flex": 90.0},
                            tool_point=[0.25, 0.0, -0.05], present_deg={"wrist_flex": 0.0})
    assert len(problems) >= 4       # watchdog + box + two joint limits (+ step)


def test_the_box_check_only_runs_when_a_point_is_given(impl):
    s = supervisor(impl)
    s.enable()
    assert s.check_move({"wrist_flex": 10.0}) == []


def test_a_boundary_value_is_accepted(impl):
    s = supervisor(impl)
    s.enable()
    assert s.check_move({"wrist_flex": 40.0}) == []
    assert s.check_move({"wrist_flex": 10.0}, tool_point=[0.10, -0.22, 0.02]) == []


def test_the_gripper_has_its_own_limits(impl):
    s = supervisor(impl)
    s.enable()
    assert s.check_move({"gripper": 30.0}) == []
    assert len(s.check_move({"gripper": 0.0})) == 1
