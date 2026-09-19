"""Tests for 15.08 — phases, grasp verification and the retry policy."""

from __future__ import annotations

import pytest


# ---------------------------------------------------------------- the phase sequence
def test_nominal_sequence(impl):
    phase = impl.Phase.PERCEIVE
    seen = [phase]
    for _ in range(20):
        phase = impl.next_phase(phase)
        seen.append(phase)
        if phase is impl.Phase.DONE:
            break
    assert seen == impl.NOMINAL


def test_verification_follows_closing(impl):
    """CLOSE is never the last word: something must check what is in the jaws."""
    assert impl.next_phase(impl.Phase.CLOSE) is impl.Phase.VERIFY
    assert impl.next_phase(impl.Phase.LIFT) is impl.Phase.VERIFY_HELD


def test_terminal_phases_absorb(impl):
    assert impl.next_phase(impl.Phase.DONE) is impl.Phase.DONE
    assert impl.next_phase(impl.Phase.FAILED) is impl.Phase.FAILED


# ---------------------------------------------------------------- grasp verification
def test_a_good_grasp_passes(impl):
    assert impl.verify_grasp(impl.GripperReading(30.5, 0.55), 30.0).ok


def test_fully_closed_jaws_are_empty(impl):
    out = impl.verify_grasp(impl.GripperReading(1.2, 0.85), 30.0)
    assert out.tag == "empty_grasp"


def test_high_load_does_not_rescue_a_fully_closed_jaw(impl):
    """A jaw closed on its own stop reads a healthy load. Width is checked FIRST for a reason."""
    assert impl.verify_grasp(impl.GripperReading(0.5, 0.99), 30.0).tag == "empty_grasp"


def test_no_load_is_empty_even_at_a_plausible_width(impl):
    out = impl.verify_grasp(impl.GripperReading(29.0, 0.02), 30.0)
    assert out.tag == "empty_grasp"


def test_a_corner_grasp_is_partial(impl):
    out = impl.verify_grasp(impl.GripperReading(17.0, 0.5), 30.0)
    assert out.tag == "partial_grasp"


def test_two_objects_at_once_is_partial(impl):
    """Wider than expected is just as wrong as narrower, and much more common than people think."""
    assert impl.verify_grasp(impl.GripperReading(41.0, 0.6), 30.0).tag == "partial_grasp"


def test_tolerances_are_respected(impl):
    assert impl.verify_grasp(impl.GripperReading(25.0, 0.5), 30.0, width_tol_mm=6.0).ok
    assert not impl.verify_grasp(impl.GripperReading(25.0, 0.5), 30.0, width_tol_mm=3.0).ok
    assert impl.verify_grasp(impl.GripperReading(3.0, 0.5), 3.0, empty_width_mm=1.0).ok


def test_every_failure_carries_a_detail(impl):
    for reading in (impl.GripperReading(0.5, 0.9), impl.GripperReading(29.0, 0.01),
                    impl.GripperReading(17.0, 0.5)):
        out = impl.verify_grasp(reading, 30.0)
        assert not out.ok
        assert out.detail, "a failure with no detail is unusable in a log"


# ---------------------------------------------------------------- the retry policy
def test_success_continues(impl):
    assert impl.RetryPolicy().decide(impl.OK, 0, 0) is impl.Action.CONTINUE


def test_a_moved_world_means_look_again(impl):
    p = impl.RetryPolicy()
    for tag in ("empty_grasp", "partial_grasp", "dropped", "collision"):
        assert p.decide(impl.Outcome(tag), 0, 0) is impl.Action.REPERCEIVE


def test_an_unchanged_world_may_be_retried_in_place(impl):
    p = impl.RetryPolicy()
    assert p.decide(impl.Outcome("plan_failed"), 0, 0) is impl.Action.RETRY


def test_holding_an_object_forbids_skipping_it(impl):
    """place_blocked happens while the object is IN the gripper: 'skip' is not a legal move."""
    assert impl.RetryPolicy().decide(impl.Outcome("place_blocked"), 0, 0) is impl.Action.RETRY


def test_impossible_grasps_skip_the_object(impl):
    p = impl.RetryPolicy()
    assert p.decide(impl.Outcome("no_grasp"), 0, 0) is impl.Action.SKIP_OBJECT
    assert p.decide(impl.Outcome("unreachable"), 0, 0) is impl.Action.SKIP_OBJECT


def test_per_object_budget_skips(impl):
    p = impl.RetryPolicy(max_attempts_per_object=3)
    assert p.decide(impl.Outcome("empty_grasp"), 2, 2) is impl.Action.REPERCEIVE
    assert p.decide(impl.Outcome("empty_grasp"), 3, 3) is impl.Action.SKIP_OBJECT


def test_the_global_budget_aborts_and_outranks_everything(impl):
    p = impl.RetryPolicy(max_total_attempts=5)
    assert p.decide(impl.Outcome("plan_failed"), 0, 5) is impl.Action.ABORT
    assert p.decide(impl.Outcome("no_grasp"), 0, 5) is impl.Action.ABORT


def test_nudge_replaces_reperceive_once_it_is_enabled_and_earned(impl):
    p = impl.RetryPolicy(allow_nudge=True, nudge_after=2)
    assert p.decide(impl.Outcome("empty_grasp"), 0, 0) is impl.Action.REPERCEIVE
    assert p.decide(impl.Outcome("empty_grasp"), 1, 1) is impl.Action.REPERCEIVE
    assert p.decide(impl.Outcome("empty_grasp"), 2, 2) is impl.Action.NUDGE


def test_nudge_stays_off_unless_allowed(impl):
    p = impl.RetryPolicy(allow_nudge=False, nudge_after=2)
    assert p.decide(impl.Outcome("empty_grasp"), 2, 2) is impl.Action.REPERCEIVE


def test_an_unknown_tag_aborts_rather_than_retrying(impl):
    """A tag nobody handled is a bug. A policy that retries it keeps a broken robot moving."""
    assert impl.RetryPolicy().decide(impl.Outcome("gripper_on_fire"), 0, 0) is impl.Action.ABORT


@pytest.mark.parametrize("attempts", [0, 1, 2, 3, 4])
def test_the_policy_never_returns_continue_for_a_failure(impl, attempts):
    p = impl.RetryPolicy(allow_nudge=True)
    for tag in ("empty_grasp", "partial_grasp", "dropped", "collision", "plan_failed",
                "place_blocked", "no_grasp", "unreachable", "mystery"):
        assert p.decide(impl.Outcome(tag), attempts, attempts) is not impl.Action.CONTINUE
