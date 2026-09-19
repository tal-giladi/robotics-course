"""15.08 — reference solution: next_phase, verify_grasp and the retry policy.

The same functions live inside a running pipeline at ``15-manipulation/code/pick_place_fsm.py``.

The pipeline is a state machine with three decisions in it:

    next_phase(phase)                   where do I go when nothing went wrong?
    verify_grasp(reading, expected)     is anything actually in the jaws?
    RetryPolicy.decide(outcome, ...)    what do I do about THIS failure?

Everything else — motion, perception, the gripper — is plumbing around those three.

The reference implementation lives at ``15-manipulation/code/pick_place_fsm.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


# --- given -----------------------------------------------------------------------------------
class Phase(str, Enum):
    """One phase per thing that can independently fail. That is the whole design rule."""

    PERCEIVE = "PERCEIVE"
    PLAN = "PLAN"
    TO_PRE_GRASP = "TO_PRE_GRASP"
    APPROACH = "APPROACH"
    CLOSE = "CLOSE"
    VERIFY = "VERIFY"
    LIFT = "LIFT"
    VERIFY_HELD = "VERIFY_HELD"
    TRANSPORT = "TRANSPORT"
    PLACE = "PLACE"
    OPEN = "OPEN"
    RETREAT = "RETREAT"
    DONE = "DONE"
    FAILED = "FAILED"


#: The nominal order, first to last.
NOMINAL: list[Phase] = [
    Phase.PERCEIVE, Phase.PLAN, Phase.TO_PRE_GRASP, Phase.APPROACH, Phase.CLOSE,
    Phase.VERIFY, Phase.LIFT, Phase.VERIFY_HELD, Phase.TRANSPORT, Phase.PLACE,
    Phase.OPEN, Phase.RETREAT, Phase.DONE,
]


@dataclass(frozen=True)
class Outcome:
    """What a phase did. ``tag`` is what the policy switches on — never a bare bool."""

    tag: str
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.tag == "ok"


OK = Outcome("ok")


class Action(str, Enum):
    CONTINUE = "continue"      # nominal: go to the next phase
    RETRY = "retry"            # run the same phase again — the world did not change
    REPERCEIVE = "reperceive"  # back to PERCEIVE — the world DID change, the plan is stale
    NUDGE = "nudge"            # change the world on purpose, then re-perceive
    SKIP_OBJECT = "skip"       # this object is not happening; try a different one
    ABORT = "abort"            # stop, with a reason


@dataclass(frozen=True)
class GripperReading:
    """What the gripper can tell you after it closes, with no extra hardware.

    width_mm: the measured jaw opening (a bus servo reports its position — 14.03).
    load:     the servo's load as a fraction of its limit.
    """

    width_mm: float
    load: float


# --- implemented ------------------------------------------------------------------------------
def next_phase(phase: Phase) -> Phase:
    if phase in (Phase.DONE, Phase.FAILED):
        return phase
    return NOMINAL[NOMINAL.index(phase) + 1]


def verify_grasp(reading: GripperReading, expected_width_mm: float, *,
                 width_tol_mm: float = 6.0, empty_width_mm: float = 4.0,
                 min_load: float = 0.10) -> Outcome:
    if reading.width_mm <= empty_width_mm:
        return Outcome("empty_grasp", f"jaws closed to {reading.width_mm:.1f} mm")
    if reading.load < min_load:
        return Outcome("empty_grasp", f"no load ({reading.load:.2f}) at {reading.width_mm:.1f} mm")
    if abs(reading.width_mm - expected_width_mm) > width_tol_mm:
        return Outcome("partial_grasp",
                       f"held {reading.width_mm:.1f} mm, expected {expected_width_mm:.1f} mm")
    return OK


@dataclass(frozen=True)
class RetryPolicy:
    """What to do about each failure tag, and how many times."""

    max_attempts_per_object: int = 3
    max_total_attempts: int = 15
    retry_in_place: frozenset[str] = frozenset({"plan_failed", "place_blocked"})
    reperceive: frozenset[str] = frozenset({"empty_grasp", "partial_grasp", "dropped", "collision"})
    give_up: frozenset[str] = frozenset({"no_grasp", "unreachable"})
    nudge_after: int = 2
    allow_nudge: bool = False

    def decide(self, outcome: Outcome, attempts_on_object: int, total_attempts: int) -> Action:
        if outcome.ok:
            return Action.CONTINUE
        if total_attempts >= self.max_total_attempts:
            return Action.ABORT
        if outcome.tag in self.give_up:
            return Action.SKIP_OBJECT
        if attempts_on_object >= self.max_attempts_per_object:
            return Action.SKIP_OBJECT
        if outcome.tag in self.retry_in_place:
            return Action.RETRY
        if outcome.tag in self.reperceive:
            if self.allow_nudge and attempts_on_object >= self.nudge_after:
                return Action.NUDGE
            return Action.REPERCEIVE
        return Action.ABORT
