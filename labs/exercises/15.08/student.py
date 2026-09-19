"""15.08 — the three functions a pick-and-place pipeline is actually made of.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.08``.
Only the standard library is needed — no numpy, no ROS, no arm.

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


# --- implement these ---------------------------------------------------------------------------
def next_phase(phase: Phase) -> Phase:
    """The phase that follows ``phase`` when nothing went wrong.

    ``DONE`` and ``FAILED`` are absorbing: they return themselves. Everything else returns the
    next entry in ``NOMINAL``.
    """
    raise NotImplementedError("next_phase")  # TODO(student)


def verify_grasp(reading: GripperReading, expected_width_mm: float, *,
                 width_tol_mm: float = 6.0, empty_width_mm: float = 4.0,
                 min_load: float = 0.10) -> Outcome:
    """Three questions, cheapest first. Return ``OK``, ``empty_grasp`` or ``partial_grasp``.

    1. ``reading.width_mm <= empty_width_mm`` → ``Outcome("empty_grasp", ...)``. The jaws closed
       all the way, so there is nothing in there whatever the load says.
    2. ``reading.load < min_load`` → ``Outcome("empty_grasp", ...)``. A second opinion, not the
       first: a jaw closed on its own stop reads a *high* load, so load alone proves nothing.
    3. ``abs(reading.width_mm - expected_width_mm) > width_tol_mm`` →
       ``Outcome("partial_grasp", ...)``. This is the one that catches a corner grasp, two objects
       at once, or the wrapper instead of the sweet.

    Otherwise ``OK``. Put something useful in ``detail`` — you will read it in a log at 23:00.
    """
    raise NotImplementedError("verify_grasp")  # TODO(student)


@dataclass(frozen=True)
class RetryPolicy:
    """What to do about each failure tag, and how many times.

    The distinction that matters: a failure that left the world **unchanged** (a planner timeout)
    can be retried in place; a failure that **changed** the world (the pads brushed the object)
    has invalidated the perception, so retrying the same plan aims at where the object used to be.
    """

    max_attempts_per_object: int = 3
    max_total_attempts: int = 15
    retry_in_place: frozenset[str] = frozenset({"plan_failed", "place_blocked"})
    reperceive: frozenset[str] = frozenset({"empty_grasp", "partial_grasp", "dropped", "collision"})
    give_up: frozenset[str] = frozenset({"no_grasp", "unreachable"})
    nudge_after: int = 2
    allow_nudge: bool = False

    def decide(self, outcome: Outcome, attempts_on_object: int, total_attempts: int) -> Action:
        """The whole policy, in one testable function. Check the conditions **in this order**:

        1. ``outcome.ok``                                → ``CONTINUE``
        2. ``total_attempts >= max_total_attempts``      → ``ABORT`` (the global budget wins over
           everything, including a tag you would normally give up on — one place to look when the
           run stops)
        3. tag in ``give_up``                            → ``SKIP_OBJECT``
        4. ``attempts_on_object >= max_attempts_per_object`` → ``SKIP_OBJECT``
        5. tag in ``retry_in_place``                     → ``RETRY``
        6. tag in ``reperceive``                         → ``NUDGE`` if ``allow_nudge`` and
           ``attempts_on_object >= nudge_after``, else ``REPERCEIVE``
        7. anything else                                 → ``ABORT``

        Step 7 is deliberate: an unrecognised tag is a **bug in your code**, not a thing to retry.
        A policy that retries what it does not understand will keep a broken robot moving.
        """
        raise NotImplementedError("RetryPolicy.decide")  # TODO(student)
