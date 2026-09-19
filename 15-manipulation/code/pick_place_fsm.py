"""Lesson 15.08 — a pick-and-place pipeline that survives its own failure rate.

A state machine, a verification step with three independent signals, and a retry policy that
knows the difference between "try again" and "look again". The simulated robot has the error
budget 15.07 measured (a ~5 mm median lateral error), so the numbers here are what that budget
does to a *task*, not to a pose.

    py pick_place_fsm.py                  every table printed in the lesson
    py pick_place_fsm.py --only policy    the retry-policy comparison

Nothing here needs ROS, an arm, or the perception stack: ``SimWorld`` is 80 lines and its point is
that a failed grasp **moves the object**, which is why retrying a stale plan spends the attempt
budget pushing the object further away.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field, replace
from enum import Enum

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]


# ============================================================================== the phases
class Phase(str, Enum):
    """One phase per thing that can independently fail. That is the whole design rule."""

    PERCEIVE = "PERCEIVE"          # look at the table
    PLAN = "PLAN"                  # 15.05 + 15.07: a grasp and four reachable waypoints
    TO_PRE_GRASP = "TO_PRE_GRASP"  # free motion, fast
    APPROACH = "APPROACH"          # straight line, slow, the only segment that can collide
    CLOSE = "CLOSE"                # close the jaws
    VERIFY = "VERIFY"              # is anything actually in there?
    LIFT = "LIFT"                  # straight up
    VERIFY_HELD = "VERIFY_HELD"    # is it STILL in there, now that gravity has had a turn?
    TRANSPORT = "TRANSPORT"        # over to the placing area
    PLACE = "PLACE"                # down to the release height
    OPEN = "OPEN"                  # let go
    RETREAT = "RETREAT"            # up and away before anything else moves
    DONE = "DONE"
    FAILED = "FAILED"


#: The nominal order. ``next_phase`` follows it; only failures leave it.
NOMINAL: list[Phase] = [
    Phase.PERCEIVE, Phase.PLAN, Phase.TO_PRE_GRASP, Phase.APPROACH, Phase.CLOSE,
    Phase.VERIFY, Phase.LIFT, Phase.VERIFY_HELD, Phase.TRANSPORT, Phase.PLACE,
    Phase.OPEN, Phase.RETREAT, Phase.DONE,
]


def next_phase(phase: Phase) -> Phase:
    """The phase that follows ``phase`` when nothing went wrong."""
    if phase in (Phase.DONE, Phase.FAILED):
        return phase
    return NOMINAL[NOMINAL.index(phase) + 1]


# ============================================================================== outcomes
@dataclass(frozen=True)
class Outcome:
    """What a phase did. ``tag`` is the thing the policy switches on — never a bare bool.

    Tags in use:
      ok                 the phase did what it was asked
      no_object          perception found nothing graspable
      no_grasp           15.05 refused (too wide, blocked, unstable)
      unreachable        a waypoint failed IK (15.07)
      plan_failed        the motion planner found no path
      collision          the arm stopped on contact during the approach
      empty_grasp        the jaws closed on nothing
      partial_grasp      the jaws closed on something the wrong size
      dropped            held after closing, gone after the lift
      place_blocked      the place target is occupied
      aborted            a human or a watchdog stopped it
    """

    tag: str
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.tag == "ok"


OK = Outcome("ok")


# ============================================================================== retry policy
class Action(str, Enum):
    CONTINUE = "continue"      # nominal: go to the next phase
    RETRY = "retry"            # run the same phase again (transient: a planner seed, a timeout)
    REPERCEIVE = "reperceive"  # go back to PERCEIVE: the WORLD changed, the plan is stale
    NUDGE = "nudge"            # change the world on purpose, then re-perceive
    SKIP_OBJECT = "skip"       # this object is not happening; try a different one
    ABORT = "abort"            # stop, with a reason


@dataclass(frozen=True)
class RetryPolicy:
    """What to do about each failure tag, and how many times.

    The distinction that matters, and the one people get wrong: a failure that **left the world
    unchanged** (a planner timeout) can be retried in place; a failure that **changed the world**
    (the pads brushed the object) has invalidated the perception, and retrying from the same plan
    aims at where the object used to be.
    """

    max_attempts_per_object: int = 3
    max_total_attempts: int = 15
    #: tags that may be retried in place, because the world did not move. ``place_blocked``
    #: belongs here and not in ``give_up``: you are HOLDING the object, so "skip it" is not an
    #: available move — the machine must find somewhere else to put it.
    retry_in_place: frozenset[str] = frozenset({"plan_failed", "place_blocked"})
    #: tags that require looking again before doing anything
    reperceive: frozenset[str] = frozenset({"empty_grasp", "partial_grasp", "dropped", "collision"})
    #: tags that mean this object cannot be picked at all right now
    give_up: frozenset[str] = frozenset({"no_grasp", "unreachable"})
    #: after this many failed attempts on one object, change the world instead of re-trying
    nudge_after: int = 2
    allow_nudge: bool = False

    def decide(self, outcome: Outcome, attempts_on_object: int, total_attempts: int) -> Action:
        """The whole policy, in one function you can unit-test."""
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
        return Action.ABORT           # an unknown tag is a bug, not a thing to retry


# ============================================================================== verification
@dataclass(frozen=True)
class GripperReading:
    """What the gripper can tell you after it closes, without any extra hardware.

    width_mm:  the commanded-vs-actual jaw opening. A hobby bus servo reports its position, so
               this is free (14.03). An empty close reads ~0; a good grasp reads the object width.
    load:      the servo's load/current as a fraction of its limit. Free on the same bus.
    """

    width_mm: float
    load: float


def verify_grasp(reading: GripperReading, expected_width_mm: float, *,
                 width_tol_mm: float = 6.0, empty_width_mm: float = 4.0,
                 min_load: float = 0.10) -> Outcome:
    """Three questions in order, cheapest first. Returns ``ok`` / ``empty_grasp`` / ``partial_grasp``.

    1. Did the jaws close all the way? Then there is nothing in there, whatever the load says.
    2. Is the load plausible? A closed-on-nothing jaw at its stop can read a high load too, so
       this is a *second* opinion, not the first.
    3. Is the width what the object's width should be? This is the one that catches the grasp that
       got a corner, or two objects at once, or the wrapper and not the sweet.

    None of the three is reliable alone. That is the lesson, and the reason the argument list has
    three tolerances rather than one threshold.
    """
    if reading.width_mm <= empty_width_mm:
        return Outcome("empty_grasp", f"jaws closed to {reading.width_mm:.1f} mm")
    if reading.load < min_load:
        return Outcome("empty_grasp", f"no load ({reading.load:.2f}) at {reading.width_mm:.1f} mm")
    if abs(reading.width_mm - expected_width_mm) > width_tol_mm:
        return Outcome("partial_grasp",
                       f"held {reading.width_mm:.1f} mm, expected {expected_width_mm:.1f} mm")
    return OK


# ============================================================================== the world
@dataclass
class SimObject:
    name: str
    x: float
    y: float
    width_mm: float
    present: bool = True


@dataclass
class SimWorld:
    """A table the machine cannot see directly, with the failure modes that actually happen.

    The important behaviour: a missed grasp **nudges the object**. That single line is why
    "retry the same plan" is a worse policy than "look again first", and no amount of control
    theory substitutes for it.
    """

    objects: list[SimObject]
    rng: np.random.Generator
    lateral_sigma_mm: float = 4.0       # the 15.07 budget, as a standard deviation
    jaw_stroke_mm: float = 45.0
    plan_fail_p: float = 0.08           # a sampling planner that sometimes does not find a path
    slip_p: float = 0.05                # held at the closing height, gone after the lift
    collision_p: float = 0.03           # the approach stops on contact
    place_blocked_p: float = 0.02
    nudge_mm: float = 12.0              # how far a failed grasp moves the object
    #: probability that a MISS produces a gripper reading that looks like a good grasp — a pad
    #: caught the edge and the object slid out as the jaws finished closing. This is the case no
    #: gripper-side verification can catch, and the reason VERIFY_HELD exists after the lift.
    convincing_miss_p: float = 0.15

    def perceive(self) -> list[SimObject]:
        """What the perception stack reports: present objects, with 15.04-scale position noise."""
        out = []
        for o in self.objects:
            if not o.present:
                continue
            out.append(replace(o, x=o.x + self.rng.normal(0, 0.003),
                               y=o.y + self.rng.normal(0, 0.003)))
        return out

    def attempt_grasp(self, target: SimObject) -> tuple[Outcome, GripperReading]:
        """Close the jaws where the plan said. Returns the outcome AND the gripper reading."""
        truth = next((o for o in self.objects if o.name == target.name and o.present), None)
        if truth is None:
            return Outcome("empty_grasp", "object is not there any more"), GripperReading(0.5, 0.02)

        # how far off, laterally, the pads actually arrive (15.07's budget)
        err_mm = abs(self.rng.normal(0.0, self.lateral_sigma_mm))
        stale_mm = math.hypot(target.x - truth.x, target.y - truth.y) * 1000.0
        err_mm = math.hypot(err_mm, stale_mm)
        slack_mm = (self.jaw_stroke_mm - truth.width_mm) / 2.0

        if err_mm > slack_mm:
            truth.x += self.rng.normal(0, self.nudge_mm / 1000.0)   # a pad pushed it
            truth.y += self.rng.normal(0, self.nudge_mm / 1000.0)
            miss = Outcome("empty_grasp", f"off by {err_mm:.1f} mm, slack {slack_mm:.1f} mm")
            if self.rng.random() < self.convincing_miss_p:
                # the pads squeezed the object, it rolled out as they finished closing, and the
                # reading at the moment of measurement is indistinguishable from a real grasp
                return miss, GripperReading(truth.width_mm + self.rng.normal(0, 1.0),
                                            self.rng.uniform(0.35, 0.8))
            return miss, GripperReading(self.rng.uniform(0.0, 2.0), self.rng.uniform(0.5, 0.9))

        if err_mm > 0.75 * slack_mm:                                # a corner grasp
            return Outcome("partial_grasp", f"off by {err_mm:.1f} mm"), \
                GripperReading(truth.width_mm * self.rng.uniform(0.45, 0.70),
                               self.rng.uniform(0.3, 0.7))

        return OK, GripperReading(truth.width_mm + self.rng.normal(0, 1.0),
                                  self.rng.uniform(0.35, 0.8))

    def lift(self, target: SimObject) -> Outcome:
        if self.rng.random() < self.slip_p:
            truth = next((o for o in self.objects if o.name == target.name), None)
            if truth is not None:
                truth.x += self.rng.normal(0, self.nudge_mm / 1000.0)
                truth.y += self.rng.normal(0, self.nudge_mm / 1000.0)
            return Outcome("dropped", "held at the closing height, gone after the lift")
        return OK

    def take(self, target: SimObject) -> None:
        for o in self.objects:
            if o.name == target.name:
                o.present = False

    def nudge(self, target: SimObject) -> None:
        """Deliberately push the object to a new spot — the 'change the world' recovery."""
        for o in self.objects:
            if o.name == target.name:
                o.x += self.rng.normal(0, 0.02)
                o.y += self.rng.normal(0, 0.02)


# ============================================================================== the machine
@dataclass
class Attempt:
    object_name: str
    phase: Phase
    outcome: Outcome
    action: Action


@dataclass
class RunReport:
    picked: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    log: list[Attempt] = field(default_factory=list)
    aborted: str | None = None


def run_pick_and_place(world: SimWorld, policy: RetryPolicy, *, max_steps: int = 400) -> RunReport:
    """Clear the table, one object at a time, with verification and a retry policy.

    Deliberately written as an explicit loop over phases rather than as a class hierarchy: the
    whole value of a state machine here is that you can read the transition table.
    """
    report = RunReport()
    total_attempts = 0
    attempts_on_object = 0
    seen: list[SimObject] = []
    target: SimObject | None = None
    phase = Phase.PERCEIVE
    skipped: set[str] = set()
    holding = False

    for _ in range(max_steps):
        if phase is Phase.DONE or phase is Phase.FAILED:
            break

        outcome = OK
        if phase is Phase.PERCEIVE:
            seen = [o for o in world.perceive() if o.name not in skipped]
            if not seen:
                break
            target = min(seen, key=lambda o: o.width_mm)      # easiest first: narrowest
        elif phase is Phase.PLAN:
            if target.width_mm + 10.0 > world.jaw_stroke_mm:
                outcome = Outcome("no_grasp", f"{target.width_mm:.0f} mm needs more stroke")
            elif world.rng.random() < world.plan_fail_p:
                outcome = Outcome("plan_failed", "no path found")
        elif phase is Phase.APPROACH:
            if world.rng.random() < world.collision_p:
                outcome = Outcome("collision", "stopped on contact during the approach")
        elif phase is Phase.CLOSE:
            truth, reading = world.attempt_grasp(target)
            holding = truth.ok
            # The machine does NOT get to see ``truth``: all it has is the gripper reading.
            outcome = verify_grasp(reading, target.width_mm)
            phase = Phase.VERIFY                               # the verification IS the outcome
        elif phase is Phase.VERIFY_HELD:
            if not holding:
                outcome = Outcome("dropped", "nothing was in the jaws; verification missed it")
            else:
                outcome = world.lift(target)
                holding = outcome.ok
        elif phase is Phase.PLACE:
            if world.rng.random() < world.place_blocked_p:
                outcome = Outcome("place_blocked", "the place target is occupied")
        elif phase is Phase.RETREAT:
            world.take(target)
            report.picked.append(target.name)
            attempts_on_object = 0
            holding = False
            phase = Phase.PERCEIVE
            report.log.append(Attempt(target.name, Phase.RETREAT, OK, Action.CONTINUE))
            continue

        action = policy.decide(outcome, attempts_on_object, total_attempts)
        report.log.append(Attempt(target.name if target else "-", phase, outcome, action))

        if not outcome.ok:
            total_attempts += 1
            attempts_on_object += 1

        if action is Action.CONTINUE:
            phase = next_phase(phase)
        elif action is Action.RETRY:
            pass                                               # same phase again
        elif action is Action.REPERCEIVE:
            holding = False                # whatever happened, the jaws are empty: open them
            phase = Phase.PERCEIVE
        elif action is Action.NUDGE:
            world.nudge(target)
            phase = Phase.PERCEIVE
        elif action is Action.SKIP_OBJECT:
            skipped.add(target.name)
            report.skipped.append(target.name)
            attempts_on_object = 0
            phase = Phase.PERCEIVE
        else:
            report.aborted = f"{outcome.tag}: {outcome.detail}"
            phase = Phase.FAILED
    return report


# ============================================================================== scenes
def table_of_five(rng: np.random.Generator, **kw) -> SimWorld:
    """Four graspable objects and one that is simply too wide for a 45 mm jaw (15.05)."""
    return SimWorld([
        SimObject("eraser", 0.188, 0.161, 24.7),
        SimObject("wooden block", 0.237, 0.059, 30.0),
        SimObject("small cube", 0.210, -0.060, 20.0),
        SimObject("wide box", 0.230, 0.110, 35.0),
        SimObject("drinks can", 0.299, -0.101, 66.0),
    ], rng, **kw)


# ============================================================================== demos
def demo_run() -> None:
    print("--- one run, every transition (policy: re-perceive after a failed grasp) ---")
    world = table_of_five(np.random.default_rng(22))
    report = run_pick_and_place(world, RetryPolicy())
    print(f"{'object':<14}{'phase':<14}{'outcome':<16}{'action':<12}detail")
    for a in report.log:
        if a.outcome.ok and a.phase not in (Phase.VERIFY, Phase.RETREAT):
            continue
        print(f"{a.object_name:<14}{a.phase.value:<14}{a.outcome.tag:<16}"
              f"{a.action.value:<12}{a.outcome.detail}")
    print(f"\n  picked: {report.picked}")
    print(f"  skipped: {report.skipped}   aborted: {report.aborted}\n")


def demo_policy() -> None:
    print("--- retry policies, 300 tables each (4 graspable objects + 1 too wide) ---")
    print(f"{'policy':<34}{'picked/4':>10}{'attempts':>10}{'skipped':>9}{'aborted':>9}")
    policies = {
        "no policy: stop on 1st failure": RetryPolicy(max_attempts_per_object=1,
                                                      reperceive=frozenset(),
                                                      retry_in_place=frozenset()),
        "blind retry (same plan)": RetryPolicy(
            retry_in_place=frozenset({"plan_failed", "empty_grasp", "partial_grasp",
                                      "dropped", "collision"}),
            reperceive=frozenset()),
        "re-perceive after a failure": RetryPolicy(),
        "  ... + nudge after 2 tries": RetryPolicy(allow_nudge=True),
        "  ... + 5 attempts per object": RetryPolicy(allow_nudge=True, max_attempts_per_object=5,
                                                     max_total_attempts=25),
    }
    for label, policy in policies.items():
        picked, attempts, skipped, aborted = [], [], [], 0
        for seed in range(300):
            world = table_of_five(np.random.default_rng(1000 + seed))
            r = run_pick_and_place(world, policy)
            picked.append(len(r.picked))
            attempts.append(sum(1 for a in r.log if not a.outcome.ok))
            skipped.append(len(r.skipped))
            aborted += int(r.aborted is not None)
        print(f"{label:<34}{np.mean(picked):>10.2f}{np.mean(attempts):>10.2f}"
              f"{np.mean(skipped):>9.2f}{aborted / 300:>9.2f}")
    print("  'picked' counts the 4 graspable objects; the drinks can is correctly skipped every")
    print("  time (66 mm > 45 mm stroke). Read the attempts column next to the picked column:")
    print("  blind retry spends 7 failed attempts to pick 2.4 objects, re-perception spends 5 to")
    print("  pick 3.6. The failed grasp MOVED the object, so the retried plan aims at where it")
    print("  used to be; the extra attempts are not merely wasted, they push the object further.\n")


def demo_verification() -> None:
    print("--- what each verification signal catches, on its own and together (2000 closes) ---")
    rng = np.random.default_rng(11)
    world = table_of_five(rng)
    truth, width_only, load_only, both = [], [], [], []
    for _ in range(2000):
        obj = SimObject("x", 0.22, 0.05, 30.0)
        world.objects = [obj]
        out, reading = world.attempt_grasp(replace(obj))
        truth.append(out.ok)
        width_only.append(reading.width_mm > 4.0)
        load_only.append(reading.load >= 0.10)
        both.append(verify_grasp(reading, 30.0).ok)
    truth = np.array(truth)
    print(f"{'signal':<34}{'misses caught':>15}{'false alarms':>15}")
    for label, pred in (("jaws not fully closed", np.array(width_only)),
                        ("servo load above 0.10", np.array(load_only)),
                        ("all three (verify_grasp)", np.array(both))):
        caught = float(np.mean(~pred[~truth])) if (~truth).any() else float("nan")
        false = float(np.mean(~pred[truth])) if truth.any() else float("nan")
        print(f"{label:<34}{caught * 100:>14.1f}%{false * 100:>14.1f}%")
    print(f"  ({int((~truth).sum())} of 2000 closes actually failed)")
    print("  The load signal alone is useless here: a jaw closed on nothing hits its own stop and")
    print("  reads a healthy load. 'Not fully closed' alone misses the corner grasp, which reads a")
    print("  plausible non-zero width. The three together catch most of it - and the residual is")
    print("  the miss that reads exactly like a grasp, which no gripper-side signal can see. That")
    print("  one is caught after the LIFT, by looking at the table again (VERIFY_HELD).\n")


def demo_error() -> None:
    print("--- the 15.07 error budget, expressed as a task success rate (200 tables each) ---")
    print(f"{'lateral sigma mm':>18}" + "".join(f"{f'{w} mm obj':>12}" for w in (15, 20, 25, 30, 35)))
    for sigma in (1.0, 2.0, 4.0, 6.0, 8.0):
        row = [f"{sigma:>18.1f}"]
        for width in (15, 20, 25, 30, 35):
            ok = 0
            for seed in range(200):
                rng = np.random.default_rng(seed)
                world = SimWorld([SimObject("o", 0.22, 0.05, float(width))], rng,
                                 lateral_sigma_mm=sigma, plan_fail_p=0.0, slip_p=0.0,
                                 collision_p=0.0, place_blocked_p=0.0)
                r = run_pick_and_place(world, RetryPolicy())
                ok += int(len(r.picked) == 1)
            row.append(f"{ok / 200 * 100:>11.0f}%")
        print("".join(row))
    print("  Read the 4 mm row: that is this course's measured budget (15.07). With verification,")
    print("  retries and re-perception a 30 mm object clears 96% and a 35 mm one manages 66%.")
    print("  A single open-loop grasp of the 30 mm block succeeds about 4 times in 5 (15.07); the")
    print("  policy is what turns that into a task you can leave running.\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["run", "policy", "verification", "error"])
    args = ap.parse_args(argv)
    if args.only in (None, "run"):
        demo_run()
    if args.only in (None, "policy"):
        demo_policy()
    if args.only in (None, "verification"):
        demo_verification()
    if args.only in (None, "error"):
        demo_error()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
