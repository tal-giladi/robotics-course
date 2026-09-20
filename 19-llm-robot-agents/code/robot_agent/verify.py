"""Verification and closed-loop execution (19.07).

[19.06](../../19.06-task-planning-decomposition.md) ends with a plan that was proved *well-formed*
and then executed open loop. This module adds the missing half: after every step, an **independent
observation** decides whether the world actually changed, and a repair ladder decides what to do
when it did not.

Three ideas, in order of importance:

* **A return code is not evidence.** ``SkillResult.ok`` says the call completed. A ``Verification``
  says the postcondition holds, measured through a *different* channel than the one that did the
  work: proprioception (``get_robot_state``) or exteroception (``detect_objects``).
* **Repair before replan.** Most failures are local (look again, drive closer, re-grasp) and cost
  seconds. Replanning costs a model call and throws away work. The ladder is
  ``retry -> local repair -> replan -> stop and report``, with a budget at every rung.
* **"Done" is a measurement.** The executor's last act is to go and look at the goal. An agent that
  reports success from its own transcript is grading its own homework.

``FaultySkills`` injects the failures this is all for: a gripper that reports a successful grasp on
an empty hand, a state reader that agrees with it, and navigation that silently stops short.

Nothing here needs a model or an API key: ``SearchReplanner`` is a deterministic offline replanner.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

from robot_agent.planning import OBJECT_PARAMS, Goal, Plan, Step, _label_matches, _var
from robot_agent.skills import JsonDict, RobotSkills, SkillResult

# ----------------------------------------------------------------------------------------------
# What a verification is
# ----------------------------------------------------------------------------------------------
VERIFY_LEVELS = ("none", "state", "perception")

#: Failure codes that mean "the observation this step was planned from is no longer true".
#: Retrying such a step unchanged cannot work; the precondition has to be re-established first.
INVALIDATES_BINDING = frozenset({"GRASP_FAILED", "OBJECT_NOT_FOUND"})


@dataclass(frozen=True)
class Verification:
    """The verdict of an independent check on one step's postcondition."""

    skill: str
    ok: bool
    code: str  # VERIFIED | NOT_VERIFIED | UNCHECKABLE | SKIPPED
    method: str  # "result" | "state" | "perception" | "-"
    message: str
    observed: JsonDict = field(default_factory=dict)
    cost_s: float = 0.0  # simulated robot seconds this check spent
    #: Is looking again worth it? Only when the failure was "I did not see what I expected":
    #: a second look then removes a detector miss. When the failure was "I still see what should
    #: be gone", a second look can only *miss* it, which would turn a real failure into a pass.
    repeatable: bool = True

    def __str__(self) -> str:
        return f"[{self.code}/{self.method}] {self.skill}: {self.message}"


@dataclass(frozen=True)
class VerifyConfig:
    level: str = "perception"  # "none": trust the code · "state": proprioception · "perception": look
    place_tolerance_m: float = 0.30
    min_confidence: float = 0.4
    looks: int = 2  # a detector that misses 8 % of the time needs a second look before you believe it


def _state(skills: RobotSkills) -> JsonDict:
    """Proprioception: free (0 simulated seconds), and it can be wrong."""
    return skills.call("get_robot_state").data


def _detections(skills: RobotSkills, cfg: VerifyConfig) -> list[JsonDict]:
    """Exteroception: 0.2 s of robot time, and it misses things."""
    r = skills.call("detect_objects", {"min_confidence": cfg.min_confidence})
    return list(r.data.get("detections", [])) if r.ok else []


# ----------------------------------------------------------------------------------------------
# One verifier per skill
# ----------------------------------------------------------------------------------------------
def verify_navigate(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                    cfg: VerifyConfig) -> Verification:
    place = str(args.get("place"))
    st = _state(skills)
    at = st.get("at_place")
    target = skills.world.places.get(place)
    dist = math.inf
    if target is not None:
        pose = st["pose"]
        dist = math.hypot(target.x - pose["x_m"], target.y - pose["y_m"])
    ok = at == place or dist <= cfg.place_tolerance_m
    return Verification(
        "navigate_to", ok, "VERIFIED" if ok else "NOT_VERIFIED", "state",
        f"at_place={at!r}, {dist:.2f} m from '{place}'" if target else f"'{place}' is not on the map",
        {"at_place": at, "distance_m": round(dist, 2) if math.isfinite(dist) else None})


def verify_pick(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                cfg: VerifyConfig) -> Verification:
    object_id = str(args.get("object_id"))
    st = _state(skills)
    holding = st.get("holding")
    if holding != object_id:
        return Verification("pick", False, "NOT_VERIFIED", "state",
                            f"the skill reported success but the robot holds {holding!r}",
                            {"holding": holding})
    if cfg.level != "perception":
        return Verification("pick", True, "VERIFIED", "state", f"holding {object_id}", {"holding": holding})
    # Exteroception: a held object is in the gripper, so it must NOT still be sitting in the scene.
    still_there = [d for d in _detections(skills, cfg) if d["object_id"] == object_id]
    if still_there:
        return Verification("pick", False, "NOT_VERIFIED", "perception",
                            f"the state says 'holding {object_id}' and the camera still sees it at "
                            f"{still_there[0].get('near_place')}",
                            {"holding": holding, "detection": still_there[0]}, cost_s=0.2,
                            repeatable=False)
    return Verification("pick", True, "VERIFIED", "perception", f"holding {object_id}, no longer in the scene",
                        {"holding": holding}, cost_s=0.2)


def verify_place(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                 cfg: VerifyConfig) -> Verification:
    location = str(args.get("location"))
    st = _state(skills)
    if st.get("holding") is not None:
        return Verification("place", False, "NOT_VERIFIED", "state",
                            f"the gripper still holds {st['holding']!r}", {"holding": st["holding"]})
    if cfg.level != "perception":
        return Verification("place", True, "VERIFIED", "state", "the gripper is empty", {"holding": None})
    placed = result.data.get("placed_pose") or {}
    hits = [d for d in _detections(skills, cfg) if d.get("near_place") == location]
    if not hits:
        return Verification("place", False, "NOT_VERIFIED", "perception",
                            f"nothing is visible on '{location}' after placing", {"placed_pose": placed}, cost_s=0.2)
    return Verification("place", True, "VERIFIED", "perception",
                        f"{hits[0]['label']} ({hits[0]['object_id']}) is on '{location}'",
                        {"detection": hits[0]}, cost_s=0.2)


def verify_detect(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                  cfg: VerifyConfig, label: str | None = None) -> Verification:
    """A detect step's postcondition is about its own result: did it find what the step was for?"""
    if label is None:
        return Verification("detect_objects", True, "UNCHECKABLE", "result",
                            "a detection with no label to match has no postcondition to check")
    hits = [d for d in result.data.get("detections", []) if _label_matches(d["label"], label)]
    ok = bool(hits)
    return Verification("detect_objects", ok, "VERIFIED" if ok else "NOT_VERIFIED", "result",
                        f"{len(hits)} detection(s) matching '{label}'",
                        {"detection": hits[0]} if hits else {"labels_seen": [d["label"] for d in
                                                                            result.data.get("detections", [])]})


VERIFIERS: dict[str, Callable[..., Verification]] = {
    "navigate_to": verify_navigate,
    "pick": verify_pick,
    "place": verify_place,
    "detect_objects": verify_detect,
}


def verify_step(skills: RobotSkills, skill: str, args: Mapping[str, Any], result: SkillResult,
                cfg: VerifyConfig = VerifyConfig(), label: str | None = None) -> Verification:
    """Independently check one step's postcondition. Read-only skills have none."""
    if cfg.level == "none":
        return Verification(skill, result.ok, "SKIPPED", "-", "verification disabled")
    if not result.ok:  # a failed call has nothing to verify; its code is already the diagnosis
        return Verification(skill, False, "NOT_VERIFIED", "result", f"the call itself failed: {result.code}")
    fn = VERIFIERS.get(skill)
    if fn is None:
        return Verification(skill, True, "UNCHECKABLE", "-", f"no verifier for '{skill}' (read-only skill)")
    if skill == "detect_objects":
        return fn(skills, args, result, cfg, label)
    return fn(skills, args, result, cfg)


def verify_goal(skills: RobotSkills, goal: Goal, cfg: VerifyConfig = VerifyConfig()) -> Verification:
    """"Done" is a measurement: drive to the surface and look at it.

    This is the only check that answers the user's question. It costs one navigation plus one
    detection, and it is the difference between "the agent says it finished" and "it is finished".
    """
    st = _state(skills)
    if st.get("at_place") != goal.surface:
        r = skills.call("navigate_to", {"place": goal.surface})
        if not r.ok:
            return Verification("goal", False, "UNCHECKABLE", "perception",
                                f"could not reach '{goal.surface}' to check: {r.code}")
    hits = [d for d in _detections(skills, cfg)
            if d.get("near_place") == goal.surface and _label_matches(d["label"], goal.label)]
    ok = bool(hits)
    return Verification("goal", ok, "VERIFIED" if ok else "NOT_VERIFIED", "perception",
                        f"a '{goal.label}' on '{goal.surface}'" if ok
                        else f"no '{goal.label}' visible on '{goal.surface}'",
                        {"detection": hits[0]} if hits else {}, cost_s=0.2)


# ----------------------------------------------------------------------------------------------
# Fault injection: the failures that make verification worth its cost
# ----------------------------------------------------------------------------------------------
FAULTS = ("none", "phantom_grasp", "blind_gripper", "short_nav")


class FaultySkills:
    """A ``RobotSkills`` look-alike that lies in one specific, realistic way.

    * ``phantom_grasp`` — ``pick`` reports SUCCEEDED without the object ever leaving the table
      (a gripper with no force sensor, closing on air). ``get_robot_state`` still tells the truth.
    * ``blind_gripper`` — ``phantom_grasp`` **and** ``get_robot_state`` reports the object as held
      (the gripper's own limit switch is the thing that broke). Only the camera can catch this.
    * ``short_nav`` — ``navigate_to`` reports SUCCEEDED from wherever it happened to stop
      (a costmap that declared the goal reached one metre early).

    Every other call passes straight through, and the log stays the real gateway's log.
    """

    def __init__(self, skills: RobotSkills, fault: str = "none", only_object: str | None = None) -> None:
        if fault not in FAULTS:
            raise ValueError(f"unknown fault {fault!r}; one of {FAULTS}")
        self.skills, self.fault, self.only_object = skills, fault, only_object
        self.lies = 0

    def __getattr__(self, name: str) -> Any:  # world, specs, allowlist, log, start, record, ...
        return getattr(self.skills, name)

    def call(self, name: str, args: Mapping[str, Any] | None = None, **kw: Any) -> SkillResult:
        args = dict(args or {})
        if self.fault in ("phantom_grasp", "blind_gripper") and name == "pick":
            target = str(args.get("object_id"))
            if self.only_object in (None, target) and self.skills.world.holding is None:
                self.lies += 1
                return SkillResult("pick", True, "SUCCEEDED", f"holding '{target}'",
                                   {"picked_object_id": target}, 4.5)
        if self.fault == "short_nav" and name == "navigate_to":
            self.lies += 1
            return SkillResult("navigate_to", True, "SUCCEEDED", f"arrived at '{args.get('place')}'", {}, 1.0)
        result = self.skills.call(name, args, **kw)
        if self.fault == "blind_gripper" and name == "get_robot_state" and self.lies:
            data = dict(result.data)
            data["holding"] = self.only_object or "bottle-1"
            return replace(result, data=data)
        return result


# ----------------------------------------------------------------------------------------------
# Replanning
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Situation:
    """What the executor can tell a replanner about why it stopped."""

    plan: Plan
    index: int  # 1-based index of the step that could not be repaired
    step: Step
    verification: Verification
    goal: Goal
    visited: tuple[str, ...]
    robot_state: JsonDict


def _first_match(result: SkillResult, label: str | None) -> JsonDict | None:
    """The first detection in a result whose label matches — what a bind step needs."""
    if label is None:
        return None
    return next((d for d in result.data.get("detections", []) if _label_matches(d["label"], label)), None)


class Replanner(Protocol):
    name: str

    def replan(self, s: Situation) -> Plan | None: ...


class SearchReplanner:
    """Deterministic, offline: when the object is not where the plan assumed, look somewhere else.

    This is the cheapest useful replanner and it covers the commonest real failure. It produces a
    fresh plan — not a patch — so the plan validator of 19.06 can check it like any other.
    """

    name = "search"

    def __init__(self, rooms: Sequence[str] = ("kitchen", "living_room", "study", "bedroom")) -> None:
        self.rooms = tuple(rooms)

    def replan(self, s: Situation) -> Plan | None:
        if s.step.skill not in ("detect_objects", "pick"):
            return None
        room = next((r for r in self.rooms if r not in s.visited), None)
        if room is None:
            return None
        return Plan(f"the {s.goal.label} is on the {s.goal.surface}", (
            Step("navigate_to", {"place": room}),
            Step("detect_objects", {}, "target", s.goal.label),
            Step("navigate_to", {"place": "$target"}),
            Step("detect_objects", {}, "target", s.goal.label),
            Step("pick", {"object_id": "$target"}),
            Step("navigate_to", {"place": s.goal.surface}),
            Step("place", {"location": s.goal.surface}),
        ))


class LLMReplanner:
    """Ask the planning model again, with a situation report appended to the original task.

    Costs one model call (~$0.01 at the 19.06 numbers) and takes seconds, which is why it is the
    *last* rung of the ladder rather than the first.
    """

    name = "llm"

    def __init__(self, llm: Any, skills: RobotSkills, task: str, max_attempts: int = 2) -> None:
        self.llm, self.skills, self.task, self.max_attempts = llm, skills, task, max_attempts
        self.calls = 0

    def replan(self, s: Situation) -> Plan | None:
        from robot_agent.planning import plan_and_repair

        self.calls += 1
        report = (f"{self.task}\n\nThe previous plan failed at step {s.index} "
                  f"({s.step.skill}): {s.verification.message}. The robot is at "
                  f"{s.robot_state.get('at_place')} holding {s.robot_state.get('holding')}. "
                  f"Already searched: {', '.join(s.visited) or 'nothing'}. Submit a new plan.")
        plan, _ = plan_and_repair(self.llm, self.skills, report, s.goal, max_attempts=self.max_attempts)
        return plan


# ----------------------------------------------------------------------------------------------
# The closed-loop executor
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class LoopConfig:
    retries: int = 2  # same step, unchanged, for a retryable failure
    looks: int = 3  # detect attempts before a bind step is declared failed
    max_replans: int = 2
    deadline_s: float = 420.0
    verify: VerifyConfig = VerifyConfig()
    check_goal: bool = True


@dataclass
class StepRecord:
    index: int
    attempt: int
    skill: str
    args: JsonDict
    code: str
    verdict: str
    t_start: float
    t_end: float
    repair: str = "-"  # what the ladder did next: "-", "retry", "repair", "replan", "abort"


@dataclass
class ExecutionReport:
    outcome: str  # "succeeded" | "goal_not_verified" | "failed" | "deadline" | "estop" | "no_plan"
    message: str
    steps: list[StepRecord] = field(default_factory=list)
    verifications: list[Verification] = field(default_factory=list)
    replans: int = 0
    goal_check: Verification | None = None
    sim_time_s: float = 0.0
    verify_cost_s: float = 0.0

    @property
    def caught(self) -> int:
        """How many times a verification contradicted a skill that had reported success."""
        return sum(1 for v in self.verifications if v.code == "NOT_VERIFIED" and "the call itself failed" not in v.message)

    def summary(self) -> str:
        return (f"{self.outcome}: {self.message} | steps {len(self.steps)} | "
                f"verifications {len(self.verifications)} (caught {self.caught}) | replans {self.replans} | "
                f"robot {self.sim_time_s:.1f} s of which {self.verify_cost_s:.1f} s verifying")


class ClosedLoopExecutor:
    """Execute a plan step by step, verify every step, repair locally, replan when repair fails."""

    def __init__(self, skills: RobotSkills | FaultySkills, goal: Goal, cfg: LoopConfig = LoopConfig(),
                 replanner: Replanner | None = None,
                 on_event: Callable[[str, dict[str, Any]], None] | None = None) -> None:
        self.skills = skills
        self.goal = goal
        self.cfg = cfg
        self.replanner = replanner
        self.on_event = on_event or (lambda kind, data: None)
        self.bindings: dict[str, JsonDict] = {}
        self.visited: list[str] = []

    # -- plumbing ---------------------------------------------------------------------------------
    @property
    def world(self) -> Any:
        return self.skills.world

    def _resolve(self, args: Mapping[str, Any]) -> JsonDict:
        out: JsonDict = {}
        for name, value in args.items():
            var = _var(value)
            if var is None:
                out[name] = value
            elif name in OBJECT_PARAMS:
                out[name] = self.bindings[var]["object_id"]
            else:
                out[name] = self.bindings[var]["near_place"]
        return out

    def _verify(self, step: Step, args: JsonDict, result: SkillResult,
                report: ExecutionReport) -> Verification:
        """Verify once; if a *perception* check says no, look again before believing it.

        A detector that misses 8 % of what is in front of it produces a false alarm in 8 % of
        verifications. Two looks take that to 0.6 % for 0.2 s. A verifier with no retry is a
        machine for generating unnecessary replans.
        """
        v = verify_step(self.skills, step.skill, args, result, self.cfg.verify, step.label)
        report.verifications.append(v)
        report.verify_cost_s += v.cost_s
        looks = 1
        while not v.ok and v.repeatable and v.method == "perception" and looks < self.cfg.verify.looks:
            looks += 1
            v = verify_step(self.skills, step.skill, args, result, self.cfg.verify, step.label)
            report.verifications.append(v)
            report.verify_cost_s += v.cost_s
        return v

    # -- the loop ---------------------------------------------------------------------------------
    def run(self, plan: Plan) -> ExecutionReport:
        t0 = self.world.t
        report = ExecutionReport("failed", "not started")
        replans = 0
        while True:
            outcome, message, failed = self._run_plan(plan, report, t0)
            if outcome != "replan":
                report.outcome, report.message = outcome, message
                break
            if self.replanner is None or replans >= self.cfg.max_replans or failed is None:
                report.outcome = "failed"
                report.message = (f"step {failed.index if failed else '?'} could not be repaired and "
                                  f"{'there is no replanner' if self.replanner is None else 'the replan budget is used up'}")
                break
            new_plan = self.replanner.replan(failed)
            replans += 1
            report.replans = replans
            self.on_event("replan", {"attempt": replans, "by": self.replanner.name,
                                     "because": str(failed.verification),
                                     "plan": new_plan.to_json() if new_plan else None})
            if new_plan is None:
                report.outcome, report.message = "failed", f"the replanner had nothing left to try: {failed.verification}"
                break
            plan = new_plan
        if report.outcome == "succeeded" and self.cfg.check_goal:
            check = verify_goal(self.skills, self.goal, self.cfg.verify)
            report.verify_cost_s += check.cost_s
            for _ in range(self.cfg.verify.looks - 1):
                if check.ok:
                    break
                check = verify_goal(self.skills, self.goal, self.cfg.verify)
                report.verify_cost_s += check.cost_s
            report.goal_check = check
            self.on_event("goal", {"verification": str(check)})
            if not check.ok:
                report.outcome, report.message = "goal_not_verified", check.message
        self.world.base.stop()
        report.sim_time_s = round(self.world.t - t0, 2)
        return report

    def _run_plan(self, plan: Plan, report: ExecutionReport,
                  t0: float) -> tuple[str, str, Situation | None]:
        """Run one plan to the end. Returns (outcome, message, situation-if-replan-needed)."""
        self.on_event("plan", {"plan": plan.to_json()})
        repairs: dict[int, int] = {}
        i = 0
        while i < len(plan.steps):
            step = plan.steps[i]
            if self.world.estop:
                return "estop", "the emergency stop is active", None
            if self.world.t - t0 > self.cfg.deadline_s:
                return "deadline", f"the {self.cfg.deadline_s:.0f} s task deadline expired", None
            budget = self.cfg.looks if step.bind else self.cfg.retries
            verdict, situation = self._do_step(plan, i, step, report, budget)
            if verdict == "ok":
                i += 1
                continue
            if verdict == "repair_pick" and repairs.get(i, 0) < self.cfg.retries:
                repairs[i] = repairs.get(i, 0) + 1  # bounded: a repair that never works is a replan
                i -= 1  # back to the detect step that bound the variable
                continue
            return "replan", f"step {i + 1} ({step.skill}) could not be repaired", situation

        return "succeeded", "every step executed and verified", None

    def _do_step(self, plan: Plan, i: int, step: Step, report: ExecutionReport,
                 budget: int) -> tuple[str, Situation | None]:
        """One step, up to ``budget`` attempts, then one rung down the ladder."""
        spec = self.skills.specs.get(step.skill)
        idempotent = spec.idempotent if spec is not None else True
        for attempt in range(1, budget + 1):
            args = self._resolve(step.args)
            t_start = self.world.t
            result = self.skills.call(step.skill, args)
            v = self._verify(step, args, result, report)
            record = StepRecord(i + 1, attempt, step.skill, args, result.code, v.code,
                                round(t_start, 2), round(self.world.t, 2))
            report.steps.append(record)
            if step.skill == "navigate_to" and v.ok and isinstance(args.get("place"), str):
                self.visited.append(args["place"])  # only places the robot was *observed* to reach
            if v.ok or v.code in ("UNCHECKABLE", "SKIPPED"):
                hit = v.observed.get("detection") or _first_match(result, step.label)
                if step.bind is not None and hit is not None:
                    self.bindings[step.bind] = hit
                elif step.bind is not None:  # --verify none: nothing looked, so nothing can bind
                    record.repair = "replan"
                    self.on_event("step", {"record": record, "verification": v})
                    return "replan", Situation(plan, i + 1, step, v, self.goal, tuple(self.visited),
                                               self.skills.call("get_robot_state").data)
                self.on_event("step", {"record": record, "verification": v})
                return "ok", None
            # --- the ladder: retry (cheap) -> repair (re-establish) -> replan (expensive) -------
            # A grasp that failed or could not be verified has invalidated the detection that
            # chose the object, so retrying the pick unchanged is known not to work: skip a rung.
            invalidated = step.skill == "pick" and (
                result.code in INVALIDATES_BINDING or v.code == "NOT_VERIFIED")
            can_repair = step.skill == "pick" and i > 0 and plan.steps[i - 1].bind is not None
            # Never repeat a skill the API declares non-idempotent (19.02): `place` has already
            # opened the gripper, and a second call returns NOT_HOLDING_OBJECT, not a second try.
            if attempt < budget and not invalidated and idempotent:
                record.repair = "retry"
                self.on_event("step", {"record": record, "verification": v})
                continue
            record.repair = "repair" if can_repair else "replan"
            self.on_event("step", {"record": record, "verification": v})
            situation = Situation(plan, i + 1, step, v, self.goal, tuple(self.visited),
                                  self.skills.call("get_robot_state").data)
            return ("repair_pick" if can_repair else "replan"), situation
        return "replan", None
