"""19.06 — reference solution. Mirrors ``19-llm-robot-agents/code/robot_agent/planning.py``."""

from __future__ import annotations

import json
import re
import sys
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "19-llm-robot-agents" / "code", _REPO / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robot_agent.skills import RobotSkills, validate_arguments  # noqa: E402

JsonDict = dict[str, Any]

MAX_STEPS = 12
VAR_RE = re.compile(r"^\$([a-z][a-z0-9_]*)$")
PLACE_PARAMS = frozenset({"place", "location"})   # a variable here = "where that object was seen"
OBJECT_PARAMS = frozenset({"object_id"})          # a variable here = "the id the detector gave it"


# ----------------------------------------------------------------------------------------------
# Given: the plan language and the shape check
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Step:
    skill: str
    args: JsonDict = field(default_factory=dict)
    bind: str | None = None   # detect_objects only: the variable this step binds
    label: str | None = None  # detect_objects only: the label the detection must match


@dataclass(frozen=True)
class Plan:
    goal: str
    steps: tuple[Step, ...]


@dataclass(frozen=True)
class Goal:
    label: str    # "water bottle"
    surface: str  # "kitchen_table"


@dataclass(frozen=True)
class PlanError:
    code: str
    message: str
    step: int | None = None  # 1-based index, None for whole-plan errors
    hint: str = ""


@dataclass
class AbstractState:
    """Everything the validator can know about the robot without running anything."""

    at: str | None = None
    holding: str | None = None
    bound: dict[str, str | None] = field(default_factory=dict)  # variable -> place it was seen at
    fresh: set[str] = field(default_factory=set)                # detected since the last navigate_to
    placed: list[tuple[str, str]] = field(default_factory=list)  # (variable, surface), in order


def is_var(value: Any) -> str | None:
    """``"$bottle"`` -> ``"bottle"``; anything else -> ``None``."""
    m = VAR_RE.fullmatch(value) if isinstance(value, str) else None
    return m.group(1) if m else None


def label_matches(label: str, wanted: str) -> bool:
    label, wanted = label.lower(), wanted.lower()
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


def parse_plan(raw: Any) -> tuple[Plan | None, list[PlanError]]:
    """GIVEN. Shape check only: dict or JSON string -> Plan, or MALFORMED errors."""
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except json.JSONDecodeError as exc:
            return None, [PlanError("MALFORMED", f"not valid JSON: {exc}")]
    if not isinstance(raw, Mapping):
        return None, [PlanError("MALFORMED", f"a plan must be an object, got {type(raw).__name__}")]
    errors: list[PlanError] = []
    goal = raw.get("goal")
    if not isinstance(goal, str) or not goal.strip():
        errors.append(PlanError("MALFORMED", "'goal' must be a non-empty string"))
        goal = ""
    steps_raw = raw.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        errors.append(PlanError("MALFORMED", "'steps' must be a non-empty list"))
        return None, errors
    if len(steps_raw) > MAX_STEPS:
        errors.append(PlanError("MALFORMED", f"{len(steps_raw)} steps is more than the limit of {MAX_STEPS}"))
        return None, errors
    steps: list[Step] = []
    for i, s in enumerate(steps_raw, start=1):
        if not isinstance(s, Mapping) or not isinstance(s.get("skill"), str):
            errors.append(PlanError("MALFORMED", "a step must be an object with a 'skill' string", i))
            continue
        args = s.get("args", {})
        if not isinstance(args, Mapping):
            errors.append(PlanError("MALFORMED", "'args' must be an object", i))
            continue
        bind, label = s.get("bind"), s.get("label")
        if bind is not None and (not isinstance(bind, str) or VAR_RE.fullmatch("$" + bind) is None):
            errors.append(PlanError("MALFORMED", f"'bind' must be a lower-case name, got {bind!r}", i))
            continue
        if bind is not None and not isinstance(label, str):
            errors.append(PlanError("MALFORMED", "a step that binds a variable must also give the 'label'", i))
            continue
        steps.append(Step(s["skill"], dict(args), bind, label if isinstance(label, str) else None))
    if errors:
        return None, errors
    return Plan(goal, tuple(steps)), []


# ----------------------------------------------------------------------------------------------
# Yours
# ----------------------------------------------------------------------------------------------
def substitute(args, spec):
    """Replace every "$var" with a value its own parameter would accept."""
    by_name = {p.name: p for p in spec.params}
    out: JsonDict = {}
    for name, value in args.items():
        p = by_name.get(name)
        if is_var(value) is None or p is None:
            out[name] = value
        else:
            out[name] = p.enum[0] if p.enum else "placeholder-0"
    return out


def apply_step(step: Step, index: int, state: AbstractState) -> list[PlanError]:
    errors: list[PlanError] = []
    if step.skill == "navigate_to":
        place = step.args.get("place")
        var = is_var(place)
        state.at = f"@{var}" if var else place if isinstance(place, str) else None
        state.fresh.clear()
    elif step.skill == "detect_objects":
        if step.bind is not None:
            state.bound[step.bind] = state.at
            state.fresh.add(step.bind)
    elif step.skill == "pick":
        var = is_var(step.args.get("object_id"))
        if state.holding is not None:
            errors.append(PlanError("PRECONDITION",
                                    f"the gripper still holds ${state.holding}: place it first", index))
        if var is not None and var in state.bound and var not in state.fresh:
            errors.append(PlanError("PRECONDITION",
                                    f"${var} was last seen from somewhere else: detect_objects again "
                                    "after arriving, so the object is in reach", index))
        if var is not None and not errors:
            state.holding = var
    elif step.skill == "place":
        location = step.args.get("location")
        var = is_var(location)
        target = f"@{var}" if var else location if isinstance(location, str) else None
        if state.holding is None:
            errors.append(PlanError("PRECONDITION", "place needs the gripper to hold something", index))
        elif state.at != target:
            errors.append(PlanError("PRECONDITION",
                                    f"the robot is at {state.at!r}, not at {target!r}: navigate_to the "
                                    "surface before placing", index))
        else:
            state.placed.append((state.holding, str(target)))
            state.holding = None
    return errors


def check_goal(plan: Plan, goal: Goal, state: AbstractState) -> list[PlanError]:
    labels = {s.bind: (s.label or "") for s in plan.steps if s.bind}
    for var, surface in state.placed:
        if surface == goal.surface and label_matches(labels.get(var, ""), goal.label):
            return []
    return [PlanError("GOAL_NOT_REACHED", f"no step places a '{goal.label}' on '{goal.surface}'")]


def validate_plan(raw, skills: RobotSkills, goal: Goal | None = None):
    plan, errors = parse_plan(raw)
    if plan is None:
        return None, errors
    state = AbstractState()
    for i, step in enumerate(plan.steps, start=1):
        spec = skills.specs.get(step.skill)
        if spec is None or step.skill not in skills.allowlist:
            errors.append(PlanError("UNKNOWN_SKILL",
                                    f"'{step.skill}' is not an available skill "
                                    f"(have: {', '.join(sorted(skills.allowlist))})", i))
            continue
        for message in validate_arguments(spec.input_schema(), substitute(step.args, spec)):
            errors.append(PlanError("INVALID_ARGUMENTS", message, i))
        for name, value in step.args.items():
            var = is_var(value)
            if var is None:
                if name in OBJECT_PARAMS:
                    errors.append(PlanError("LITERAL_OBJECT_ID",
                                            f"{name}={value!r} was written before the robot looked: "
                                            "object ids come from detect_objects", i))
                continue
            if var not in state.bound:
                errors.append(PlanError("UNBOUND_VARIABLE", f"${var} is used before any step binds it", i))
            elif name not in PLACE_PARAMS and name not in OBJECT_PARAMS:
                errors.append(PlanError("INVALID_ARGUMENTS", f"a variable is not allowed for '{name}'", i))
        errors += apply_step(step, i, state)
    if goal is not None and not errors:
        errors += check_goal(plan, goal, state)
    return (plan if not errors else None), errors
