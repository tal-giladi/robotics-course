"""19.06 — the plan validator: accept a whole plan, or say exactly why not, before anything moves.

The plan language, the shape check (``parse_plan``) and the data types are given. You write the
part that matters: argument checking with variables, symbolic execution of the preconditions, the
goal check, and the ``validate_plan`` that runs them in order.

Nothing in here may touch the robot. A rejected plan must cost zero seconds of robot time.

Fill in every ``TODO(student)``; check with ``python course.py check 19.06``.
"""

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
def substitute(args: Mapping[str, Any], spec: Any) -> JsonDict:
    """Replace every ``"$var"`` with a value that parameter's own schema would accept.

    A variable stands for something only a running robot can know, so it cannot be checked against
    the schema — but the parameter must still be *present* and every other argument must still be
    checked. Return a copy of ``args`` where, for each key:

    * the value is not a variable, or the key is not one of ``spec.params``  -> keep it unchanged
      (an unknown key must survive so the schema can report it as an unknown parameter);
    * otherwise -> ``p.enum[0]`` if that parameter declares an enum, else ``"placeholder-0"``
      (which satisfies the ``object_id`` pattern ``[a-z]+-\\d+``).

    ``spec`` is a ``SkillSpec``; ``spec.params`` is a tuple of ``Param`` with ``.name`` and ``.enum``.
    """
    # TODO(student): implement.
    raise NotImplementedError("substitute")


def apply_step(step: Step, index: int, state: AbstractState) -> list[PlanError]:
    """Check this step's preconditions against ``state``, apply its postconditions, return errors.

    ``index`` is the 1-based step number, for the error. Per skill:

    ``navigate_to(place)``
        ``state.at`` becomes ``f"@{var}"`` when ``place`` is a variable (an opaque token meaning
        "the place that object was seen at"), else the literal place. Then **clear
        ``state.fresh``** — driving away makes every earlier detection stale.

    ``detect_objects``
        if the step binds a variable: ``state.bound[step.bind] = state.at`` and add it to
        ``state.fresh``.

    ``pick(object_id)``
        * ``state.holding is not None`` -> ``PRECONDITION``,
          ``f"the gripper still holds ${state.holding}: place it first"``
        * the variable is bound but **not** in ``state.fresh`` -> ``PRECONDITION``,
          ``f"${var} was last seen from somewhere else: detect_objects again after arriving, "
          "so the object is in reach"``
        * no errors -> ``state.holding = var``

    ``place(location)``
        * ``state.holding is None`` -> ``PRECONDITION``, ``"place needs the gripper to hold something"``
        * else if ``state.at`` differs from the target (``f"@{var}"`` or the literal) ->
          ``PRECONDITION``, ``f"the robot is at {state.at!r}, not at {target!r}: navigate_to the "
          "surface before placing"``
        * else -> append ``(state.holding, str(target))`` to ``state.placed`` and empty the gripper.

    Read-only skills (``list_places``, ``get_robot_state``) change nothing and are always legal.
    """
    # TODO(student): implement.
    raise NotImplementedError("apply_step")


def check_goal(plan: Plan, goal: Goal, state: AbstractState) -> list[PlanError]:
    """Did the plan put a ``goal.label`` object on ``goal.surface``?

    Look through ``state.placed``. A ``(variable, surface)`` entry counts when ``surface`` equals
    ``goal.surface`` and the label of the step that bound that variable matches ``goal.label``
    (use ``label_matches``). No match -> one ``PlanError("GOAL_NOT_REACHED",
    f"no step places a '{goal.label}' on '{goal.surface}'")`` with no step number.
    """
    # TODO(student): implement.
    raise NotImplementedError("check_goal")


def validate_plan(raw: Any, skills: RobotSkills, goal: Goal | None = None) -> tuple[Plan | None, list[PlanError]]:
    """Accept a plan, or return every reason it cannot be executed. Never touches the robot.

    1. ``parse_plan``. If it returned no plan, return ``(None, errors)``.
    2. With a fresh ``AbstractState``, walk the steps (1-based index). For each:

       a. ``spec = skills.specs.get(step.skill)``. If it is missing, or the name is not in
          ``skills.allowlist``: ``PlanError("UNKNOWN_SKILL", f"'{step.skill}' is not an available "
          f"skill (have: {', '.join(sorted(skills.allowlist))})", i)`` and **continue** — there is
          no schema and no transition for a skill that does not exist.
       b. arguments: one ``PlanError("INVALID_ARGUMENTS", message, i)`` per message from
          ``validate_arguments(spec.input_schema(), substitute(step.args, spec))``.
       c. variables, for each argument:
          * not a variable and the key is in ``OBJECT_PARAMS`` -> ``PlanError("LITERAL_OBJECT_ID",
            f"{name}={value!r} was written before the robot looked: object ids come from "
            "detect_objects", i)``
          * a variable that nothing has bound yet -> ``PlanError("UNBOUND_VARIABLE",
            f"${var} is used before any step binds it", i)``
          * a variable on a parameter that is in neither ``PLACE_PARAMS`` nor ``OBJECT_PARAMS``
            -> ``PlanError("INVALID_ARGUMENTS", f"a variable is not allowed for '{name}'", i)``
       d. ``apply_step``.

    3. If ``goal`` is not ``None`` **and there are no errors yet**, add ``check_goal``. (A plan
       that is already broken has not earned a goal verdict.)
    4. Return ``(plan, [])`` when there are no errors, else ``(None, errors)``.
    """
    # TODO(student): implement.
    raise NotImplementedError("validate_plan")
