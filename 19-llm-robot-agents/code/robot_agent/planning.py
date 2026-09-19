"""Plan first, validate, then execute (19.06).

The agent loop of 19.03 lets the model decide one step at a time. This module is the other half of
the pattern: the model writes the **whole plan** in one turn, a deterministic validator either
accepts it or hands back machine-readable errors, and only an accepted plan is compiled into the
behavior tree of 19.05 and executed.

Three pieces:

* ``Plan`` / ``Step`` — the plan language. It is deliberately smaller than Python: a flat list of
  skill calls plus **variables**, because the one thing a planner cannot know in advance is the
  ``object_id`` the detector will hand out. ``detect_objects`` binds a variable, later steps refer
  to it as ``"$bottle"``.
* ``validate_plan`` — schema, allowlist, argument validation and a **symbolic execution** of the
  plan against an abstract world state (where the robot is, what it holds, which variables are
  bound and still fresh). Every rejection is a ``PlanError`` with a code and a hint the planner
  can act on.
* ``plan_to_tree`` — compiles an accepted plan into the same ``SkillAction`` leaves the hand-written
  tree in :mod:`robot_agent.bt` uses, under the same e-stop guard and deadline.

``plan_and_repair`` closes the loop: plan → validate → return the errors to the model → replan, at
most ``max_attempts`` times. Nothing here needs an API key: :class:`MockPlanner` produces the same
plans offline, including the broken ones the exercises ask you to catch.
"""

from __future__ import annotations

import itertools
import json
import re
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from robot_agent.llm import AssistantMessage, LLMBackend, Message, ToolResult, ToolResultsMessage, UserMessage
from robot_agent.skills import JsonDict, RobotSkills, SkillResult, validate_arguments

MAX_STEPS = 12
VAR_RE = re.compile(r"^\$([a-z][a-z0-9_]*)$")
PLACE_PARAMS = frozenset({"place", "location"})  # a variable here means "where that object was seen"
OBJECT_PARAMS = frozenset({"object_id"})  # a variable here means "the id the detector gave it"


# ----------------------------------------------------------------------------------------------
# The plan language
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Step:
    """One skill call in a plan. ``bind``/``label`` are only meaningful on ``detect_objects``."""

    skill: str
    args: JsonDict = field(default_factory=dict)
    bind: str | None = None  # variable name, without the '$'
    label: str | None = None  # the object label the detection must match

    def to_json(self) -> JsonDict:
        out: JsonDict = {"skill": self.skill, "args": dict(self.args)}
        if self.bind is not None:
            out["bind"] = self.bind
        if self.label is not None:
            out["label"] = self.label
        return out


@dataclass(frozen=True)
class Plan:
    goal: str
    steps: tuple[Step, ...]

    def to_json(self) -> JsonDict:
        return {"goal": self.goal, "steps": [s.to_json() for s in self.steps]}

    def pretty(self) -> str:
        lines = [f"goal: {self.goal}"]
        for i, s in enumerate(self.steps, start=1):
            extra = f"  -> ${s.bind} = first '{s.label}'" if s.bind else ""
            lines.append(f"  {i:>2}. {s.skill}({json.dumps(s.args, separators=(',', ':'))}){extra}")
        return "\n".join(lines)


@dataclass(frozen=True)
class Goal:
    """What the plan has to achieve, in the validator's terms."""

    label: str  # "water bottle"
    surface: str  # "kitchen_table"


@dataclass(frozen=True)
class PlanError:
    code: str  # MALFORMED | UNKNOWN_SKILL | INVALID_ARGUMENTS | LITERAL_OBJECT_ID |
    # UNBOUND_VARIABLE | PRECONDITION | GOAL_NOT_REACHED
    message: str
    step: int | None = None  # 1-based index of the offending step, None for whole-plan errors
    hint: str = ""

    def __str__(self) -> str:
        where = f"step {self.step}: " if self.step else ""
        return f"[{self.code}] {where}{self.message}" + (f" ({self.hint})" if self.hint else "")


PLAN_HINTS = {
    "MALFORMED": "Return one JSON object with 'goal' (string) and 'steps' (list of at most "
                 f"{MAX_STEPS} steps, each with 'skill' and 'args').",
    "UNKNOWN_SKILL": "Use only the skills listed in the tool definitions. Do not invent skills.",
    "INVALID_ARGUMENTS": "Fix the arguments to match the skill's schema. Do not guess units or place names.",
    "LITERAL_OBJECT_ID": "You cannot know an object_id before the robot looks. Add a detect_objects step "
                         "with 'bind' and 'label', then pass \"$<name>\".",
    "UNBOUND_VARIABLE": "Bind the variable with an earlier detect_objects step ('bind': '<name>').",
    "PRECONDITION": "Reorder the plan so every skill's preconditions hold when it runs.",
    "GOAL_NOT_REACHED": "The last step must place the requested object on the requested surface.",
}


# ----------------------------------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------------------------------
def parse_plan(raw: Any) -> tuple[Plan | None, list[PlanError]]:
    """Shape check only: ``raw`` (a dict or a JSON string) -> ``Plan``, or the structural errors."""
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except json.JSONDecodeError as exc:
            return None, [PlanError("MALFORMED", f"not valid JSON: {exc}", hint=PLAN_HINTS["MALFORMED"])]
    if not isinstance(raw, Mapping):
        return None, [PlanError("MALFORMED", f"a plan must be an object, got {type(raw).__name__}",
                                hint=PLAN_HINTS["MALFORMED"])]
    errors: list[PlanError] = []
    goal = raw.get("goal")
    if not isinstance(goal, str) or not goal.strip():
        errors.append(PlanError("MALFORMED", "'goal' must be a non-empty string", hint=PLAN_HINTS["MALFORMED"]))
        goal = ""
    steps_raw = raw.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        errors.append(PlanError("MALFORMED", "'steps' must be a non-empty list", hint=PLAN_HINTS["MALFORMED"]))
        return None, errors
    if len(steps_raw) > MAX_STEPS:
        errors.append(PlanError("MALFORMED", f"{len(steps_raw)} steps is more than the limit of {MAX_STEPS}",
                                hint=PLAN_HINTS["MALFORMED"]))
        return None, errors
    steps: list[Step] = []
    for i, s in enumerate(steps_raw, start=1):
        if not isinstance(s, Mapping) or not isinstance(s.get("skill"), str):
            errors.append(PlanError("MALFORMED", "a step must be an object with a 'skill' string", i,
                                    PLAN_HINTS["MALFORMED"]))
            continue
        args = s.get("args", {})
        if not isinstance(args, Mapping):
            errors.append(PlanError("MALFORMED", "'args' must be an object", i, PLAN_HINTS["MALFORMED"]))
            continue
        bind, label = s.get("bind"), s.get("label")
        if bind is not None and (not isinstance(bind, str) or VAR_RE.fullmatch("$" + bind) is None):
            errors.append(PlanError("MALFORMED", f"'bind' must be a lower-case name, got {bind!r}", i,
                                    PLAN_HINTS["MALFORMED"]))
            continue
        if bind is not None and not isinstance(label, str):
            errors.append(PlanError("MALFORMED", "a step that binds a variable must also give the 'label' "
                                                 "the detection has to match", i, PLAN_HINTS["MALFORMED"]))
            continue
        steps.append(Step(s["skill"], dict(args), bind, label if isinstance(label, str) else None))
    if errors:
        return None, errors
    return Plan(goal, tuple(steps)), []


# ----------------------------------------------------------------------------------------------
# Symbolic execution
# ----------------------------------------------------------------------------------------------
@dataclass
class AbstractState:
    """What the validator can know about the robot without running anything.

    ``fresh`` holds the variables detected at the place the robot is standing at *now*: driving
    away invalidates them, which is exactly the ``pick`` precondition "object detected recently,
    within 0.75 m".
    """

    at: str | None = None
    holding: str | None = None
    bound: dict[str, str | None] = field(default_factory=dict)  # variable -> place it was seen at
    fresh: set[str] = field(default_factory=set)
    placed: list[tuple[str, str]] = field(default_factory=list)  # (variable, surface) in order


def _var(value: Any) -> str | None:
    """'$bottle' -> 'bottle'; anything else -> None."""
    m = VAR_RE.fullmatch(value) if isinstance(value, str) else None
    return m.group(1) if m else None


def validate_plan(raw: Any, skills: RobotSkills, goal: Goal | None = None) -> tuple[Plan | None, list[PlanError]]:
    """Accept a plan, or return every reason it cannot be executed. Never touches the robot.

    Checks in order: shape, allowlist, arguments (against the same JSON Schema the LLM was given),
    variables, preconditions by symbolic execution, and — when ``goal`` is given — whether the plan
    achieves the goal at all.
    """
    plan, errors = parse_plan(raw)
    if plan is None:
        return None, errors
    state = AbstractState()
    for i, step in enumerate(plan.steps, start=1):
        spec = skills.specs.get(step.skill)
        if spec is None or step.skill not in skills.allowlist:
            errors.append(PlanError("UNKNOWN_SKILL", f"'{step.skill}' is not an available skill "
                                                     f"(have: {', '.join(sorted(skills.allowlist))})",
                                    i, PLAN_HINTS["UNKNOWN_SKILL"]))
            continue
        # Arguments: variables stand in for values only a running robot can know, so each variable
        # is replaced by a value its parameter would accept and checked separately below.
        for message in validate_arguments(spec.input_schema(), _substitute(step.args, spec)):
            errors.append(PlanError("INVALID_ARGUMENTS", message, i, PLAN_HINTS["INVALID_ARGUMENTS"]))
        errors += _check_variables(step, i, state)
        errors += _apply(step, i, state)
    if goal is not None and not errors:
        errors += _check_goal(plan, goal, state)
    return (plan if not errors else None), errors


def _substitute(args: Mapping[str, Any], spec: Any) -> JsonDict:
    """Replace every ``"$var"`` with a value its own parameter would accept.

    Unknown keys are left alone so the schema still reports them as unknown parameters.
    """
    by_name = {p.name: p for p in spec.params}
    out: JsonDict = {}
    for name, value in args.items():
        p = by_name.get(name)
        if _var(value) is None or p is None:
            out[name] = value
        else:
            out[name] = p.enum[0] if p.enum else "placeholder-0"
    return out


def _check_variables(step: Step, i: int, state: AbstractState) -> list[PlanError]:
    errors = []
    for name, value in step.args.items():
        var = _var(value)
        if var is None:
            if name in OBJECT_PARAMS:
                errors.append(PlanError("LITERAL_OBJECT_ID", f"{name}={value!r} was written before the robot "
                                                             "looked: object ids come from detect_objects",
                                        i, PLAN_HINTS["LITERAL_OBJECT_ID"]))
            continue
        if var not in state.bound:
            errors.append(PlanError("UNBOUND_VARIABLE", f"${var} is used before any step binds it", i,
                                    PLAN_HINTS["UNBOUND_VARIABLE"]))
        elif name not in PLACE_PARAMS and name not in OBJECT_PARAMS:
            errors.append(PlanError("INVALID_ARGUMENTS", f"a variable is not allowed for '{name}'", i,
                                    PLAN_HINTS["INVALID_ARGUMENTS"]))
    return errors


def _apply(step: Step, i: int, state: AbstractState) -> list[PlanError]:
    """Check this step's preconditions against ``state`` and apply its postconditions."""
    errors: list[PlanError] = []
    if step.skill == "navigate_to":
        place = step.args.get("place")
        var = _var(place)
        # "$target" means "the place $target was seen at": an opaque token, not a map name.
        state.at = f"@{var}" if var else place if isinstance(place, str) else None
        state.fresh.clear()  # driving away makes every earlier detection stale
    elif step.skill == "detect_objects":
        if step.bind is not None:
            state.bound[step.bind] = state.at
            state.fresh.add(step.bind)
    elif step.skill == "pick":
        var = _var(step.args.get("object_id"))
        if state.holding is not None:
            errors.append(PlanError("PRECONDITION", f"the gripper still holds ${state.holding}: place it first",
                                    i, PLAN_HINTS["PRECONDITION"]))
        if var is not None and var in state.bound and var not in state.fresh:
            errors.append(PlanError("PRECONDITION", f"${var} was last seen from somewhere else: detect_objects "
                                                    "again after arriving, so the object is in reach",
                                    i, PLAN_HINTS["PRECONDITION"]))
        if var is not None and not errors:
            state.holding = var
    elif step.skill == "place":
        location = step.args.get("location")
        var = _var(location)
        target = f"@{var}" if var else location if isinstance(location, str) else None
        if state.holding is None:
            errors.append(PlanError("PRECONDITION", "place needs the gripper to hold something", i,
                                    PLAN_HINTS["PRECONDITION"]))
        elif state.at != target:
            errors.append(PlanError("PRECONDITION", f"the robot is at {state.at!r}, not at {target!r}: "
                                                    "navigate_to the surface before placing", i,
                                    PLAN_HINTS["PRECONDITION"]))
        else:
            state.placed.append((state.holding, str(target)))
            state.holding = None
    return errors


def _check_goal(plan: Plan, goal: Goal, state: AbstractState) -> list[PlanError]:
    labels = {s.bind: (s.label or "") for s in plan.steps if s.bind}
    for var, surface in state.placed:
        if surface == goal.surface and _label_matches(labels.get(var, ""), goal.label):
            return []
    return [PlanError("GOAL_NOT_REACHED", f"no step places a '{goal.label}' on '{goal.surface}'",
                      hint=PLAN_HINTS["GOAL_NOT_REACHED"])]


def _label_matches(label: str, wanted: str) -> bool:
    label, wanted = label.lower(), wanted.lower()
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


# ----------------------------------------------------------------------------------------------
# Compiling an accepted plan into a behavior tree (19.05)
# ----------------------------------------------------------------------------------------------
_ids = itertools.count()


@dataclass(frozen=True)
class PlanExecConfig:
    retries: int = 2  # per motion / manipulation step
    looks: int = 3  # detect_objects attempts per detect step
    deadline_s: float = 300.0
    tick_period_s: float = 0.1


def plan_to_tree(plan: Plan, skills: RobotSkills, cfg: PlanExecConfig = PlanExecConfig()) -> tuple[Any, dict[str, JsonDict]]:
    """Accepted plan -> (behavior tree root, the dict the detections get bound into).

    The leaves are :class:`robot_agent.bt.SkillAction`, so cancellation, logging and the gateway's
    checks are exactly the ones the hand-written tree uses. Import is local: only this function
    needs py_trees, so validation stays a pure-stdlib dependency.
    """
    import py_trees

    from robot_agent.bt import Check, SimTimeout, SkillAction

    world = skills.world
    ns = f"/plan{next(_ids)}"
    bindings: dict[str, JsonDict] = {}

    def resolve(args: JsonDict) -> JsonDict:
        out: JsonDict = {}
        for name, value in args.items():
            var = _var(value)
            if var is None:
                out[name] = value
            elif name in OBJECT_PARAMS:
                out[name] = bindings[var]["object_id"]
            else:
                out[name] = bindings[var]["near_place"]
        return out

    def binder(step: Step) -> Callable[[Any, SkillResult], Any]:
        def on_success(_bb: Any, result: SkillResult) -> Any:
            hits = [d for d in result.data.get("detections", []) if _label_matches(d["label"], step.label or "")]
            if not hits:
                return py_trees.common.Status.FAILURE  # Retry looks again; detection misses happen
            bindings[step.bind or ""] = hits[0]
            return py_trees.common.Status.SUCCESS
        return on_success

    def leaf_for(i: int, step: Step) -> Any:
        return SkillAction(f"{i}. {step.skill}", skills, step.skill,
                           lambda _bb, s=step: resolve(s.args), ns, cfg.tick_period_s,
                           binder(step) if step.bind else None)

    # The compiler adds the recovery the plan language cannot express: a step that consumes the
    # variable the previous step bound is *fused* with it, so a retry re-runs the detection too.
    # Without that, retrying a failed grasp re-uses a detection the failure has already invalidated.
    children, i = [], 0
    while i < len(plan.steps):
        step = plan.steps[i]
        nxt = plan.steps[i + 1] if i + 1 < len(plan.steps) else None
        fuse = step.bind is not None and nxt is not None and f"${step.bind}" in nxt.args.values()
        if fuse and nxt is not None:
            pair = py_trees.composites.Sequence(f"{i + 1}-{i + 2}. bind+{nxt.skill}", memory=True,
                                                children=[leaf_for(i + 1, step), leaf_for(i + 2, nxt)])
            children.append(py_trees.decorators.Retry(f"{i + 1}-{i + 2} x{cfg.looks}", pair, num_failures=cfg.looks))
            i += 2
            continue
        repeats = cfg.looks if step.bind else cfg.retries
        leaf = leaf_for(i + 1, step)
        children.append(py_trees.decorators.Retry(f"{i + 1}. {step.skill} x{repeats}", leaf, num_failures=repeats)
                        if repeats > 1 else leaf)
        i += 1
    body = py_trees.composites.Sequence("Plan", memory=True, children=children)
    root = py_trees.composites.Sequence("PlanTask", memory=False, children=[
        Check("EStopReleased?", lambda: not world.estop),
        SimTimeout("PlanDeadline", body, cfg.deadline_s, clock=lambda: world.t),
    ])
    return root, bindings


# ----------------------------------------------------------------------------------------------
# Getting a plan out of a model (or out of a mock)
# ----------------------------------------------------------------------------------------------
PLANNER_SYSTEM_PROMPT = """\
You plan tasks for karmel, a small home robot. You do not control the robot: you submit one plan
and a deterministic validator either accepts it or returns errors.

Call submit_plan exactly once with a complete plan.

Rules:
- Every step is one of the skills whose tool definitions you were given, with arguments that match
  its schema. Use exact place names.
- You cannot know an object_id in advance. A detect_objects step may carry "bind": "<name>" and
  "label": "<what to look for>"; later steps then refer to "$<name>". A variable in object_id means
  the detected object; a variable in place or location means the place it was seen at.
- pick needs: gripper empty, and a detect_objects (binding that variable) after the last
  navigate_to, so the object is in reach. place needs: holding something, and standing at that
  surface.
- Plans are at most {max_steps} steps. If the request is ambiguous, submit no plan and say so.
"""


def plan_tool_definition(skills: RobotSkills) -> JsonDict:
    """The ``submit_plan`` tool: the planner's only output channel, schema-checked like any skill."""
    return {
        "name": "submit_plan",
        "description": "Submit the complete plan for the requested task. Call this exactly once.",
        "input_schema": {
            "type": "object",
            "properties": {
                "goal": {"type": "string", "description": "One sentence: the state of the world when the plan is done."},
                "steps": {
                    "type": "array", "minItems": 1, "maxItems": MAX_STEPS,
                    "items": {
                        "type": "object",
                        "properties": {
                            "skill": {"type": "string", "enum": sorted(skills.allowlist)},
                            "args": {"type": "object", "description": "Arguments for that skill's schema."},
                            "bind": {"type": "string", "description": "detect_objects only: name a variable for the "
                                                                      "object found, referred to later as \"$name\"."},
                            "label": {"type": "string", "description": "detect_objects only: the label to match."},
                        },
                        "required": ["skill", "args"],
                    },
                },
            },
            "required": ["goal", "steps"],
            "additionalProperties": False,
        },
    }


@dataclass
class PlanAttempt:
    plan: Plan | None
    errors: list[PlanError]
    text: str = ""
    raw: JsonDict | None = None


def plan_once(llm: LLMBackend, skills: RobotSkills, task: str, goal: Goal | None = None,
              history: Sequence[Message] = ()) -> tuple[PlanAttempt, list[Message]]:
    """One planning turn: ask for a plan, validate it, return the attempt and the new transcript."""
    messages: list[Message] = list(history) or [UserMessage(task)]
    tools = [plan_tool_definition(skills), *skills.tool_definitions()]
    system = PLANNER_SYSTEM_PROMPT.format(max_steps=MAX_STEPS)
    reply: AssistantMessage = llm.complete(system, messages, tools)
    messages.append(reply)
    calls = [c for c in reply.tool_calls if c.name == "submit_plan"]
    if not calls:
        return PlanAttempt(None, [PlanError("MALFORMED", "the planner did not submit a plan",
                                            hint=PLAN_HINTS["MALFORMED"])], reply.text), messages
    raw = calls[0].arguments
    plan, errors = validate_plan(raw, skills, goal)
    messages.append(ToolResultsMessage((ToolResult(
        calls[0].id,
        json.dumps({"accepted": plan is not None, "errors": [str(e) for e in errors]}, ensure_ascii=False),
        is_error=plan is None), )))
    return PlanAttempt(plan, errors, reply.text, dict(raw)), messages


def plan_and_repair(llm: LLMBackend, skills: RobotSkills, task: str, goal: Goal | None = None,
                    max_attempts: int = 3) -> tuple[Plan | None, list[PlanAttempt]]:
    """Plan, validate, feed the errors back, replan. The robot never moves while this runs."""
    attempts: list[PlanAttempt] = []
    messages: list[Message] = [UserMessage(task)]
    for _ in range(max_attempts):
        attempt, messages = plan_once(llm, skills, task, goal, messages)
        attempts.append(attempt)
        if attempt.plan is not None:
            return attempt.plan, attempts
    return None, attempts


class MockPlanner:
    """Offline stand-in for a planning model: emits a named plan, and repairs it when told why.

    ``variant`` picks the first plan it submits; after a rejection it submits ``good`` (which is
    what a competent model does with an error list). Use it to run the whole lab with no API key.
    """

    name = "mock-planner"

    def __init__(self, variant: str = "good", repair: bool = True,
                 obj: str = "water bottle", where: str = "kitchen", target: str = "kitchen_table") -> None:
        self.variant, self.repair, self.obj, self.where, self.target = variant, repair, obj, where, target
        self.calls = 0

    def plans(self) -> dict[str, JsonDict]:
        goal = f"the {self.obj} is on the {self.target}"
        good = {"goal": goal, "steps": [
            {"skill": "navigate_to", "args": {"place": self.where}},
            {"skill": "detect_objects", "args": {}, "bind": "target", "label": self.obj},
            {"skill": "navigate_to", "args": {"place": "$target"}},
            {"skill": "detect_objects", "args": {}, "bind": "target", "label": self.obj},
            {"skill": "pick", "args": {"object_id": "$target"}},
            {"skill": "navigate_to", "args": {"place": self.target}},
            {"skill": "place", "args": {"location": self.target}},
        ]}
        return {
            "good": good,
            # A plausible-looking plan that invents the object id the detector has not issued yet.
            "literal_id": {"goal": goal, "steps": [
                {"skill": "navigate_to", "args": {"place": self.where}},
                {"skill": "pick", "args": {"object_id": "bottle-1"}},
                {"skill": "navigate_to", "args": {"place": self.target}},
                {"skill": "place", "args": {"location": self.target}},
            ]},
            # Skills that do not exist, and a place that is not on the map.
            "hallucinated": {"goal": goal, "steps": [
                {"skill": "open_fridge", "args": {}},
                {"skill": "navigate_to", "args": {"place": "garage"}},
                {"skill": "grasp", "args": {"target": self.obj}},
            ]},
            # Right skills, wrong order: it picks from across the room and places before arriving.
            "bad_order": {"goal": goal, "steps": [
                {"skill": "detect_objects", "args": {}, "bind": "target", "label": self.obj},
                {"skill": "pick", "args": {"object_id": "$target"}},
                {"skill": "place", "args": {"location": self.target}},
                {"skill": "navigate_to", "args": {"place": self.target}},
            ]},
            # Perfectly valid, and it never achieves the goal.
            "sightseeing": {"goal": goal, "steps": [
                {"skill": "navigate_to", "args": {"place": self.where}},
                {"skill": "detect_objects", "args": {}, "bind": "target", "label": self.obj},
                {"skill": "get_robot_state", "args": {}},
            ]},
        }

    def complete(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> AssistantMessage:
        from robot_agent.llm import ToolCall

        self.calls += 1
        variant = self.variant if self.calls == 1 or not self.repair else "good"
        plan = self.plans()[variant]
        return AssistantMessage(f"Plan ({variant}), {len(plan['steps'])} steps.",
                                (ToolCall(f"plan_{self.calls}", "submit_plan", plan),), stop_reason="tool_use")
