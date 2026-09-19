"""Explicit (hierarchical) finite state machines for robot tasks (19.04).

A tiny SMACH/YASMIN-style engine in plain Python:

* a ``State`` declares its outcomes and implements ``execute(bb) -> outcome``;
* a ``StateMachine`` maps every (state, outcome) to the next state or to one of its own outcomes;
  ``validate()`` refuses a machine with an unmapped outcome, an unknown target or an unreachable state;
* a ``StateMachine`` is itself a ``State``, so machines nest (hierarchical FSM);
* preemption (e-stop, operator cancel) and a task deadline are checked between states, and the
  running skill is told to stop through the same ``CancelToken`` the agent loop uses.

``build_fetch_fsm`` wires the fetch-and-place task to the same ``RobotSkills`` gateway the LLM uses.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from robot_agent.skills import CancelToken, RobotSkills, SkillResult

Blackboard = dict[str, Any]


class State:
    outcomes: tuple[str, ...] = ()

    def execute(self, bb: Blackboard) -> str:
        raise NotImplementedError


class CallbackState(State):
    def __init__(self, outcomes: tuple[str, ...], fn: Callable[[Blackboard], str]) -> None:
        self.outcomes = outcomes
        self.fn = fn

    def execute(self, bb: Blackboard) -> str:
        return self.fn(bb)


@dataclass
class TraceEntry:
    t: float
    machine: str
    state: str
    outcome: str
    next: str


class InvalidMachine(ValueError):
    pass


@dataclass
class StateMachine(State):
    name: str
    outcomes: tuple[str, ...]
    clock: Callable[[], float] = field(default=lambda: 0.0)
    preempt: Callable[[Blackboard], str | None] = field(default=lambda bb: None)  # returns an outcome to abort with
    max_transitions: int = 200
    states: dict[str, State] = field(default_factory=dict)
    transitions: dict[str, dict[str, str]] = field(default_factory=dict)
    start: str | None = None
    trace: list[TraceEntry] = field(default_factory=list)

    def add_state(self, name: str, state: State, transitions: Mapping[str, str]) -> StateMachine:
        if name in self.states:
            raise InvalidMachine(f"duplicate state {name}")
        self.states[name] = state
        self.transitions[name] = dict(transitions)
        if isinstance(state, StateMachine):
            state.trace = self.trace  # one timeline for the whole hierarchy
        if self.start is None:
            self.start = name
        return self

    def validate(self) -> None:
        if not self.states or self.start not in self.states:
            raise InvalidMachine(f"{self.name}: no start state")
        for name, state in self.states.items():
            mapping = self.transitions[name]
            for outcome in state.outcomes:
                if outcome not in mapping:
                    raise InvalidMachine(f"{self.name}.{name}: outcome '{outcome}' has no transition")
            for outcome, target in mapping.items():
                if outcome not in state.outcomes:
                    raise InvalidMachine(f"{self.name}.{name}: transition for undeclared outcome '{outcome}'")
                if target not in self.states and target not in self.outcomes:
                    raise InvalidMachine(f"{self.name}.{name}: '{outcome}' goes to unknown '{target}'")
            if isinstance(state, StateMachine):
                state.validate()
        reachable, frontier = {self.start}, [self.start]
        while frontier:
            for target in self.transitions[frontier.pop()].values():
                if target in self.states and target not in reachable:
                    reachable.add(target)
                    frontier.append(target)
        if unreachable := set(self.states) - reachable:
            raise InvalidMachine(f"{self.name}: unreachable states {sorted(unreachable)}")

    def execute(self, bb: Blackboard) -> str:
        self.validate()
        current = self.start
        assert current is not None
        for _ in range(self.max_transitions):
            if (forced := self.preempt(bb)) is not None:
                if forced not in self.outcomes:
                    raise InvalidMachine(f"{self.name}: preemption outcome '{forced}' is not an outcome of this machine")
                self.trace.append(TraceEntry(self.clock(), self.name, current, forced, forced))
                return forced
            outcome = self.states[current].execute(bb)
            if outcome not in self.states[current].outcomes:
                raise InvalidMachine(f"{self.name}.{current} returned undeclared outcome '{outcome}'")
            nxt = self.transitions[current][outcome]
            self.trace.append(TraceEntry(round(self.clock(), 2), self.name, current, outcome, nxt))
            if nxt in self.outcomes:
                return nxt
            current = nxt
        raise RuntimeError(f"{self.name}: more than {self.max_transitions} transitions (a loop without a counter?)")

    def to_mermaid(self) -> str:
        lines = ["stateDiagram-v2", f"    [*] --> {self.start}"]
        for name, mapping in self.transitions.items():
            for outcome, target in mapping.items():
                lines.append(f"    {name} --> {'[*]' if target in self.outcomes else target}: {outcome}"
                             + (f" ({target})" if target in self.outcomes else ""))
        return "\n".join(lines)


# ----------------------------------------------------------------------------------------------
# Fetch-and-place as a hierarchical state machine over the skill API
# ----------------------------------------------------------------------------------------------
@dataclass
class FetchConfig:
    object_label: str
    target: str
    rooms: tuple[str, ...] = ("living_room", "kitchen", "study", "bedroom")
    looks_per_room: int = 3
    max_nav_attempts: int = 2
    max_grasp_attempts: int = 3
    deadline_s: float = 300.0


def _matches(label: str, wanted: str) -> bool:
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


def build_fetch_fsm(skills: RobotSkills, cfg: FetchConfig, cancel: CancelToken | None = None) -> StateMachine:
    """FETCH (top) = FIND (sub-machine) -> APPROACH -> CONFIRM -> PICK -> DELIVER -> PLACE."""
    world = skills.world
    cancel = cancel or CancelToken()
    clock = lambda: world.t  # noqa: E731

    def run(bb: Blackboard, name: str, **args: Any) -> SkillResult:
        r = skills.call(name, args, cancel=cancel)
        bb.setdefault("log", []).append((round(world.t, 2), name, args, r.code))
        return r

    def preempt(bb: Blackboard) -> str | None:
        if world.estop or cancel.cancelled:
            return "preempted"
        if world.t - bb["t_start"] > cfg.deadline_s:
            return "timeout"
        return None

    def failed_because_of_preemption(r: SkillResult) -> bool:
        return r.code in ("CANCELED",) or world.estop

    # --- FIND: search rooms until the object is seen ---------------------------------------------
    def look(bb: Blackboard) -> str:
        r = run(bb, "detect_objects")
        hits = [d for d in r.data.get("detections", []) if _matches(d["label"], cfg.object_label)] if r.ok else []
        if hits:
            bb["object"] = hits[0]
            return "found"
        bb["looks"] = bb.get("looks", 0) + 1
        return "look_again" if bb["looks"] < cfg.looks_per_room else "not_here"

    def next_room(bb: Blackboard) -> str:
        bb["looks"] = 0
        while bb.setdefault("rooms_left", list(cfg.rooms)):
            room = bb["rooms_left"].pop(0)
            r = run(bb, "navigate_to", place=room)
            if r.ok:
                return "arrived"
            if failed_because_of_preemption(r):
                return "preempted"
        return "no_rooms_left"

    find = StateMachine("FIND", ("found", "not_found", "preempted", "timeout"), clock=clock, preempt=preempt)
    find.add_state("LOOK", CallbackState(("found", "look_again", "not_here"), look),
                   {"found": "found", "look_again": "LOOK", "not_here": "NEXT_ROOM"})
    find.add_state("NEXT_ROOM", CallbackState(("arrived", "no_rooms_left", "preempted"), next_room),
                   {"arrived": "LOOK", "no_rooms_left": "not_found", "preempted": "preempted"})

    # --- the task --------------------------------------------------------------------------------
    def navigate_state(place_key: str, counter: str) -> Callable[[Blackboard], str]:
        def fn(bb: Blackboard) -> str:
            place = bb["object"]["near_place"] if place_key == "object" else cfg.target
            r = run(bb, "navigate_to", place=place)
            if r.ok:
                bb[counter] = 0
                return "arrived"
            if failed_because_of_preemption(r):
                return "preempted"
            bb[counter] = bb.get(counter, 0) + 1
            return "retry" if bb[counter] < cfg.max_nav_attempts else "failed"
        return fn

    def confirm(bb: Blackboard) -> str:
        for _ in range(2):
            r = run(bb, "detect_objects")
            hits = [d for d in r.data.get("detections", []) if _matches(d["label"], cfg.object_label)] if r.ok else []
            if hits:
                bb["object"] = hits[0]
                return "visible"
        return "lost"

    def pick(bb: Blackboard) -> str:
        r = run(bb, "pick", object_id=bb["object"]["object_id"])
        if r.ok:
            return "holding"
        if failed_because_of_preemption(r):
            return "preempted"
        bb["grasp_attempts"] = bb.get("grasp_attempts", 0) + 1
        if bb["grasp_attempts"] >= cfg.max_grasp_attempts:
            bb["failure"] = f"pick failed {bb['grasp_attempts']} times: {r.code}"
            return "failed"
        return "out_of_reach" if r.code == "OUT_OF_REACH" else "grasp_failed"

    def place(bb: Blackboard) -> str:
        r = run(bb, "place", location=cfg.target)
        if r.ok:
            return "placed"
        bb["failure"] = f"place failed: {r.code}"
        return "preempted" if failed_because_of_preemption(r) else "failed"

    top = StateMachine("FETCH", ("succeeded", "failed", "not_found", "preempted", "timeout"), clock=clock, preempt=preempt)
    top.add_state("FIND", find, {"found": "APPROACH", "not_found": "not_found", "preempted": "preempted", "timeout": "timeout"})
    top.add_state("APPROACH", CallbackState(("arrived", "retry", "failed", "preempted"), navigate_state("object", "nav_fail")),
                  {"arrived": "CONFIRM", "retry": "APPROACH", "failed": "failed", "preempted": "preempted"})
    top.add_state("CONFIRM", CallbackState(("visible", "lost"), confirm), {"visible": "PICK", "lost": "failed"})
    top.add_state("PICK", CallbackState(("holding", "grasp_failed", "out_of_reach", "failed", "preempted"), pick),
                  {"holding": "DELIVER", "grasp_failed": "CONFIRM", "out_of_reach": "APPROACH", "failed": "failed",
                   "preempted": "preempted"})
    top.add_state("DELIVER", CallbackState(("arrived", "retry", "failed", "preempted"), navigate_state("target", "nav_fail")),
                  {"arrived": "PLACE", "retry": "DELIVER", "failed": "failed", "preempted": "preempted"})
    top.add_state("PLACE", CallbackState(("placed", "failed", "preempted"), place),
                  {"placed": "succeeded", "failed": "failed", "preempted": "preempted"})
    return top


def run_fetch(skills: RobotSkills, cfg: FetchConfig, cancel: CancelToken | None = None) -> tuple[str, Blackboard, StateMachine]:
    """Run the fetch FSM; afterwards the robot is always stopped."""
    fsm = build_fetch_fsm(skills, cfg, cancel)
    bb: Blackboard = {"t_start": skills.world.t}
    try:
        outcome = fsm.execute(bb)
    finally:
        skills.world.base.stop()
    return outcome, bb, fsm
