"""19.04 — reference solution. The same engine as ``19-llm-robot-agents/code/robot_agent/fsm.py``.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

Blackboard = dict[str, Any]


class InvalidMachine(ValueError):
    """A machine that must not be allowed to run (or a state that broke its own contract)."""


class State:
    """One activity with a closed set of outcomes. ``execute`` returns exactly one of them."""

    outcomes: tuple[str, ...] = ()

    def execute(self, bb: Blackboard) -> str:
        raise NotImplementedError


class CallbackState(State):
    """A state made from a plain function — the usual way to wrap one skill call."""

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


@dataclass
class StateMachine(State):
    """A machine that is itself a state, so machines nest.

    ``outcomes`` are the machine's own outcomes — the names it can return to *its* caller.
    ``preempt(bb)`` returns one of those outcomes to abort with, or ``None`` to carry on; it is
    checked before every state, which is how one e-stop condition covers the whole machine.
    """

    name: str
    outcomes: tuple[str, ...]
    clock: Callable[[], float] = field(default=lambda: 0.0)
    preempt: Callable[[Blackboard], str | None] = field(default=lambda bb: None)
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
            state.trace = self.trace          # one timeline for the whole hierarchy
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
                    raise InvalidMachine(
                        f"{self.name}: preemption outcome '{forced}' is not an outcome of this machine")
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
