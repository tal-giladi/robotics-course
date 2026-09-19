"""19.04 — the state-machine engine: validate it, then run it.

The task logic is not the exercise; the *engine* is. You implement the three methods that make a
transition table trustworthy: ``add_state``, ``validate`` and ``execute``.

Fill in every ``TODO(student)``; check with ``python course.py check 19.04``.
Standard library only.
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
        """Register ``state`` under ``name`` with its outcome -> target mapping. Returns ``self``.

        * A repeated name is ``InvalidMachine(f"duplicate state {name}")``.
        * Each target is either another state's name or one of **this machine's** outcomes.
        * The first state added becomes ``start``.
        * When ``state`` is itself a ``StateMachine``, give it *this* machine's ``trace`` list
          (``state.trace = self.trace``) so the whole hierarchy shares one timeline.
        """
        # TODO(student): implement.
        raise NotImplementedError("add_state")

    def validate(self) -> None:
        """Raise ``InvalidMachine`` for any machine that must not run. Return ``None`` if it may.

        Check, with these exact message formats:

        1. no states, or ``start`` is not one of them
           -> ``f"{self.name}: no start state"``
        2. a declared outcome of a state with no transition
           -> ``f"{self.name}.{state}: outcome '{outcome}' has no transition"``
        3. a transition for an outcome the state never declares (a typo, or a stale table)
           -> ``f"{self.name}.{state}: transition for undeclared outcome '{outcome}'"``
        4. a target that is neither a state nor one of this machine's outcomes
           -> ``f"{self.name}.{state}: '{outcome}' goes to unknown '{target}'"``
        5. sub-machines: validate them too (recursively)
        6. states that cannot be reached from ``start`` by following transitions
           -> ``f"{self.name}: unreachable states {sorted(unreachable)}"``

        Checks 2–4 are why a transition table beats an ``if``/``elif`` chain: a missing branch is
        an error here instead of silence on the robot.
        """
        # TODO(student): implement.
        raise NotImplementedError("validate")

    def execute(self, bb: Blackboard) -> str:
        """Run the machine and return one of ``self.outcomes``.

        1. ``validate()`` first — never run an invalid machine.
        2. From ``start``, at most ``max_transitions`` times:

           a. ``forced = self.preempt(bb)``. If it is not ``None``: it must be one of
              ``self.outcomes``, else ``InvalidMachine(f"{self.name}: preemption outcome "
              f"'{forced}' is not an outcome of this machine")``. Append a ``TraceEntry(
              self.clock(), self.name, current, forced, forced)`` and return ``forced``.
           b. ``outcome = self.states[current].execute(bb)``. If it is not one of that state's
              declared outcomes:
              ``InvalidMachine(f"{self.name}.{current} returned undeclared outcome '{outcome}'")``.
           c. Look up the target, append ``TraceEntry(round(self.clock(), 2), self.name, current,
              outcome, target)``.
           d. If the target is one of ``self.outcomes``, return it. Otherwise continue from it.

        3. Running out of transitions is
           ``RuntimeError(f"{self.name}: more than {self.max_transitions} transitions "
           f"(a loop without a counter?)")`` — a bounded crash with a trace beats a robot
           driving in circles.
        """
        # TODO(student): implement.
        raise NotImplementedError("execute")
