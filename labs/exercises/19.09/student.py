"""19.09 — the safety layer: deterministic rules between the agent and the robot.

Six rules, in one fixed order, all of them outside the model. Every one must hold on its own, so
none of them may depend on another having run first.

The data types and the rule order are given. You write the checks.

Fill in every ``TODO(student)``; check with ``python course.py check 19.09``.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

JsonDict = dict[str, Any]

#: The order the rules run in: most absolute first, most expensive (a human) last.
RULES = ("estop", "mode", "geofence", "goal_lock", "budget", "confirmation")


# ----------------------------------------------------------------------------------------------
# Given
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Decision:
    allowed: bool
    rule: str  # one of RULES, or "-"
    code: str  # LOCKED | NOT_ALLOWED | GEOFENCE | GOAL_VIOLATION | BUDGET_EXCEEDED | NOT_CONFIRMED | OK
    message: str = ""


ALLOW = Decision(True, "-", "OK")


@dataclass(frozen=True)
class Zone:
    """A keep-out rectangle in the map frame."""

    name: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    reason: str = ""

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.x_min - margin <= x <= self.x_max + margin
                and self.y_min - margin <= y <= self.y_max + margin)


@dataclass
class Budget:
    """What the *robot* may spend. Not tokens: metres, seconds and irreversible actions."""

    max_distance_m: float = 60.0
    max_robot_time_s: float = 420.0
    max_manipulations: int = 4
    max_calls_per_skill: int = 25
    distance_m: float = 0.0
    robot_time_s: float = 0.0
    manipulations: int = 0
    calls: dict[str, int] = field(default_factory=dict)


@dataclass
class Context:
    """Everything the checks may look at. Note what is NOT here: the model, and its explanation."""

    skills: Sequence[str]  # every skill the robot has
    effects: Mapping[str, str]  # skill -> "none" | "motion" | "manipulation"
    mode_skills: Sequence[str]  # what the current operating mode allows
    confirm_effects: Sequence[str] = ()  # effect classes needing a human
    zones: Sequence[Zone] = ()
    margin_m: float = 0.15
    path: Sequence[tuple[float, float]] | None = None  # the planned route, if any
    destination: tuple[float, float] | None = None
    goal_surface: str | None = None  # from the USER's request, never from the world
    goal_label: str | None = None
    budget: Budget = field(default_factory=Budget)
    locked: bool = False
    confirmed: bool = False  # what the human actually said


# ----------------------------------------------------------------------------------------------
# Yours
# ----------------------------------------------------------------------------------------------
def zone_violated(ctx: Context) -> Zone | None:
    """The first keep-out zone the robot would enter, or ``None``.

    Check **every point of ``ctx.path``** and then ``ctx.destination`` (either may be ``None``),
    each against every zone with ``ctx.margin_m``. Checking only the destination is the mistake
    that makes a geofence decorative: a permitted room reached by driving through the nursery.
    """
    # TODO(student): implement.
    raise NotImplementedError("zone_violated")


def budget_exceeded(skill: str, effect: str, budget: Budget) -> str | None:
    """A one-line reason this call would exceed the physical budget, or ``None``. In order:

    1. ``budget.calls.get(skill, 0) >= max_calls_per_skill`` ->
       ``f"{skill} called {n} times (limit {max})"``
    2. ``effect == "manipulation"`` and ``manipulations >= max_manipulations`` ->
       ``f"{n} manipulations already (limit {max})"``
    3. ``distance_m >= max_distance_m`` -> ``f"{d:.1f} m driven (limit {max:.0f} m)"``
    4. ``robot_time_s >= max_robot_time_s`` -> ``f"{t:.0f} s of robot time (limit {max:.0f} s)"``
    """
    # TODO(student): implement.
    raise NotImplementedError("budget_exceeded")


def goal_lock_violated(skill: str, args: Mapping[str, Any], ctx: Context) -> str | None:
    """Would this call put the object somewhere the user did not ask for?

    Only ``place`` is constrained, and only when ``ctx.goal_surface`` is set. Return ``None`` when
    ``args["location"]`` equals it, otherwise the message
    ``f"the user asked for the {label} on the '{surface}'; this call would put it on '{where}'"``.

    Notice what this check does *not* look at: what the robot read, what the model argued, how
    plausible the new location is. The goal came from the user's channel; nothing else may write
    it. That is the whole defence against the note on the wall.
    """
    # TODO(student): implement.
    raise NotImplementedError("goal_lock_violated")


def check(skill: str, args: Mapping[str, Any], ctx: Context) -> Decision:
    """Run the rules in ``RULES`` order and return the first denial, else ``ALLOW``. Pure.

    * ``ctx.locked`` -> ``Decision(False, "estop", "LOCKED", "the robot is locked out")``
    * ``skill not in ctx.mode_skills`` -> rule ``"mode"``, code ``NOT_ALLOWED``, message
      ``f"'{skill}' is not allowed in this mode (allowed: {', '.join(sorted(mode_skills)) or 'nothing'})"``
    * ``skill == "navigate_to"`` and ``zone_violated`` -> rule ``"geofence"``, code ``GEOFENCE``,
      message ``f"the route to '{args['place']}' enters the keep-out zone '{zone.name}'"`` plus
      ``f" ({zone.reason})"`` when there is a reason
    * ``goal_lock_violated`` -> rule ``"goal_lock"``, code ``GOAL_VIOLATION``, that message
    * ``budget_exceeded`` (with ``ctx.effects.get(skill, "none")``) -> rule ``"budget"``, code
      ``BUDGET_EXCEEDED``, message ``f"physical budget: {reason}"``
    * the effect is in ``ctx.confirm_effects`` and ``not ctx.confirmed`` -> rule
      ``"confirmation"``, code ``NOT_CONFIRMED``, message ``f"a human did not confirm {skill}"``
    """
    # TODO(student): implement.
    raise NotImplementedError("check")


def visible_tools(ctx: Context) -> list[str]:
    """The tool names to put in the model's context: sorted, and only what the mode allows.

    A tool the model can see is a tool it will eventually try. Removing temptation is cheaper than
    refusing it, and a deterministic order keeps the client's prompt cache warm.
    """
    # TODO(student): implement.
    raise NotImplementedError("visible_tools")
