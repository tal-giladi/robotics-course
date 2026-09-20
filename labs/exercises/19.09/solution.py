"""19.09 — reference solution: the safety layer's rules, in order."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

JsonDict = dict[str, Any]

RULES = ("estop", "mode", "geofence", "goal_lock", "budget", "confirmation")


@dataclass(frozen=True)
class Decision:
    allowed: bool
    rule: str
    code: str
    message: str = ""


ALLOW = Decision(True, "-", "OK")


@dataclass(frozen=True)
class Zone:
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
    skills: Sequence[str]
    effects: Mapping[str, str]
    mode_skills: Sequence[str]
    confirm_effects: Sequence[str] = ()
    zones: Sequence[Zone] = ()
    margin_m: float = 0.15
    path: Sequence[tuple[float, float]] | None = None
    destination: tuple[float, float] | None = None
    goal_surface: str | None = None
    goal_label: str | None = None
    budget: Budget = field(default_factory=Budget)
    locked: bool = False
    confirmed: bool = False


def zone_violated(ctx: Context) -> Zone | None:
    points = list(ctx.path or ())
    if ctx.destination is not None:
        points.append(ctx.destination)
    for x, y in points:
        for zone in ctx.zones:
            if zone.contains(x, y, ctx.margin_m):
                return zone
    return None


def budget_exceeded(skill: str, effect: str, budget: Budget) -> str | None:
    if budget.calls.get(skill, 0) >= budget.max_calls_per_skill:
        return f"{skill} called {budget.calls[skill]} times (limit {budget.max_calls_per_skill})"
    if effect == "manipulation" and budget.manipulations >= budget.max_manipulations:
        return f"{budget.manipulations} manipulations already (limit {budget.max_manipulations})"
    if budget.distance_m >= budget.max_distance_m:
        return f"{budget.distance_m:.1f} m driven (limit {budget.max_distance_m:.0f} m)"
    if budget.robot_time_s >= budget.max_robot_time_s:
        return f"{budget.robot_time_s:.0f} s of robot time (limit {budget.max_robot_time_s:.0f} s)"
    return None


def goal_lock_violated(skill: str, args: Mapping[str, Any], ctx: Context) -> str | None:
    if ctx.goal_surface is None or skill != "place":
        return None
    where = str(args.get("location"))
    if where == ctx.goal_surface:
        return None
    return (f"the user asked for the {ctx.goal_label} on the '{ctx.goal_surface}'; this call "
            f"would put it on '{where}'")


def check(skill: str, args: Mapping[str, Any], ctx: Context) -> Decision:
    if ctx.locked:
        return Decision(False, "estop", "LOCKED", "the robot is locked out")
    if skill not in ctx.mode_skills:
        return Decision(False, "mode", "NOT_ALLOWED",
                        f"'{skill}' is not allowed in this mode "
                        f"(allowed: {', '.join(sorted(ctx.mode_skills)) or 'nothing'})")
    effect = ctx.effects.get(skill, "none")
    if skill == "navigate_to":
        zone = zone_violated(ctx)
        if zone is not None:
            return Decision(False, "geofence", "GEOFENCE",
                            f"the route to '{args.get('place')}' enters the keep-out zone "
                            f"'{zone.name}'" + (f" ({zone.reason})" if zone.reason else ""))
    violation = goal_lock_violated(skill, args, ctx)
    if violation is not None:
        return Decision(False, "goal_lock", "GOAL_VIOLATION", violation)
    over = budget_exceeded(skill, effect, ctx.budget)
    if over is not None:
        return Decision(False, "budget", "BUDGET_EXCEEDED", f"physical budget: {over}")
    if effect in ctx.confirm_effects and not ctx.confirmed:
        return Decision(False, "confirmation", "NOT_CONFIRMED", f"a human did not confirm {skill}")
    return ALLOW


def visible_tools(ctx: Context) -> list[str]:
    return sorted(set(ctx.skills) & set(ctx.mode_skills))
