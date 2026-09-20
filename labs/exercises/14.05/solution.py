"""14.05 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

# --- given ---------------------------------------------------------------------------------------
EPS = 1e-9


def wrap_angle(a: float) -> float:
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


def planar_fk(lengths: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
    x = y = phi = 0.0
    for length, qi in zip(lengths, q):
        phi += qi
        x += length * math.cos(phi)
        y += length * math.sin(phi)
    return x, y, wrap_angle(phi)


@dataclass(frozen=True)
class TwoLinkSolution:
    q1: float
    q2: float
    elbow: str


# --- solution ------------------------------------------------------------------------------------
def reachability(l1: float, l2: float, r: float, tol: float = EPS) -> str:
    if l1 <= 0 or l2 <= 0:
        raise ValueError("link lengths must be positive")
    if r < 0:
        raise ValueError("r is a distance and cannot be negative")
    inner, outer = abs(l1 - l2), l1 + l2
    if abs(r - inner) <= tol:          # before "inside": with l1 == l2 the hole has radius 0
        return "boundary_inner"
    if abs(r - outer) <= tol:
        return "boundary_outer"
    if r < inner:
        return "inside"
    if r > outer:
        return "outside"
    return "interior"


def two_link_ik(l1: float, l2: float, x: float, y: float, tol: float = EPS) -> list[TwoLinkSolution]:
    r = math.hypot(x, y)
    where = reachability(l1, l2, r, tol)
    if where in ("inside", "outside"):
        return []
    c2 = (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c2 = max(-1.0, min(1.0, c2))                       # kill the 1.0000000000000002 case
    s2_abs = math.sqrt(max(0.0, 1.0 - c2 * c2))
    branches = ((-s2_abs, "up"),) if where != "interior" else ((-s2_abs, "up"), (s2_abs, "down"))
    out = []
    for s2, elbow in branches:
        q2 = math.atan2(s2, c2)
        q1 = math.atan2(y, x) - math.atan2(l2 * s2, l1 + l2 * c2)
        out.append(TwoLinkSolution(wrap_angle(q1), q2, elbow))
    return out


def pick_solution(solutions: Sequence[TwoLinkSolution], q_current: Sequence[float]) -> TwoLinkSolution:
    if not solutions:
        raise ValueError("no solutions to pick from")
    q1_now, q2_now = float(q_current[0]), float(q_current[1])

    def travel(s: TwoLinkSolution) -> float:
        return max(abs(wrap_angle(s.q1 - q1_now)), abs(wrap_angle(s.q2 - q2_now)))

    best = solutions[0]
    best_cost = travel(best)
    for candidate in solutions[1:]:
        cost = travel(candidate)
        if cost < best_cost:            # strict: ties keep the earlier candidate
            best, best_cost = candidate, cost
    return best
