"""14.05 — Analytic inverse kinematics of a planar 2-link arm, and choosing between branches.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.05``.
Only the standard library and numpy are needed.

The arm is the planar model of the SO-101's upper arm + forearm from 14.04: two revolute joints
in a vertical plane, link lengths ``l1`` and ``l2`` (metres), joint angles ``q1`` and ``q2``
(radians, ``q2`` measured relative to link 1). Forward kinematics is given below; your job is to
go the other way, and then to decide *which* of the answers to send to the arm.

Units, everywhere: metres and radians. Angles are wrapped to (-pi, pi].
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

# --- given ---------------------------------------------------------------------------------------
EPS = 1e-9


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


def planar_fk(lengths: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
    """Tip (x, y, phi) of a planar arm. Link i points along q1 + ... + qi; phi = sum(q), wrapped.

    This is 14.04's forward kinematics, here so the tests (and you) can round-trip every IK answer
    back through it. An IK solution that does not reproduce its own target is not a solution.
    """
    x = y = phi = 0.0
    for length, qi in zip(lengths, q):
        phi += qi
        x += length * math.cos(phi)
        y += length * math.sin(phi)
    return x, y, wrap_angle(phi)


@dataclass(frozen=True)
class TwoLinkSolution:
    """One IK branch. ``elbow`` is "up" (q2 < 0) or "down" (q2 > 0)."""

    q1: float
    q2: float
    elbow: str


# --- TODO(student) -------------------------------------------------------------------------------
def reachability(l1: float, l2: float, r: float, tol: float = EPS) -> str:
    """Classify a target distance ``r`` from the shoulder against the arm's annulus of reach.

    The workspace of a 2-link arm is the annulus between ``|l1 - l2|`` (fully folded) and
    ``l1 + l2`` (fully stretched). Return exactly one of:

    * ``"inside"``          — closer than ``|l1 - l2|``: inside the dead hole, unreachable
    * ``"boundary_inner"``  — exactly ``|l1 - l2|`` (within ``tol``): one solution, arm folded
    * ``"interior"``        — strictly between the two radii: two solutions
    * ``"boundary_outer"``  — exactly ``l1 + l2`` (within ``tol``): one solution, arm stretched
    * ``"outside"``         — farther than ``l1 + l2``: unreachable

    ``r`` must not be negative, and both lengths must be positive.

    Note the order you test in: when ``l1 == l2`` the inner radius is 0, so ``r = 0`` is
    ``"boundary_inner"``, not ``"inside"``. Check the two boundaries first.
    """
    raise NotImplementedError  # TODO(student)


def two_link_ik(l1: float, l2: float, x: float, y: float, tol: float = EPS) -> list[TwoLinkSolution]:
    """Every analytic IK solution for the tip at ``(x, y)``.

    The law of cosines gives the elbow angle from the target distance alone::

        cos(q2) = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)

    and each choice of ``sin(q2) = -+ sqrt(1 - cos^2 q2)`` gives one branch. With ``q2`` known,
    the shoulder angle is the direction of the target minus the angle the second link contributes::

        q1 = atan2(y, x) - atan2(l2 * sin(q2), l1 + l2 * cos(q2))

    Return, with ``q1`` wrapped to (-pi, pi]:

    * ``[]`` when the target is unreachable,
    * **one** solution on either boundary — the two branches coincide there, and returning the
      same pose twice would make a caller think it has a choice,
    * **two** solutions in the interior, elbow **"up"** (``q2 < 0``) first, then **"down"**.

    Do not trust floating point at the boundary: ``cos(q2)`` can come out as 1.0000000000000002,
    and ``sqrt`` of a small negative number is a NaN that propagates silently into the arm.
    Clamp it, and use ``reachability`` to decide how many branches exist.
    """
    raise NotImplementedError  # TODO(student)


def pick_solution(solutions: Sequence[TwoLinkSolution], q_current: Sequence[float]) -> TwoLinkSolution:
    """The branch closest to where the arm already is — the fix for the elbow flip of 14.05-E5.

    "Closest" is the largest joint move, minimised: for each candidate take
    ``max(|wrap(q1 - q1_current)|, |wrap(q2 - q2_current)|)`` and return the smallest.
    Comparing raw differences instead of wrapped ones makes a +179 deg -> -179 deg step look like
    358 deg of travel, and the arm takes the long way round.

    Ties go to the first candidate (so the order ``two_link_ik`` returns stays meaningful).
    Raise ``ValueError`` on an empty list: "no solution" is not something to paper over.
    """
    raise NotImplementedError  # TODO(student)
