"""20.05 — turning runs into verdicts, and two suites into a regression report.

A scenario run produces numbers. Two functions turn those numbers into decisions:

* ``verdict`` — one run against its thresholds: every reason it failed, not the first one;
* ``regressions`` — two suite runs against each other: what got worse, and by how much.

Fill in every ``TODO(student)``; check with ``python course.py check 20.05``.
Standard library only.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def verdict(run: Mapping[str, Any], thresholds: Mapping[str, Any]) -> list[str]:
    """Every threshold this run violated, as short keys, sorted. ``[]`` means the scenario passed.

    ``run`` has: ``reached`` (bool), ``collided`` (bool), ``duration_s``, ``min_clearance_m``,
    ``interventions`` (int), ``blind_distance_m``.
    ``thresholds`` has: ``outcome`` (``"reach"`` or ``"stop-safely"``), ``max_time_s``,
    ``min_clearance_m``, ``max_interventions``, ``max_blind_distance_m``.

    The keys you may return:

    =================  ==========================================================================
    ``"not-reached"``  ``outcome`` is ``"reach"`` and ``reached`` is False
    ``"should-stop"``  ``outcome`` is ``"stop-safely"`` and ``reached`` is True — the robot got
                       somewhere it should not have been able to reach
    ``"lucky-stop"``   ``outcome`` is ``"stop-safely"``, it did not reach, and ``interventions``
                       is 0: it stopped, but nothing in the design made it stop
    ``"collided"``     ``collided``
    ``"too-slow"``     ``duration_s`` > ``max_time_s``
    ``"too-close"``    ``min_clearance_m`` < ``min_clearance_m`` threshold
    ``"too-many-stops"``  ``interventions`` > ``max_interventions``
    ``"drove-blind"``  ``blind_distance_m`` > ``max_blind_distance_m``
    =================  ==========================================================================

    Report **all** of them. A test that stops at the first failure makes you re-run the whole
    suite once per bug, and a robot run is not cheap.

    >>> ok = {"reached": True, "collided": False, "duration_s": 13.1, "min_clearance_m": 0.118,
    ...       "interventions": 0, "blind_distance_m": 0.0}
    >>> limits = {"outcome": "reach", "max_time_s": 60.0, "min_clearance_m": 0.04,
    ...           "max_interventions": 4, "max_blind_distance_m": 0.05}
    >>> verdict(ok, limits)
    []
    >>> verdict({**ok, "collided": True, "duration_s": 99.0}, limits)
    ['collided', 'too-slow']
    """
    raise NotImplementedError  # TODO(student)


def regressions(before: Mapping[str, Mapping[str, Any]], after: Mapping[str, Mapping[str, Any]],
                thresholds: Mapping[str, Mapping[str, Any]]) -> list[str]:
    """Compare two suite runs and report what a reviewer has to look at, sorted by scenario id.

    ``before`` and ``after`` map a scenario id to that scenario's ``run`` mapping; ``thresholds``
    maps a scenario id to its thresholds. Scenarios that are not in **both** runs are skipped —
    a new scenario is not a regression, and a deleted one is not a fix.

    For each scenario, in this order, append at most these lines:

    * ``"REGRESSION <id>: <k1>, <k2>"`` — it passed before and fails now; the keys are the new
      verdict, comma-separated in sorted order. This is the line that blocks a release.
    * ``"FIXED <id>"`` — it failed before and passes now.
    * ``"WORSE <id>: clearance"`` — ``min_clearance_m`` dropped by more than 0.02 m, whether or
      not it still passes. A margin that halves is a warning even while it is inside the limit.
    * ``"WORSE <id>: time"`` — ``duration_s`` grew by more than 25 % **and** by more than 1 s
      (the second condition keeps noise on short runs out of the report).

    A scenario can produce both a REGRESSION line and a WORSE line.

    >>> limits = {"S1": {"outcome": "reach", "max_time_s": 60.0, "min_clearance_m": 0.04,
    ...                  "max_interventions": 4, "max_blind_distance_m": 0.05}}
    >>> good = {"reached": True, "collided": False, "duration_s": 13.0, "min_clearance_m": 0.12,
    ...         "interventions": 0, "blind_distance_m": 0.0}
    >>> regressions({"S1": good}, {"S1": good}, limits)
    []
    >>> regressions({"S1": good}, {"S1": {**good, "collided": True}}, limits)
    ['REGRESSION S1: collided']
    """
    raise NotImplementedError  # TODO(student)
