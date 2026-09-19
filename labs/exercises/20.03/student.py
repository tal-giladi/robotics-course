"""20.03 — bringup order and the one state everything else reads.

Three functions turn "is the robot up?" from a feeling into a value:

* ``start_waves``   — the dependency graph, layered into groups that can start in parallel;
* ``component_level`` — a component's reported level, with silence treated as a level of its own;
* ``robot_state``   — per-component levels plus the e-stop, reduced to INIT / READY / DEGRADED /
  FAULT / ESTOP, which is what decides whether a mission may start.

Fill in every ``TODO(student)``; check with ``python course.py check 20.03``.
Standard library only.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

#: diagnostic_msgs/DiagnosticStatus levels, worst last.
LEVELS: tuple[str, ...] = ("OK", "WARN", "ERROR", "STALE")
#: The robot states, in the order the tests below expect you to check them.
STATES: tuple[str, ...] = ("ESTOP", "INIT", "FAULT", "DEGRADED", "READY")


def start_waves(requires: Mapping[str, Sequence[str]]) -> list[list[str]]:
    """Layer the bringup graph: wave 0 depends on nothing, wave n only on waves before it.

    ``requires`` maps each unit to the units it needs. Return a list of waves, each **sorted
    alphabetically** so the answer is identical on every machine and a diff is readable.

    Raise ``ValueError`` if a dependency names a unit that is not in ``requires`` (a typo in a
    launch file), or if the graph has a cycle (two nodes each waiting for the other — the
    bringup would hang forever and you would blame the hardware).

    >>> start_waves({"a": [], "b": ["a"], "c": ["a"], "d": ["b", "c"]})
    [['a'], ['b', 'c'], ['d']]
    """
    raise NotImplementedError  # TODO(student)


def component_level(reported: str | None, last_stamp_s: float | None, now_s: float,
                    period_s: float, tolerance: float = 3.0) -> str:
    """The level to use for a component right now, from what it last said and when.

    * never reported at all (``reported`` or ``last_stamp_s`` is ``None``) -> ``"STALE"``;
    * silent for more than ``tolerance`` reporting periods -> ``"STALE"``, whatever it last said.
      A crashed driver's last message was "OK", and believing it is the classic integration bug;
    * otherwise the level it reported.

    >>> component_level("OK", 10.0, 10.5, period_s=1.0)
    'OK'
    >>> component_level("OK", 10.0, 14.0, period_s=1.0)
    'STALE'
    >>> component_level(None, None, 5.0, period_s=1.0)
    'STALE'
    """
    raise NotImplementedError  # TODO(student)


def robot_state(levels: Mapping[str, str], criticality: Mapping[str, str], *,
                estop: bool = False, bringup_complete: bool = True,
                bringup_timed_out: bool = False) -> str:
    """The single state that decides what the robot is allowed to do.

    ``criticality`` maps each component to ``"required"``, ``"degraded"`` or ``"optional"``.
    Components missing from ``criticality`` are ignored (they are not part of this robot).

    The order of the tests *is* the policy — get it wrong and a robot that is merely still
    booting reports FAULT, or a fault gets hidden behind a DEGRADED:

    1. ``estop`` -> ``"ESTOP"``. The button is a fact about the hardware; nothing overrides it.
    2. the bringup is neither complete nor timed out -> ``"INIT"``. A LiDAR that needs 4 s to
       spin up is not a fault for those 4 s.
    3. any **required** component at ``"ERROR"`` or ``"STALE"`` -> ``"FAULT"``.
    4. any **required or degraded** component at ``"WARN"`` or worse -> ``"DEGRADED"``.
       An ``"optional"`` component never degrades the robot: losing the LLM bridge means no new
       missions, not an unsafe robot.
    5. otherwise ``"READY"``.

    >>> robot_state({"lidar": "OK"}, {"lidar": "required"})
    'READY'
    >>> robot_state({"lidar": "STALE"}, {"lidar": "required"})
    'FAULT'
    >>> robot_state({"lidar": "STALE"}, {"lidar": "required"}, estop=True)
    'ESTOP'
    """
    raise NotImplementedError  # TODO(student)
