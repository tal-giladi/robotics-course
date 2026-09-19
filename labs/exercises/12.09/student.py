"""12.09 — The mission layer above Nav2: named places and a waypoint state machine.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.09``.
Only the standard library and PyYAML are needed.

``nav2_simple_commander`` gives you one primitive — send a goal, poll until it finishes, read the
result — and *no* policy. The policy is yours: what "the kitchen" means, how many times to retry a
stop, whether a failed stop skips or aborts the mission, and what to do on arrival. That policy is
what you implement here, against the same ``Navigator`` protocol that
``nav2_simple_commander.robot_navigator.BasicNavigator`` satisfies, so the code you write runs
unchanged on the robot.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

import yaml


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Place:
    """A goal pose with a name a human uses."""

    name: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


class TaskResult(Enum):
    """Mirrors ``nav2_simple_commander.robot_navigator.TaskResult``."""

    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


@dataclass
class Feedback:
    """The parts of ``NavigateToPose.Feedback`` a mission layer uses."""

    distance_remaining: float = 0.0
    navigation_time: float = 0.0
    number_of_recoveries: int = 0


class Navigator(Protocol):
    """What a mission needs from a navigation stack."""

    def go_to_pose(self, place: Place) -> bool: ...     # False = the goal was rejected
    def is_task_complete(self) -> bool: ...             # poll; False while the robot is driving
    def get_feedback(self) -> Feedback | None: ...
    def get_result(self) -> TaskResult: ...
    def cancel_task(self) -> None: ...


class StopState(str, Enum):
    PENDING = "pending"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    SKIPPED = "skipped"
    CANCELED = "canceled"


@dataclass
class Stop:
    """One waypoint of a mission, plus what to do when the robot gets there."""

    place: Place
    task: Callable[[Place], None] | None = None
    max_attempts: int = 2
    state: StopState = StopState.PENDING
    attempts: int = 0
    nav_time_s: float = 0.0
    recoveries: int = 0


@dataclass
class MissionReport:
    stops: list[Stop]

    @property
    def succeeded(self) -> int:
        return sum(s.state is StopState.SUCCEEDED for s in self.stops)

    @property
    def complete(self) -> bool:
        return all(s.state is StopState.SUCCEEDED for s in self.stops)

    @property
    def total_time_s(self) -> float:
        return sum(s.nav_time_s for s in self.stops)


# ==================================================================================================
#  Your turn
# ==================================================================================================
def load_places(path: Path | str) -> dict[str, Place]:
    """Load a places file into ``{name: Place}``.

    The file looks like::

        frame_id: map          # optional, default "map"
        places:
          kitchen: {x: 5.0, y: 2.3, yaw: 1.5708}
          bedroom: {x: 5.0, y: 3.2}          # yaw is optional, default 0.0

    Use ``yaml.safe_load``. Every Place gets its name from the key and the file's ``frame_id``.
    """
    # TODO(student)
    raise NotImplementedError("load_places")


def mission_from_names(names: Iterable[str], places: dict[str, Place], **kwargs: Any) -> WaypointMission:
    """Build a mission from place names.

    An unknown name must raise ``KeyError`` whose message contains the offending name **and** the
    known names (a mission that silently skips "kitchn" is worse than one that refuses to start).
    Pass ``**kwargs`` on to ``WaypointMission``.
    """
    # TODO(student)
    raise NotImplementedError("mission_from_names")


@dataclass
class WaypointMission:
    """Visit ``stops`` in order, retrying and reporting.

    ``run(navigator, max_iterations)``:

    * For each stop, attempt it (below). If an attempt sequence says the mission cannot continue,
      mark every still-PENDING stop SKIPPED and stop. Return a ``MissionReport`` over ``self.stops``.

    One stop, up to ``stop.max_attempts`` times:

    1. ``stop.attempts += 1``, ``stop.state = StopState.ACTIVE``.
    2. ``navigator.go_to_pose(stop.place)``. If it returns False the goal was rejected: this attempt
       is over, go round the loop again.
    3. Poll: ``while not navigator.is_task_complete()``. Count the polls; if they exceed
       ``max_iterations``, ``navigator.cancel_task()`` and leave the loop.
    4. Read ``navigator.get_feedback()`` once. If it is not None, **add** its ``navigation_time``
       to ``stop.nav_time_s`` and its ``number_of_recoveries`` to ``stop.recoveries`` (a retried
       stop accumulates both).
    5. ``navigator.get_result()``:
       - SUCCEEDED -> ``stop.state = SUCCEEDED``, call ``stop.task(stop.place)`` if there is one,
         and the mission continues.
       - CANCELED -> ``stop.state = CANCELED``; the mission does **not** continue (somebody
         cancelled on purpose; do not retry).
       - anything else -> log it and try again.
    6. Out of attempts: ``stop.state = FAILED``. The mission continues unless ``stop_on_failure``.

    Use ``self.log(...)`` for messages so the checker can run quietly.
    """

    stops: list[Stop]
    stop_on_failure: bool = False
    log: Callable[[str], None] = print

    def run(self, navigator: Navigator, max_iterations: int = 100_000) -> MissionReport:
        # TODO(student)
        raise NotImplementedError("WaypointMission.run")
