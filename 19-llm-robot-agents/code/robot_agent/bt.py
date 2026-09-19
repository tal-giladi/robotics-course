"""The fetch-and-place task as a behavior tree with py_trees (19.05).

Version-sensitive (verified 2026-09 against py_trees 2.6.0 on PyPI, https://py-trees.readthedocs.io/).

Structure (``build_fetch_tree``)::

    FetchTask                 Sequence(memory=False)  <- re-checks the guard on EVERY tick (reactive)
    ├── EStopReleased?        condition
    └── TaskDeadline          SimTimeout (simulated clock)
        └── Fetch             Sequence(memory=True)   <- resumes the running child
            ├── FindObject    Selector(memory=True)
            │   ├── ObjectKnown?
            │   ├── LookHere      Retry(3) -> DetectObject
            │   └── Search<room>  Sequence(memory=True): NavigateTo(room), Retry(3) -> DetectObject  (one per room)
            ├── PickWithRetries   Retry(3) -> Sequence(memory=True): NavigateTo(object), DetectObject, Pick
            ├── Deliver           Retry(2) -> NavigateTo(target)
            └── Place

Action leaves are *non-blocking*: ``initialise`` starts a skill (an ``ActionHandle``, the twin of a
ROS 2 action goal), ``update`` advances it by one tick period and returns RUNNING until it ends,
``terminate(INVALID)`` cancels it. That is the same pattern as BehaviorTree.CPP's StatefulActionNode
(onStart / onRunning / onHalted) that Nav2's BT nodes use.
"""

from __future__ import annotations

import itertools
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import py_trees
from py_trees.common import Access, Status

from robot_agent.sim_world import ActionHandle
from robot_agent.sim_world import Status as ActionStatus
from robot_agent.skills import RobotSkills, SkillResult, result_from_action

_ids = itertools.count()


def _matches(label: str, wanted: str) -> bool:
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


class SkillAction(py_trees.behaviour.Behaviour):
    """A leaf that runs one skill through the gateway without blocking the tree."""

    def __init__(self, name: str, skills: RobotSkills, skill: str, args: Callable[[Any], dict[str, Any]],
                 ns: str, tick_period_s: float = 0.1,
                 on_success: Callable[[Any, SkillResult], Status] | None = None) -> None:
        super().__init__(name)
        self.skills, self.skill, self.args, self.tick_period_s, self.on_success = skills, skill, args, tick_period_s, on_success
        self.bb = self.attach_blackboard_client(name=name, namespace=ns)
        for key in ("object", "last_result"):
            self.bb.register_key(key, access=Access.WRITE)
        self.handle: ActionHandle | None = None
        self.call_args: dict[str, Any] = {}
        self.rejected: SkillResult | None = None

    def initialise(self) -> None:
        self.call_args = self.args(self.bb)
        started = self.skills.start(self.skill, self.call_args)
        self.handle, self.rejected = (started, None) if isinstance(started, ActionHandle) else (None, started)

    def update(self) -> Status:
        if self.rejected is not None:
            self.bb.last_result = self.rejected
            self.feedback_message = self.rejected.code
            return Status.FAILURE
        assert self.handle is not None
        state = self.handle.step(self.tick_period_s)
        if state is ActionStatus.RUNNING:
            self.feedback_message = str(self.handle.feedback)
            return Status.RUNNING
        result = result_from_action(self.skill, self.handle.result, self.skills.world.t - self.handle.started_at)  # type: ignore[arg-type]
        self.skills.record(self.skill, self.call_args, result, self.handle.started_at)
        self.bb.last_result = result
        self.feedback_message = result.code
        if not result.ok:
            return Status.FAILURE
        return self.on_success(self.bb, result) if self.on_success else Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        # Halted while running (a guard failed, a timeout fired, an ancestor was preempted): cancel the goal.
        if new_status == Status.INVALID and self.handle is not None and self.handle.status is ActionStatus.RUNNING:
            self.handle.cancel()
            while self.handle.step() is ActionStatus.RUNNING:
                pass
            self.skills.world.base.stop()
            self.skills.record(self.skill, self.call_args, result_from_action(
                self.skill, self.handle.result, self.skills.world.t - self.handle.started_at), self.handle.started_at)  # type: ignore[arg-type]
            self.feedback_message = "halted"
        self.handle = None


class Check(py_trees.behaviour.Behaviour):
    """A condition leaf: SUCCESS if the predicate holds, else FAILURE. Never RUNNING, never side effects."""

    def __init__(self, name: str, predicate: Callable[[], bool]) -> None:
        super().__init__(name)
        self.predicate = predicate

    def update(self) -> Status:
        return Status.SUCCESS if self.predicate() else Status.FAILURE


class SimTimeout(py_trees.decorators.Decorator):
    """py_trees' Timeout uses the wall clock; robots in simulation (and in ROS, with use_sim_time) need the sim clock."""

    def __init__(self, name: str, child: py_trees.behaviour.Behaviour, duration_s: float, clock: Callable[[], float]) -> None:
        super().__init__(name=name, child=child)
        self.duration_s, self.clock, self.deadline = duration_s, clock, 0.0

    def initialise(self) -> None:
        self.deadline = self.clock() + self.duration_s

    def update(self) -> Status:
        if self.decorated.status == Status.RUNNING and self.clock() > self.deadline:
            self.feedback_message = "timed out"
            self.decorated.stop(Status.INVALID)
            return Status.FAILURE
        return self.decorated.status


@dataclass
class FetchTreeConfig:
    object_label: str
    target: str
    rooms: tuple[str, ...] = ("living_room", "kitchen", "study", "bedroom")
    looks: int = 3
    grasp_attempts: int = 3
    deliver_attempts: int = 2
    deadline_s: float = 300.0
    tick_period_s: float = 0.1


def build_fetch_tree(skills: RobotSkills, cfg: FetchTreeConfig) -> tuple[py_trees.behaviour.Behaviour, Any]:
    world = skills.world
    ns = f"/fetch{next(_ids)}"
    reader = py_trees.blackboard.Client(name="FetchTree", namespace=ns)
    for key in ("object", "last_result"):
        reader.register_key(key, access=Access.WRITE)
    reader.object = None
    reader.last_result = None

    def remember_object(bb: Any, result: SkillResult) -> Status:
        hits = [d for d in result.data.get("detections", []) if _matches(d["label"], cfg.object_label)]
        if not hits:
            return Status.FAILURE
        bb.object = hits[0]
        return Status.SUCCESS

    def detect(name: str) -> SkillAction:
        return SkillAction(name, skills, "detect_objects", lambda bb: {}, ns, cfg.tick_period_s, remember_object)

    def navigate(name: str, place: Callable[[Any], str]) -> SkillAction:
        return SkillAction(name, skills, "navigate_to", lambda bb: {"place": place(bb)}, ns, cfg.tick_period_s)

    Retry, Sequence, Selector = py_trees.decorators.Retry, py_trees.composites.Sequence, py_trees.composites.Selector

    find = Selector("FindObject", memory=True, children=[
        Check("ObjectKnown?", lambda: reader.object is not None),
        Retry("LookHere", detect("DetectHere"), num_failures=cfg.looks),
        *[Sequence(f"Search {room}", memory=True, children=[
            navigate(f"GoTo {room}", lambda bb, r=room: r),
            Retry(f"Look in {room}", detect(f"Detect in {room}"), num_failures=cfg.looks),
        ]) for room in cfg.rooms],
    ])
    pick = Retry("PickWithRetries", Sequence("Approach+Pick", memory=True, children=[
        navigate("GoToObject", lambda bb: bb.object["near_place"]),
        detect("ConfirmObject"),
        SkillAction("Pick", skills, "pick", lambda bb: {"object_id": bb.object["object_id"]}, ns, cfg.tick_period_s),
    ]), num_failures=cfg.grasp_attempts)
    deliver = Retry("Deliver", navigate("GoToTarget", lambda bb: cfg.target), num_failures=cfg.deliver_attempts)
    place = SkillAction("Place", skills, "place", lambda bb: {"location": cfg.target}, ns, cfg.tick_period_s)

    fetch = Sequence("Fetch", memory=True, children=[find, pick, deliver, place])
    root = Sequence("FetchTask", memory=False, children=[
        Check("EStopReleased?", lambda: not world.estop),
        SimTimeout("TaskDeadline", fetch, cfg.deadline_s, clock=lambda: world.t),
    ])
    return root, reader


@dataclass
class TreeRun:
    status: Status
    ticks: int
    sim_time_s: float
    trace: list[tuple[float, str, str]] = field(default_factory=list)  # (t, leaf name, status) on changes


def run_tree(root: py_trees.behaviour.Behaviour, skills: RobotSkills, max_ticks: int = 20_000,
             on_tick: Callable[[int, py_trees.behaviour.Behaviour], None] | None = None) -> TreeRun:
    """Tick until the root stops RUNNING. ``on_tick`` may inject events (an e-stop press, an obstacle)."""
    tree = py_trees.trees.BehaviourTree(root)
    t0, trace, last = skills.world.t, [], None
    tick = 0
    for tick in range(1, max_ticks + 1):
        if on_tick is not None:
            on_tick(tick, root)
        tree.tick()
        tip = tree.tip()
        key = (tip.name if tip else "-", root.status.value)
        if key != last:
            trace.append((round(skills.world.t, 2), key[0], (tip.status.value if tip else "-")))
            last = key
        if root.status != Status.RUNNING:
            break
    skills.world.base.stop()
    return TreeRun(root.status, tick, round(skills.world.t - t0, 2), trace)
