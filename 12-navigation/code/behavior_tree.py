"""12.06 — A tiny behavior-tree engine with Nav2's control nodes, and Nav2's default tree.

    python 12-navigation/code/behavior_tree.py             # three scenarios, tick by tick
    python 12-navigation/code/behavior_tree.py --xml FILE  # run your own BehaviorTree.CPP v4 XML

The engine implements the four nodes that make Nav2's navigation tree work (``PipelineSequence``,
``RecoveryNode``, ``RoundRobin``, ``RateController``) plus the standard BehaviorTree.CPP v4
``Sequence``, ``Fallback`` and ``ReactiveFallback``. The tick logic follows the Jazzy sources
(nav2_behavior_tree 1.3.13 and BehaviorTree.CPP 4.x). The leaves are *fakes*: scripted planner,
controller and recovery servers, so Nav2's real default XML runs on a laptop with no ROS and you
can watch exactly which node is ticked when a plan fails.

A teaching model, not a reimplementation: no typed ports, no SKIPPED status, no subtrees, no
logging protocol, and time advances one ``bt_loop_duration`` per tick instead of with a real clock.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

HERE = Path(__file__).resolve().parent


class Status(str, Enum):
    """BehaviorTree.CPP node status (``IDLE`` means "not ticked since the last reset")."""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


@dataclass
class Context:
    """Everything a tick needs: the blackboard, the clock, and a trace of what was ticked."""

    blackboard: dict[str, object] = field(default_factory=dict)
    now: float = 0.0                       # seconds
    tick_index: int = 0
    trace: list[tuple[int, str, Status]] = field(default_factory=list)

    def record(self, name: str, status: Status) -> Status:
        self.trace.append((self.tick_index, name, status))
        return status

    def leaves_of_tick(self, tick_index: int) -> list[tuple[str, Status]]:
        return [(n, s) for i, n, s in self.trace if i == tick_index]


# --- nodes ---------------------------------------------------------------------------------------
class BTNode:
    """Base class.

    ``tick`` returns a Status. ``halt`` is the *deep* reset: it clears this node's own bookkeeping
    and halts its children. ``halt_child`` follows BehaviorTree.CPP v4: a child is deeply halted
    only when it is RUNNING; a child that already finished merely has its status reset. That one
    rule is why Nav2's RoundRobin remembers which recovery it ran last — it returns SUCCESS, so it
    is never deeply halted, so its child index survives (lesson 12.06).
    """

    def __init__(self, name: str, children: Sequence[BTNode] = ()) -> None:
        self.name = name
        self.children: list[BTNode] = list(children)
        self.status = Status.IDLE

    def tick(self, ctx: Context) -> Status:
        raise NotImplementedError

    def reset_status(self) -> None:
        self.status = Status.IDLE

    def halt(self) -> None:
        self.halt_children()
        self.status = Status.IDLE

    def halt_children(self) -> None:
        for child in self.children:
            self.halt_child(child)

    @staticmethod
    def halt_child(child: BTNode) -> None:
        if child.status is Status.RUNNING:
            child.halt()
        child.reset_status()

    def execute(self, ctx: Context) -> Status:
        self.status = self.tick(ctx)
        return self.status

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.name!r}, {len(self.children)} children)"


class Leaf(BTNode):
    """An action or condition: ``fn(ctx, node)`` returns a Status.

    ``state`` is the leaf's per-activation scratch space (ticks left, which scripted outcome it
    drew). Resetting the status clears it: a finished action node starts a fresh goal next time.
    """

    def __init__(self, name: str, fn: Callable[[Context, Leaf], Status], ports: dict[str, str] | None = None) -> None:
        super().__init__(name)
        self.fn = fn
        self.ports = ports or {}
        self.state: dict[str, object] = {}

    def tick(self, ctx: Context) -> Status:
        return ctx.record(self.name, self.fn(ctx, self))

    def reset_status(self) -> None:
        self.state.clear()
        self.status = Status.IDLE

    def halt(self) -> None:
        self.reset_status()


class Sequence(BTNode):
    """BehaviorTree.CPP v4 ``Sequence`` (with memory): resumes at the child that was RUNNING."""

    def __init__(self, name: str = "Sequence", children: Sequence[BTNode] = ()) -> None:
        super().__init__(name, children)
        self.index = 0

    def halt(self) -> None:
        self.index = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        while self.index < len(self.children):
            status = self.children[self.index].execute(ctx)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.FAILURE:
                self.index = 0
                self.halt_children()
                return Status.FAILURE
            self.index += 1
        self.index = 0
        self.halt_children()
        return Status.SUCCESS


class Fallback(BTNode):
    """BehaviorTree.CPP v4 ``Fallback`` (with memory): the mirror image of Sequence."""

    def __init__(self, name: str = "Fallback", children: Sequence[BTNode] = ()) -> None:
        super().__init__(name, children)
        self.index = 0

    def halt(self) -> None:
        self.index = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        while self.index < len(self.children):
            status = self.children[self.index].execute(ctx)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.index = 0
                self.halt_children()
                return Status.SUCCESS
            self.index += 1
        self.index = 0
        self.halt_children()
        return Status.FAILURE


class ReactiveFallback(BTNode):
    """No memory: every tick starts at the first child, so conditions are re-checked every tick."""

    def tick(self, ctx: Context) -> Status:
        for i, child in enumerate(self.children):
            status = child.execute(ctx)
            if status is Status.RUNNING:
                for j, other in enumerate(self.children):
                    if j != i:
                        self.halt_child(other)
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.halt_children()
                return Status.SUCCESS
        self.halt_children()
        return Status.FAILURE


class PipelineSequence(BTNode):
    """Nav2's PipelineSequence: keeps re-ticking the earlier children while a later one is RUNNING.

    Ticks children in order every tick. FAILURE anywhere halts everything and fails. A child that
    returns RUNNING ends the tick only the first time it does so (``i >= last_child_ticked``);
    after that the tick carries on to the later children. That is what lets
    ``RateController -> ComputePathToPose`` replan while ``FollowPath`` is still driving.
    """

    def __init__(self, name: str = "PipelineSequence", children: Sequence[BTNode] = ()) -> None:
        super().__init__(name, children)
        self.last_child_ticked = 0

    def halt(self) -> None:
        self.last_child_ticked = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        for i, child in enumerate(self.children):
            status = child.execute(ctx)
            if status is Status.FAILURE:
                self.halt_children()
                self.last_child_ticked = 0
                return Status.FAILURE
            if status is Status.RUNNING and i >= self.last_child_ticked:
                self.last_child_ticked = i
                return Status.RUNNING
            # SUCCESS, or an earlier child that is still RUNNING: carry on down the pipeline
        self.halt_children()
        self.last_child_ticked = 0
        return Status.SUCCESS


class RecoveryNode(BTNode):
    """Nav2's RecoveryNode: child 0 is the work, child 1 is the recovery, retried N times.

    SUCCESS only if child 0 succeeds. If child 0 fails and retries remain, child 1 is ticked **in
    the same tick**; if the recovery then succeeds, child 0 is ticked again, also in the same tick.
    """

    def __init__(self, name: str = "RecoveryNode", children: Sequence[BTNode] = (), number_of_retries: int = 1) -> None:
        super().__init__(name, children)
        if children and len(children) != 2:
            raise ValueError(f"RecoveryNode {name!r} must have exactly 2 children")
        self.number_of_retries = number_of_retries
        self.retry_count = 0
        self.index = 0

    def halt(self) -> None:
        self.retry_count = 0
        self.index = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        while self.index < len(self.children) and self.retry_count <= self.number_of_retries:
            status = self.children[self.index].execute(ctx)
            if self.index == 0:
                if status is Status.SUCCESS:
                    self.halt_child(self.children[1])
                    self.halt()
                    return Status.SUCCESS
                if status is Status.RUNNING:
                    return Status.RUNNING
                if self.retry_count < self.number_of_retries:
                    self.halt_child(self.children[0])
                    self.index = 1
                    continue
                self.halt()
                return Status.FAILURE
            # the recovery child
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.halt_child(self.children[1])
                self.retry_count += 1
                self.index = 0
                continue
            self.halt()
            return Status.FAILURE
        self.halt()
        return Status.FAILURE


class RoundRobin(BTNode):
    """Nav2's RoundRobin: one child per SUCCESS, the next child immediately on FAILURE.

    The index advances on every non-RUNNING result and is reset only by a deep ``halt``, which a
    parent performs only while this node is RUNNING. So the recovery subtree really does cycle
    clear costmaps -> spin -> wait -> back up over successive failures.
    """

    def __init__(self, name: str = "RoundRobin", children: Sequence[BTNode] = ()) -> None:
        super().__init__(name, children)
        self.index = 0
        self.failed = 0

    def halt(self) -> None:
        self.index = 0
        self.failed = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        n = len(self.children)
        while self.failed < n:
            status = self.children[self.index].execute(ctx)
            if status is Status.RUNNING:
                return Status.RUNNING
            self.index = (self.index + 1) % n
            if status is Status.SUCCESS:
                self.failed = 0
                self.halt_children()
                return Status.SUCCESS
            self.failed += 1
        self.halt()
        return Status.FAILURE


class RateController(BTNode):
    """Nav2's RateController: ticks its child at most ``hz`` times a second, RUNNING in between.

    Exception: once the child is RUNNING it is ticked every tick until it finishes, so a slow
    planner is never cut off half way.
    """

    def __init__(self, name: str = "RateController", children: Sequence[BTNode] = (), hz: float = 10.0) -> None:
        super().__init__(name, children)
        self.hz = hz
        self.period = 1.0 / hz
        self.start = 0.0
        self.first_time = True

    def halt(self) -> None:
        self.first_time = True
        super().halt()

    def tick(self, ctx: Context) -> Status:
        child = self.children[0]
        if self.status is Status.IDLE:                   # a fresh activation restarts the clock
            self.start = ctx.now
            self.first_time = True
        if self.first_time or child.status is Status.RUNNING or ctx.now - self.start >= self.period:
            self.first_time = False
            status = child.execute(ctx)
            if status is Status.SUCCESS:
                self.start = ctx.now                     # restart the clock when the child finishes
            return status
        return Status.RUNNING


class Inverter(BTNode):
    """SUCCESS <-> FAILURE, RUNNING unchanged."""

    def tick(self, ctx: Context) -> Status:
        status = self.children[0].execute(ctx)
        if status is Status.SUCCESS:
            return Status.FAILURE
        if status is Status.FAILURE:
            return Status.SUCCESS
        return status


CONTROL_NODES: dict[str, Callable[..., BTNode]] = {
    "Sequence": Sequence,
    "Fallback": Fallback,
    "ReactiveFallback": ReactiveFallback,
    "PipelineSequence": PipelineSequence,
    "RecoveryNode": RecoveryNode,
    "RoundRobin": RoundRobin,
    "RateController": RateController,
    "Inverter": Inverter,
}


# --- XML -----------------------------------------------------------------------------------------
def build(element: ET.Element, leaves: dict[str, Callable[[Context, Leaf], Status]]) -> BTNode:
    """Turn one BehaviorTree.CPP v4 XML element into a node, recursively."""
    tag = element.tag
    ports = dict(element.attrib)
    name = ports.pop("name", tag)
    if tag in CONTROL_NODES:
        children = [build(child, leaves) for child in element]
        if tag == "RecoveryNode":
            return RecoveryNode(name, children, number_of_retries=int(ports.get("number_of_retries", 1)))
        if tag == "RateController":
            return RateController(name, children, hz=float(ports.get("hz", 10.0)))
        return CONTROL_NODES[tag](name, children)
    if tag not in leaves:
        raise KeyError(f"no fake implementation registered for leaf <{tag}>; known: {sorted(leaves)}")
    return Leaf(name, leaves[tag], ports)


def load_tree(xml_text: str, leaves: dict[str, Callable[[Context, Leaf], Status]]) -> BTNode:
    """Parse ``<root><BehaviorTree ID="MainTree">...`` and build the main tree."""
    root = ET.fromstring(xml_text)
    main_id = root.attrib.get("main_tree_to_execute", "MainTree")
    trees = {bt.attrib.get("ID", "MainTree"): bt for bt in root.iter("BehaviorTree")}
    body = list(trees[main_id])
    if len(body) != 1:
        raise ValueError("a BehaviorTree must have exactly one root child")
    return build(body[0], leaves)


# --- the fake Nav2 servers ------------------------------------------------------------------------
@dataclass
class FakeNav2:
    """Scripted planner / controller / behavior servers, so the real tree can run offline.

    ``plan_outcomes`` and ``follow_outcomes`` are consumed one per *activation* (not per tick):
    True means the server succeeds, False means it fails and sets an error code. The ``*_ticks``
    fields are how long an activation stays RUNNING, in ticks of 10 ms.
    """

    plan_outcomes: list[bool] = field(default_factory=lambda: [True])
    follow_outcomes: list[bool] = field(default_factory=lambda: [True])
    follow_ticks: int = 300                       # 300 * 10 ms = 3 s of driving
    follow_fail_ticks: int = 150                  # how long a doomed FollowPath drives before failing
    plan_ticks: int = 1
    clear_ticks: int = 2                          # a costmap-clearing service round trip
    spin_ticks: int = 120
    backup_ticks: int = 200
    goal_was_updated: bool = False
    counters: dict[str, int] = field(default_factory=dict)

    def _next(self, outcomes: list[bool], name: str) -> bool:
        i = self.counters.get(name, 0)
        self.counters[name] = i + 1
        return outcomes[min(i, len(outcomes) - 1)]

    def _busy(self, node: Leaf, ticks: int) -> bool:
        """True while this activation is still RUNNING."""
        left = int(node.state.get("ticks_left", ticks))  # type: ignore[arg-type]
        if left > 1:
            node.state["ticks_left"] = left - 1
            return True
        node.state["ticks_left"] = 0
        return False

    # --- leaves ---------------------------------------------------------------------------------
    def compute_path_to_pose(self, ctx: Context, node: Leaf) -> Status:
        if self._busy(node, self.plan_ticks):
            return Status.RUNNING
        if "outcome" not in node.state:
            node.state["outcome"] = self._next(self.plan_outcomes, "plan")
        if node.state["outcome"]:
            ctx.blackboard["path"] = f"path@tick{ctx.tick_index}"
            ctx.blackboard["compute_path_error_code"] = 0
            return Status.SUCCESS
        ctx.blackboard["compute_path_error_code"] = 208      # ComputePathToPose NO_VALID_PATH
        return Status.FAILURE

    def follow_path(self, ctx: Context, node: Leaf) -> Status:
        if "outcome" not in node.state:
            node.state["outcome"] = self._next(self.follow_outcomes, "follow")
        ticks = self.follow_ticks if node.state["outcome"] else self.follow_fail_ticks
        if self._busy(node, ticks):
            return Status.RUNNING
        if not node.state["outcome"]:
            ctx.blackboard["follow_path_error_code"] = 105   # FollowPath FAILED_TO_MAKE_PROGRESS
            return Status.FAILURE
        ctx.blackboard["follow_path_error_code"] = 0
        return Status.SUCCESS

    def clear_costmap(self, ctx: Context, node: Leaf) -> Status:
        return Status.RUNNING if self._busy(node, self.clear_ticks) else Status.SUCCESS

    def spin(self, ctx: Context, node: Leaf) -> Status:
        return Status.RUNNING if self._busy(node, self.spin_ticks) else Status.SUCCESS

    def wait(self, ctx: Context, node: Leaf) -> Status:
        seconds = float(node.ports.get("wait_duration", 5.0))
        return Status.RUNNING if self._busy(node, max(1, int(seconds / 0.01))) else Status.SUCCESS

    def back_up(self, ctx: Context, node: Leaf) -> Status:
        return Status.RUNNING if self._busy(node, self.backup_ticks) else Status.SUCCESS

    def goal_updated(self, ctx: Context, node: Leaf) -> Status:
        return Status.SUCCESS if self.goal_was_updated else Status.FAILURE

    def would_planner_recovery_help(self, ctx: Context, node: Leaf) -> Status:
        return Status.SUCCESS if ctx.blackboard.get("compute_path_error_code") == 208 else Status.FAILURE

    def would_controller_recovery_help(self, ctx: Context, node: Leaf) -> Status:
        return Status.SUCCESS if ctx.blackboard.get("follow_path_error_code") in (105, 106) else Status.FAILURE

    def selector(self, ctx: Context, node: Leaf) -> Status:
        return Status.SUCCESS

    def leaves(self) -> dict[str, Callable[[Context, Leaf], Status]]:
        return {
            "ComputePathToPose": self.compute_path_to_pose,
            "FollowPath": self.follow_path,
            "ClearEntireCostmap": self.clear_costmap,
            "Spin": self.spin,
            "Wait": self.wait,
            "BackUp": self.back_up,
            "GoalUpdated": self.goal_updated,
            "WouldAPlannerRecoveryHelp": self.would_planner_recovery_help,
            "WouldAControllerRecoveryHelp": self.would_controller_recovery_help,
            "PlannerSelector": self.selector,
            "ControllerSelector": self.selector,
        }


# --- running -------------------------------------------------------------------------------------
def run(tree: BTNode, ctx: Context, max_ticks: int = 4000, bt_loop_duration: float = 0.01) -> Status:
    """Tick ``tree`` at ``1 / bt_loop_duration`` Hz until it returns SUCCESS or FAILURE."""
    status = Status.RUNNING
    for _ in range(max_ticks):
        status = tree.execute(ctx)
        if status is not Status.RUNNING:
            return status
        ctx.tick_index += 1
        ctx.now += bt_loop_duration
    return status


def summarize(ctx: Context, bt_loop_duration: float = 0.01) -> Iterator[str]:
    """One line per tick in which a leaf was ticked, collapsing identical consecutive lines."""
    previous: str | None = None
    repeats = 0
    for tick in sorted({i for i, _, _ in ctx.trace}):
        line = "  ".join(f"{n}:{s.value[:4]}" for n, s in ctx.leaves_of_tick(tick))
        if line == previous:
            repeats += 1
            continue
        if repeats:
            yield f"           ... {repeats} more identical ticks"
            repeats = 0
        previous = line
        yield f"  t={tick * bt_loop_duration:6.2f}s  {line}"
    if repeats:
        yield f"           ... {repeats} more identical ticks"


DEFAULT_TREE = HERE / "nav2_trees" / "navigate_to_pose_w_replanning_and_recovery.xml"


def scenarios() -> list[tuple[str, FakeNav2]]:
    """The three runs printed by ``main``."""
    return [
        ("A. nominal: replan at 1 Hz while FollowPath drives for 3 s",
         FakeNav2(follow_ticks=300)),
        ("B. the controller fails once (FAILED_TO_MAKE_PROGRESS), then succeeds",
         FakeNav2(follow_outcomes=[False, True], follow_ticks=200, follow_fail_ticks=150)),
        ("C. the planner never finds a path (goal inside a wall): all six retries",
         FakeNav2(plan_outcomes=[False])),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a Nav2 behavior tree against fake servers.")
    parser.add_argument("--xml", type=Path, default=DEFAULT_TREE, help="behavior tree XML to run")
    parser.add_argument("--max-ticks", type=int, default=4000)
    args = parser.parse_args()

    xml_text = args.xml.read_text(encoding="utf-8")
    print(f"tree: {args.xml.name}   (bt_loop_duration 10 ms, so one tick = 0.01 s)\n")
    for title, nav2 in scenarios():
        tree = load_tree(xml_text, nav2.leaves())
        ctx = Context()
        result = run(tree, ctx, max_ticks=args.max_ticks)
        print(title)
        for line in summarize(ctx):
            print(line)
        recoveries = sum(1 for _, n, s in ctx.trace if n in {"Spin", "Wait", "BackUp"} and s is Status.SUCCESS)
        print(f"  -> {result.value} after {ctx.tick_index + 1} ticks ({ctx.now:.2f} s), "
              f"{nav2.counters.get('plan', 0)} planner activations, "
              f"{recoveries} motion recoveries\n")


if __name__ == "__main__":
    main()
