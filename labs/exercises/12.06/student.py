"""12.06 — Nav2's four behavior-tree control nodes.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.06``.
Only the standard library is needed.

The framework below is given. You implement the tick logic of the four nodes that Nav2 adds to
BehaviorTree.CPP and that its default navigation tree is built from:

    PipelineSequence   RecoveryNode   RoundRobin   RateController

Two rules of the framework matter for all of them:

1. ``execute()`` stores the returned status on the node. A node's ``status`` therefore tells you
   what it did last time: IDLE (never ticked, or deeply halted), RUNNING, SUCCESS or FAILURE.
2. ``halt_child(child)`` is BehaviorTree.CPP v4's reset rule: a child is **deeply** halted (its
   ``halt()``, which clears its own bookkeeping) only if it is RUNNING. A child that already
   finished merely gets its status cleared and keeps its internal state. Use ``halt_child`` /
   ``halt_children`` wherever Nav2 calls ``haltChild`` / ``haltChildren``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from enum import Enum


class Status(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class Context:
    """The clock and the tick trace. One tick of the tree = one ``bt_loop_duration``."""

    def __init__(self, now: float = 0.0) -> None:
        self.now = now
        self.tick_index = 0
        self.trace: list[tuple[int, str, Status]] = []

    def record(self, name: str, status: Status) -> Status:
        self.trace.append((self.tick_index, name, status))
        return status

    def names(self) -> list[str]:
        return [name for _, name, _ in self.trace]


class BTNode:
    """Base class: ``tick`` does the work, ``execute`` remembers the result."""

    def __init__(self, name: str, children: Sequence[BTNode] = ()) -> None:
        self.name = name
        self.children: list[BTNode] = list(children)
        self.status = Status.IDLE

    def tick(self, ctx: Context) -> Status:
        raise NotImplementedError

    def execute(self, ctx: Context) -> Status:
        self.status = self.tick(ctx)
        return self.status

    def reset_status(self) -> None:
        self.status = Status.IDLE

    def halt(self) -> None:
        """Deep reset: this node's bookkeeping plus its children."""
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


class ScriptedLeaf(BTNode):
    """A fake action/condition: returns ``results`` one per tick, repeating the last forever."""

    def __init__(self, name: str, results: Sequence[Status]) -> None:
        super().__init__(name)
        self.results = list(results)
        self.ticks = 0

    def tick(self, ctx: Context) -> Status:
        result = self.results[min(self.ticks, len(self.results) - 1)]
        self.ticks += 1
        return ctx.record(self.name, result)

    def halt(self) -> None:
        self.status = Status.IDLE


class CallbackLeaf(BTNode):
    """A leaf driven by a function ``fn(ctx, node) -> Status`` (used by the checker's fake Nav2)."""

    def __init__(self, name: str, fn: Callable[[Context, CallbackLeaf], Status]) -> None:
        super().__init__(name)
        self.fn = fn
        self.state: dict[str, object] = {}

    def tick(self, ctx: Context) -> Status:
        return ctx.record(self.name, self.fn(ctx, self))

    def reset_status(self) -> None:
        self.state.clear()
        self.status = Status.IDLE

    def halt(self) -> None:
        self.reset_status()


class Sequence(BTNode):
    """Given, for reference: BehaviorTree.CPP v4 Sequence (with memory)."""

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


class ReactiveFallback(BTNode):
    """Given, for reference: no memory — every tick starts at the first child again."""

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


# ==================================================================================================
#  Your turn
# ==================================================================================================
class PipelineSequence(BTNode):
    """Nav2's PipelineSequence.

    Every tick, walk the children in order:

    * FAILURE from any child -> ``halt_children()``, reset ``last_child_ticked`` to 0, return FAILURE.
    * SUCCESS -> carry on to the next child.
    * RUNNING -> if this child's index is ``>= last_child_ticked``, remember the index and return
      RUNNING (the tick stops here). Otherwise carry on to the next child: an earlier stage of the
      pipeline is allowed to run while a later one already started.
    * Every child returned SUCCESS -> ``halt_children()``, reset ``last_child_ticked``, return SUCCESS.

    This is what makes ``<PipelineSequence><RateController><ComputePathToPose/></RateController>
    <FollowPath/></PipelineSequence>`` replan while the robot is still driving.
    """

    def __init__(self, name: str = "PipelineSequence", children: Sequence[BTNode] = ()) -> None:
        super().__init__(name, children)
        self.last_child_ticked = 0

    def halt(self) -> None:
        self.last_child_ticked = 0
        super().halt()

    def tick(self, ctx: Context) -> Status:
        # TODO(student)
        raise NotImplementedError("PipelineSequence.tick")


class RecoveryNode(BTNode):
    """Nav2's RecoveryNode: exactly two children — child 0 is the work, child 1 is the recovery.

    Loop **inside one tick**, while ``index < 2`` and ``retry_count <= number_of_retries``:

    * ticking child 0:
        - SUCCESS -> ``halt_child(children[1])``, ``halt()``, return SUCCESS.
        - RUNNING -> return RUNNING (keep ``index`` at 0, keep the retry count).
        - FAILURE -> if ``retry_count < number_of_retries``: ``halt_child(children[0])``, set
          ``index = 1`` and continue the loop (so the recovery runs in this same tick).
          Otherwise ``halt()`` and return FAILURE.
    * ticking child 1:
        - RUNNING -> return RUNNING.
        - SUCCESS -> ``halt_child(children[1])``, ``retry_count += 1``, ``index = 0``, continue
          the loop (child 0 is tried again in this same tick).
        - FAILURE -> ``halt()``, return FAILURE: if the recovery itself fails, nothing will help.

    If the loop ends, ``halt()`` and return FAILURE.
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
        # TODO(student)
        raise NotImplementedError("RecoveryNode.tick")


class RoundRobin(BTNode):
    """Nav2's RoundRobin: hand out one child per SUCCESS, skip to the next one on FAILURE.

    Loop **inside one tick** while ``failed < len(children)``:

    * tick ``children[index]``.
    * RUNNING -> return RUNNING (do not move the index).
    * otherwise advance ``index`` to ``(index + 1) % len(children)`` **first**, then:
        - SUCCESS -> ``failed = 0``, ``halt_children()``, return SUCCESS.
        - FAILURE -> ``failed += 1`` and keep looping (the next recovery is tried immediately).

    If every child failed: ``halt()`` and return FAILURE.

    Note what is NOT here: nothing resets ``index`` on SUCCESS. Only a deep ``halt`` does, and a
    parent only halts this node deeply while it is RUNNING. That is why the recovery subtree
    cycles clear-costmaps -> spin -> wait -> back up across successive navigation failures.
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
        # TODO(student)
        raise NotImplementedError("RoundRobin.tick")


class RateController(BTNode):
    """Nav2's RateController: throttle the single child to ``hz``, return RUNNING in between.

    * If ``self.status`` is IDLE this is a fresh activation: set ``self.start = ctx.now`` and
      ``self.first_time = True``. (A finished node keeps status SUCCESS/FAILURE, not IDLE, so it
      does NOT restart its clock — that is what makes the throttling work at all.)
    * Tick the child when ``first_time`` is set, OR the child's own status is RUNNING (never cut a
      running action off), OR ``ctx.now - self.start >= self.period``. Then clear ``first_time``.
      If the child returns SUCCESS, restart the clock (``self.start = ctx.now``). Return the
      child's status either way.
    * Otherwise do not tick the child: return RUNNING.
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
        # TODO(student)
        raise NotImplementedError("RateController.tick")


def run(tree: BTNode, ctx: Context, max_ticks: int = 4000, bt_loop_duration: float = 0.01) -> Status:
    """Given: tick ``tree`` until it finishes, advancing the clock one loop period per tick."""
    status = Status.RUNNING
    for _ in range(max_ticks):
        status = tree.execute(ctx)
        if status is not Status.RUNNING:
            return status
        ctx.tick_index += 1
        ctx.now += bt_loop_duration
    return status
