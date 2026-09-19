"""19.05 — reference solution: a minimal behavior-tree engine (tick, memory, halting)."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum


class Status(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"
    INVALID = "INVALID"  # never ticked, or halted by something above


class Behaviour:
    """A node. ``status`` starts INVALID and holds the result of the last tick.

    Subclasses override ``update`` (required), and ``initialise`` / ``terminate`` when they have
    something to set up or to cancel.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.status = Status.INVALID
        self.children: list[Behaviour] = []

    # --- the three hooks a subclass may override ---
    def initialise(self) -> None:
        """Called on *entry*: the first tick, and again after the node stopped running."""

    def update(self) -> Status:
        """One tick of work. Must return SUCCESS, FAILURE or RUNNING — never INVALID."""
        raise NotImplementedError("update")

    def terminate(self, new_status: Status) -> None:
        """Called when the node stops running: it finished, or something above halted it
        (``new_status is Status.INVALID``). This is where an action leaf cancels its goal."""

    # --- the protocol ---
    def tick(self) -> Status:
        if self.status is not Status.RUNNING:
            self.initialise()
        new = self.update()
        if new is not Status.RUNNING:
            self.terminate(new)
        self.status = new
        return new

    def stop(self, new_status: Status = Status.INVALID) -> None:
        """Halt this node and everything under it.

        If this node was RUNNING, call ``terminate(new_status)`` — this is the halt path that
        cancels a goal on a real robot. Then set ``self.status`` and stop every child the same
        way, whatever their status.
        """
        if self.status is Status.RUNNING:
            self.terminate(new_status)
        self.status = new_status
        for child in self.children:
            child.stop(new_status)


class Composite(Behaviour):
    def __init__(self, name: str, children: list[Behaviour], memory: bool = True) -> None:
        super().__init__(name)
        self.children = list(children)
        self.memory = memory
        self.current = 0

    def initialise(self) -> None:
        self.current = 0

    def halt_after(self, index: int) -> None:
        """Stop every child after ``index`` that is still RUNNING (it has been abandoned)."""
        for child in self.children[index + 1:]:
            if child.status is Status.RUNNING:
                child.stop(Status.INVALID)


class Sequence(Composite):
    """"and, in order": SUCCESS when every child succeeded, FAILURE at the first that fails."""

    def update(self) -> Status:
        """Tick children from ``self.current`` when ``memory`` is true, else from 0.

        * a child returns RUNNING -> remember its index in ``self.current`` and return RUNNING;
        * a child returns FAILURE -> remember its index, ``halt_after`` it (a later child may
          still be RUNNING from a previous tick — it has just been abandoned) and return FAILURE;
        * every child succeeded -> SUCCESS.
        """
        start = self.current if self.memory else 0
        for i in range(start, len(self.children)):
            status = self.children[i].tick()
            if status is Status.RUNNING:
                self.current = i
                return Status.RUNNING
            if status is Status.FAILURE:
                self.current = i
                self.halt_after(i)
                return Status.FAILURE
        self.current = len(self.children) - 1
        return Status.SUCCESS


class Selector(Composite):
    """"try these in order": SUCCESS at the first child that succeeds, FAILURE when all fail."""

    def update(self) -> Status:
        """The mirror image of ``Sequence.update``: SUCCESS short-circuits, FAILURE continues."""
        start = self.current if self.memory else 0
        for i in range(start, len(self.children)):
            status = self.children[i].tick()
            if status is Status.RUNNING:
                self.current = i
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.current = i
                self.halt_after(i)
                return Status.SUCCESS
        self.current = len(self.children) - 1
        return Status.FAILURE


class Decorator(Behaviour):
    def __init__(self, name: str, child: Behaviour) -> None:
        super().__init__(name)
        self.child = child
        self.children = [child]


class Retry(Decorator):
    """Re-enter the child after a failure, up to ``num_failures`` attempts in total."""

    def __init__(self, name: str, child: Behaviour, num_failures: int) -> None:
        super().__init__(name, child)
        self.num_failures = num_failures
        self.failures = 0

    def initialise(self) -> None:
        self.failures = 0

    def update(self) -> Status:
        """Tick the child.

        * SUCCESS or RUNNING -> return it unchanged;
        * FAILURE -> count it. If the count has reached ``num_failures``, return FAILURE.
          Otherwise ``self.child.stop(Status.INVALID)`` — so the next tick *re-enters* it and
          its ``initialise`` runs again — and return RUNNING (one attempt per tick).
        """
        status = self.child.tick()
        if status is not Status.FAILURE:
            return status
        self.failures += 1
        if self.failures >= self.num_failures:
            return Status.FAILURE
        self.child.stop(Status.INVALID)   # so the next tick re-enters it
        return Status.RUNNING


class Inverter(Decorator):
    """SUCCESS <-> FAILURE; RUNNING passes through."""

    def update(self) -> Status:
        status = self.child.tick()
        if status is Status.SUCCESS:
            return Status.FAILURE
        if status is Status.FAILURE:
            return Status.SUCCESS
        return status


class Succeeder(Decorator):
    """FAILURE becomes SUCCESS; RUNNING passes through. "Try it, but do not block the task."."""

    def update(self) -> Status:
        status = self.child.tick()
        return Status.SUCCESS if status is Status.FAILURE else status


class Condition(Behaviour):
    """A guard: SUCCESS while the predicate holds, FAILURE otherwise. Never RUNNING, no effects."""

    def __init__(self, name: str, predicate: Callable[[], bool]) -> None:
        super().__init__(name)
        self.predicate = predicate

    def update(self) -> Status:
        return Status.SUCCESS if self.predicate() else Status.FAILURE
