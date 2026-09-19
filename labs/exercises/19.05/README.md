# 19.05 — Build the behavior-tree tick engine

Lesson: [19.05 Behavior trees](../../../19-llm-robot-agents/19.05-behavior-trees.md)

`py_trees` already works. Writing the engine once is how you stop guessing what `memory` does and
why a guard that is never re-asked is the most expensive bug in the module. There is no robot in
this file — it is forty lines of control flow that decide whether a real one stops.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `Behaviour.tick()` | the protocol: enter (`initialise`) → `update` → finish (`terminate`) |
| `Behaviour.stop(new_status)` | the halt path: terminate if RUNNING, then halt every child |
| `Sequence.update()` | "and, in order", honouring `memory` |
| `Selector.update()` | "try these in order", honouring `memory` |
| `Retry.update()` | re-enter the child after a failure, up to a budget |
| `Inverter.update()`, `Succeeder.update()` | the two one-line decorators |

`Status`, `Composite.halt_after`, `Decorator` and `Condition` are given.

## Check

```bash
python course.py check 19.05              # your code
python course.py check 19.05 --solution   # the reference, to see what passing looks like
```

The tests journal every `initialise`, `update` and `terminate`, so they pin the semantics rather
than the outcomes: `initialise` runs once per *entry*, not per tick; a finished node is re-entered
on the next tick; `stop` terminates only a node that was RUNNING, and cascades; a `memory=False`
sequence re-asks its guard on every tick and **halts** the work to its right when that guard
fails; a `memory=True` sequence asks its guard exactly once (the bug, made visible); a
`memory=False` selector interrupts a lower-priority running child when a higher-priority one
succeeds; `Retry` re-enters its child on each attempt and resets its counter on re-entry.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `tick` is four lines in the order given in the docstring. The `if self.status is not
  Status.RUNNING: self.initialise()` line is the whole difference between "resume" and "restart".
* `Sequence.update` and `Selector.update` are the same loop with SUCCESS and FAILURE swapped.
  Write one, then copy it and swap two names.
* `halt_after(i)` exists because of one case: without memory, an *earlier* child can end the
  composite while a *later* child is still RUNNING from the previous tick. That later child is
  now abandoned and must be told — on a robot, that call is what cancels a navigation goal.
* In `Retry`, `self.child.stop(Status.INVALID)` after a counted failure is what makes the next
  tick an entry. Without it the child keeps its FAILURE status and never re-initialises.
* Returning RUNNING from `Retry` after a failure means one attempt per tick. That is deliberate:
  it keeps the guards above in the loop instead of burning the whole retry budget inside one tick.
