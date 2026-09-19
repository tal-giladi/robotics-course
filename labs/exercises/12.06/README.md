# 12.06 — Nav2's four behavior-tree control nodes

Lesson: [12.06 Nav2 architecture — servers, lifecycle and behavior trees](../../../12-navigation/12.06-nav2-architecture.md)

Nav2's whole navigation policy — "replan at 1 Hz while driving; if planning fails, clear the global
costmap and try again; if that fails too, spin, wait, back up, and give up after six tries" — is not
in any C++ file. It is in one XML file plus the tick semantics of four control nodes. Implement
those four and the policy falls out.

## What to implement (`student.py`)

| Class | Does |
|---|---|
| `PipelineSequence.tick` | ticks the children in order every tick; a RUNNING child stops the tick only the first time, so earlier stages keep being re-ticked |
| `RecoveryNode.tick` | child 0 is the work, child 1 is the recovery; loops inside a single tick, up to `number_of_retries` times |
| `RoundRobin.tick` | one child per SUCCESS, straight to the next on FAILURE; the index survives a parent's shallow reset |
| `RateController.tick` | throttles its child to `hz`, returns RUNNING in between, never cuts a RUNNING child off |

The framework (`Status`, `BTNode`, `ScriptedLeaf`, `Sequence`, `ReactiveFallback`, `run`) is given.
Each docstring spells out the algorithm step by step. Only the standard library is needed.

The one rule to keep in mind is `BTNode.halt_child`, BehaviorTree.CPP v4's reset rule: a child is
**deeply** halted only when it is RUNNING; a child that already returned SUCCESS or FAILURE merely
has its status cleared and keeps its own bookkeeping. Use `halt_child` / `halt_children` wherever
the docstring says to halt something.

## Check

```bash
python course.py check 12.06              # your code
python course.py check 12.06 --solution   # the reference, to see what passing looks like
```

The tests tick hand-built trees (memory, retry counting, round-robin order, throttling rate) and
then run a rebuild of Nav2 Jazzy's real `navigate_to_pose_w_replanning_and_recovery.xml` through
your nodes: three planner activations over three seconds of driving, a controller failure absorbed
by clearing the local costmap, and an unreachable goal that walks the recovery subtree
`clear -> spin -> wait -> back up -> clear -> spin` before aborting.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* `RecoveryNode` and `RoundRobin` both use a `while` loop **inside one tick**. If your recovery
  only runs on the next tick, you have the structure wrong.
* `RoundRobin` advances its index on SUCCESS *and* on FAILURE, but only SUCCESS returns.
* Nothing in `RoundRobin.tick` resets the index. If your recovery subtree clears the costmaps six
  times instead of cycling, you are resetting it somewhere (or calling `halt()` instead of
  `halt_children()`).
* `RateController` must not restart its clock when its own status is SUCCESS or FAILURE — only
  when it is IDLE. Get this wrong and the planner runs at the full 100 Hz loop rate.
