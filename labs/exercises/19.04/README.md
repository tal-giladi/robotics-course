# 19.04 — Build the state-machine engine

Lesson: [19.04 State machines for robot tasks](../../../19-llm-robot-agents/19.04-state-machines.md)

The fetch task is not the exercise — the engine under it is. Three methods turn a dict of
transitions into something you can trust with a moving robot: one that registers states, one that
refuses to run a broken machine, and one that runs it and leaves a trace you can read afterwards.

## What to implement (`student.py`)

| Method | Does |
|---|---|
| `StateMachine.add_state(name, state, transitions)` | register a state, set the start, share the trace with sub-machines |
| `StateMachine.validate()` | refuse unmapped outcomes, undeclared transitions, targets that do not exist, unreachable states — recursively |
| `StateMachine.execute(bb)` | validate, then run: preempt → execute → check the outcome → trace → transition |

`State`, `CallbackState`, `TraceEntry`, `InvalidMachine` and the dataclass fields are given. The
docstrings carry the exact message formats the tests match on.

## Check

```bash
python course.py check 19.04              # your code
python course.py check 19.04 --solution   # the reference, to see what passing looks like
```

The tests are about structure, not about robots: the states are plain callbacks and the whole
suite runs in well under a second. They pin the four validation failures with their messages, a
self-loop as legal, recursive validation of a sub-machine, a shared trace across a hierarchy,
timestamps taken from the injected `clock`, preemption that aborts *before* the next state and
propagates out of a sub-machine, a preemption outcome the machine never declared as a bug, and
the transition bound that turns a runaway loop into a `RuntimeError` with 25 trace entries.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `validate` is three graph checks. Do them per state in one pass (outcomes vs transitions, both
  directions, and targets), then do reachability once at the end with a stack starting from
  `start`.
* Reachability only follows targets that are *states*; a target that is one of the machine's own
  outcomes is an exit, not an edge.
* In `execute`, check preemption **before** executing the state, not after. Checking after means
  one more activity runs than the operator asked for.
* The preemption trace entry is the odd one out: its `outcome` and its `next` are the same value,
  because the machine is leaving rather than transitioning.
* `TraceEntry.t` is `round(self.clock(), 2)` on the normal path. Inject the clock — never call
  `time.time()` inside the engine, or the machine cannot be tested and will not work under
  simulated time (`use_sim_time`, in ROS terms).
* Let the `for _ in range(self.max_transitions)` loop end naturally and raise after it; a manual
  counter is one more thing to get wrong.
