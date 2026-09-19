# 20.03 — Bringup order and the one state everything else reads

Lesson: [20.03 Software integration — bringup, health monitoring and diagnostics](../../../20-final-robot/20.03-software-integration-bringup.md)

"Is the robot up?" has to be a value, not a feeling. Three functions produce it: the order the
units start in, what a component's silence means, and the single state that gates every mission.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `start_waves(requires)` | the dependency graph layered into parallel waves, sorted |
| `component_level(reported, last_stamp_s, now_s, period_s)` | the level to use now, staleness included |
| `robot_state(levels, criticality, ...)` | INIT / READY / DEGRADED / FAULT / ESTOP |

## Check

```bash
python course.py check 20.03
python course.py check 20.03 --solution
```

The tests run your functions over karmel's real bringup graph and compare the result with
[`20-final-robot/code/bringup_health.py`](../../../20-final-robot/code/bringup_health.py).

## Hints

* `start_waves` is Kahn's algorithm taken a whole layer at a time: repeatedly take every
  remaining unit whose dependencies are already done. When a pass finds nothing, what is left is
  a cycle — that is your `ValueError`, and it beats a bringup that hangs with no explanation.
* Sort every wave. An unsorted set gives a different answer per run and makes diffs useless.
* `component_level` is where the real bug lives: a crashed node's last message said `"OK"`.
  Believe the clock, not the message.
* In `robot_state` the order of the tests *is* the policy. Write the five branches in the order
  the docstring gives and do not merge them.
* `"optional"` components never change the state. A robot without its LLM bridge is a robot that
  takes no new missions, not a robot in a fault.
