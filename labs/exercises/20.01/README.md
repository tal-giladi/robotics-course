# 20.01 — The integration review, as code

Lesson: [20.01 Final robot system architecture](../../../20-final-robot/20.01-final-architecture.md)

Two nodes that agree on the topic name and the message type can still exchange nothing at all.
These two functions are the review you run *before* the integration week, against the interface
table in [`20-final-robot/code/system_contracts.py`](../../../20-final-robot/code/system_contracts.py).

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `qos_incompatibility(pub, sub)` | why this publisher/subscriber pair exchanges no data, or `None` |
| `interface_rules(contract)` | the names of every rule one interface breaks, sorted |

The docstrings give the exact rules and the order to check them in.

## Check

```bash
python course.py check 20.01              # your code
python course.py check 20.01 --solution   # the reference, to see what passing looks like
```

The tests run your rules over the real karmel interface table and over the deliberately broken
variant, and require that you find the same four faults the course's own checker finds.

## Hints

* DDS matches **request against offered**: a subscriber may ask for the same or *less* than the
  publisher promises. `reliable` is more than `best_effort`; `transient_local` is more than
  `volatile`; a shorter deadline is more than a longer one, and more than none at all.
* Check reliability before durability before deadline — a profile can break more than one rule,
  and the tests pin which one you report.
* `interface_rules` returns *rule names*, not sentences, so build a `set` and `sorted()` it.
* A subscriber whose `rate_hz` is `None` or `0.0` is event-driven (`/goal_pose`, an action goal);
  it has no rate requirement and must not produce a `rate` finding.
* `KNOWN_FRAMES` lives in `system_contracts`. `frame_id=None` (`/tf`, `/joint_states`) is legal —
  those messages carry their frames inside.
