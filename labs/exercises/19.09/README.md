# 19.09 — The safety layer's rules

Lesson: [19.09 Safety boundaries for LLM-controlled robots](../../../19-llm-robot-agents/19.09-agent-safety-boundaries.md)

Six deterministic rules between the agent and the robot. No model, no prompt, no explanation —
just a `Decision`. The tests check them one at a time *and* in the order they must run.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `zone_violated` | the first keep-out zone the **route** would enter |
| `budget_exceeded` | metres, seconds, manipulations, calls per skill |
| `goal_lock_violated` | would this `place` put the object somewhere the user did not ask for |
| `check` | the six rules in `RULES` order, first denial wins |
| `visible_tools` | what the model is even shown |

`Decision`, `Zone`, `Budget` and `Context` are given, with the exact codes and messages in the
docstrings.

## Check

```bash
python course.py check 19.09              # your code
python course.py check 19.09 --solution   # the reference
```

The last test is the one that matters: it arms each rule **alone** and asserts that it denies its
own attack. Defence in depth is only depth if every layer is load-bearing by itself; a suite that
only ever runs the full stack cannot tell you that the geofence broke three commits ago.

## Hints

* `check` is pure. It takes no robot and returns no side effects, which is why it can be called
  before the call, logged, replayed and unit-tested exhaustively.
* The geofence checks the **path**, then the destination. A rule that only checks the destination
  is a rule an attacker routes around, and so does ordinary A*.
* `goal_lock_violated` must not look at anything the robot read. The comparison is between the
  call and the user's request, and there is no third input.
* The mode check comes before the geofence for a reason worth being able to state: a skill the
  mode forbids should be refused even when the geometry happens to be fine, so the denial message
  names the real reason.
* `budget_exceeded` returns a *string*, so the denial can tell the model which limit it hit. "No"
  with no number is a denial the agent will keep testing.
