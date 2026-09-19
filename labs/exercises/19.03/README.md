# 19.03 — Build the agent loop

Lesson: [19.03 Tool calling — an LLM operating the simulated robot](../../../19-llm-robot-agents/19.03-tool-calling-sim-robot.md)

A model that proposes skill calls and a gateway that executes them are both given. What is missing
is the loop between them — and, more to the point, the four budgets that make it terminate and the
one call that leaves the robot stopped whatever happens.

## What to implement (`student.py`)

| Name | Is |
|---|---|
| `Budget` | already written: turns, tool calls, robot seconds, consecutive errors |
| `AgentRun` | already written: outcome, final text, transcript, tool log, turn count |
| `run_agent(llm, skills, task, budget, system_prompt)` | **yours** — the loop |

The docstring of `run_agent` is the specification, step by step, including the exact outcome
strings.

## Check

```bash
python course.py check 19.03              # your code
python course.py check 19.03 --solution   # the reference, to see what passing looks like
```

The tests run offline against a scripted "model" and against the course's deterministic mock
planner, so they pin behaviour rather than wording: the full fetch task ends with the bottle on
the `kitchen_table` in 14 turns and 13 calls at seed 0; all of one turn's results come back in
exactly **one** message, in order; a failed call is still reported with `is_error` and its hint;
the model's own call id is used as the idempotency key, so the same id twice consumes no extra
robot time; each of the four budgets produces its own outcome string; a success in between two
pairs of failures is *not* three errors in a row; `refusal` and `max_tokens` are outcomes, not
completions; and `base.stop()` is the last thing that happens before the function returns.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Write the exit path once as a local `finish(outcome, text, turn)` that calls
  `skills.world.base.stop()` and builds the `AgentRun`, then `return finish(...)` everywhere. If
  the stop is written out at each `return`, one of them will eventually be missing.
* `for turn in range(1, b.max_llm_turns + 1)` — falling out of the `for` **is** the `turn_limit`
  case, so it needs no counter of its own.
* Get `tools = skills.tool_definitions()` once, before the loop. Rebuilding it per turn is both
  wasteful and the thing that silently destroys prompt caching.
* The budget check for tool calls has to happen in two places: inside the per-call loop (so the
  remaining calls of that turn get a `BUDGET_EXCEEDED` result instead of executing) and after the
  turn (so the run ends).
* Robot time is `skills.world.t`, which only advances while the robot does something. Measure it
  as a *delta* from the start of the run, not as an absolute.
* `result.for_llm()` already produces the compact JSON string that goes into a `ToolResult`.
