# 19.02 — Generate the schema, then enforce it

Lesson: [19.02 Designing the robot's skill API — the tools an LLM may call](../../../19-llm-robot-agents/19.02-robot-skill-api.md)

A parameter declaration goes in; the JSON Schema the model reads and the gateway enforces comes
out. Then a call's arguments go in and every reason to refuse them comes out — before anything
moves. These two functions are the whole difference between "the prompt asked the model to be
careful" and "the robot cannot be asked to do that".

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `param_schema(name, type, description, unit, minimum, maximum, enum, pattern, default)` | one parameter → its JSON Schema fragment, with the description **generated** from the constraints |
| `validate_arguments(schema, args)` | a call → the list of reasons to reject it (`[]` means legal) |

The docstrings give the exact message formats the tests expect.

## Check

```bash
python course.py check 19.02              # your code
python course.py check 19.02 --solution   # the reference, to see what passing looks like
```

The tests pin the traps that matter on a real robot: an `int` is a valid `number` but `True` is
not; `NaN` and `inf` are not finite; a type error must not also produce a range error (comparing
`"30 s"` with `1.0` raises); an unknown key is reported once and never type-checked; all three
mistakes in one call are reported together, because the model gets one correction per turn; and a
range error names its unit — `timeout_s: 900 s is above the maximum 300.0 s`, not `900 > 300`.

The last test runs the same five calls through your validator and through the course's
(`19-llm-robot-agents/code/robot_agent/skills.py`) and requires the same verdicts.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Build the description by appending to a local string as you add keys, in the order
  unit → range → default. The `Range:` sentence appears when **either** bound is given, so
  `Range: [0.0, None].` is correct for a half-open range — an open end the model can see beats a
  constraint it has to guess.
* In `validate_arguments`, do the whole-object checks first (`required`, then unknown keys), then
  loop over the arguments. Skip any key that has no entry in `properties`: you already reported it.
* After you append a type error for a parameter, `continue` — the remaining checks on that
  parameter are all comparisons, and comparisons with the wrong type either raise or lie.
* `isinstance(True, int)` is `True` in Python. Test `isinstance(value, bool)` *first* for both the
  `number` and the `integer` cases.
* Use `re.fullmatch`, not `re.match`: `re.match(r"[a-z]+-\d+", "bottle-1 and a knife")` succeeds.
