# 19.06 — Write the plan validator

Lesson: [19.06 Task planning and decomposition — LLM plans, validated execution](../../../19-llm-robot-agents/19.06-task-planning-decomposition.md)

A model hands you seven lines of JSON. Before a wheel turns, you have to decide whether that
sequence is executable at all — and if not, say why in terms the model can repair. The whole
point is that a wrong plan costs **zero seconds of robot time**.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `substitute(args, spec)` | `"$var"` → a value that parameter's own schema accepts, so the rest can still be checked |
| `apply_step(step, index, state)` | one step's preconditions and postconditions over the abstract state |
| `check_goal(plan, goal, state)` | did the plan put a `goal.label` object on `goal.surface`? |
| `validate_plan(raw, skills, goal)` | the seven checks, in order |

`Step`, `Plan`, `Goal`, `PlanError`, `AbstractState`, `parse_plan`, `is_var` and `label_matches`
are given. The docstrings carry the exact error codes and message fragments the tests look for.

## Check

```bash
python course.py check 19.06              # your code
python course.py check 19.06 --solution   # the reference, to see what passing looks like
```

The tests cover each check on its own and then the interactions: a hallucinated skill is reported
without an argument check behind it (there is no schema); a literal `object_id` is rejected **and**
leaves the abstract gripper empty, so a later `place` fails too; a variable on a parameter that is
neither a place nor an object id is an argument error; driving away makes a detection stale and
picking then fails, while re-detecting after arriving makes the same plan legal; a read-only step
in the middle must *not* invalidate a detection; the goal check rejects the right object on the
wrong surface and the wrong object on the right one; and a plan with structural errors gets no
goal verdict at all. One test asserts the property everything else rests on: after validating good
and bad plans, `skills.log` is empty and `world.t` is still `0.0`.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `substitute` must keep keys it does not recognise. Dropping an unknown key hides the very error
  (`speed: unknown parameter`) that the schema exists to report.
* The placeholder for a non-enum string parameter has to satisfy that parameter's `pattern`.
  `"placeholder-0"` matches `[a-z]+-\d+`; `"x"` does not.
* In `apply_step`, a variable used as a *place* becomes an opaque token (`f"@{var}"`), not a map
  name. The validator does not know where the object is — only that "there" is the same "there"
  in a later step.
* `state.fresh.clear()` belongs in `navigate_to`, unconditionally. That one line is the whole
  reach precondition.
* Apply the postcondition only when the step's own preconditions held. A `pick` that failed its
  checks must not mark the gripper full, or every later step reports a phantom error.
* In `validate_plan`, `continue` after `UNKNOWN_SKILL`: with no `SkillSpec` there is no schema to
  validate against and no transition to apply.
* The goal check runs only when nothing else failed. A plan that is already broken has not earned
  a verdict on whether it *would* have achieved the goal.
