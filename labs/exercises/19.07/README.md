# 19.07 — Verify every step, then repair

Lesson: [19.07 Perception–action loops — verifying every step](../../../19-llm-robot-agents/19.07-perception-action-loops.md)

A skill returns `SUCCEEDED`. Did anything happen? The only honest answer comes from a **different
channel** than the one that did the work — and from deciding, in code, what to do when the two
disagree.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `verify_navigate` | did the robot arrive, by proprioception (free) |
| `verify_pick` | is the object in the gripper, by state *and* by camera (0.2 s) |
| `verify_place` | is the object on the surface |
| `verify_detect` | did the detection find what the step was for |
| `verify_step` | dispatch, plus the two cases that come first |
| `choose_repair` | one rung of the ladder: ok · retry · repair · replan |
| `should_look_again` | is a second observation worth 0.2 s |

`Verification`, `label_matches`, `read_state` and `look` are given. The docstrings carry the exact
fields the tests read.

## Check

```bash
python course.py check 19.07              # your code
python course.py check 19.07 --solution   # the reference
```

The tests use `FaultySkills` from the lesson's code to inject three real failures: a gripper that
reports a grasp on an empty hand (`phantom_grasp`), the same gripper with a limit switch that
agrees with it (`blind_gripper`), and navigation that reports arrival from where it stopped
(`short_nav`). Each one is caught by exactly one channel, which is the whole point of the
exercise: the first two are indistinguishable from success until something *looks*.

## Hints

* A verifier never reads `result.ok` to decide. It reads the world.
* `verify_pick`'s perception check fails when it *still sees* the object. Looking again can only
  miss it, so that verification is `repeatable=False` — otherwise a second look converts a real
  failure into a pass, and your test suite will not notice.
* `verify_place`'s perception check fails when it *does not see* the object. There a second look
  is worth 0.2 s: it removes a detector miss.
* A failed call is `NOT_VERIFIED` with method `"result"` and **no** extra observation. Spending
  robot time to confirm a failure the skill already reported is how a verified agent becomes
  slower than an unverified one for no benefit.
* `choose_repair` must never return `"retry"` for a skill whose spec says `idempotent=False`.
  `place` has already opened the gripper; a second call returns `NOT_HOLDING_OBJECT`.
* A `pick` that failed or could not be verified has moved the object. Skip the retry rung and go
  straight to `"repair"` — that is the same fusing rule the 19.06 compiler applies, made explicit.
