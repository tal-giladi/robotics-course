# 19.08 — Object memory that decays and is revised

Lesson: [19.08 Memory and the agent's world model](../../../19-llm-robot-agents/19.08-memory-and-world-model.md)

Storing "I saw the keys on the desk" is easy. The exercise is the arithmetic that decides, ten
minutes later, whether that sentence is still worth a 40-second drive.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `belief_now` | exponential decay, with a half-life that depends on the label |
| `observe` | positive evidence **and** the negative evidence of looking and not seeing |
| `where_is` | candidates for a label, ranked by belief now |
| `best_place` | the place worth driving to, or `None` (= "search") |
| `forget` | drop what is no longer worth carrying |
| `choose_start` | memory → map prior → first room, and which tier it used |

`ObjectBelief`, `half_life_for` and `label_matches` are given.

## Check

```bash
python course.py check 19.08              # your code
python course.py check 19.08 --solution   # the reference
```

## Hints

* Clamp the age at 0 before the decay. A robot's clock is not monotonic across a reboot, and a
  negative age makes a belief *grow*.
* The negative update is filtered by place. Looking at the kitchen counter and not seeing the keys
  is evidence about the kitchen counter and about nothing else.
* An object seen somewhere new is a **new fact**: reset `times_seen` and `times_missed` before
  updating, or the old place's miss count silently follows it around.
* `best_place` returning `None` is a feature. "Search" is a correct answer; "drive to where it was
  an hour ago" is a wrong answer that costs 40 seconds and then still has to search.
* Store `untrusted_text_seen` in its own field and never merge it into the label, the place or
  anything else that gets rendered as a fact. That separation is the whole defence in
  [19.09](../../../19-llm-robot-agents/19.09-agent-safety-boundaries.md).
