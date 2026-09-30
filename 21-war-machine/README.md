# 21 · War machine — weapons for karmel

> **Standalone module.** This module is parallel to the main course: it is not (yet) registered
> in `curriculum/syllabus.yaml`, so `course.py` does not know about it. The format is the course
> format; the links are relative and resolve on GitHub.

Turning the finished course robot (karmel, module 20) into a **war machine**: add-ons, payloads
and weapons it can drop, throw, shoot, stick, spray and smoke — with at least three trigger
types (manual, wireless, vision, sound), at least three releaser types, a full Israeli
over-the-counter BOM with shop links, and the weight, power and stability math for the added load.

**10 lessons · ≈ 17 h 30 min** · [BOM (Israeli OTC, with links)](hardware/war-machine-bom.md) ·
[Build progress log](PROGRESS.md)

| Id | Lesson | Level | Time | Prerequisites | Hardware |
|---|---|---|---|---|---|
| 21.01 | [War machine architecture — what changes on karmel](21.01-war-machine-architecture.md) | advanced | 1 h 30 min | 20.01, 20.02 | robot-base |
| 21.02 | [Payloads — the BOM of things you can throw, shoot, stick and spray](21.02-payloads-and-bom.md) | advanced | 1 h 30 min | 21.01 | — |
| 21.03 | [Releasers I — latches, clamps and pins (static payloads)](21.03-releasers-i-latches.md) | advanced | 2 h | 21.01, 21.02 | weapon-rail |
| 21.04 | [Releasers II — launch mechanisms (kinetic payloads)](21.04-releasers-ii-launch.md) | advanced | 2 h | 21.02, 21.03 | weapon-rail, cannon |
| 21.05 | [Triggers I — remote control](21.05-triggers-remote.md) | advanced | 1 h 30 min | 21.03 | keypad, rc-link |
| 21.06 | [Triggers II — vision detection](21.06-triggers-vision.md) | advanced | 2 h | 21.05, 13.07, 13.10 | camera |
| 21.07 | [Triggers III — sound detection](21.07-triggers-sound.md) | advanced | 1 h 30 min | 21.05 | mic |
| 21.08 | [Weights, power and stability of the war load](21.08-weights-power-stability.md) | advanced | 2 h | 21.04, 21.05 | robot-base |
| 21.09 | [What can go wrong — failure modes and safety](21.09-failure-modes-and-safety.md) | advanced | 1 h 30 min | 21.08 | robot-base, estop |
| 21.10 | [Integration, test range and the war demo](21.10-integration-and-demo.md) | advanced | 2 h | 21.06, 21.07, 21.08, 21.09 | full build |

Start with `21.01-war-machine-architecture.md`. Track your own progress in
[PROGRESS.md](PROGRESS.md) — the module was written across sessions and that file is the
restart contract.

## Code (ships with the module, in `code/`)

| File | What it is | Run |
|---|---|---|
| `war_power_budget.py` | the CG/tipping model, the Thevenin sag model, the firing-energy budget, the package budgets | `py 21-war-machine/code/war_power_budget.py mass` · `sag --current 15` · `power` · `packages` · `delta` · `check` |
| `releaser.py` | the flap torque budget, the solenoid pin force, the pulse-width model | `py 21-war-machine/code/releaser.py flap` · `pin` · `pulse` · `check` |
| `projectile_range.py` | the launch model: muzzle velocity, ideal and dragged range, recoil, the lobber verdict | `py 21-war-machine/code/projectile_range.py slingshot` · `cannon` · `arm` · `coga` · `recoil` · `check` |
| `trigger_pipeline.py` | the Weapon FSM + the four triggers (button, RC/PPM, vision dwell, sound double-clap) and the e-stop interlock | `py 21-war-machine/code/trigger_pipeline.py demo` |
| `test_war_machine.py` | the module's unit tests (33 tests, no hardware) | `py -m pytest 21-war-machine/code -q` |
| `exercises/student.py` | the stubs for the `-E2` coding exercises (10 stubs, one per lesson) | fill in, then `py -m pytest 21-war-machine/code/exercises -k <id>` |
| `exercises/test_exercise.py` | the checkers for the stubs (skip until filled — a fresh clone is green) | `py -m pytest 21-war-machine/code/exercises -k 2101_e2` (etc.) |
| `validate_module21.py` | this module's format + link + chain validator (wraps the course's `tools/validate.py`) | `py 21-war-machine/code/validate_module21.py` |

The exercise checkers are keyed by lesson: `2101_e2`, `2103_e2`, `2104_e2`, `2105_e2`,
`2106_e2`, `2107_e2`, `2108_e2`. A fresh clone runs `33 passed, 19 skipped` (the skips are
the unfilled stubs).

## Research

- [references/war-machine-research-2026-09.md](references/war-machine-research-2026-09.md) —
  the dated shop-verification log behind the BOM (the MAINTAINING.md convention: a price is a
  fact only with a date).
