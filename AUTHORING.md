# Authoring guide

How lessons in this repository are written. Every lesson, human- or AI-written, follows this
guide so the course feels like one course, the tooling can parse it, and an AI teacher can
teach from it.

## 1. The student

An experienced software engineer / CTO: strong C#/.NET, architecture, distributed systems,
APIs, SQL, JavaScript, Git, AI/LLM experience, some Python, ML concepts but not mastery.
Knows **nothing** about robotics, electronics, mechanics, control, robotics math, ROS or
embedded — unless they say so.

- Never teach basic programming (loops, classes, HTTP, JSON, Git basics, async as a concept).
- Always explain robotics, electronics, math and physics from first principles *or* link the
  optional foundation lesson that does.
- Use analogies to distributed systems / .NET when they genuinely clarify
  (ROS topics ≈ pub/sub on a message bus; actions ≈ long-running jobs with progress and cancel;
  a hardware abstraction layer ≈ a repository interface with a fake for tests).
  Do not force C# where Python or C++ is the standard.
- Tone: direct, practical, respectful of the reader's time. No filler, no hype, no
  "in this exciting lesson". Short paragraphs. Concrete numbers.

## 2. The principle: main path + optional side paths

- The **main path** (modules `00`–`20`) is practical robotics. It never blocks on theory.
- When a main lesson uses a concept that may be new (vectors, PWM, Gaussians, quaternions…)
  say so in one sentence and link the foundation lesson:
  `> [!NOTE] New to vectors? → [FM.05 Vectors from zero](../optional-foundations/mathematics/FM.05-vectors.md). Skip it if you already know them.`
- Foundation lessons (`FM`, `FE`, `FP`, `FL`, `FPY`, `FC`, `FCT`, `FML`, `FCV`, `F3D`) are
  self-contained, short, and **every concept ends with a robotics example**
  ("A vector isn't abstract here: it's the robot's velocity…"). They are not a degree.
- Prefer experiments to theory: observe the problem → add the fix → observe → then explain the
  math. (PID: hold a speed, see the error, add P, see what happens, add I, add D, then derive.)

## 3. Files, ids and metadata

- Metadata lives **only** in [`curriculum/syllabus.yaml`](curriculum/syllabus.yaml). Do not
  invent new ids, change titles or prerequisites in the lesson file. If the syllabus is wrong,
  fix the syllabus.
- Lesson path: `<module dir>/<id>-<slug>.md`, e.g. `04-ros2/04.03-nodes-and-cli.md`,
  `optional-foundations/mathematics/FM.05-vectors.md`. Projects: `projects/P04-pid-movement.md`.
- `python tools/build.py` injects the generated blocks (see §4). Run it after writing.
  `python tools/validate.py` must pass.

## 4. Lesson structure (required, in this order)

Headings must match exactly (the validator checks the `## ` headings in order). Use `###`
freely inside sections.

```markdown
# 04.03 — Nodes and the ros2 command line

<!-- glance:start -->
<!-- glance:end -->

## What you will learn
3–6 bullets of *capabilities* ("Start a node and list its topics"), not topics.

## Why it matters
One or two paragraphs grounded in the final robot. Why would a robot need this?

<!-- prereqs:start -->
<!-- prereqs:end -->

## Concept
The intuitive explanation. Analogies, a story, a picture in words. No equations yet.
Include at least one "Ask your teacher" tip (see §7).

## Technical explanation
The real explanation. When the topic has depth, split into levels the reader can stop at:
### Level 1 — Intuition   (optional here if Concept already did it)
### Level 2 — Practical
### Level 3 — Mathematics
### Level 4 — Implementation
Math is written in LaTeX ($…$ / $$…$$) AND always followed by a concrete numerical example.

## Diagram
At least one diagram: Mermaid (```mermaid) for flows/architectures/state machines, or an
ASCII/Unicode diagram in a ```text block for geometry, wiring and frames. Wiring diagrams
list every connection in a table as well (pin → pin, wire color suggestion, voltage).

## Code
Complete, runnable code with the path it lives at. Prefer code that lives in `labs/` and is
tested; show the important part inline and link the file. State the exact command to run it.
No "…" elisions inside code the reader is expected to run.

## Exercise
One or more exercises, each with an id and a type tag:
### Exercise 04.03-E1 — Title  `[hardware]` | `[simulation]` | `[coding]` | `[numerical]` | `[debugging]` | `[predict]` | `[design]`
Each exercise states: goal, steps, and **what hardware it needs** (only items in HARDWARE.md,
or "none"). Include a "predict what will happen" exercise where it fits: ask for a
prediction *before* running.
If an automated checker exists: "Check it: `python course.py check 09.04`".

## Expected result
For EVERY exercise: what the reader should see (numbers with tolerances, a plot shape,
terminal output, robot behavior). An exercise without an expected result is a bug.

## Troubleshooting
Symptom-driven and systematic. Format:
### Symptom: the motor doesn't move
A short decision procedure: measure/observe X → if A then … else … Ordered from
most likely/cheapest check to least. Teach the method, not just a list of fixes.

## Common mistakes
Bullet list: the mistake, why it happens, how to avoid it.

## Knowledge check
5–8 questions mixing recall, numerical, "predict what happens" and debugging scenarios.
Every answer hidden in a details block:
1. Question?
   <details><summary>Answer</summary>

   Answer with explanation.
   </details>

## Practical challenge
One harder, open-ended task that proves "I can actually do this". Include acceptance
criteria. This is what "mastered" means in the progress tool.

## You can skip this if…
A concrete skip test: "Answer knowledge-check questions 3, 5 and 6 correctly without
looking, and you can … ". Then: `python course.py skip 04.03 --reason "…"`.

## Go deeper
Authoritative resources only, with full clickable URLs and one line each on why/when:
official docs → original papers → university courses → high-quality tutorials → books.
Never "search for X". Use `references/research/learning-resources-2026-09.md` as a source.

## Progress checkpoint
- [ ] I can … (3–5 self-assessment checkboxes mirroring "What you will learn")
Then the commands:
`python course.py complete 04.03` (exercises done) · `python course.py master 04.03`
(challenge done) · `python course.py struggle 04.03 "what was hard"`.
Finish with a one-line pointer to the next lesson.
```

Foundation lessons use the same structure (their "Exercise" sections are usually
`[numerical]`, `[coding]` or small `[hardware]` experiments; the "Why it matters" must name
the main-path lessons that need it). Project files use the project template in
[`projects/README.md`](projects/README.md).

## 5. Length and digestibility

- A lesson is one sitting: typically 1,500–3,500 words plus code. If it needs more, the
  syllabus should split it.
- Many small lessons + many experiments + optional deep dives. Not textbook chapters.

## 6. Safety is inline, everywhere

Whenever a step involves batteries, mains chargers, soldering, moving wheels, arms, lasers
(LiDAR is eye-safe Class 1 — say so), heat, or autonomous motion, put a safety callout
**before** the step:

```markdown
> [!CAUTION]
> **Safety:** Put the robot on a stand (wheels in the air) before the first motor test.
```

Standing rules every hardware lesson assumes (see [SAFETY.md](SAFETY.md)): wheels off the
ground for first tests; a reachable power switch; a software watchdog that stops motors when
commands stop; fused battery; never leave charging Li-ion/LiPo unattended; keep pets and
children out of the test area; arms start with low torque and speed limits.

## 7. Interactive learning hooks

The course is studied with an AI coding agent as a teacher (see [CLAUDE.md](CLAUDE.md)).
Encourage it explicitly:

```markdown
> [!TIP]
> **Ask your teacher:** "Explain the Kalman gain without equations, then again with a numerical example."
```

Offer 1–3 such prompts per lesson at the points where readers typically get stuck.

## 8. Version-sensitive content

Anything that depends on fast-moving versions (ROS 2 distro, Gazebo, Ubuntu, Nav2/MoveIt
parameters, JetPack, model names, Python package APIs, prices) gets a dated marker:

```markdown
> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09 against ROS 2 Jazzy / Gazebo Harmonic).
> Before following these steps, check the current docs: https://docs.ros.org/en/jazzy/Installation.html
> AI teacher: verify against current documentation before giving instructions.
```

The single source for versions is [`curriculum/versions.yaml`](curriculum/versions.yaml).
Use its values; do not invent others. Mark deprecated packages/APIs explicitly
("⚠ Deprecated: Gazebo Classic reached end-of-life …"). Never teach deprecated tools as the
main path.

## 9. Hardware and prices

Only reference hardware listed in [HARDWARE.md](HARDWARE.md) (catalog ids `robot-base`, `imu`,
`camera`, `lidar`, `jetson`, `depth-camera`, `arm`, `estop`, `tools`). Every exercise
that needs hardware must say which, and should offer a simulation alternative when one
exists. Prices are always approximate and dated ("≈ ₪45, Sept 2026").

## 10. Code standards

- Python ≥ 3.10 (Ubuntu 24.04 ships 3.12), type hints, `dataclasses`, no global state,
  hardware behind interfaces with fakes for tests.
- Code that lives in `labs/` must have tests (`pytest`) that run without hardware.
- ROS 2 code targets the distro in `curriculum/versions.yaml`, Python launch files,
  `ament_python` unless C++ is the point.
- Firmware for the Pico is MicroPython in `labs/firmware/pico/` (C++ appears in FC lessons).
- Use SI units (meters, radians, seconds) everywhere, as REP-103 requires.

## 11. Links

- Internal links are relative and must resolve (`tools/validate.py` checks).
- External links: full `https://` URLs, prefer official docs, verified when written.

## 12. Diagrams conventions

- Frames: x forward (red), y left (green), z up (blue) — REP-103.
- Mermaid for architecture/sequence/state; ASCII in ```text for geometry and wiring.
