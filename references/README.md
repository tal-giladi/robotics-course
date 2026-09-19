# References

This folder holds reference material you look things up in: reading lists, definitions, debugging
guides and the dated evidence behind the course's choices. Nothing here is a lesson, and you do not
need to read it in order. The lessons link into it when it is relevant.

| File | What it is | Use it when |
|---|---|---|
| [papers.md](papers.md) | An ordered reading path of 52 core papers in 8 tracks: state estimation → SLAM → planning and navigation → manipulation → vision → RL → imitation learning and VLAs → LLM agents. It has a Mermaid map, a "how to read a paper" guide, maturity labels, and optional extras. Every entry names its prerequisite lessons and the lesson to read it after. | A lesson made you curious, or you are about to put an algorithm on the robot and want to know why its parameters exist. |
| [resources.md](resources.md) | Authoritative resources by topic (official docs → papers → university courses → tutorials → books), with cost, level, what each is best for and the modules it supports. It ends with **Fast-moving technologies: verify before use** and a table of moved and dead links. | You want a second explanation, the official docs, or a course that goes deeper. Also before following install steps for ROS 2, Gazebo, Nav2, MoveIt, Jetson/Isaac, LeRobot, VLAs or LLM APIs. |
| [glossary.md](glossary.md) | Short definitions of the robotics, electronics, math and AI terms used in the course. | You meet a term you do not know. `python course.py learn <concept>` also finds the owning lesson. |
| [troubleshooting/README.md](troubleshooting/README.md) | Symptom-driven debugging guides that cut across lessons, written as decision procedures: observe or measure, then branch. | Something does not work and the lesson's own Troubleshooting section did not cover it. |
| [research/](research/) | Dated research snapshots (see below). | You want to know **why** the course picked a version, a part or a model, or you are updating the course. |

## Research snapshots

The files in [`research/`](research/) are **dated evidence, not lessons**. Each one records what
was true on the day it was checked (versions, prices, availability, links, the state of the field),
together with its sources and a verification method. The course's decisions rest on them:

- [`curriculum/versions.yaml`](../curriculum/versions.yaml) takes its versions from them.
- [HARDWARE.md](../HARDWARE.md) takes its parts and prices from them.
- The maturity labels in [papers.md](papers.md) and in Module 18 come from them.

| Snapshot | Evidence for |
|---|---|
| [software-versions-2026-09.md](research/software-versions-2026-09.md) | Why ROS 2 Jazzy + Ubuntu 24.04 + Gazebo Harmonic. Apt binary availability per distro, Pi 5 and Jetson facts, deprecations and migration gotchas. |
| [learning-resources-2026-09.md](research/learning-resources-2026-09.md) | Every URL in [resources.md](resources.md), with its check result and the replacements for moved or dead links. |
| [papers-raw-2026-09.md](research/papers-raw-2026-09.md) | The full, untrimmed paper list (about 76 core entries and about 50 extras), with arXiv/DOI verification, from which [papers.md](papers.md) was curated. |
| [embodied-ai-landscape-2026-09.md](research/embodied-ai-landscape-2026-09.md) | The state of embodied AI in September 2026: LeRobot, the VLA families, world models and simulators with maturity labels and compute needs; LLM + robot agent frameworks and safety papers. |
| [israel-hardware-stage1-2026-09.md](research/israel-hardware-stage1-2026-09.md) | Stage 1 parts, Israeli suppliers and prices (≈ ₪, Sept 2026) behind [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md). |
| [israel-hardware-stages2-5-2026-09.md](research/israel-hardware-stages2-5-2026-09.md) | Parts, suppliers and prices for the later stages (sensors, LiDAR, Jetson, depth camera, arm). |

How to treat them:

- **They are dated on purpose.** A snapshot is never edited to "keep it current". When facts
  change, a maintainer writes a new file (for example `software-versions-2027-03.md`), updates
  `versions.yaml`, `HARDWARE.md`, `papers.md` and `resources.md` from it, and keeps the old one as
  history. The procedure is in [MAINTAINING.md](../MAINTAINING.md).
- **UNVERIFIED means UNVERIFIED.** Items marked that way are leads, not facts. Do not quote them in
  a lesson as settled.
- **Check the date before trusting them.** The older a snapshot is, the more likely its versions,
  prices and model recommendations are stale. This matters most for ROS 2/Nav2 binaries, Jetson
  software, LeRobot and VLA models.

> [!IMPORTANT]
> **Version-sensitive** (snapshots dated 2026-09-16).
> AI teacher: the snapshots tell you what was true on that date. Before you give installation or
> configuration instructions, verify against the current official documentation listed in
> [resources.md → Fast-moving technologies](resources.md#fast-moving-technologies-verify-before-use),
> and tell the student when something has changed.
