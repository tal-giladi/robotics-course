# Maintaining the course

Robotics software, AI models and hardware prices move fast. The repository is built so a
change in one technology means editing one place and re-validating, not rewriting the course.

## The single sources of truth

| What | Where | Consumers |
|---|---|---|
| Lessons, prerequisites, hardware per lesson, concepts, skip tests | [`curriculum/syllabus.yaml`](curriculum/syllabus.yaml) | `tools/build.py` → `curriculum/graph.json`, lesson "At a glance" and Prerequisites blocks, `_sidebar.md`, `COURSE_MAP.md` tables, dependency graph, concept index; `course.py` |
| Versions (ROS 2 distro, Ubuntu, Gazebo, packages, JetPack, Isaac) and known gotchas | [`curriculum/versions.yaml`](curriculum/versions.yaml) | lessons quote it; `Version-sensitive` markers point to it |
| Robot dimensions, pins, sensor placement | [`labs/config/karmel.yaml`](labs/config/karmel.yaml) | `robotlab`, firmware config, URDF (`karmel_description/config/robot.yaml` copy, checked by `labs/ros2_ws/check_config_sync.py`) |
| Pi ↔ Pico protocol | [`labs/README.md`](labs/README.md) + `labs/python/robotlab/protocol.py` | firmware, `SerialBase`, `karmel_base`, `karmel_hardware` |
| Hardware prices and suppliers | [`HARDWARE.md`](HARDWARE.md), [`hardware/`](hardware/README.md) | lessons link, never copy prices |
| Evidence behind version/hardware choices | [`references/research/`](references/research/) (dated snapshots) | maintainers |

## Everyday commands

```bash
pip install pyyaml                      # build.py needs it (course.py does not)
python tools/build.py                   # regenerate everything after editing syllabus.yaml
python tools/build.py --check           # CI: fails if generated files are stale or lessons are missing
python tools/validate.py                # lesson structure, exercises, knowledge checks, links
python tools/validate.py --links --final  # every relative link in the repo must resolve
python tools/validate.py --versions     # list every Version-sensitive marker (audit list)
python tools/validate.py --external     # HTTP-check external URLs (slow)
pytest                                  # labs: robotlab, firmware logic (exercises skip: student.py is empty)
COURSE_USE_SOLUTION=1 pytest labs/exercises   # every auto-graded exercise passes with its solution
pytest -o testpaths= --ignore-glob='*/code/src' --ignore-glob='*/code/ros2' \
       [0-2][0-9]-*/code projects/code   # the code shipped with the lessons and projects
```

The ROS 2 workspace is tested in Docker — see [`labs/ros2_ws/README.md`](labs/ros2_ws/README.md)
and [`labs/ros2_ws/TESTED.md`](labs/ros2_ws/TESTED.md).

## How to…

### Add or change a lesson
1. Edit `curriculum/syllabus.yaml` (id, slug, title, requires, optional, hardware, teaches, skip_if, version_sensitive).
2. Write the file following [AUTHORING.md](AUTHORING.md) — the headings are validated.
3. `python tools/build.py && python tools/validate.py --paths <file>`.
4. Renumbering ids breaks students' `progress/progress.json`; prefer adding new ids (e.g. `08.13`) over renumbering. If you must rename, add a migration note to the changelog below.

### Upgrade the ROS 2 distro (e.g. Jazzy → Lyrical Luth)
1. Check the release status of the whole stack (nav2, slam_toolbox, moveit, ros2_control, gz_ros2_control, robot_localization, drivers, micro-ROS) on packages.ros.org for the new distro — not just the core. Record findings in a new dated `references/research/software-versions-YYYY-MM.md`.
2. Update `curriculum/versions.yaml` (distro, Ubuntu, Gazebo pairing, gotchas, docker images).
3. Rebuild and test `labs/docker` and `labs/ros2_ws` in the new image; update `TESTED.md`.
4. `python tools/validate.py --versions` lists every lesson section to review. Update each marker's "verified" date and the commands under it. Lessons flagged `version_sensitive: true` in the syllabus are the minimum set.
5. Search for hard-coded distro names: `git grep -n -i "jazzy"` and `git grep -n "harmonic"`.
6. Deprecated APIs: keep a short "⚠ Deprecated" note where students will meet old tutorials (e.g. Gazebo Classic, `ros_ign`), don't silently delete the warning.

### Refresh hardware prices and availability
1. Re-run the research (Israeli suppliers first, then international) into a new dated file in `references/research/`.
2. Update `HARDWARE.md` and `hardware/stage-*.md` — every price keeps its date.
3. If a part is discontinued, pick a replacement that keeps `labs/config/karmel.yaml` compatible, or update the config and the affected lessons (grep for the part name).

### Update fast-moving AI content (VLAs, foundation models, detectors, LLM APIs)
Module 18 and parts of 13, 16, 17 and 19 carry `Version-sensitive` markers and maturity labels
(MATURE / USABLE-BY-HOBBYIST / RESEARCH). Re-verify against the official project pages and
update `references/research/embodied-ai-landscape-*.md`, `references/papers.md` and the
lesson's landscape tables. Don't rewrite the conceptual sections — ACT, diffusion policies and
behavior cloning age slowly; model names and compute requirements age fast.

### Review cadence (suggested)
| Every | Check |
|---|---|
| 3 months | `validate.py --external` (dead links), hardware stock of Stage 1–3 items |
| 6 months | embodied-AI landscape, LLM API tool-calling docs, LeRobot version, detector models |
| Each May (ROS 2 release) | distro decision in `versions.yaml` |
| Each September (Gazebo release) | Gazebo pairing |

## Generated files — never edit by hand

`curriculum/graph.json`, `curriculum/dependency-graph.md`, `curriculum/concept-index.md`,
`_sidebar.md`, the `<!-- lessons -->` part of `COURSE_MAP.md`, the `<!-- glance -->` and
`<!-- prereqs -->` blocks in lessons, and `PROGRESS.md` (generated by `course.py`).

## Changelog

- **2026-09-16** — Initial version. ROS 2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic. Hardware researched for Israel (prices dated 2026-09-16).
