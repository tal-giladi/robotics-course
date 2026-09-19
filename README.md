# Practical Robotics: From Software Engineer to Autonomous AI Robot

A complete, hands-on robotics course for an experienced software / AI engineer who knows
little or nothing about robotics — from *"I know software"* to *"I can design, assemble,
program, control and extend an autonomous robot, and connect modern AI, vision and LLMs to it."*

You build **karmel**, a small differential-drive robot, and grow it stage by stage: motors and
encoders → ROS 2 → simulation → sensors → feedback control → odometry → localization → SLAM →
autonomous navigation → computer vision → a robotic arm → manipulation → learned policies →
an LLM agent that can take *"find the bottle and put it on the table"* and execute it safely.

**21 main modules · 218 lessons · 105 optional foundation lessons · 18 projects · runnable labs
(pure-Python simulator, Pico firmware, ROS 2 Jazzy workspace with Gazebo Harmonic)**

> [!NOTE]
> Designed for a student living in **Israel**: every hardware recommendation is checked against
> Israeli retailers, import rules and local prices (dated). See [HARDWARE.md](HARDWARE.md).

---

## How the course works

**Main path = practical robotics. Side paths = optional foundations.**

The main path never stops to teach you a semester of math or electronics. When a lesson needs
vectors, PWM, Gaussians or quaternions, it tells you in one line and links a short optional
foundation lesson — *"New to vectors? → FM.05. Skip it if you know them."* You learn a
prerequisite exactly when you need it, and skip what you already know.

```text
                     OPTIONAL FOUNDATIONS (take only what you need)
   Math · Electronics · Physics · Linux · Python for C# devs · C/C++ & embedded
   Control theory · Machine learning · Computer vision · 3D printing & CAD
        ┊            ┊             ┊              ┊             ┊
        ▼            ▼             ▼              ▼             ▼
 00 Orientation → 01 First robot → 02 Electronics → 03 Software → 04 ROS 2 → 05 Frames
 → 06 Simulation → 07 Sensors → 08 Control → 09 Odometry → 10 Localization → 11 SLAM
 → 12 Navigation → 13 Computer vision → 14 Robotic arm → 15 Manipulation → 16 ML for robots
 → 17 Reinforcement learning → 18 Embodied AI → 19 LLM robot agents → 20 FINAL ROBOT
```

The full roadmap, with which foundations feed which stage: **[COURSE_MAP.md](COURSE_MAP.md)**.

Every lesson has the same shape: what you will learn · why it matters · prerequisites ·
concept · technical explanation (intuition → practical → math → implementation) · diagram ·
code · exercises · expected result · troubleshooting · common mistakes · knowledge check ·
practical challenge · "you can skip this if…" · go deeper · progress checkpoint.

## Start here

1. Read [00.07 How to study this course](00-orientation/00.07-how-to-use-this-course.md) (25 min).
2. Check where you are and what's next:

   ```bash
   python course.py status
   python course.py next
   ```

3. Start [00.01 What is a robot?](00-orientation/00.01-what-is-a-robot.md) — no hardware needed for
   the orientation module, and you can do much of the course in simulation before buying anything.

### Study with an AI teacher

The course is built to be studied with a coding agent (Claude Code or similar) open in this
repository. [CLAUDE.md](CLAUDE.md) tells the agent how to teach: it reads your progress, teaches
lessons interactively, explains things differently when you're stuck, finds the foundation
lesson you're missing, verifies version-sensitive instructions against current docs, and
updates your progress. Try:

- *"I'm starting lesson 07.3."*
- *"Explain PID without mathematics." — then — "Now explain it mathematically."*
- *"I don't understand why this matrix multiplication works. Show me a concrete numerical example."*
- *"I know Kalman filters already. Let me skip this."*
- *"I tried the exercise and got this error: …"*
- *"Look up the current Nav2 documentation for this parameter."*

### Track your progress

```bash
python course.py next                    # what should I study next?
python course.py start 04.3              # I'm working on it
python course.py complete 04.3           # I did the exercises ("I can do this")
python course.py master 04.3             # I did the practical challenge
python course.py skip FM.5 --reason "I know vectors"
python course.py struggle 10.5 "covariance update"   # recommends foundations
python course.py why 11.7                # what do I still need before SLAM on my robot?
python course.py learn quaternions       # which lesson teaches this?
python course.py check 09.4              # run an auto-graded exercise
python course.py hw buy lidar            # hardware you own
```

State lives in `progress/progress.json` (machine-readable — your AI teacher reads it) and is
rendered to [PROGRESS.md](PROGRESS.md). Commit both. `course.py` needs only Python 3.10+.

## Hardware: buy in stages

You never buy everything up front. Stage 0 is a laptop (simulation). Stage 1 is the first robot;
each later stage adds one capability — IMU and camera, LiDAR, optional Jetson/depth, the arm,
final integration. Prices, Israeli suppliers and alternatives: [HARDWARE.md](HARDWARE.md).
Safety rules that apply everywhere: [SAFETY.md](SAFETY.md).

## Repository map

```text
robotics-course/
├── README.md · COURSE_MAP.md · PROGRESS.md · HARDWARE.md · SAFETY.md
├── CLAUDE.md / AGENTS.md          how an AI agent should teach you
├── AUTHORING.md · MAINTAINING.md  how the course is written and kept current
├── course.py                      progress tool (stdlib only)
├── curriculum/
│   ├── syllabus.yaml              single source of truth: lessons, prerequisites, hardware, concepts
│   ├── versions.yaml              single source of truth for version-sensitive choices
│   ├── graph.json                 generated dependency graph (read by course.py and agents)
│   ├── dependency-graph.md        Mermaid prerequisite graphs
│   └── concept-index.md           concept → lesson
├── 00-orientation/ … 20-final-robot/   the main path (one folder per module)
├── optional-foundations/          mathematics, electronics, physics, linux-and-tools, python,
│                                  cpp-and-embedded, control-theory, machine-learning,
│                                  computer-vision, 3d-printing-and-cad
├── projects/                      P01–P17 and the final project
├── labs/                          runnable code: robotlab simulator/HAL, auto-graded exercises,
│                                  Pico firmware, Pi scripts, ROS 2 workspace, Docker dev env
├── hardware/                      staged buying guides, Israeli suppliers, importing, tools
├── references/                    papers, resources, glossary, troubleshooting guides, research notes
└── tools/                         build.py (generate), validate.py (check)
```

## Read it as a website

The repository is also a [docsify](https://docsify.js.org/) site (`index.html`). Serve locally
with `python -m http.server 3000` and open http://localhost:3000, or read it on GitHub Pages.

## Software baseline (verified 2026-09)

ROS 2 **Jazzy Jalisco** (LTS) on **Ubuntu 24.04**, **Gazebo Harmonic**, Python 3.12 on the robot,
MicroPython on a Raspberry Pi Pico 2. Why not the newer Lyrical Luth yet, and how to upgrade:
[curriculum/versions.yaml](curriculum/versions.yaml) and [MAINTAINING.md](MAINTAINING.md).
