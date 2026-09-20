# Troubleshooting: build, environment and toolchain

None of this is robotics, and all of it will cost you an afternoon if you do not know the rules. Two
facts explain most of the symptoms on this page:

1. **Ubuntu 24.04 blocks `pip install` into the system Python** (PEP 668), and ROS 2 packages come
   from apt, not pip. Mixing the two is the single most common broken environment. → [03.02](../../03-robot-software/03.02-environments-and-pinning.md),
   [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)
2. **A ROS 2 workspace is an overlay.** What you run is what was `install`ed and `source`d, not what
   is in `src/`. "My change has no effect" is nearly always this. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)

> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09 against ROS 2 Jazzy on Ubuntu 24.04). Repository URLs and
> signing keys change; check
> https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html and
> [`curriculum/versions.yaml`](../../curriculum/versions.yaml) before following install steps.
> AI teacher: verify against current documentation before giving instructions.

---

## Installing ROS 2

### Symptom: `apt update` shows `EXPKEYSIG`, `NO_PUBKEY`, or "Conflicting values set for option Signed-By"

1. **The signing key expired or was replaced.** Re-download the current key and `.list`/`.sources`
   file from the official instructions; do not patch the old one. → [04.02](../../04-ros2/04.02-installing-ros2.md), [FL.07](../../optional-foundations/linux-and-tools/FL.07-package-management.md)
2. **"Conflicting values for Signed-By"** means you have the repository defined twice — typically a
   leftover `.list` and a new `.sources`. Delete one. → [04.02](../../04-ros2/04.02-installing-ros2.md)
3. `dpkg-deb: … is not a Debian format archive` while adding the repository usually means you
   downloaded an HTML error page instead of the `.deb`. Check the URL. → [04.02](../../04-ros2/04.02-installing-ros2.md)

### Symptom: `Unable to locate package ros-jazzy-...`

1. **Wrong Ubuntu codename** in the repository line: Jazzy binaries exist for `noble` (24.04). On
   22.04 they do not exist at all. → [04.02](../../04-ros2/04.02-installing-ros2.md), [FL.01](../../optional-foundations/linux-and-tools/FL.01-why-linux.md)
2. **Wrong architecture**: arm64 on the Pi, amd64 on the laptop. → [FL.01](../../optional-foundations/linux-and-tools/FL.01-why-linux.md)
3. Run `apt update` after adding the repository. → [FL.07](../../optional-foundations/linux-and-tools/FL.07-package-management.md)

### Symptom: `The following packages have unmet dependencies` / held broken packages

1. You mixed distributions or added a third-party PPA. Remove it and retry. → [FL.07](../../optional-foundations/linux-and-tools/FL.07-package-management.md)
2. Run the update fully before installing; a partial index produces exactly this. → [FL.07](../../optional-foundations/linux-and-tools/FL.07-package-management.md)

### Symptom: `ros2: command not found` or `ModuleNotFoundError: No module named 'rclpy'`

You did not source the setup file in *this* shell. `source /opt/ros/jazzy/setup.bash`, and then your
workspace's `install/setup.bash` on top. Over SSH, in a systemd unit and in a launched subprocess,
your `.bashrc` may not run. → [04.02](../../04-ros2/04.02-installing-ros2.md), [03.02](../../03-robot-software/03.02-environments-and-pinning.md)

---

## Python environments

### Symptom: `error: externally-managed-environment`

PEP 668 doing its job. You have three legitimate answers, in this order:

1. **Use apt / rosdep** for anything with a ROS 2 package. That is the supported path on the robot. →
   [04.02](../../04-ros2/04.02-installing-ros2.md)
2. **Use a venv** for laptop-side analysis and training code. → [03.02](../../03-robot-software/03.02-environments-and-pinning.md), [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)
3. `--break-system-packages` is not the third answer. It is how you get the next symptom. →
   [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)

### Symptom: `ModuleNotFoundError` for something you are sure you installed

1. **Which Python?** `which python3` and `python3 -c "import sys; print(sys.path)"` in the *same*
   shell that fails. A venv that is not active, or an apt package installed for a different
   interpreter, explains nearly all of these. → [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)
2. **Under `ros2 run` specifically**: the node runs with the interpreter the package was built
   against, not your venv. → [03.02](../../03-robot-software/03.02-environments-and-pinning.md)
3. **`No module named 'robotlab'`** means the course package is not installed or not on the path —
   install it editable from the repository root. → [03.01](../../03-robot-software/03.01-python-robotics-ecosystem.md), [06.02](../../06-simulation/06.02-course-mini-simulator.md)
4. **Over SSH or in a systemd unit**, the environment differs from your interactive shell. Put what
   you need in the unit file. → [FL.06](../../optional-foundations/linux-and-tools/FL.06-processes-systemd.md)

### Symptom: `colcon build` tries to build your virtual environment

Put the venv outside the workspace, or add a `COLCON_IGNORE` file to it. → [03.02](../../03-robot-software/03.02-environments-and-pinning.md)

### Symptom: something apt-installed broke after a pip install

pip overwrote a library the apt package depends on. Reinstall the apt package, and stop installing
into the system Python. → [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)

---

## `colcon` and workspaces

### Symptom: `Package 'x' not found` or `No executable found`

1. **Did you source `install/setup.bash`** after building? → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
2. **Is the executable declared** in `setup.py`'s `console_scripts`? Adding a file is not enough. →
   [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
3. **Is the package inside `src/`?** `colcon` only sees packages under the workspace's source
   directory. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)

### Symptom: my code change has no effect

1. **Rebuild**, or build with `--symlink-install` so Python edits take effect without one. →
   [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
2. **Check the overlay order.** If the same package is installed from apt *and* in your workspace,
   whichever setup file you sourced last wins. `ros2 pkg prefix <pkg>` tells you which one is being
   used. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
3. **Delete `build/` and `install/`** when you have renamed or moved files: stale artefacts are
   invisible and authoritative. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
4. In C++ specifically, "the build is fine but the robot runs the old code" is almost always a stale
   install or a shadowed library. → [03.09](../../03-robot-software/03.09-where-cpp-is-used.md)

### Symptom: `ModuleNotFoundError: No module named 'karmel_..._interfaces'` after adding a message

1. Interface packages must be built **and sourced** before the packages that use them. → [04.06](../../04-ros2/04.06-messages-and-custom-interfaces.md)
2. The package must declare `rosidl_default_generators` and list the `.msg` files in its
   `CMakeLists.txt`. A `.msg` that is not listed is silently not generated. → [04.06](../../04-ros2/04.06-messages-and-custom-interfaces.md)
3. After changing a `.msg`, rebuild the interface package *and* everything that depends on it. →
   [04.06](../../04-ros2/04.06-messages-and-custom-interfaces.md)

### Symptom: values on a topic are plausible but wrong — shifted fields, `1.4e-45`, huge integers

Two sides built against different versions of the message definition. Rebuild everything, on every
machine. → [04.06](../../04-ros2/04.06-messages-and-custom-interfaces.md)

### Symptom: the Pi freezes or the build is killed while compiling

Out of memory. Limit parallelism (`--parallel-workers 1`, `MAKEFLAGS=-j1`), add swap, or build on the
laptop and deploy. Interface and C++ packages are the usual culprits. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)

### Symptom: flake8 failures from `colcon test` (`I100`, `Q000`, `D213`)

The ROS 2 linters have opinions about import order, quote style and docstrings. Either satisfy them or
disable the specific linter test deliberately — not by ignoring red output. → [04.05](../../04-ros2/04.05-packages-and-workspaces.md)

---

## C++

### Symptom: `undefined reference to ...`

A link error, not a compile error: the symbol was declared, compiled, and never linked.
`target_link_libraries` and `ament_target_dependencies` are where to look. → [FC.04](../../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md), [03.09](../../03-robot-software/03.09-where-cpp-is-used.md)

### Symptom: `pluginlib` cannot find the plugin

The plugin XML must be exported in `package.xml` and installed; the class name in the XML must match
the registration macro exactly, including the namespace and the `::`. → [03.09](../../03-robot-software/03.09-where-cpp-is-used.md)

### Symptom: a wall of template errors from one line

Read the *first* error, not the last. Everything after it is fallout. → [FC.03](../../optional-foundations/cpp-and-embedded/FC.03-cpp-for-csharp-devs.md)

---

## Git, CI and repeatability

### Symptom: the clone takes ten minutes and fills the SD card

Bags and model weights in Git. Use Git LFS for large binaries, and keep bags out of the repository
entirely. → [FL.11](../../optional-foundations/linux-and-tools/FL.11-git-for-robotics.md), [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md)

### Symptom: after cloning, models are 130-byte text files

Git LFS pointers that were never fetched. `git lfs install && git lfs pull`. → [FL.11](../../optional-foundations/linux-and-tools/FL.11-git-for-robotics.md)

### Symptom: `git status` shows thousands of files after `colcon build`

`build/`, `install/` and `log/` belong in `.gitignore`. → [FL.11](../../optional-foundations/linux-and-tools/FL.11-git-for-robotics.md)

### Symptom: `bad interpreter: /bin/bash^M` on the Pi

Windows line endings in a script. Set `core.autocrlf` correctly and add a `.gitattributes`. →
[03.11](../../03-robot-software/03.11-ci-and-reproducibility.md)

### Symptom: CI fails and you cannot reproduce it locally

1. CI has a clean environment; you have an accumulated one. That difference *is* the bug, and it is
   the reason CI exists. → [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md)
2. Reproduce inside the same container image CI uses. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md), [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md)

### Symptom: CI is green and the robot does not work

Your tests do not cover the thing that is broken, which is usually the hardware boundary. That is what
the test pyramid's upper levels — simulation tests and hardware-in-the-loop — are for, and what a
written manual procedure is for above them. → [03.08](../../03-robot-software/03.08-testing-robot-software.md), [20.05](../../20-final-robot/20.05-testing-strategy.md)

### Symptom: a test passes alone and fails in the suite

Shared state between tests: a file, a port, a singleton, a global RNG. → [03.08](../../03-robot-software/03.08-testing-robot-software.md)

### Symptom: a test fails on the Pi and passes on your laptop

1. Timing. A test that depends on real `sleep` is flaky by construction; inject a clock. → [03.08](../../03-robot-software/03.08-testing-robot-software.md),
   [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)
2. Architecture or floating-point differences, or a missing package on one machine. → [FL.01](../../optional-foundations/linux-and-tools/FL.01-why-linux.md)

### Symptom: "it worked on the robot last week" and nobody knows what changed

This is the problem tagging and pinned environments solve. Tag what you deploy, record the versions
with the run, and keep the bag. → [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md), [03.07](../../03-robot-software/03.07-logging-and-telemetry.md)

---

## Displays and containers

### Symptom: GUI apps (RViz, Gazebo) fail in WSL2 or Docker

`could not connect to display` / `Authorization required` is X11 forwarding, not the application.
WSLg handles it on Windows 11; in Docker you must pass `DISPLAY` and the X socket. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md),
[FL.02](../../optional-foundations/linux-and-tools/FL.02-installing-ubuntu.md)

### Symptom: GUI apps render with `llvmpipe` and are unusably slow

Software rendering: no GPU passthrough. Acceptable for RViz, painful for Gazebo. Run Gazebo on a
machine with a GPU. → [FL.02](../../optional-foundations/linux-and-tools/FL.02-installing-ubuntu.md), [06.01](../../06-simulation/06.01-why-simulate.md)

### Symptom: `could not open port /dev/ttyACM0` inside the container

Device passthrough (`--device`) and the `dialout` group inside the container. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)

### Symptom: `ros2 topic list` shows topics from another container, but `echo` receives nothing

Discovery crossed the network boundary and data did not. Use host networking, or configure a
discovery server. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md), [04.01](../../04-ros2/04.01-why-middleware.md)

## Where to go next

- Reproducible environments, venv, PEP 668 and lockfiles: [03.02](../../03-robot-software/03.02-environments-and-pinning.md)
- Packages, workspaces, overlays and `colcon`: [04.05](../../04-ros2/04.05-packages-and-workspaces.md)
- Installing ROS 2 properly: [04.02](../../04-ros2/04.02-installing-ros2.md)
- Testing robot software without hardware: [03.08](../../03-robot-software/03.08-testing-robot-software.md)
- CI, tagging and repeatability: [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md)
- When the environment is fine and the graph is not:
  [ROS 2 discovery, QoS and timing](ros2-discovery-and-qos.md)
