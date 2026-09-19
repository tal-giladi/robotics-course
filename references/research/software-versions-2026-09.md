# Robotics software versions — research snapshot (2026-09-16)

Every row carries its source. "verified 2026-09" = fetched from the source on 2026-09-16. "UNVERIFIED" = could not confirm from an official source; treat as a lead, not a fact.
Binary availability was checked directly against the apt indexes on packages.ros.org (`ros2/ubuntu/dists/{noble,resolute,jammy}/main/binary-{amd64,arm64}/Packages.gz`) and `ros2-testing`, plus `ros/rosdistro` `*/distribution.yaml` (master). That is stronger evidence than index.ros.org summaries, which fall back to other distros' tabs.

## Recommendation for this course

| Item | Pick | Why (source) |
|---|---|---|
| ROS 2 distro | **Jazzy Jalisco** (LTS, EOL May 2029) | Only LTS with the *complete* course stack as apt binaries on amd64 + arm64 today: navigation2/nav2_bringup, slam_toolbox, MoveIt, ros2_control, gz_ros2_control, TurtleBot 3 + 4 sim, micro-ROS branches, Isaac ROS. Lyrical is newer but, as of 2026-09-16, `ros-lyrical-navigation2` and `ros-lyrical-nav2-bringup` are **not in the main apt repo** (only in ros2-testing, built 2026-09-15; open issue github.com/ros/rosdistro/issues/53937). Also missing on Lyrical: turtlebot4, cartographer_ros, rplidar_ros, micro_ros_setup branch, Isaac ROS support. MoveIt tutorials recommend Jazzy on 24.04 (moveit.picknik.ai). verified 2026-09 |
| Ubuntu | **24.04 LTS Noble** (amd64 desktop; arm64 on Pi 5 / Jetson) | Jazzy Tier 1 = Ubuntu 24.04 amd64+arm64 (ros2_documentation Release-Jazzy-Jalisco.rst). verified 2026-09 |
| Gazebo | **Harmonic (LTS)** via `ros-jazzy-ros-gz` | Official pairing: Jazzy ↔ Harmonic, "recommended" (gazebosim.org/docs/latest/ros_installation). verified 2026-09 |
| Python | **3.12.3** (system python on 24.04) | REP-2000 Jazzy table + packages.ubuntu.com/noble/python3. verified 2026-09 |
| Nav2 | **1.3.13** (`ros-jazzy-navigation2`) | packages.ros.org noble main, amd64+arm64. verified 2026-09 |
| MoveIt 2 | **2.12.4** (`ros-jazzy-moveit`) | packages.ros.org noble main. verified 2026-09 |
| ros2_control / ros2_controllers | **4.48.x / 4.42.1** | packages.ros.org noble main. verified 2026-09 |
| slam_toolbox | **2.8.5** | packages.ros.org noble main. verified 2026-09 |
| Plan to revisit | Lyrical Luth + Ubuntu 26.04 + Gazebo Jetty for the **next** course run, once Nav2 metapackage/bringup, TurtleBot, micro-ROS and Isaac ROS land | see gaps above |

---

## 1. ROS 2 distributions

| Distro | Released | EOL | LTS | Tier 1 platforms | Python (Tier-1 Ubuntu) | Source |
|---|---|---|---|---|---|---|
| Humble Hawksbill | 2022-05-23 | May 2027 | yes | Ubuntu 22.04 amd64+arm64, Windows 10 amd64 (REP lists RHEL 8 as Tier 2) | 3.10.4 | raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Releases.rst ; raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2000.rst — verified 2026-09 |
| Jazzy Jalisco | 2024-05-23 | May 2029 | yes | Ubuntu 24.04 amd64+arm64, Windows 10 (VS2019) amd64; Tier 2 RHEL 9; Tier 3 macOS, Debian Bookworm, Ubuntu 22.04 (src) | 3.12.3 | Releases.rst; source/Releases/Release-Jazzy-Jalisco.rst — verified 2026-09 |
| Kilted Kaiju | 2025-05-23 | **Dec 2026** per docs Releases.rst; **Nov 2026** per REP-2000 (conflict — plan for Nov 2026) | no | Ubuntu 24.04 amd64+arm64, Windows 10 amd64 | 3.12.3 | Releases.rst; reps.openrobotics.org/rep-2000 — verified 2026-09 |
| Lyrical Luth | 2026-05-22 | May 2031 | yes | Ubuntu 26.04 "Resolute" amd64+arm64, Windows 11 (VS2022) amd64 | 3.14.3 (python3 on resolute) | discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021 ; source/Releases/Release-Lyrical-Luth.rst ; packages.ubuntu.com/resolute/python3 — verified 2026-09 |
| Rolling Ridley | continuous | n/a | n/a | Moved to Ubuntu 26.04 (Resolute). ros2-testing already has 2205 `ros-rolling-*` debs for resolute; main repo still showed Rolling on noble at check time (mid-transition) | — | packages.ros.org indexes; github.com/ros2/ci/issues/838 — verified 2026-09 |
| Makoa Mata-mata (next) | May 2027 | Dec 2028 | no | — | — | Releases.rst — verified 2026-09 |

- Iron EOL 2024-12-04; Galactic/Foxy long EOL (Releases.rst). verified 2026-09
- New learner in Sept 2026 → **Jazzy** (see table at top). Humble is still LTS but EOL May 2027, is on Ubuntu 22.04 and pairs with Gazebo Fortress; do not start a new course on it. Kilted EOL is weeks away. Lyrical is the future but the navigation stack apt binaries are incomplete in main (checked 2026-09-16).
- Lyrical-specific learner-visible features: rclpy AsyncNode / asyncio, EventsCBGExecutor ("10% to 15% less CPU"), rosbag2 Python API for record/play, `setup.fish` (Release-Lyrical-Luth.rst). verified 2026-09

## 2. Gazebo (new Gazebo, formerly Ignition)

| Release | Released | EOL | LTS | Source |
|---|---|---|---|---|
| Fortress | Sep 2021 | May 2027 | yes | gazebosim.org/docs/latest/releases — verified 2026-09 |
| Garden | Sep 2022 | Nov 2024 (EOL) | no | same |
| Harmonic | Sep 2023 | May 2029 | yes | same |
| Ionic | Sep 2024 | Dec 2026 | no | same |
| Jetty | Sep 2025 | May 2031 | yes | same |
| "M" | Mar 2027 | Dec 2028 | no | same |
| Gazebo Classic (gazebo11) | — | **EOL 2025-01-29** | — | classic.gazebosim.org ; discourse.openrobotics.org/t/gazebo-classic-end-of-life/48748 — verified 2026-09 |

Note: the releases page as fetched shows Fortress EOL May 2027 and Harmonic EOL May 2029 (older docs said Sep 2026 / Sep 2028). Treat the page values as current.

Official ROS ↔ Gazebo pairing (gazebosim.org/docs/latest/ros_installation, verified 2026-09):

| ROS 2 | Recommended Gazebo | Install |
|---|---|---|
| Humble | Fortress (Harmonic "possible with caution", via `ros-humble-ros-gzharmonic` from packages.osrfoundation.org) | `ros-humble-ros-gz` |
| Jazzy | **Harmonic** (vendor packages from packages.ros.org) | `sudo apt install ros-jazzy-ros-gz` |
| Kilted | Ionic | `ros-kilted-ros-gz` |
| Lyrical | Jetty | `ros-lyrical-ros-gz` |
| Rolling | Jetty | `ros-rolling-ros-gz` |

- Since Jazzy, Gazebo libs come to ROS as `gz_*_vendor` packages (gz_sim_vendor, gz_cmake_vendor, sdformat_vendor …) — Release-Jazzy-Jalisco.rst. verified 2026-09
- Standalone Harmonic binaries (`gz-harmonic`) exist for Ubuntu 22.04 and 24.04; cannot co-install with gazebo11 by default (gazebosim.org/docs/harmonic/install_ubuntu). verified 2026-09
- Package names today (jazzy `ros_gz` 1.0.24): `ros_gz`, `ros_gz_bridge`, `ros_gz_sim`, `ros_gz_image`, `ros_gz_interfaces`, `ros_gz_sim_demos` (rosdistro jazzy/distribution.yaml). verified 2026-09
- `ros_ign_*` shim packages exist **only in Humble** (ros_ign, ros_ign_bridge, ros_ign_gazebo …); they are gone from Jazzy and Lyrical releases (rosdistro yaml). README: "ros_ign prefixed packages are shim packages that redirect to their ros_gz counterpart" (github.com/gazebosim/ros_gz). verified 2026-09
- `ign_ros2_control` exists only on Humble; Jazzy+ = `gz_ros2_control` (rosdistro yaml). verified 2026-09
- Gazebo Classic ROS packages: `gazebo_ros_pkgs` has a jazzy rosdistro entry but **no `ros-jazzy-gazebo-ros` binary** in noble main; no entry at all on Lyrical. verified 2026-09
- gz_ros2_control (jazzy branch, github.com/ros-controls/gz_ros2_control): matrix Jazzy↔Harmonic (`ros-jazzy-gz-ros2-control`), Kilted↔Ionic, Lyrical/Rolling↔Jetty, Humble↔Harmonic source-only. URDF: `<ros2_control name="GazeboSimSystem" type="system"><hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin>` and `<plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">`. verified 2026-09
- Official diff-drive examples:
  - `ros2 launch gz_ros2_control_demos diff_drive_example.launch.py` (+ `ros2 run gz_ros2_control_demos example_diff_drive`) — github.com/ros-controls/gz_ros2_control/tree/jazzy/gz_ros2_control_demos/launch. verified 2026-09
  - `ros2 launch ros_gz_sim_demos diff_drive.launch.py` — github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos/launch. verified 2026-09
  - Project template: github.com/gazebosim/ros_gz_project_template. verified 2026-09

## 3. Binary availability (apt, packages.ros.org main, checked 2026-09-16)

Legend: version = binary present; — = absent from main; (T) = present only in ros2-testing.

| Package (deb name suffix) | Jazzy noble amd64 | Jazzy noble arm64 | Lyrical resolute amd64/arm64 | Notes / source |
|---|---|---|---|---|
| navigation2 / nav2-bringup | 1.3.13 | 1.3.13 | — (T 1.5.1, built 2026-09-15). Most nav2-* components 1.5.1 in main, but metapackage, nav2_bringup, nav2_smac_planner missing | rosdistro issue #53937 open 2026-09-14 |
| nav2-minimal-tb3-sim / tb4-sim | 1.0.1 | 1.0.1 | 1.3.0 | nav2 sims moved to nav2_minimal_turtlebot_simulation |
| slam-toolbox | 2.8.5 | 2.8.5 | 2.10.0 | index.ros.org/p/slam_toolbox |
| robot-localization | 3.8.3 | 3.8.3 | 3.10.0 | index.ros.org/p/robot_localization |
| ros2-control | 4.48.0 | 4.48.0 | 6.9.0 | index.ros.org/p/ros2_control |
| ros2-controllers / diff-drive-controller | 4.42.1 | 4.42.1 | 6.9.0 | index.ros.org/p/diff_drive_controller |
| gz-ros2-control | 1.2.20 | 1.2.20 | 3.0.8 | index.ros.org/p/gz_ros2_control |
| ros-gz / ros-gz-sim / ros-gz-bridge | 1.0.24 | 1.0.24 | 3.0.9 | |
| moveit (+ moveit-py) | 2.12.4 | 2.12.4 | 2.15.0 (T 2.15.1) | index.ros.org/p/moveit; tutorials build from source or Docker (moveit.picknik.ai/main/doc/tutorials/getting_started) |
| cartographer-ros | 2.0.9003 | 2.0.9003 | — | status "maintained"; last upstream release tag 2024-04; no Lyrical binary — treat as legacy, prefer slam_toolbox |
| turtlebot3 (meta) / turtlebot3-gazebo | 2.3.6 / 2.3.7 | same | meta —, turtlebot3-gazebo 2.3.7 | TB3 meta source-only on Lyrical |
| turtlebot4-simulator | 2.0.2 | 2.0.2 | — (not in rosdistro for Kilted/Lyrical) | |
| rplidar-ros (Slamtec) | 2.1.0 | 2.1.0 | — | |
| sllidar_ros2 | not in rosdistro — source only | | | github.com/Slamtec/sllidar_ros2 (last push 2024-07) |
| ldlidar (ldlidar_stl_ros2) | not in rosdistro — source only | | | github.com/ldrobotSensorTeam/ldlidar_stl_ros2 (last push 2024-02) |
| v4l2-camera | 0.7.3 | 0.7.3 | 0.8.1 | |
| camera-ros (libcamera node) | 0.7.0 | 0.7.0 | 0.7.0 | pulls `ros-<distro>-libcamera` 0.7.2 (upstream, not RPi fork) — see §4 |
| depthai-ros-driver | 2.12.2 (v2 API); `depthai_ros_v3` 3.2.1 separate repo | same | — (T 3.4.0) | |
| realsense2-camera | 4.58.4 | 4.58.4 | — (T 4.58.4) | |
| micro-ros-agent | **no apt binary on any distro** (not in rosdistro) | | | build via micro_ros_setup or Docker `microros/micro-ros-agent:jazzy` |
| imu-tools | 2.1.5 | 2.1.5 | 2.2.2 | |
| image-pipeline | 5.0.13 | 5.0.13 | 7.1.7 | |
| vision-msgs | 4.1.1 | 4.1.1 | 4.2.0 | |
| behaviortree-cpp (v4) | 4.10.0 | 4.10.0 | 4.10.0 | Nav2 Jazzy requires BT.CPP 4.5+ |
| py-trees-ros | 2.5.0 | 2.5.0 | 2.5.0 | |
| foxglove-bridge | 3.5.0 | 3.5.0 | 3.4.3 | now released from github.com/foxglove/foxglove-sdk (repo `foxglove-sdk`, tag ros-v3.5.0 2026-08-26) |
| rosbag2-storage-mcap | 0.26.11 | 0.26.11 | 0.33.3 | MCAP is default (see §8) |
| rmw-zenoh-cpp | 0.2.10 | 0.2.10 | 0.10.5 | Tier 1 since Kilted |
| desktop / ros-base | 0.11.0 | 0.11.0 | 0.13.0 | |

All rows: packages.ros.org/ros2/ubuntu/dists/*/main Packages.gz + github.com/ros/rosdistro master — verified 2026-09.
Humble sanity check (jammy amd64): nav2-bringup 1.1.20, moveit 2.5.10, slam-toolbox 2.6.10, turtlebot4-simulator 1.0.3, ros-gz-sim 0.244.26 — verified 2026-09.

## 4. Raspberry Pi 5

| Fact | Source |
|---|---|
| ROS docs: arm64 is Tier 1, arm32 Tier 3. Two supported paths: **(a) 64-bit Ubuntu on the Pi + binary ROS**, or **(b) 64-bit Raspberry Pi OS + ROS in Docker** (Pi OS is Debian → Tier 3 natively). | raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Get-Started/Installation/Installing-on-Raspberry-Pi.rst — verified 2026-09 |
| Ubuntu for Pi images lack `-updates`/`-backports` suites by default; add them to `/etc/apt/sources.list.d/ubuntu.sources` before installing ROS. | same — verified 2026-09 |
| Pi 5 is supported (server + desktop, certified "SDC") on Ubuntu **24.04 LTS** and 26.04 LTS; not on 22.04. Pi 500 / CM5: S D. 26.04 needs Pi 5 EEPROM ≥ 2025-02-11. | ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/ — verified 2026-09 |
| Current image: `ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz` ("Pi 3, 4, 5, CM4, Zero 2 W"). ubuntu.com/download/raspberry-pi now headlines 26.04.1 LTS. | cdimage.ubuntu.com/releases/24.04/release/ ; ubuntu.com/download/raspberry-pi — verified 2026-09 |
| **Camera caveat:** Ubuntu docs: "The libcamera stack is not currently operational on Ubuntu releases before 25.04"; "Since Ubuntu 25.04, all Raspberry Pi models with a CSI port can use the camera stack" (`rpicam-apps` in archive, picamera2 via PPA). ⇒ On **24.04 + Jazzy**, CSI camera needs a workaround. | ubuntu.com/hardware/docs/boards/how-to/special_hardware/rpi-camera/ — verified 2026-09 |
| camera_ros: apt `ros-$ROS_DISTRO-camera-ros` pulls upstream libcamera that "may not contain full support for all Raspberry Pi camera modules"; for full support build the `raspberrypi` libcamera fork from source in the colcon workspace ("the only option for using the 'raspberrypi' fork on Ubuntu"). | raw.githubusercontent.com/christianrauch/camera_ros/main/README.md — verified 2026-09 |
| Pi OS uses Raspberry Pi's libcamera fork; apps are `rpicam-*` (formerly libcamera-*) since Bookworm; Pi 5 has two MIPI camera connectors. Current Pi OS = Trixie (Debian 13). | raspberrypi.com/documentation/computers/camera_software.html ; raspberrypi.com/news/trixie-the-new-version-of-raspberry-pi-os/ — verified 2026-09 |
| Practical options for Pi camera + Jazzy: (1) Ubuntu 24.04 + camera_ros built against raspberrypi/libcamera fork; (2) Pi OS Trixie host + `ros:jazzy-ros-base` container with device passthrough; (3) USB UVC camera + `v4l2_camera`/`usb_cam` (avoids the issue). Which of (1)/(2) works reliably for Camera Module 3 on Pi 5 — **UNVERIFIED** (no official end-to-end doc). | synthesis of above |
| GPIO: gpiozero docs: "**Only lgpio works on the Pi 5**"; RPiGPIOFactory marked "(not Pi 5)". Default factory order: lgpio, rpigpio, pigpio, native. ⇒ **RPi.GPIO does not work on Pi 5**; use `gpiozero` (lgpio backend) or `lgpio` directly. | gpiozero.readthedocs.io/en/latest/api_pins.html — verified 2026-09 |
| Availability of `python3-lgpio` / `python3-gpiozero` debs in Ubuntu 24.04 arm64 archive | UNVERIFIED (not checked) |

## 5. NVIDIA Jetson

| Fact | Source |
|---|---|
| Latest JetPack: **7.2.1** (Jetson Linux / L4T **39.2.1**), supports AGX Thor, T5000, T4000 **and Jetson Orin family**. JetPack 7 base: **Ubuntu 24.04 LTS**, Linux 6.8, CUDA 13.0. | developer.nvidia.com/embedded/jetpack-archive ; developer.nvidia.com/embedded/jetpack — verified 2026-09 |
| Orin Nano (Super) Dev Kit user guide targets JetPack 7.2.1. **No more microSD image** from 7.2: write unified ISO to USB stick, install to microSD/NVMe. Requires JetPack-6-generation UEFI/QSPI firmware first (factory 36.0 or older → do JP6 update path). Power mode "MAXN SUPER" selectable. | docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html — verified 2026-09 |
| Previous line JetPack 6.2.3 (L4T 36.5.2, Ubuntu 22.04) still listed for Orin. | jetpack-archive — verified 2026-09 (Ubuntu 22.04 base from memory: UNVERIFIED) |
| Isaac ROS latest **4.6.0** (2026-08-18; isaac_ros_common tag v4.6-0): "designed and tested to be compatible with **ROS 2 Jazzy**"; Jetson Thor + Orin on **JetPack 7.2**; x86: Ampere+ GPU, Ubuntu 24.04, CUDA 13.2+, driver 595+; Jetson needs 128 GB+ NVMe. Runs via Isaac ROS dev containers. | nvidia-isaac-ros.github.io/getting_started/ ; nvidia-isaac-ros.github.io/releases/ ; github.com/NVIDIA-ISAAC-ROS/isaac_ros_common — verified 2026-09 |
| Isaac Sim latest **6.1.0** (GitHub release 2026-09-10; 6.0.0 2026-06-04). Req: Ubuntu 22.04/24.04 or Windows 11, x86_64; min RTX 4080 16 GB VRAM, 32 GB RAM, 50 GB SSD; driver ≥ 595.58.03 (Linux); no GPUs without RT cores; aarch64 only on DGX Spark. ROS 2 bridge: Humble and Jazzy (Jazzy recommended on 24.04); bundled ROS libs use Python 3.12; WSL2 path deprecated. | github.com/isaac-sim/IsaacSim/releases ; docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html ; …/install_ros.html — verified 2026-09 |
| Isaac Lab: last stable **2.3.2** (2026-02-02); **3.0.0-beta2(.patch1)** (2026-07-02) is the default branch. Docs "main" page still says Isaac Sim 5.1.0, Python 3.11, Ubuntu 22.04/Win 11, 32 GB RAM, 16 GB VRAM. Which Isaac Sim version 3.0 beta pairs with — UNVERIFIED. | github.com/isaac-sim/IsaacLab/releases ; isaac-sim.github.io/IsaacLab/main/source/setup/installation/ — verified 2026-09 |

## 6. Docker images and Windows

| Fact | Source |
|---|---|
| Official image `ros` (Docker Official Images): tags `jazzy`, `jazzy-ros-core(-noble)`, `jazzy-ros-base(-noble)`, `jazzy-perception(-noble)`; **amd64 + arm64**; rebuilt 2026-09-16. | hub.docker.com/_/ros (API v2 tags) — verified 2026-09 |
| `osrf/ros`: `jazzy-desktop(-noble)`, `jazzy-desktop-full(-noble)`, `jazzy-simulation(-noble)`; **amd64 only**; updated 2026-09-10. | hub.docker.com/r/osrf/ros — verified 2026-09 |
| ROS docs recommend `ros:{DISTRO}-ros-core` / ros-base / perception on Raspberry Pi OS. Variants defined by REP-2001. | Installing-on-Raspberry-Pi.rst — verified 2026-09 |
| micro-ROS agent image `microros/micro-ros-agent` tags: humble, jazzy, kilted, rolling (no lyrical). | hub.docker.com/r/microros/micro-ros-agent — verified 2026-09 |
| WSLg: Linux GUI apps (X11 + Wayland) on **Windows 11** or Windows 10 build 19044+, WSL 2 only, needs vGPU driver (Intel/AMD/NVIDIA) for HW-accelerated OpenGL; `wsl --install` / `wsl --update`. | learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps — verified 2026-09 |
| Assessment: Windows 11 + WSL2 (Ubuntu 24.04) + apt Jazzy is a reasonable path for RViz/rqt/Gazebo GUI via WSLg. WSL is **not** a ROS platform tier (ROS Tier 1 on Windows is native Windows 10 binaries, Windows 11 from Lyrical). Gazebo rendering performance under WSLg, and Docker Desktop containers reaching WSLg display (mounting `/tmp/.X11-unix`, `/mnt/wslg`) — **UNVERIFIED**, no official ROS/Gazebo doc found. Isaac Sim marks its WSL2 ROS path as deprecated. | synthesis; REP-2000; Isaac Sim install_ros.html |

## 7. micro-ROS

| Fact | Source |
|---|---|
| micro_ros_setup branches: humble, iron, **jazzy**, kilted (default), rolling — **no lyrical branch**; latest tag 6.1.0 (2026-08-11). README distro table: Humble/Iron/Jazzy/Rolling supported. | api.github.com/repos/micro-ROS/micro_ros_setup ; raw README (jazzy branch) — verified 2026-09 |
| micro-ROS-Agent has a `lyrical` branch (agent only). | github.com/micro-ROS/micro-ROS-Agent branches — verified 2026-09 |
| Raspberry Pi Pico (RP2040): github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk — branches humble, iron, jazzy, kilted, rolling. | GitHub API — verified 2026-09 |
| ESP32: github.com/micro-ROS/micro_ros_espidf_component — branches humble, iron, jazzy, kilted, rolling (+ support-esp32p4 WIP). | GitHub API — verified 2026-09 |
| Arduino: micro_ros_arduino releases v2.0.8-jazzy/-kilted/-rolling (2025-09-30). PlatformIO: micro_ros_platformio (main, last push 2025-09). | GitHub API — verified 2026-09 |
| RTOS: FreeRTOS and Zephyr primary; boards incl. ESP32, ST Nucleo, Olimex STM32-E407; community platforms "may have lack of official support". | micro_ros_setup README — verified 2026-09 |
| Build/run agent: `ros2 run micro_ros_setup create_agent_ws.sh` → `build_agent.sh` → `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0` (or Docker). | micro_ros_setup README — verified 2026-09 (serial args: standard usage, exact flags UNVERIFIED) |
| micro.ros.org hardware page (`/docs/overview/hardware/`) returned 404; Pico 2 (RP2350) support — UNVERIFIED. | — |

## 8. Deprecations / gotchas a course must warn about

| Item | Detail | Source |
|---|---|---|
| Gazebo Classic | EOL 2025-01-29; no `gazebo_ros` binaries for Jazzy/Noble or Lyrical. Old tutorials using `gazebo_ros`, `spawn_entity.py`, `libgazebo_ros_diff_drive.so` do not apply. Use `ros_gz_sim` (`gz_sim.launch.py`, `create`) + `ros_gz_bridge`. | §2 sources — verified 2026-09 |
| ros_ign → ros_gz | `ros_ign_*` shims only on Humble; `ign_ros2_control` → `gz_ros2_control`; `ign gazebo` CLI → `gz sim`. | rosdistro yaml; ros_gz README — verified 2026-09 (CLI rename from memory: UNVERIFIED here) |
| rosbag2 default storage | Since **Iron**, `ros2 bag record` writes **MCAP** by default; sqlite3 still selectable with `--storage sqlite3`; reading auto-detects. | Release-Iron-Irwini.rst ("switches to using mcap as the default"); github.com/ros2/rosbag2 README — verified 2026-09 |
| RPi.GPIO on Pi 5 | Does not work; use gpiozero + lgpio. | gpiozero docs — verified 2026-09 |
| Pi camera on Ubuntu 24.04 | libcamera stack not operational before Ubuntu 25.04. | Ubuntu hardware docs — verified 2026-09 |
| diff_drive_controller (Humble→Jazzy) | `~/cmd_vel` **must be `TwistStamped`** (use_stamped_vel gone); `wheels_per_side` removed; limit booleans deprecated (set limits to NaN). teleop_twist_keyboard needs `stamped:=true` — that param name UNVERIFIED. | github.com/ros-controls/ros2_controllers/blob/jazzy/doc/release_notes.rst — verified 2026-09 |
| joint_trajectory_controller (Humble→Jazzy) | `angle_wraparound` param removed (auto from continuous joints); `start_with_holding` removed; `allow_nonzero_velocity_at_trajectory_end` default false. | same — verified 2026-09 |
| ros2_controllers on Lyrical | Release no longer contains `effort_controllers`, `position_controllers`, `velocity_controllers`, `gripper_controllers` packages (present in Jazzy). Replacement guidance UNVERIFIED (likely forward_command_controller / parallel_gripper_controller). | rosdistro jazzy vs lyrical package lists — verified 2026-09 |
| Nav2 Iron→Jazzy | BT.CPP 4.5+ (XML format v4); `plugin_lib_names` now only for *custom* BT nodes; plugin names standardized to `::` (e.g. `nav2_navfn_planner::NavfnPlanner`, `nav2_behaviors::Spin`); collision monitor polygon `points` now a string `"[[x,y],...]"`; RPP `use_interpolation` removed; Smac `viz_expansions` → `debug_visualizations`; new `enable_stamped_cmd_vel` (default false in Jazzy); TB3/TB4 sims moved to `nav2_minimal_turtlebot_simulation` on new Gazebo. | github.com/ros-navigation/docs.nav2.org (rolling) …/migration_guides/iron/Iron.md — verified 2026-09 |
| Nav2 Jazzy→Kilted (future) | `cmd_vel` defaults to **TwistStamped**; global costmap `map_topic` removed (set on StaticLayer); BT nodes use `nav_msgs/Goals`. | …/migration_guides/jazzy/Jazzy.md — verified 2026-09 |
| Nav2 Kilted→Lyrical (future) | New `nav2_ros_common` / Nav2 LifecycleNode API; `action_server_result_timeout` removed; RPP `stateful` removed; `TruncatePathLocal` `robot_frame` → `robot_base_frame`. | …/migration_guides/kilted/Kilted.md — verified 2026-09 |
| rclcpp (Jazzy) | Old `callback(std::shared_ptr<MessageT>)` subscription signatures removed; `rclcpp/qos_event.hpp` removed. | Release-Jazzy-Jalisco.rst — verified 2026-09 |
| Kilted EOL | Nov/Dec 2026 — do not teach on Kilted. | §1 |
| Cartographer | Binaries on Jazzy but no upstream development; no Lyrical binary. Teach slam_toolbox. | §3 |
| JetPack 7.2 install | No SD-card image for Orin Nano; USB ISO + JP6 firmware prerequisite. | §5 |

## 9. Python

| Fact | Source |
|---|---|
| Ubuntu 24.04 python3 = **3.12.3**; Jazzy/Kilted built against 3.12.3. Ubuntu 26.04 python3 = 3.14.3 (Lyrical). Humble/22.04 = 3.10. | packages.ubuntu.com/noble/python3 ; packages.ubuntu.com/resolute/python3 ; REP-2000 — verified 2026-09 |
| PEP 668: system Python on Ubuntu 24.04 carries an `EXTERNALLY-MANAGED` marker, so `pip install` (incl. `--user`) outside a venv is refused ("externally-managed-environment"). | peps.python.org/pep-0668/ ; packaging.python.org/en/latest/specifications/externally-managed-environments/ ; ubuntu.com/developers/docs/tutorials/python-use/ — verified 2026-09 |
| ROS docs, order of preference for extra Python deps: (1) **rosdep key** in `package.xml` + `rosdep install`; (2) **apt** `python3-<pkg>`; (3) pip. Binaries require the **system interpreter**; a venv must use system python; **conda is likely incompatible**. Venv recipe: create venv inside workspace, `touch venv/COLCON_IGNORE`, pip install, source `/opt/ros/jazzy/setup.bash`, `colcon build`. | raw.githubusercontent.com/ros2/ros2_documentation/rolling/source/Developer-Tools/Build/Using-Python-Packages.rst — verified 2026-09 |
| rosdep and pip keys after PEP 668: rosdep docs **recommend** `export PIP_BREAK_SYSTEM_PACKAGES=1` (or `/etc/pip.conf` `[install] break-system-packages = true`) so `rosdep install` can pip-install globally. | github.com/ros-infrastructure/rosdep/blob/master/doc/pip_and_pep_668.rst (docs.ros.org copy blocked by bot protection) — verified 2026-09 |
| Course practice (recommendation): prefer apt/rosdep; for pip-only deps use `python3 -m venv --system-site-packages` so rclpy and `/opt/ros` packages stay visible, add `COLCON_IGNORE`, and run nodes with that venv active. The `--system-site-packages` detail and making `ros2 run` pick the venv interpreter (e.g. `colcon build` from inside the active venv, or `[build_scripts] executable=/usr/bin/env python3` in setup.cfg) are community practice — **UNVERIFIED** in official ROS docs. | synthesis |

## Access notes

- docs.ros.org, www.ros.org/reps, and docs.ros.org/en/independent (rosdep) returned Anubis bot-protection pages; the same content was read from the GitHub sources (ros2/ros2_documentation, ros-infrastructure/rep, ros-infrastructure/rosdep).
- docs.nav2.org migration URLs 404 under the old paths; content read from github.com/ros-navigation/docs.nav2.org (branch `rolling`, `docs/configuration_and_development/migration_guides/`).
- index.ros.org summaries sometimes show another distro's version when a distro has no release; all availability claims above come from the apt indexes instead.
