# Docker: ROS 2 Jazzy + Gazebo Harmonic for the course

One image runs every ROS 2 lab on Windows 11, Linux and CI: ROS 2 Jazzy (Ubuntu 24.04),
Gazebo Harmonic, ros2_control + gz_ros2_control, Nav2, slam_toolbox, robot_localization,
teleop_twist_keyboard, RViz, pyserial and pytest. Base: `osrf/ros:jazzy-desktop-full`
(amd64 only; see [Raspberry Pi and Jetson](#raspberry-pi-and-jetson-arm64)).

| File | What it is |
|---|---|
| `Dockerfile` | stage `deps` = ROS + apt packages + user `ros` (dev container); stage `final` = `deps` + `colcon build` of `labs/ros2_ws` (CI) |
| `compose.yaml` | services `wslg` (Windows 11), `linux` (X11), `ci` (headless) |
| `entrypoint.sh` | sources `/opt/ros/jazzy` and the workspace, then runs your command |
| `Dockerfile.dockerignore` | only `ros2_ws/`, `config/` and the protocol module go into the build context |

Inside the container the repo layout is mirrored at `~/labs` (`~/labs/ros2_ws`, `~/labs/config`,
`~/labs/python`), so paths in lessons work the same on the host and in the container.

## Build (once)

All commands run from `labs/`:

```bash
cd labs
docker compose -f docker/compose.yaml build wslg     # the dev image, karmel-ros:jazzy-deps (~7 GB, 10–20 min)
docker compose -f docker/compose.yaml build ci       # + the workspace baked in, karmel-ros:jazzy
```

## Windows 11 (Docker Desktop + WSLg)

Requirements: Windows 11 (or Windows 10 21H2+), WSL 2 (`wsl --update`), Docker Desktop with the
WSL 2 backend. WSLg — Windows' built-in Wayland/X server for Linux apps — displays the windows;
you do not install an X server.

```powershell
cd labs
docker compose -f docker/compose.yaml run --rm wslg
```

In the container shell:

```bash
colcon build --symlink-install          # first time, and after changing C++ or package metadata
source install/setup.bash
ros2 launch karmel_description display.launch.py      # RViz window appears on the Windows desktop
```

How it works: Docker Desktop exposes WSLg's sockets inside its VM at
`/run/desktop/mnt/host/wslg`. The service mounts `…/wslg/.X11-unix` → `/tmp/.X11-unix` and
`…/wslg` → `/mnt/wslg`, and sets `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`,
`XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir`, `PULSE_SERVER=/mnt/wslg/PulseServer`.
Tested 2026-09-17 on Windows 11 + Docker Desktop (engine 29.7.2) with these mounts and variables
(`docker run`): RViz opened on the Windows desktop with OpenGL 4.5.
The harmless warning `QStandardPaths: wrong permissions on runtime directory` can be ignored.

If you run Docker Engine *inside your own WSL 2 Ubuntu* (not Docker Desktop), the sockets are at
`/mnt/wslg` instead: change the two volume sources in the `wslg` service to
`/tmp/.X11-unix:/tmp/.X11-unix` and `/mnt/wslg:/mnt/wslg`.

Rendering is software/virtual-GPU: RViz is smooth, Gazebo's GUI is usable but slow on large
worlds. Run Gazebo headless (`headless:=true`) and look at the robot in RViz when it lags.

## Linux desktop (X11)

```bash
xhost +local:                                   # once per login: allow local containers to open windows
cd labs
docker compose -f docker/compose.yaml run --rm linux
```

The `linux` service uses `network_mode: host` and `ipc: host`, mounts `/tmp/.X11-unix`, passes
`DISPLAY`, and gives the container `/dev/dri` (Intel/AMD GPU). NVIDIA: install the NVIDIA
Container Toolkit and add `gpus: all` to the service. On Wayland desktops XWayland provides
`DISPLAY`, so the same service works.

## Networking and DDS (read this before you drive a real robot)

ROS 2 nodes find each other with DDS multicast discovery. Everything with the same
`ROS_DOMAIN_ID` that can reach each other's network sees each other's topics.

- **Pick your own domain.** `ROS_DOMAIN_ID=17 docker compose … run --rm linux`. The default 0 is
  what every tutorial uses; on a shared network (or with several containers on one Docker
  network) someone else's `/cmd_vel` will move your robot. While testing this image on a machine
  with other ROS containers on the default Docker bridge, the simulated robot started driving
  with nobody commanding it in its own container; a private domain fixed it.
- **Stay private:** `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keeps discovery inside the
  container (the `ci` service and `tools/smoke_test.sh` do this).
- **Talking to the Raspberry Pi from a Linux laptop:** the `linux` service uses host networking,
  so it joins your LAN; set the same `ROS_DOMAIN_ID` on the Pi and the laptop. Multicast must be
  allowed on the Wi-Fi (many guest/mesh networks block it).
- **From Windows:** Docker Desktop's "host" network is its VM, not your LAN, so DDS discovery
  cannot reach the Pi from a Docker Desktop container. For the real robot, use ROS natively in a
  WSL 2 Ubuntu 24.04 with mirrored networking (`networkingMode=mirrored` in `%UserProfile%\.wslconfig`),
  or run the GUI tools on the Pi side via SSH, or use Foxglove over a websocket.
- **Shared memory:** `shm_size: 1gb` — Gazebo and Fast DDS shared-memory transport fail in odd ways
  with Docker's default 64 MB.

## Headless mode and CI

No display needed: Gazebo runs server-only (`-s --headless-rendering`) and renders the GPU LiDAR
and camera with EGL (Mesa llvmpipe on machines without a GPU).

```bash
cd labs
docker compose -f docker/compose.yaml run --rm ci
```

runs `colcon test`, `check_config_sync.py`, and `tools/smoke_test.sh` (Gazebo headless in the
apartment world → controllers active → `/scan`, `/imu`, `/camera/camera_info` frames → drive with
`TwistStamped` → `/odom` moves → slam_toolbox publishes `/map`). Add Nav2:

```bash
docker compose -f docker/compose.yaml run --rm ci bash tools/smoke_test.sh --nav
```

Or in any dev container: `ros2 launch karmel_bringup robot.launch.py sim:=true headless:=true`.

CPU-only machines: the camera costs the most. `enable_camera:=false` raised the real-time factor
from ~0.35 to ~0.7 in the apartment world on the test machine.

## Serial devices (real hardware from a Linux PC)

Uncomment `group_add: [dialout]` and the `/dev/ttyACM0` device line in the `linux` service.
Docker Desktop on Windows cannot pass USB serial devices into containers; use
[usbipd-win](https://learn.microsoft.com/windows/wsl/connect-usb) with native WSL, or run
`karmel_base` on the Pi (the normal setup).

## Raspberry Pi and Jetson (arm64)

`osrf/ros:jazzy-desktop-full` is amd64 only. On the robot use `ros:jazzy-ros-base` (amd64 +
arm64) — no Gazebo or RViz there:

```bash
docker build -f docker/Dockerfile --target deps --build-arg BASE_IMAGE=ros:jazzy-ros-base -t karmel-ros:jazzy-robot .
```

(Not tested on arm64. The apt list includes simulation packages, so expect a larger image than needed;
on the Pi the course recommends native Ubuntu 24.04 + apt ROS 2, lesson 04.02.)

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `could not connect to display` | Linux: run `xhost +local:`. Windows: WSL 2 not updated (`wsl --update`) or Docker Desktop not using the WSL 2 backend |
| Robot moves by itself | Another ROS system on your domain. Set `ROS_DOMAIN_ID` or `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| `Switch controller timed out` | Gazebo started slowly; `robot.launch.py` already waits 60 s. Close other heavy containers |
| `/scan` rate far below 10 Hz | Sim runs slower than real time (software rendering). Check `gz topic -e -t /stats -n 1`; use `enable_camera:=false` |
| `Permission denied: 'log'` in `colcon build` | Workspace directory owned by root: `sudo chown -R ros:ros ~/labs/ros2_ws` |
