# What was tested, and how

Date: 2026-09-16/17. Host: Windows 11 Pro (10.0.26200), Docker Desktop, engine 29.7.2, 16 CPUs,
15 GB RAM for the Docker VM, no GPU passthrough (Gazebo sensors rendered by Mesa llvmpipe via EGL).
The host was shared with other busy containers; the simulation ran at a real-time factor of
0.3–0.9 (`gz topic -e -t /stats`), so wall-clock rates below are lower than the simulated rates.

Image: `karmel-ros:jazzy-deps` / `karmel-ros:jazzy` built from `labs/docker/Dockerfile`
(`FROM osrf/ros:jazzy-desktop-full`, pulled 2026-09-16). Package versions in the image:
ros2_control / hardware_interface 4.48.0, gz_ros2_control 1.2.20, ros_gz_sim 1.0.22,
navigation2 1.3.13, slam_toolbox 2.8.5, teleop_twist_keyboard 2.4.1, diagnostic_updater 4.2.7.

Unless stated otherwise, commands ran in a dev container with `labs/ros2_ws/src`, `labs/config` and
`labs/python` bind-mounted (as `compose.yaml` does) and
`ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (see "Findings" for why).

## 1. Build and unit tests

```bash
colcon build --symlink-install        # 6 packages, no errors, no compiler warnings (-Wall -Wextra -Wpedantic)
colcon test && colcon test-result --verbose
python3 check_config_sync.py          # "robot config in sync"
```

Result: **65 tests, 0 errors, 0 failures, 0 skipped.**

| Suite | Tests |
|---|---|
| karmel_base (pytest) | 41: protocol copy byte-identical to `labs/python/robotlab/protocol.py`; every README wire line encodes/decodes; corruption and malformed payloads rejected; `wire.py` helpers; kinematics (saturation keeps curvature, exact-arc odometry); fake Pico model (watchdog, error codes `unknown_command`/`unknown_param`/`out_of_range`/`bad_args`), real pty round trip with pyserial; flake8; pep257 |
| karmel_hardware (gtest) | 8 protocol tests (same lines/checksums as Python, lowercase hex accepted, battery −1, flags ≤ 255) + pluginlib loads `karmel_hardware/KarmelSystem` |
| karmel_description (pytest) | 6: xacro renders (sim and real), all links present, plugins/sensors present, dimensions and total mass follow `robot.yaml`, inertia tensors physical, optical-frame rotation, config copy in sync |
| karmel_bringup (pytest) | 5: diff_drive values and `base_frame_id` match `robot.yaml`; Nav2 footprint; 0.3 m/s limit and stamped cmd_vel on every Nav2 server; EKF override |

Also: `check_urdf` parses the real URDF (tree rooted at `base_footprint`); `gz sdf -k` validates
`empty.sdf`, `apartment.sdf`, `tabletop.sdf`; `gz sdf -p` converts the sim URDF.

## 2. Simulation, headless (empty world, then apartment)

```bash
ros2 launch karmel_bringup robot.launch.py sim:=true headless:=true
```

| Check | Command | Result |
|---|---|---|
| Controllers | `ros2 control list_controllers` | joint_state_broadcaster, diff_drive_controller **active** |
| LiDAR | `ros2 topic hz /scan`; `ros2 topic echo --once /scan --field header` | 7.6 Hz wall (RTF ≈ 0.8), frame `laser` |
| Camera | `ros2 topic hz /camera/image_raw`; echo `/camera/camera_info` header | 3 Hz wall (llvmpipe), frame `camera_optical_frame` |
| IMU | echo `/imu` header | frame `imu_link` |
| cmd_vel remap | `ros2 topic info /cmd_vel -v` | `geometry_msgs/msg/TwistStamped`, subscriber `diff_drive_controller` |
| Drive | `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}, angular: {z: 0.3}}}"` for 6 s | `/odom` x 0 → 0.406, y 0.148, yaw 0.66 rad; `tf2_echo odom base_footprint` agrees; twist 0 after publishing stopped (cmd_vel timeout) |
| EKF option | `robot.launch.py sim:=true headless:=true use_ekf:=true` | `/odometry/filtered` publishing; controller `enable_odom_tf` = False; `ekf_filter_node` is the odom TF publisher |

## 3. SLAM (apartment world)

```bash
ros2 launch karmel_bringup robot.launch.py sim:=true headless:=true world:=apartment enable_camera:=false x:=-1.5 y:=-0.5
ros2 launch karmel_bringup slam.launch.py
python3 tools/drive_pattern.py --ros-args -p use_sim_time:=true     # spin 360°, forward, turn, forward
ros2 topic echo --once /map --field info
ros2 run nav2_map_server map_saver_cli -f slam_apartment --ros-args -p use_sim_time:=true
```

Result: `/map` published (109 × 99 cells at 0.05 m), `map → odom → base_footprint` TF complete,
no "Failed to compute odom pose" warnings, map saved.

## 4. Nav2 (apartment world, ground-truth map)

```bash
ros2 launch karmel_bringup navigation.launch.py initial_pose:=true x:=-0.8 y:=-0.5
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.8, y: 0.3}, orientation: {w: 1.0}}}}"
```

Result: both lifecycle managers "Managed nodes are active"; **Goal finished with status: SUCCEEDED**
(1.28 m, around the coffee-table area), controller log "Reached the goal!", final
`map → base_footprint` = (−1.736, 0.263).

## 5. Real-robot path 1: karmel_base against a fake Pico

pty (`karmel_base/fake_pico`):

```bash
ros2 run karmel_base fake_pico --link /tmp/ttyKARMEL
ros2 launch karmel_base base.launch.py port:=/tmp/ttyKARMEL
```

| Check | Result |
|---|---|
| Handshake | "connected to /tmp/ttyKARMEL", "Pico firmware fake-1.0, protocol v1" |
| `/odom` rate | 50.0 Hz |
| `/battery_state` voltage, `/range/front`, `/joint_states` names | 12.30 V, 2.0 m, `[left_wheel_joint, right_wheel_joint]` |
| Drive `0.2 m/s × 3 s`, then `1.0 rad/s × 2 s` (`tools/drive_pattern.py`) | odom x = 0.600, yaw = 2.02 rad; `odom → base_footprint` TF identical |
| cmd_vel timeout | `/odom` twist 0.30 m/s while publishing, 0 after the publisher stopped |
| `/diagnostics` | "serial connection: connected", "motors: ok", battery status |
| Serial loss | killed fake_pico → "serial connection lost: read failed: [Errno 5] Input/output error", retries every 2 s; restarted fake_pico → "connected", `/odom` back at 50 Hz |

TCP (canonical `labs/python/robotlab/fake_pico.py`):

```bash
cd ~/labs/python && python3 -m robotlab.fake_pico --port 5760
ros2 launch karmel_base base.launch.py port:=socket://localhost:5760
```

First run **failed**: the node reconnected every second ("no telemetry for 1.0 s"). Cause: it sized
reads with `in_waiting`, which pyserial's `socket://` backend reports as 0/1, so it read one byte per
cycle. Fixed (read up to 4096 bytes until drained). After the fix: firmware "fake-0.1.0, protocol v1",
`/odom` 50.0 Hz, 0.36 m/s peak for a 0.3 m/s command then 0, battery 12.23 V, range 1.825 m.

## 6. Real-robot path 2: ros2_control + karmel_hardware against the fake Pico

```bash
ros2 run karmel_base fake_pico --link /tmp/ttyKARMEL
ros2 launch karmel_bringup robot.launch.py sim:=false serial_device:=/tmp/ttyKARMEL
ros2 control list_hardware_interfaces
```

Result: KarmelSystem on_init → on_configure ("Pico firmware fake-1.0, protocol v1") → on_activate
("Receiving telemetry (battery 12.30 V)"); both controllers active; `left/right_wheel_joint/velocity`
command interfaces claimed, position/velocity states available; publishing
`TwistStamped {x: 0.3, z: 0.5}` → `/odom` linear.x peaked at 0.33 m/s and returned to 0; joint
positions advanced (19.2 / 26.9 rad, right wheel faster in a left turn).

## 7. GUI on Windows 11 (WSLg through Docker Desktop)

```bash
docker run --rm -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg \
  -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir karmel-ros:jazzy-deps rviz2
```

Result: RViz started ("OpenGl version: 4.5"). `/mnt/wslg` as the source path does **not** work with
Docker Desktop (empty directory); `/run/desktop/mnt/host/wslg` does. The Linux X11 service was not
tested (no Linux desktop available). Gazebo GUI was not tested (headless only).

## 8. CI image

```bash
cd labs
docker compose -f docker/compose.yaml build ci
docker compose -f docker/compose.yaml run --rm ci bash -c 'colcon test ...; python3 check_config_sync.py; bash tools/smoke_test.sh --nav'
```

Result (image built from this workspace; `smoke_test.sh` rerun after a parsing fix, see below):

```text
colcon test: 65 tests, 0 errors, 0 failures, 0 skipped
robot config in sync
== 1. Gazebo (headless) + controllers, apartment world
PASS  joint_state_broadcaster and diff_drive_controller active
== 2. Sensors
PASS  /scan publishing (8.363 Hz wall clock)
PASS  /scan frame_id = laser
PASS  /imu frame_id = imu_link
PASS  /camera/camera_info frame_id = camera_optical_frame
== 3. Drive with TwistStamped on /cmd_vel
PASS  /odom x moved from -9.450530766997686e-13 to 0.8079482053583625
== 4. SLAM (slam_toolbox online async)
PASS  /map published (width 108 cells)
== 5. Nav2 on the ground-truth map
PASS  NavigateToPose (-0.589, -0.646) -> (-1.8, 0.3): Goal finished with status: SUCCEEDED
SMOKE TEST PASSED
```

The first CI run reported one false FAIL in step 3: `ros2 topic echo` printed
"A message was lost!!!" before the value and the script parsed that line. The script now keeps
only numeric lines; everything else passed in both runs.

## Findings fixed while testing

1. **Split TF tree.** Odometry published `odom → base_link`, but `base_link` already has the parent
   `base_footprint` (URDF root), so slam_toolbox "Failed to compute odom pose". Now every odometry
   source (diff_drive_controller, base_node, EKF) publishes `odom → base_footprint`; a test pins it.
2. **Controller activation timed out** (`Switch controller timed out after 5 seconds`) when Gazebo
   started slowly. Spawners now use `--switch-timeout 60`.
3. **Nav2 goals aborted** with "Timed out while waiting for action server to acknowledge goal request
   for compute_path_to_pose" on a loaded machine. `bt_navigator.default_server_timeout` 20 → 200 ms.
4. **Robot drove off by itself** on domain 0: other ROS containers on the Docker bridge network were
   visible. Tests use `ROS_DOMAIN_ID` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; documented.
5. **socket:// reads** in base_node (section 5).
6. Canonical protocol: `karmel_base/protocol.py` replaced by a byte copy of robotlab's; C++ decoder
   aligned (lowercase hex, battery −1, flags ≤ 255); fake Pico now answers like the firmware
   (hello → `I` only, error codes).

## Not tested

- Real hardware (Pico, motors, RPLIDAR C1, BNO055, webcam).
- Gazebo GUI, Linux X11 compose service, arm64 builds.
- `tabletop.sdf` beyond SDF validation.
- `display.launch.py` with GUI (xacro and URDF were checked; RViz start was tested separately).
