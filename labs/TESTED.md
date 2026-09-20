# labs/ — what was built, what was run, what passed

This file records a verification pass over `labs/` done on **2026-09-20**. It is the evidence
behind the claim that the lab code works; `labs/ros2_ws/TESTED.md` records the earlier,
more detailed pass over the ROS 2 workspace's behaviour.

> [!NOTE]
> Everything below ran in Docker on one Windows 11 host. Nothing here was run on the real
> robot — see [What is untested](#what-is-untested-and-why).

## Environment

| | |
|---|---|
| Host | Windows 11 Pro 10.0.26200, Docker Desktop engine 29.7.2, no GPU passthrough |
| Image | `karmel-ros:jazzy-deps`, built from `labs/docker/Dockerfile` (`FROM osrf/ros:jazzy-desktop-full`), image id `sha256:47b7748b…`, created 2026-09-17 |
| OS in image | Ubuntu 24.04.4 LTS, Python 3.12.3 |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Gazebo Sim 8.11.0 (Harmonic) |
| Packages | navigation2 1.3.13, slam_toolbox 2.8.5, ros2_control 4.48.0, ros2_controllers 4.42.1, ros_gz_sim 1.0.22, gz_ros2_control 1.2.20, robot_localization 3.8.3 |

These match `curriculum/versions.yaml` (ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic,
navigation2 1.3.x, slam_toolbox 2.8.x, ros2_control 4.48.x, ros2_controllers 4.42.x,
gz_ros2_control 1.2.x).

Every container was isolated: `--network none`, `ROS_DOMAIN_ID=73`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `--shm-size 1gb`. Sources were bind-mounted
read-only and copied into the container's workspace, so nothing in the repo was written by a
container. Rendering was software (Mesa llvmpipe via EGL), so the simulation ran below real
time and the wall-clock rates below are lower than the simulated rates.

## 1. Build and unit tests

```bash
docker run -d --name wsverify-dev --network none --shm-size 1gb \
  -e ROS_DOMAIN_ID=73 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v <repo>/labs:/host/labs:ro karmel-ros:jazzy-deps sleep infinity
# inside: cp /host/labs/{ros2_ws/src,ros2_ws/tools,config,python/robotlab/protocol.py} into ~/labs
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --return-code-on-test-failure && colcon test-result
python3 check_config_sync.py
```

| | Before the fixes | After the fixes |
|---|---|---|
| `colcon build` | 6 packages, exit 0 | 6 packages, exit 0 |
| `colcon test` | **65 tests, 0 errors, 0 failures, 0 skipped** | **69 tests, 0 errors, 0 failures, 0 skipped** |
| `check_config_sync.py` | `robot config in sync` | `robot config in sync` |

The four new tests are the regression guards for the defects fixed below:
`test_centre_of_mass_is_inside_the_support_polygon` in
`karmel_description/test/test_description.py`, and
`test_diff_drive_uses_current_acceleration_parameters`,
`test_global_costmap_marks_only_nearby_lidar_returns` and
`test_map_thresholds_keep_unknown_cells_unknown` in
`karmel_bringup/test/test_config.py`. Each was checked against the pre-fix value and fails on
it — e.g. with `caster_offset_x_m: -0.10` the first reports
`centre of mass x=+0.0017 m is outside the support polygon [-0.100, +0.000]`.

Pure-Python labs, on the Windows host with `py` (Python 3.14):

```bash
py -m pytest labs/python/tests        # 262 passed
```

## 2. End-to-end simulation smoke test

```bash
ROS_DOMAIN_ID=73 bash tools/smoke_test.sh --nav
```

```text
== 1. Gazebo (headless) + controllers, apartment world
PASS  joint_state_broadcaster and diff_drive_controller active
== 2. Sensors
PASS  /scan publishing (7.964 Hz wall clock)
PASS  /scan frame_id = laser
PASS  /imu frame_id = imu_link
PASS  /camera/camera_info frame_id = camera_optical_frame
== 3. Drive with TwistStamped on /cmd_vel
PASS  /odom x moved from -6.49e-19 to 0.812
== 4. SLAM (slam_toolbox online async)
PASS  /map published (width 109 cells)
== 5. Nav2 on the ground-truth map
PASS  NavigateToPose (-0.578, -0.639) -> (-1.8, 0.3): Goal finished with status: SUCCEEDED

SMOKE TEST PASSED
```

The TF tree was checked separately: `/tf_static` has exactly one parent per frame, rooted at
`base_footprint`, and `tf2_echo odom base_footprint` resolves — no frame has two parents.

## 3. The defects that were fixed, and the measurement for each

### 3.1 The robot rested 5.6° nose-down in Gazebo

`labs/config/karmel.yaml` and `karmel_description/config/robot.yaml`:
`caster_offset_x_m: -0.10` → `0.10`.

The centre of mass is ~1.7 mm *ahead* of the wheel axle (the camera and the front range
sensor are forward of it), so with the only third contact 0.10 m *behind* the axle the chassis
tipped onto its front edge: ground clearance 12 mm over a 125 mm half-length is
`atan(0.012/0.125) = 5.5°`. Moving the caster in front puts the support polygon under the
centre of mass. Measured in the apartment world, robot stationary at the spawn pose
(−1.5, −0.5), from `/imu` and `/scan`:

| | Before | After |
|---|---|---|
| pitch from `/imu` orientation | **+5.581°** | **−0.000°** |
| roll | +0.000° | +0.000° |
| `/imu` linear acceleration x | **−1.0073 m/s²** | +0.0719 m/s² |
| `/scan` range at 0° (wall is 1.95 m away) | **1.5514 m** (the floor) | **1.9471 m** (the wall) |
| `/scan` range at ±7° / ±21° / ±42° | 1.56/1.69/2.10 m — an `r0/cos θ` floor arc | 1.97/2.07/3.66 m — real geometry |

`karmel_description/test/test_description.py::test_centre_of_mass_is_inside_the_support_polygon`
now computes the CoM from the rendered URDF and fails if it leaves the support polygon, so
moving a sensor forward later cannot silently bring the pitch back.

**Residual, measured, not a regression:** because the CoM is only 1.7 mm from the axle, a
full-rate acceleration step (`/cmd_vel` 0.5 m/s, limit 1.0 m/s²) rocks the robot to **−5.6°
nose-up** for the duration of the acceleration, then it settles level again. Nose-up is benign
for a 2D LiDAR in a room with full-height walls (the beam still hits wall, 0.19 m higher at
1.95 m) where nose-down was not (it hit the floor). Braking causes no rocking.

### 3.2 Global costmap closed the 0.8 m doorways

`karmel_bringup/config/nav2_params.yaml`, **global costmap only**:
`obstacle_layer.scan.obstacle_max_range: 3.5` → `1.5`, `raytrace_max_range: 4.0` → `3.0`.
The local costmap is unchanged (4.0 / 3.5): it lives in `odom`, where there is no localization
error to smear.

A far LiDAR return is marked with the full AMCL pose error, so in the `map` frame it lands in
the wrong cell; inflated by 0.35 m that closes the apartment's 0.8 m doorways and cross-room
goals fail with `NO_VALID_PATH` (208). Mark near, clear far.

Verification: `NavigateToPose` on the ground-truth map, robot spawned in the living room at
(−1.5, −0.5) with AMCL seeded at that pose, driving through a doorway to another room. Each
row is one full launch of Gazebo + `robot.launch.py` + `navigation.launch.py`:

| Robot | Global costmap ranges | Goal | Result |
|---|---|---|---|
| pitched (caster −0.10) | 3.5 / 4.0 (before) | bedroom (1.6, 1.4), north door | SUCCEEDED |
| pitched (caster −0.10) | 3.5 / 4.0 (before) | **kitchen (2.0, −2.1), south door** | **ABORTED, `error_code: 208` NO_VALID_PATH** |
| level (caster +0.10) | 3.5 / 4.0 (before) | bedroom (1.6, 1.4) | SUCCEEDED |
| level (caster +0.10) | **1.5 / 3.0 (after)** | bedroom (1.6, 1.4) | SUCCEEDED |
| level (caster +0.10) | **1.5 / 3.0 (after)** | **kitchen (2.0, −2.1)** | SUCCEEDED |

So the failure is real and reproducible, but only on the harder doorway: the south door is
partly blocked by the open door leaf, which leaves less than the nominal 0.8 m. The north
doorway planned fine either way. The two defects also compound — a nose-down LiDAR puts wrong
marks everywhere, and a long `obstacle_max_range` then spreads them across the map.

### 3.3 2,607 unknown cells were served as free space

`karmel_bringup/maps/apartment.yaml` (and the generator that writes it,
`karmel_gazebo/worlds/generate_apartment.py`, and `map_saver.free_thresh_default` in
`nav2_params.yaml`): `free_thresh: 0.25` → `0.196`.

`apartment.pgm` is 134 × 114 = 15,276 cells: 4,565 at value 0 (occupied), 8,104 at 254 (free)
and **2,607 at 205**, the unknown grey that `map_saver` writes. `map_server` converts a pixel
to occupancy as `occ = (255 − value)/255`, so 205 gives `occ = 0.19608`, and the trinary rule
`occ < free_thresh → FREE` turned every one of those 2,607 cells into free space with
`free_thresh: 0.25`. At 0.196 they stay unknown, which is what `map_saver_cli` itself writes.
`test_map_thresholds_keep_unknown_cells_unknown` now asserts `free_thresh <= (255−205)/255`
for every map in the package and for the map-saver default.

Measured by running `nav2_map_server map_server` on each version of the yaml and counting the
cells of the `/map` it serves:

```text
free_thresh 0.25   free(0)=10711  occupied(100)=4565  unknown(-1)=    0
free_thresh 0.196  free(0)= 8104  occupied(100)=4565  unknown(-1)= 2607
```

### 3.4 Deprecated controller parameter

`karmel_bringup/config/controllers.yaml`: `linear.x.min_acceleration: -1.0` →
`linear.x.max_deceleration: -1.0`, and the same for `angular.z`.

`diff_drive_controller_parameters.hpp` in the image describes `min_acceleration` as
`"deprecated, use max_deceleration"`. The sign convention is the same
(`max_acceleration >= 0`, `max_deceleration <= 0`), and the reverse-direction limits default to
`-max_acceleration` / `-max_deceleration`. Verified on the running controller:

```text
linear.x.max_acceleration   1.0
linear.x.max_deceleration  -1.0
linear.x.min_acceleration   nan      # no longer set
angular.z.max_deceleration -5.0
```

### 3.5 Two defects that were already fixed in the tree

- **`karmel_base`'s `base_frame`.** `base_node.py` already declares
  `base_frame` with the default `base_footprint`, `base.launch.py` sets it explicitly, and
  `controllers.yaml`, `ekf.yaml`, `slam_toolbox_online_async.yaml` and AMCL all agree. The TF
  check above confirms a single-parent tree. The only remaining `base_link` in
  `nav2_params.yaml` outside the costmaps is `docking_server.base_frame`, which is a lookup
  frame for the docking pose, not a TF publisher, and is left at the Nav2 default.
- **`labs/docker/compose.yaml`, `check_config_sync.py`, `tools/smoke_test.sh`.** All three
  exist and were exercised in this pass; the instructions in lesson 06.09 are true.

## 4. Effect on the lessons that document these defects

Lessons were not edited. Two things a reader will now find different from the text:

- **11.09 Level 3b is now history, not a reproduction.** The measurements the case study
  quotes were confirmed exactly (5.6° pitch, 1.56 m forward range), and its step 7 gives the
  change that this pass applied. A reader following step 7 will find the fix already in place.
- **Exercise 11.09-E4 no longer has an answer.** It asks the reader to record their own bag
  and find the floor ring in the first scan; with the fix shipped there is no ring, so its
  expected result (95 of 360 beams outlying, `r0 ≈ 1.56 m`, pitch 6.1°) cannot be reproduced
  without reverting `caster_offset_x_m` to `-0.10` first.
- **The map-quality numbers in 11.06 and 11.09 were not re-measured** (73.6% → 100% first-scan
  score, explored area 34.1 → 24.2 m², wall accuracy 55.8%). Nothing here contradicts them;
  they simply were not re-run in this pass.

## What is untested, and why

- **All real hardware.** Nothing was run on a Raspberry Pi 5, a Pico 2, the motors, encoders,
  IMU, VL53L1X, LiDAR or camera. `karmel_hardware` is exercised only through its unit tests and
  `karmel_base/fake_pico` over a pty. Every number above is from simulation.
- **arm64.** The `deps` image was not built or run for the Raspberry Pi / Jetson.
- **GUI paths.** RViz, the Gazebo GUI, the `wslg` and `linux` compose services and
  `display.launch.py` were not opened in this pass (the containers had `--network none` and no
  display); `labs/docker/README.md` records the earlier 2026-09-17 WSLg test.
- **The doorway fix was not isolated from the caster fix.** `NO_VALID_PATH` reproduced with
  both defects present (see the table in §3.2) and is gone with both fixed. The run that would
  separate them — level robot, pre-fix ranges, kitchen goal — was not done, so how much of the
  repair each change contributes is unmeasured. The 1.5 / 3.0 values are kept because the
  reasoning holds independently of the pitch: a 3.5 m return marked in the `map` frame commits
  the full localization error to a cell.
- **Long-run and loop-closure SLAM quality.** Map accuracy numbers were not re-measured here;
  see `labs/ros2_ws/TESTED.md` and lesson 11.06 for those.
- **`generate_apartment.py --map` was not re-run.** `apartment.yaml` was edited in place and
  the generator was changed to match, so the next regeneration produces the same file; the
  `.pgm` is unchanged either way.
