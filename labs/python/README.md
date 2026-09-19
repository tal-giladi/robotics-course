# robotlab — the course's pure-Python robotics library

`robotlab` is what the non-ROS labs import: the hardware abstraction (`hal`), SE(2) geometry,
the robot config loader and a 2D differential-drive simulator that is realistic enough to teach
odometry, PID, Kalman/particle filters, mapping, planning and RL — and small enough to read.

```bash
pip install -r labs/requirements.txt
pip install -e labs/python          # optional: pytest from the repo root finds robotlab without it
python -m pytest                    # from the repo root: library tests + exercises
python labs/examples/sim_quickstart.py square.png
```

Conventions everywhere: SI units, angles in radians wrapped to (−π, π], REP-103 frames (x forward,
y left, z up, counter-clockwise positive). Robot numbers come from `labs/config/karmel.yaml` —
never hard-code them.

| Module | What it gives you |
|---|---|
| `robotlab.hal` | `DifferentialBase` protocol, `BaseState`, `FLAG_WATCHDOG=1`, `FLAG_LOW_BATTERY=2`, `FLAG_RANGE_ERROR=4`, `FLAG_VELOCITY_MODE=8` |
| `robotlab.geometry` | `wrap_angle`, `angle_diff`, `deg2rad`, `rad2deg`, `rotation_matrix`, `SE2` |
| `robotlab.config` | `load_config()` → `KarmelConfig` dataclasses |
| `robotlab.sim` | `World`, `DiffDriveSim`, `DiffDriveParams`, `SensorParams`, `SimBase`, `LaserScan`, `LandmarkObservation`, `OccupancyGrid`, `arc_update` |
| `robotlab.sim.viz` | matplotlib helpers (import explicitly) |

## Geometry

```python
from robotlab.geometry import SE2, wrap_angle, angle_diff

world_T_robot = SE2(1.0, 2.0, 0.5)            # theta is wrapped on construction
robot_T_lidar = SE2(0.0, 0.0, 0.0)
world_T_lidar = world_T_robot @ robot_T_lidar  # compose
back = world_T_robot.inverse()
rel = a.between(b)                             # a.inverse() @ b
pts_world = world_T_lidar.apply(scan.points()) # (N, 2) -> (N, 2), vectorized
x, y, theta = world_T_robot                    # unpacks like a tuple
M = world_T_robot.as_matrix(); SE2.from_matrix(M)
wrap_angle(4.0); wrap_angle(np.array([...]))   # float -> float, array -> array
```

## Config

```python
from robotlab.config import load_config
cfg = load_config()                  # path arg > $KARMEL_CONFIG > labs/config/karmel.yaml
cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
cfg.drive.ticks_per_wheel_rev        # encoder_cpr_motor * gear_ratio * quadrature_multiplier
cfg.drive.meters_per_tick
cfg.chassis.footprint_radius_m       # circle covering the chassis (the sim's collision radius)
cfg.sensors.lidar.samples, cfg.battery.low_warning_v, cfg.serial.watchdog_ms, cfg.pins["encoder_left_a"]
```

Missing keys raise `ConfigError` naming the key; extra keys are ignored.

## Worlds

```python
from robotlab.sim import World
room = World.rectangle_room(4.0, 3.0)              # walls of [0, 4] x [0, 3]
home = World.apartment()                           # 6 x 5 m, 4 rooms, furniture, 10 landmarks
                                                   # good start pose: (1.0, 1.3, 0.0)
custom = World.from_segments(segments,             # (M, 4) [x0, y0, x1, y1]
                             circles=[[2, 1, 0.3]],# (K, 3) [cx, cy, r]
                             landmarks=[[0, 1]], landmark_ids=[7])

ranges = home.raycast((x, y), angles, max_range=12.0)    # (N,), inf = no hit
ranges = home.raycast(origins_Nx2, angles_N)             # one origin per ray: particle filters
home.distance_to_obstacles(points_Nx2)                   # clearance, e.g. likelihood fields
home.collides(x, y, radius)

grid = home.to_occupancy_grid(resolution=0.05, margin=0.5)
grid.data            # int8 [rows=y, cols=x]: 0 free, 100 occupied, -1 unknown; row 0 = bottom
grid.origin, grid.resolution, grid.world_to_cell(x, y), grid.cell_to_world(row, col)
grid.save("apartment.yaml")                    # ROS map_server format: .yaml + .pgm
OccupancyGrid.load("my_nav2_map.yaml")         # trinary / scale / raw maps from map_saver
```

`raycast` does a 360-beam scan of the apartment in under 1 ms; 1000 particles × 30 beams is one call (~0.1 s).

## The simulator

```python
from robotlab.sim import DiffDriveSim, DiffDriveParams, SensorParams, World

sim = DiffDriveSim(World.apartment(),
                   DiffDriveParams.realistic(),   # or .ideal()
                   SensorParams.realistic(),      # or .ideal()
                   pose=(1.0, 1.3, 0.0), seed=42)
sim.set_duty(0.5, 0.5)            # open loop, -1..1
sim.set_velocity(8.0, 8.0)        # rad/s, tracked by the simulated firmware PID
sim.step(0.02)                    # one physics step; sim.advance(2.0) runs many

sim.pose, sim.wheel_rad_s, sim.yaw_rate, sim.collided, sim.t   # ground truth
sim.ticks                          # (left, right) int encoder counts
sim.wheel_velocity_estimate        # tick-difference estimate, like the firmware
sim.battery_v
sim.gyro_z()                       # truth + bias + noise
sim.front_range()                  # ToF cone from the sensor mount; None when out of range
sim.lidar_scan()                   # LaserScan
sim.observe_landmarks()            # [LandmarkObservation(id, range_m, bearing_rad)], from base_link
sim.reset(pose)                    # RL episodes
```

What `DiffDriveSim.step` models (read it — lesson 06.02 walks through it): firmware velocity PID
→ deadband + saturation + per-motor gain + battery voltage → first-order motor lag (exact
discretization) → wheel travel with the *true* wheel radii and random slip → exact-arc pose
update, blocked by obstacles (wheels stall) → quantized encoder ticks → battery drain.

**Presets.** `ideal()` = karmel.yaml exactly, no noise; motor lag, deadband, saturation and tick
quantization stay (they are physics). `realistic()` = right motor 6% weaker, wheel radii
−0.5%/+0.8%, true wheelbase +4%, 2% slip, battery sag; sensor noise (LiDAR σ=1 cm with 2% dropouts,
ToF σ=1 cm, landmarks σ=5 cm / 0.03 rad, gyro bias 0.01 rad/s + σ=0.005). Change any knob with
`dataclasses.replace(DiffDriveParams.ideal(), wheel_separation_scale=1.05, encoder_bits=16)`.
Nominal `wheel_radius_m`/`wheel_separation_m` are what estimators should use; the `*_scale` and
`motor_gain_*` fields are the hidden truth that calibration labs recover.

**Determinism.** All randomness comes from `sim.rng` (`numpy.random.default_rng(seed)`); the same
seed and the same call sequence give bit-identical runs. Sensor calls consume random numbers, so
calling `lidar_scan()` more often changes later noise.

### LaserScan (sensor_msgs/LaserScan semantics, REP-117)

`angle_min = -π`, `angle_increment = 2π/N`, ranges in the LiDAR frame (`sim.lidar_pose` in the
world). `+inf` = nothing within `range_max`, `-inf` = closer than `range_min` (0.15 m), `nan` =
dropout. `scan.angles`, `scan.valid` (mask), `scan.points()` → `(K, 2)` valid points in the
sensor frame.

## SimBase — the simulator as a `DifferentialBase`

```python
from robotlab.sim import SimBase
base = SimBase(sim, dt=0.02)          # lockstep: each read() advances 0.02 s of sim time
for _ in range(500):
    base.set_wheel_velocity(6.0, 6.0) # send a command EVERY cycle...
    state = base.read()               # ...the 0.3 s firmware watchdog is simulated
    estimate = odom.update(state.left_ticks, state.right_ticks)
truth = base.sim.pose                 # compare estimate with ground truth
scan = base.scan(); base.gyro_z(); base.landmarks()   # extras, not in the protocol
base.close()
```

* `realtime=True` makes `read()` sleep so sim time follows the wall clock (teleop demos).
* `watchdog_s=None` disables the watchdog; when it trips, motors stop and `FLAG_WATCHDOG` is set
  until the next command. `FLAG_VELOCITY_MODE` and `FLAG_LOW_BATTERY` are reported as on the robot.
* LiDAR is not in `BaseState`: call `base.scan()` at your scan rate (every 5th read = 10 Hz).

## Plotting (`robotlab.sim.viz`)

```python
import matplotlib; matplotlib.use("Agg")   # scripts/tests without a display
from robotlab.sim import viz
fig, ax = viz.new_axes(world, title="EKF")
viz.draw_occupancy_grid(ax, grid)
viz.draw_trajectory(ax, true_poses, label="truth")                 # list of SE2 or (N, 3) array
viz.draw_trajectory(ax, est_poses, linestyle="--", label="EKF")
viz.draw_covariance_ellipse(ax, mu[:2], P[:2, :2], n_sigma=2)
viz.draw_scan(ax, sim.lidar_pose, scan)
viz.draw_robot(ax, sim.pose)
anim = viz.animate(world, true_poses, scans=scans, estimates=est_poses)
anim.save("run.gif", writer="pillow")
```

## Exercises

Auto-graded exercises live in `labs/exercises/<lesson-id>/` and import student code through the
`impl` fixture — see `labs/exercises/_template/README.md` and the worked example `09.04`.
