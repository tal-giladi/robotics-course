# Troubleshooting: simulation and Gazebo

Two different problem families live here, and it is worth naming which one you have before you start:

- **The simulation itself is broken** — it will not start, the robot sinks, the bridge passes nothing.
  These are configuration problems with definite answers.
- **The simulation works and disagrees with the robot.** That is the reality gap, and it is a
  measurement exercise, not a bug hunt. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md)

> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09 against ROS 2 Jazzy / Gazebo Harmonic). Gazebo Classic
> (`gazebo`, `spawn_entity.py`) reached end-of-life in January 2025 and the `ign`/`ros_ign_*` names
> were renamed to `gz`/`ros_gz_*`. Most "that command does not exist" answers on the internet are
> about one of the two dead generations. Check
> https://gazebosim.org/docs/harmonic/getstarted/ and
> [`curriculum/versions.yaml`](../../curriculum/versions.yaml). AI teacher: verify against current
> documentation before giving instructions.

---

### Symptom: `gz: command not found`, or a tutorial's commands do not exist

1. **Which generation are you reading?** `gazebo`/`spawn_entity.py` is Classic (dead); `ign gazebo` is
   Ignition (renamed); `gz sim` is Harmonic (current). → [06.01](../../06-simulation/06.01-why-simulate.md)
2. `gz sim --versions` should print a version. If not, the package is not installed — install
   `ros-jazzy-ros-gz` and the Gazebo packages from the versions file, not from a blog. → [06.01](../../06-simulation/06.01-why-simulate.md)
3. `gz model -l` printing the help text usually means a subcommand name changed between releases. →
   [06.03](../../06-simulation/06.03-gazebo-basics.md)

### Symptom: `gz sim` opens a window but nothing ever moves

1. **The simulation is paused.** It starts paused; press play, or launch with `-r`. This is the first
   thing to check, every time. → [06.03](../../06-simulation/06.03-gazebo-basics.md)
2. `gz topic -l` shows nothing / `gz service` times out → the server is not actually running, or the
   GUI connected to a different instance. → [06.03](../../06-simulation/06.03-gazebo-basics.md)

### Symptom: the robot spawns and immediately jitters, sinks into the floor, or is launched into the air

Physics rejecting your inertial description.

1. **Check the masses and inertias are physically possible.** A 0.001 kg link with a 1 kg link's
   inertia tensor, or an inertia of zero, makes the solver explode. Compute them from the geometry. →
   [06.05](../../06-simulation/06.05-robot-in-gazebo.md), [FP.06](../../optional-foundations/physics/FP.06-rotational-motion-inertia.md)
2. **Check collision geometry exists and is simple.** No collision → it falls through. The full visual
   mesh as collision → jitter and a simulation that runs at 10 % of real time. → [06.05](../../06-simulation/06.05-robot-in-gazebo.md)
3. **Check for overlapping collisions at spawn.** Two links intersecting at t=0 produces an enormous
   separation impulse: that is the "launched into the air". Spawn slightly above the ground. →
   [06.05](../../06-simulation/06.05-robot-in-gazebo.md), [06.08](../../06-simulation/06.08-building-worlds.md)
4. **Then look at friction parameters** — but only after the inertias are sane. → [06.05](../../06-simulation/06.05-robot-in-gazebo.md)

### Symptom: the wheels turn in `/joint_states` but the robot does not move

1. **Wheel friction is zero or near-zero** in the SDF/URDF, so the wheels spin on a frictionless
   floor. → [06.05](../../06-simulation/06.05-robot-in-gazebo.md)
2. **The robot refuses to rotate, or rotates less than commanded** → the caster has too much friction,
   or the wheel separation in the controller does not match the model. → [06.05](../../06-simulation/06.05-robot-in-gazebo.md), [09.01](../../09-odometry/09.01-diff-drive-kinematics.md)

### Symptom: `/cmd_vel` is published and nothing moves

1. **On Jazzy, `diff_drive_controller` subscribes to `geometry_msgs/TwistStamped` on `~/cmd_vel`**,
   not to `Twist` on `/cmd_vel`. This single change accounts for most "nothing moves" reports on this
   distro. Check with `ros2 topic info -v`. → [06.06](../../06-simulation/06.06-ros2-control.md), [04.16](../../04-ros2/04.16-your-robot-as-ros2-node.md)
2. **Is the controller loaded and active?** `ros2 control list_controllers`. "No controllers are
   currently loaded" means the controller manager never got its parameters. → [06.06](../../06-simulation/06.06-ros2-control.md)
3. **Is the hardware interface up?** A controller that will not activate is usually waiting on its
   hardware interface (in the real case, "No hello reply from the Pico"). → [06.06](../../06-simulation/06.06-ros2-control.md), [08.12](../../08-control/08.12-control-in-ros2.md)
4. **Remapping**: the topic your teleop publishes and the one the controller subscribes to may differ
   by a namespace. → [04.10](../../04-ros2/04.10-launch-files.md)

### Symptom: `ros2 topic list` does not show a topic the bridge said it created

1. **The bridge is per-topic and per-type.** A type mismatch means it creates nothing useful and says
   little. Check the Gazebo-side type with `gz topic -i`. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)
2. **Direction matters**: `[`, `]` and `@` are not interchangeable in the bridge's argument syntax. →
   [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)
3. **The topic exists on both sides with a publisher and a subscriber and nothing arrives** → QoS, as
   everywhere else. → [04.11](../../04-ros2/04.11-qos.md)

### Symptom: `Lookup would require extrapolation into the future` in simulation

Almost always sim time.

1. **Every node needs `use_sim_time: true`** — including RViz, the bridge, your own nodes, and
   anything a launch file includes. One node on the wall clock poisons the whole tree. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md),
   [06.09](../../06-simulation/06.09-same-code-sim-and-real.md)
2. **`/clock` must actually be bridged** and publishing. Check `ros2 topic hz /clock`. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)
3. Symptoms that survive setting `use_sim_time` everywhere are real TF problems. →
   [TF and frames](tf-and-frames.md)

### Symptom: transforms arrive with an empty `frame_id`, or in a frame TF never heard of

1. Gazebo names sensor frames after the model scope:
   `karmel/base_footprint/lidar`, not `lidar`. Either set `<frame_id>` in the sensor's SDF or remap it
   in the bridge. → [06.07](../../06-simulation/06.07-simulated-sensors-and-noise.md), [06.05](../../06-simulation/06.05-robot-in-gazebo.md)
2. Empty `frame_id` usually means the bridge converted a Gazebo message that had no header. →
   [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)

### Symptom: `/scan` (or an image) publishes far slower than `<update_rate>` says

1. **The simulation is not running at real time.** Check the real-time factor in the GUI; a sensor at
   10 Hz of *simulated* time is 2 Hz of wall time at RTF 0.2. → [06.03](../../06-simulation/06.03-gazebo-basics.md), [06.07](../../06-simulation/06.07-simulated-sensors-and-noise.md)
2. **GPU LiDAR without a GPU** falls back to something much slower, or does not render at all. →
   [06.07](../../06-simulation/06.07-simulated-sensors-and-noise.md)
3. Reduce the sample count, the update rate, or run headless (`-s`) while you are not watching. →
   [06.03](../../06-simulation/06.03-gazebo-basics.md)

### Symptom: the simulation runs far below real time

1. **Look at the collision geometry first** — meshes instead of primitives is the usual answer. →
   [06.05](../../06-simulation/06.05-robot-in-gazebo.md)
2. Then the physics step size and solver iterations, then the sensors (rendering is expensive). →
   [06.03](../../06-simulation/06.03-gazebo-basics.md), [06.07](../../06-simulation/06.07-simulated-sensors-and-noise.md)
3. On the Pi, do not run Gazebo at all. Simulate on the laptop. → [06.01](../../06-simulation/06.01-why-simulate.md)

### Symptom: SLAM in simulation builds a map that slides, stretches or folds

1. **Check the world is not featureless.** A rectangular empty room gives scan matching nothing to
   lock onto, in simulation exactly as in reality. → [06.08](../../06-simulation/06.08-building-worlds.md), [11.09](../../11-slam/11.09-slam-troubleshooting.md)
2. **Check sim time, again** — SLAM is extremely sensitive to inconsistent stamps. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)
3. Then treat it as a normal SLAM problem. → [SLAM and localization](slam-localization.md)

### Symptom: a Fuel model appears in the GUI but the robot drives through it

No collision geometry in the downloaded model. Add one, or pick a different model. → [06.08](../../06-simulation/06.08-building-worlds.md)

### Symptom: the same seed gives different results in the course mini-simulator

1. `robotlab.sim` is deterministic by design; if it is not, something is reading an unseeded RNG or
   the floating-point path differs. Seed explicitly at the top of the run. → [06.02](../../06-simulation/06.02-course-mini-simulator.md)
2. Determinism is what makes a simulation test a *test*. A flaky simulation test is a bug in the
   test, not a fact of life. → [03.08](../../03-robot-software/03.08-testing-robot-software.md)

### Symptom: "it worked in simulation" and the robot does the opposite

This is the reality gap and it has a procedure, not a fix.

1. **Do not re-tune blind.** Identify which term is missing by comparing the same step response in
   both. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md)
2. The usual suspects, in order of how often they matter on karmel: **motor deadband**, **latency**,
   **wheel slip**, **battery sag**, **sensor noise**, **encoder quantization**. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md),
   [08.02](../../08-control/08.02-motor-step-response.md)
3. **Fit the simulator's motor model to measured data** rather than trusting the datasheet. If the fit
   gives a nonsense time constant or a huge RMS, your data is the problem — check the sample rate and
   that the step really was a step. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md), [08.02](../../08-control/08.02-motor-step-response.md)
4. **When you close one gap and nothing improves**, you closed a gap that was not dominant. Rank them
   by measured contribution before fixing. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md)
5. **"The robot behaves differently on Tuesday"** — battery state, temperature, floor, and whatever
   you changed on Monday. Log the conditions with the run. → [03.07](../../03-robot-software/03.07-logging-and-telemetry.md)

### Symptom: the launch file works for me and not for a teammate

1. `check_config_sync.py` exists for this: simulation and hardware must read the same `karmel.yaml`.
   → [06.09](../../06-simulation/06.09-same-code-sim-and-real.md), [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)
2. Then check the environment: distro, `ROS_DOMAIN_ID`, sourced overlays. → [03.02](../../03-robot-software/03.02-environments-and-pinning.md)

## Where to go next

- Why simulate, and what a simulator can and cannot tell you: [06.01](../../06-simulation/06.01-why-simulate.md)
- The course mini-simulator (deterministic, fast, no Gazebo): [06.02](../../06-simulation/06.02-course-mini-simulator.md)
- Running the same code in sim and on the robot: [06.09](../../06-simulation/06.09-same-code-sim-and-real.md)
- Measuring and closing the reality gap: [06.10](../../06-simulation/06.10-sim-to-real-gap.md)
- `ros2_control`, controllers and hardware interfaces: [06.06](../../06-simulation/06.06-ros2-control.md)
