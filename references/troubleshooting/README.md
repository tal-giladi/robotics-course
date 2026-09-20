# Troubleshooting

Symptom-driven debugging guides for the problems that cut across lessons. Every lesson has its own
Troubleshooting section for its own failures; these pages are for the failures that do not belong to
one lesson — the ones you meet in module 12 that were actually caused by something in module 01.

**Every symptom here was observed and measured in a lesson.** Nothing is invented. Each entry is a
decision procedure, cheapest and most likely check first, and it links to the lesson that teaches the
fix rather than repeating it.

## Which page

| Page | Use it when |
|---|---|
| [Power and battery](power-and-battery.md) | The Pi reboots, the pack cuts out, the fuse blows, voltages are not what the formula says, runtime is short. |
| [Motors and drivers](motors-and-drivers.md) | A wheel does not turn, turns the wrong way, only goes one way, stutters, whines, or the driver gets hot. |
| [Encoders, odometry and control](control-and-odometry.md) | Counts do not change or drift, the robot curves, a PID oscillates, odometry disagrees with the tape measure. |
| [Serial link to the Pico](serial-and-pico.md) | `/dev/ttyACM0` is missing or denied, no hello reply, rising `bad_lines`, the watchdog trips, motors stutter every few hundred ms. |
| [ROS 2 discovery, QoS and timing](ros2-discovery-and-qos.md) | Nodes cannot see each other, a topic lists but delivers nothing, a callback never fires, a service call deadlocks, ghost nodes. |
| [TF and frames](tf-and-frames.md) | "does not exist", "extrapolation into the future/past", unconnected trees, data rotated 90°, detections on the ceiling. |
| [Sensors](sensors.md) | An I²C scan is empty, a range sensor lies, the IMU drifts, the LiDAR misses glass, the camera is dark or late. |
| [Simulation and Gazebo](simulation.md) | `gz` commands do not exist, the robot sinks or jitters, the bridge passes nothing, sim time is wrong, sim and robot disagree. |
| [Navigation](navigation.md) | Nav2 does not move, aborts with a numeric error, weaves, refuses doorways, runs recoveries for ever. |
| [SLAM and localization](slam-localization.md) | The map smears, doubles or folds; AMCL never converges; the robot teleports in RViz. |
| [The arm](arm.md) | A servo does not answer, the arm jumps on torque enable, IK "fails" at reachable points, MoveIt refuses to plan, the grasp misses by a constant. |
| [Build and toolchain](build-and-toolchain.md) | `colcon` failures, `ModuleNotFoundError`, PEP 668, GPG and apt errors, "my code change has no effect", CI green but robot broken. |

## The method, in one page

The lessons teach this repeatedly ([02.09](../../02-robot-electronics/02.09-electrical-debugging-method.md),
[04.13](../../04-ros2/04.13-debugging-and-introspection.md),
[05.11](../../05-frames-and-transforms/05.11-debugging-tf.md),
[07.11](../../07-sensors/07.11-sensor-troubleshooting.md),
[11.09](../../11-slam/11.09-slam-troubleshooting.md)). It is the same method every time:

1. **State the symptom as an observation, not a theory.** "The Pi reboots when the motors start" is a
   symptom. "The buck converter is bad" is a guess you have not earned yet.
2. **Reproduce it on demand.** A fault you cannot trigger is a fault you cannot fix. If it is
   intermittent, find the variable that makes it more likely (load, temperature, movement, a specific
   cable).
3. **Bisect the chain.** Every robot failure lives somewhere on a chain — battery → switch → driver →
   motor, or sensor → driver → topic → QoS → callback → controller. Measure in the middle, and you
   halve the search. This is binary search with a multimeter.
4. **Measure, do not guess.** One voltage, one `ros2 topic hz`, one `tf2_echo`, one logged number
   beats an hour of theory. Write the number down.
5. **Change one thing at a time,** and put it back if it did not help. Two simultaneous changes have
   already cost you the ability to attribute the result.
6. **Know what "working" looks like** before you start, in numbers: "encoder counts rise by ~2464 per
   wheel turn", "`/scan` at 10 Hz", "11.4 V at the driver under load".
7. **When you find it, write down the root cause** — not just the fix. The same class of fault will
   come back in a different module.

> [!CAUTION]
> **Safety:** debugging is when people skip the rules. Wheels in the air for any motor test, battery
> disconnected before rewiring, arm torque limits low, hand on the stop. The full list is in
> [SAFETY.md](../../SAFETY.md).

> [!TIP]
> **Ask your teacher:** "Here is my symptom and the two measurements I already took — what is the next
> cheapest measurement that would split the possibilities in half?"

## Two failures that are almost never what they look like

- **"It worked in simulation."** Simulation confirms your model, not your robot. The gap is taught in
  [06.10](../../06-simulation/06.10-sim-to-real-gap.md); the usual causes are motor deadband, latency,
  wheel slip and sensor noise that the model does not have.
- **"It worked yesterday."** Something changed: a battery state of charge, a launch argument, a
  parameter file, a rebuilt workspace, ambient light, or a cable you moved. Diff the change, do not
  re-derive the physics. [03.11](../../03-robot-software/03.11-ci-and-reproducibility.md) is about
  making "what changed" answerable.

## See also

- [Glossary](../glossary.md) — if the error message uses a word you do not know.
- [SAFETY.md](../../SAFETY.md) — the ten standing rules.
- [`python course.py struggle <lesson> "…"`](../../README.md) — record where you got stuck, so the
  teacher can recommend the foundation lesson you are actually missing.
