# Troubleshooting: SLAM and localization

The rule that saves the most time here: **a bad map is almost always bad inputs, not bad parameters.**
Before touching a single SLAM setting, prove that odometry, TF, the scan and the clock are all
correct — in that order. Parameter tuning on top of a broken input produces a map that is wrong in a
new way. → [11.09](../../11-slam/11.09-slam-troubleshooting.md)

**The input checklist, every time:**

| Input | How to check it is good | Lesson |
|---|---|---|
| Odometry | drive a 2 m square; the reported pose closes to within a few cm | [09.05](../../09-odometry/09.05-calibrating-odometry.md) |
| TF | `view_frames` shows one tree; `odom → base_link` smooth, `laser` present | [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md) |
| Scan | `/scan` at its nominal rate; walls look like walls in RViz with fixed frame `odom` | [07.07](../../07-sensors/07.07-2d-lidar.md) |
| Time | one clock; `use_sim_time` consistent; stamps are capture times | [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md) |

---

### Symptom: `/map` never appears

1. **Is the node running and receiving?** `ros2 topic hz /scan` and check `slam_toolbox`'s log for
   "waiting for transform". → [11.06](../../11-slam/11.06-slam-toolbox-simulation.md)
2. **TF**: `slam_toolbox` needs `odom → base_link` and `base_link → laser`. A missing static transform
   stops it silently. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
3. **QoS**: `/map` is published transient-local; a plain subscriber may see nothing even though it is
   being published. → [04.11](../../04-ros2/04.11-qos.md)
4. **`use_sim_time`** on `slam_toolbox` too. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)

### Symptom: the map is a smeared mess, or walls are doubled

Smearing means the pose used to place a scan was wrong when the scan was placed.

1. **Check odometry first.** If `/odom` drifts a metre over a short run, no SLAM front end can fix it.
   Calibrate before mapping. → [09.05](../../09-odometry/09.05-calibrating-odometry.md), [11.09](../../11-slam/11.09-slam-troubleshooting.md)
2. **Check the scan's timestamp.** A constant stamp offset smears every scan by (offset × speed).
   Drive slower and see whether the smear shrinks proportionally — that is diagnostic of a timing
   problem, not a noise problem. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)
3. **Check the `base_link → laser` transform**, including its yaw. A few degrees of mounting error
   rotates every scan and produces a map that "drifts" as you turn. → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
4. **Then drive more slowly and turn more slowly.** Scan matching has a convergence basin; fast
   rotation walks out of it. → [11.04](../../11-slam/11.04-scan-matching-icp.md), [11.07](../../11-slam/11.07-mapping-your-room.md)
5. Only then adjust scan-matcher parameters, one at a time, on a **recorded bag** so the comparison is
   fair. → [11.09](../../11-slam/11.09-slam-troubleshooting.md), [04.14](../../04-ros2/04.14-rosbag2.md)

### Symptom: walls are dotted or have holes

1. Cells are not being updated enough times to cross the occupancy threshold — drive past more slowly,
   or check the inverse sensor model's hit/miss probabilities. → [11.02](../../11-slam/11.02-occupancy-grid-mapping.md)
2. Maximum-range returns must not be marked as hits. Check the driver's "invalid" convention (0, inf,
   or NaN) and that your code honours it. → [07.07](../../07-sensors/07.07-2d-lidar.md), [11.02](../../11-slam/11.02-occupancy-grid-mapping.md)

### Symptom: a "ghost" wall, or a second copy of a room offset and rotated

1. **This is a data-association / loop-closure failure**, not a noise problem. The front end matched
   the wrong place. → [11.03](../../11-slam/11.03-the-slam-problem.md), [11.05](../../11-slam/11.05-pose-graphs-loop-closure.md)
2. **Featureless corridors** are the classic cause: a long, straight, symmetric corridor gives ICP no
   constraint along its length. → [11.09](../../11-slam/11.09-slam-troubleshooting.md), [11.04](../../11-slam/11.04-scan-matching-icp.md)
3. **A whole wing folded into another room** means a loop closure was accepted between two genuinely
   different places. Tighten the match-acceptance threshold and re-run the bag. → [11.05](../../11-slam/11.05-pose-graphs-loop-closure.md),
   [11.07](../../11-slam/11.07-mapping-your-room.md)
4. **No loop closures ever found** is the opposite failure: the search radius is too small for your
   odometry drift, or the threshold too tight. → [11.05](../../11-slam/11.05-pose-graphs-loop-closure.md)

### Symptom: the map rotates slowly as you drive

A systematic heading error: gyro bias, a wrong wheel separation, or a yawed LiDAR mount. Fix it
upstream, in that order of cheapness. → [07.05](../../07-sensors/07.05-imu-fundamentals.md), [09.05](../../09-odometry/09.05-calibrating-odometry.md), [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)

### Symptom: the map is fine, then suddenly folds

A single bad loop closure. Record the bag, find the moment, and look at what the robot was seeing.
Usually a symmetric place (two identical doorways, a repeated corridor). → [11.05](../../11-slam/11.05-pose-graphs-loop-closure.md), [11.09](../../11-slam/11.09-slam-troubleshooting.md)

### Symptom: "Message Filter dropping message: frame 'laser' … queue is full"

The transform for that scan's timestamp is not available when the scan arrives.

1. `odom → base_link` is not being published fast enough, or at all, at those times. → [09.07](../../09-odometry/09.07-odometry-in-ros2.md)
2. Clock mismatch between the scan publisher and the TF publisher. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md), [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md)
3. This message is a *timing* diagnosis, not a SLAM diagnosis. → [11.09](../../11-slam/11.09-slam-troubleshooting.md)

### Symptom: the robot "teleports" in RViz

1. **`map → odom` jumping is normal** — that is the localizer correcting drift. Jumping by metres is
   not. → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [10.09](../../10-localization/10.09-amcl-in-ros2.md)
2. **Two publishers** of `map → odom` (AMCL and SLAM both running) will fight. Run one. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
3. **Watch the right frame**: with the fixed frame set to `odom`, the robot is smooth and the map
   moves; with `map`, the robot jumps. Neither is a bug. → [05.09](../../05-frames-and-transforms/05.09-rviz.md)

### Symptom: AMCL logs "Please set the initial pose" and no `/amcl_pose` appears

1. AMCL does not guess. Give it an initial pose (RViz's 2D Pose Estimate, or a published
   `/initialpose`), or enable global initialisation and accept that it takes time and motion to
   converge. → [10.09](../../10-localization/10.09-amcl-in-ros2.md)
2. It also needs the map from the map server, on a transient-local subscription. → [10.09](../../10-localization/10.09-amcl-in-ros2.md)

### Symptom: the robot jumps around the map, or the scan does not lie on the walls

1. **The scan not lying on the walls is the real diagnosis** — the pose is wrong. Set the initial pose
   accurately and drive a few metres; particle filters converge with *motion*, not with time. →
   [10.09](../../10-localization/10.09-amcl-in-ros2.md), [10.08](../../10-localization/10.08-particle-filter-mcl.md)
2. **Check odometry noise parameters** (`alpha1..alpha5`). Too small and the filter is overconfident
   and cannot correct; too large and it wanders. → [10.09](../../10-localization/10.09-amcl-in-ros2.md)
3. **Check the map matches reality.** A map built before the furniture moved will not match. →
   [11.07](../../11-slam/11.07-mapping-your-room.md)

### Symptom: AMCL is confidently in the wrong place and never recovers

1. That is the kidnapped-robot problem. Increase the random-particle injection rate (`recovery_alpha`)
   so the filter can consider other hypotheses. → [10.09](../../10-localization/10.09-amcl-in-ros2.md), [10.01](../../10-localization/10.01-what-is-localization.md)
2. Symmetric environments (a corridor of identical doors) genuinely have multiple valid hypotheses.
   Add a landmark — an AprilTag is cheap and decisive. → [10.10](../../10-localization/10.10-fiducial-landmarks.md)

### Symptom: global localization never converges

1. Particle count too low for the map's size. → [10.08](../../10-localization/10.08-particle-filter-mcl.md)
2. The robot is not moving enough; without motion, the likelihoods of symmetric hypotheses never
   separate. → [10.08](../../10-localization/10.08-particle-filter-mcl.md)
3. The likelihood field's σ is too small, so every particle's weight is effectively zero. → [10.08](../../10-localization/10.08-particle-filter-mcl.md)

### Symptom: every particle weight is `nan`, or every weight is zero

1. Zeros: the measurement model is too sharp for the actual error, so no particle is plausible.
   Widen it. → [10.08](../../10-localization/10.08-particle-filter-mcl.md)
2. NaNs: a division by a zero normalising constant, or a log of zero. Normalise in log space. →
   [10.08](../../10-localization/10.08-particle-filter-mcl.md)

### Symptom: the filter (EKF or KF) is smooth and slowly drifts away from the truth

1. **Run the NEES check in simulation.** A plot cannot tell you a filter is overconfident; NEES can.
   If NEES is several times the state dimension, your covariances are lying. → [10.05](../../10-localization/10.05-kalman-filter-multivariate.md),
   [10.06](../../10-localization/10.06-extended-kalman-filter.md)
2. **Process noise Q too small** → the filter stops listening to measurements. **Measurement noise R
   too small** → it chases noise. Set both from measured sensor statistics, not from taste. →
   [10.04](../../10-localization/10.04-kalman-filter-1d.md), [07.01](../../07-sensors/07.01-sensor-fundamentals.md)
3. **`LinAlgError: Singular matrix`** in the update means R (or S) is degenerate — usually a zero
   variance somewhere in a message that declared "unknown" as 0. Check the covariance conventions. →
   [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md), [10.06](../../10-localization/10.06-extended-kalman-filter.md)
4. **P losing positive-definiteness** after a while is numerical: use the Joseph form or symmetrise P
   each step. → [10.05](../../10-localization/10.05-kalman-filter-multivariate.md)

### Symptom: `/odometry/filtered` is identical to `/odom`

`robot_localization` is not actually fusing the second source: check the `imu0`/`odom0` config
matrices, that the topic names are right, and that the IMU's frame is in TF. → [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md)

### Symptom: the EKF output wanders more than raw odometry

1. The IMU's declared covariance is too optimistic, or its yaw is being fused in the wrong frame. →
   [07.06](../../07-sensors/07.06-imu-in-ros2.md), [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md)
2. Check you are not fusing the same information twice (odometry yaw *and* IMU yaw as absolute). →
   [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md)

### Symptom: the same bag gives a different map every time

Non-determinism in a pipeline that should be reproducible. Check thread counts, unseeded RNGs, and
whether messages are being dropped under load — a map that depends on CPU load is not a map you can
debug. → [11.09](../../11-slam/11.09-slam-troubleshooting.md), [03.08](../../03-robot-software/03.08-testing-robot-software.md)

### Symptom: the map is fine in RViz but the saved `.pgm` is worse

The saved map applies thresholds. Check `occupied_thresh`/`free_thresh` in the saved `.yaml`, and
remember unknown (−1) becomes a specific grey that other tools may read as free. → [11.02](../../11-slam/11.02-occupancy-grid-mapping.md),
[11.06](../../11-slam/11.06-slam-toolbox-simulation.md)

## Where to go next

- The systematic SLAM debugging method: [11.09](../../11-slam/11.09-slam-troubleshooting.md)
- What SLAM is made of (front end, back end, loop closure): [11.03](../../11-slam/11.03-the-slam-problem.md)
- Scan matching and why corridors are hard: [11.04](../../11-slam/11.04-scan-matching-icp.md)
- Pose graphs and loop closure: [11.05](../../11-slam/11.05-pose-graphs-loop-closure.md)
- Mapping your own home, in practice: [11.07](../../11-slam/11.07-mapping-your-room.md)
- AMCL and the `map → odom` transform: [10.09](../../10-localization/10.09-amcl-in-ros2.md)
- Filter consistency and NEES: [10.05](../../10-localization/10.05-kalman-filter-multivariate.md)
