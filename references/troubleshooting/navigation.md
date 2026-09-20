# Troubleshooting: navigation

Nav2 is a stack of lifecycle nodes, and it fails in layers. Diagnose top-down, because a costmap
problem and a TF problem produce identical behaviour from the outside:

```text
  goal ──▶ BT navigator ──▶ planner server ──▶ global costmap ──▶ map + TF
                    │                                              ▲
                    └──▶ controller server ──▶ local costmap ──▶ sensors
                                    │
                                    └──▶ cmd_vel ──▶ velocity smoother ──▶ base
```

**Before anything else, check the layers underneath.** Nav2 assumes odometry, TF and localization are
correct; if they are not, every Nav2 symptom is a lie. → [12.06](../../12-navigation/12.06-nav2-architecture.md)

> [!CAUTION]
> **Safety:** first autonomous runs happen in a clear area, at ≤ 0.3 m/s, with a hand on the stop, and
> stairs blocked. → [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md), [SAFETY.md](../../SAFETY.md)

---

### Symptom: `ros2 action send_goal /navigate_to_pose` says the action server is not available

1. **Check the lifecycle state**, not whether the process is running:
   `ros2 lifecycle get /bt_navigator`. Nav2 servers sit `inactive` until the lifecycle manager
   activates them, and they look perfectly alive in `ros2 node list`. → [04.15](../../04-ros2/04.15-lifecycle-nodes.md), [12.06](../../12-navigation/12.06-nav2-architecture.md)
2. **If the lifecycle manager reports a failure**, one server failed to configure — usually a
   parameter file problem. Read the log from the *first* failure, not the last. → [12.07](../../12-navigation/12.07-nav2-in-simulation.md),
   [04.09](../../04-ros2/04.09-parameters.md)
3. **Namespace mismatch**: the action is `/navigate_to_pose` only if nothing namespaced it. →
   [04.10](../../04-ros2/04.10-launch-files.md)

### Symptom: goals abort instantly with "Timed out while waiting for action server to acknowledge goal request"

1. The server is overloaded or blocked, not absent. Check CPU on the Pi. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
2. Check the BT navigator is active and the behavior tree XML it loaded actually exists — a missing
   node plugin fails at tick time. → [12.06](../../12-navigation/12.06-nav2-architecture.md)
3. `waitUntilNav2Active()` hanging in a Simple Commander script is the same condition seen from the
   Python side. → [12.09](../../12-navigation/12.09-waypoints-and-missions.md)

### Symptom: the goal aborts with error 202 (TF_ERROR)

TF, not navigation. The `map → odom → base_link` chain is broken or stale.
→ [TF and frames](tf-and-frames.md), [12.07](../../12-navigation/12.07-nav2-in-simulation.md)

### Symptom: the goal aborts with 208 (NO_VALID_PATH) although the route looks open

1. **Look at the costmap, not the map.** In RViz, display the global costmap and see what the planner
   sees. Nine times in ten the corridor is closed by inflation. → [12.04](../../12-navigation/12.04-costmaps.md), [12.07](../../12-navigation/12.07-nav2-in-simulation.md)
2. **Check the footprint and inflation radius** against the real doorway width. Inflation radius
   larger than half the doorway makes the doorway impassable, correctly. → [12.04](../../12-navigation/12.04-costmaps.md)
3. **Check the goal is not inside an inflated cell.** A goal 10 cm from a wall is often in lethal or
   inscribed space. → [12.04](../../12-navigation/12.04-costmaps.md)
4. **Check for stale obstacles** left in the costmap after the obstacle moved — see the next symptom.
5. Only then look at the planner's own parameters. → [12.02](../../12-navigation/12.02-grid-path-planning.md), [12.06](../../12-navigation/12.06-nav2-architecture.md)

### Symptom: obstacles stay in the costmap after they are gone

1. **Clearing requires ray casting through free space**, which requires the sensor to actually see
   through the cell. A 2D scan cannot clear a cell it cannot reach. → [12.04](../../12-navigation/12.04-costmaps.md), [11.02](../../11-slam/11.02-occupancy-grid-mapping.md)
2. **Check `raytrace_max_range` vs `obstacle_max_range`**: if obstacles are marked further than they
   are cleared, they accumulate for ever. → [12.04](../../12-navigation/12.04-costmaps.md)
3. **Check the sensor's stamps and frame.** Marking uses the transform at the scan's time; a bad
   stamp smears obstacles across the map. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)

### Symptom: obstacles appear that are not there

1. The LiDAR is seeing part of the robot, or the floor because of a tilt in the mount. → [07.07](../../07-sensors/07.07-2d-lidar.md),
   [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
2. Reflective floors and glass produce phantom returns. → [07.07](../../07-sensors/07.07-2d-lidar.md)
3. The costmap's `z` filtering is wrong for a 3D source. → [12.04](../../12-navigation/12.04-costmaps.md)

### Symptom: the robot drives straight into an obstacle that is not in the map

1. **Is the obstacle layer subscribed and receiving?** `ros2 topic hz` on the scan topic the layer is
   configured with — a typo in the topic name silently disables the layer. → [12.04](../../12-navigation/12.04-costmaps.md),
   [12.01](../../12-navigation/12.01-the-navigation-problem.md)
2. **Is the local costmap updating?** Watch it in RViz while you put a box in front of the robot. →
   [12.04](../../12-navigation/12.04-costmaps.md)
3. **Is the collision monitor configured?** It is the independent last line of defence and it does not
   depend on the costmap being right. → [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)

### Symptom: the robot weaves left and right along a straight path

1. **Controller gains before planner settings.** The weave is the local controller over-correcting;
   reduce its angular gain or increase the look-ahead distance. → [12.05](../../12-navigation/12.05-local-planning-obstacle-avoidance.md), [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
2. **Check the base's own heading loop is not fighting it.** Two controllers correcting heading at
   different rates produces exactly this. → [08.10](../../08-control/08.10-straight-line-heading-control.md)
3. **Check odometry quality.** A noisy `/odom` yaw makes any path follower weave. →
   [Encoders, odometry and control](control-and-odometry.md)
4. **Check the control loop is achieving its rate.** "Controller missed its desired rate" in the log
   means the weave is a timing artefact. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md), [04.12](../../04-ros2/04.12-executors-and-callbacks.md)

### Symptom: the robot cuts corners into walls

Pure-pursuit-style controllers cut corners by construction. Reduce the look-ahead, increase the
inflation, or use a controller with an explicit path-following cost (MPPI). → [12.05](../../12-navigation/12.05-local-planning-obstacle-avoidance.md)

### Symptom: DWB/MPPI stops in front of an obstacle and never goes around

1. The local planner searches a *velocity* window; if every sampled trajectory collides, it stops.
   Check the acceleration limits are not so tight that no evasive trajectory is feasible. →
   [12.05](../../12-navigation/12.05-local-planning-obstacle-avoidance.md)
2. Check the critics' weights: a very high obstacle cost with a low path-alignment cost makes stopping
   the best option. → [12.05](../../12-navigation/12.05-local-planning-obstacle-avoidance.md)
3. Replanning may not be enabled: the robot follows a stale global path into a new obstacle. →
   [12.06](../../12-navigation/12.06-nav2-architecture.md)

### Symptom: the robot reaches the goal area but never reports success

1. **Goal tolerances**: `xy_goal_tolerance` and `yaw_goal_tolerance`. A yaw tolerance tighter than the
   robot's rotational resolution produces an infinite last turn. → [12.07](../../12-navigation/12.07-nav2-in-simulation.md), [12.01](../../12-navigation/12.01-the-navigation-problem.md)
2. The deadband again: the final correction is too small to move the wheels. → [08.11](../../08-control/08.11-rotate-exactly-90.md)
3. Check which node owns the goal check — the controller's goal checker, not the BT. → [12.06](../../12-navigation/12.06-nav2-architecture.md)

### Symptom: the robot creeps everywhere at a fraction of its speed

1. The speed-limit filter or a speed-restricted zone is active. → [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)
2. The velocity smoother's acceleration limits are too tight for the commanded profile. →
   [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
3. The controller is de-rating for proximity to obstacles that are not really there. → [12.04](../../12-navigation/12.04-costmaps.md)

### Symptom: navigation keeps running recoveries and eventually aborts

Recoveries are a symptom, never a cause.

1. **Look at what triggered the first one**, in the log. Everything after it is consequence. →
   [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)
2. The usual roots: localization lost (→ [SLAM and localization](slam-localization.md)), costmap
   wrong, or the robot physically stuck.
3. **If the robot wedges and no recovery frees it**, the recovery set is wrong for the geometry —
   back-up needs clearance behind, spin needs clearance around. → [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)

### Symptom: a keepout zone has no effect

1. The costmap filter needs both the filter mask *and* the filter info topic, and the filter must be
   listed in the costmap's plugin list. Missing any one of the three fails silently. → [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)
2. Check the mask's resolution and origin match the map's. → [11.01](../../11-slam/11.01-map-representations.md)

### Symptom: everything worked in simulation and nothing works on the robot

1. **Re-check the layers below Nav2 on hardware**: odometry calibration, TF, and localization. Nav2 is
   almost never the thing that broke. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
2. **Real sensors have noise, latency and blind spots** the simulated ones did not. Re-tune inflation
   and the obstacle layer against real scans. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md), [06.10](../../06-simulation/06.10-sim-to-real-gap.md)
3. **The Pi is slower than your laptop.** MPPI in particular may not achieve its rate; measure before
   blaming behaviour. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
4. **Nav2 shutting itself down after a minute on the Pi** is usually memory or a watchdog on a server
   that missed its rate. Check `journalctl` and free memory. → [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md), [FL.06](../../optional-foundations/linux-and-tools/FL.06-processes-systemd.md)

### Symptom: Ctrl-C leaves the robot driving

The mission script died without cancelling the goal. Always cancel in a `finally`, and rely on the
`cmd_vel` watchdog as the backstop — not as the plan. → [12.09](../../12-navigation/12.09-waypoints-and-missions.md), [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)

## Where to go next

- The navigation problem, stated: [12.01](../../12-navigation/12.01-the-navigation-problem.md)
- Costmaps, layers and inflation: [12.04](../../12-navigation/12.04-costmaps.md)
- Local planners and their trade-offs: [12.05](../../12-navigation/12.05-local-planning-obstacle-avoidance.md)
- Nav2's architecture and behavior tree: [12.06](../../12-navigation/12.06-nav2-architecture.md)
- Tuning Nav2 on the real robot: [12.08](../../12-navigation/12.08-nav2-on-the-real-robot.md)
- Recoveries, keepouts and navigation safety: [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)
