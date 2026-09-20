# Troubleshooting: TF and frames

Frame bugs are the most common bugs in this course, and they are also the most mechanical to fix,
because tf2 tells you exactly what it could not do. Read the error text literally: it names the
frames, the time, and whether the problem is *existence*, *connectivity* or *timing*.

**The four questions, in order:**

1. **Does the frame exist?** → `ros2 run tf2_tools view_frames` (writes a PDF of the whole tree)
2. **Is the tree connected?** → one parent per frame, no second root
3. **Is the transform available at the time you asked for?** → buffer length, clock, stamps
4. **Is the transform correct?** → `tf2_echo`, then compare against a tape measure

→ [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md), [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)

> [!TIP]
> **Ask your teacher:** "Here is my `view_frames` output — is this a broken tree, a timing problem, or
> a wrong constant?"

---

### Symptom: `"xxx" passed to lookupTransform argument ... does not exist`

1. **Spelling and leading slash.** `base_link` and `/base_link` are different strings; ROS 2 does not
   use leading slashes. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
2. **Nobody publishes it yet.** Run `view_frames` and look: if the frame is absent, the publisher is
   not running, or it failed to start. `robot_state_publisher` is the usual missing one. →
   [05.08](../../05-frames-and-transforms/05.08-urdf-and-xacro.md)
3. **A namespace or a `frame_prefix` changed the name.** Simulated sensors often publish
   `karmel/base_footprint/lidar`, not `lidar`. Read the actual `frame_id` on the message with
   `ros2 topic echo --field header.frame_id`. → [06.07](../../06-simulation/06.07-simulated-sensors-and-noise.md), [06.05](../../06-simulation/06.05-robot-in-gazebo.md)
4. **The frame appears only while something runs.** A detection node that publishes an object frame
   only on detection will produce this error between detections; that is expected, and your code must
   handle the exception. → [05.10](../../05-frames-and-transforms/05.10-object-from-camera-to-map.md)

### Symptom: `Could not find a connection between 'a' and 'b' … two or more unconnected trees`

1. **`view_frames` will show two roots.** The fix is always the same: publish the one static transform
   that joins them. → [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md)
2. **The usual missing link** is `base_link → sensor_frame` (a sensor mount you never described) or
   `map → odom` (the localizer is not running). → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
3. **Two publishers of the same edge** create the opposite problem: a frame that flickers between two
   positions. Find both and delete one. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md), [15.10](../../15-manipulation/15.10-mobile-manipulation.md)
4. Every frame has exactly one parent. If you catch yourself wanting two, you want a different tree.
   → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)

### Symptom: `Lookup would require extrapolation into the future`

You asked for a transform at a time newer than the newest data.

1. **Are you asking at `now()` for data stamped slightly in the past?** Ask at the message's own
   stamp, or use `Time()` (zero) to mean "the latest available". → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
2. **Are the clocks the same?** In simulation, every node needs `use_sim_time: true`. One node on the
   wall clock and the rest on `/clock` produces exactly this error, at a stable offset. →
   [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md), [06.09](../../06-simulation/06.09-same-code-sim-and-real.md)
3. **Between two machines**, check NTP/chrony. Tens of milliseconds of clock skew is enough. →
   [03.10](../../03-robot-software/03.10-networking-for-robots.md)
4. **Use a timeout on the lookup** so a few-millisecond race waits instead of throwing. → [05.10](../../05-frames-and-transforms/05.10-object-from-camera-to-map.md)

### Symptom: `Lookup would require extrapolation into the past`

1. **The buffer is too short** for the delay in your pipeline (a slow detector asks about a frame from
   2 s ago). Increase the `Buffer` cache time. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
2. **The publisher stopped.** Check `ros2 topic hz /tf` for the specific broadcaster. → [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md)
3. **A bag is playing** without `--clock`, or your node is not on sim time. → [04.14](../../04-ros2/04.14-rosbag2.md)

### Symptom: lookups with a timeout always fail after exactly the timeout, although `tf2_echo` works

1. **You are not spinning.** A `Buffer`/`TransformListener` only fills while the node spins; a
   blocking `lookup_transform` inside a callback of a single-threaded executor starves the listener
   that would satisfy it. → [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md), [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
2. Fix by using the async API, or by giving the listener its own callback group and a multi-threaded
   executor. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md)

### Symptom: sensor data is rotated 90°, mirrored, or appears on a wall or ceiling

1. **Suspect the optical frame first.** `camera_link` is x-forward; `camera_optical_frame` is
   z-forward, x-right, y-down. A missing fixed rotation between them puts every detection on a wall.
   → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [13.05](../../13-computer-vision/13.05-pinhole-camera-model.md)
2. **Check REP-103**: x forward, y left, z up. A model imported from CAD with z-forward will look
   exactly like this. → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [FM.11](../../optional-foundations/mathematics/FM.11-euler-angles.md)
3. **Check the `rpy` in your URDF/xacro.** ROS uses fixed-axis XYZ; scipy's `from_euler('xyz')` with
   lowercase letters is intrinsic and gives a different matrix for the same three numbers. →
   [05.05](../../05-frames-and-transforms/05.05-rotations-in-3d.md), [FM.11](../../optional-foundations/mathematics/FM.11-euler-angles.md)
4. **Verify numerically, not visually**: transform a point you know the answer for (1 m in front of
   the sensor) and check the result in `base_link`. → [05.10](../../05-frames-and-transforms/05.10-object-from-camera-to-map.md)

### Symptom: transformed points are right when the robot faces +x, and wrong at other headings

This is the signature of a missing rotation: you added the translation and forgot to rotate.

1. The error's *size* stays constant and its *direction* turns with the robot. That pattern is
   diagnostic. → [05.03](../../05-frames-and-transforms/05.03-rigid-transforms-2d.md)
2. Apply R then t, in that order: `p_parent = R · p_child + t`. → [05.03](../../05-frames-and-transforms/05.03-rigid-transforms-2d.md), [05.04](../../05-frames-and-transforms/05.04-homogeneous-transforms.md)
3. Use the homogeneous 4×4 form and let matrix multiplication compose the chain for you, instead of
   doing it by hand. → [05.04](../../05-frames-and-transforms/05.04-homogeneous-transforms.md)

### Symptom: the points are off by a constant few centimetres

1. This is a *measurement* error, not a maths error: the sensor is not where the URDF says it is.
   Measure the mount offset with a ruler and put it in `karmel.yaml`/the xacro. → [05.04](../../05-frames-and-transforms/05.04-homogeneous-transforms.md),
   [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)
2. For the arm, a constant offset between "where it thinks it grasps" and "where it grasps" is
   hand-eye calibration. → [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)

### Symptom: the robot's position in RViz jumps, but the robot drives fine

1. **That is the design.** `map → odom` is corrected in jumps by the localizer; `odom → base_link` is
   smooth. Controllers must run in `odom`, goals live in `map`. → [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [10.09](../../10-localization/10.09-amcl-in-ros2.md)
2. If it jumps by metres, the localizer is unhappy, not TF: → [SLAM and localization](slam-localization.md).
3. If RViz's fixed frame is `odom` and the *map* appears to move instead, that is the same effect seen
   from the other side. → [05.09](../../05-frames-and-transforms/05.09-rviz.md)

### Symptom: RViz shows no robot although TF looks fine

1. **Global Status red** → the fixed frame does not exist. Set it to something that does. →
   [05.09](../../05-frames-and-transforms/05.09-rviz.md)
2. **Robot model display grey or empty** → the `robot_description` did not reach RViz. It is
   published transient-local; check the QoS and that `robot_state_publisher` started. → [05.08](../../05-frames-and-transforms/05.08-urdf-and-xacro.md),
   [04.11](../../04-ros2/04.11-qos.md)
3. **Mesh missing** → a `package://` path that does not resolve in the environment RViz runs in. →
   [05.08](../../05-frames-and-transforms/05.08-urdf-and-xacro.md)

### Symptom: part of the tree is missing, or a frame flickers between two positions

1. **Flicker = two publishers** for the same edge. Very common after adding the arm to the base:
   both bringups publish `base_link → arm_base`. → [15.10](../../15-manipulation/15.10-mobile-manipulation.md), [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
2. **Missing movable joints** in `/tf` means `robot_state_publisher` is running but `/joint_states`
   is not arriving. → [05.08](../../05-frames-and-transforms/05.08-urdf-and-xacro.md), [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

### Symptom: nothing works and I do not know where to start

Run this, in this order, every time. It takes two minutes and it has never failed to localise the
problem. → [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md)

```bash
ros2 run tf2_tools view_frames          # the whole tree as a PDF: existence + connectivity
ros2 run tf2_ros tf2_echo <parent> <child>   # a specific edge: values + timing
ros2 topic hz /tf /tf_static            # is anyone publishing, and how often
ros2 topic echo --field header.frame_id /<sensor_topic>   # what frame does the data claim
```

## Where to go next

- The systematic frame-debugging method: [05.11](../../05-frames-and-transforms/05.11-debugging-tf.md)
- tf2 broadcasting and lookups: [05.07](../../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)
- The standard frame conventions (REP-103, REP-105): [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
- URDF and xacro: [05.08](../../05-frames-and-transforms/05.08-urdf-and-xacro.md)
- Turning a detection into a map position: [05.10](../../05-frames-and-transforms/05.10-object-from-camera-to-map.md)
- Timing and synchronisation of sensor data: [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)
