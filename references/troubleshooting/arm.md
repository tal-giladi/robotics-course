# Troubleshooting: the arm and manipulation

> [!CAUTION]
> **Safety:** the arm moves faster than you can react and has pinch points along its whole length.
> Keep your face out of the workspace, start every session with low torque and speed limits, verify
> the e-stop before the first move, and never enable torque without knowing the current joint
> positions. → [14.11](../../14-robotic-arm/14.11-arm-safety.md), [SAFETY.md](../../SAFETY.md)

Manipulation failures fall into four layers, and the fastest way to fix one is to identify the layer
first:

| Layer | Question | Page/lesson |
|---|---|---|
| Servos | does the joint answer, hold, and report where it is? | [14.03](../../14-robotic-arm/14.03-controlling-servos.md) |
| Kinematics | does FK/IK agree with a ruler? | [14.04](../../14-robotic-arm/14.04-forward-kinematics.md), [14.05](../../14-robotic-arm/14.05-inverse-kinematics.md) |
| Planning | does MoveIt believe the world? | [14.09](../../14-robotic-arm/14.09-moveit2-motion-planning.md), [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md) |
| Perception → grasp | is the object where the pipeline says it is? | [15.03](../../15-manipulation/15.03-hand-eye-calibration.md), [15.04](../../15-manipulation/15.04-object-pose-estimation.md) |

---

## Servos

### Symptom: `lerobot-find-port` lists nothing, or `connect()` times out

1. Port name and permissions, exactly as for the Pico. → [FL.05](../../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md), [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md)
2. Bus power: the servo bus needs its own supply at the correct voltage, and the USB adapter's logic
   ground must be common with it. → [FE.11](../../optional-foundations/electronics/FE.11-servos-and-steppers.md), [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md)
3. Baud rate: smart servos ship at different default baud rates; scan. → [02.10](../../02-robot-electronics/02.10-can-bus-and-smart-actuators.md)

### Symptom: one motor answers and the next one does not

1. **Daisy chains fail at a point.** The last servo that answers tells you where. Check that
   connector and the cable after it. → [02.10](../../02-robot-electronics/02.10-can-bus-and-smart-actuators.md), [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md)
2. **Duplicate ids** make two servos answer at once and both appear broken. Set ids one at a time,
   with only that servo connected. → [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md)
3. **Voltage drop along the chain**: the last servo may be browning out under load while the first is
   fine. Measure at the far end while moving. → [FE.11](../../optional-foundations/electronics/FE.11-servos-and-steppers.md), [02.10](../../02-robot-electronics/02.10-can-bus-and-smart-actuators.md)
4. Errors that get worse as the robot *moves* are cable flex, not software. → [02.07](../../02-robot-electronics/02.07-wiring-harness.md)

### Symptom: the arm jumps violently the moment I enable torque

The servo is holding the last commanded goal, which is not where the arm currently is.

1. **Always read the present positions and write them as the goal *before* enabling torque.** This is
   the rule, not a workaround. → [14.03](../../14-robotic-arm/14.03-controlling-servos.md), [14.11](../../14-robotic-arm/14.11-arm-safety.md)
2. Enable joints one at a time, from the base up, at a low torque limit. → [14.11](../../14-robotic-arm/14.11-arm-safety.md)
3. If it still jumps, a calibration offset of ~360° on one joint will do it. → [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md)

### Symptom: the arm collapses whenever the script ends

Torque is disabled on exit — often by the servo library's own cleanup. Decide deliberately: hold
position (and accept the heating), or move to a rest pose and then release. Never leave it to chance
with the arm extended over something breakable. → [14.03](../../14-robotic-arm/14.03-controlling-servos.md), [14.11](../../14-robotic-arm/14.11-arm-safety.md)

### Symptom: a joint sags, buzzes, or gets hot while holding

1. **Sagging** is insufficient holding torque for the load at that extension. Reduce the payload, add
   a counterweight, or keep the arm closer in. → [14.01](../../14-robotic-arm/14.01-arm-anatomy.md), [FE.11](../../optional-foundations/electronics/FE.11-servos-and-steppers.md)
2. **Buzzing** is the servo's internal loop oscillating: reduce its P gain, or check for a mechanical
   backlash the loop is chasing. → [14.03](../../14-robotic-arm/14.03-controlling-servos.md)
3. **Hot** means it is stalling against something. Find the obstruction before you raise the torque
   limit. A hot servo is a servo about to fail. → [14.11](../../14-robotic-arm/14.11-arm-safety.md)
4. **`supply X V outside 4.5–8.4 V`** is a real measurement, not a spurious warning. Fix the supply. →
   [14.03](../../14-robotic-arm/14.03-controlling-servos.md), [02.02](../../02-robot-electronics/02.02-power-budget.md)

### Symptom: `move did not finish within 20 s`

1. Check the joint actually moved at all (read present position). No movement → torque off, blocked,
   or the goal is outside the servo's limits. → [14.03](../../14-robotic-arm/14.03-controlling-servos.md)
2. Check the goal is inside the software joint limits you set, and that those match the URDF. →
   [14.03](../../14-robotic-arm/14.03-controlling-servos.md), [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

---

## Kinematics

### Symptom: FK is right at q = 0 and wrong everywhere else

A parameter, not the maths.

1. **Check the joint axes and their signs** in your DH table or URDF against the physical arm: rotate
   one joint by 90° and compare. → [14.04](../../14-robotic-arm/14.04-forward-kinematics.md)
2. **Check angle offsets**: the servo's zero is not the model's zero unless you made it so. →
   [14.02](../../14-robotic-arm/14.02-buying-and-assembling-the-arm.md), [14.04](../../14-robotic-arm/14.04-forward-kinematics.md)
3. **Check degrees vs radians** at the boundary. → [FP.01](../../optional-foundations/physics/FP.01-kinematics-units.md)

### Symptom: FK is right in simulation and 2 cm off on the real arm

Link lengths. Measure the real arm with calipers; printed and moulded parts differ from the drawing,
and the gripper's tool point is rarely where you assumed. → [14.04](../../14-robotic-arm/14.04-forward-kinematics.md), [14.01](../../14-robotic-arm/14.01-arm-anatomy.md)

### Symptom: IK "fails" on a point I can see the arm reaching

1. **Check the point is in the workspace *with the requested orientation*.** A 5-DOF arm cannot
   achieve every position-and-orientation pair; the position alone may be reachable and the pose not.
   → [14.01](../../14-robotic-arm/14.01-arm-anatomy.md), [14.05](../../14-robotic-arm/14.05-inverse-kinematics.md)
2. **Check the joint limits** used by the solver match the real ones. → [14.05](../../14-robotic-arm/14.05-inverse-kinematics.md)
3. **Check the seed**: a numerical solver started far from a solution may not converge. Seed it with
   the current configuration. → [14.05](../../14-robotic-arm/14.05-inverse-kinematics.md)
4. **Near the workspace edge the solver oscillates** — that is a singularity, and damping (damped
   least squares) is the fix, not more iterations. → [14.06](../../14-robotic-arm/14.06-jacobian.md)
5. "Results are non-deterministic between runs" means a random seed; fix it if you need
   repeatability. → [14.05](../../14-robotic-arm/14.05-inverse-kinematics.md)

### Symptom: the arm shudders or lunges during a Cartesian jog

1. **Singularity.** `LinAlgError: Singular matrix` is the same thing, caught. Use the damped
   pseudo-inverse and clamp joint velocities. → [14.06](../../14-robotic-arm/14.06-jacobian.md)
2. **Check the Jacobian against a numerical one** (finite differences) before blaming the hardware —
   they should agree to several decimal places. → [14.06](../../14-robotic-arm/14.06-jacobian.md)
3. **One joint saturating** bends the path: the tool leaves the straight line because a joint hit its
   velocity limit. Slow the Cartesian speed. → [14.06](../../14-robotic-arm/14.06-jacobian.md), [14.07](../../14-robotic-arm/14.07-trajectories.md)

### Symptom: a trajectory is correct in simulation and lurches on the real arm

1. **Check the joints arrive together**: a trajectory that ignores per-joint velocity limits finishes
   at different times per joint. → [14.07](../../14-robotic-arm/14.07-trajectories.md)
2. **`cannot move 0.9 in 0.500 s with a_max = 2.0`** is the profile telling you the request is
   infeasible. Believe it. → [14.07](../../14-robotic-arm/14.07-trajectories.md)
3. **Jerk at the end** means the profile does not decelerate to zero, or the tolerance band is
   entered at speed. → [14.07](../../14-robotic-arm/14.07-trajectories.md)

---

## Planning and the scene

### Symptom: `Controller 'arm_controller' failed to activate`

1. The joint names in the controller YAML must exactly match the URDF's joint names. → [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)
2. The hardware interface must be up and claiming those joints. → [06.06](../../06-simulation/06.06-ros2-control.md)

### Symptom: the trajectory aborts with `PATH_TOLERANCE_VIOLATED`

The arm cannot follow the commanded trajectory closely enough.

1. Slow the trajectory down first — that alone tells you whether it is a speed problem or a tracking
   problem. → [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)
2. Check for a sagging joint (insufficient torque) or an obstruction. → [14.03](../../14-robotic-arm/14.03-controlling-servos.md)
3. Widen the tolerance only once you know why it was exceeded. → [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

### Symptom: every plan fails with "Start state appears to be in collision"

1. **The model disagrees with reality, or with itself.** Display the planning scene in RViz and look —
   usually two adjacent links whose collision geometry overlaps at the current pose. → [14.09](../../14-robotic-arm/14.09-moveit2-motion-planning.md)
2. **Check the allowed collision matrix** for the adjacent pairs. → [14.09](../../14-robotic-arm/14.09-moveit2-motion-planning.md)
3. **Check `/joint_states` is current.** A stale start state is "in collision" with a world that has
   moved. → [14.08](../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

### Symptom: the plan is collision-free in RViz and the arm hits the object

1. **The scene is stale or incomplete.** MoveIt only avoids what it knows about. → [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md)
2. **The held object must be attached** after the grasp, or the planner will drive it through things
   (and, confusingly, will also try to avoid it as a separate obstacle). → [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md)
3. **Collision geometry is smaller than reality** — the padded mesh, the gripper pads, the cable loom.
   → [14.09](../../14-robotic-arm/14.09-moveit2-motion-planning.md), [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md)

### Symptom: the planner refuses motions it used to manage, or every grasp pose is a collision

1. You added collision objects (the table) with too much padding, or the object's bounding box
   includes the surface it stands on. → [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md)
2. Remove the support surface from the object cluster before building its collision shape. →
   [15.04](../../15-manipulation/15.04-object-pose-estimation.md)

---

## Perception to grasp

### Symptom: the gripper is repeatable to 1 mm but 15 mm from the target

A constant offset is a calibration, not a noise problem.

1. **Hand-eye calibration.** Solve AX = XB with enough varied poses; a residual above ~10 mm means
   the poses were not varied enough (rotate, do not just translate). → [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)
2. **Check which frame the calibration returned** — eye-in-hand and eye-to-hand are different
   equations and swapping them gives a plausible, wrong answer. → [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)
3. **Check the tool frame**: the calibration may be right and your TCP offset wrong. → [14.04](../../14-robotic-arm/14.04-forward-kinematics.md)
4. "The rotation is right and the translation is wrong by a constant" points specifically at the tool
   offset or the target board's origin. → [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)

### Symptom: scatter of 5 mm or more between identical runs

Not calibration — repeatability. Look at servo backlash, a flexing mount, and whether the perception
pipeline itself is noisy (run it 20 times on a stationary object and measure the spread). →
[15.07](../../15-manipulation/15.07-detect-and-approach.md), [15.04](../../15-manipulation/15.04-object-pose-estimation.md)

### Symptom: the object pose estimate is offset in one direction, or the yaw flips between frames

1. **A consistent offset** usually means the centroid of a *partial* point cloud (you see one side of
   the object) rather than its centre. Model it, or grasp relative to the visible face. →
   [15.04](../../15-manipulation/15.04-object-pose-estimation.md)
2. **Yaw flipping by 180°** is a symmetric object with an ambiguous principal axis. Constrain it to
   the half-turn the gripper can reach. → [15.04](../../15-manipulation/15.04-object-pose-estimation.md)
3. **Two objects merging into one** is a clustering tolerance that is too large; **far too many
   objects** is one that is too small, or the table plane was not removed. → [15.04](../../15-manipulation/15.04-object-pose-estimation.md),
   [13.08](../../13-computer-vision/13.08-depth-and-3d-vision.md)

### Symptom: the grasp is right in simulation and misses on the robot

1. Re-measure the hand-eye calibration on the real setup. → [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)
2. Check the pre-grasp approach is along the gripper's axis: "the arm arcs over the object on its way
   down" means it is interpolating in joint space, not Cartesian. → [15.07](../../15-manipulation/15.07-detect-and-approach.md)
3. "The object is detected in the wrong place while the arm is moving" is a latency/stamp problem:
   the detection is being transformed with a pose from a different moment. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md), [15.07](../../15-manipulation/15.07-detect-and-approach.md)

### Symptom: the object slides out of the jaws during the lift

1. Compute the required friction force and compare it against the friction cone at your grip force —
   the number will usually tell you immediately whether it is grip force or geometry. → [15.01](../../15-manipulation/15.01-physics-of-grasping.md)
2. **The object rotating rather than sliding** means the grasp line does not pass near the centre of
   mass. → [15.01](../../15-manipulation/15.01-physics-of-grasping.md)
3. **The object squirting out as the jaws close** means the two contacts are not opposed; add
   compliance or approach differently. → [15.02](../../15-manipulation/15.02-grippers.md)

### Symptom: the robot tips forward when the arm extends

Static stability: the combined centre of mass left the support polygon. Retract before driving,
restrict the arm's workspace while the base is not stationary, and check the number rather than
guessing. → [FP.07](../../optional-foundations/physics/FP.07-center-of-mass-stability.md), [15.10](../../15-manipulation/15.10-mobile-manipulation.md)

### Symptom: the arm cannot reach the object after a successful navigation

Navigation's goal tolerance is larger than the arm's workspace margin. Either tighten the final
approach (a visual-servo or a short base correction), or plan the base pose from the object's
position rather than from a fixed offset. → [15.10](../../15-manipulation/15.10-mobile-manipulation.md), [15.07](../../15-manipulation/15.07-detect-and-approach.md)

### Symptom: TF is full of duplicate or conflicting frames after adding the arm

Two bringups publishing the same edge. → [TF and frames](tf-and-frames.md), [15.10](../../15-manipulation/15.10-mobile-manipulation.md)

## Where to go next

- Arm anatomy, workspace and DOF: [14.01](../../14-robotic-arm/14.01-arm-anatomy.md)
- Servo control, torque enable and joint limits: [14.03](../../14-robotic-arm/14.03-controlling-servos.md)
- Arm safety (the rules, not the suggestions): [14.11](../../14-robotic-arm/14.11-arm-safety.md)
- Hand-eye calibration: [15.03](../../15-manipulation/15.03-hand-eye-calibration.md)
- The full detect → plan → grasp pipeline and where it breaks: [15.07](../../15-manipulation/15.07-detect-and-approach.md)
- Collision avoidance and the planning scene: [15.09](../../15-manipulation/15.09-collision-avoidance-and-recovery.md)
