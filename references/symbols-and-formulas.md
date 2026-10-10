# Symbols and formulas

Every math symbol and every formula used in the course, on one page, with the lesson that teaches it. Use it to look up what a letter means (is $k$ a gain, a counter or a spring constant here?) or to find a formula without hunting for the lesson. The lesson is always the place where the formula is derived and tried with real numbers; this page is only the index.

- **The same letter means different things in different lessons.** $k$ is a time step in a filter, a spring constant in physics and a gain in control. Each meaning has its own row; the lesson column tells you which one you are reading.
- **Units are SI**: metres, radians, seconds, volts, amps, kilograms. "—" means the quantity has no unit.
- **Words, not letters?** Terms like *innovation* or *duty cycle* are in the [Glossary](glossary.md).

> [!TIP]
> **Ask your teacher:** "In lesson 10.06, which of these symbols are inputs, which are the filter's state, and which does the filter compute?"

**Symbols:** [A](#a) [B](#b) [C](#c) [D](#d) [E](#e) [F](#f) [G](#g) [H](#h) [I](#i) [J](#j) [K](#k) [L](#l) [M](#m) [N](#n) [O](#o) [P](#p) [Q](#q) [R](#r) [S](#s) [T](#t) [U](#u) [V](#v) [W](#w) [X](#x) [Y](#y) [Z](#z) [α](#greek-alpha) [β](#greek-beta) [γ](#greek-gamma) [δ Δ](#greek-delta) [ε](#greek-epsilon) [ζ](#greek-zeta) [η](#greek-eta) [θ](#greek-theta) [κ](#greek-kappa) [λ](#greek-lambda) [μ](#greek-mu) [ν](#greek-nu) [π](#greek-pi) [ρ](#greek-rho) [σ](#greek-sigma) [τ](#greek-tau) [φ](#greek-phi) [χ](#greek-chi) [ψ](#greek-psi) [ω](#greek-omega) [other](#other-symbols)

**Formulas:** [Mathematics foundations (FM)](#mathematics-foundations-fm) · [First robot, electronics and robot software (00–03, FE)](#first-robot-electronics-and-robot-software-0003-fe) · [ROS 2, frames, simulation, sensors and physics (04–07, FP)](#ros-2-frames-simulation-sensors-and-physics-0407-fp) · [Control, odometry and localization (08–10, FCT)](#control-odometry-and-localization-0810-fct) · [SLAM, navigation and computer vision (11–13, FCV)](#slam-navigation-and-computer-vision-1113-fcv) · [Robotic arms and manipulation (14–15, labs)](#robotic-arms-and-manipulation-1415-labs) · [Machine learning, RL, embodied AI and LLM agents (16–19, FML)](#machine-learning-rl-embodied-ai-and-llm-agents-1619-fml) · [Final robot, module 21, projects and other foundations](#final-robot-module-21-projects-and-other-foundations)

---

## Symbols

Sorted by letter. Bold vectors and matrices ($\mathbf{x}$, $P$), hats ($\hat{x}$, an estimate) and subscripts ($v_L$) sort under their base letter. Greek letters come after Z, in Greek alphabetical order.

### A

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $A$ | ambient photons (light the sensor did not send) | — | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $A$ | area of a contour or mask | px² | [13.03](../13-computer-vision/13.03-classical-detection.md), [13.11](../13-computer-vision/13.11-segmentation.md) |
| $A$ | demonstrated action chunk | unit of action | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $A$ | DLT matrix of stacked correspondences | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $A$ | frontal (cross-section) area of a projectile | m² | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $A$ | motion of the gripper between two poses (from FK) | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $A$ | part surface area | cm² | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $A$ | suction-cup area | m² | [15.02](../15-manipulation/15.02-grippers.md) |
| $A$ | wire cross-section area | m² | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $A(s, a)$ | advantage: how much better than average an action was | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $A, B$ | events (sets of outcomes) | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $A, B$ | matrices (capital letters) | varies | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $A$, $B$ | state-space system and input matrices ($\dot x = Ax + Bu$) | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $A$, $B$ | the two encoder channels (each 0 or 1) | — | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $A, B, P$ | path segment start, segment end, robot position | m | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $A$, $b$ | matrix and offset of an affine correction | —, m | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $A/m$ | area-to-mass ratio, sets how hard drag slows a projectile | m²/kg | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $A^+$ | pseudo-inverse: least-squares or minimum-norm solver for any matrix | — | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^{-1}$ | inverse matrix: undoes $A$ | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $A^0$ | clean demonstrated action chunk | normalized | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\hat A^0$, $\hat\varepsilon$ | predicted clean chunk and predicted noise | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $A^T$ | transpose: rows and columns swapped | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $A^k$ | action chunk after $k$ noise steps | normalized | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $A_{IPROPI}$ | current-mirror gain of the DRV8874 (µA out per A of motor current) | A/A | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $A_d$, $B_d$ | discrete-time (zero-order-hold) versions of $A$, $B$ | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $A_{ij}, B_{ij}$ | gripper / camera relative motion between poses $i$ and $j$ | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $A_{ij}$, $B_{ij}$ | Jacobians: how edge error changes with node $i$ / node $j$ | varies | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $A_{\text{narrow}}$, $A$ | area of a narrow passage / of the whole sampling region | m² | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $A_t$, $\hat A_t$ | advantage at step $t$ and its estimate | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $a$ | acceleration (in the constant-velocity model) | m/s² | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $a$ | action | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $a$ | amplitude of the limit cycle the relay produces | rad/s | [08.08](../08-control/08.08-tuning-pid.md) |
| $a$ | apparent size of the blob | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $a$ | braking deceleration | m/s² | [00.02](../00-orientation/00.02-the-robotics-stack.md), [00.06](../00-orientation/00.06-safety-mindset.md) |
| $a$ | braking deceleration (or acceleration) | m/s² | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $a$ | centre distance between two meshing gears | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $a$ | deceleration (braking) | m/s² | [12.01](../12-navigation/12.01-the-navigation-problem.md), [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $a$ | Euler gap-shrink factor per step, $1 - \Delta t/\tau$ | — | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $a$ | extra vertical acceleration of the lift | m/s² | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $a$ | forward acceleration of the robot (tipping check) | m/s² | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\hat{\mathbf{a}}$ | joint axis unit vector | — | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $a$ | linear acceleration (or deceleration when braking) | m/s² | [04.08](../04-ros2/04.08-actions.md), [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $a$ | linear acceleration or deceleration of the robot | m/s² | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $a$ | neuron output (activation) | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $a$ | odometry yaw-rate scale error (wheel-separation error) | — | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $a$ | slope of range noise with distance, $\sigma_r = a\,r$ | — | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $\hat a$ | unit vector of a rotation axis | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\lVert a \rVert$ | length (norm) of a vector | — | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $a'$ | candidate next action | — | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\mathbf a$, $\mathbf b$ | two embedding vectors | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $a$, $b$ | exact discrete wheel-model coefficients: carry-over and input gain per period | —, rad/s per duty | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $a$, $b$ | half-length and half-width of a rectangular footprint ($a \ge b$) | m | [12.04](../12-navigation/12.04-costmaps.md) |
| $a$, $b$ | kernel half-widths: kernel size $(2a+1)\times(2b+1)$ | px | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $a$, $b$ | slope / offset of a current-sense calibration fit | A/V, A | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $a$, $b$ | slope and intercept of a learned line | varies | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $a, b, c$ | side lengths of a triangle (law of cosines); $a$, $b$ also arm link lengths | m | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $a$, $b$, $c$ | vertex vectors of one STL triangle | mm | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $\mathbf{a}\cdot\mathbf{b}$ | dot product: how much two vectors point the same way | product of units | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{a}\times\mathbf{b}$ | cross product: vector perpendicular to both, length = parallelogram area | product of units | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $[\hat a]$ | skew-symmetric (cross-product) matrix of $\hat a$ | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $a_0$ | DDPM: the clean demonstrated action | model units | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a_0$ | flow matching: the noise sample at the start of the path | model units | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a_0 \ldots a_3$ | coefficients of a cubic trajectory polynomial | rad/sᵏ | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $a_0 \dots a_3$ | coefficients of a characteristic polynomial | — | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $a_1$ | flow matching: the demonstrated action at the end of the path | model units | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a_{\max}$ | acceleration limit | rad/s² (or °/s², m/s²) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $a_{\max}$ | largest acceleration before the wheels slip | m/s² | [06.05](../06-simulation/06.05-robot-in-gazebo.md), [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $a_{\min}, a_{\max}$ | range of one action dimension | unit of action | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $a_c$ | centripetal acceleration (towards the centre of a turn) | m/s² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $a_i, b_i, c_i$ | coefficients and right-hand side of linear equation $i$ ($a_i x + b_i y = c_i$) | varies | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\mathbf{a}_i^\top$ | row $i$ of $A$ (one equation) | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $a_{ij}$ | entry in row $i$, column $j$: how much input $j$ contributes to output $i$ | varies | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $a_k$ | alignment of field $k$ (its start must be a multiple of this) | bytes | [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md) |
| $a_{max}$ | acceleration limit of a motion profile / command ramp | rad/s² (or m/s²) | [08.11](../08-control/08.11-rotate-exactly-90.md), [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $a_{max}, v_{max}$ | acceleration and speed limits of a motion profile | m/s², m/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $a_{\text{next}}$ | action actually taken next (SARSA) | — | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $a_t$ | action commanded at time $t$ | — | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $a_t$ | noisy action at step or time $t$ | model units | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a_t$ | tangential acceleration of a point on a spinning body | m/s² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $a_t^{(i)}$ | $i$-th prediction (oldest first) for the action at time $t$ | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $a_\text{tip}$ | acceleration at which the robot starts to tip over | m/s² | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $a_{tip}$ | base acceleration at which the robot tips over | m/s² | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $a_{tip}, b_{tip}$ | acceleration / braking deceleration that tips the robot over | m/s² | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $a_{\text{tool}}$ | acceleration of the tool | m/s² | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $a_v$, $a_\omega$ | linear and angular acceleration limits | m/s², rad/s² | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $a_x, a_y, a_z$ | accelerometer readings along the sensor's $x$, $y$, $z$ axes | m/s² | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $a_x, a_y, a_z$ | accelerometer readings along x, y, z | m/s² | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md), [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $a_x, a_y, a_z$ | components of the tool's approach axis (where the gripper points) | — | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\mathrm{acc}(b)$, $\mathrm{conf}(b)$ | accuracy and mean confidence in bin $b$ | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\text{age}_n$ | age of the data being acted on after $n$ messages | s | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| AP, mAP | average precision (one class) / mean over classes | — | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| ${}^{a}p$ | point $p$ expressed in frame $a$ | m | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{a}T_{b}$ | transform giving frame $b$'s pose in frame $a$; maps points from $b$ to $a$ | — | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $\operatorname{atan2}(y, x)$ | angle of the point $(x, y)$ in $(-\pi, \pi]$, correct in all four quadrants | rad | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |

### B

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $B$ | basis matrix: columns are frame C's axes written in frame A | — | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $B$ | batch of transitions | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $B$ | batch size | — | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md), [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $B$ | bytes per image frame | B | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $B$ | control matrix: how the input enters the state | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $B$ | magnetic field strength | T | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $B$ | motion of the camera (target seen from the camera) between the same two poses | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $B$ | number of LiDAR beams used per particle | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $B$ | stereo baseline: distance between the two cameras | m | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md), [11.08](../11-slam/11.08-visual-slam.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $B$ | total bandwidth a robot's topics use | B/s | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $b$ | action token bin index (0–255) | — | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $b$ | bias (constant offset) term of a model | varies | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $b$ | bytes stored per map cell | B | [11.01](../11-slam/11.01-map-representations.md) |
| $b$ | constant gyro bias (drift rate) | rad/s | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $b$ | constant sensor bias (gyro or accelerometer) | rad/s or m/s² | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $b$ | damping slope of the motor line, $\tau_s/\omega_0$ | N·m·s/rad | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $b$ | effective wheelbase | m | [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md) |
| $b$ | gear face width | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $b$ | gradient vector of the pose graph in Gauss-Newton | varies | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $b$ | gyro bias | rad/s | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md), [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $b$ | offset of the printer's hole-size fit | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $\mathbf{b}$ | right-hand side vector (measurements) | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $b$ | sensor bias: constant offset of the readings | same as reading | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md), [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $b$ | SigLIP learned bias | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $b$ | track width (wheel separation), ros2_control name | m | [06.06](../06-simulation/06.06-ros2-control.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $b$ | track width / wheel separation: distance between the two tyre centre-lines | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $b$ | wheel separation (same as $L$, notation used in FM.19) | m | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $b$ | wheel separation (track width) | m | [P03](../projects/P03-encoder-movement.md) |
| $b$ | wheel separation (track width) between the tyre contact centres | m | [08.01](../08-control/08.01-open-vs-closed-loop.md), [08.10](../08-control/08.10-straight-line-heading-control.md), [09.05](../09-odometry/09.05-calibrating-odometry.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $b$ | wheel separation (track) | m | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $b(s)$ | baseline: expected return used to centre the signal | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $b_1$, $b_2$ | constant biases of those two measurements | rad/s | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $b_g$ | gyro bias as an extra filter state | rad/s | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $b_{gyro}$ | gyro bias: reading of the gyro when the robot is still | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md), [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\hat b_{gyro}$, $\sigma_{\hat b}$ | estimated gyro bias and its standard error | rad/s | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $b_i$ | the $i$-th byte of a message | — | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $\mathbf{b}_i, z_i$ | position of beacon $i$ / measured range to it | m | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $b_{max}$ | largest braking deceleration before the wheels slip | m/s² | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $b_{rot}$, $b_{trans}$ | motion blur from rotation / sideways translation | px | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $b_{\text{true}}$ | actual track width found by calibration | m | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\text{baud}$ | UART bit rate | bit/s | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| $bel(\mathbf{x}_k)$ | belief: probability distribution over the state after the update | 1/state unit | [10.01](../10-localization/10.01-what-is-localization.md) |
| $\overline{bel}(\mathbf{x}_k)$ | predicted belief: after the motion step, before the measurement | 1/state unit | [10.01](../10-localization/10.01-what-is-localization.md) |
| $\binom{k}{x}$ | binomial coefficient: number of ways to choose $x$ of $k$ | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $\binom{n}{2}$ | number of pose pairs from $n$ poses | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |

### C

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $C$ | 2×2 covariance of the object's footprint points | m² | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $C$ | angle between sides $a$ and $b$ of a triangle (elbow angle) | rad | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $C$ | battery capacity | Ah | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [00.06](../00-orientation/00.06-safety-mindset.md), [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md) |
| $C$ | capacitance | F | [01.07](../01-first-robot/01.07-power-system.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md), [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $\mathcal{C}$ | configuration space: all possible $q$ | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $C$ | controller block in a block diagram | depends on loop | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $C$ | cost of a printed part including waste | ₪ | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $C$ | number of colour channels per pixel | — | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $C$ | number of image channels | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $C$ | number of object classes of a detector | — | [16.03](../16-machine-learning/16.03-fine-tuning-a-detector.md) |
| $C$ | offset subtracted from the local mean in adaptive thresholding | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $C$ | set of object classes | — | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $C(s)$ | controller transfer function | — | [08.07](../08-control/08.07-pid-mathematics.md) |
| $C_{Ah}$ | battery capacity in amp-hours | Ah | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $C_{ILS}$ | landed cost of an import in shekels | ₪ | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $C_b$ | total I²C bus capacitance | F | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $\mathcal{C}_{\text{free}}$ | free space: all positions/configurations where the robot touches nothing | — | [12.01](../12-navigation/12.01-the-navigation-problem.md), [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $C_i$ | worst-case execution time (WCET) of task $i$ | s | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $C_{\text{in}}, C_{\text{out}}$ | number of input and output channels of a conv layer | — | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $C_{material}$ | material cost of a print | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $C_{own}$ | total cost of printing $N$ parts on your own printer | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $C_\text{pack}$, $C_\text{cell}$ | capacity of the pack / one cell | Ah | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $C_{pin}$ | input capacitance of a pin | F | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $C_{printer}$ | price of the printer | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $C_\text{rate}$ | C-rating: current as a multiple of capacity | 1/h | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $C_{service}$ | total cost of ordering $N$ parts from a print service | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $c$ | bytes per pixel (3 for `bgr8`, 1 `mono8`, 2 `16UC1`) | B | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $\mathbf{c}$ | centre of mass position | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $c$ | circularity (1 for a perfect disc) | — | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $\mathbf{c}$ | combined centre of mass of base + arm + payload | m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $c$ | cost of one VLM question | $ | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $c$ | costmap cost value (0–254) | — | [12.04](../12-navigation/12.04-costmaps.md) |
| $c$ | free corridor width for the robot's center | m | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $c$ | object's centre of mass | m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $c$ | odometry yaw-rate error per metre driven (unequal-wheel error) | rad/m | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $c$ | offset term of a linear model ($\omega = k\,u + c$, $c = -k d_0$) | rad/s | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $c$ | specific heat of a metal | J/(kg·K) | [01.03](../01-first-robot/01.03-workbench-and-tools.md), [FE.17](../optional-foundations/electronics/FE.17-soldering.md) |
| $c$ | speed of light, 299 792 458 | m/s | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $c$ | speed of sound | m/s | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $c$ | speed of sound (343 at 20 °C) | m/s | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $c$ | speed of the wave (sound or light) | m/s | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $c$ | UCB exploration bonus weight | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $c$ | XOR checksum byte | — | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $c(i)$ | cluster assigned to point $i$ | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $c(s)$ | normalized costmap cost (closeness to obstacles) at path point $s$, in $[0,1]$ | — | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $c$, $p$ | current and previous raw counter values | ticks | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $c, s$ | shorthand for $\cos\theta$ and $\sin\theta$ | — | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $c_-, c_0, c_+$ | matching costs at disparities $d^*-1, d^*, d^*+1$ | — | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $c_0$, $c_1$ | coefficients of a discrete characteristic quadratic | — | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $c_L$, $c_R$ | correction factors applied to the left / right wheel radius | — | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $c_\alpha, s_\alpha$ | shorthand for $\cos\alpha$, $\sin\alpha$ | — | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $c_b, r_b$ | current and reference fraction of data in bin $b$ (PSI) | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\bar{c}_{home}$ | average cost of one print at home | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $c_i$ | processing (compute) time of chain step $i$ | s | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $c_i$ | target pose in the camera frame at pose $i$, $T_{\text{cam,target}}^{(i)}$ (from solvePnP) | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $c_{lin}$ | linear (gamma-removed) color channel | — | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $c_{rr}$ | rolling-resistance coefficient | — | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $\bar{c}_{service}$ | average cost of one part from a service | ₪ | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $c_v, c_e$ | weights of the value loss and entropy bonus | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $c_x, c_y$ | principal point: pixel where the optical axis hits the image | px | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $c_x$, $c_y$ | principal point: pixel where the optical axis hits the image | px | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $c_x, c_y, w, h$ | box center and size | px | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\mathrm{clamp}(\cdot)$ | limit a value to a range, e.g. ±1 duty | — | [08.05](../08-control/08.05-integral-control.md) |
| $\cos(a, b)$ | cosine similarity between two vectors (1 = same direction) | — | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $\text{cost}_n$ | costmap cost of cell $n$ | — | [12.04](../12-navigation/12.04-costmaps.md) |
| $\operatorname{cov}(X, Y)$ | covariance: do $X$ and $Y$ err in the same direction? | unit$_X$·unit$_Y$ | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\operatorname{cov}(a,b)$ | covariance between two variables | product of units | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |

### D

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $D$ | data dimension | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $\mathcal{D}$ | demonstration dataset of (observation, action) pairs | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $D$ | determinant of a 2×2 system ($a_1 b_2 - a_2 b_1$); zero means the equations are not independent | — | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $D$ | distance of a move, $\lvert q_1 - q_0 \rvert$ | rad | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $D$ | FMEA detection score, high means hard to detect (1-10) | — | [21.09](../21-war-machine/21.09-failure-modes-and-safety.md) |
| $D$ | number of disparities a stereo matcher searches | px | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $D$ | padded action vector width | — | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $D$ | PWM duty cycle (fraction of the period the switch is on) | — | [01.08](../01-first-robot/01.08-first-motor-spin.md), [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $D$ | straight-line distance to a sphere's center | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $D$ | total distance of a goal | m | [04.08](../04-ros2/04.08-actions.md) |
| $D$ | total distance rolled in 10 wheel turns (radius calibration) | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $D$, $D_k$ | derivative term of the controller output | — | [08.06](../08-control/08.06-derivative-control.md) |
| $D_\text{KL}$ | KL divergence | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $D_L$, $D_R$ | left / right wheel diameter | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $D_{applied}$ | duty actually sent after deadband compensation | — | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $D_{kick}$ | one-sample spike of D-on-error after a setpoint step | — | [08.06](../08-control/08.06-derivative-control.md) |
| $d$ | allowed stopping distance | m | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $d$ | cost (time) of trusting memory and being wrong | s | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $d$ | deadband: smallest duty at which the motor actually turns | — | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $d$ | disparity: horizontal pixel shift of a point between left and right images | px | [11.08](../11-slam/11.08-visual-slam.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\mathbf{d}$ | displacement from robot to goal ($\mathbf{g} - \mathbf{p}$) | m | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $d$ | distance a force acts over (stroke) | m | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $d$ | distance a wheel (robot) has rolled | m | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $d$ | distance between a parallel axis and the centre of mass | m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $d$ | distance between a signal pair and a motor wire | m | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $d$ | distance driven | m | [P08](../projects/P08-odometry.md) |
| $d$ | distance from camera to marker | m | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $d$ | distance from the grasp line to the object's centre of mass | m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $d$ | distance from ticks for one wheel | m | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $d$ | distance of a cell from the nearest lethal (obstacle) cell | m | [12.04](../12-navigation/12.04-costmaps.md) |
| $d$ | distance of a map cell from the nearest obstacle | m | [P11](../projects/P11-autonomous-navigation.md) |
| $d$ | distance to a target measured by echo time | m | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $d$ | distance to an object | m | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $d$ | distance travelled by a wheel | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $d$ | disturbance acting on the loop (rug, slope, battery sag) | depends on loop | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $d$ | gear pitch diameter, the circle two gears roll on | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $d$ | half the track width (wheel distance from the robot centre) | m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $d$ | half-difference of the wheel speeds, $(\omega_R-\omega_L)/2$ | rad/s | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $d$ | Mahalanobis distance, error measured in standard deviations | — | [P10](../projects/P10-localization.md) |
| $d$ | motor deadband: duty below which the wheel does not turn | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $d$ | motor duty cycle (fraction of full power) | — | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $d$ | noise density from an IMU datasheet | unit/√Hz | [07.06](../07-sensors/07.06-imu-in-ros2.md) |
| $d$ | number of dimensions of the configuration space | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $d$ | octree depth: how many times the cube is split in half | — | [11.01](../11-slam/11.01-map-representations.md) |
| $\mathbf{d}$ | offset vector from the centre of mass to a new reference point | m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $d$ | plane offset (signed distance of the plane from the origin) | m | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $d$ | plane offset in $\mathbf{n}\cdot\mathbf{p} + d = 0$ | m | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $d$ | PWM duty command (0 to 1) in the duty-to-speed fit or ripple formula | — | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $d$ | PWM duty cycle, 0 to 1 | — | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $d$ | queue depth (number of messages buffered) | — | [04.07](../04-ros2/04.07-services.md) |
| $d$ | range to a landmark | m | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $d$ | ray direction through a pixel | — | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $d$ | RC channel value decoded from pulse width (0 to 1) | — | [21.05](../21-war-machine/21.05-triggers-remote.md) |
| $d$ | relay amplitude: how far the bang-bang command swings around $u_0$ | — | [08.08](../08-control/08.08-tuning-pid.md) |
| $d$ | ROS domain ID (`ROS_DOMAIN_ID`) | — | [04.01](../04-ros2/04.01-why-middleware.md), [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $d$ | signed distance (here: angle) of a profiled move | rad (or m) | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $d$ | signed distance from a point to a support-polygon edge (stability margin) | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $d$ | stereo disparity: pixel shift of a point between left and right images | px | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $d$ | suction-cup diameter | m | [15.02](../15-manipulation/15.02-grippers.md) |
| $d$ | token (embedding) dimension | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $d$ | unit closing direction of the jaw | — | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $d$ | wire diameter | mm | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $d$, $d^2$ | Mahalanobis distance (squared): distance from the mean in units of σ | — | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $d^2$ | squared Mahalanobis distance: distance measured in standard deviations, squared | — | [11.03](../11-slam/11.03-the-slam-problem.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $d_0$ | duty-axis intercept of the duty-to-speed fit (duty where speed is zero) | — | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $d_0$ | motor deadband: duty below which the motor does not turn | — | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $d_1$, $d_2$ | descriptor distance to the best / second-best match | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $d_{CAD}$ | hole diameter drawn in CAD | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_L, d_R$ | distance rolled by the left / right wheel in one step | m | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $d_L$, $d_R$ | distance travelled by the left / right wheel | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $d_L, d_R$ | distance travelled by the left / right wheel in a step | m | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $d_L$, $d_R$ | distance travelled by the left / right wheel in one step (from ticks) | m | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $d_M$ | Mahalanobis distance: distance in "σ units" along the uncertainty ellipse | — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $d_{\min}, d_{\max}$ | shortest / longest measurable distance | m | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $d_{\pi}$ | distribution of states a policy visits | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $d_a$ | gear outside (tip) diameter | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $d_{\text{blind}}$ | distance driven before a teleop stop takes effect | m | [01.15](../01-first-robot/01.15-teleop-from-laptop.md) |
| $d_{brake}$ | braking distance | m | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $d_{\text{brake}}$ | distance at which to start stopping | m | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $d_c$ | horizontal distance from the drive axle to the caster | m | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $d_{coast}$ | distance travelled before the watchdog stops the robot | m | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $d_\text{dead}$ | distance travelled during the decision dead time | m | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $d_{est}$ | distance the code computes for a wheel from its ticks | m | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $d_f$ | gear root diameter | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $d_{floor}$ | distance at which a sensor's cone hits the floor | mm | [F3D.05](../optional-foundations/3d-printing-and-cad/F3D.05-mounting-bracket.md) |
| $d_g$ | horizontal distance from the drive axle to the centre of mass | m | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $d_i$ | done flag: 1 if transition $i$ terminated | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $d_i$ | measured distance in calibration run $i$ | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $d_i$ | tape-measured distance of run $i$ | m | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $d_{\text{iteration}}$ | distance the robot travels between two loop decisions | m | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $d_j$ | distance from a beam endpoint to the nearest obstacle in the map | m | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $d_k$ | distance travelled at step $k$ | m | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $d_k$ | key dimension | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $d_{\text{lerobot}}$ | joint angle as LeRobot reports it | ° | [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md) |
| $d_\text{margin}$ | free distance available for stopping | m | [20.04](../20-final-robot/20.04-safety-case.md) |
| $d_\text{margin}$ | real stopping margin to the obstacle | m | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $d_{max}$ | largest disparity searched | px | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $d_{measured}$ | hole diameter measured on the print | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_n$ | distance covered after control step $n$ | m | [04.08](../04-ros2/04.08-actions.md) |
| $d_{\text{react}}$ | distance travelled before a reaction takes effect | m | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $d_\text{right}, d_\text{left}$ | distance travelled by the right and left wheels | m | [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md) |
| $d_{\text{sensed}}$ | distance to an obstacle as reported by the sensor | m | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $d_\text{stop}$ | distance the robot travels before it is stopped | m | [P01](../projects/P01-manual-drive.md), [20.04](../20-final-robot/20.04-safety-case.md) |
| $d_\text{stop}$ | stopping distance | m | [04.08](../04-ros2/04.08-actions.md) |
| $d_{\text{stop}}$ | stopping distance | m | [12.01](../12-navigation/12.01-the-navigation-problem.md), [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $d_{\text{stop}}$ | total stopping distance: latency travel plus braking | m | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $d_t$ | distance to the goal at step $t$ | m | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $d_{target}$ | hole diameter you want after printing | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_{tick}$ | wheel travel per tick | m | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $d_{\text{tool}}$ | largest tool motion allowed between two collision checks | m | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| deadband | duty below which the motor does not turn | — | [08.04](../08-control/08.04-proportional-control.md) |
| $\Delta\text{ticks}$ | ticks counted during a run | — | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $\det A$ | determinant: signed area/volume scale factor; 0 means not invertible | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md), [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $\det R$ | determinant of $R$ (+1 for a proper rotation) | — | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $DG$ | DDS domain gain: port gap between two ROS domain IDs (default 250) | — | [04.01](../04-ros2/04.01-why-middleware.md) |
| $di/dt$ | rate of change of current | A/s | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $\operatorname{diag}(\dots)$ | diagonal matrix with these entries, zeros elsewhere | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $\text{DOF}$ | degrees of freedom of a CAD sketch | — | [F3D.02](../optional-foundations/3d-printing-and-cad/F3D.02-cad-basics.md) |
| DR | dynamic range of a sensor | dB | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $dt$ | control/sample time step | s | [14.06](../14-robotic-arm/14.06-jacobian.md), [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $dt$ | physics or simulation time step | s | [06.01](../06-simulation/06.01-why-simulate.md), [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $dt$ | sampling interval of the control/encoder loop | s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\text{duty}$ | PWM duty cycle sent to the motor (fraction of full power) | — | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $dx$, $dy$ | cell steps along x and y between beam start and end (Bresenham) | cells | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\mathrm{dz}(u)$ | deadband function: the part of the duty that actually drives the motor | — | [08.02](../08-control/08.02-motor-step-response.md) |
| $\mathrm{dz}(u)$ | deadzone function: duty above the deadband, with sign | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |

### E

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\mathbf{E}$ | 3×3 identity matrix (in the parallel-axis tensor formula) | — | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $E$ | back-EMF: voltage a spinning motor generates | V | [02.02](../02-robot-electronics/02.02-power-budget.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $E$ | energy | J or Wh | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $E$ | energy (stored, delivered to a payload, or of a pulse) | J | [21.01](../21-war-machine/21.01-war-machine-architecture.md), [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $E$ | energy stored in a battery | Wh | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $E$ | essential matrix (relates normalized points of two views) | — | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $E$ | kinetic energy returned by braking | J | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $E$ | number of epochs | — | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $E(R, t)$ | sum of squared distances between matched points | m² | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $E(\Delta x, \Delta y)$ | intensity change when shifting a window by $(\Delta x, \Delta y)$ | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $E[X]$ | expected value: probability-weighted average | as $X$ | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $E_L$ | energy stored in an inductor | J | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $E_b$ | energy stored in the battery | J or Wh | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $E_b$ | UMBmark wheelbase (track width) error | — | [P08](../projects/P08-odometry.md) |
| $E_b$ | wheelbase ratio $b_{actual}/b_{nominal}$ | — | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $E_d$ | UMBmark ratio error between left and right wheel radii | — | [P08](../projects/P08-odometry.md) |
| $E_d$ | wheel-diameter ratio $D_R/D_L$ (1 = equal wheels) | — | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $E_\text{extra}$ | extra transitions needed for aborts | — | [19.04](../19-llm-robot-agents/19.04-state-machines.md) |
| $E_k$ | kinetic energy of a moving robot or arm | J | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $E_{max,syst}$ | UMBmark benchmark: the larger of the two cluster offsets | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $E_{usable}$ | battery energy you can actually use | Wh | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $e$ | across-corners width of a hex nut | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $e$ | control error, setpoint minus measurement | depends on loop | [FL.10](../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md) |
| $e$ | control error: setpoint minus measurement, $r - y$ | same as $r$ | [08.01](../08-control/08.01-open-vs-closed-loop.md), [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $e$ | energy per metre driven | J/m | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $\mathbf{e}$ | estimation error, true state minus estimate | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $e$ | heading error (wrapped difference target − current) | rad | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $e$ | heading error to the goal | rad | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md), [17.08](../17-reinforcement-learning/17.08-reward-design.md) |
| $e$ | heading error: how far the robot must turn to face the goal | rad | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\bar e$ | mean error = bias | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\tilde e$ | median error | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $e$ | position error caused by stale data while turning | m | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $e$ | position error of a detection projected with a stale transform | m | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $e$ | residual: prediction minus truth | unit of $y$ | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\mathbf{e}(p)$ | stacked residual vector of all runs' end poses | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $e(q)$ | task-space error still to be removed | m, rad | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $e_1$, $e_2$ | reprojection error of best / second-best pose solution | px | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $e_\theta$ | heading error, wrapped | rad | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\mathbf{e}_a$ | one-hot vector for action $a$ | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $e_i$ | error of trial $i$ (estimate minus ground truth) | as measured | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $e_{ij}$ | error of a constraint between poses $i$ and $j$ | m, rad | [11.03](../11-slam/11.03-the-slam-problem.md), [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\mathbf{e}_j$ | unit vector along state component $j$ | — | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\mathbf{e}_j^{(i)}$ | map point where beam $j$ ends if particle $i$ is right | m | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $e_{max}$ | largest hole-position error that still lets screws go in | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $e_r$, $e_b$ | relative error in wheel radius / wheelbase | — | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $e_{ss}$ | steady-state error left by a P loop | rad/s | [08.04](../08-control/08.04-proportional-control.md) |
| $(e_x, e_y)$ | direction vector of a support-polygon edge | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $\ell$ | distance from the jaw pivot to the pad centre | m | [15.02](../15-manipulation/15.02-grippers.md) |
| $\ell$ | horizontal distance from the axle to the centre of mass | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\ell$ | length of the parallel wire run | m | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $\ell$ | per-sample loss (Huber in DQN) | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $\ell$ | RS-485 / CAN cable length | m | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| $\ell$, $\ell(n, m)$ | length (cost) of the step from cell $n$ to neighbour $m$ | m or cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $\ell_{front}, \ell_{rear}$ | distance from the centre of mass to the front / rear tipping edge | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $\text{end}_k$ | byte just after field $k$ | bytes | [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md) |
| $\text{ENOB}$ | effective number of bits of an ADC | bit | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| extent | sum of all joint ranges (MoveIt's maximum extent) | rad | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |

### F

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $F$ | fastener (screw) diameter | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $F$ | force | N | [06.03](../06-simulation/06.03-gazebo-basics.md), [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $F$ | force (actuator, spring pull, edge force) | N | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $\mathbf{F}$ | force vector | N | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $F$ | fundamental matrix (relates pixel points of two views) | — | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $F$ | Kalman state-transition (motion) matrix | — | [13.12](../13-computer-vision/13.12-tracking.md) |
| $F$ | normal (squeeze) force per pad | N | [15.01](../15-manipulation/15.01-physics-of-grasping.md), [15.02](../15-manipulation/15.02-grippers.md) |
| $F$ | pinch / contact force | N | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $F$ | state transition matrix of a Kalman filter (state now → state next step) | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $F$ | state-transition matrix (EKF: Jacobian $\partial f/\partial\mathbf{x}$) | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $F(s, s')$ | shaping reward added on a transition | — | [17.08](../17-reinforcement-learning/17.08-reward-design.md) |
| $F(\mathbf{x})$ | nonlinear least-squares cost, half the sum of squared residuals | unit² | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $F(x)$ | total pose-graph cost: sum of weighted squared edge errors | — | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $F$, $H$ | state-transition and measurement matrices (or their Jacobians in the EKF) | — | [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $F_\text{mag}$ | pulling force of a solenoid | N | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $F_t$ | tangential force on a gear tooth | N | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $f$ | camera focal length | px | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $f$ | camera frame rate | Hz | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $f$ | control rate | Hz | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $f$ | fill ratio: area over enclosing-circle area | — | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $f$ | focal length | px | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $f$ | focal length in pixels | px | [11.08](../11-slam/11.08-visual-slam.md), [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $f$ | force applied at the tool (e.g. a payload's weight) | N | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $f$ | fraction of runtime spent in the part you optimise | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $f$ | frame rate | Hz | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $f$ | frequency of an oscillation in the loop | Hz | [03.04](../03-robot-software/03.04-timing-and-concurrency.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $f$ | infill fraction | — | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $f$ | interrupt (encoder edge) rate | 1/s | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $f$ | loop rate: how many times per second a control loop runs | Hz | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $f$ | MoveIt `longest_valid_segment_fraction` (collision-check spacing as a fraction of the extent) | — | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| $f$ | rate at which the controller needs results | Hz | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $f$ | rate of a timer or topic | Hz | [04.07](../04-ros2/04.07-services.md), [04.11](../04-ros2/04.11-qos.md) |
| $f$ | real-time factor (RTF): sim seconds per wall second | — | [06.04](../06-simulation/06.04-bridging-gazebo-ros2.md) |
| $f(n)$ | A* priority $g(n) + h(n)$ | m or cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $f(q)$ | forward kinematics reduced to the task coordinates | m, rad | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $f(\mathbf{s}, t)$ | right-hand side of the ODE: rate of change of the state | state/s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $f(\mathbf{x}, \mathbf{u})$ | nonlinear motion model | mixed | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $f(x, u, t)$ | ODE right-hand side: rate of change of the state | state unit/s | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $f$, $f_{alias}$ | true signal frequency and the frequency it appears at after sampling | Hz | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $f, f_x, f_y$ | camera focal length in pixels (horizontal / vertical) | px | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $\hat{\mathbf{f}}, \hat{\mathbf{l}}$ | robot's forward / left unit vectors expressed in map coordinates | — | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $f_{\text{PID}}$ | rate of the wheel speed PID loop | Hz | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $f_{PWM}$ | PWM frequency | Hz | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $f_{\max}$ | fastest safe ping rate | Hz | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $f_\phi$, $g_\theta$ | autoencoder encoder and decoder | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $f_\theta$ | the model: a function with adjustable parameters $\theta$ | — | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $f_{\text{actual}}$ | rate a naive loop really achieves | Hz | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $f_{bw}$ | closed-loop bandwidth | Hz | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $f_c$ | control loop rate | Hz | [04.08](../04-ros2/04.08-actions.md) |
| $f_c$ | cutoff frequency of an RC low-pass filter | Hz | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $f_\text{control}$ | control rate | Hz | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $f_{edges}$ | encoder edge rate the counter must keep up with | Hz | [00.04](../00-orientation/00.04-robot-computers.md), [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $f_i$ | publish rate of topic $i$ | Hz | [04.14](../04-ros2/04.14-rosbag2.md) |
| $f_i$ | publish rate of topic (or chain step) $i$ | Hz | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $f_i, w_i$ | rate and run time of callback $i$ | Hz, s | [04.12](../04-ros2/04.12-executors-and-callbacks.md) |
| $f_n$ | normal force at a contact (push, never pull) | N | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $f_{px}$ | focal length in pixels | px | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $f_s$ | sample rate | Hz | [07.06](../07-sensors/07.06-imu-in-ros2.md), [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $f_s$ | sample rate, $1/T$ | Hz | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $f_s$ | sampling rate | Hz | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $f_t$ | tangential (sideways) force at a contact | N | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $f_t$ | telemetry rate | Hz | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $f_{usable}$ | usable fraction of battery energy | — | [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $f_{wheels}$ | fraction of the robot's weight carried by the drive wheels | — | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $f_x$ | camera focal length in pixels | px | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $f_x$, $f_y$ | focal length in pixels along x / y | px | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $f_x, f_y$ | focal lengths in pixels | px | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $f_y$ | camera focal length in pixels | px | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $ff$ | feedforward term of the wheel-speed controller | — (duty) | [P04](../projects/P04-pid-movement.md) |
| $\mathrm{FK}(q)$ | forward kinematics: gripper position for joints $q$ | m | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $FN_t, FP_t, IDSW_t, GT_t$ | misses, false positives, ID switches, ground-truth objects in frame $t$ | — | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\text{FOV}$ | horizontal field of view of the camera | rad | [21.06](../21-war-machine/21.06-triggers-vision.md) |

### G

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $G$ | gearbox ratio (motor turns per wheel turn) | — | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $G$ | noise-input matrix: how noise sources enter the state (EKF: $\partial f/\partial\mathbf{u}$) | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.06](../10-localization/10.06-extended-kalman-filter.md), [09.06](../09-odometry/09.06-accumulated-error.md) |
| $G$ | plant block (motor driver + motor + gearbox + wheel), or its transfer function $G(s)$ | depends on loop | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [08.07](../08-control/08.07-pid-mathematics.md) |
| $G(x, y)$ | filtered image (correlation or convolution output) | — | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $G, Q$ | Jacobian with respect to motion inputs / covariance of motion noise | —, units² | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $G_t$ | discounted return from time $t$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $G_t$ | return-to-go from time $t$ | reward units | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $G_x$, $G_y$ | Sobel gradients along x and y | — | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $g$ | factor by which the motor gain drops when the battery sags (e.g. $9.9/12.6$) | — | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $g$ | gravitational acceleration (9.81) | m/s² | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md), [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $g$ | gravitational acceleration, 9.81 | m/s² | [06.03](../06-simulation/06.03-gazebo-basics.md), [FP.02](../optional-foundations/physics/FP.02-forces-newton.md), [15.01](../15-manipulation/15.01-physics-of-grasping.md), [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $\mathbf{g}$ | gravity vector | m/s² | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $\mathbf{g}$ | position of the goal | m | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\hat g$ | sampled policy-gradient estimate | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $g$ | slope (gain) of the printer's hole-size fit | — | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $g$ | the current gradient | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $g(n)$ | cost of the best known path from start to cell $n$ | m or cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $g(x)$ | CRC-16 generator polynomial | — | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $g$, $o$ | two-point calibration gain / offset | —, V | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $g_a, g_n$ | analytic and numerical gradients (for gradient checking) | — | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $g_{front}$, $g_{rear}$ | gap between straightedges at the front / rear of the wheels | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $g_i$ | gripper pose in the base frame at calibration pose $i$, $T_{\text{base,gripper}}^{(i)}$ | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $(g_x, g_y)$ | goal position | m | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $g_z$ | gyro reading about z | rad/s | [07.05](../07-sensors/07.05-imu-fundamentals.md) |

### H

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $H$ | action chunk length | — | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $H$ | approximate Hessian of the pose graph (system matrix in Gauss-Newton) | varies | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $H$ | clearance hole diameter | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $H$ | cross-covariance matrix of centered point pairs (Kabsch) | m² | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $H$ | Hessian: curvature matrix of the loss | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $H$ | Hessian: matrix of second derivatives (curvature) of a cost | varies | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $H$ | hidden layer width (number of neurons) | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $H$ | homography: 3×3 matrix mapping one image plane to another | — | [13.04](../13-computer-vision/13.04-features-and-matching.md), [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $H$ | measurement matrix (which state parts the detector sees) | — | [13.12](../13-computer-vision/13.12-tracking.md) |
| $H$ | measurement matrix: which state combination a sensor sees (EKF: $\partial h/\partial\mathbf{x}$) | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $H$ | real height of an object | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md), [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $H$ | sensor block in a block diagram (encoder + speed estimator) | — | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $H, H_\beta$ | measurement Jacobian (EKF); its bearing row | 1/m, — | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $H$, $S$, $V$ | hue, saturation, value (HSV) | °, 0–255 | [13.01](../13-computer-vision/13.01-images-as-data.md), [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $H[\pi]$, $\mathcal{H}$ | policy entropy: how random the policy is | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $H_0, H_1$ | hypotheses "not good enough" and "good enough" | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $H_{deg}$ | hue in degrees (OpenCV stores $H_{deg}/2$) | ° | [13.01](../13-computer-vision/13.01-images-as-data.md) |
| $H_{floor}$ | homography from floor coordinates to pixels | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $h$ | actions executed from each chunk before re-planning | — | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md), [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $h$ | camera height above the floor | m | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $h$ | cylinder height | m | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $h$ | drop height | m | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $h$ | heading as a unit vector $(\cos\theta, \sin\theta)$ | — | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $h$ | height of the centre of mass | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md), [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $h$ | height of the centre of mass above the floor | m | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md), [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $h$ | height of the LiDAR above the floor | m | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $h$ | height of the sensor's optical centre above the floor | mm | [F3D.05](../optional-foundations/3d-printing-and-cad/F3D.05-mounting-bracket.md) |
| $h$ | hidden-layer activations | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $h$ | hypotenuse of a right triangle | m | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $h$ | hysteresis margin: how far above a threshold a value must rise before the alarm clears | V | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md), [04.09](../04-ros2/04.09-parameters.md) |
| $h$ | number of attention heads | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $h$ | object height in the image | px | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $\hat{\mathbf{h}}$ | robot heading unit vector | — | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $h$ | step size of a numerical derivative or integrator | s (or input unit) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $h$ | study hours per week | h | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |
| $\mathbf{h}$ | the 9 entries of $H$ stacked into a vector | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $h(n)$ | heuristic: optimistic guess of remaining cost from $n$ to goal | m or cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h(\mathbf{x})$ | nonlinear measurement model (e.g. range–bearing to a landmark) | mixed | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $h$, $C^*$ | LCh hue angle and chroma | °, — | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $h^*(n)$ | true cost of the best path from $n$ to the goal | m or cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h_c$, $h_o$ | height of the camera / of the object point above the floor | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $h_{\text{oct}}$ | octile distance heuristic for 8-connected grids | cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h_p$ | height at which the robot pushes | m | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $h_{px}$ | box height in pixels | px | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| HFOV | horizontal field of view | rad | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $\text{HFOV}$ | horizontal field of view of the camera | rad | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $\text{hfov}, \text{HFOV}$ | horizontal field of view of a camera | rad | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $hp(i)$ | set of tasks with higher priority than task $i$ | — | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |

### I

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\mathbf{I}$ | 3×3 inertia tensor | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I$ | average current drawn from the battery | A | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $I$ | current | A | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $I$ | electric current | A | [07.05](../07-sensors/07.05-imu-fundamentals.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md), [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $I$ | identity matrix | — | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $I$ | identity matrix (the "do nothing" transform) | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $I$ | moment of inertia (rotational mass) | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I$ | moment of inertia of the throw arm plus payload | kg·m² | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $I(y, x)$, $I'(y, x)$ | image intensity before / after filtering | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $I^2t$ | energy-like measure of a current pulse that a fuse responds to | A²s | [01.07](../01-first-robot/01.07-power-system.md) |
| $I_0$ | no-load current (spent on friction) | A | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $I_0, I_s$ | no-load current / stall current of a motor | A | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $I_\text{CC}$ | charger constant-current phase current | A | [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md) |
| $I_{LED}$ | LED current | A | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $I_{OL}$ | current a device can sink while staying below $V_{OL}$ | A | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $I_{TRIP}$ | driver current-limit trip point | A | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $I_{\max}$ | worst-case current the fuse must pass | A | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $I_\text{ampacity}$ | current a wire can carry continuously without overheating | A | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $I_{arm}$ | moment of inertia of the bare arm | kg·m² | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $I_{battery}$ | current drawn from the battery | A | [01.01](../01-first-robot/01.01-plan-the-build.md) |
| $I_{branch}$, $R_{branch}$ | current through / resistance of one wiring branch | A, Ω | [01.07](../01-first-robot/01.07-power-system.md) |
| $I_{candidate}$ | integral value the anti-windup check would accept this period | — | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $I_{cm}$ | moment of inertia about an axis through the centre of mass | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_\text{continuous}$ | steady current a branch draws in normal running | A | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $I_{fuse}$ | fuse rating | A | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $I_\text{fuse}$ | fuse rating | A | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $I_{in}$ | regulator input current | A | [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $I_k$ | integral term, stored already multiplied by $k_i$ (in duty units) | — | [08.05](../08-control/08.05-integral-control.md) |
| $I_{limit}$ | motor driver current limit per motor | A | [01.07](../01-first-robot/01.07-power-system.md), [01.08](../01-first-robot/01.08-first-motor-spin.md), [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md) |
| $I_\text{load}$ | load current through a regulator | A | [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $I_{max}$ | wire or connector current rating | A | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $I_{out}$ | regulator output current | A | [01.01](../01-first-robot/01.01-plan-the-build.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $I_{peak}$ | peak inrush current at switch-on | A | [01.07](../01-first-robot/01.07-power-system.md) |
| $I_{rest}$ | current at rest | A | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $I_{reverse}$ | current when reversing a motor at full speed | A | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $I_\text{stall}$ | stall current | A | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $I_{worst}$ | worst-case total battery current | A | [01.07](../01-first-robot/01.07-power-system.md), [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $I_x$, $I_y$ | image gradients along x and y | 1/px | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $I_{xx}, I_{yy}, I_{zz}$ | moment of inertia about the x, y, z axis (resistance to spinning) | kg·m² | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_{xy}, I_{xz}, I_{yz}$ | products of inertia (off-diagonal terms) | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_{yaw,eff}$ | effective yaw inertia including the spinning rotors | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $i$ | beam index in a LaserScan | — | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $i$ | DDS participant index on one machine | — | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $i$ | gear ratio | — | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $i$ | motor current | A | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\mathbf{i}, \mathbf{j}, \mathbf{k}$ | quaternion imaginary units | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\text{IAE}$, $\text{ITAE}$, $\text{ISE}$ | error-integral scores used to compare tunings | rad, rad·s, rad²/s | [08.08](../08-control/08.08-tuning-pid.md) |
| $\text{ICC}$ | instantaneous centre of curvature: the point the robot rotates about | m (a point) | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\text{IFOV}$ | angle seen by one pixel | rad/px | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $\int_a^b f\,dt$ | integral: accumulated area under $f$ from $a$ to $b$ | unit$_f$·s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| IoU | intersection over union of two boxes or masks | — | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |

### J

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $J$ | differential-drive kinematics matrix: wheel speeds → $(v, \omega)$ | m, m/m | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $J$ | DWA trajectory score (lower is better) | — | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $J$ | impulse (change in momentum) of a launch | kg·m/s | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $J$ | Jacobian of back-projection w.r.t. $(u, v, Z)$ | varies | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $J$ | Jacobian used to propagate covariance through a function | varies | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $J$ | Jacobian: how fast the tool moves per unit joint speed, $\partial f/\partial q$ | m/rad (rows of linear motion), — (angular rows) | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md), [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $J$ | Jacobian: matrix of partial derivatives $\partial e_k/\partial p_j$ | depends | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $J$ | k-means inertia: total squared distance to assigned centres | unit² | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $J$ | LQR / MPC cost to minimise | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $J$ | rotational inertia (at the gearbox output) | kg·m² | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $J$ | rotational inertia of a motor or wheel assembly | kg·m² | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md), [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\dot J$ | time derivative of the Jacobian | m/(rad·s) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $\lVert J \rVert_{\max}$ | largest column norm of the linear Jacobian over the workspace | m/rad | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| $J(\pi)$ | expected cost (failed steps) of a policy | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $J(\theta)$ | expected return of the policy (objective to maximize) | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $J^{+}$ | pseudo-inverse of the Jacobian | rad/m | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $J^{-1}$ | inverse kinematics matrix: $(v, \omega)$ → wheel speeds | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $J_{\text{global}}$, $J_{\text{local}}$ | cost minimized by the global planner / local planner | — | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $J_i$, $J_{:,i}$ | column $i$ of the Jacobian: the tool motion produced by joint $i$ alone | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $J_{ij}$ | Jacobian entry $\partial f_i/\partial x_j$ | varies | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $J_r$ | rotor inertia of a motor | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $J_{reflected}$ | rotor inertia as felt at the gearbox output | kg·m² | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $J_{xj}$ | entry of $J$: tool speed along base $x$ per rad/s of joint $j$ | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $j$ | imaginary unit, $\sqrt{-1}$ | — | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $j$ | jump: input pixels between neighbouring outputs | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |

### K

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $K$ | a convolution kernel (small weight matrix) | — | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $K$ | camera intrinsic matrix | px | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $K$ | camera intrinsic matrix ($f_x, f_y, c_x, c_y$) | px | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $K$ | camera intrinsics matrix (3D point → pixel) | px | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $K$ | cross-product matrix of the axis $\mathbf{k}$ ($K\mathbf{v} = \mathbf{k}\times\mathbf{v}$) | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $K$ | Kalman gain | — | [13.12](../13-computer-vision/13.12-tracking.md), [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $K$ | Kalman gain: how far the estimate moves toward the measurement | — | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $K$ | morphology structuring element | — | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $K$ | motor (plant) gain: steady wheel speed per unit of duty above the deadband | rad/s per duty | [08.02](../08-control/08.02-motor-step-response.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $K$ | motor steady-state gain, wheel speed per unit duty | rad/s | [P04](../projects/P04-pid-movement.md) |
| $K$ | number of beams used per scan | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $K$ | number of classes | — | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md), [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $K$ | number of diffusion noise levels | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $K$ | number of Euler sampling steps | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $K$ | number of MPPI samples | — | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $K$ | proportional gain of the tick-sync loop: duty added per tick of difference | 1/tick | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $K$ | state-feedback gain matrix ($u = -Kx$) | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $K$ | steady-state gain of a motor: fraction of the commanded speed actually reached | — | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $K$ | steady-state gain of the motor model $K/(\tau s + 1)$ | rad/s per duty | [08.07](../08-control/08.07-pid-mathematics.md) |
| $K(i, j)$ | filter kernel weight | — | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $K_\infty$, $P_\infty$ | steady-state Kalman gain and posterior variance | —, m² | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $K_i$ | integral gain (foundations notation) | duty per rad | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $K_{nom}$ | nominal (assumed) plant gain used by a feedforward | rad/s per duty | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $K_p$ | proportional gain (foundations notation; duty per rad/s for speed, duty per rad for angle loops) | duty per (rad/s) or duty/rad | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $K_p$ | proportional gain (same as $k_p$) | depends on loop | [FL.10](../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md) |
| $K_p$ | proportional gain of a controller (command = $K_p$ × error) | 1/s for heading | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $K_{p,crit}$ | critical P gain: above it the loop is unstable | duty per (rad/s) or duty/rad | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md), [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $K_u$ | ultimate gain: P gain at which the loop oscillates with constant amplitude | duty per (rad/s) | [08.08](../08-control/08.08-tuning-pid.md) |
| $K_v$ | BLDC speed rating: no-load RPM per volt | RPM/V | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $k$ | `cost_scaling_factor`: how fast inflation cost decays with distance | 1/m | [12.04](../12-navigation/12.04-costmaps.md) |
| $k$ | action chunk length: actions predicted per call | — | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md), [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $k$ | chi-squared threshold on $d_M^2$ that sets the ellipse probability (5.991 = 95% in 2D) | — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $k$ | convolution kernel size (side length) | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $k$ | depth-camera noise constant in $\sigma_Z \approx kZ^2$ | 1/m | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $k$ | depth-noise coefficient in $\sigma_Z = kZ^2$ | 1/m | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $k$ | diffusion noise level index | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $k$ | discharge rate of the simulated battery | V/s | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $k$ | divider ratio $R_{bottom}/(R_{top}+R_{bottom})$ | — | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $k$ | exchange rate | ₪ per USD | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $k$ | grid cells per axis | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $k$ | growth coefficient of odometry error with distance | m/√m or — | [P08](../projects/P08-odometry.md) |
| $k$ | Harris constant (≈ 0.04) | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $k$ | how many command periods the watchdog tolerates | — | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $k$ | kernel half-size: a $(2k+1)\times(2k+1)$ kernel | px | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $k$ | learned P-controller gain of the docking policy | 1/s | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $k$ | loop index in a sweep ($k = 0..19$) | — | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $k$ | Mahalanobis threshold $d^2 = k$ that defines an uncertainty ellipse | — | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $k$ | motor constant (back-EMF V·s/rad = torque N·m/A) | V·s/rad | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $k$ | motor gain: steady-state speed per unit duty above the deadband | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $k$ | motor gain: wheel speed gained per unit of duty | rad/s per unit duty | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $k$ | Nav2 `cost_scaling_factor`, how fast cost decays | 1/m | [P11](../projects/P11-autonomous-navigation.md) |
| $k$ | number of arms (actions) of a bandit | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $k$ | number of fragments a large message is split into | — | [04.11](../04-ros2/04.11-qos.md) |
| $k$ | number of global abort conditions | — | [19.04](../19-llm-robot-agents/19.04-state-machines.md) |
| $k$ | number of independent trials/readings | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $k$ | number of lidar beams needed on an object to detect it | — | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $k$ | number of missed reporting periods before a component is declared STALE | — | [20.03](../20-final-robot/20.03-software-integration-bringup.md) |
| $k$ | number of samples (filter steps) | — | [04.09](../04-ros2/04.09-parameters.md) |
| $k$ | number of successes in $n$ trials | — | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $k$ | proportional gain of a simple slow-down-near-the-wall controller | 1/s | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $k$ | radius of gyration $\sqrt{I/m}$: a sanity check on inertia | m | [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $k$ | random wheel-slip constant: variance of a wheel's travel per metre rolled | m | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $k$ | scalar multiplier of a vector; repulsion strength in potential-field steering | varies | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $k$ | shrinkage fraction of a printed part | — | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $k$ | slope of the duty-to-speed fit | m/s per unit duty | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $k$ | spring stiffness | N/m | [06.01](../06-simulation/06.01-why-simulate.md) |
| $k$ | spring stiffness of the slingshot bands | N/m | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $k$ | ticks counted in one sampling interval | ticks | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\mathbf{k}$ | unit rotation axis (axis-angle) | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $k$ | usable fraction of the battery (you stop before empty) | — | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $k$ | wheel-noise constant: travel σ after 1 m is $k$ (variance $k^2\lvert d\rvert$) | m$^{1/2}$ | [10.06](../10-localization/10.06-extended-kalman-filter.md), [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $k$ (subscript) | sample / time-step index, e.g. $\omega_k$, $\mathbf{x}_k$ | — | [08.02](../08-control/08.02-motor-step-response.md), [10.01](../10-localization/10.01-what-is-localization.md) |
| $k$, $p$, $s$ | convolution kernel size, padding, stride | px | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $k_1$ | first radial lens-distortion coefficient | — | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $\mathbf{k}_1 \dots \mathbf{k}_4$ | slope evaluations inside a Runge-Kutta step | state/s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $k_1, b$ | BM25 term-saturation and length-normalization parameters | — | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $k_1$, $k_2$, $k_3$ | radial lens distortion coefficients | — | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $k_d$ | derivative gain | duty per (rad/s²) | [08.06](../08-control/08.06-derivative-control.md) |
| $k_d$ | derivative gain of a PID controller | depends on loop | [P04](../projects/P04-pid-movement.md) |
| $k_e$ | back-EMF constant | V·s/rad | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $k_e$ | motor back-EMF constant (volts per rad/s) | V·s/rad | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $k_{ff,a}$ | acceleration-feedforward gain | duty per (rad/s²) | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $k_i$ | integral gain | duty per rad (duty per (rad/s·s)) | [08.05](../08-control/08.05-integral-control.md) |
| $k_i$ | integral gain of a PID controller | depends on loop | [P04](../projects/P04-pid-movement.md) |
| $k_p$ | proportional gain | duty per (rad/s) | [08.04](../08-control/08.04-proportional-control.md) |
| $k_p$ | proportional gain of a PID controller | depends on loop | [P04](../projects/P04-pid-movement.md) |
| $k_{p,h}$ | heading-loop proportional gain: yaw rate commanded per radian of error | 1/s | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $k_t$ | motor torque constant (torque per amp) | N·m/A | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $k_t$ | torque constant | N·m/A | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\mathrm{KL}$ | Kullback–Leibler divergence: how different two distributions are | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |

### L

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $L$ | audio frame level | dBFS | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| $L$ | cable length (one way) | m | [01.04](../01-first-robot/01.04-raspberry-pi-setup.md), [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $L$ | characteristic length dividing torque so it is comparable to force (50 mm) | m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $L$ | Cholesky factor of $\Sigma$ (lower-triangular, $\Sigma = LL^\top$) | units | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $L$ | chord length of that arc | m | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $\mathcal L$ | contrastive training loss | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $L$ | dead time (pure delay) before the motor starts responding | s | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $L$ | detector processing time per frame | s | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $L$ | inductance (motor winding or wire) | H | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $L$ | inference latency of a model | s | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $L$ | lag in Ziegler–Nichols open-loop rules (= dead time $\theta$) | s | [08.08](../08-control/08.08-tuning-pid.md) |
| $L$ | local price of an item | ₪ | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $L$ | lookahead distance to the carrot | m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md), [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $L$ | loop gain: product of all gains around a feedback loop, e.g. $k_p K$ | — | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [08.04](../08-control/08.04-proportional-control.md) |
| $L$ | loss: how wrong a prediction is (lower is better) | varies | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $L$ | motor winding inductance | H | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $L$ | nominal length between features | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $L$ | one-way wire length | m | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $L$ | payload length | bytes | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $L$ | rod length | m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $L$ | sensing latency: how old a reading is when you use it; also total reaction latency $L = \sum_i L_i$ | s | [00.01](../00-orientation/00.01-what-is-a-robot.md), [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $L$ | side length of a fiducial marker | m | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $L$ | side length of the UMBmark square | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $L$ | total latency of a remote policy call | s | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $L$ | track width: distance between the two drive wheels | m | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [06.01](../06-simulation/06.01-why-simulate.md) |
| $L$ | wheel separation (distance between left and right wheel centers) | m | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $L$ | wheel separation (track width) | m | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md), [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $L$ | wheel separation (track width), notation of 09.01–09.04 | m | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $L$ | wheel separation (track), in 09.02's notation | m | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $L(s)$, $L(j\omega)$ | open-loop transfer function (controller × plant × sensor) as a function of frequency | — | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $L$, $L_p$ | interaction matrix (image Jacobian): how fast features move per camera twist; $L_p$ for one point | 1/m, — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L$, $W$ | length / width of a rectangular object in the image | px | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $L^*$, $a^*$, $b^*$ | CIELAB lightness, green–red, blue–yellow | — | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $L^{+}$ | damped pseudo-inverse of $L$ | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L_1, L_2, L_3$ | SO-101 axis-to-axis link lengths (shoulder_lift→elbow, elbow→wrist, wrist_flex→tool) | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $L_\text{event}$ | loudness of the sound event | dB SPL | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $L_\text{floor}$ | loudness of the room's background noise | dB SPL | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $L_i$ | heat-set insert length | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $L_i$ | latency of one stage in the sense-to-act path (sensor, transport, processing, loop) | s | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $L_{moment}$ | interaction matrix of the moment features | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L_{screw}$ | screw length | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $l$ | log-odds of a probability, $\ln\frac{p}{1-p}$ | — | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $l$ | log-odds that a grid cell is occupied | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $\mathbf{l} = (a, b, c)$ | line $au + bv + c = 0$ as a vector | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $l$, $l_i$, $l_{t,i}$ | log-odds of a cell being occupied (0 = unknown) | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $l_0$ | prior log-odds of every cell before any measurement | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $l_1, l_2$ | link lengths of a 2-link arm | m | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $l_1, l_2$ | link lengths of the planar 2-link arm (upper arm, forearm) | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\mathbf{l}_2$ | epipolar line in image 2 | — | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $l_{free}$ | log-odds added to each cell a beam passes through | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $l_{occ}$ | log-odds added to the cell where a beam ends | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $l_x$, $l_y$ | known map position of a landmark | m | [10.01](../10-localization/10.01-what-is-localization.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $l_x, l_y$ | landmark position in the map | m | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\mathrm{LN}$, $\mathrm{MHA}$ | layer normalization; multi-head attention | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $\log R$ | axis-angle 3-vector of a rotation (direction = axis, length = angle) | rad | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |

### M

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $M$ | a general linear transformation matrix | — | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $M$ | correlation matrix $\sum_i \beta_i \alpha_i^T$ used to solve for $R_X$ | rad² | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $M$ | mass of the whole robot (recoil) | kg | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $M$ | modulus of an $N$-bit counter, $2^N$ | ticks | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $M$ | mutual inductance between a motor wire and a signal pair | H | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $M$ | number of MPPI samples | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $M$ | number of particles in a particle filter | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $M$ | observation size sent per call | bytes | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $M$ | steady-state predicted variance of a 1D Kalman filter | m² | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $M$ | structure tensor (sum of gradient products in a window) | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $M$ | tool pose at the zero configuration, $T_{\text{base,tool}}(0)$ | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $M$ | wheel-noise covariance | m² | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $M$ | wheel-travel noise covariance (input noise) | m² | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| ${}^{M}\hat T_{B}$ | localization's estimate of the robot pose in the map | — | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| $M(\hat{\mathbf{a}}, q)$ | motion a joint adds at position $q$ | — | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $M, O, F, B, C, K, L, T$ | frame names: map, odom, base_footprint, base_link, camera_link, camera optical, laser, tool | — | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md), [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $M$, $m$, $C$ | max, min, and range (chroma) of $R,G,B$ | 0–255 | [13.01](../13-computer-vision/13.01-images-as-data.md) |
| $M_1$, $M_2$ | two segmentation masks (pixel sets) | — | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $M_{2D}$, $M_{3D}$ | memory needed by a dense 2D / 3D grid map | B | [11.01](../11-slam/11.01-map-representations.md) |
| $M_i$ | modified z-score of trial $i$ ($\lvert M_i\rvert > 3.5$ = potential outlier) | — | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $M_\text{infer}$, $M_\text{train}$ | GPU memory floor for inference and for full training | bytes | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $M_p$ | maximum overshoot as a fraction of the step | — | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $M_{pq}$ | image moment of order $(p, q)$ | varies | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $m$ | effective moving mass | kg | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $m$ | gear module, pitch diameter per tooth | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $m$ | mass | kg | [00.06](../00-orientation/00.06-safety-mindset.md), [01.03](../01-first-robot/01.03-workbench-and-tools.md), [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $m$ | mass of the object | kg | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $m$ | mass of the robot (tipping) | kg | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $m$ | mean of the two wheel speeds, $(\omega_L+\omega_R)/2$ | rad/s | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $m$ | moment feature vector | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $m$ | payload (projectile) mass | kg | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $m$ | printed part mass | g | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md), [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $m$ | safety margin for sensor noise | m | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $m$ | signed motor command in the serial protocol (−1000 to 1000) | — | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $m$ | temporal-ensembling coefficient ($> 0$ favours older predictions) | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $m$ | the whole map | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md), [11.03](../11-slam/11.03-the-slam-problem.md) |
| $m \times n$ | shape of a matrix: $m$ rows, $n$ columns | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $\hat m, \hat v$ | Adam's start-up-bias-corrected averages | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $m$, $v$ | Adam's running averages of the gradient and of the squared gradient | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $m_{\max}$ | heaviest object a given grip can hold | kg | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $m_b, \mathbf{c}_b$ | mass and centre of mass of the mobile base | kg, m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $m_d$ | mask: 1 if action dimension $d$ is real, 0 if padding | — | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $m_i$ | grid cell $i$ of the map (occupied or free) | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $m_i$ | mass of part $i$ in the parts table | kg | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $m_i, \mathbf{c}_i(q)$ | mass and centre of mass of arm link $i$ (moves with $q$) | kg, m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $m_i, \mathbf{p}_i$ | mass and position of part $i$ | kg, m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $m_k, r_k$ | mass of piece $k$ and its distance from the axis | kg, m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $m_p$ | payload mass | kg | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $m_{proj}$ | projectile mass | kg | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $m_{robot}$ | robot mass | kg | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $m_x, m_y, m_z$ | magnetometer readings along x, y, z | µT | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\text{MAD}$ | median absolute deviation (outlier-resistant spread) | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\mathrm{MAD}$ | median absolute deviation: outlier-resistant spread | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\text{MAE}$ | mean absolute error | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| margin | distance from the centre of mass to the nearest support-polygon edge (negative = tips) | m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $\text{mid}$ | middle of the recorded calibration range of a joint | counts | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |

### N

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $N$ | average number of photo-electrons a pixel collects | e⁻ | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $N$ | encoder counts (ticks) per wheel revolution | — | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $N$ | encoder counts per wheel revolution (CPR, after gearbox and quadrature) | — | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $N$ | encoder ticks per full wheel revolution (fixed property of the robot) | ticks | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $N$ | encoder ticks per revolution in the metres-per-tick factor | ticks/rev | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $N$ | encoder ticks per wheel revolution (CPR × gearing × 4) | ticks/rev | [P03](../projects/P03-encoder-movement.md), [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $N$ | gear ratio (motor turns per output turn) | — | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $N$ | model parameter count | — | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $N$ | MPC prediction horizon (number of steps) | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $N$ | normal force (high-school friction law) | N | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $N$ | normal force pressing two surfaces together | N | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $N$ | number of audio samples in an RMS window | — | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $N$ | number of bits of a hardware counter | — | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $N$ | number of control steps to finish a goal | — | [04.08](../04-ros2/04.08-actions.md) |
| $N$ | number of matched point pairs | — | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $N$ | number of particles | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $N$ | number of points | — | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $N$ | number of prints | — | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $N$ | number of RANSAC iterations | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $N$ | number of readings averaged | — | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $N$ | number of returned photons | — | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $N$ | number of samples / readings | — | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $N$ | number of samples averaged | — | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $N$ | number of samples in a moving-average window | — | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $N$ | number of stationary gyro samples averaged | — | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $N$ | number of training examples (or samples in a batch) | — | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $N$ | QoS history depth (queue length) | — | [04.11](../04-ros2/04.11-qos.md) |
| $N$ | raw ADC reading (0–65535 from `read_u16`) | — | [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md), [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $\mathcal{N}(0, \sigma^2)$ | normal (Gaussian) distribution with mean 0 and variance $\sigma^2$ | — | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $\mathcal N(\mu, \sigma^2)$ | Gaussian (normal) distribution with mean $\mu$ and variance $\sigma^2$ | — | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\mathcal{N}(\mu, \sigma^2)$ | Gaussian (normal) distribution with mean $\mu$, variance $\sigma^2$ | — | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\mathcal{N}(\mu, \sigma^2)$ | normal (Gaussian) distribution with mean and variance | — | [10.02](../10-localization/10.02-uncertainty-gaussians.md), [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $N(a)$ | number of times action $a$ was tried | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $N$, $M$ | number of scan points / map points in nearest-neighbour matching | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $N^*$ | break-even number of prints where owning a printer pays off | — | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $N_c'$ | caster load measured while tilted | N | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $N_{cpr}$ | encoder pulses per motor revolution on one channel (PPR) | — | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $N_\text{eff}$ | effective number of independent predictions averaged | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $N_{eff}$ | effective number of particles (how many carry real weight) | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $N_\text{frames}$ | number of frames in a dataset | — | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $N_\text{in}$ | total input tokens over a task | tokens | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md) |
| $N_{\text{rays}}$ | raycasts per filter update | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $N_{rev}$ | encoder ticks per wheel revolution (2464 on karmel) | ticks/rev | [08.03](../08-control/08.03-wheel-speed-estimation.md), [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $N_{ticks}$ | encoder ticks per wheel revolution | — | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $N_{ticks}$ | encoder ticks per wheel revolution (same as $N_{rev}$) | ticks/rev | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $N_\text{ticks}$ | number of behaviour-tree ticks in a task | — | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $N_{ticks}$ | ticks per wheel revolution after ×4 quadrature decoding | — | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $N_w, N_c$ | normal (support) force on the drive wheels / on the caster | N | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $n$ | AWG gauge number | — | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $n$ | encoder ticks counted so far (live counter value) | ticks | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $n$ | frame length before COBS encoding | bytes | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $n$ | input dimension | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $n$ | new 16-bit encoder count | ticks | [FPY.01](../optional-foundations/python/FPY.01-python-for-csharp-devs.md) |
| $n$ | number of components on a fault path | — | [02.09](../02-robot-electronics/02.09-electrical-debugging-method.md) |
| $n$ | number of contacts (pads, fingers) | — | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $n$ | number of encoder ticks | ticks | [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $n$ | number of frames | — | [16.02](../16-machine-learning/16.02-robot-data.md) |
| $n$ | number of grid cells | — | [10.03](../10-localization/10.03-bayes-filter.md) |
| $n$ | number of Li-ion cells in series | — | [04.09](../04-ros2/04.09-parameters.md) |
| $n$ | number of messages processed so far | — | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| $n$ | number of periodic tasks | — | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $n$ | number of runs per direction | — | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $n$ | number of samples | — | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md), [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $n$ | number of samples / tree nodes | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $n$ | number of states of a state machine | — | [19.04](../19-llm-robot-agents/19.04-state-machines.md) |
| $n$ | number of straight segments approximating a circle | — | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $n$ | number of tokens in the sequence | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $n$ | number of translation units that include the header | — | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $n$ | number of trials | — | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md), [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md), [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md), [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $n$ | side of the ToF region of interest in SPADs (n×n of 16×16) | — | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\mathbf{n}$ | surface normal (vector perpendicular to a surface) | — | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{n}$ | unit normal of a plane | — | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $n$ | unit normal of a plane | — | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\hat n$ | unit vector along a rotation axis | — | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $n$ | wheel (output shaft) speed in revolutions per minute | RPM | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $n$, $m$ | number of state variables, number of measurement components | — | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $n$, $n_i$ | unit surface normal of a wall at a target point | — | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $n_L$, $n_R$ | cumulative encoder tick counts of left / right wheel | ticks | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $n_b$ | number of predictions in confidence bin $b$ | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $n_{bits}$ | worst-case bits in one classic CAN frame | — | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| $n_{\text{cells}}$ | number of cells in series | — | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $n_{\text{eff}}$ | effective sample size: equivalent number of independent frames | — | [16.02](../16-machine-learning/16.02-robot-data.md) |
| $n_i$ | size of dataset $i$ | — | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $n_{\text{in}}$ | fan-in: number of inputs feeding a neuron | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $n_k$ | cumulative tick count at sample $k$ | ticks | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $n_k$ | encoder count at sample $k$ | ticks | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $n_\text{missed}$ | number of timer callbacks missed while blocked | — | [04.07](../04-ros2/04.07-services.md) |
| $n_q$ | number of queries | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $\lvert\nabla I\rvert$ | gradient magnitude (edge strength) | — | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\nabla f$ | gradient: vector of partial derivatives, points uphill | unit$_f$/unit$_x$ | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\nabla_\theta L$ | gradient: direction in which the loss rises fastest | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\neg m_i$ | the event "cell $i$ is free" | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $NM_H$, $NM_L$ | high / low noise margin of a logic link | V | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |

### O

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $O$ | FMEA occurrence (likelihood) score (1-10) | — | [21.09](../21-war-machine/21.09-failure-modes-and-safety.md) |
| $O$ | observation (images, joint angles) | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $O(n^2)$ | cost growing with the square of the state size | — | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $o$ | observation the generator is conditioned on | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $o$ | per-joint offset between LeRobot zero and URDF zero | ° | [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md) |
| $o$ | per-message storage overhead in a bag | bytes | [04.14](../04-ros2/04.14-rosbag2.md) |
| $o_t$ | observation at time $t$ (images, joint angles) | — | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $o_x$, $o_y$ | world coordinates of the grid's bottom-left corner (map origin) | m | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\odot$ | element-wise multiplication | — | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\text{offset}_k$ | byte offset where field $k$ starts in a C-style struct | bytes | [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md) |
| $\oplus$ | XOR (exclusive or) of bytes | — | [01.10](../01-first-robot/01.10-pi-pico-protocol.md), [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $\otimes$ | Hamilton (quaternion) product: composes rotations | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |

### P

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $P$ | CAN payload size | bytes | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| $P$ | cell strings in parallel in a pack | — | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $P$ | covariance matrix of the state estimate | m², rad² (mixed) | [10.02](../10-localization/10.02-uncertainty-gaussians.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $P$ | goods value of an import | USD | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $P$ | Kalman state covariance | px² | [13.12](../13-computer-vision/13.12-tracking.md) |
| $P$ | Kalman state covariance (uncertainty of the state) | units² | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $P$ | number of parameters | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $P$ | perimeter of a contour | px | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $P$ | power | W | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $P$ | processing time per message in a subscriber | s | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| $P$ | proportional term $k_p e$ of the controller output | — | [08.05](../08-control/08.05-integral-control.md) |
| $P$ | push force against a wall (in the pushing test) | N | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $P$ | solution of the Riccati equation (LQR) | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $P$ | ViT patch size (side) | px | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $P(A \mid B)$ | conditional probability of $A$ given that $B$ is known to be true | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A)$ | probability of event $A$, between 0 and 1 | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(\text{miss})$ | probability that no sample lands in the passage | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $P(s' \mid s, a)$ | transition probabilities: the world's physics | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $P(x \mid z)$ | posterior: belief after the measurement | — | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $P(x)$ | prior: belief before the measurement | — | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $P(z \mid x)$ | likelihood: sensor model from calibration | — | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $P(z)$ | evidence: normalizer so the posterior sums to 1 | — | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $P, P_{out}, P_{in}$ | power; output and input power | W | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P$, $Q$, $R$ | Kalman filter: estimate variance, process-noise variance, measurement-noise variance | m² | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $P$, $R$, $F_1$ | precision, recall, and their harmonic mean | — | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $P_0$, $x_0$ | initial covariance and initial state of a filter | m², m | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $P_{5V}$ | power drawn from the 5 V rail | W | [01.07](../01-first-robot/01.07-power-system.md) |
| $P_{\text{avg}}$ | average power draw | W | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $P_{avg}$ | average power drawn from the battery | W | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P_c = (X, Y, Z)$ | 3D point in the camera optical frame (x right, y down, z forward) | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $P_\text{clean}$ | probability of an episode with no mistake | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $P_{drive}, P_{fixed}$ | power for driving / constant power of electronics | W | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P_{elec}$ | electrical power drawn by the motor | W | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P_\text{heat}$ | heat dissipated in a regulator | W | [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $P_i, f_i, \eta_i$ | power, duty fraction of time, and efficiency of load $i$ | W, —, — | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P_\text{in}$ | input power a switching regulator draws | W | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $P_j$ | known 3D position of corner $j$ on the board / marker | m | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md), [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $P_{kg}$ | filament price per kilogram | ₪/kg | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $P_{max}$ | largest push force before the wheels slip | N | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $P_{\text{miss}}$ | chance two bit flips slip past the XOR checksum | — | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $P_{out}$ | regulator output power | W | [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $P_\text{sim}$, $P_\text{real}$ | simulator and real-world dynamics | — | [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md) |
| $P_{\text{total}}$ | total power draw of all loads | W | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $P_{undetected}$ | chance a corrupted message passes the checksum | — | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $P_w$ | 3D point in the world/robot frame | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $P_{\text{wire}}$ | heat dissipated in a wire | W | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $\bar p$ | average of the two success rates compared | — | [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| $\hat p$ | calibrated probability | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $p$ | circular pitch, tooth-to-tooth distance on the pitch circle | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $p$ | commanded torque as a percentage of stall torque | % | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $p$ | contact point | m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $p$ | IPv4 prefix length (the `/24`) | bits | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $p$ | marker width in the image | px | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $\bar p$ | mean of the measured tool positions | m | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $p$ | number of CPU cores | — | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $\hat p$ | observed success rate $k/n$ | — | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md), [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $p$ | order of an integration method (error ∝ $h^p$) | — | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $p$ | padding: zeros added around the image border | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $p$ | parameter vector $(r_L, r_R, b)$ being calibrated | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $p$ | participant index: 0 for the first ROS process on the host, 1 for the second, ... | — | [04.01](../04-ros2/04.01-why-middleware.md) |
| $p$ | per-trial probability (e.g. of a false echo); also outlier fraction | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $p$ | pitch (downward tilt) of the LiDAR | rad | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $p$ | pole: a root of the characteristic polynomial; behaviour $e^{pt}$ | 1/s | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\mathbf{p}$ | position of the robot (a point) | m | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $p$ | predicted probability of the true class | — | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $p$ | previous 16-bit encoder count | ticks | [FPY.01](../optional-foundations/python/FPY.01-python-for-csharp-devs.md) |
| $p$ | probability one network packet (fragment) is lost | — | [04.11](../04-ros2/04.11-qos.md) |
| $p$ | probability that a cell is occupied | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p$ | probability that a grid cell is occupied | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $p$ | probability that one uniform sample lands in the narrow passage | — | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $p$ | RANSAC confidence of drawing at least one all-inlier sample | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $\mathbf{p}$ | robot pose being tested in a world | m, rad | [06.08](../06-simulation/06.08-building-worlds.md) |
| $p$ | true (unknown) success rate | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\mathbf{p} = (x, y, 1)$ | point in homogeneous coordinates | px | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $p'_i$, $q'_i$ | points with their centroid subtracted (centered) | m | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $p(A \mid B)$ | probability of $A$ given $B$ | — | [10.01](../10-localization/10.01-what-is-localization.md) |
| $p(a \mid o)$ | distribution of demonstrated actions for an observation | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $p(t)$ | belief that a remembered object is still there | — | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $p(x)$ | probability density function of a continuous variable | 1/unit of $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\bar p$, $\bar q$ | centroids (mean points) of the source and target sets | m | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $p$, $p_{goal}$ | position along a line and the goal position | m | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $p, q, r$ | body angular velocity measured by a gyro about body $x$, $y$, $z$ | rad/s | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $p^\star$ | belief threshold above which trusting memory pays | — | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $p_0$ | detector confidence at observation time | — | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $p_0$ | prior occupancy probability (0.5 = unknown) | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p_0, p_1$ | success rates defining $H_0$ and $H_1$ | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $p_1$ | probability a burst crosses the HIGH threshold | — | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $p_1$, $p_2$ | tangential lens distortion coefficients | — | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $p_2$ | probability a second burst lands in the double-clap window | — | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $\mathbf{p}_A$ | a point expressed in frame A coordinates | m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $p_{\min}, p_{\max}$ | workspace box limits | m | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $p_{\text{base}}, p_{\text{optical}}$ | the same point in the base frame / camera optical frame | m | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $p_{cam}$ | camera position | m | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $p_{\text{commanded}}$, $p_{\text{cmd}}$ | position you asked the tool to go to | m | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $p_f$ | probability one video frame gives a false detection | — | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $p_{false}$ | probability the detector says "door" in a wall cell | — | [10.03](../10-localization/10.03-bayes-filter.md) |
| $p_{\text{frame}}$ | coordinates of a point expressed in the named frame | m | [05.01](../05-frames-and-transforms/05.01-why-frames.md) |
| $p_\text{hit}$ | inverse sensor model: occupancy probability of the cell where the beam ends | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p_{hit}$ | probability the detector says "door" in a door cell | — | [10.03](../10-localization/10.03-bayes-filter.md) |
| $p_i$ | any point on joint $i$'s axis, in the base frame | m | [14.04](../14-robotic-arm/14.04-forward-kinematics.md), [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $p_i$ | measured tool position on run $i$ | m | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $p_i$ | probability of sampling dataset $i$ | — | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $p_i$ | softmax probability of class $i$ | — | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $p_i$, $q_i$ | matched points: source (new scan) and target (old scan) | m | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $p_i^{robot}$ | LiDAR point in the robot frame | m | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $p_i^{world}$ | the same point in the world frame | m | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $p_\text{in}$, $p_\text{out}$ | price per million input / output tokens | $/MTok | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $p_{interp}$ | precision envelope (best precision at that recall or higher) | — | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $p_j$ | likelihood of beam $j$ | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $p_k$ | softmax probability of class $k$ | — | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $p_{\text{meas}}$ | position measured with a ruler | m | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $p_\text{miss}$ | detector miss rate per look | — | [19.07](../19-llm-robot-agents/19.07-perception-action-loops.md) |
| $p_\text{miss}$ | inverse sensor model: occupancy probability of a cell the beam passes through | — | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\hat{\mathbf{p}}_t, \mathbf{p}_t$ | estimated / true position at time sample $t$ | m | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $p_{\text{tool}}$ | tool (gripper reference point) position | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\mathbf{p}_{tool}(q)$ | tool position (where the payload sits) | m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $\parallel$ | "in parallel with" operator for resistances | — | [01.03](../01-first-robot/01.03-workbench-and-tools.md), [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $\partial f/\partial x$ | partial derivative: change of $f$ per unit $x$ with other inputs held fixed | unit$_f$/unit$_x$ | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $PB$ | DDS port base (default 7400), first UDP port ROS 2 uses | — | [04.01](../04-ros2/04.01-why-middleware.md) |
| $PG$ | DDS participant gain: port gap between two ROS processes on one host (default 2) | — | [04.01](../04-ros2/04.01-why-middleware.md) |
| $\text{PM}$ | phase margin: extra lag the loop tolerates before oscillating | ° or rad | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\text{PPR}$ | pulses per revolution of the encoder disc on one channel | — | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |

### Q

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $Q$ | battery capacity (charge) | Ah | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $Q$ | electric charge (battery capacity in Ah) | C or Ah | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $Q$ | heat needed to warm a solder joint | J | [01.03](../01-first-robot/01.03-workbench-and-tools.md) |
| $Q$ | LQR state weight: how much being off-target costs | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $Q$ | process noise covariance | px² | [13.12](../13-computer-vision/13.12-tracking.md) |
| $Q$ | process-noise variance (covariance) added per prediction step | m² | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $Q$ | wheel-travel noise covariance in 09.06 (called $M$ in 10.06) | m² | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $Q(a)$ | estimated mean reward of action $a$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $Q, K, V$ | queries, keys and values: learned projections of tokens | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $Q^\pi(s, a)$ | action value: expected return taking $a$ first, then following $\pi$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $Q_\theta(s, a)$ | Q-function computed by a neural network | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $q$ | configuration: the smallest set of numbers that fixes where every part of the robot is | m, rad | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $\ddot q$ | joint accelerations | rad/s² | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q$ | joint position (angle for a revolute joint, distance for a prismatic one) | rad or m | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $\dot q$ | joint speeds | rad/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $q$ | joint vector: one angle per arm joint | rad | [14.01](../14-robotic-arm/14.01-arm-anatomy.md), [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\dot q$ | joint velocities | rad/s | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $q$ | probability of a false detection per frame | — | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $q$ | process-noise rate of a random walk | m²/s | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $-q$ | same rotation as $q$ (double cover) | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q$ | squared distance to the landmark, $\delta_x^2 + \delta_y^2$ | m² | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $q$ | squared range to the landmark, $u^2 + v^2$ | m² | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $q$ | stored INT8 integer code | — | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $q$ | subscription queue depth | — | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $q$ | unit quaternion: orientation as axis and half-angle, $(x, y, z, w)$ | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q = (x, y, z, w)$ | unit quaternion representing a 3D rotation | — | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $q = 0$ | the URDF zero pose (the "L" pose), a modelling choice, not the servo's 0° | — | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $q(t)$ | trajectory: joint position as a function of time | rad | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q^*$ | conjugate quaternion: same axis, opposite angle (= inverse for unit $q$) | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q_*(a)$ | true mean reward of bandit action $a$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $\dot q_0$ | secondary joint motion projected into the null space (redundancy resolution) | rad/s | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $q_0, q_1$ | start and goal joint positions of a move | rad | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q_1, q_2$ | shoulder angle (from $+x$) and elbow angle (relative to link 1) | rad | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $q_\phi$ | CVAE encoder (only used in training) | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $q_c$, $q_m$ | candidate joint command and last safe measured joints | rad | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $q_i$ | angle of joint $i$ (in the planar arm, $q_2$ is measured relative to the previous link) | rad | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $q_i$ | i-th entry of the ROS 6-D pose $(x, y, z, \text{rot}_x, \text{rot}_y, \text{rot}_z)$ | m or rad | [09.07](../09-odometry/09.07-odometry-in-ros2.md) |
| $\dot q_{\text{peak}}, \ddot q_{\text{peak}}$ | highest speed / acceleration reached during a move | rad/s, rad/s² | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q_{\text{start}}, q_{\text{goal}}, q_{\text{rand}}, q_{\text{near}}, q_{\text{new}}$ | RRT configurations: start, goal, random sample, nearest tree node, newly added node | m, rad | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $q_{\text{urdf}}$ | joint angle in the URDF / ROS convention | rad | [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md) |

### R

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $R$ | 3×3 rotation matrix | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $R$ | Borenstein: radius of the arc the robot drives instead of a straight leg | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $R$ | Harris corner response | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $R$ | LQR effort weight: how much using control costs | mixed | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $R$ | measurement noise covariance | px² | [13.12](../13-computer-vision/13.12-tracking.md) |
| $R$ | measurement-noise variance (covariance) of a sensor | m² | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $R$ | motor winding resistance | Ω | [01.08](../01-first-robot/01.08-first-motor-spin.md), [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md), [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $R$ | radius of a turn | m | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $R$ | radius of the arc the robot drives | m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $R$ | radius of the arc the tool traces in a joint-space move | m | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $R$ | range to the target (LiDAR) | m | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $R$ | rate of loud ambient bursts in the room | 1/h | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $R$ | reaction rate in Ziegler–Nichols open-loop rules, $K/\tau$ | rad/s² per duty | [08.08](../08-control/08.08-tuning-pid.md) |
| $R$ | real radius of a sphere (ball) | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $R$ | resistance | Ω | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $R$ | return (reward) of an episode or action | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $R$ | rotation matrix | — | [11.04](../11-slam/11.04-scan-matching-icp.md), [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $R$ | rotation matrix (orthonormal, det +1) | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R$ | rotation matrix (turns vectors from one frame's axes to another's) | — | [05.01](../05-frames-and-transforms/05.01-why-frames.md), [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R$ | set of rays that return a finite range in both scans | — | [06.08](../06-simulation/06.08-building-worlds.md) |
| $R$ | turning radius of the robot | m | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $R$ | turning radius: signed distance from base_link to the ICC (positive = left) | m | [08.01](../08-control/08.01-open-vs-closed-loop.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $R$ | wheel radius | m | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [06.01](../06-simulation/06.01-why-simulate.md) |
| $R'$ | resistance per metre of one conductor | Ω/m | [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) |
| $R(\psi)$ | 2D rotation matrix by angle $\psi$ | — | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $R(\theta)$ | 2D rotation by angle $\theta$ | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R(\theta)$ | 2D rotation matrix for angle $\theta$ | — | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R(s, a, s')$ | reward function | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $R$, $G$, $B$ | red, green, blue channel values | 0–255 | [13.01](../13-computer-vision/13.01-images-as-data.md) |
| $R/\ell$ | wire resistance per metre | Ω/m | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $R^{\mathsf T}$ | transpose of $R$ (equals its inverse for a rotation) | — | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R_A, R_B, R_X$ | rotation parts of $A$, $B$, $X$ | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $R_{A\,B}$ | rotation of frame B relative to A; columns are B's axes in A | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_{CK}$ | rotation from camera optical frame to camera_link | — | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $R_{CS}$ | current-sense resistor on the driver carrier | Ω | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $R_{DS}$ | on-resistance of the FETs in the current path | Ω | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $R_{DS(on)}$ | on-resistance of a MOSFET switch | Ω | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md), [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md) |
| $R_{IPROPI}$ | resistor on the IPROPI current-sense pin | Ω | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $R_M$, $L_M$ | motor winding resistance / inductance | Ω, H | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $R_{\min}$ | minimum arc radius before RPP slows down | m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $R_{\theta JA}$ | thermal resistance junction-to-air (°C rise per W) | °C/W | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $R_{base,cam}$ | camera orientation in the base frame | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $R_{\text{base}\leftarrow\text{optical}}$ | rotation that converts camera-optical-frame coordinates into `base_link` coordinates | — | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $R_{branch}$ | resistance of fuse, relay, connectors and wire in a branch | Ω | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $R_\text{contacts}$ | resistance of connectors and contacts | Ω | [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $R_{\text{err}}$ | relative rotation between two orientations | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_i$ | worst-case response time of task $i$ | s | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $R_i$, $t_i$ | pose of the calibration board in view $i$ | —, m | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $R_i$, $t_i$, $\theta_i$ | rotation, position and heading of node $i$ | —, m, rad | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $R_{ij}$ | entry in row $i$, column $j$ of a rotation matrix | — | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $R_{ij}$ | entry of a rotation matrix (0-indexed as in numpy, for angle extraction) | — | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $R_{int}$ | battery pack internal resistance | Ω | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $R_{\text{internal}}$ | internal resistance of the battery (and wiring) | Ω | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $R_p$ | pull-up resistor value | Ω | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $R_{p,min}$, $R_{p,max}$ | smallest / largest legal I²C pull-up resistor | Ω | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $R_\text{pack}$ | total pack internal resistance | Ω | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $R_{parallel}$ | total resistance of resistors in parallel | Ω | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $R_{path}$ | total resistance from cells to loads | Ω | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $R_{pull}$ | pull-up / pull-down resistor | Ω | [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $R_{series}$ | total resistance of resistors in series | Ω | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $R_{\text{shunt}}$ | shunt resistor value | Ω | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $R_{source}$ | source resistance of battery plus wiring | Ω | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $\hat R_t$ | value target for the critic | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $R_t$, $R_b$ | top / bottom resistor of the battery divider | Ω | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $R_{th}$ | Thevenin (source) resistance of a divider | Ω | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $R_{top}$, $R_{bottom}$ | upper / lower resistor of a voltage divider | Ω | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $R_{\text{wire}}$ | resistance of a wire | Ω | [00.06](../00-orientation/00.06-safety-mindset.md), [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $R_x(\alpha), R_y(\beta), R_z(\gamma)$ | elementary 3D rotations about the $x$, $y$, $z$ axis | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_x, R_y, R_z$ | rotation about the x, y, z axis | — | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $R_z$, $t_z$, $\theta_z$ | rotation, translation and heading of the measurement $z_{ij}$ | —, m, rad | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $r$ | (effective) wheel radius | m | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $r$ | distance from the shoulder to the target point | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md), [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\hat r$ | estimated wheel radius | m | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $r$ | image radius of the ball | px | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $r$ | lever arm: distance from the rotation axis to where a force acts | m | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\mathbf{r}$ | lever arm: vector from the joint to where the force acts (or offset of a point on a rotating body) | m | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $r$ | lever: distance from the joint axis to where force is applied | m | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $r$ | map resolution: side length of one grid cell | m | [11.01](../11-slam/11.01-map-representations.md), [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $r$ | measured range to a landmark | m | [10.01](../10-localization/10.01-what-is-localization.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $r$ | radius of a hole or circle | mm | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $r$ | radius of the contact patch a soft pad flattens into | m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $r$ | range (distance) to a point or object measured by a sensor | m | [05.01](../05-frames-and-transforms/05.01-why-frames.md), [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [07.07](../07-sensors/07.07-2d-lidar.md) |
| $r$ | range: distance from the sensor to a point (polar/spherical radius) | m | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $r$ | receptive field: input pixels that influence one output | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $\mathbf{r}$ | residual vector: measured minus predicted | as $\mathbf{b}$ | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $r$ | reward returned after an action | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $r$ | setpoint (reference): the value you want, e.g. desired wheel speed | rad/s (or the controlled quantity's unit) | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [08.04](../08-control/08.04-proportional-control.md) |
| $r$ | throw-arm radius (pivot to payload) | m | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $r$ | training throughput | samples/s | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $r$ | turning radius of a constant-velocity arc, $v/\omega$ | m | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $r$ | wheel radius | m | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md), [06.06](../06-simulation/06.06-ros2-control.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md), [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md), [P03](../projects/P03-encoder-movement.md) |
| $r$ | wheel radius (0.045 m on karmel) | m | [08.09](../08-control/08.09-feedforward-exact-speed.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $r(\theta)$ | range at which a tilted LiDAR beam at angle $\theta$ hits the floor | m | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\hat r$, $\hat b$ | least-squares estimates of wheel radius / wheelbase | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $(r, \phi)$ | polar coordinates: range and angle | m, rad | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md), [07.07](../07-sensors/07.07-2d-lidar.md) |
| $r$, $r_n$ | RRT* rewiring radius (with $n$ nodes) | m or rad | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $r^2$ | squared normalized distance from the image center, $x^2 + y^2$ | — | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $\mathbf{r}_1, \mathbf{r}_2$ | first two columns of the camera rotation | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $r_1, r_2$ | ranges of two LiDAR beams | m | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $r_L$, $r_R$ | effective radius of the left / right wheel | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $r_L$, $r_R$ | left / right wheel radius | m | [08.10](../08-control/08.10-straight-line-heading-control.md), [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $r_{\max}$ | maximum reach of the arm from the base origin | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $r_\pi$, $P_\pi$ | expected reward vector and transition matrix under policy $\pi$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $r_{\text{circ}}$ | circumscribed radius, smallest circle around the footprint | m | [P11](../projects/P11-autonomous-navigation.md) |
| $r_{\text{circumscribed}}$, $r_{\text{circ}}$ | circumscribed radius: smallest circle around the footprint | m | [12.04](../12-navigation/12.04-costmaps.md) |
| $r_e$ | radius of the minimum enclosing circle | px | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $r_{\text{eff}}$ | effective wheel radius implied by odometry | m | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $r_{exp}$ | expected image radius of the ball at a given row | px | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $r_i$ | range of LiDAR beam $i$ | m | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $r_i(\mathbf{p})$ | range of lidar ray $i$ from pose $\mathbf{p}$ | m | [06.08](../06-simulation/06.08-building-worlds.md) |
| $r_{\text{inflation}}$ | Nav2 inflation radius, distance where cost falls to 0 | m | [P11](../projects/P11-autonomous-navigation.md) |
| $r_{\text{insc}}$ | inscribed radius, largest circle inside the robot footprint | m | [P11](../projects/P11-autonomous-navigation.md) |
| $r_{\text{inscribed}}$, $r_{\text{in}}$ | inscribed radius: largest circle inside the footprint | m | [12.04](../12-navigation/12.04-costmaps.md), [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $r_j$, $a_j$ | measured range of LiDAR beam $j$ and its angle on the robot | m, rad | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $r_k$ | measured range of LiDAR beam $k$ | m | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $r_k$ | recall at step $k$ | — | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $r_{nom}$, $r_{true}$ | nominal (assumed) vs true wheel radius | m | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $r_{sp}$, $\omega_{sp}$ | wheel-speed setpoint fed to the feedforward | rad/s | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $\text{range\_{min}}, \text{range\_{max}}$ | lowest / highest raw reading recorded during calibration | counts | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $\text{raw}$ | servo encoder reading (0–4095) | counts | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $\text{raw}_{u16}$ | raw 16-bit-scaled ADC reading | — | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $\text{RMSE}$ | root-mean-square error | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\mathrm{RMSE}$ | root-mean-square error: typical total error vs truth | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| roll, pitch, yaw | rotation about x (roll), y (pitch), z (yaw) | rad | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| roll, pitch, yaw | rotations about x, y, z (URDF `rpy`) | rad | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\text{Rot}(\texttt{axis}, q)$ | rotation by $q$ about the joint axis | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\text{RPM}$ | revolutions per minute | rev/min | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\text{RPN}$ | FMEA risk priority number | — | [21.09](../21-war-machine/21.09-failure-modes-and-safety.md) |
| $\text{RTF}$ | real-time factor of speech recognition | — | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |

### S

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $S$ | cells in series in a pack | — | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $S$ | checkerboard square size | m | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $S$ | cost (time) of a full search | s | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $S$ | covariance of the innovation (prediction + measurement uncertainty) | m² | [11.03](../11-slam/11.03-the-slam-problem.md), [13.12](../13-computer-vision/13.12-tracking.md) |
| $S$ | FMEA severity score (1-10) | — | [21.09](../21-war-machine/21.09-failure-modes-and-safety.md) |
| $S$ | innovation (co)variance: how big the surprise should be | m² | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $S$ | number of wall segments in the simulated world | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $S$ | overall speed-up (Amdahl's law) | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $S$ | samples processed during training (steps × batch) | — | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $S$ | scaling matrix | — | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $S$ | sensitivity $1/(1+L)$: fraction of a disturbance's effect that feedback leaves | — | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $S$ | shipping cost of an import | USD | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $S(\mathbf{p}, \alpha)$ | scan-change score: how much the scan changes when moving in direction $\alpha$ | m | [06.08](../06-simulation/06.08-building-worlds.md) |
| $S(r)$ | least-squares cost: sum of squared residuals | m² | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\mathcal{S}$, $\mathcal{A}$ | set of states, set of actions | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $S^{(m)}$, $S_{min}$ | cost of sample trajectory $m$, and the lowest cost | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\mathcal{S}_i$ | screw axis of joint $i$ in the base frame at $q = 0$ (product of exponentials) | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $S_k$ | trajectory cost of MPPI sample $k$ | — | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $s$ | 2-bit quadrature state built from channels A and B | — | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $s$ | accelerometer scale factor | — | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $s$ | across-flats width of a hex nut | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $s$ | arc length each wheel travels during an in-place turn | m | [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $s$ | arc length: distance travelled along a circle | m | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $s$ | CLIP temperature, $e^{\text{logit\_scale}}$ (100) | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $s$ | convolution stride: step when sliding the kernel | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $s$ | distance between the two gap-measurement points | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $s$ | distance driven along the path | m | [08.01](../08-control/08.01-open-vs-closed-loop.md), [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $s$ | docking stand-off distance in front of the tag | m | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $s$ | homogeneous scale factor of a projection (equals depth $Z$) | m | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $s$ | Laplace (complex frequency) variable; poles are values of $s$ | 1/s | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $s$ | linear classifier score ($> 0$ means the positive class) | — | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $s$ | marker side length in the image | px | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $s$ | path parameter along a Cartesian straight line (0 at start, 1 at end) | — | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $s$ | per-joint sign between LeRobot and URDF, $\pm 1$ | — | [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md) |
| $s$ | points per RANSAC sample (4 for a homography, 3 for a plane) | — | [13.04](../13-computer-vision/13.04-features-and-matching.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $s$ | quantization scale: real value per integer step | varies | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $s$ | real width of a marker | m | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $\mathbf{s}$ | robot state $(x, y, \theta)$ in an ODE | m, m, rad | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $s$ | safety factor | — | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $s$ | sagitta, largest gap between a true circle and its polygon export | mm | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $s$ | sample standard deviation (divides by $N-1$) | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $s$ | scale factor applied to both wheel speeds to respect the motor limit | — | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $s$ | side length of a fiducial tag | m | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $s$ | side length of the cube an octree covers | m | [11.01](../11-slam/11.01-map-representations.md) |
| $s$ | side length of the test square | m | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s$ | sideways offset of a specular (mirror) reflection | m | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $s$ | speed-up of the optimised part alone | — | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $s$ | unknown global scale of a monocular reconstruction | — | [11.08](../11-slam/11.08-visual-slam.md) |
| $s(x)$ | anomaly score: reconstruction error | unit of $x$ | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $s$, $s'$ | current state and next state | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $s, s^{*}$ | current and goal image feature vectors | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $s_2$ | shorthand for $\sin q_2$ | — | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $s_i$ | serialised size of one message of topic $i$ | B | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $s_i$ | serialized message size of topic $i$ | bytes | [04.14](../04-ros2/04.14-rosbag2.md) |
| $s_i$ | wheel-distance difference $d_R - d_L$ in spin run $i$ | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\mathrm{sat}(\cdot)$ | saturation: clip the duty to ±1 | — | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\text{SE}$ | standard error of the mean, $s/\sqrt{n}$ | as $e$ | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| SE(2), SE(3) | the set of rigid motions (rotation + translation) in 2D / 3D | — | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $\mathrm{SE}(k)$ | standard error of the fitted gain | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\text{SEM}$ | standard error of a cluster mean, $\sigma/\sqrt n$ | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\mathrm{SEM}$ | standard error of the mean: uncertainty of an average | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\sin\theta, \cos\theta$ | $y$ and $x$ coordinate of the point at angle $\theta$ on the unit circle | — | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\text{size}_k$ | size of field $k$ | bytes | [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md) |
| slope | motor gain: wheel speed per unit duty above the deadband | rad/s | [08.04](../08-control/08.04-proportional-control.md) |
| SNR | signal-to-noise ratio | — | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| SO(3) | the set of all 3D rotation matrices | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $SoC_\text{now}$ | present state of charge (fraction) | — | [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md) |
| stroke | maximum opening of the jaw | m | [15.05](../15-manipulation/15.05-grasp-planning.md), [15.07](../15-manipulation/15.07-detect-and-approach.md) |

### T

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $T$ | air temperature | °C | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md), [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $T$ | duration of a move | s | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $T$ | encoder sample period / time window over which ticks are counted | s | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $T$ | episode horizon in steps | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $T$ | episode length in steps | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $T$ | homogeneous transform: rotation and translation packed in one 3×3 (2D) or 4×4 (3D) matrix | — / m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T$ | loop period, the time between two loop iterations ($T = 1/f$) | s | [00.01](../00-orientation/00.01-what-is-a-robot.md), [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $T$ | number of diffusion steps | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $T$ | period between messages, $1/f$ | s | [04.11](../04-ros2/04.11-qos.md) |
| $T$ | planning horizon of the local planner | s | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $T$ | pure delay in a frequency-response formula | s | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $T$ | PWM period, one on-plus-off cycle of the switching signal | s | [01.08](../01-first-robot/01.08-first-motor-spin.md), [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $T$ | rigid transform (rotation + translation) found by scan matching | — | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $T$ | sample period of a digital loop | s | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $T$ | temperature (chip sensor, or air temperature for the speed of sound) | °C | [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $T$ | temperature for calibrating confidence scores | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $T$ | time of one lidar revolution, $1/f$ | s | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $T$ | torque on a gear | N·m | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $T$ | total duration of a motion profile | s | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $T$ | velocity measurement window (time over which ticks are counted) | s | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $T(\texttt{xyz},\texttt{rpy})$ | fixed joint origin from the URDF (from CAD) | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T(y, x)$ | adaptive (local) threshold | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $T^*$ | desired tool pose (IK input) | — | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $T_1$, $T_2$ | torques on driving and driven gear | N·m | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $T_{1/2}$ | belief half-life | s | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $T_A$ | ambient (room) temperature | °C | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $T_{A\,B}$ | pose of frame B in frame A; converts B coordinates into A | — / m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T_J$ | chip junction temperature | °C | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $T_{a,b}$ | 4×4 homogeneous transform: pose of frame $b$ expressed in frame $a$ | — (m in translation) | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{a,b}$ | transform giving the pose of frame $b$ in frame $a$ | — | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $T_{a\,b}$ | rigid transform of frame $b$ expressed in frame $a$ | m, rad | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $T_b$ | duration of a blocking call | ms | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $T_b$ | time a callback blocks the node (e.g. waiting on a service) | s | [04.07](../04-ros2/04.07-services.md), [04.15](../04-ros2/04.15-lifecycle-nodes.md) |
| $T_{\text{base,optical}}$ | pose of the camera's optical frame in the base frame | — | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $T_{\text{base},\text{tool}}(q)$ | pose of the tool in the base frame, as a function of the joints (forward kinematics) | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{\text{before}}$ | seconds of clearance the collision monitor keeps | s | [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md) |
| $T_c$ | period between motor commands | ms | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $T_{command}$ | period between motor commands | s | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $T_{\text{control}}$ | control loop period | s | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $T_d$ | derivative time (standard PID form), $k_d/k_p$ | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $T_{delay}$ | total loop delay (latency) | s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $T_{environment}$ | hottest temperature the part will see | °C | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $T_{err}$ | PBVS pose error: object pose seen from where the camera should end up | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $T_i$ | integral time (standard PID form), $k_p/k_i$ | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $T_i$ | period of task $i$ | s | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $T_\text{inf}, T_\text{rtt}, T_\text{jitter}$ | inference time, network round trip, mean jitter | s | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $T_k$ | pose of camera keyframe $k$ (in $SE(3)$, 6 numbers) | m, rad | [11.08](../11-slam/11.08-visual-slam.md) |
| $T_{limit}$ | temperature where a plastic starts to soften | °C | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $T_o$, $T_p$, $T_a$ | observation frames used, actions predicted, actions executed | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $T_{\text{parent},\text{child}}(q)$ | transform across one URDF joint | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_\text{poll}$ | sampling period of a decision loop | s | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $T_{\text{rebuild}}$ | time to rebuild after editing a header | s | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $T_\text{report}$ | a component's own reporting period | s | [20.03](../20-final-robot/20.03-software-integration-bringup.md) |
| $T_\text{rt}$ | decision round-trip time | s | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $T_{\text{sensor}}$, $T_{\text{costmap}}$, $T_{\text{controller}}$, $T_{\text{latency}}$ | latencies of each pipeline stage / total | s | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $T_\text{tick}$ | behaviour-tree tick period | s | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $T_{\text{total}}$ | total study time for the course | h | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |
| $T_u$ | ultimate period: period of that oscillation | s | [08.08](../08-control/08.08-tuning-pid.md) |
| $T_w, T_s$ | wall-clock duration and matching simulated duration | s | [06.04](../06-simulation/06.04-bridging-gazebo-ros2.md) |
| $T_\text{wall}$ | wall-clock time of a parallel build | s | [04.05](../04-ros2/04.05-packages-and-workspaces.md) |
| $T_{\text{watchdog}}$ | firmware watchdog timeout | s | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $T_\text{worst}$ | worst-case end-to-end latency of a processing chain | s | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $T_{\text{°C}}$ | CPU temperature in degrees Celsius | °C | [FL.04](../optional-foundations/linux-and-tools/FL.04-filesystem.md) |
| $t$ | binary threshold value | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $t$ | diffusion step index | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $t$ | elapsed time | s | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $t$ | exposure time | s | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $t$ | flow-matching time, 0 = noise, 1 = data | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $t$ | fraction along a segment of the closest point (0 at $A$, 1 at $B$) | — | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $t$ | interpolation fraction (0 = first orientation, 1 = second) | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $t$ | round-trip echo time | s | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $t$ | shell (wall) thickness of a print | cm | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $t$ | thickness of the clamped part | mm | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $t$ | time | s | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $t$ | time (also latency before braking; also battery runtime) | s (h for runtime) | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $t$ | time step / number of decisions so far | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $t$ | translation part of a transform (where the child frame origin is in the parent) | m | [05.01](../05-frames-and-transforms/05.01-why-frames.md), [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $t$ | translation vector | m | [11.04](../11-slam/11.04-scan-matching-icp.md), [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $\mathbf{t}$ | translation vector (origin of the child frame in the parent frame) | m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $t$ | translation vector (robot position) | m | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $t$ | true (reference) value a sensor should read | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $t$ | VLM answer latency | s | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $\mathbf{t}$, rpy | URDF origin: translation and roll-pitch-yaw of a joint | m, rad | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $[\mathbf{t}]_\times$ | cross-product matrix of translation $\mathbf t$ | m | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $t_0$ | time at which the step was applied | s | [08.02](../08-control/08.02-motor-step-response.md) |
| $t_0$ | time the step is applied | s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $t_{0.975,\,n-1}$ | t-distribution multiplier for a 95% confidence interval with $n$ trials | — | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $t_{50}, t_{90}, t_{99}$ | time to reach 50 / 90 / 99 % of the final speed | s | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $t_\text{CV}$ | duration of the constant-voltage charging tail | h | [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md) |
| $t_X, t_{A_i}, t_{B_i}$ | translation parts of $X$, $A_i$, $B_i$ | m | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $t_a$ | time spent accelerating in a trapezoidal profile | s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $t_a$ | time spent accelerating to cruise speed | s | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $t_{accel}$, $t_{cruise}$ | duration of the ramp and of the cruise phase | s | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $t_b$ | worst switch-bounce duration | ms | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $t_{bit}$ | duration of one UART bit | s | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| $t_c$ | compile time of one translation unit | s | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $t_c$ | predicted time to collision along the commanded twist | s | [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md) |
| $t_c$, $t_{\text{cruise}}$ | time spent cruising at constant speed | s | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $t_{dead}$ | dead time before braking starts | s | [20.04](../20-final-robot/20.04-safety-case.md) |
| $t_\text{dead}$ | dead time of a stop mechanism (detect + act) before braking | s | [20.04](../20-final-robot/20.04-safety-case.md) |
| $t_{dead}$ | dead time when both bridge switches are off | s | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $t_e$ | exposure time | s | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $t_{\text{echo}}$ | round-trip time of the echo | s | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $t_{err}, \theta u_{err}$ | translation and axis-angle rotation parts of $T_{err}$ | m, rad | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $t_h$ | time one interrupt handler takes | s | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $t_i$ | duration of one lesson | h | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |
| $t_\text{inf}$ | inference time of one policy call | s | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $t_k$ | build time of package $k$ | s | [04.05](../04-ros2/04.05-packages-and-workspaces.md) |
| $t_k$ | timestamp of sample $k$ (from the robot's clock) | s | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $t_{\text{land}}$ | time for a dropped object to hit the ground | s | [06.03](../06-simulation/06.03-gazebo-basics.md) |
| $t_\text{last}$ | time of a component's last diagnostic report | s | [20.03](../20-final-robot/20.03-software-integration-bringup.md) |
| $t_{lat}$ | reaction latency before braking starts | s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $t_{\text{latency}}$ | age of the distance reading | s | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $t_{\text{link}}$ | link time | s | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $t_{\text{network}}$, $t_{\text{loop}}$, $t_{\text{motor}}$ | network, control-loop and motor delays | s | [01.15](../01-first-robot/01.15-teleop-from-laptop.md) |
| $t_\text{now}$ | current time | s | [20.03](../20-final-robot/20.03-software-integration-bringup.md) |
| $t_{on}$ | on-time within one PWM period | s | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $t_\text{open}$ | time the robot runs open loop on one chunk | s | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $t_p$ | peak time: time to the first overshoot peak | s | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $t_p$ | perception time per frame | s | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $t_r$ | fastest real event that must not be missed | ms | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $t_r$ | rise time (30 % to 70 % of the supply) | s | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $t_r$ | rise time: 10 % to 90 % of a step | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $t_r$ | rolling-shutter readout time (top row to bottom row) | s | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $t_r$, $t_f$ | switching rise / fall time | s | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $t_{r,max}$ | maximum allowed I²C rise time | s | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md) |
| $t_\text{react}$ | time from an event to the robot reacting | s | [04.13](../04-ros2/04.13-debugging-and-introspection.md) |
| $t_\text{react}$, $t_\text{cancel}$ | reaction time; time to cancel the running action | s | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $t_{\text{reaction}}$ | reaction time before braking starts | s | [P01](../projects/P01-manual-drive.md) |
| $t_{\text{reaction}}$ | total reaction time (latency plus stopping) | s | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $t_{\text{ring}}$ | ringing time of the transducer after it transmits | s | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $t_{run}$ | run time on one charge | h | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $t_{\text{run}}$ | runtime on one charge | h | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $t_{s,2\%}$, $t_s$ | settling time: until the response stays within 2 % | s | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $t_\text{seen}$ | time the object was last seen | s | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $t_\text{step}$ | time per training step | s | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $t_{step}$ | time the open-loop step was applied | s | [08.08](../08-control/08.08-tuning-pid.md) |
| $t_{\text{stop}}$ | time the motors take to stop | s | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $t_\text{stop}$ | time to stop | s | [04.08](../04-ros2/04.08-actions.md) |
| $t_\text{tools}, t_\text{sys}$ | tokens of tool definitions and of the system prompt | tokens | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md) |
| $t_\text{transcript}(k)$ | tokens of conversation so far at turn $k$ | tokens | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md) |
| $t_\text{transport}, t_\text{queue}, t_\text{filter lag}$ | delay from the network, from waiting in a queue, and from filter smoothing | s | [04.13](../04-ros2/04.13-debugging-and-introspection.md) |
| $t_{watchdog}$ | watchdog timeout | s | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $t_{\text{wd}}$ | watchdog timeout | s | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $t_x, t_y$ | components of the translation | m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $\tan\theta$ | $\sin\theta/\cos\theta$, opposite over adjacent | — | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\text{tf}, \text{idf}$ | term frequency in an episode; inverse document frequency | — | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| ticks_per_rev | encoder ticks per wheel revolution (CPR) | ticks | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\text{torque\_limit}$ | fraction of stall torque you allow the gripper servo to use | — | [15.02](../15-manipulation/15.02-grippers.md) |
| $TP$, $FP$, $FN$ | true positives, false positives, false negatives | — | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $TP, TN, FP, FN$ | counts of true positives, true negatives, false positives, false negatives | — | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\operatorname{tr}(R)$ | trace: sum of the diagonal entries | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\texttt{Acceleration}$ | servo acceleration register (1 unit = 100 steps/s²) | 100 steps/s² | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\texttt{Goal\_Velocity}$ | servo speed register | steps/s | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\texttt{stable\_ms}$ | debounce time a level must hold before it is accepted | ms | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $\texttt{Torque\_Limit}$ | servo torque cap register, 0..1000 for 0..100 % of stall | 0.1 % | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |

### U

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $U$ | CPU utilisation of a set of periodic tasks | — | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $U$ | MPPI nominal control sequence $(u_0, \dots, u_{T-1})$ | m/s, rad/s | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $U$ | MPPI nominal control sequence $(v_t, \omega_t)$ | m/s, rad/s | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $U$ | network uplink bandwidth | bit/s | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $U \Sigma V^\top$ | singular value decomposition of the centred data | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $U$, $S$, $V$ | factors of the SVD $H = U S V^\top$ | — | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $U, \Sigma, V$ | factors of the singular value decomposition (SVD) | — | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $U, \Sigma, V$ | SVD factors: rotation, diagonal of singular values, rotation | — | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md), [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\mathcal U[0,1)$ | uniform distribution on $[0, 1)$ | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $u$ | commanded duty, in $[-1, 1]$ | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $u$ | controller output / command sent to the actuator (PWM duty for karmel's motors) | — (duty, −1…1) | [08.01](../08-control/08.01-open-vs-closed-loop.md), [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $u$ | effective motor voltage $d \cdot V$ | V | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md), [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $u$ | motor command (duty) | — | [P04](../projects/P04-pid-movement.md) |
| $u$ | motor command / duty cycle used as model input | — | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\mathbf u$ | odometry input, wheel travels $(d_L, d_R)$ | m | [09.06](../09-odometry/09.06-accumulated-error.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\hat{\mathbf{u}}$ | unit direction of a path segment (or of a rotation axis in FM.12) | — | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $u = (v, \omega)$ | velocity command: linear and angular speed | m/s, rad/s | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $u(t)$ | command input at time $t$ | rad/s | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $u(t)$ | PWM input as a function of time (0 or 1) | — | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\bar u$, $\bar v$ | centroid of a blob/mask in pixels | px | [13.03](../13-computer-vision/13.03-classical-detection.md), [13.11](../13-computer-vision/13.11-segmentation.md) |
| $u, v$ | offsets from robot to landmark: $u = l_y - y$, $v = l_x - x$ (here $v$ is not a speed) | m | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $(u, v)$ | coordinates of a point along the child frame's own axes | m | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $(u, v)$ | pixel coordinates | px | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $(u, v)$ | pixel coordinates (column, row) in an image | px | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $(u, v)$ | pixel coordinates (u right, v down) | px | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $(u, v)$ | pixel coordinates: $u$ right, $v$ down from the image's top-left corner | px | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $u_0$ | base duty that the correction is added to (tick-sync loop) | — | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $u_0$ | size of an input step | — | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $u_0$, $u_j$ | random start and comb-tooth positions of systematic resampling | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\mathbf{u}_{1:k}$, $\mathbf{z}_{1:k}$ | all inputs / all measurements from step 1 to $k$ | depends | [10.01](../10-localization/10.01-what-is-localization.md) |
| $u_{1:t}$ | odometry / control inputs up to time $t$ | varies | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $u_L$, $u_R$ | x pixel of the same point in the left / right image | px | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $u_R$ | duty sent to the right motor | — | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $\hat{u}_\alpha$ | unit vector in direction $\alpha$ | — | [06.08](../06-simulation/06.08-building-worlds.md) |
| $u_{db}$ | deadband: duty below which the wheel does not move | — | [08.02](../08-control/08.02-motor-step-response.md) |
| $u_{\text{ff}}$ | feed-forward duty that should produce $\omega^*$ | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $u_{ff}$ | feedforward command computed from the motor model | — | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $(u_i, v_i)$ | pixel coordinates of mask pixel $i$ | px | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $u_i, v_j$ | unit-length embeddings of anchors and candidates | — | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $\hat u_{ij}$ | detected pixel of corner $j$ in view $i$ | px | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $u_k$ | 1D control: distance driven since the last step (from encoders) | m | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $\mathbf{u}_k$ | control / odometry input moving the state from $k-1$ to $k$ | depends | [10.01](../10-localization/10.01-what-is-localization.md) |
| $u_{kj}$ | pixel where keyframe $k$ observed point $j$ | px | [11.08](../11-slam/11.08-visual-slam.md) |
| $u_{step}$ | size of the open-loop command step used for a fit | — | [08.08](../08-control/08.08-tuning-pid.md) |

### V

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\bar{V}$ | average voltage the motor sees under PWM | V | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $V$ | battery voltage | V | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md), [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $V$ | battery voltage measured now | V | [08.02](../08-control/08.02-motor-step-response.md), [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $V$ | number of grid cells (graph vertices) | — | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $V$ | part volume | cm³ | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $V$ | voltage | V | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $V$ | voltage (potential difference), sometimes written $U$ | V | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $V$ | voltage across a coil or load | V | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $V$ | voltage applied to the motor | V | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\mathcal{V}(k)$ | set of map points visible in keyframe $k$ | — | [11.08](../11-slam/11.08-visual-slam.md) |
| $V(t)$ | simulated battery voltage at time $t$ | V | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $V, \Lambda$ | eigenvector matrix and diagonal eigenvalue matrix ($\Sigma = V\Lambda V^\top$) | — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $V$, $\Lambda$ | eigenvector matrix and diagonal eigenvalue matrix of a covariance | —, m² | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $V^*$, $Q^*$ | optimal state and action values | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V^\pi(s)$ | state value: expected return from $s$ under $\pi$ | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V_0$ | battery voltage at start (full) | V | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $V_0$ | bus voltage before braking energy arrives | V | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_{ADC}$ | voltage at the ADC pin | V | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_{DD}$, $V_{CC}$ | logic supply voltage | V | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $V_F$ | forward voltage of a diode or LED | V | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $V_{IH}$ | input voltage guaranteed to read as 1 | V | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |
| $V_{IL}$ | input voltage guaranteed to read as 0 | V | [01.07](../01-first-robot/01.07-power-system.md), [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |
| $V_{IOVDD}$ | the Pico's I/O supply voltage | V | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $V_L$ | voltage across an inductor | V | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $V_M$ | motor supply voltage of the H-bridge | V | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_{OH}$ | output-high voltage a chip guarantees when driving 1 | V | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |
| $V_{OL}$ | output-low voltage a chip guarantees when driving 0 | V | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md), [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |
| $V_{PIN,FT}$ | maximum voltage on a 5 V-tolerant pin | V | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $V_{Pi}$ | voltage that actually arrives at the Raspberry Pi | V | [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) |
| $V_R$ | voltage measured across a resistor | V | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [FE.03](../optional-foundations/electronics/FE.03-multimeter.md) |
| $V_S$ | supply voltage of a chip (INA219) | V | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md) |
| $V_{SD}$ | body-diode forward voltage of a FET | V | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_{SLEEP}$ | voltage on the driver's SLEEP pin (sets VREF on karmel) | V | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $V_{VREF}$ | reference voltage on the VREF pin | V | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_\phi(s)$ | critic: learned state-value network | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $V_{avg}$ | average voltage of a PWM signal | V | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $V_{bat}$ | battery voltage | V | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_{\text{batt}}$ | battery voltage | V | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{batt}$ | battery voltage | V | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $V_{battery}$ | battery voltage at the regulator input | V | [01.01](../01-first-robot/01.01-plan-the-build.md) |
| $V_{be}$ | voltage of the Pico's internal temperature-sensing diode | V | [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md) |
| $V_{bus}$ | motor supply (bus) voltage | V | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $V_{\text{cell}}$ | nominal voltage of one cell | V | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_{crit}$ | critical battery threshold (stop now) | V | [04.09](../04-ros2/04.09-parameters.md) |
| $V_{\text{cutoff}}$ | voltage where the battery protection cuts off | V | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $V_d$, $\Omega_d$ | dynamic window: reachable linear / angular speeds in the next step | m/s, rad/s | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $V_\text{drop}$ | voltage lost across a bipolar driver | V | [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md) |
| $V_f$ | fault voltage on a pin | V | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $V_\text{floor}$ | lowest voltage the simulated battery falls to | V | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $V_{full}$ | voltage of a full pack | V | [04.09](../04-ros2/04.09-parameters.md), [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $V_{high}$ | voltage of the PWM "on" level | V | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $V_{in}$, $V_{out}$ | input / output voltage of a divider or regulator | V | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $V_{induced}$ | voltage induced in the signal pair | V | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $V_k$ | first $k$ principal directions | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $V_{load}$ | voltage the load actually sees | V | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $V_{load}$, $I_{load}$ | voltage / current under load | V, A | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $V_\text{low}$ | low-battery warning threshold | V | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $V_m$ | voltage across the motor | V | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $V_{min}, V_{max}$ | lowest and highest safe pack voltage | V | [04.09](../04-ros2/04.09-parameters.md) |
| $V_{nom}$ | nominal battery voltage | V | [02.02](../02-robot-electronics/02.02-power-budget.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $V_{nom}$ | nominal battery voltage at which motor constants were measured (10.8 V) | V | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $V_{\text{nom}}$ | nominal battery voltage the gain refers to | V | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $V_\text{nominal}$ | nominal pack voltage | V | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_{oc}$ | open-circuit (no-load) battery voltage | V | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $V_{offset}$ | ground offset caused by current in a shared wire | V | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_{\text{open circuit}}$ | battery voltage with no load | V | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $V_\text{pack}$ | pack voltage | V | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_{\text{pin}}$ | voltage at the ADC pin | V | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{rated}$ | voltage a solenoid's force rating is quoted at | V | [21.03](../21-war-machine/21.03-releasers-i-latches.md) |
| $V_{ref}$ | ADC reference voltage (full scale) | V | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{rest}$ | battery voltage at rest | V | [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_\text{shift}$ | ground shift: voltage across a ground wire carrying current | V | [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $V_{\text{shunt}}$ | voltage across the current-sense shunt resistor | V | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $V_{source}$ | voltage of the signal feeding a divider | V | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $V_{supply}$ | supply voltage | V | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) |
| $V_{\text{terminal}}$ | voltage at the battery terminals under load | V | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_{tri}$ | signed volume of the tetrahedron a triangle forms with the origin | mm³ | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $V_{true}$, $V_{adc}$ | true (meter) voltage / ADC-reported voltage | V | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $\mathbf{v}$ | a vector (bold = vector) | varies | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $v$ | forward (linear) speed of the robot | m/s | [04.03](../04-ros2/04.03-nodes-and-cli.md), [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v$ | forward (linear) speed of the robot or a wheel rim | m/s | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $v$ | forward speed command | m/s | [17.01](../17-reinforcement-learning/17.01-rl-framing.md), [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $v$ | forward speed of the robot (of base_link, midway between the wheels) | m/s | [08.10](../08-control/08.10-straight-line-heading-control.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $v$ | linear speed of the robot | m/s | [11.09](../11-slam/11.09-slam-troubleshooting.md), [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $v$ | momentum "velocity": running sum of past gradients | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $v$ | payload speed (muzzle or arrival) | m/s | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $v$ | rim speed of one wheel (how fast the tyre surface moves) | m/s | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $v$ | robot forward (linear) speed | m/s | [00.01](../00-orientation/00.01-what-is-a-robot.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $v$ | robot linear speed | m/s | [P01](../projects/P01-manual-drive.md) |
| $v$ | tip speed of the arm | m/s | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $\hat{\mathbf{v}}$ | unit vector: direction of $\mathbf{v}$ with length 1 (hat = unit length) | — | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $v = (v_x, v_y, v_z, \omega_x, \omega_y, \omega_z)$ | camera twist: linear and angular velocity | m/s, rad/s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\mathbf v$, $\mathbf t_k$ | CLIP image embedding / text embedding of phrase $k$ | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $\lVert\mathbf{v}\rVert$ | norm: Euclidean length of a vector | same as $\mathbf{v}$ | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $v_1, \dots, v_n$ | components of a vector | varies | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{v}_A, \mathbf{v}_C$ | the same vector written in frame A / frame C coordinates | varies | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $v_L$, $v_R$ | left / right wheel rim (ground) speed | m/s | [08.01](../08-control/08.01-open-vs-closed-loop.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $v_L$, $v_R$ | rim speed of the left / right wheel | m/s | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $v_{\max}$ | maximum safe speed for a given stopping distance and latency | m/s | [00.02](../00-orientation/00.02-the-robotics-stack.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $v_{\max}$ | velocity limit | rad/s (or °/s, m/s) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $v_{\max}$, $a_{\max}$ | speed limit and acceleration limit | m/s, m/s² | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $v_{\max}^{\text{wheel}}$ | top rim speed of a wheel | m/s | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $v_\theta$ | velocity-prediction network (flow matching) | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $v_c$ | commanded camera twist (velocity) | m/s, rad/s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $v_{cam}, \omega_{cam}$ | camera linear / angular velocity in the camera frame | m/s, rad/s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $v_h$ | image row of the horizon | px | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $v_k$ | measurement noise | m | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $v_k$ | speed sample number $k$ | m/s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $v_{kick}$ | recoil speed the robot picks up when it fires | m/s | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $v_{max}$ | cruise-speed limit of a motion profile | rad/s (or m/s) | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $v_\text{max}$ | highest speed that still stops inside the margin | m/s | [20.04](../20-final-robot/20.04-safety-case.md) |
| $v_\text{max}$ | maximum safe speed | m/s | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $v_{peak}$ | highest speed the profile reaches | rad/s (or m/s) | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $v_{peak}$ | top speed of a triangular profile (too short to reach $v_{max}$) | m/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v_{\text{tip}}$ | velocity of the arm tip | m/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $v_{\text{tool}}$, $v$ | linear velocity of the tool | m/s | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $v_w$ | rim speed of one wheel | m/s | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $v_x, v_y$ | velocity components along world $x$ and $y$ | m/s | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\mathrm{Var}(x)$ | variance (spread) of the inputs | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |

### W

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $W$ | firmware watchdog timeout | ms | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $W$ | image width | px | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $W$ | mechanical work done by an actuator | J | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $W$ | number of weeks to finish the course | — | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |
| $W$ | weight $mg$ | N | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $W$ | weight matrix $\operatorname{diag}(1/\sigma_i^2)$ for weighted least squares | 1/unit² | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $W$ | weight matrix of a layer, shape (out × in) | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $W$, $D$, $H$ | width, depth and height of the area a map covers | m | [11.01](../11-slam/11.01-map-representations.md) |
| $W, H$ | image width and height | px | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $W$, $W_{out}$ | input / output width of a convolution layer | px | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $W_1, b_1, W_2, b_2$ | weights and biases of layers 1 and 2 of an MLP | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $W_Q, W_K, W_V, W_O$ | projection matrices for queries, keys, values and output | — | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $W_{\text{in}}, W_{\text{out}}$ | input and output width of a conv layer | px | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $w$ | clear width of a doorway | m | [P11](../projects/P11-autonomous-navigation.md) |
| $w$ | door width | m | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $w$ | fraction of matches that are inliers | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $w$ | homogeneous coordinate: 1 for a point (translation applies), 0 for a direction | — | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $w$ | homogeneous coordinate: 1 for points, 0 for directions | — | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $w$ | image width | px | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $w$ | image width (in the focal-length formula) | px | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $w$ | loop body (work) time | s | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $w$ | object width across the jaw | m | [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md) |
| $w$ | particle weight | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $w$ | RC pulse width | µs | [21.05](../21-war-machine/21.05-triggers-remote.md) |
| $w$ | spacing of the signal pair (loop width) | m | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $w$ | third homogeneous coordinate ($w = 0$ is a point at infinity) | — | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $w$ | time one callback takes to run (work time) | s | [04.11](../04-ros2/04.11-qos.md), [04.12](../04-ros2/04.12-executors-and-callbacks.md) |
| $w$ | waste fraction (supports, purge, failed prints) | — | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $w$ | weight converting radians to metres in the residual | m/rad | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $w$ | weight on the heuristic in weighted A* ($w > 1$) | — | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $w$ | weight trading safety (cost) against distance | — | [12.01](../12-navigation/12.01-the-navigation-problem.md), [12.04](../12-navigation/12.04-costmaps.md) |
| $w$ | weight vector of a linear model | varies | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $w$ | width of an object | m | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $w$ | Yoshikawa manipulability: volume of the velocity ellipse, how agile the arm is at a pose | mⁿ/radⁿ | [14.06](../14-robotic-arm/14.06-jacobian.md), [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $w$ (quaternion) | scalar part: $\cos(\theta/2)$, how much it rotates | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $w = (f_x, f_y, \tau)$ | planar wrench: force at a contact plus its torque about the centre of mass | N, N, N·m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $w(i, j)$ | convolution kernel weight at offset $(i,j)$ | — | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md), [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $w$, $h$ | image width and height sent to a VLM | px | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $w^{(m)}$ | softmax weight of sample $m$ | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $w^*$ | least-squares weights from the normal equations | varies | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $w_1$, $w_2$ | fusion weights of the two measurements | — | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $w_1, w_{\text{gyro}}$ | weight given to a measurement when fusing | — | [07.06](../07-sensors/07.06-imu-in-ros2.md) |
| $w_\cap$, $h_\cap$ | width / height of the overlap of two boxes | px | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $w_{avg}$, $w_{slow}$, $w_{fast}$ | average particle weight and its slow / fast running averages (augmented MCL) | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $w_d$ | disturbance wrench the grasp must resist | N, N·m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $w_i$ | temporal-ensembling weight of prediction $i$ | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $w_i$ | weight of particle $i$ (all weights sum to 1) | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $w_k$ | MPPI weight of sample $k$ | — | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $w_k$ | process noise: random error of the motion | m | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $w_k, t_k$ | weight and normalised value of score term $k$ in the grasp score | — | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $\mathbf{w}_k$, $\mathbf{v}_k$ | process-noise and measurement-noise vectors | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $w_p, w_g, w_o, w_e$ | local-planner weights for path distance, goal distance, obstacles, effort | — | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $w_{px}$ | width of the tag in the image | px | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $\mathrm{wrap}(\cdot)$ | wrap an angle into $(-\pi, \pi]$ | rad | [08.10](../08-control/08.10-straight-line-heading-control.md), [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\operatorname{wrap}(\theta)$ | the angle brought into the range $(-\pi, \pi]$ | rad | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\operatorname{wrap}(a)$ | angle $a$ folded into $(-\pi, \pi]$ | rad | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |

### X

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $X$ | feature matrix: one example per row, last column of ones for the bias | varies | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $X$ | hand–eye transform, the unknown: $T_{\text{gripper,cam}}$ (eye-in-hand) or $T_{\text{base,cam}}$ (eye-to-hand) | — | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $X$ | random variable: a number that depends on a random outcome | varies | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $X, Y$ | point coordinates in the camera optical frame (right, down) | m | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $X, Y, Z$ | 3D point in the camera optical frame ($Z$ = depth forward) | m | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $X, Y, Z$ / $X_n, Y_n, Z_n$ | CIE XYZ color and the reference white | — | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $X_{\text{hand-eye}}$ | hand–eye calibration result from 15.03 | — | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $X_i$, $Z_{ij}$ | pose of node $i$ / measurement $z_{ij}$ written as a transform matrix | — | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $X_j$ | 3D map point $j$ | m | [11.08](../11-slam/11.08-visual-slam.md) |
| $\bar x$ | data mean | unit of $x$ | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $x$ | features: the inputs a model sees for one example | varies | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\hat{x}$ | fused estimate from two measurements | same as measurement | [07.06](../07-sensors/07.06-imu-in-ros2.md) |
| $x$ | hidden state being estimated (door open/closed, robot cell, wall distance) | varies | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $\mathbf{x}$ | Kalman state of a box: center, size and their velocities | px, px/frame | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\hat{\mathbf{x}}$ | least-squares estimate of the unknowns | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\bar x$ | mean of the readings | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\hat{\mathbf{x}}$ | normalized image point $K^{-1}\mathbf{x}$ | — | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $x$ | real (float) value a quantized number represents | varies | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $x$ | remaining distance to the dock | m | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\bar{x}$ | sample mean | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $x$ | spring stretch (draw length) | m | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $x$ | state estimate (1D: distance to the wall) | m | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $x$ | state of an ODE / state-space system | mixed | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md), [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\mathbf{x}$ | vector of unknowns in $A\mathbf{x} = \mathbf{b}$ | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $x(t)$ | position at time $t$ | m | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\bar x$, $\bar P$ | predicted state and its variance (prior, before the update) | m, m² | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $\dot x, \ddot x, \dddot x$ | first, second, third time derivative of $x$ (velocity, acceleration, jerk) | unit/s, /s², /s³ | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\dot x$, $\dot y$, $\dot\theta$ | time derivatives of the pose (world-frame velocity and yaw rate) | m/s, m/s, rad/s | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\mathbf{x}$, $\bar{\mathbf{x}}$ | state vector estimate (posterior) and predicted state (prior) | mixed | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $x$, $x_t$, $x_{1:t}$ | robot pose (at time $t$; the whole trajectory up to $t$) | m, rad | [11.02](../11-slam/11.02-occupancy-grid-mapping.md), [11.03](../11-slam/11.03-the-slam-problem.md) |
| $x, y$ | normalised image coordinates (lens-independent) | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $x$, $y$ | normalized image coordinates $X/Z$, $Y/Z$ | — | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $x$, $y$ | robot position in the world | m | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $x$, $y$ | robot position in the world (odom/map) frame | m | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $(x, y)$ | Cartesian coordinates: $x$ forward, $y$ left (REP-103) | m | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $(x, y, \theta)$ | 2D pose: position plus heading | m, m, rad | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $x, y, z$ | box dimensions (when used in inertia formulas) | m | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $\hat{\mathbf{x}}, \hat{\mathbf{y}}, \hat{\mathbf{z}}$ | standard basis vectors (unit arrows along the axes) | — | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $x, y, z$ (quaternion) | vector part: rotation axis times $\sin(\theta/2)$ | — | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $(x, y, z)$ | position of the tool in the base frame | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\mathbf{x}^{(i)}$, $\mathbf{x}_i$ | pose of particle $i$ | m, m, rad | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $x^*$ | best (optimal) set of poses found by the optimizer | m, rad | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $x^*$ | desired task coordinates, e.g. $(x, y, z, \text{pitch})$ | m, rad | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $x_0, v_0$ | initial position and initial speed | m, m/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $(x_0, y_0)$, $(x_1, y_1)$ | start and end cell of a LiDAR beam on the grid | cells | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $x_{\min}, x_{\max}$ | measured range of a tensor during calibration | varies | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $x_\tau$ | point on the noise-to-data path at time $\tau$ | unit of action | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $x_a$, $x_c$, $x_m$ | position of the axle, the caster, the centre of mass along the robot | m | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $x_c$ | horizontal distance from the tipping edge to the centre of mass | m | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $x_c$, $x'_c$ | raw / normalized network input of channel $c$ | 0–255, — | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $x_c, z_c$ | centre of mass forward position (from the axle) / height | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $x_\text{caster}$ | forward position of the ball caster (front pivot) | m | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $x_{caster}, x_{axle}$ | forward position of the caster / drive axle | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $x_\text{cg}$ | horizontal position of the centre of gravity, measured forward from the wheel axle | m | [20.02](../20-final-robot/20.02-hardware-integration.md), [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $x_{cg}$, $y_{cg}$, $r_{cg}$ | centre of gravity of a cluster of end errors, and its distance from zero | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $x_d$, $y_d$ | distorted normalized coordinates | — | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $x_g, y_g$ | centroid of the object's blob in the image | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $x_i$ | audio samples (full scale = 1) | — | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| $x_i$ | forward position of part $i$ | m | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $x_i$ | one audio sample | — (full scale) | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $x_i$ | regressor: effective drive of sample $i$ | — | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $x_i$ | the $i$-th reading in a set of measurements | same as reading | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $x_i = (x, y, \theta)$ | pose of keyframe (graph node) $i$ in the map frame | m, rad | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $x_i^+$ | positive partner of example $x_i$ (e.g. same place, other view) | — | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $x_k$ | raw input sample number $k$ to a filter | same as signal | [04.09](../04-ros2/04.09-parameters.md) |
| $x_k$ | raw input sample to a filter | signal unit | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\mathbf{x}_k$ | robot state (pose $(x, y, \theta)$) at step $k$ | m, m, rad | [10.01](../10-localization/10.01-what-is-localization.md) |
| $x_r, y_r$ | lookahead point in the robot frame | m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $(x_r, y_r)$ | carrot (lookahead point) in the robot's `base_link` frame | m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $(x_s, y_s, \theta_s)$ | pose of the LiDAR in the map frame | m, rad | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\mathbf{x}_{\text{true}}$ | true robot pose (from tape measure) | m | [P10](../projects/P10-localization.md) |
| $x_{zmp}$ | zero-moment point: where the floor effectively pushes back when accelerating | m | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |

### Y

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $Y$ | grayscale brightness (luma) | 0–255 | [13.01](../13-computer-vision/13.01-images-as-data.md) |
| $Y$ | Lewis form factor of a tooth shape | — | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $Y_{\text{EOL}}$, $M_{\text{EOL}}$ | year and month support ends | year, month | [FL.01](../optional-foundations/linux-and-tools/FL.01-why-linux.md) |
| $Y_{\text{now}}$, $M_{\text{now}}$ | current year and month | year, month | [FL.01](../optional-foundations/linux-and-tools/FL.01-why-linux.md) |
| $Y_{o,i,j}$ | conv output for channel $o$ at position $(i, j)$ | — | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $y$ | label: the correct answer for an example | varies | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\bar y$ | mean of the labels | unit of $y$ | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $y$ | output of the plant: the true controlled value (e.g. true wheel speed) | rad/s (or the controlled quantity's unit) | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $\hat y$ | the model's prediction of $y$ | same as $y$ | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\dot y$, $\ddot y$ | first / second time derivative of the output | unit/s, unit/s² | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $y_i$ | regression target for transition $i$ | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $y_k$ | filtered output after sample $k$ | same as signal | [04.09](../04-ros2/04.09-parameters.md) |
| $y_k$ | filtered output of an EMA / low-pass filter | signal unit | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\dot y_k$ | filtered rate of change of the measurement | rad/s² | [08.06](../08-control/08.06-derivative-control.md) |
| $y_m$ | measured output, as the sensor reports it | same as $y$ | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $\dot y_{raw}$ | unfiltered rate of change of the measurement | rad/s² | [08.06](../08-control/08.06-derivative-control.md) |

### Z

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $Z$ | depth of a point along the camera's optical axis | m | [11.08](../11-slam/11.08-visual-slam.md), [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [15.04](../15-manipulation/15.04-object-pose-estimation.md), [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $Z$ | distance to the object along the optical axis | m | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $Z^{*}$ | depth at the goal, used as the depth estimate | m | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $Z_1, H_1$ | batch pre-activations and activations of layer 1 (one row per sample) | — | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $Z_{min}$ | minimum measurable stereo depth | m | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $z$ | Cartesian coordinate pointing up | m | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $z$ | compressed code (PCA or autoencoder bottleneck) | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $z$ | CVAE "style" latent (set to 0 at run time) | — | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $z$ | discrete-time pole (stable if inside the unit circle) | — | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $z$ | latent sample of a VAE | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $z$ | measurement | varies | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $z$ | normal quantile (1.96 for 95%) | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md), [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| $z$ | number of gear teeth | — | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $z$ | number of standard deviations away from the mean (z-score) | — | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $z$ | pre-activation: $Wx + b$ before the nonlinearity | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\hat z$ | predicted measurement of a candidate landmark | m | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $z$ | quantization zero point: the integer that stands for 0.0 | — | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $\hat z$ | unit vector out of the plane (planar Jacobian) | — | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\mathbf{z}$ | vector of independent standard normal samples | — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $z$ | z-score: how many σ a value is from the mean | — | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $z$, $z_t$, $z_{1:t}$ | sensor measurement (at time $t$; all measurements up to $t$) | varies | [11.02](../11-slam/11.02-occupancy-grid-mapping.md), [11.03](../11-slam/11.03-the-slam-problem.md) |
| $z_1$, $z_2$ | two measurements of the same quantity | rad/s | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $z_{\alpha/2}, z_\beta$ | normal quantiles for significance and power | — | [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| $z_{\min}$ | closest depth a stereo camera can measure | m | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $z_\text{cg}$ | height of the centre of gravity above the floor | m | [20.02](../20-final-robot/20.02-hardware-integration.md), [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $z_{hit}$, $z_{rand}$ | mixture weights: beam hits the map vs random reading | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $z_i$ | height of part $i$ | m | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $z_i$ | logit (raw network score) of class $i$ | — | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $z_i$ | robust z-score of reading $i$ (outlier test) | — | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $z_i$ | unit axis of joint $i$, expressed in the base frame | — | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $z_{ij}$ | measured pose of node $j$ seen from node $i$ | m, rad | [11.03](../11-slam/11.03-the-slam-problem.md), [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\mathbf{z}_k$ | detector measurement at frame $k$ | px | [13.12](../13-computer-vision/13.12-tracking.md) |
| $z_k$ | logit: the raw score for class $k$ before softmax | — | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\mathbf{z}_k$ | measurement at step $k$ | depends | [10.01](../10-localization/10.01-what-is-localization.md) |
| $z_{max}$ | maximum sensor range | m | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $z_{min}$ | fewest teeth before a 20° gear is undercut | — | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $z_{table}$ | height of the table top in the base frame | m | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\hat z_{tag}$ | unit vector of the tag's z axis (out of the paper) | — | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |

### Greek: alpha

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\alpha$ | angle of the ellipse's major axis | rad | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\alpha$ | angular acceleration | rad/s² | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md), [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\alpha$ | Borenstein: turn error per 90° corner (wheelbase error) | rad | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\alpha$ | complementary-filter weight on the gyro (1 = gyro only, 0 = wheels only) | — | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\alpha$ | derivative-filter factor $\Delta t/(\tau_d + \Delta t)$ | — | [08.06](../08-control/08.06-derivative-control.md) |
| $\alpha$ | direction of the test displacement | rad | [06.08](../06-simulation/06.08-building-worlds.md) |
| $\alpha$ | EMA smoothing factor: fraction moved toward each new sample | — | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\alpha$ | first-order low-pass coefficient of the derivative filter | — | [08.06](../08-control/08.06-derivative-control.md) |
| $\alpha$ | fraction of the remaining speed gap closed in one step | — | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\alpha$ | friction-cone half-angle | rad | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\alpha$ | full beam (cone) angle of a range sensor | rad | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $\alpha$ | mixture temperature (1 = proportional to size, 0 = uniform) | — | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $\alpha$ | orientation of the blob in the image | rad | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\alpha$ | per-sample fraction of the gap closed by a first-order system, $1 - e^{-\Delta t/\tau}$ | — | [08.02](../08-control/08.02-motor-step-response.md) |
| $\alpha$ | SAC entropy temperature | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\alpha$ | significance level: accepted false-positive rate of a test | — | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md), [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\alpha$ | smoothing factor of an exponential moving-average (low-pass) filter, 0 to 1 | — | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md), [04.09](../04-ros2/04.09-parameters.md) |
| $\alpha$ | step size (learning rate) of a value update | — | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\alpha$ | upward tilt of the sensor | ° | [F3D.05](../optional-foundations/3d-printing-and-cad/F3D.05-mounting-bracket.md) |
| $\alpha_i, \beta_i$ | axis-angle vectors of $R_{A_i}$ and $R_{B_i}$ | rad | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $\alpha_k$ | angle of LiDAR beam $k$ relative to the sensor | rad | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\alpha_{\text{ROI}}$ | field of view of a reduced region of interest | ° | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\alpha_{slow}$, $\alpha_{fast}$ | smoothing rates of those running averages | — | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\alpha_t$ | $1 - \beta_t$ | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $\bar\alpha_t$ | product of $\alpha_s$ up to $t$: fraction of the clean signal left | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |

### Greek: beta

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\beta$ | accepted rate of missing a truly good policy | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\beta$ | bearing of a pixel column from the optical axis | rad | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\beta$ | bearing to a landmark relative to the robot's nose (positive = left) | rad | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\beta$ | bearing: direction to a point or goal | rad | [05.01](../05-frames-and-transforms/05.01-why-frames.md), [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\beta$ | Borenstein: heading gained per straight leg (diameter error) | rad | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\beta$ | known tilt angle used to measure CoM height | rad | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $\beta$ | momentum coefficient (how much past gradient is kept) | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\beta$ | weight of the KL term in a VAE / CVAE loss | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $\beta_1, \beta_2$ | Adam's decay rates for $m$ and $v$ (typically 0.9, 0.999) | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\beta_k, \alpha_k, \bar\alpha_k$ | noise schedule, $1 - \beta_k$, and cumulative product | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\beta_t$ | diffusion noise schedule: noise added at step $t$ | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |

### Greek: gamma

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\gamma$ | discount factor: how much a future reward counts | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md), [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $\gamma$ | RL discount factor | — | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\gamma$ | RRT* radius constant | m or rad | [12.03](../12-navigation/12.03-sampling-based-planning.md) |

### Greek: delta

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\Delta$ | a relative motion (pose increment) applied to a transform | m, rad | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $\Delta$ | difference in success rate to detect | — | [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| $\delta$ | Gauss–Newton parameter update step | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\delta$ | Huber loss threshold between quadratic and linear regions | unit of $y$ | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\delta$ | jitter: how much longer a loop actually ran than planned | s | [00.04](../00-orientation/00.04-robot-computers.md) |
| $\Delta$ | quantization step (one tick / ADC count); also gap between outlier value and normal mean | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\Delta$ | sample spacing along a LiDAR beam | m | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |
| $\Delta$ | sideways displacement of a point caused by heading error | m | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\delta$ | small test displacement | m | [06.08](../06-simulation/06.08-building-worlds.md) |
| $\delta$ | sub-pixel disparity offset from a parabola fit | px | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\delta$ | TD error: target minus current estimate | — | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\Delta$ | tick change, corrected for counter wrap-around | ticks | [FPY.01](../optional-foundations/python/FPY.01-python-for-csharp-devs.md) |
| $\boldsymbol\Delta$ | update step for the parameters in Gauss-Newton / LM | as $\mathbf{x}$ | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\Delta d$ | disparity matching error | px | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $\Delta d$ | distance error caused by a radius error | m | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\Delta d_L$, $\Delta d_R$ | wheel distance change in one step (08.10 notation) | m | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\Delta I$ | peak-to-peak current ripple during PWM | A | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $\Delta I_{pp}$ | peak-to-peak current ripple during PWM | A | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\Delta L$ | change in length from shrinkage | mm | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $\Delta n$ | number of ticks counted in one sample window | — | [00.04](../00-orientation/00.04-robot-computers.md), [01.09](../01-first-robot/01.09-reading-encoders.md), [03.05](../03-robot-software/03.05-hardware-abstraction.md) |
| $\Delta n$ | tick-count difference (between wheels, or between two samples) | ticks | [08.01](../08-control/08.01-open-vs-closed-loop.md), [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $\Delta n^{L}_k$, $\Delta n^{R}_k$ | left / right tick deltas at step $k$ | — | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $\Delta p$ | change in momentum | kg·m/s | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $\Delta p$ | gauge vacuum: how far below atmosphere the pump pulls | Pa | [15.02](../15-manipulation/15.02-grippers.md) |
| $\Delta p$ | small pose change $(\Delta x, \Delta y, \Delta\theta)$ fed into a Jacobian (p for pose) | m, m, rad | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta q$ | joint step (IK) or joint error (error budget) | rad | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md), [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $\Delta r$ | size of a setpoint step | rad/s | [08.06](../08-control/08.06-derivative-control.md) |
| $\Delta r$, $\bar r$ | radius difference between the wheels, and their mean radius | m | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\Delta r$, $\Delta c$ | row and column differences between two cells | cells | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $\Delta s$ | distance driven by base_link in one step | m | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\Delta s$ | distance per encoder count | m | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $\Delta s$ | distance the robot centre moves in one odometry step | m | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $\Delta s_{tick}$ | distance the wheel rim moves per encoder tick | m | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\Delta t$ | control period | s | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\Delta t$ | control-loop / sample period | s | [08.02](../08-control/08.02-motor-step-response.md) |
| $\Delta t$ | latency between image capture and transform lookup | s | [P13](../projects/P13-robot-plus-vision.md) |
| $\Delta T$ | relative motion between two poses, as a transform | — / m | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $\Delta t$ | sample period of a sensor | s | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $\Delta t$ | sample window / time step | s | [01.09](../01-first-robot/01.09-reading-encoders.md), [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\Delta T$ | temperature margin of a material | °C | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $\Delta T$ | temperature rise | K | [01.03](../01-first-robot/01.03-workbench-and-tools.md), [FE.17](../optional-foundations/electronics/FE.17-soldering.md) |
| $\Delta t$ | time offset (latency) between a detection and the transform used | s | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $\Delta t$ | time step / control period | s | [04.08](../04-ros2/04.08-actions.md), [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\Delta t$ | time step between updates | s | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\delta t$ | timestamp skew between a scan and the pose used to place it | s | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\Delta u_k$ | change of the command in one period (velocity-form PID) | — | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\Delta v$ | same speed resolution expressed as ground speed | m/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta V$ | voltage drop or droop | V | [01.07](../01-first-robot/01.07-power-system.md), [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $\Delta V$ | voltage lost in a wire run (out and back) | V | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $\Delta v_{min}$ | speed resolution of encoder-based velocity | m/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\Delta x$ | correction applied to all poses in one Gauss-Newton step | m, rad | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\Delta x, \Delta y$ | displacement along x and y during a motion step | m | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $\Delta x, \Delta y$ | offset from robot to goal along $x$ / $y$ | m | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\Delta x, \Delta y, \Delta\theta$ | small error (or nudge) in the robot pose, input to a Jacobian | m, m, rad | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\delta y$ | lateral position error | m | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\Delta z$ | depth error | m | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $\Delta\omega$ | encoder speed resolution: speed that one tick per interval stands for | rad/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta\omega$ | smallest change in wheel speed that can be measured | rad/s | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\Delta\omega$ | speed resolution, the speed of one tick per sample | rad/s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\Delta\omega$ | speed resolution: smallest non-zero change of the estimate (one tick) | rad/s | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\Delta\omega$ | speed resolution: smallest speed change one tick can show | rad/s | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\Delta\omega_{min}$ | speed resolution at sample rate $f_s$ | rad/s | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $\Delta\omega_{pp}$ | peak-to-peak speed ripple caused by PWM | — (fraction of full speed) | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\Delta\psi$ | heading error accumulated from gyro drift | rad | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $\Delta\theta$ | angle between two orientations | rad | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\Delta\theta$ | angular step between two LiDAR beams | rad | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [07.07](../07-sensors/07.07-2d-lidar.md) |
| $\Delta\theta$ | bearing error caused by latency | rad | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $\Delta\theta$ | change in robot heading | rad | [01.06](../01-first-robot/01.06-mechanical-assembly.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\Delta\theta$ | heading change in one odometry step | rad | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $\Delta\theta$ | heading change in one step | rad | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\delta\theta$ | heading error | rad | [09.06](../09-odometry/09.06-accumulated-error.md), [10.01](../10-localization/10.01-what-is-localization.md) |
| $\delta\theta$ | heading error (or small rotation) | rad | [11.03](../11-slam/11.03-the-slam-problem.md), [11.04](../11-slam/11.04-scan-matching-icp.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\Delta\theta$ | rotation between consecutive keyframes | rad | [11.06](../11-slam/11.06-slam-toolbox-simulation.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\Delta\theta$ | small angle step: one encoder tick, one LiDAR beam spacing, or a heading error | rad | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md), [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\Delta_{dB}$ | how far a sound event is above the background | dB | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $\delta_t$ | one-step TD error at step $t$ | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\delta_{total}$ | total toe angle of the two wheels | rad | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\delta_x$, $\delta_y$ | landmark position relative to the robot, $l_x - x$, $l_y - y$ | m | [10.06](../10-localization/10.06-extended-kalman-filter.md) |

### Greek: epsilon

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\epsilon$ | floating-point machine precision (~1e-16) | — | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\epsilon$ | Gaussian noise sample (flow-matching start point) | unit of action | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $\epsilon$ | heading error made at one corner | rad | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\epsilon$ | how late each relative sleep wakes up | ms | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $\bar\epsilon$ | NEES averaged over $N$ Monte Carlo runs | — | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\epsilon$ | NEES: normalized estimation error squared (needs ground truth) | — | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\epsilon$ | PPO clip range (e.g. 0.2) | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\epsilon$ | probability of an unrecoverable mistake per step / decision | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md), [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $\epsilon$ | small nudge used for a numerical (finite-difference) gradient | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\epsilon$ | threshold on $\sigma_{\min}$ below which variable damping kicks in | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\epsilon$ | tiny constant to avoid division by zero | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\epsilon^{(m)}$ | random perturbation of the control sequence for sample $m$ | m/s, rad/s | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\epsilon_\nu$ | NIS: normalized innovation squared (works on the real robot) | — | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\epsilon_\theta$ | noise-prediction network | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $\varepsilon$ | average oversleep of `sleep()` | s | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\varepsilon$ | exploration rate of ε-greedy | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $\varepsilon$ | Ferrari–Canny grasp quality: radius of the largest ball around the origin inside the grasp wrench hull | — | [15.01](../15-manipulation/15.01-physics-of-grasping.md), [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $\varepsilon$ | random measurement noise added to a reading | same as reading | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $\varepsilon$ | random noise added to a measurement, $\varepsilon \sim \mathcal N(0, \sigma^2)$ | unit of $y$ | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\varepsilon$ | small probability mixed into every cell to allow recovery from kidnapping | — | [10.03](../10-localization/10.03-bayes-filter.md) |
| $\varepsilon$ | small step for numerical (finite-difference) Jacobians | state unit | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\varepsilon_\theta$ | noise-prediction network of a diffusion policy | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\varepsilon_k$ | random noise added to the control sequence for sample $k$ | m/s, rad/s | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\varepsilon_x$, $\varepsilon_y$ | UMBmark return-position error, measured minus odometry | m | [09.05](../09-odometry/09.05-calibrating-odometry.md) |

### Greek: zeta

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\zeta$ | damping ratio of a second-order response | — | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\zeta$ | damping ratio of a second-order system (1 = critically damped, no overshoot) | — | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |

### Greek: eta

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\eta$ | efficiency (output power / input power) | — | [01.01](../01-first-robot/01.01-plan-the-build.md), [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $\eta$ | efficiency of the linkage or the suction cup | — | [15.02](../15-manipulation/15.02-grippers.md) |
| $\eta$ | efficiency: fraction of power that comes out | — | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $\eta$ | fraction of braking energy returned to the bus | — | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\eta$ | gearbox efficiency | — | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\eta$ | learning rate (step size) of gradient descent | varies | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\eta$ | learning rate: step size of gradient descent | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\eta$ | normaliser that makes the belief sum (integrate) to 1 | — | [10.01](../10-localization/10.01-what-is-localization.md), [10.03](../10-localization/10.03-bayes-filter.md) |

### Greek: theta

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\theta$ | an angle; robot heading (yaw in 2D), measured from $+x$, counter-clockwise positive | rad | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\theta$ | angle between two vectors | rad | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\theta$ | angle of incidence of a sound beam on a wall | rad | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $\theta$ | azimuth: bearing of a point from $+x$, counter-clockwise positive | rad | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\theta$ | camera pitch | rad | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\bar\theta$ | circular mean of several angles | rad | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\hat\theta$ | circular-mean heading of the particle cloud | rad | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\theta$ | corner angle | rad | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $\theta$ | dead time: total loop delay (sampling + filter + transport) | s | [08.07](../08-control/08.07-pid-mathematics.md), [08.08](../08-control/08.08-tuning-pid.md) |
| $\theta$ | gradient direction (edge normal) | rad | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\theta$ | heading (yaw) of the robot in the plane | rad | [04.03](../04-ros2/04.03-nodes-and-cli.md), [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\theta$ | in-place rotation angle of the robot | rad | [P03](../projects/P03-encoder-movement.md) |
| $\theta$ | launch angle above horizontal | rad | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\theta$ | model parameters ("weights"), the numbers training adjusts | — | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md), [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\theta$ | object or jaw yaw (an axis: $\theta$ and $\theta + 180°$ are the same) | rad | [15.04](../15-manipulation/15.04-object-pose-estimation.md), [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $\theta$ | pitch: rotation about the left $y$ axis (positive = nose down) | rad | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $\theta$ | robot heading (or commanded turn angle) | rad | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md), [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\theta$ | robot heading (yaw), wrapped to $(-\pi, \pi]$ | rad | [08.10](../08-control/08.10-straight-line-heading-control.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\theta$ | rotation angle about an axis (axis-angle form) | rad | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $\theta$ | rotation angle between two scans | rad | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $\theta$ | slope angle of a ramp or tilt angle of a body | rad | [06.03](../06-simulation/06.03-gazebo-basics.md), [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $\theta$ | taper angle of the gripped faces, or the tilt of a ramp at which the object slides | rad | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\theta(t)$ | wheel angle (integral of wheel speed) | rad | [08.02](../08-control/08.02-motor-step-response.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\theta/\tau$ | controllability ratio: how delay-dominated the plant is | — | [08.08](../08-control/08.08-tuning-pid.md) |
| $\theta^*$ | the parameters with the lowest training loss | — | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\theta^-$ | weights of the frozen target network | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $\theta_1, \dots, \theta_6$ | joint angles of an arm | rad | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $\theta_{\min}$ | angle of the first beam (`angle_{min}`) | rad | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\theta_{\text{deg}}$ | joint angle in degrees as LeRobot reports it | ° | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $\theta_{FoV}$ | full field-of-view angle of a range sensor | ° | [F3D.05](../optional-foundations/3d-printing-and-cad/F3D.05-mounting-bracket.md) |
| $\theta_i$ | angle of lidar beam $i$ | rad | [07.07](../07-sensors/07.07-2d-lidar.md), [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $\theta_i$ | angle of LiDAR beam number $i$ | rad | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\theta_i, d_i, a_i, \alpha_i$ | Denavit–Hartenberg parameters of joint $i$: rotation about z, offset along z, length along x, twist about x | rad, m, m, rad | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\theta_m$ | heading at the middle of the step (midpoint model) | rad | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\theta_m$, $\mu$ | midpoint heading of a step, $\theta + \Delta\theta/2$ | rad | [09.06](../09-odometry/09.06-accumulated-error.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\theta_{\text{measured}}$ | turn angle actually measured on the floor | ° | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\theta_{profile}(t)$, $\dot\theta_{profile}(t)$ | profile's planned heading change and yaw rate at time $t$ | rad, rad/s | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\theta_r$ | wheel-angle setpoint | rad | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\theta_{\text{rad}}, \theta_{\text{deg}}$ | the same angle in radians / in degrees | rad / ° | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\theta_{slip}$ | ramp angle where an object starts to slide | rad | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\theta_{sp}$, $\theta_{target}$ | heading setpoint | rad | [08.10](../08-control/08.10-straight-line-heading-control.md), [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\theta_{start}$ | heading when the turn began | rad | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\theta_t, \theta_c$ | target heading / current heading | rad | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\theta_{transport}$ | transport delay (communication, e.g. USB or Wi-Fi) | s | [08.07](../08-control/08.07-pid-mathematics.md) |

### Greek: kappa

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\kappa$ | curvature of the pure-pursuit arc | 1/m | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\kappa$ | path curvature, $1/R$ | 1/m | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md), [08.10](../08-control/08.10-straight-line-heading-control.md), [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\kappa$ | systematic heading error per metre driven (from unequal wheels) | rad/m | [09.06](../09-odometry/09.06-accumulated-error.md), [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\kappa(A)$ | condition number: how much noise in $\mathbf{b}$ can be amplified in $\mathbf{x}$ | — | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\kappa(o)$ | motion kernel: probability the robot really moved offset $o$ cells | — | [10.03](../10-localization/10.03-bayes-filter.md) |

### Greek: lambda

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\lambda$ | damping factor (damped least squares / Levenberg-Marquardt) | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md), [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\lambda$ | damping of damped least squares (smallest singular value you will invert honestly) | m/rad | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md), [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\lambda$ | GAE parameter: trades bias against variance | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\lambda$ | MPPI temperature: how strongly low-cost samples dominate | — | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\lambda$ | MPPI temperature: low = pick the best, high = average everything | — | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\Lambda$ | running log-likelihood ratio of the SPRT | — | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\lambda$ | visual-servo gain | 1/s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\lambda_1$, $\lambda_2$ | eigenvalues (of the structure tensor, or of a covariance) | varies | [13.04](../13-computer-vision/13.04-features-and-matching.md), [13.11](../13-computer-vision/13.11-segmentation.md) |
| $\lambda_1$, $\lambda_2$ | eigenvalues of a 2×2 covariance: variances along the ellipse axes | m² | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\lambda_{\max}$ | largest eigenvalue of the Hessian (steepest curvature) | — | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\lambda_{\max}$ | maximum damping reached at the singularity (variable damping) | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\lambda_\text{false}$ | expected false fires per hour | 1/h | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $\lambda_i, \mathbf{v}_i$ | eigenvalues / eigenvectors of $\Sigma$ (ellipse axis variances and directions) | units², — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\lambda_k, v_k$ | eigenvalues / eigenvectors of $C$ (spread and direction of the footprint) | m², — | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\lambda_\text{max}, \lambda_\text{min}$ | largest / smallest curvature (eigenvalue of $H$) | varies | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |

### Greek: mu

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\mu$ | coefficient of friction of a pair of materials | — | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\mu$ | damping inside $L^{+}$ | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\mu$ | friction coefficient | — | [06.03](../06-simulation/06.03-gazebo-basics.md), [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\boldsymbol\mu$ | mean of a Gaussian | m | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\boldsymbol\mu$ | mean vector | as $\mathbf{x}$ | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\boldsymbol{\mu}$ | pose estimate (mean) reported by the filter | m | [P10](../projects/P10-localization.md) |
| $\mu$ | true mean of a distribution | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\mu(x), \sigma(x)$ | VAE encoder outputs: mean and spread of the latent | — | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $\mu_0$ | magnetic constant (permeability of free space) | T·m/A | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\mu_0$ | permeability of free space ($4\pi \times 10^{-7}$) | H/m | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $\mu_0$, $\mu_1$ | mean intensity of pixels below / above $t$ | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $\mu_{20}, \mu_{02}, \mu_{11}$ | second-order central image moments of the blob | — | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\mu_\theta(s)$, $\sigma_\theta$ | mean and standard deviation of a Gaussian policy | unit of action | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\mu_c$, $\sigma_c$ | ImageNet mean / standard deviation of channel $c$ | — | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $\mu_d^{(k)}, \sigma_d^{(k)}$ | mean and std of action dimension $d$ in dataset $k$ | unit of action | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $\mu_j$ | centre of cluster $j$ | unit of $x$ | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $\mu_{\text{lip}}$ | friction of the cup lip against the surface (sideways hold) | — | [15.02](../15-manipulation/15.02-grippers.md) |
| $\mu_\text{prior}, \mu_\text{post}$ | mean of prior / posterior | as $x$ | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $\mu_s, \mu_k$ | static / kinetic (sliding) friction coefficient | — | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |

### Greek: nu

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\nu$ | innovation: measured minus predicted, $z - \hat z$ | m | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $\nu$, $\boldsymbol\nu$ | innovation: measurement minus predicted measurement | m (sensor unit) | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\nu$, $S$ | innovation and its covariance | m, m² | [10.04](../10-localization/10.04-kalman-filter-1d.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |

### Greek: pi

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\pi$ | policy (probability of each action) | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\pi$ | policy: maps state to action (or action probabilities) | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $\pi$ | ratio of circumference to diameter; half a turn in radians (3.14159) | — | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\pi(a \mid s)$ | probability the policy picks $a$ in state $s$ | — | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $\pi(K, \cdot)$ | pinhole projection of a 3D point to a pixel | px | [11.08](../11-slam/11.08-visual-slam.md), [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $\pi^*$ | optimal policy | — | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $\pi^*$ | the expert (human demonstrator) policy | — | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $\pi_\theta(a \mid s)$ | parameterized policy | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |

### Greek: rho

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\rho$ | correlation between neighbouring frames | — | [16.02](../16-machine-learning/16.02-robot-data.md) |
| $\rho$ | correlation coefficient, −1…1 | — | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\rho$ | Lowe ratio-test threshold (0.7–0.8) | — | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $\rho$ | material density (PLA about 1.24) | g/cm³ | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $\rho$ | robust loss function (e.g. Huber) | — | [11.08](../11-slam/11.08-visual-slam.md) |
| $\rho$ | utilization of one executor thread: fraction of time busy in callbacks | — | [04.12](../04-ros2/04.12-executors-and-callbacks.md) |
| $\rho_\text{AWG}$ | resistance per metre of a wire gauge | Ω/m | [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $\rho_{\text{AWG}}$ | resistance per metre of a wire gauge | Ω/m | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $\rho_{Cu}$ | resistivity of copper ($1.72 \times 10^{-8}$) | Ω·m | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $\rho_t$ | PPO probability ratio new policy / old policy | — | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\rho_{XY}$ | correlation coefficient, covariance scaled to $[-1, 1]$ | — | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |

### Greek: sigma

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\sigma$ | a path from start to goal | — | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $\sigma$ | bending stress at the tooth root | MPa | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $\Sigma$ | covariance matrix | varies | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\Sigma$ | covariance matrix of a Gaussian | m² | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\Sigma$ | covariance matrix of the pose estimate | m² | [P10](../projects/P10-localization.md) |
| $\Sigma$ | covariance matrix: variances on the diagonal, covariances off it | units² | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\sigma$ | Gaussian blur width | px | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\sigma$ | measurement noise standard deviation | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\Sigma$ | pixel covariance of a mask (its shape) | px² | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $\sigma$ | standard deviation (noise) of single ADC readings | V | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $\sigma$ | standard deviation of a measurement | varies | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\sigma$ | standard deviation of measurement noise | rad/s | [08.06](../08-control/08.06-derivative-control.md) |
| $\sigma$ | standard deviation of noise (how spread out repeated readings are) | same as reading | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md), [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\sigma$ | standard deviation of the lateral reach error | m | [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md) |
| $\sigma$ | standard deviation: typical size of the random error | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma \pm j\omega$ | complex pole pair: envelope rate $\sigma$, oscillation frequency $\omega$ | 1/s, rad/s | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\sigma(\cdot)$ | sigmoid function | — | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $\sigma(\cdot)$ | sigmoid function $1/(1+e^{-z})$, squashes a score into a probability | — | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md), [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md), [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\sigma$, $\sigma'$ | pixel noise standard deviation before / after filtering | 0–255 | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $\Sigma$, $\Sigma'$ | pose covariance before / after an odometry step | m², rad² (mixed) | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma^2$ | variance of the label noise (the error floor no model can beat) | unit of $y$ squared | [FML.03](../optional-foundations/machine-learning/FML.03-train-validation-test.md) |
| $\sigma^2$ | variance: average squared deviation from the mean | unit² | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma_{\bar x}$ | standard deviation of the mean of $N$ readings | as $x$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma_1$, $\sigma_2$ | standard deviations of those two measurements | rad/s | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\sigma_{\min}$ | smallest singular value of $J$, the singularity alarm | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\sigma_{\min}$ | smallest singular value of the arm Jacobian, near zero at a singularity | — | [P14](../projects/P14-robotic-arm.md) |
| $\sigma_\omega$ | noise of encoder-derived speed from quantization | rad/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\sigma_\omega$ | standard deviation of the speed estimate | rad/s | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\sigma_\theta$ | heading standard deviation | rad | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma_\theta$ | standard deviation of heading | rad | [P08](../projects/P08-odometry.md) |
| $\sigma_a$ | standard deviation of random acceleration (constant-velocity model) | m/s² | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\sigma_{\text{AMCL}}$ | standard deviation of AMCL's pose estimate | m | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $\sigma_{avg}$ | noise after averaging $N$ samples | V | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $\sigma_B^2$ | Otsu between-class variance | — | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $\sigma_D$ | standard deviation of the D term caused by noise | — | [08.06](../08-control/08.06-derivative-control.md) |
| $\sigma_d$, $\Delta d$ | disparity matching error | px | [11.08](../11-slam/11.08-visual-slam.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\sigma_d, \sigma_\theta$ | standard deviation of driven distance / of heading | m, rad | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\sigma_H$, $\sigma_h$ | uncertainty of real object height / of box height | m, px | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\sigma_{hit}$ | standard deviation of the hit model (how far an endpoint may miss a wall) | m | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\sigma_i$ | singular values of $J$ (semi-axes of the velocity ellipse) | m/rad | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\sigma_i$ | standard deviation of error source $i$ | m | [15.07](../15-manipulation/15.07-detect-and-approach.md) |
| $\sigma_i$ (SVD) | singular values of a matrix (stretch factors), $\sigma_1 \geq \sigma_2 \geq \dots$ | varies | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\sigma_k$ | $k$-th singular value | — | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $\sigma_k$ | standard deviation of fresh noise in a DDPM step | — | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\sigma_L^2$, $\sigma_R^2$ | variance of left / right wheel travel | m² | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma_{lat}$, $\sigma_{along}$ | position standard deviation sideways / along the direction of travel | m | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma_n$ | pixel noise standard deviation | 0–255 | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\sigma_n$ | standard deviation of the position (tick) measurement error | ticks | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\Sigma_p$ | covariance of a 3D point estimate | m² | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\sigma_\text{prior}, \sigma_z, \sigma_\text{post}$ | σ of prior, measurement, posterior | as $x$ | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $\sigma_r$ | sensor read noise | e⁻ | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $\sigma_r$, $\sigma_\varphi$ | range and bearing measurement standard deviations | m, rad | [10.06](../10-localization/10.06-extended-kalman-filter.md), [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $\sigma_u$, $\sigma_v$ | pixel uncertainty of a detection | px | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\sigma_v$ | noise of a velocity computed by differencing positions | unit/s | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\sigma_x$ | standard deviation of position along x | m | [P08](../projects/P08-odometry.md) |
| $\sigma_x, \sigma_v$ | noise on a position reading / on the speed derived from it | m, m/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\sigma_x$, $\sigma_y$, $\sigma_{yaw}$ | position / yaw standard deviations | m, m, rad | [09.07](../09-odometry/09.07-odometry-in-ros2.md), [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\sigma_Z$ | standard deviation of the depth noise | m | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\sigma_Z$, $\Delta Z$ | depth error (standard deviation) | m | [11.08](../11-slam/11.08-visual-slam.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |

### Greek: tau

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\tau$ | contrastive temperature: sharpens or softens the softmax | — | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $\tau$ | flow-matching time, 0 = noise, 1 = data | — | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $\tau$ | joint torques | N·m | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\tau$ | motor (mechanical) time constant: time the motor needs to respond to a command | s | [00.01](../00-orientation/00.01-what-is-a-robot.md), [FE.07](../optional-foundations/electronics/FE.07-pwm.md), [03.08](../03-robot-software/03.08-testing-robot-software.md) |
| $\tau$ | motor time constant (first-order lag) | s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\tau$ | motor time constant, time to reach 63 % of final speed | s | [P04](../projects/P04-pid-movement.md), [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\tau$ | motor time constant: time to reach 63 % of a speed change | s | [06.01](../06-simulation/06.01-why-simulate.md), [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\tau$ | NMS IoU threshold | — | [13.10](../13-computer-vision/13.10-object-detection-models.md) |
| $\tau$ | pure delay / latency in a loop or network link | s | [03.04](../03-robot-software/03.04-timing-and-concurrency.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $\tau$ | RC (or RL) time constant of a circuit | s | [01.07](../01-first-robot/01.07-power-system.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $\tau$ | resistor tolerance (fraction, e.g. 0.05 for ±5 %) | — | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $\tau$ | soft target-update rate | — | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md), [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\tau$ | time constant of a first-order system (e.g. motor spin-up) | s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\tau$ | time constant of a first-order system: time to cover 63.2 % of a step | s | [08.02](../08-control/08.02-motor-step-response.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\tau$ | time constant of the motor (or of a filter) | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau$ | timer resolution when timing between ticks | s | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\tau$ | torque (turning force) | N·m | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\tau$ | torque on a shaft | N·m | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\boldsymbol{\tau}$ | torque vector (axis it tends to turn about, times strength) | N·m | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\tau$ | trajectory: one episode's sequence of states and actions | — | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\tau_{\max}$ | largest twisting torque a pad patch can resist about its normal | N·m | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\tau_{cl}$ | desired closed-loop time constant (lambda tuning's single knob) | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau_{cl}$ | desired closed-loop time constant (lambda tuning) | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau_d$ | time constant of the derivative filter | s | [08.06](../08-control/08.06-derivative-control.md) |
| $\tau_e$ | electrical time constant of the motor winding | s | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\tau_f$ | equivalent time constant (≈ lag) of an EMA / speed filter | s | [08.03](../08-control/08.03-wheel-speed-estimation.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\tau_{filter}$ | lag added by the speed estimator's filter | s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau_m$ | mechanical time constant of a spinning wheel | s | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\tau_{motor}, \tau_{out}$ | motor torque / gearbox output torque | N·m | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\tau_s$ | stall torque of a motor | N·m | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\tau_{\text{stall}}$ | servo stall torque | N·m | [15.02](../15-manipulation/15.02-grippers.md) |

### Greek: phi

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\phi$ | activation function (ReLU, tanh, sigmoid, GELU) | — | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\phi$ | angle a wheel has turned through | rad | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\phi$ | angle between the lever arm and the force | rad | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\phi$ | direction the planar arm's tool points, $q_1 + q_2$ | rad | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\phi$ | elevation angle above the $xy$ plane (spherical coordinates) | rad | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\Phi$ | magnetic flux in the solenoid | Wb | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $\phi$ | roll: rotation about the forward $x$ axis | rad | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $\phi$ | wheel rotation angle | rad | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\Phi(s)$ | potential function of the state used for shaping | — | [17.08](../17-reinforcement-learning/17.08-reward-design.md) |
| $\phi(x)$ | engineered features computed from the raw inputs | varies | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\dot\phi, \dot\theta, \dot\psi$ | rate of change of roll, pitch, yaw (not the same as gyro rates) | rad/s | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $\phi_i$ | total wheel angle in calibration run $i$ | rad | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\phi_m, \phi_w$ | motor shaft angle / wheel angle | rad | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\varphi$ | measured bearing to a landmark, relative to the robot's heading | rad | [10.01](../10-localization/10.01-what-is-localization.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\varphi$ | phase lag caused by a delay | ° | [03.04](../03-robot-software/03.04-timing-and-concurrency.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $\varphi_i$ | mean wheel rotation in calibration run $i$ | rad | [09.05](../09-odometry/09.05-calibrating-odometry.md) |

### Greek: chi

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\chi^2$ | chi-squared threshold for $d^2$ gating, or total weighted squared error of a graph | — | [11.03](../11-slam/11.03-the-slam-problem.md), [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\chi^2_{3,0.99}$ | 99 % chi-square threshold with 3 degrees of freedom, used as a Mahalanobis gate | — | [P13](../projects/P13-robot-plus-vision.md) |
| $\chi^2_n$ | chi-square distribution with $n$ degrees of freedom (sum of $n$ squared standard normals) | — | [10.02](../10-localization/10.02-uncertainty-gaussians.md), [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |

### Greek: psi

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\psi$ | goal heading facing into the tag | rad | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $\psi$ | robot heading (yaw) in the world | rad | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $\psi$ | yaw (heading) from IMU / magnetometer | rad | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\psi$ | yaw angle when converted to a quaternion | rad | [09.07](../09-odometry/09.07-odometry-in-ros2.md) |
| $\psi$ | yaw: rotation about the up $z$ axis (heading) | rad | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |

### Greek: omega

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\Omega$ | 4D angle between two quaternions used by slerp | rad | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\omega$ | angular speed of a single wheel or motor | rad/s | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\omega$ | angular speed of the throw arm | rad/s | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\omega$ | angular velocity of a wheel | rad/s | [00.04](../00-orientation/00.04-robot-computers.md), [01.09](../01-first-robot/01.09-reading-encoders.md), [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\omega$ | angular velocity of a wheel or motor shaft | rad/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\boldsymbol{\omega}$ | angular velocity vector: rotation axis times rate | rad/s | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\hat\omega$ | estimated wheel speed (the hat means "estimate") | rad/s | [00.04](../00-orientation/00.04-robot-computers.md) |
| $\omega$ | frequency of a sinusoidal input (frequency response) | rad/s | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\omega$ | joint angular speed | rad/s | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $\omega$ | motor shaft speed (before the gearbox) | rad/s | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\omega$ | motor speed as a fraction of full speed (PWM ripple model) | — | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\dot{\omega}$ | rate of change of wheel speed (angular acceleration) | rad/s² | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $\omega$ | robot angular velocity (yaw rate) | rad/s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega$ | robot yaw rate, positive counter-clockwise (left) | rad/s | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega$ | robot yaw rate: how fast the whole robot turns | rad/s | [00.02](../00-orientation/00.02-the-robotics-stack.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $\omega$ | rotation rate of the robot while searching | rad/s | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $\Omega$ | sample space: every possible outcome | — | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $\omega$ | turn rate (angular velocity) of the robot | rad/s | [11.06](../11-slam/11.06-slam-toolbox-simulation.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\omega$ | turn rate command | rad/s | [17.01](../17-reinforcement-learning/17.01-rl-framing.md), [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $\omega$ | turn rate of the robot (angular velocity about the vertical axis) | rad/s | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $\omega$ | turn rate of the robot body (yaw rate), positive = left | rad/s | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\omega$ | wheel angular speed | rad/s | [08.02](../08-control/08.02-motor-step-response.md) |
| $\omega$ | wheel speed | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\omega^*$ | target wheel speed | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\omega_0$ | no-load speed of a motor | rad/s | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\omega_0$, $\omega_1$ | fraction of pixels below / above threshold $t$ | — | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $\omega_1$, $\omega_2$ | angular speeds of driving and driven gear | rad/s | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $\omega_{180}$ | frequency where the loop phase reaches −180° | rad/s | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\omega_\infty$ | steady-state wheel speed | rad/s | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\omega_\infty$ | target (final) speed in the discrete simulation update | rad/s | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\omega_{\max}$ | top wheel angular speed | rad/s | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\omega_{\max}^{\text{body}}$ | top rotation rate of the robot in place | rad/s | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\omega_{\times}$ | crossover speed: above it counting ticks beats timing them | rad/s | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\omega_{base}$ | angular velocity expressed in the base frame | rad/s | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\omega_{BW}$ | bandwidth of the inner control loop | rad/s | [P04](../projects/P04-pid-movement.md) |
| $\omega_c$ | crossover frequency: where the loop gain $\lvert L\rvert = 1$; roughly how fast the loop responds | rad/s | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\omega_d$ | damped natural frequency | rad/s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\omega_d$ | damped oscillation frequency while ringing | rad/s | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\omega_f$ | final speed a step response approaches, $K\,\mathrm{dz}(u)$ | rad/s | [08.02](../08-control/08.02-motor-step-response.md) |
| $\omega_{fwd}$ | common forward wheel speed in the heading controller | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\hat\omega_i$ | unit rotation axis of joint $i$ (in PoE) | — | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\Omega_{ij}$ | information matrix (inverse covariance): how much to trust constraint $ij$ | 1/m², 1/rad² | [11.03](../11-slam/11.03-the-slam-problem.md), [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\hat\omega_k$ | estimated wheel speed at sample $k$ | rad/s | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\omega_k$ | wheel speed estimated at sample $k$ | rad/s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega_L, \omega_R$ | angular speed of the left / right wheel | rad/s | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\omega_L, \omega_R$ | angular speed of the left / right wheel (short form) | rad/s | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [06.01](../06-simulation/06.01-why-simulate.md) |
| $\omega_L$, $\omega_R$ | left / right wheel angular speed | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega_L$, $\omega_R$ | left and right wheel angular speeds | rad/s | [P04](../projects/P04-pid-movement.md) |
| $\omega_l$, $\omega_r$ | left and right wheel speeds | rad/s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega_{\text{left}}, \omega_{\text{right}}$ | angular speed of the left / right wheel | rad/s | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $\omega_{max}$ | wheel speed at duty 1.0 at nominal voltage (17 rad/s on karmel) | rad/s | [08.09](../08-control/08.09-feedforward-exact-speed.md), [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $\omega_{max}$ | yaw-rate limit of the heading/turn controller | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\omega_{measured}$, $\omega_{gyro}$ | yaw rate reported by the gyro | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md), [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\omega_{motor}, \omega_{out}$ | motor shaft speed / gearbox output speed | rad/s | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\omega_n$ | natural frequency | rad/s | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\omega_n$ | natural frequency of a second-order system | rad/s | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\omega_\text{no-load}$ | no-load motor speed | rad/s | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\omega_{odom}$ | yaw rate computed from wheel odometry | rad/s | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\omega_\text{out}$, $\tau_\text{out}$ | speed / torque at the gearbox output | rad/s, N·m | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\omega_{ss}$ | steady-state (final) wheel speed | rad/s | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega_{ss}$ | steady-state wheel speed for a constant duty | rad/s | [08.02](../08-control/08.02-motor-step-response.md) |
| $\omega_\text{target}$ | commanded motor speed in a first-order motor model | rad/s | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\omega_\text{target}$ | wheel speed the motor is commanded to reach | rad/s | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\omega_{top}(V)$ | wheel speed at duty 1.0 at the current battery voltage | rad/s | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $\omega_w$ | angular speed of one wheel | rad/s | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega_z$ | robot yaw rate (notation of 08.10–08.11) | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\omega_z$ | yaw rate of the robot | rad/s | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\omega_{z,cmd}$ | yaw-rate command from the heading controller | rad/s | [08.10](../08-control/08.10-straight-line-heading-control.md) |

### Other symbols

| Symbol | Meaning | Unit | Lesson |
|---|---|---|---|
| $\mathbb{1}[\cdot]$ | indicator: 1 if the condition holds, else 0 | — | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |

---

## Formulas

Grouped by area in course order, foundations of mathematics first. Within an area, grouped by topic. Each formula is listed at the lesson that teaches it first.

## Mathematics foundations (FM)

### Algebra, units and solving equations
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x^2 = c \Rightarrow x = \pm\sqrt{c}$ | square root has two roots; keep the physically meaningful one | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $x = \frac{-B \pm \sqrt{B^2 - 4AC}}{2A}$ | quadratic formula for $Ax^2 + Bx + C = 0$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $a_1 x + b_1 y = c_1,\ a_2 x + b_2 y = c_2$ | two linear equations in two unknowns | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $x = \frac{c_1 b_2 - c_2 b_1}{a_1 b_2 - a_2 b_1},\ y = \frac{a_1 c_2 - a_2 c_1}{a_1 b_2 - a_2 b_1}$ | Cramer's rule: solution of a 2×2 system | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $D = a_1 b_2 - a_2 b_1 \ne 0$ | condition for a unique solution (equations independent) | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |

### Wheels, encoders and motors
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = 2\pi r \cdot \frac{n}{N}$ | distance rolled after $n$ encoder ticks | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $n = \frac{d \cdot N}{2\pi r}$ | ticks needed to drive distance $d$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $r = \frac{dN}{2\pi n}$ | effective wheel radius from a measured run | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $s = \theta \cdot L/2$ | distance each wheel rolls when the robot turns in place by $\theta$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\omega = k \cdot (\text{duty} - d_0)$ | linear motor model with deadband | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\text{duty} = \omega/k + d_0$ | feedforward: duty needed for a target wheel speed | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\omega = k \cdot \text{duty} + c,\ c = -k d_0$ | the same motor model in linear-regression form | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\omega_{\text{rad/s}} = \text{RPM} \cdot \frac{2\pi}{60}$ | RPM to rad/s | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\Delta\theta = 2\pi/N$ | angle of one encoder tick | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md), [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $N = \text{PPR} \times \text{gear ratio} \times 4$ | ticks per wheel revolution with quadrature (11 × 56 × 4 = 2464) | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\Delta\theta = k \cdot \frac{2\pi}{N}$ | wheel angle turned for $k$ ticks | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\omega = \frac{k \cdot 2\pi / N}{dt} = k\,\Delta\omega$ | wheel speed from ticks counted in one interval | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta\omega = \frac{2\pi}{N \cdot dt}$ | encoder speed resolution (one tick per interval) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta v = \Delta\omega \cdot r$ | speed resolution as ground speed | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\sigma_\omega \approx \frac{\Delta\omega}{\sqrt6}$ | noise of encoder speed from quantization | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\phi_w = \phi_m/56,\ \dot s = r\,\dot\phi_m/56$ | gearbox chain rule: motor angle → wheel angle → ground speed | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\tau\,\dot\omega = \omega_\text{target} - \omega$ | first-order motor model | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $v(t) = v_\text{max}(1 - e^{-t/\tau})$ | motor spin-up speed profile | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\omega_{k+1} = \omega_\text{target} + (\omega_k - \omega_\text{target})\,e^{-h/\tau}$ | exact discretization of the first-order motor (stable for any step) | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $d_i = r\,\phi_i,\quad \phi = \text{ticks} \cdot 2\pi/N$ | wheel-radius calibration model | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |

### Motion, braking and energy
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = \frac{v^2}{2a}$ | stopping distance at speed $v$ with deceleration $a$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $v = \sqrt{2ad}$ | highest speed that can stop within $d$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $d = vt + \frac{v^2}{2a}$ | stopping distance including reaction latency $t$ | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $t = C / I$ | battery runtime | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $x = \frac{1}{2}at^2 \Rightarrow \dot x = at$ | position and speed under constant acceleration | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $g\sin\alpha,\ g\cos\alpha$ | gravity along / into a ramp of angle $\alpha$ | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |

### Angles and radians
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta_{\text{rad}} = \theta_{\text{deg}} \cdot \frac{\pi}{180},\ \theta_{\text{deg}} = \theta_{\text{rad}} \cdot \frac{180}{\pi}$ | degree/radian conversion | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $s = r\theta$ | arc length (radians only) | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $v = \omega r$ | rim speed / ground speed of a wheel | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md), [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\theta \equiv \theta + 2\pi k$ | angles a whole number of turns apart are the same direction | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\operatorname{wrap}(\theta) = \theta - 2\pi \left\lceil \frac{\theta - \pi}{2\pi} \right\rceil$ | wrap any angle into $(-\pi, \pi]$ | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $e = \operatorname{wrap}(\theta_t - \theta_c)$ | shortest signed heading error | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\bar\theta = \operatorname{atan2}(\text{mean}(\sin\theta),\ \text{mean}(\cos\theta))$ | circular mean of angles | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $\omega = K_p \cdot e$ | proportional heading controller (clamped) | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md) |
| $r\,\Delta\theta$ | spacing between neighboring LiDAR hits at range $r$ | [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md), [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |

### Trigonometry
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $(r\cos\theta, r\sin\theta)$ | components of an arrow of length $r$ at angle $\theta$ | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\sin^2\theta + \cos^2\theta = 1$ | Pythagorean identity | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\tan\theta = \frac{\sin\theta}{\cos\theta} = \frac{\text{opposite}}{\text{adjacent}}$ | tangent | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| adjacent $= h\cos\theta$, opposite $= h\sin\theta$ | right-triangle sides from hypotenuse | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $v_x = v\cos\theta,\ v_y = v\sin\theta$ | decompose a velocity into world components | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\operatorname{atan2}(y, x) = \theta \in (-\pi, \pi]$ with $\cos\theta = \frac{x}{\sqrt{x^2+y^2}},\ \sin\theta = \frac{y}{\sqrt{x^2+y^2}}$ | quadrant-correct angle of a point | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\operatorname{wrap}(\operatorname{atan2}(\Delta y, \Delta x) - \theta)$ | heading error to a goal | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $a_x = -g\sin\theta$ | accelerometer reading at pitch $\theta$ (nose-down positive) | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\theta = \operatorname{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$ | robust pitch (tilt) from accelerometer | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\sin\theta \approx \theta,\ \tan\theta \approx \theta,\ \cos\theta \approx 1 - \theta^2/2$ | small-angle approximation | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $c^2 = a^2 + b^2 - 2ab\cos C$ | law of cosines | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $C = \arccos\frac{a^2 + b^2 - c^2}{2ab}$ | elbow angle for a reach distance (2-link IK) | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\lvert a - b\rvert \le c \le a + b$ | reach limits of a two-link arm | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $d = h/\tan\alpha$ | where a downward-tilted sensor ray hits the floor | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| width $= 2\,d\tan(\alpha/2)$ | width of a sensor cone of full angle $\alpha$ at distance $d$ | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |

### Coordinates
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x = r\cos\theta,\ y = r\sin\theta$ | polar to Cartesian | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $r = \sqrt{x^2 + y^2},\ \theta = \operatorname{atan2}(y, x)$ | Cartesian to polar | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $\theta_i = \theta_{\min} + i \cdot \Delta\theta$ | angle of LiDAR beam $i$ | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $i = (\theta - \theta_{\min})/\Delta\theta$ | beam index that looks at angle $\theta$ | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $d = \sqrt{r_1^2 + r_2^2 - 2r_1r_2\cos(\theta_2 - \theta_1)}$ | distance between two polar points | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $x = r\cos\phi\cos\theta,\ y = r\cos\phi\sin\theta,\ z = r\sin\phi$ | spherical to Cartesian (elevation from the plane) | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $r = \sqrt{x^2+y^2+z^2},\ \theta = \operatorname{atan2}(y, x),\ \phi = \operatorname{atan2}(z, \sqrt{x^2+y^2})$ | Cartesian to spherical | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |
| $(r, \theta, z)$ | cylindrical coordinates: polar plus height | [FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md) |

### Vectors, dot and cross product
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{a} + \mathbf{b} = (a_x + b_x,\ a_y + b_y),\ k\mathbf{a} = (k a_x,\ k a_y)$ | component-wise addition and scaling | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{d} = \mathbf{g} - \mathbf{p}$ | displacement: point minus point is a vector | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{p} + \mathbf{v}\Delta t$ | new position after moving with velocity $\mathbf{v}$ | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\lVert\mathbf{v}\rVert = \sqrt{v_1^2 + v_2^2 + \dots + v_n^2}$ | Euclidean norm (length) | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\lVert k\mathbf{v}\rVert = \lvert k\rvert\,\lVert\mathbf{v}\rVert$ | scaling a vector scales its length | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\lVert\mathbf{a} + \mathbf{b}\rVert \le \lVert\mathbf{a}\rVert + \lVert\mathbf{b}\rVert$ | triangle inequality | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\hat{\mathbf{v}} = \frac{\mathbf{v}}{\lVert\mathbf{v}\rVert}$ | unit vector (direction) | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{v} = v_x\hat{\mathbf{x}} + v_y\hat{\mathbf{y}}$ | linear combination of basis vectors | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\hat{\mathbf{f}} = (\cos\theta, \sin\theta),\ \hat{\mathbf{l}} = (-\sin\theta, \cos\theta)$ | robot forward and left directions in the map | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{v}_\text{rel} = \mathbf{v}_B - \mathbf{v}_A$ | relative velocity | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\frac{1}{N}\sum_i \mathbf{p}_i$ | centroid of a point cluster | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| attraction $+\ k\,\hat{\mathbf{d}}_\text{away}/d$ | potential-field steering (sum of attract and repel vectors) | [FM.05](../optional-foundations/mathematics/FM.05-vectors.md) |
| $\mathbf{a}\cdot\mathbf{b} = \sum_i a_i b_i = \lVert\mathbf{a}\rVert\,\lVert\mathbf{b}\rVert\cos\theta$ | dot product | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\theta = \arccos\frac{\mathbf{a}\cdot\mathbf{b}}{\lVert\mathbf{a}\rVert\lVert\mathbf{b}\rVert}$ | angle between two vectors (also cosine similarity) | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{a}\cdot\mathbf{a} = \lVert\mathbf{a}\rVert^2$ | dot with itself is squared length | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{a}\cdot\hat{\mathbf{u}}$, $(\mathbf{a}\cdot\hat{\mathbf{u}})\,\hat{\mathbf{u}}$ | scalar and vector projection onto a unit direction | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\frac{\mathbf{a}\cdot\mathbf{b}}{\mathbf{b}\cdot\mathbf{b}}\mathbf{b}$ | projection onto a non-unit direction | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $t = \frac{(P-A)\cdot(B-A)}{(B-A)\cdot(B-A)}$, closest point $A + t(B - A)$ | progress along a path segment (clamp $t$ to $[0,1]$) | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\lVert P - (A + t(B-A))\rVert$ | cross-track error | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{a}\times\mathbf{b} = a_x b_y - a_y b_x$ | 2D cross product: positive = $\mathbf{b}$ is to the left of $\mathbf{a}$ | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{a}\times\mathbf{b} = (a_y b_z - a_z b_y,\ a_z b_x - a_x b_z,\ a_x b_y - a_y b_x)$ | 3D cross product | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\lVert\mathbf{a}\times\mathbf{b}\rVert = \lVert\mathbf{a}\rVert\lVert\mathbf{b}\rVert\sin\theta$ | cross product length = parallelogram area | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{b}\times\mathbf{a} = -(\mathbf{a}\times\mathbf{b})$ | cross product is anti-commutative | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\hat{\mathbf{x}}\times\hat{\mathbf{y}} = \hat{\mathbf{z}},\ \hat{\mathbf{y}}\times\hat{\mathbf{z}} = \hat{\mathbf{x}},\ \hat{\mathbf{z}}\times\hat{\mathbf{x}} = \hat{\mathbf{y}}$ | right-handed frame | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\boldsymbol{\tau} = \mathbf{r}\times\mathbf{F}$ | torque of a force about a joint | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{v} = \boldsymbol{\omega}\times\mathbf{r}$ | velocity of a point on a rotating body | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |
| $\mathbf{n} = (p_2 - p_1)\times(p_3 - p_1)$ | surface normal from three points | [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md) |

### Matrices and linear transformations
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $A\mathbf{x} = \begin{pmatrix} a_{11}x_1 + a_{12}x_2 \\ a_{21}x_1 + a_{22}x_2 \end{pmatrix}$ | matrix times vector | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $c_{ij} = \sum_{k=1}^{n} a_{ik} b_{kj}$ | matrix product (row $i$ dotted with column $j$) | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $(m \times n)(n \times p) = (m \times p)$ | shape rule: inner dimensions must match | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $AB\mathbf{x} = A(B\mathbf{x})$, $AB \ne BA$ | products apply right to left and do not commute | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $I\mathbf{x} = \mathbf{x},\ AI = IA = A$ | identity matrix | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $(A^T)_{ij} = a_{ji},\ (A^T)^T = A$ | transpose | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $(AB)^T = B^T A^T$ | transpose of a product reverses order | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $\mathbf{a}\cdot\mathbf{b} = \mathbf{a}^T\mathbf{b}$ | dot product as a matrix product | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $AA^{-1} = A^{-1}A = I$ | inverse (square, $\det A \ne 0$) | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $(AB)^{-1} = B^{-1}A^{-1}$ | inverse of a product reverses order | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $\begin{pmatrix} a & b \\ c & d \end{pmatrix}^{-1} = \frac{1}{ad - bc}\begin{pmatrix} d & -b \\ -c & a \end{pmatrix}$ | 2×2 inverse | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $\det\begin{pmatrix} a & b \\ c & d\end{pmatrix} = ad - bc$ | 2×2 determinant: signed area of the column parallelogram | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md), [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $\mathbf{x}' = F\mathbf{x},\ P' = FPF^T$ | Kalman prediction of state and covariance (preview) | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $M\begin{pmatrix} a \\ b \end{pmatrix} = a\,(M\hat{\mathbf{x}}) + b\,(M\hat{\mathbf{y}})$ | columns of a matrix are where the basis vectors go | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $M(\mathbf{a} + \mathbf{b}) = M\mathbf{a} + M\mathbf{b},\ M(k\mathbf{a}) = kM\mathbf{a}$ | linearity (so $M\mathbf{0} = \mathbf{0}$: no translation possible) | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $\det(AB) = \det A \cdot \det B$ | determinant of a product | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $\mathbf{v}_A = B\,\mathbf{v}_C,\ \mathbf{v}_C = B^{-1}\mathbf{v}_A$ | change of basis between frames | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $B^{-1} = B^T$ (orthonormal $B$, $B^TB = I$) | inverse of an orthonormal basis is its transpose | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $R_{\text{base}\leftarrow\text{optical}} = \begin{pmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{pmatrix}$ | camera optical frame ($z$ fwd, $x$ right, $y$ down) to `base_link` | [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $(u, v) = \big((K\mathbf{p})_1,\ (K\mathbf{p})_2\big)/(K\mathbf{p})_3$ | project a 3D camera point to pixels with intrinsics $K$ | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md), [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |

### Differential-drive kinematics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v = \frac{r(\omega_R + \omega_L)}{2},\ \omega = \frac{r(\omega_R - \omega_L)}{L}$ | forward kinematics: wheel speeds → robot speed and turn rate | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md), [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\omega_R = \frac{v + \omega L/2}{r},\ \omega_L = \frac{v - \omega L/2}{r}$ | inverse kinematics: command → wheel speeds | [FM.01](../optional-foundations/mathematics/FM.01-algebra-refresher.md) |
| $\begin{pmatrix} v \\ \omega \end{pmatrix} = \begin{pmatrix} r/2 & r/2 \\ -r/L & r/L \end{pmatrix}\begin{pmatrix} \omega_L \\ \omega_R \end{pmatrix}$ | kinematics as a matrix $J$ | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $J^{-1} = \begin{pmatrix} 1/r & -L/(2r) \\ 1/r & L/(2r) \end{pmatrix}$ | inverse kinematics matrix | [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $x \mathrel{+}= v\cos\theta\,\Delta t,\ y \mathrel{+}= v\sin\theta\,\Delta t$ | dead-reckoning (Euler) pose update | [FM.03](../optional-foundations/mathematics/FM.03-trigonometry.md) |
| $\dot{\mathbf{s}} = (v\cos\theta,\ v\sin\theta,\ \omega)$ | unicycle model as an ODE | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $R = v/\omega$ | radius of the circle driven at constant $v$, $\omega$ | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $x = R\sin(\omega t),\ y = R(1 - \cos\omega t)$ | exact circular path from the origin facing $+x$ | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |

### Homogeneous transforms and frames
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{p}_{\text{robot}} = R\,\mathbf{p}_{\text{lidar}} + \mathbf{t}$ | rigid transform of a point: rotate then translate | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T = \begin{pmatrix} R & \mathbf{t} \\ \mathbf{0}^T & 1 \end{pmatrix},\ T\begin{pmatrix}\mathbf{p} \\ 1\end{pmatrix} = \begin{pmatrix} R\mathbf{p} + \mathbf{t} \\ 1\end{pmatrix}$ | homogeneous transform (points $w = 1$, directions $w = 0$) | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T = \begin{pmatrix} \cos\theta & -\sin\theta & t_x \\ \sin\theta & \cos\theta & t_y \\ 0 & 0 & 1\end{pmatrix}$ | 2D pose $(t_x, t_y, \theta)$ as a 3×3 matrix (SE(2)) | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $\mathbf{p}_A = T_{A\,B}\,\mathbf{p}_B$ | convert a point from frame B into frame A | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T_{A\,C} = T_{A\,B}\,T_{B\,C}$ | chain transforms; inner frame names cancel | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T_{\text{map base}} = T_{\text{map odom}}\,T_{\text{odom base}}$ | REP-105 chain for the robot pose in the map | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T_{B\,A} = T_{A\,B}^{-1} = \begin{pmatrix} R^T & -R^T\mathbf{t} \\ \mathbf{0}^T & 1\end{pmatrix}$ | closed-form inverse of a rigid transform | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $T_{\text{new}} = T_{\text{old}}\,\Delta T$ | apply a motion expressed in the robot's own frame (odometry) | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $\Delta T = T_1^{-1}T_2$ | relative pose between two poses | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |
| $(uw, vw, w) \sim (u, v, 1)$ | projective: divide by $w$ to get the image point | [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md) |

### Rotations, Euler angles and quaternions
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $R(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}$ | 2D rotation matrix | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md), [FM.08](../optional-foundations/mathematics/FM.08-matrices-as-transformations.md) |
| $R(\alpha)R(\beta) = R(\alpha + \beta)$ | 2D rotations commute and add | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_x(\alpha) = \begin{pmatrix} 1 & 0 & 0 \\ 0 & \cos\alpha & -\sin\alpha \\ 0 & \sin\alpha & \cos\alpha \end{pmatrix}$ | rotation about $x$ (roll) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_y(\beta) = \begin{pmatrix} \cos\beta & 0 & \sin\beta \\ 0 & 1 & 0 \\ -\sin\beta & 0 & \cos\beta \end{pmatrix}$ | rotation about $y$ (pitch; positive = nose down) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_z(\gamma) = \begin{pmatrix} \cos\gamma & -\sin\gamma & 0 \\ \sin\gamma & \cos\gamma & 0 \\ 0 & 0 & 1 \end{pmatrix}$ | rotation about $z$ (yaw) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R^TR = I,\ \det R = +1$ | definition of a rotation (SO(3)) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R^{-1} = R^T$ | inverse rotation is the transpose | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\lVert R\mathbf{v}\rVert = \lVert\mathbf{v}\rVert$ | rotations preserve lengths | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\mathbf{p}' = R\mathbf{p}$ (active), $\mathbf{p}_B = R^T\mathbf{p}_A$ (passive) | rotate a vector vs re-express it in a rotated frame | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\mathbf{p}_A = R_{A\,B}\,\mathbf{p}_B$ | frame-named rotation | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R_{\text{err}} = R_a^TR_b$ | relative rotation between two orientations | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\operatorname{tr}(R) = 1 + 2\cos\theta,\ \theta = \arccos\frac{\operatorname{tr}(R) - 1}{2}$ | rotation angle from the trace (orientation error) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $\mathbf{k} \propto (R_{32} - R_{23},\ R_{13} - R_{31},\ R_{21} - R_{12})$ | rotation axis from a matrix | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R = I + \sin\theta\,K + (1 - \cos\theta)\,K^2$ | Rodrigues' formula: axis-angle → matrix | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $M = U\Sigma V^T \Rightarrow R = UV^T$ | nearest rotation matrix (re-orthonormalization) | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| $R = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$ | ROS roll-pitch-yaw convention | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $\phi = \operatorname{atan2}(R_{21}, R_{22}),\ \theta = -\arcsin(R_{20}),\ \psi = \operatorname{atan2}(R_{10}, R_{00})$ | extract roll, pitch, yaw from a rotation matrix | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $(\phi + 180°,\ 180° - \theta,\ \psi + 180°) \equiv (\phi, \theta, \psi)$ | two Euler triples for the same orientation | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $R_z(\psi)R_y(90°)R_x(\phi)$ depends only on $\phi - \psi$ | gimbal lock at pitch 90° | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $\dot\phi = p + (q\sin\phi + r\cos\phi)\tan\theta,\ \dot\theta = q\cos\phi - r\sin\phi,\ \dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta}$ | Euler-angle rates from gyro body rates | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| $q = (\hat{\mathbf{u}}\sin\tfrac{\theta}{2},\ \cos\tfrac{\theta}{2})$ | unit quaternion from axis and angle | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $x^2 + y^2 + z^2 + w^2 = 1$ | unit-quaternion constraint | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q = (0, 0, \sin\frac{\psi}{2}, \cos\frac{\psi}{2}),\ \psi = 2\operatorname{atan2}(z, w)$ | quaternion of a pure yaw and back | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\psi = \operatorname{atan2}\big(2(wz + xy),\ 1 - 2(y^2 + z^2)\big)$ | exact yaw from any quaternion | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{ijk} = -1$ | quaternion algebra rules | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q_a \otimes q_b = \big(w_a\mathbf{v}_b + w_b\mathbf{v}_a + \mathbf{v}_a\times\mathbf{v}_b,\ w_aw_b - \mathbf{v}_a\cdot\mathbf{v}_b\big)$ | Hamilton product (compose rotations) | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $R(q_a \otimes q_b) = R(q_a)R(q_b)$ | quaternion product matches matrix composition | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q^* = (-\mathbf{v}, w) = q^{-1}$ | conjugate = inverse for unit quaternions | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $(\mathbf{p}', 0) = q \otimes (\mathbf{p}, 0) \otimes q^*$ | rotate a vector with a quaternion | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q \equiv -q$ | double cover: same rotation | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\Delta\theta = 2\arccos\big(\lvert q_1 \cdot q_2\rvert\big)$ | angle between two orientations | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $\operatorname{slerp}(q_1, q_2, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega}q_1 + \frac{\sin(t\Omega)}{\sin\Omega}q_2,\ \cos\Omega = q_1\cdot q_2$ | spherical linear interpolation (flip $q_2$ if the dot is negative) | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q_{k+1} = q_k \otimes \Big(\tfrac{\boldsymbol{\omega}}{\lVert\boldsymbol{\omega}\rVert}\sin\tfrac{\lVert\boldsymbol{\omega}\rVert\Delta t}{2},\ \cos\tfrac{\lVert\boldsymbol{\omega}\rVert\Delta t}{2}\Big)$ | integrate a gyro in quaternions | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |
| $q_{\text{map cam}} = q_{\text{map base}} \otimes q_{\text{base cam}}$ | frame-named quaternion chain | [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md) |

### Probability
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $P(\Omega) = 1,\ 0 \le P(A) \le 1$ | axioms of probability | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(\text{not } A) = 1 - P(A)$ | complement rule | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A \text{ or } B) = P(A) + P(B) - P(A \text{ and } B)$ | sum rule (overlap subtracted) | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A \mid B) = \frac{P(A \text{ and } B)}{P(B)}$ | conditional probability | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A \text{ and } B) = P(A \mid B)\,P(B) = P(B \mid A)\,P(A)$ | product rule | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A \text{ and } B) = P(A)\,P(B) \Leftrightarrow P(A \mid B) = P(A)$ | independence | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(A) = \sum_i P(A \mid B_i)\,P(B_i)$ | law of total probability | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(\text{none in } k) = (1-p)^k,\ P(\text{at least one}) = 1 - (1-p)^k$ | repeated independent trials | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(X = x) = \binom{k}{x} p^x (1-p)^{k-x}$ | binomial distribution | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $E[X] = \sum_x x\,P(X = x)$; binomial $E[X] = k\,p$ | expected value | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $\sqrt{p(1-p)/n}$ | typical error of a probability estimated from $n$ trials | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $P(\text{both}) = \sum_s P(s)\,P(A \mid s)\,P(B \mid s)$ | joint failure with a common cause (conditional independence) | [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |

### Distributions and noise
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\int p(x)\,dx = 1,\ P(a < X < b) = \int_a^b p(x)\,dx$ | probability density: only areas are probabilities | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $p(x) = 1/(b-a)$, $\mu = (a+b)/2$, $\sigma^2 = (b-a)^2/12$ | uniform distribution on $[a, b]$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma = \Delta/\sqrt{12}$ | quantization noise of step $\Delta$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\bar{x} = \frac{1}{N}\sum x_i,\ s^2 = \frac{1}{N-1}\sum(x_i - \bar{x})^2$ | sample mean and sample variance | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\mu = E[X],\ \sigma^2 = E[(X-\mu)^2]$ | true mean and variance | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $p(x) = \frac{1}{\sigma\sqrt{2\pi}}\exp\!\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$ | Gaussian density $\mathcal{N}(\mu, \sigma^2)$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $z = (x-\mu)/\sigma$ | z-score | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\mu \pm 1\sigma, 2\sigma, 3\sigma$ contain 68.3%, 95.4%, 99.7% | 68-95-99.7 rule | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| threshold $= d_\text{safe} + 3\sigma$ | one-sided 3σ safety margin | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $Y = aX + b \Rightarrow \mu_Y = a\mu_X + b,\ \sigma_Y = \lvert a\rvert\sigma_X$ | scaling a random variable | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma^2_{X+Y} = \sigma^2_X + \sigma^2_Y$ | variances of independent errors add | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma_{\bar x} = \frac{\sigma}{\sqrt{N}}$ | averaging $N$ independent readings | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma_n = \sigma_\text{step}\sqrt{n}$ | random walk: integrated noise grows like $\sqrt{t}$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $d = c\,t/2,\ c = 331.3 + 0.606\,T$ | ultrasonic range from echo time and temperature | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $z = x + b + \mathcal{N}(0,\sigma^2)$ | additive Gaussian sensor noise model with bias | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| excess kurtosis of a sum of $N$ uniforms $= -1.2/N$; skewness of $N$ exponentials $= 2/\sqrt{N}$ | central limit theorem in numbers | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $E[(x-\mu)^3]/\sigma^3$ | skewness | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma^2_\text{total} = \sigma^2 + p(1-p)\Delta^2$ | variance of a Gaussian with outlier fraction $p$ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $1.4826 \cdot \operatorname{median}(\lvert x - \operatorname{median}(x)\rvert)$ | robust σ from the MAD | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |
| $\sigma/\sqrt{2N}$ | uncertainty of an estimated σ | [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md) |

### Covariance and multivariate Gaussians
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\operatorname{cov}(X, Y) = E[(X - \mu_X)(Y - \mu_Y)]$ | covariance | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\operatorname{cov} \approx \frac{1}{N-1}\sum_i (x_i - \bar x)(y_i - \bar y)$ | sample covariance | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\rho_{XY} = \frac{\operatorname{cov}(X,Y)}{\sigma_X\,\sigma_Y}$ | correlation coefficient | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma = E[(\mathbf{x}-\boldsymbol\mu)(\mathbf{x}-\boldsymbol\mu)^\top],\ \Sigma_{ij} = \operatorname{cov}(x_i, x_j)$ | covariance matrix | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma = \Sigma^\top,\ \mathbf{v}^\top\Sigma\mathbf{v} \geq 0$ | covariance is symmetric and positive semi-definite | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $p(\mathbf{x}) = \frac{1}{\sqrt{(2\pi)^n \det\Sigma}}\exp\!\left(-\tfrac{1}{2}(\mathbf{x}-\boldsymbol\mu)^\top\Sigma^{-1}(\mathbf{x}-\boldsymbol\mu)\right)$ | multivariate Gaussian density | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $d_M^2 = (\mathbf{x}-\boldsymbol\mu)^\top\Sigma^{-1}(\mathbf{x}-\boldsymbol\mu)$ | squared Mahalanobis distance (gating test vs chi-squared threshold) | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma = V\Lambda V^\top$, semi-axes $\sqrt{k\lambda_i}$ | uncertainty ellipse ($k = 5.991$ for 95% in 2D) | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\boldsymbol\mu_y = A\boldsymbol\mu_x + \mathbf{b},\ \Sigma_y = A\,\Sigma_x A^\top$ | linear propagation of mean and covariance | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma_\text{world} = R\,\Sigma_\text{robot} R^\top$ | rotate a covariance into another frame | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma_y \approx J\,\Sigma_x J^\top$ | linearized propagation through a nonlinear function | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $J = \begin{bmatrix} \cos\theta & -d\sin\theta \\ \sin\theta & d\cos\theta \end{bmatrix}$ | Jacobian of $(x, y) = (d\cos\theta, d\sin\theta)$ w.r.t. $(d, \theta)$ | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\sigma_\text{sideways} = d\cdot\sigma_\theta$ | sideways error = distance × heading error | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma = LL^\top,\ \mathbf{x} = \boldsymbol\mu + L\mathbf{z}$ | sample a correlated Gaussian with Cholesky | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\Sigma_{k+1} = F\Sigma_k F^\top + G\,Q\,G^\top$ | pose covariance propagation per motion step | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| index $= 6\cdot\text{row} + \text{col}$ | ROS 6×6 covariance array indexing | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |
| $\sqrt{2/N}$ | relative error of a sample covariance entry | [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md) |

### Bayes' rule
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $P(x \mid z) = \frac{P(z \mid x)\,P(x)}{P(z)},\ P(z) = \sum_{x'} P(z \mid x')\,P(x')$ | Bayes' rule: posterior = likelihood × prior / evidence | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md), [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |
| $\frac{P(\text{open} \mid z)}{P(\text{closed} \mid z)} = \frac{P(z \mid \text{open})}{P(z \mid \text{closed})} \cdot \frac{P(\text{open})}{P(\text{closed})}$ | odds form: posterior odds = likelihood ratio × prior odds | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $l = \ln\frac{p}{1-p}$ | log-odds | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $l_t = l_{t-1} + \ln\frac{P(z_t \mid \text{open})}{P(z_t \mid \text{closed})}$ | log-odds update (occupancy grids) | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $p = \frac{1}{1 + e^{-l}}$ | log-odds back to probability | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $\frac{1}{\sigma_\text{post}^2} = \frac{1}{\sigma_\text{prior}^2} + \frac{1}{\sigma_z^2}$ | fusing two Gaussians: information (1/variance) adds | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |
| $\mu_\text{post} = \sigma_\text{post}^2\left(\frac{\mu_\text{prior}}{\sigma_\text{prior}^2} + \frac{z}{\sigma_z^2}\right)$ | fused mean (1D Kalman update) | [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md) |

### Derivatives and Jacobians
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\frac{dx}{dt} = \dot x(t) = \lim_{h \to 0}\frac{x(t+h) - x(t)}{h}$ | derivative (rate of change) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{dt}c = 0$ | derivative of a constant | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{dt}t^n = n\,t^{n-1}$ | power rule | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{dt}\sin(\omega t) = \omega\cos(\omega t),\ \frac{d}{dt}\cos(\omega t) = -\omega\sin(\omega t)$ | derivatives of sine and cosine | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{dt}e^{kt} = k\,e^{kt}$ | derivative of an exponential | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{du}\sqrt u = \frac{1}{2\sqrt u}$ | derivative of a square root | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{d}{dz}\operatorname{atan}z = \frac{1}{1+z^2}$ | derivative of arctangent | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $(af + bg)' = af' + bg'$ | linearity | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $(fg)' = f'g + fg'$ | product rule | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $f(g(t))' = f'(g(t))\cdot g'(t)$ | chain rule | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{x(t+h) - x(t)}{h}$, error $\approx \frac{h}{2}\lvert\ddot x\rvert$ | forward difference (first order) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{x(t+h) - x(t-h)}{2h}$, error $\approx \frac{h^2}{6}\lvert\dddot x\rvert$ | central difference (second order) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $x(t+h) = x + h\dot x + \frac{h^2}{2}\ddot x + \frac{h^3}{6}\dddot x + \dots$ | Taylor expansion | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| roundoff error $\approx \epsilon/h$ | why tiny $h$ makes numerical derivatives worse | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\sigma_v = \frac{\sqrt{2} \cdot \sigma}{dt}$ | noise of velocity from differencing noisy positions | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\nabla f = [\partial f/\partial x,\ \partial f/\partial y]^\top$ | gradient (points uphill) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $J_{ij} = \partial f_i/\partial x_j,\ \Delta\mathbf{f} \approx J \cdot \Delta\mathbf{x}$ | Jacobian: local linear map from input changes to output changes | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\frac{\partial}{\partial x}\operatorname{atan2}(u, v) = \frac{v \cdot \partial u/\partial x - u \cdot \partial v/\partial x}{u^2 + v^2}$ | derivative of atan2 | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $d = \sqrt{(l_x - x)^2 + (l_y - y)^2}$ | range from robot to landmark | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\beta = \operatorname{atan2}(l_y - y,\ l_x - x) - \theta$ | bearing from robot to landmark | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\partial d/\partial x = -v/d,\ \partial d/\partial y = -u/d,\ \partial d/\partial\theta = 0$ | range row of the measurement Jacobian | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\partial\beta/\partial x = u/q,\ \partial\beta/\partial y = -v/q,\ \partial\beta/\partial\theta = -1$ | bearing row of the measurement Jacobian | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\begin{bmatrix}\Delta d \\ \Delta\beta\end{bmatrix} \approx H \begin{bmatrix}\Delta x \\ \Delta y \\ \Delta\theta\end{bmatrix}$ | pose error → predicted measurement error (EKF $H$) | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\Delta\beta \approx H_\beta \cdot \Delta p$ | bearing change from a small pose change | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\lVert\nabla d\rVert = 1,\ \lVert\nabla\beta\rVert = 1/d$ | range sensitivity is 1:1; bearing sensitivity falls with distance | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |

### Robot arm kinematics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x = l_1\cos q_1 + l_2\cos(q_1+q_2),\ y = l_1\sin q_1 + l_2\sin(q_1+q_2)$ | forward kinematics of a 2-link planar arm | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $J = \begin{bmatrix} -l_1\sin q_1 - l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2) \\ l_1\cos q_1 + l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2) \end{bmatrix}$ | 2-link arm Jacobian | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\begin{bmatrix}\Delta x \\ \Delta y\end{bmatrix} \approx J \begin{bmatrix}\Delta q_1 \\ \Delta q_2\end{bmatrix}$ | small joint changes → tip displacement | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $v_{\text{tip}} = J\dot q$ | joint speeds → tip velocity | [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md) |
| $\dot{\mathbf{q}} = J^+\mathbf{v}$ | minimum-norm joint speeds for a desired tip velocity (redundant arm) | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $(J^\top J + \lambda^2 I)^{-1}J^\top$ | damped least squares near singularities | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\lVert\mathbf{p}\rVert \le l_1 + l_2$ | reachability check before solving IK | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |

### Integration and simulation
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x(T) = x(0) + \int_0^T v(t)\,dt$ | position from integrated velocity | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\frac{d}{dT}\int_0^T v(t)\,dt = v(T)$ | fundamental theorem of calculus | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\int_0^T c\,dt = cT,\ \int_0^T at\,dt = \frac{1}{2}aT^2$ | integrals of constant and linear rates | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\int_0^T \cos(\omega t)\,dt = \frac{1}{\omega}\sin(\omega T)$ | integral of cosine | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\int_0^T e^{-t/\tau}\,dt = \tau\,(1 - e^{-T/\tau})$ | integral of exponential decay | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\sum_{k=0}^{N-1} v_k\,\Delta t$ | left Riemann sum (first order, what Euler does) | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\sum_{k=0}^{N-1} \frac{v_k + v_{k+1}}{2}\,\Delta t$ | trapezoidal rule (second order) | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\mathbf{s}_{k+1} = \mathbf{s}_k + h\,f(\mathbf{s}_k, t_k)$ | Euler step | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\mathbf{k}_1 = f(\mathbf{s}_k, t_k),\ \mathbf{s}_{k+1} = \mathbf{s}_k + h\,f(\mathbf{s}_k + \tfrac{h}{2}\mathbf{k}_1,\ t_k + \tfrac{h}{2})$ | midpoint method (RK2) | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\mathbf{s}_{k+1} = \mathbf{s}_k + \frac{h}{2}(\mathbf{k}_1 + \mathbf{k}_2),\ \mathbf{k}_2 = f(\mathbf{s}_k + h\mathbf{k}_1)$ | Heun's method (another RK2) | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\mathbf{s}_{k+1} = \mathbf{s}_k + \tfrac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$ | classic Runge-Kutta 4 | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| global error $\propto h^p$ ($p$ = 1, 2, 4) | order of Euler, RK2, RK4 | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\omega_{k+1} - \omega_\text{target} = \left(1 - \frac{h}{\tau}\right)(\omega_k - \omega_\text{target})$ | Euler on a first-order system | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $h < 2\tau$ (stable), $h \le \tau/10$ (accurate) | Euler stability and accuracy rule of thumb | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| error $= b\,t$ (rate), $\frac{1}{2}b\,t^2$ (acceleration) | integrated constant bias | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |
| $\int_0^T v\sin(bt)\,dt = \frac{v}{b}(1 - \cos bT) \approx \frac{1}{2}vbT^2$ | sideways drift from a growing heading error | [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md) |

### Linear systems and least squares
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $A\mathbf{x} = \mathbf{b}$ | linear system (unique solution when $\det A \neq 0$) | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md), [FM.07](../optional-foundations/mathematics/FM.07-matrices.md) |
| $J(\mathbf{x}) = \lVert A\mathbf{x} - \mathbf{b}\rVert^2 = \sum_{i=1}^{m}(\mathbf{a}_i^\top\mathbf{x} - b_i)^2$ | least-squares cost | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\nabla J = 2A^\top(A\mathbf{x} - \mathbf{b})$ | gradient of the least-squares cost | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^\top A\,\mathbf{x} = A^\top\mathbf{b},\ \hat{\mathbf{x}} = (A^\top A)^{-1}A^\top\mathbf{b}$ | normal equations | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^\top\mathbf{r} = 0$ | residual is perpendicular to every column of $A$ | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\hat r = \frac{\sum_i \phi_i d_i}{\sum_i \phi_i^2}$ | one-parameter least squares (wheel radius) | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\sigma_{\hat r} = s/\sqrt{\sum\phi_i^2}$ | uncertainty of a one-parameter fit | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\operatorname{Cov}(\hat{\mathbf{x}}) = s^2 (A^\top A)^{-1}$ | covariance of least-squares parameters | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^\top W A\,\mathbf{x} = A^\top W\mathbf{b},\ W = \operatorname{diag}(1/\sigma_i^2)$ | weighted least squares | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\kappa(A) = \sigma_\text{max}/\sigma_\text{min}$ | condition number | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $\kappa(A^\top A) = \kappa(A)^2$ | why forming $A^\top A$ loses accuracy | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A = U\Sigma V^\top$ | singular value decomposition | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^+ = V\Sigma^+U^\top$ | pseudo-inverse | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| $A^+ = (A^\top A)^{-1}A^\top$ (overdetermined), $A^+ = A^\top(AA^\top)^{-1}$ (underdetermined) | pseudo-inverse for full-rank cases | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |

### Optimization
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{x}_{k+1} = \mathbf{x}_k - \eta\,\nabla f(\mathbf{x}_k)$ | gradient descent | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $f(\mathbf{x}) = \frac{1}{2}\mathbf{x}^\top H\mathbf{x} \Rightarrow \mathbf{x}_{k+1} = (I - \eta H)\mathbf{x}_k$ | gradient descent on a quadratic | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $0 < \eta < \frac{2}{\lambda_\text{max}}$ | learning-rate stability condition | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| per-step factor $1 - \eta\lambda_i$ | error shrink factor along each curvature direction | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $F(\mathbf{x}) = \frac{1}{2}\lVert\mathbf{r}(\mathbf{x})\rVert^2 = \frac{1}{2}\sum_i r_i(\mathbf{x})^2$ | nonlinear least-squares cost | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\nabla F = J^\top\mathbf{r}$ | gradient of a least-squares cost | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\mathbf{r}(\mathbf{x} + \boldsymbol\Delta) \approx \mathbf{r} + J\boldsymbol\Delta$ | linearized residual | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $J^\top J\,\boldsymbol\Delta = -J^\top\mathbf{r},\ \mathbf{x} \leftarrow \mathbf{x} + \boldsymbol\Delta$ | Gauss-Newton step | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\left(J^\top J + \lambda I\right)\boldsymbol\Delta = -J^\top\mathbf{r}$ | Levenberg-Marquardt step | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $\boldsymbol\Delta \approx -\frac{1}{\lambda}J^\top\mathbf{r}$ (large $\lambda$) | LM becomes a short gradient-descent step | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| $r_i(\mathbf{p}) = \lVert\mathbf{p} - \mathbf{b}_i\rVert - z_i$, row $(\mathbf{p} - \mathbf{b}_i)^\top/\lVert\mathbf{p} - \mathbf{b}_i\rVert$ | range-beacon localization residual and Jacobian row | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |

### Statistics for experiments
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\bar e = \frac{1}{n}\sum e_i$ | mean error (bias) | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $s = \sqrt{\frac{1}{n-1}\sum (e_i - \bar e)^2}$ | sample standard deviation (spread) | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\text{RMSE} = \sqrt{\frac{1}{n}\sum e_i^2}$ | root-mean-square error | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\text{MAE} = \frac{1}{n}\sum \lvert e_i\rvert$ | mean absolute error | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\text{RMSE}^2 = \bar e^2 + \sigma^2,\ \sigma^2 = \frac{1}{n}\sum(e_i - \bar e)^2$ | error = bias² + spread² | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\sqrt{\frac{1}{T}\sum_t \lVert\hat{\mathbf{p}}_t - \mathbf{p}_t\rVert^2}$ | trajectory RMSE (absolute trajectory error) | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\text{MAD} = \operatorname{median}(\lvert e_i - \operatorname{median}(e)\rvert)$ | median absolute deviation | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $M_i = \frac{0.6745\,(e_i - \tilde e)}{\text{MAD}},\ \lvert M_i\rvert > 3.5$ | modified z-score outlier test | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\text{SE} = s/\sqrt{n}$ | standard error of the mean | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $\bar e \pm t_{0.975,\,n-1}\,\frac{s}{\sqrt n}$ | 95% confidence interval for the mean | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| $n \approx (t \cdot s/\text{half-width})^2$ | trials needed for a target CI half-width | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| upper bound $\approx 3/n$ | rule of three: 95% bound on a rate after 0 events in $n$ trials | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md), [FM.13](../optional-foundations/mathematics/FM.13-probability-basics.md) |

## First robot, electronics and robot software (00–03, FE)

### Loop timing, latency and stopping
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T = 1/f$ | loop period from loop rate | [00.01](../00-orientation/00.01-what-is-a-robot.md), [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $d_{\text{iteration}} = v\,T = \frac{v}{f}$ | distance travelled between two decisions | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $d_{\text{react}} = v\,(T + L + \tau)$ | distance travelled before a reaction takes effect | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $v = \min(0.5,\; k \cdot (d_{\text{sensed}} - 0.30))$, $v \ge 0$ | proportional slow-down controller near a wall | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $T \le \tau/10$, i.e. $f_{\text{PID}} \ge \frac{10}{\tau}$ | rate separation: a loop must run ≥ 10× faster than what it controls | [00.02](../00-orientation/00.02-the-robotics-stack.md), [00.04](../00-orientation/00.04-robot-computers.md) |
| $L = \sum_i L_i$ | total reaction latency is the sum of every stage | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $d_{\text{stop}} = v \sum_i L_i + \frac{v^2}{2a}$ | stopping distance: latency travel plus braking | [00.02](../00-orientation/00.02-the-robotics-stack.md), [00.06](../00-orientation/00.06-safety-mindset.md) |
| $v_{\max} = a\left(-L + \sqrt{L^2 + \frac{2d}{a}}\right)$ | maximum safe speed for stopping distance $d$ and latency $L$ | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $v\,t_{\text{wd}} + \frac{v^2}{2a}$ | worst-case travel after a crash, with a watchdog | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $d_{\text{brake}} = v \cdot t_{\text{latency}} + v \cdot t_{\text{stop}} + m$ | distance at which to start stopping for an obstacle | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $d_{\text{blind}} = v \cdot \left(t_{\text{network}} + t_{\text{loop}} + t_{\text{motor}}\right)$ | distance driven blind over a teleop link | [01.15](../01-first-robot/01.15-teleop-from-laptop.md) |
| $d \approx v\,\tau$ | extra distance from stale data or motor lag | [03.08](../03-robot-software/03.08-testing-robot-software.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $f_{\text{actual}} = \frac{1}{T + w + \varepsilon}$ | real rate of a naive sleep-based loop | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\varphi = 360° \cdot f \cdot \tau$ | phase lag from a pure delay | [03.04](../03-robot-software/03.04-timing-and-concurrency.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $\text{age}_n \approx P + n\,(P - T)$ | data age with an unbounded queue when processing is slower than publishing | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| $\text{age} \le P + T$ | data age with a keep-last-1 queue | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| $1/T - 1/P$ | backlog growth rate (messages per second) of an unbounded queue | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| $t_{watchdog} > k \cdot T_{command}$, $d_{coast} = v \cdot t_{watchdog}$ | watchdog timeout: long enough not to trip on jitter, short enough not to coast far | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |

### Encoders and wheel speed
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $N = \text{PPR} \times \text{gear ratio} \times 4$ | encoder counts per wheel revolution (×4 quadrature) | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $N_{ticks} = N_{cpr} \cdot G \cdot 4$ | ticks per wheel revolution | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\Delta s = \frac{2\pi r}{N}$ | distance per encoder count | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $d_{tick} = \frac{2\pi r}{N_{ticks}}$ | wheel travel per tick | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $f_{\text{edges}} = \frac{\omega}{2\pi} N$ | encoder edge rate from wheel angular speed | [00.04](../00-orientation/00.04-robot-computers.md) |
| $f_{edges} = \frac{n}{60} \cdot N_{ticks}$ | encoder edge rate from wheel RPM | [01.01](../01-first-robot/01.01-plan-the-build.md), [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $v = \frac{n}{60} \cdot 2\pi r$ | rim speed from wheel RPM | [01.01](../01-first-robot/01.01-plan-the-build.md) |
| $s = (B \ll 1) \mid A$ | 2-bit quadrature state from channels A and B | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\omega = \frac{2\pi}{N_{ticks}} \cdot \frac{\Delta n}{\Delta t}$ | wheel speed from ticks counted in a window | [01.09](../01-first-robot/01.09-reading-encoders.md), [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\hat\omega = 2\pi\,\Delta n / (N\,T)$ | wheel speed estimate assuming the loop period $T$ | [00.04](../00-orientation/00.04-robot-computers.md) |
| $(T+\delta)/T$ | error factor of a speed estimate when the loop ran $\delta$ late | [00.04](../00-orientation/00.04-robot-computers.md) |
| $\Delta\omega = \frac{2\pi}{N_{ticks}\,\Delta t}$ | speed resolution of one tick per window | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\Delta n = \frac{\omega\,\Delta t}{2\pi}\,N$ | ticks expected for a wheel speed over a time step | [03.05](../03-robot-software/03.05-hardware-abstraction.md), [03.08](../03-robot-software/03.08-testing-robot-software.md) |
| $v = \frac{\Delta\text{ticks}}{\Delta t}\cdot\frac{2\pi r}{N}$ | rim speed from ticks | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $v = \omega\,r$ | rim speed from wheel angular speed | [03.06](../03-robot-software/03.06-configuration-and-calibration.md), [03.10](../03-robot-software/03.10-networking-for-robots.md) |
| $v \approx k\,(d - d_0)$ | duty-to-speed fit (inverse: $d = v/k + d_0$) | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $r = D/(10 \cdot 2\pi)$ | effective wheel radius from a 10-turn roll | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |

### Differential drive and odometry
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v = \frac{v_L + v_R}{2}$, $\omega = \frac{v_R - v_L}{b}$ | robot speed and turn rate from the two wheel speeds | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $R = v/\omega$ | radius of the circle the robot drives | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $\omega = 2 v_R / b$ (with $v_L = -v_R$) | turn rate when rotating in place | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $d = r\,\phi$ | wheel distance from wheel rotation | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\Delta\theta = \frac{d_R - d_L}{b}$ | heading change from the two wheel distances | [01.06](../01-first-robot/01.06-mechanical-assembly.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md), [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\Delta\theta = \frac{(r_R - r_L)\phi}{b}$ | heading error caused by unequal wheel radii | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $L\,\Delta\theta/2$ | sideways offset after driving length $L$ with heading error $\Delta\theta$ | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\theta\,b/2$ | arc each wheel travels to turn $\theta$ in place | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $(R_{turn} \pm b/2)\,\theta$ | inner / outer wheel arcs on a turn of radius $R_{turn}$ | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s = \frac{d_L + d_R}{2}$ | distance travelled by the robot centre | [01.09](../01-first-robot/01.09-reading-encoders.md), [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $d_k = \frac{\pi r}{N}\left(\Delta n^{L}_k + \Delta n^{R}_k\right)$ | centre distance per step from tick deltas | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $\sum_{i=1}^{4}\left[\text{translate}(s) \cdot \text{rotate}(\tfrac{\pi}{2})\right] = \text{identity}$ | a perfect square closes on itself | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s\,\epsilon$ | end-position error per remaining side from a corner heading error $\epsilon$ | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $b_{\text{true}} = b \times 90° / \theta_{\text{measured}}$ | track-width calibration from a measured 90° turn | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\Delta d = d \cdot e_r$ | distance error from a relative radius error | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\Delta\theta = \theta \cdot e_b$ | heading error from a relative wheelbase error | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $v_{\max}^{\text{wheel}} = \omega_{\max} \, r$, $\omega_{\max}^{\text{body}} = \frac{2 v_{\max}^{\text{wheel}}}{b}$ | top robot speed and top rotation rate | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |

### Chassis mechanics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\delta_{total} = \arctan\frac{g_{rear} - g_{front}}{s}$ | total toe angle of the wheels | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\sin(\delta)$, $1/\cos(\delta) - 1$ | sideways scrub per metre / odometry error from toe | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $f_{wheels} = \frac{x_m - x_c}{x_a - x_c}$ | fraction of weight on the drive wheels | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $a > g \frac{\ell}{h}$ | acceleration at which the robot tips onto its tail | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $E_k = \tfrac{1}{2} m v^2$ | kinetic energy of a moving body | [00.06](../00-orientation/00.06-safety-mindset.md) |

### Sensors
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = \frac{c \cdot t}{2}$ | distance from echo round-trip time (ultrasonic, ToF) | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md), [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $c = 331.3 + 0.606\,T$ | speed of sound in air vs temperature | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $\text{gap} \approx R\,\Delta\theta$ | spacing between neighbouring LiDAR points | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $\Delta\psi = b\,t$ | heading error from a constant gyro bias | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $V = \frac{N}{65535} \cdot 3.3\ \text{V}$, $T = 27 - \frac{V - 0.706}{0.001721}$ | Pico internal temperature sensor | [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md) |

### Electricity basics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $R = \frac{V}{I}$, $V = I \cdot R$, $I = \frac{V}{R}$ | Ohm's law | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $P = V \cdot I$ | electrical power | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $P = I^2 R = \frac{V^2}{R}$ | power in a resistor (wire heating) | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [00.06](../00-orientation/00.06-safety-mindset.md) |
| $E = P \cdot t$, $E = V_{nominal} \cdot Q$ | energy from power and time; battery energy | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $R_{series} = R_1 + R_2 + \dots$ | resistors in series | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $\frac{1}{R_{parallel}} = \frac{1}{R_1} + \frac{1}{R_2} + \dots$; two: $\frac{R_1 R_2}{R_1 + R_2}$ | resistors in parallel | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md) |
| $R = \frac{V_{supply} - V_F}{I_{LED}}$ | LED series resistor | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $V_{out} = V_{in} \cdot \frac{R_{bottom}}{R_{top} + R_{bottom}}$ | voltage divider | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $k = \frac{R_{bottom}}{R_{top}+R_{bottom}}$ | divider ratio | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md) |
| $R_{top} \parallel R_{bottom}$ | divider source (Thevenin) resistance | [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $V_{out} = V_{source} \cdot \frac{R_{bottom}(1 \pm \tau)}{R_{top}(1 \pm \tau) + R_{bottom}(1 \pm \tau)}$ | divider output at every tolerance corner | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $(V_f - 3.8)/R$ | fault current into the 3.3 V rail through the clamp diode | [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $I = \frac{3.3}{R_{pull}}$ | current through a pull-up while the line is pulled low | [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $Q = m \cdot c \cdot \Delta T$ | heat needed to warm a mass of metal | [01.03](../01-first-robot/01.03-workbench-and-tools.md), [FE.17](../optional-foundations/electronics/FE.17-soldering.md) |

### Wiring, voltage drop and grounding
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $V_{Pi} = V_{supply} - I \cdot 2 L \cdot R'$ | voltage left at the Pi after a two-conductor cable | [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) |
| $R = 2 L R'$, $\Delta V = I R$, $P = I^2 R$ | lead resistance (two conductors of length $L$), drop and heat | [01.07](../01-first-robot/01.07-power-system.md), [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) |
| $V_{offset} = I_{branch} \cdot R_{branch}$ | ground offset from current in a shared wire | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_\text{shift} = I \cdot R_\text{wire}$, $R_\text{wire} = \rho_\text{AWG} \cdot L + R_\text{contacts}$ | ground shift in a return wire | [FE.16](../optional-foundations/electronics/FE.16-grounding.md) |
| $d = 0.127 \times 92^{(36-n)/39}$ mm | wire diameter from AWG number | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $R/\ell = \rho_{Cu}/A$ | wire resistance per metre | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $V = I \cdot \frac{R}{\ell} \cdot 2\ell$, $I_{max} \ge 1.25\,I_{worst}$, $I_{max} \ge I_{fuse}$ | harness checks: drop, heat margin, fuse protects the wire | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| rating × 0.8 | derate connectors to 80 % of their rated current | [02.07](../02-robot-electronics/02.07-wiring-harness.md) |
| $V = L \frac{di}{dt}$ | inductive voltage spike (ground bounce) | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md), [02.08](../02-robot-electronics/02.08-from-breadboard-to-perfboard.md) |
| $M \approx \frac{\mu_0 \ell}{2\pi} \ln\!\left(\frac{d + w}{d}\right)$, $V_{induced} = M \frac{di}{dt}$ | magnetic coupling into a signal pair | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $C V^2 f$ | power wasted charging a capacitor every PWM cycle | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| $\text{walk} = O(n)$, $\text{bisect} = O(\log_2 n)$ | measurements needed to find a fault by walking vs bisecting | [02.09](../02-robot-electronics/02.09-electrical-debugging-method.md) |

### Power budget, regulators and fuses
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $I_{battery} = \frac{V_{out} \cdot I_{out}}{\eta \cdot V_{battery}}$ | battery current of a switching regulator | [01.01](../01-first-robot/01.01-plan-the-build.md), [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $I_{in} = \frac{P_{out}}{\eta \, V_{in}}$ | constant-power regulator input current | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $P_\text{heat} = P_\text{out}\left(\frac{1}{\eta} - 1\right)$ | heat in a switching regulator | [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $P_\text{heat} = (V_\text{in} - V_\text{out}) \cdot I_\text{load}$, $\eta = \frac{V_\text{out}}{V_\text{in}}$ | heat and efficiency of a linear regulator | [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $V_\text{out} \approx D \cdot V_\text{in}$, $D \approx \frac{V_\text{out}}{V_\text{in}}$ | buck converter duty | [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md) |
| $I_{worst} = 2 I_{limit} + \frac{P_{5V}}{\eta \, V_{bat}}$ | worst-case battery current (sizes the fuse) | [01.07](../01-first-robot/01.07-power-system.md) |
| $I_{peak} = \frac{V}{R}$, $I^2t = \int i^2\,dt = \frac{V^2 C}{2R}$ | switch-on inrush peak and fuse stress | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_{bus} = V_{rest} - I \cdot R_{path}$ | bus voltage sag under load | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $R_{source} = \frac{V_{rest} - V_{load}}{I_{load} - I_{rest}}$ | measured source resistance | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $I_{reverse} \approx \frac{V + E}{R} \approx \frac{2V}{R}$ | current when reversing a motor at full speed | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $C = I\,t/\Delta V$ | capacitance needed to hold up a load for time $t$ | [02.02](../02-robot-electronics/02.02-power-budget.md) |

### Batteries
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $E = n_{\text{cells}} \cdot V_{\text{cell}} \cdot C$ | battery energy | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $E_b = V \cdot C$ | battery energy (Wh; ×3600 for J) | [00.06](../00-orientation/00.06-safety-mindset.md) |
| $V_\text{pack} = S \cdot V_\text{cell}$, $C_\text{pack} = P \cdot C_\text{cell}$, $E = V_\text{nominal} \cdot C_\text{pack}$ | pack voltage, capacity and energy from its SxP layout | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $I = C_\text{rate} \times C_\text{pack}$ | current from a C-rating | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $V_{\text{terminal}} = V_{\text{open circuit}} - I \cdot R_{\text{internal}}$ | terminal voltage sag under load | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $R = (V_\text{rest} - V_\text{load}) / I$ | measured internal resistance | [FE.13](../optional-foundations/electronics/FE.13-batteries.md), [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $t_{\text{run}} \approx \frac{k \cdot E}{P_{\text{avg}}}$ | runtime estimate | [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md) |
| $t \approx \frac{C_{Ah} \cdot V_{nom} \cdot f_{usable}}{P_{avg}}$ | runtime estimate | [02.02](../02-robot-electronics/02.02-power-budget.md), [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $t = \text{capacity} / \text{current}$ (× 0.8 usable) | runtime from current | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $\sum I\,\Delta t$ | coulomb counting: charge used | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $t \approx \frac{C \cdot (0.8 - SoC_\text{now})}{I_\text{CC}} + t_\text{CV}$ | charging time to 80 % | [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md) |

### Battery and current measurement
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $V_{ADC} = V_{bat}\frac{22}{100 + 22}$ | karmel's battery divider at the ADC pin | [01.07](../01-first-robot/01.07-power-system.md) |
| $V_{\text{pin}} = V_{\text{batt}}\cdot\frac{R_b}{R_t + R_b}$, $V_{\text{batt}} = V_{\text{pin}}\cdot\frac{R_t + R_b}{R_b}$ | battery divider and its inverse | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{\text{pin}} = \frac{N}{65535}\cdot 3.3$ | ADC raw reading to pin voltage | [01.14](../01-first-robot/01.14-battery-monitoring.md), [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $I = \frac{V_{\text{shunt}}}{R_{\text{shunt}}}$ | current from a shunt resistor | [01.14](../01-first-robot/01.14-battery-monitoring.md) |

### ADC
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $V_{pin} = \text{raw}_{u16} \cdot \frac{V_{ref}}{65535}$ | ADC reading to volts | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{ref}/2^{\text{bits}}$ | size of one ADC step | [FE.08](../optional-foundations/electronics/FE.08-adc.md), [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md) |
| $\sigma_{avg} = \frac{\sigma}{\sqrt{N}}$ | noise reduction by averaging | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| $V_{true} = g \cdot V_{adc} + o$, $g = \frac{V_{true,hi} - V_{true,lo}}{V_{adc,hi} - V_{adc,lo}}$, $o = V_{true,lo} - g \cdot V_{adc,lo}$ | two-point ADC calibration | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |

### PWM and motor drivers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T = \frac{1}{f}$, $D = \frac{t_{on}}{T}$, $V_{avg} = D \cdot V_{high}$ | PWM period, duty and average voltage | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\bar{V} = D \cdot V_{bus}$ | average motor voltage under PWM | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\tau \frac{d\omega}{dt} = u(t) - \omega$ | first-order motor speed model driven by PWM | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\Delta\omega_{pp} = \frac{(1 - e^{-DT/\tau})(1 - e^{-(1-D)T/\tau})}{1 - e^{-T/\tau}} \approx D(1-D)\,T/\tau$ | speed ripple from PWM | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\text{duty\_u16} = \text{round}(\lvert m \rvert \times 65535 / 1000)$ | protocol command to Pico PWM register | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\lfloor n/2 \rfloor \bmod 8$ | PWM slice of GPIO $n$ on the Pico 2 | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\tau_e = L/R$; $\tau_e = \frac{L_M}{R_M + R_{DS(on)}}$ | electrical time constant of the motor winding | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $\Delta I = \frac{V_{bus}\,D(1-D)}{f\,L}$ | PWM current ripple | [01.08](../01-first-robot/01.08-first-motor-spin.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $D_{applied} = \operatorname{sign}(D)\left[d + \lvert D \rvert\,(1 - d)\right]$, $D \ne 0$ | deadband compensation | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $I_{limit} = \frac{V_{SLEEP}}{R_{CS} \cdot A_{IPROPI}}$ | DRV8874 carrier current limit | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $I_{TRIP} = \frac{V_{VREF}}{R_{IPROPI} \cdot A_{IPROPI}}$ | DRV8874 current-limit trip point | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md), [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $A_{IPROPI} \cdot R_{IPROPI}$ | current-sense scale (V per A) | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $I_{INA} = a \cdot V_{ADC} + b$ | current-sense calibration fit | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $P = I^2 R_{DS(on)} + \tfrac12 V_M I (t_r + t_f) f + 2 V_{SD} I \, t_{dead} f$ | driver losses: conduction + switching + dead time | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $P = I^2 \, R_{DS(on)}$ | heat in a MOSFET switch | [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md), [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md) |
| $P = V_\text{drop} \cdot I$ | heat in a bipolar (L298) driver | [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md) |
| $T_J = T_A + P\,R_{\theta JA}$ | chip junction temperature | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V = \sqrt{V_0^2 + 2 \eta E/C}$ | bus voltage after braking energy is dumped into the capacitor | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_M + 2V_{SD}$ | voltage opposing the current in coast (body diodes) | [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |

### DC motors, gears, servos and steppers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $V = I R + k\,\omega$ | DC motor electrical equation (back-EMF $k\omega$) | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\tau = k\,(I - I_0)$ | DC motor torque | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $I_\text{stall} = \frac{V}{R}$, $\omega_\text{no-load} \approx \frac{V - I_0 R}{k}$ | stall current and no-load speed | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $k = \frac{V - I_0 R}{\omega_\text{no-load}}$ | motor constant from measurements | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $I = (V - k\omega)/R$ | motor current at speed $\omega$ | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $P = \tau\,\omega$ | mechanical power (peaks at half no-load speed) | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\omega_\text{out} = \frac{\omega_\text{motor}}{N}$, $\tau_\text{out} = \tau_\text{motor} \cdot N \cdot \eta$ | gearbox speed and torque | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $1\ \text{kg·cm} = 0.0981\ \text{N·m}$ | hobby torque unit conversion | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $k_t = \frac{60}{2\pi\,K_v}$ | torque constant from Kv | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\text{steps per mm} = \frac{\text{full steps per turn} \times \text{microsteps}}{\text{mm per turn}}$ | stepper resolution | [FE.11](../optional-foundations/electronics/FE.11-servos-and-steppers.md) |
| $\text{ticks} \times 360/4096$ | smart-servo position to degrees | [FE.11](../optional-foundations/electronics/FE.11-servos-and-steppers.md) |

### Logic levels and buses
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $NM_H = V_{OH,min} - V_{IH,min}$, $NM_L = V_{IL,max} - V_{OL,max}$ | noise margins of a logic link | [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md), [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md) |
| $V_{IH} = 0.7\,V_S$, $V_{IL} = 0.3\,V_S$ | CMOS thresholds as fractions of supply | [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md) |
| $R_{p,min} = \frac{V_{DD} - V_{OL,max}}{I_{OL}}$ | smallest I²C pull-up | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $t_r = 0.8473 \cdot R_p \cdot C_b$, $R_{p,max} = \frac{t_r}{0.8473 \cdot C_b}$ | I²C rise time and largest pull-up | [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $\tau = RC$, 10–90 % rise ≈ $2.2\tau$ | RC rise time of a pulled-up line | [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md) |
| $t_{bit} = \frac{1}{\text{baud}}$, $\text{bytes/s} = \frac{\text{baud}}{10}$ | UART bit time and throughput (8N1) | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| $n_{bits} = (34 + 8P) + \left\lfloor \frac{34 + 8P - 1}{4} \right\rfloor + 13$ | worst-case bits in a classic CAN frame | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| $\ell \times \text{rate} < 10^7$ | cable length × bit rate rule (m·bit/s) | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| $R = \frac{120 \times 120}{120 + 120} = 60\ \Omega$ | CAN bus with two termination resistors | [FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md) |

### Data integrity and compute cost
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $c = b_1 \oplus b_2 \oplus \dots \oplus b_n$ | XOR checksum | [03.03](../03-robot-software/03.03-robust-serial-communication.md), [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $P_{undetected} \approx \frac{1}{2^8}$ (CRC-16: $\frac{1}{2^{16}}$) | chance random corruption passes an 8-bit check | [01.10](../01-first-robot/01.10-pi-pico-protocol.md), [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $P_{\text{miss}} = \frac{8\binom{L}{2}}{\binom{8L}{2}} \approx \frac{1}{8}$ | chance two bit flips pass the XOR checksum | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $g(x) = x^{16} + x^{12} + x^5 + 1$ | CRC-16-CCITT generator polynomial (0x1021) | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $\lceil n/254 \rceil$ | COBS framing overhead in bytes | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| $N_{\text{rays}} = M \times B$ | raycasts per particle-filter update | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $O(N_{\text{rays}} \times S)$ | cost of raycasting in a world with $S$ segments | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |
| $O(N \cdot M)$ vs $O(M \log M) + O(N \log M)$ | brute-force vs KD-tree nearest-neighbour search | [03.01](../03-robot-software/03.01-python-robotics-ecosystem.md) |

### Capacitors, inductors and diodes
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $Q = C V$, $I = C \frac{dV}{dt}$, $E = \tfrac{1}{2} C V^2$, $\tau = R C$ | capacitor basics | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $\Delta V = \frac{I \cdot \Delta t}{C}$ | bulk capacitor droop during a current burst | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md), [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $f_c = \frac{1}{2\pi R C}$ | RC low-pass filter cutoff | [FE.08](../optional-foundations/electronics/FE.08-adc.md), [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $V_L = L \frac{dI}{dt}$, $E_L = \tfrac{1}{2} L I^2$ | inductor voltage and stored energy | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $I\sqrt{L/C}$ | peak voltage when an inductor's current has nowhere to go | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |
| $\tau = L/R$ | RL decay time constant (flyback) | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md) |
| $P = V_F \cdot I$ | heat in a diode | [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md) |

### Shopping and planning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $C_{ILS} = (P + S)\cdot k$ if $P \le 75$; $(P + S)\cdot 1.18 \cdot k$ if $75 < P \le 500$ | landed cost of an import into Israel | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $L > 1.18 \times k \times (P + S)$ | when importing over the limit beats a local price | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| $T_{\text{total}} = \sum_{\text{main}} t_i + \sum_{\text{foundations taken}} t_j + \sum_{\text{projects}} t_k$ | total course time | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |
| $W = \frac{T}{h}$ | weeks to finish at $h$ hours per week | [00.07](../00-orientation/00.07-how-to-use-this-course.md) |

## ROS 2, frames, simulation, sensors and physics (04–07, FP)

### ROS 2 middleware and networking
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{discovery multicast} = PB + DG \cdot d$ | UDP port for multicast discovery in domain $d$ | [04.01](../04-ros2/04.01-why-middleware.md) |
| $\text{discovery unicast}(p) = PB + DG \cdot d + 10 + PG \cdot p$ | UDP port for unicast discovery of participant $p$ | [04.01](../04-ros2/04.01-why-middleware.md) |
| $\text{data unicast}(p) = PB + DG \cdot d + 11 + PG \cdot p$ | UDP port for user data of participant $p$ | [04.01](../04-ros2/04.01-why-middleware.md) |
| $(1-\text{loss})^{\text{fragments}}$ | chance a large message arrives when every fragment must arrive | [04.01](../04-ros2/04.01-why-middleware.md) |
| $P(\text{sample arrives}) = (1-p)^k$ | chance a $k$-fragment message survives packet loss $p$ (best effort) | [04.11](../04-ros2/04.11-qos.md) |
| $T = 1/f$ | period from rate | [04.11](../04-ros2/04.11-qos.md) |
| $\text{age} \approx (N - 1)\,T$ | age of the message a slow subscriber processes with queue depth $N$ | [04.11](../04-ros2/04.11-qos.md) |
| $1 - \dfrac{1/w}{f}$ | fraction of messages a slow subscriber never processes (any depth) | [04.11](../04-ros2/04.11-qos.md) |
| $\text{offset}_{k} = \left\lceil \frac{\text{end}_{k-1}}{a_k} \right\rceil a_k, \qquad \text{end}_k = \text{offset}_k + \text{size}_k$ | struct field layout with alignment padding | [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md) |
| $\sum_i f_i \cdot (s_i + o)$ | rosbag recording size in bytes per second | [04.14](../04-ros2/04.14-rosbag2.md) |
| $\text{bits/s} = W \times H \times C \times \text{bytes} \times 8 \times \text{fps}$ | raw camera bandwidth | [07.08](../07-sensors/07.08-rgb-cameras.md) |

### ROS 2 timing, executors and builds
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T_\text{wall} \approx \max_{\text{paths } p} \sum_{k \in p} t_k$ | parallel build time = longest dependency chain (critical path) | [04.05](../04-ros2/04.05-packages-and-workspaces.md) |
| $n_\text{missed} \approx \lfloor T_b \cdot f \rfloor$ | timer callbacks missed while a callback blocks for $T_b$ | [04.07](../04-ros2/04.07-services.md) |
| $\max(0, \lfloor T_b f \rfloor - d)$ | messages dropped when the queue of depth $d$ overflows during blocking | [04.07](../04-ros2/04.07-services.md) |
| $d = v \cdot T_b$ | distance driven unsupervised while a node is blocked | [04.15](../04-ros2/04.15-lifecycle-nodes.md) |
| $\rho = \sum_i f_i\,w_i$ | utilization of a single-threaded executor | [04.12](../04-ros2/04.12-executors-and-callbacks.md) |
| $f \approx 20(1-w) + 1$ | effective rate of a 20 Hz timer sharing a thread with a 1 Hz callback taking $w$ s | [04.12](../04-ros2/04.12-executors-and-callbacks.md) |
| $t_{\text{react}} = t_{\text{transport}} + t_{\text{queue}} + t_{\text{filter lag}}$ | end-to-end reaction latency budget | [04.13](../04-ros2/04.13-debugging-and-introspection.md) |
| $T_s = f \, T_w$ | simulated time elapsed for wall time $T_w$ at real-time factor $f$ | [06.04](../06-simulation/06.04-bridging-gazebo-ros2.md) |

### Battery monitoring and filtering
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $V(t) = \max\big(V_\text{floor},\; V_0 - k\,t\big) + \varepsilon, \qquad \varepsilon \sim \mathcal{N}(0, \sigma^2)$ | simulated battery: linear discharge plus Gaussian noise | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $P(\varepsilon > h) = P\!\left(z > \tfrac{h}{\sigma}\right)$ | chance noise alone crosses a hysteresis margin $h$ | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $h \geq 5\sigma$ | rule of thumb: hysteresis at least 5 noise standard deviations | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md) |
| $y \leftarrow \alpha x + (1-\alpha) y$ | exponential moving average (low-pass filter) update | [04.04](../04-ros2/04.04-topics-publishers-subscribers.md), [04.09](../04-ros2/04.09-parameters.md) |
| $y_k = \alpha x_k + (1-\alpha) y_{k-1}$ | exponential moving average, indexed form | [04.09](../04-ros2/04.09-parameters.md) |
| $(1-\alpha)^k$ | weight still left on an old value after $k$ filter steps | [04.09](../04-ros2/04.09-parameters.md) |
| $k = \frac{\ln 0.05}{\ln(1-\alpha)}$ | filter steps until 95 % of a step change is followed | [04.09](../04-ros2/04.09-parameters.md) |
| $(1-\alpha)/\alpha$ | average lag of an EMA filter, in samples | [04.10](../04-ros2/04.10-launch-files.md), [04.13](../04-ros2/04.13-debugging-and-introspection.md) |
| $V_{min} = 3.0\,n, \qquad V_{max} = 4.25\,n$ | safe voltage range of an $n$-cell Li-ion pack | [04.09](../04-ros2/04.09-parameters.md) |
| $V_{crit} + h \le V_{low} < V_{full}$ | valid ordering of battery thresholds | [04.09](../04-ros2/04.09-parameters.md) |
| $1 - \frac{V_{\text{cutoff}}}{V_{\text{full}}}$ | fraction of top speed lost as the battery drains (no speed control) | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |

### Kinematics: motion in a line
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v(t) = \frac{dx}{dt}, \qquad a(t) = \frac{dv}{dt}$ | speed and acceleration as derivatives | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $x(t) = x_0 + \int_0^t v(\tau)\, d\tau$ | position as the integral of speed | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v_k \approx \frac{x_k - x_{k-1}}{\Delta t}, \qquad x_k \approx x_{k-1} + v_k \Delta t$ | discrete derivative and integral (what code does) | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v = v_0 + a t$ | speed under constant acceleration | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $x = x_0 + v_0 t + \tfrac{1}{2} a t^2$ | position under constant acceleration | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v^2 = v_0^2 + 2 a (x - x_0)$ | speed vs distance under constant acceleration | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $d_{brake} = \frac{v_0^2}{2\lvert a \rvert}$ | braking distance | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $d_\text{stop} = \frac{v^2}{2a}, \qquad t_\text{stop} = \frac{v}{a}$ | stopping distance and time | [04.08](../04-ros2/04.08-actions.md) |
| $v t_{lat} + v^2/(2a) = d - \text{margin}$ | safe top speed given latency and available distance | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $t_a = v_{max}/a_{max}$ | acceleration time of a trapezoidal profile | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $v_{peak} = \sqrt{a_{max} d}$ | peak speed of a triangular profile over distance $d$ | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\sigma_v = \frac{\sqrt{2}\,\sigma_x}{\Delta t}$ | noise on speed computed by differencing noisy positions | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $d_n = \min(D, n \, v \, \Delta t)$ | distance after $n$ control steps toward goal $D$ | [04.08](../04-ros2/04.08-actions.md) |
| $N = \lceil D / (v \Delta t) \rceil$ | number of control steps to reach goal $D$ | [04.08](../04-ros2/04.08-actions.md) |
| $z(t) = z_0 - \tfrac{1}{2} g t^2$ | height of a falling object | [06.03](../06-simulation/06.03-gazebo-basics.md) |
| $t_{\text{land}} = \sqrt{\frac{2h}{g}}$ | fall time from height $h$ | [06.03](../06-simulation/06.03-gazebo-basics.md) |

### Wheels, encoders and differential drive
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v = \omega r$ | linear speed of a wheel rim | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md), [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\omega = \text{RPM} \times \frac{2\pi}{60}$ | rpm to rad/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\Delta s_{tick} = \frac{2\pi r}{N_{ticks}}$ | distance per encoder tick | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md), [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $N = \text{PPR} \times \text{gear ratio} \times 4$ | encoder counts per wheel revolution with quadrature (e.g. $11 \times 56 \times 4$) | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $\Delta v_{min} = \Delta s_{tick} \cdot f_s$ | speed resolution when counting ticks per sample | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $\Delta\omega = \frac{2\pi}{N\,T}$ | speed resolution of tick counting over window $T$ | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\frac{\Delta\omega}{\omega} = \frac{\tau N \omega}{2\pi}$ | relative error of timing between ticks with timer resolution $\tau$ | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\omega_{\times} = \frac{2\pi}{N\sqrt{T\tau}}$ | crossover speed between tick-timing and tick-counting | [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $\omega_{\text{left}} = \frac{v - \omega L/2}{R} \qquad \omega_{\text{right}} = \frac{v + \omega L/2}{R}$ | inverse kinematics: wheel speeds from robot command | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [06.06](../06-simulation/06.06-ros2-control.md) |
| $v = R(\omega_R + \omega_L)/2, \qquad \omega = R(\omega_R - \omega_L)/L$ | forward kinematics: robot motion from wheel speeds | [06.01](../06-simulation/06.01-why-simulate.md) |
| $\Delta s = \frac{d_R + d_L}{2}, \quad \Delta\theta = \frac{d_R - d_L}{L}, \quad r = \frac{\Delta s}{\Delta\theta}$ | odometry step from wheel distances | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md) |
| $x \mathrel{+}= r\left[\sin(\theta + \Delta\theta) - \sin\theta\right], \qquad y \mathrel{-}= r\left[\cos(\theta + \Delta\theta) - \cos\theta\right]$ | exact arc odometry update | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $x \mathrel{+}= \Delta s \cos\theta$ | straight-line odometry update (when $\Delta\theta = 0$) | [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md) |
| $r = v/\omega$ | turning radius of a constant command | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $\theta = \omega t$ | heading after turning at constant rate (from $\theta_0 = 0$) | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $\Delta x = r \sin\theta, \qquad \Delta y = r\,(1 - \cos\theta)$ | displacement along an arc, in the starting frame | [04.03](../04-ros2/04.03-nodes-and-cli.md) |
| $x \mathrel{+}= v\cos\theta\,dt, \quad y \mathrel{+}= v\sin\theta\,dt, \quad \theta = \operatorname{wrap}(\theta + \omega\,dt)$ | unicycle model integration step | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\omega = 2.0\,e, \qquad v = \min(0.3, 0.5\,d)$ | simple go-to-goal controller (turn toward goal, slow near it) | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |

### 2D pose and angles
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\operatorname{wrap}(a) = \pi - \big((\pi - a) \bmod 2\pi\big)$ | fold an angle into $(-\pi, \pi]$ | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\beta = \operatorname{atan2}(g_y - y,\ g_x - x)$ | bearing from the robot to a goal | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\sqrt{(g_x - x)^2 + (g_y - y)^2}$ | distance to the goal | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $e = \operatorname{wrap}(\beta - \theta)$ | heading error to the goal | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\bar\theta = \operatorname{atan2}\Big(\sum_i \sin\theta_i,\ \sum_i \cos\theta_i\Big)$ | circular mean of angles | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\operatorname{atan2}(h_1 \times h_2,\ h_1 \cdot h_2) = \operatorname{atan2}(\sin(\theta_2-\theta_1), \cos(\theta_2-\theta_1))$ | signed angle between two headings | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $\theta = \operatorname{wrap}(90° - \text{compass})$ | compass bearing (clockwise from north) to ROS heading (counter-clockwise from east) | [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md) |
| $(x, y) = (r\cos\phi,\ r\sin\phi)$ | polar to Cartesian | [05.01](../05-frames-and-transforms/05.01-why-frames.md), [07.07](../07-sensors/07.07-2d-lidar.md) |

### Rigid transforms (2D)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $p_{\text{to}} = R\,p_{\text{from}} + t$ | change a point's frame: rotate then translate | [05.01](../05-frames-and-transforms/05.01-why-frames.md) |
| $R(\theta) = \begin{bmatrix} c & -s \\ s & c \end{bmatrix}$ | 2D rotation matrix | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{a}p = {}^{a}T_{b}\;{}^{b}p$ | apply a transform to a point | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{a}p = R(\theta)\,{}^{b}p + t, \qquad t = (x, y)$ | transform a 2D point by pose $(x, y, \theta)$ | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{a}T_{c} = {}^{a}T_{b}\;{}^{b}T_{c}$ | compose transforms (inner frame names cancel) | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{b}T_{a} = ({}^{a}T_{b})^{-1}$ | inverse transform reverses direction | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $x = x_1 + c_1 x_2 - s_1 y_2, \qquad y = y_1 + s_1 x_2 + c_1 y_2, \qquad \theta = \operatorname{wrap}(\theta_1 + \theta_2)$ | compose two 2D poses | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $x' = -c\,x - s\,y, \qquad y' = s\,x - c\,y, \qquad \theta' = -\theta$ | invert a 2D pose | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{a}p = R_1({R_2\,{}^{c}p + t_2}) + t_1 = (R_1R_2)\,{}^{c}p + (R_1t_2 + t_1)$ | why composition multiplies rotations and rotates the second translation | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R_1R_2 = R(\theta_1+\theta_2)$ | 2D rotations add angles | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R^{-1} = R^{\mathsf T} = R(-\theta)$ | inverse of a rotation is its transpose | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{b}p = R^{\mathsf T}\,{}^{a}p - R^{\mathsf T}t$ | transform a point back (inverse) | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| $R_1t_2 + t_1 \ne R_2t_1 + t_2$ | transforms do not commute: order matters | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{M}T_{B}\;\Delta$ vs $\Delta\;{}^{M}T_{B}$ | right-multiply = move in the robot's own frame; left-multiply = move in the map frame | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |
| ${}^{A}T_{B'} = ({}^{M}T_{A})^{-1}\,{}^{M}T_{B'}$ | relative pose between two poses known in the same frame | [05.03](../05-frames-and-transforms/05.03-rigid-transforms-2d.md) |

### Homogeneous transforms and 3D rotations
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $(x, y) \to (x, y, 1)$; direction $(x, y, 0)$ | homogeneous coordinates: $w=1$ point, $w=0$ direction | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $\begin{bmatrix} R_{3\times3} & t_{3\times1} \\ 0\ 0\ 0 & 1\end{bmatrix}$ | 4×4 homogeneous transform (3×3 in 2D) | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $\begin{bmatrix} R_1 & t_1 \\ 0 & 1\end{bmatrix}\begin{bmatrix} R_2 & t_2 \\ 0 & 1\end{bmatrix} = \begin{bmatrix} R_1R_2 & R_1t_2 + t_1 \\ 0 & 1\end{bmatrix}$ | composition as one matrix multiply | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $\begin{bmatrix} R & t \\ 0 & 1\end{bmatrix}^{-1} = \begin{bmatrix} R^{\mathsf T} & -R^{\mathsf T}t \\ 0 & 1\end{bmatrix}$ | inverse of a homogeneous transform | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| ${}^{M}T_{K} = {}^{M}T_{F}\,{}^{F}T_{B}\,{}^{B}T_{C}\,{}^{C}T_{K}$ | transform chain map to camera optical frame | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $R^{\mathsf T}R = I, \qquad \det R = +1$ | test that a matrix is a valid rotation | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $R = U\Sigma V^{\mathsf T} \Rightarrow R_{\text{fixed}} = UV^{\mathsf T}$ | re-orthonormalize a drifted rotation via SVD | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $x \times y = z$ | right-handed axes | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| $R_x(r) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_r & -s_r \\ 0 & s_r & c_r \end{bmatrix},\quad R_y(p) = \begin{bmatrix} c_p & 0 & s_p \\ 0 & 1 & 0 \\ -s_p & 0 & c_p \end{bmatrix},\quad R_z(y) = \begin{bmatrix} c_y & -s_y & 0 \\ s_y & c_y & 0 \\ 0 & 0 & 1 \end{bmatrix}$ | elementary rotations about x, y, z | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $R = R_z(\text{yaw})\;R_y(\text{pitch})\;R_x(\text{roll})$ | ROS / URDF roll-pitch-yaw convention | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md), [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $\text{pitch} = \arcsin(-R_{31}),\qquad \text{roll} = \operatorname{atan2}(R_{32}, R_{33}),\qquad \text{yaw} = \operatorname{atan2}(R_{21}, R_{11})$ | roll-pitch-yaw from a rotation matrix (valid for $\lvert\text{pitch}\rvert < 90°$) | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $q = \big(n_x \sin\tfrac{\theta}{2},\ n_y \sin\tfrac{\theta}{2},\ n_z \sin\tfrac{\theta}{2},\ \cos\tfrac{\theta}{2}\big), \qquad x^2+y^2+z^2+w^2 = 1$ | quaternion from axis and angle (ROS order x, y, z, w) | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $q = (0, 0, \sin\frac{\text{yaw}}{2}, \cos\frac{\text{yaw}}{2})$ | quaternion for a pure yaw | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $\text{yaw} = \operatorname{atan2}\big(2(wz + xy),\ 1 - 2(y^2 + z^2)\big)$ | yaw from a quaternion | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $q \equiv -q$ | a quaternion and its negative are the same rotation | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $\lvert q_1\cdot q_2 \rvert \approx 1$ | test that two quaternions are (nearly) the same rotation | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $q \leftarrow -q$ if $q\cdot q_{\text{ref}} < 0$ | keep quaternion sign continuous over time | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |
| $R(q_1 q_2) = R(q_1)\,R(q_2)$ | quaternion product composes rotations | [05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md) |

### Frames, tf2 and URDF
| Formula | Name / what it gives | Lesson |
|---|---|---|
| ${}^{M}T_{O} = {}^{M}\hat T_{B}\;\big({}^{O}T_{B}\big)^{-1}$ | map to odom correction published by localization | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| ${}^{M}T_{O}\,{}^{O}T_{B} = {}^{M}\hat T_{B}$ | map pose = correction times odometry pose | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| $T = \big({}^{F}T_{\text{target}}(t_{\text{target}})\big)^{-1}\;{}^{F}T_{\text{source}}(t_{\text{source}})$ | tf2 time travel: transform between frames at two different times via fixed frame $F$ | [05.07](../05-frames-and-transforms/05.07-tf2-broadcast-listen.md) |
| ${}^{P}T_{C}(q) = \begin{bmatrix} R_z(y)R_y(p)R_x(r) & \mathbf{t} \\ \mathbf{0} & 1\end{bmatrix} \cdot M(\hat{\mathbf{a}}, q)$ | URDF joint transform: fixed origin times joint motion | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $M = \begin{bmatrix} \text{Rot}(\hat{\mathbf{a}}, q) & \mathbf{0}\\ \mathbf{0}&1\end{bmatrix}$ | motion of a revolute / continuous joint | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $M = \begin{bmatrix} I & q\hat{\mathbf{a}}\\ \mathbf{0}&1\end{bmatrix}$ | motion of a prismatic joint | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| ${}^{A}T_{B} = \big({}^{\text{root}}T_{A}\big)^{-1}\,{}^{\text{root}}T_{B}$ | transform between any two links in a tree | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| $e \approx r\,\omega\,\Delta t + v\,\Delta t$ | error from using a transform $\Delta t$ out of date (rotation dominates) | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |

### Cameras and depth
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $X = \frac{(u - c_x)\,Z}{f_x},\qquad Y = \frac{(v - c_y)\,Z}{f_y},\qquad Z = Z$ | back-project a pixel with known depth to a 3D point (pinhole model) | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md), [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $u = c_x + f_x \frac{X}{Z}, \qquad v = c_y + f_y \frac{Y}{Z}$ | project a 3D point to a pixel | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md) |
| $f_x = f_y = \frac{W/2}{\tan(\text{hfov}/2)}$ | focal length in pixels from field of view | [05.10](../05-frames-and-transforms/05.10-object-from-camera-to-map.md), [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md), [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $\text{IFOV} = \frac{\text{HFOV}}{W}$ | angle per pixel | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $\text{blur}_{px} = \frac{\omega\, t_e}{\text{IFOV}}$ | motion blur in pixels while turning | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $\text{skew}_{px} = \frac{\omega\, t_r}{\text{IFOV}}$ | rolling-shutter skew in pixels while turning | [07.08](../07-sensors/07.08-rgb-cameras.md) |
| $z = \frac{f B}{d}$ | stereo depth from disparity | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $\frac{\partial z}{\partial d} = -\frac{f B}{d^2} = -\frac{z^2}{f B}$ | sensitivity of stereo depth to disparity | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $\Delta z = \frac{z^2}{f B}\,\Delta d$ | stereo depth error grows with distance squared | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |
| $z_{\min} = \frac{fB}{d_{\max}}$ | closest measurable stereo depth | [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md) |

### Range sensors: ultrasonic, ToF, lidar
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = \frac{c \cdot t_{\text{echo}}}{2}$ | time-of-flight distance (round trip, so divide by 2) | [07.03](../07-sensors/07.03-ultrasonic-sensors.md), [07.04](../07-sensors/07.04-tof-sensors.md) |
| $c(T) = 331.3\,\sqrt{1 + \frac{T}{273.15}}\ \text{m/s}$ | speed of sound vs air temperature | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $343/c(T)$ | scale error of a sensor that assumes 343 m/s | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $r = d\,\tan\frac{\alpha}{2}$ | radius of a sensor's beam cone at distance $d$ | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $s = d\,\tan(2\theta)$ | sideways offset of a specular reflection off a wall at incidence $\theta$ | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $d_{\min} = \frac{c\,t_{\text{ring}}}{2}$ | blind zone caused by transducer ringing | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $f_{\max} \approx \frac{1}{1.5 \times 2 d_{\max}/c}$ | fastest ping rate without ghost echoes | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $h/\tan(\alpha/2 - \text{tilt})$ | distance where a tilted beam cone hits the floor (sensor at height $h$) | [07.03](../07-sensors/07.03-ultrasonic-sensors.md) |
| $N \propto 1/d^2$ | returned photons fall with distance squared | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\sigma \propto \frac{\sqrt{N}}{N} = \frac{1}{\sqrt N} \propto d$ | ToF noise grows linearly with distance (dark) | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\sigma \propto \frac{\sqrt{N + A}}{N}$ | ToF noise with ambient light | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\alpha_{\text{ROI}} \approx 27° \times \frac{n}{16}$ | field of view of an $n \times n$ ToF region of interest | [07.04](../07-sensors/07.04-tof-sensors.md) |
| $\text{beams per revolution} = \frac{\text{samples/s}}{\text{rev/s}}, \qquad \Delta\theta = \frac{360°}{\text{beams}}$ | lidar angular resolution | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $s = d \cdot \Delta\theta$ | gap between neighbouring lidar points at distance $d$ | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $d \le w/(k\,\Delta\theta)$ | max distance at which an object of width $w$ still gets $k$ beams | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $\text{translation} = v\,T, \qquad \text{arc smear} = \omega\,T\,r$ | lidar scan distortion from moving during one revolution | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $x_i = r_i \cos(\theta_i), \quad y_i = r_i \sin(\theta_i), \quad \theta_i = \texttt{angle\_{min}} + i \cdot \texttt{angle\_increment}$ | LaserScan ranges to points | [07.07](../07-sensors/07.07-2d-lidar.md) |
| $\text{angle\_{max}} = \pi - 2\pi/n$ | last beam angle of an $n$-beam 360° scan (no duplicate at $+\pi$) | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $S(\mathbf{p}, \alpha) = \frac{1}{\lvert R \rvert}\sum_{i \in R} \bigl\lvert r_i(\mathbf{p} + \delta \hat{u}_\alpha) - r_i(\mathbf{p}) \bigr\rvert$ | scan-change score: can scan matching tell you moved? | [06.08](../06-simulation/06.08-building-worlds.md) |
| $\min_\alpha S(\mathbf{p}, \alpha)$ | worst-direction score (low = world is ambiguous) | [06.08](../06-simulation/06.08-building-worlds.md) |

### IMU
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $a_x \approx -9.81\sin\theta$ | accelerometer reading on a slope (gravity leaks into x) | [07.05](../07-sensors/07.05-imu-fundamentals.md), [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $a_x = -g\sin\theta + b, \qquad a_z = g\cos\theta + b$ | tilted accelerometer readings with bias | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $\text{roll} = \operatorname{atan2}(a_y,\ a_z), \qquad \text{pitch} = \operatorname{atan2}(-a_x,\ \sqrt{a_y^2+a_z^2})$ | tilt from gravity (robot at rest) | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\Delta\psi(t) = b\,t$ | heading drift from integrating a gyro bias | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\sigma\sqrt{\Delta t\, t}$ (white noise), $b\,t$ (bias) | angle error after integrating a gyro for time $t$ | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $t = \sigma^2 \Delta t / b^2$ | time after which bias drift beats noise drift | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $b = \frac{\text{up} + \text{down}}{2}, \qquad s = \frac{\text{up} - \text{down}}{2g}, \qquad \text{corrected} = \frac{\text{raw} - b}{s}$ | six-position accelerometer calibration | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\text{offset} = \frac{\max + \min}{2}, \qquad \text{scale} = \frac{\overline{r}}{r_{\text{axis}}}, \qquad \text{corrected} = (\text{raw} - \text{offset}) \times \text{scale}$ | magnetometer hard-iron / soft-iron calibration | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\psi = \operatorname{atan2}(-m_y,\ m_x)$ | compass heading from a level magnetometer | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $B = \mu_0 I / 2\pi r$ | magnetic field near a wire carrying current $I$ (motor wires disturb the compass) | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\lvert \mathbf{a} \rvert \approx g$ | at rest the accelerometer magnitude should equal gravity | [07.05](../07-sensors/07.05-imu-fundamentals.md), [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |
| $\hat{x} = \frac{\sigma_2^2\,x_1 + \sigma_1^2\,x_2}{\sigma_1^2 + \sigma_2^2}, \qquad w_1 = \frac{\sigma_2^2}{\sigma_1^2 + \sigma_2^2}$ | fuse two measurements, weighting each by the other's variance | [07.06](../07-sensors/07.06-imu-in-ros2.md) |
| $\text{variance} = d^2 \cdot f_s/2$ | IMU covariance from datasheet noise density | [07.06](../07-sensors/07.06-imu-in-ros2.md) |

### Measurement statistics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\bar{x}=\frac1n\sum x_i$ | mean | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\sigma=\sqrt{\frac{1}{n-1}\sum (x_i-\bar x)^2}$ | sample standard deviation (precision) | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $b=\bar x-t$ | bias (accuracy error) | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\mathrm{RMSE}=\sqrt{\frac1n\sum (x_i-t)^2}\approx\sqrt{b^2+\sigma^2}$ | total error combining bias and noise | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\mathrm{SEM}=\frac{\sigma}{\sqrt N}$ | averaging $N$ readings shrinks noise by $\sqrt N$ (not bias) | [07.01](../07-sensors/07.01-sensor-fundamentals.md), [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\mathbb{E}[\bar x] = t + b$ | averaging converges to truth plus bias | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\mathrm{MAD}=\operatorname{median}_i\big(\lvert x_i-\operatorname{median}(x)\rvert\big)$ | median absolute deviation | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $z_i=\frac{\lvert x_i-\operatorname{median}(x)\rvert}{1.4826\cdot \mathrm{MAD}},\qquad \text{outlier if } z_i>3.5$ | robust outlier test | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $\sigma/\sqrt{2n}$ | standard error of an estimated standard deviation from $n$ samples | [06.07](../06-simulation/06.07-simulated-sensors-and-noise.md) |

### Simulation and motor response
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $dt < 2\sqrt{m/k}$ | stable physics step for a spring-mass system | [06.01](../06-simulation/06.01-why-simulate.md) |
| $\omega(t + dt) = \omega_\text{target} + \left(\omega(t) - \omega_\text{target}\right) e^{-dt/\tau}$ | exact first-order motor response step | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\alpha = 1 - e^{-dt/\tau}$ | exact fraction of the speed gap closed per step | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\alpha = dt/\tau$ | Euler approximation (only valid for $dt \ll \tau$; unstable for $dt > 2\tau$) | [06.02](../06-simulation/06.02-course-mini-simulator.md), [06.01](../06-simulation/06.01-why-simulate.md) |
| $dt < \tau/5$ | rule of thumb for an accurate time step | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\omega_\text{target} = \text{sign}(u) \cdot \frac{\lvert u \rvert - u_\text{dead}}{1 - u_\text{dead}} \cdot \omega_\text{max}$ | command to wheel speed with motor deadband | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| $\dot{\omega}(t) = \frac{K\,u(t - L) - \omega(t)}{\tau}$ | first-order-plus-dead-time motor model (gain, delay, lag) | [06.10](../06-simulation/06.10-sim-to-real-gap.md) |
| $\mu \ge a/g$ | friction needed to accelerate at $a$ without slipping | [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $a_{\max} = \mu g$ | largest acceleration friction allows (all weight on driven wheels) | [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $k = \sqrt{I/m}$ | radius of gyration (inertia sanity check) | [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $I_1 + I_2 \ge I_3$ | triangle inequality every physical inertia must satisfy | [06.05](../06-simulation/06.05-robot-in-gazebo.md), [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |

### Forces, friction and traction
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $F = ma$ | Newton's second law | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md), [06.03](../06-simulation/06.03-gazebo-basics.md) |
| $W = mg$ | weight | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $F = m a + m g \sin\theta + c_{rr} m g \cos\theta$ | drive force to accelerate up a slope with rolling resistance | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $N_w + N_c = W\cos\theta$ | support forces balance the weight component into the slope | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $N_c \, d_c = W\cos\theta \, d_g - (W\sin\theta + m a)\, h$ | moment balance about the axle: caster load | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $N_c = \frac{W d_g - m a h}{d_c}, \qquad N_w = W - N_c$ | load on caster and drive wheels while accelerating on flat ground | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $d_g = d_c\,N_c/W$ | find centre of mass position from a scale under the caster | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md) |
| $F_{friction} \le \mu_s N$ | static friction limit (no slip) | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $F_{friction} = \mu_k N$ | kinetic (sliding) friction | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\mu_s = \tan\theta_{slip}$ | measure friction with a tilting ramp | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $mg\sin\theta > \mu\, mg\cos\theta \iff \mu < \tan\theta$ | condition for sliding down a slope | [06.03](../06-simulation/06.03-gazebo-basics.md) |
| $a = g(\sin\theta - \mu\cos\theta)$ | acceleration of a block sliding down a slope | [06.03](../06-simulation/06.03-gazebo-basics.md) |
| $m a + c_{rr} W \le \mu N_w$ | traction condition for the drive wheels | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $a_{max} = g\,\frac{\mu\,(1 - d_g/d_c) - c_{rr}}{1 - \mu\, h/d_c}$ | max acceleration before wheel slip (with load transfer) | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $b_{max} = g\,\frac{\mu(1 - d_g/d_c) + c_{rr}}{1 + \mu h/d_c}$ | max braking before wheel slip | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $P_{max} = \frac{\mu W (1 - d_g/d_c)}{1 - \mu h_p/d_c}$ | max push force before wheel slip | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\mu = \frac{P}{W(1-d_g/d_c) + P h_p/d_c}$ | friction coefficient from a measured push force | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\omega r - v$ | wheel slip speed | [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |

### Torque, gears and motors
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\tau = r \times F$, magnitude $\tau = r F \sin\phi$ | torque from a force on a lever | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $F = \tau / r$ | wheel push force from wheel torque | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $1\ \text{kg·cm} = 0.0981\ \text{N·m}$ | datasheet torque unit conversion | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\omega_{out} = \frac{\omega_{motor}}{N}, \qquad \tau_{out} = \eta\, N\, \tau_{motor}$ | gearbox trades speed for torque | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $v = \omega_{out}\, r, \qquad F = \frac{\tau_{out}}{r}$ | wheel speed and push force from gearbox output | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $P = \tau\omega = F v$ | mechanical power | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md), [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $\omega = \omega_0\left(1 - \frac{\tau}{\tau_s}\right), \qquad I \approx I_0 + (I_s - I_0)\frac{\tau}{\tau_s}, \qquad P_{out} = \tau\omega$ | DC motor speed-torque and current-torque lines | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $P_{max} = \frac{\tau_s \omega_0}{4}$ | peak motor output power (at half stall torque) | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\tau = \tau_s(d - \omega/\omega_0)$ | motor torque at PWM duty $d$ | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $d = \omega/\omega_0 + \tau/\tau_s$ | duty needed for a given speed and torque | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\omega \le 0.8 \times \omega_0(1 - \tau/\tau_s)$ | design rule: keep 20 % speed headroom | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $\omega_{12} = \omega \times 12/V_{batt}$ | scale a motor speed rating to a different battery voltage | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |

### Energy, power and batteries
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\eta = P_{out}/P_{in}$ | efficiency | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $E = V_{nom} \times Q$ | battery energy from voltage and capacity | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P = VI$ | electrical power | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $R = V/I_s, \qquad k_e = V/\omega_0, \qquad k_t = \tau_s/(I_s - I_0)$ | DC motor constants from datasheet numbers | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $I = I_0 + \frac{\tau}{k_t}, \qquad V_m = I R + k_e \omega, \qquad P_{elec} = V_m I$ | DC motor electrical model: current, voltage, input power | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $I^2R$ | heat lost in motor windings | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $P_{elec}/V_{batt}$ | battery current drawn by a motor through a driver | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $P_{avg} = \sum_i \frac{P_i \cdot f_i}{\eta_i}, \qquad t_{run} = \frac{E_{usable}}{P_{avg}}$ | power budget and run time | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |
| $e = \frac{P_{drive}(v) + P_{fixed}}{v}$ | energy per metre: there is a best cruising speed | [FP.05](../optional-foundations/physics/FP.05-energy-power-efficiency.md) |

### Rotational motion and inertia
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\tau = I\alpha$ | Newton's second law for rotation | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\tau = J\alpha$ | torque to spin up a wheel/rotor of inertia $J$ | [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md) |
| $a_t = \alpha r$ | tangential acceleration from angular acceleration | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $a_c = v^2/R = \omega^2 R$ | centripetal acceleration in a turn | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I = \sum_k m_k r_k^2$ | moment of inertia from point masses | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_{xx} = \frac{m}{12}(y^2+z^2), \quad I_{yy} = \frac{m}{12}(x^2+z^2), \quad I_{zz} = \frac{m}{12}(x^2+y^2)$ | inertia of a solid box | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md), [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_{xx} = I_{yy} = \frac{m}{12}(3r^2+h^2), \quad I_{zz} = \frac{m r^2}{2}$ | inertia of a solid cylinder (axis along z) | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $I_{xx} = I_{yy} = I_{zz} = \frac{2}{5} m r^2$ | inertia of a solid sphere | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md) |
| $\frac{1}{12}mL^2$ | inertia of a thin rod about its centre | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $mL^2/3$ | inertia of a thin rod about one end | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I = I_{cm} + m d^2$ | parallel-axis theorem | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\mathbf{I} = \mathbf{I}_{cm} + m(\lVert\mathbf{d}\rVert^2 \mathbf{E} - \mathbf{d}\mathbf{d}^\top)$ | parallel-axis theorem for the full inertia tensor | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\boldsymbol{\tau} = \mathbf{I}\boldsymbol{\alpha}$ | rotation law with the 3×3 inertia tensor (body not already spinning) | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\mathbf{I}' = R\,\mathbf{I}\,R^\top$ | rotate an inertia tensor into another frame | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I \approx m \times (\text{size}/3)^2$ | quick estimate of inertia | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $J_{reflected} = N^2 J_r$ | motor rotor inertia seen through a gearbox of ratio $N$ | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\tfrac12 J_r (N\omega)^2$ | kinetic energy stored in a geared rotor | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $b = \tau_s/\omega_0, \qquad \tau_m = J/b, \qquad J = \tau_m\, b$ | estimate wheel+rotor inertia from a measured spin-up time constant | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $I_{yaw,eff} = I_{zz} + 2 J (d/r)^2$ | yaw inertia including spinning wheel rotors | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $\tau = 2Fd$ | yaw torque from two wheels pushing in opposite directions | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |
| $mgL/2$ | gravity torque on a horizontal arm link at its joint | [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md) |

### Centre of mass and stability
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{c} = \frac{\sum_i m_i \mathbf{p}_i}{\sum_i m_i}$ | centre of mass of several parts | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $d = \frac{(p_x - x_1)(-e_y) + (p_y - y_1)\,e_x}{\sqrt{e_x^2 + e_y^2}},\qquad (e_x, e_y) = (x_2-x_1,\ y_2-y_1)$ | signed distance from the CoM to a support-polygon edge | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $\ell_{front} = x_{caster} - x_c, \qquad \ell_{rear} = x_c - x_{axle}$ | lever arms to the tipping edges | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $x_c = d_c \frac{N_c}{W}$ | CoM forward position from a scale under the caster | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $h = \frac{x_c - d_c\,N_c'/W}{\tan\beta}$ | CoM height from a second weighing while tilted by $\beta$ | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $x_{zmp} = x_c - \frac{h\,a_x}{g}$ | zero-moment point shifts backwards when accelerating | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $b_{tip} = g\,\frac{\ell_{front}}{h}, \qquad a_{tip} = g\,\frac{\ell_{rear}}{h}$ | braking / acceleration that tips the robot | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $a = g\,d_g/h$ | acceleration that lifts the caster (wheelie) | [FP.02](../optional-foundations/physics/FP.02-forces-newton.md), [FP.03](../optional-foundations/physics/FP.03-friction-traction.md) |
| $\tan\theta > \ell/h$ | slope angle that tips the robot over | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |
| $g\,\text{margin}/h$ | tipping acceleration from a stability margin | [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md) |

## Control, odometry and localization (08–10, FCT)

### Feedback basics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $e = r - y$ | control error | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [08.06](../08-control/08.06-derivative-control.md) |
| $u_R = u_0 + K\,e$ | tick-synchronisation loop: right duty from tick difference | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $y = G_2 G_1 u$ | blocks in series: gains multiply | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $y = (G_1 + G_2) u$ | blocks in parallel: gains add | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $\frac{y}{r} = \frac{GC}{1 + GC} = \frac{L}{1+L}, \quad L = GC$ | closed-loop gain of negative feedback; loop gain | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md), [08.04](../08-control/08.04-proportional-control.md) |
| $e = r/(1+L)$ | steady-state error of a P-only loop | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $S = 1/(1+L)$ | sensitivity: how much of a plant change or disturbance feedback leaves | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $u = r/K_{nom} + K_p e$ | feedforward + P command | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $y = r\,\frac{K/K_{nom} + L}{1 + L}$ | steady state of feedforward + P | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |
| $y = r(1+L)/(1+1.1L)$ | true output with feedforward + P when the sensor reads 10 % high | [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md) |

### Motor model and step response
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\omega_{ss} = K\,(u - u_{db}) \quad \text{for } u > u_{db}$, 0 inside the deadband | static motor model: steady speed from duty | [08.02](../08-control/08.02-motor-step-response.md) |
| $\mathrm{dz}(u) = \mathrm{sign}(u)\,\max(\lvert u\rvert - u_{db},\,0)$ | deadband function | [08.02](../08-control/08.02-motor-step-response.md) |
| $\tau\,\frac{d\omega}{dt} = -\omega + K\,\mathrm{dz}(u)$ | first-order motor dynamics | [08.02](../08-control/08.02-motor-step-response.md) |
| $\tau \frac{dy}{dt} + y = K u$ | general first-order system | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\omega(t) = \omega_f \left(1 - e^{-(t - t_0)/\tau}\right), \quad \omega_f = K\,\mathrm{dz}(u)$ | step response of the motor | [08.02](../08-control/08.02-motor-step-response.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\theta(t) = \omega_f\left[(t - t_0) - \tau\left(1 - e^{-(t-t_0)/\tau}\right)\right]$ | wheel angle during a step (integral of speed) | [08.02](../08-control/08.02-motor-step-response.md) |
| $\omega_{k} = \omega_{k-1} + \left(1 - e^{-\Delta t/\tau}\right)\left(K\,\mathrm{dz}(u_{k-1}) - \omega_{k-1}\right)$ | exact discrete motor model (duty held per period) | [08.02](../08-control/08.02-motor-step-response.md) |
| $\alpha = 1 - e^{-\Delta t/\tau}$ | fraction of the remaining gap covered per sample | [08.02](../08-control/08.02-motor-step-response.md) |
| $K \times V/V_{nom}$ | motor gain corrected for the present battery voltage (gain scales with voltage) | [08.02](../08-control/08.02-motor-step-response.md) |
| $t = \tau \ln 100 = 4.6\,\tau$ | time to get within 1 % of the final value | [08.02](../08-control/08.02-motor-step-response.md) |
| $t_r = \tau\ln 9 = 2.20\,\tau$ | 10–90 % rise time of a first-order lag | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $t_{s,2\%} = \tau\ln 50$ | 2 % settling time of a first-order lag | [08.07](../08-control/08.07-pid-mathematics.md) |
| $K u_0/\tau$ | initial slope of a first-order step response | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $G(s) = \frac{K}{\tau s + 1}\,e^{-\theta s} \iff y(t) = K\,u_{step}\left(1 - e^{-(t - t_{step} - \theta)/\tau}\right)$ | first-order-plus-dead-time (FOPDT) model fitted from one step | [08.08](../08-control/08.08-tuning-pid.md) |
| $L\frac{di}{dt} = V - R i - k_e\omega, \quad J\frac{d\omega}{dt} = k_t i$ | full brushed DC motor model (current and speed) | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $i = (V - k_e\omega)/R$ | current when inductance is neglected ($L = 0$) | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\tau = JR/(k_t k_e)$ | mechanical time constant from motor constants | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |

### Speed estimation and filtering
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\hat\omega_k = \frac{n_k - n_{k-1}}{N_{rev}} \cdot \frac{2\pi}{T}$ | wheel speed from ticks counted in a window | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\hat\omega_k = \frac{2\pi}{N_{rev}}\cdot\frac{n_k - n_{k-1}}{t_k - t_{k-1}}$ | wheel speed using the robot's own timestamps (robust to lost samples) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\Delta\omega = \frac{2\pi}{N_{rev}\,T}$ | speed resolution (one tick per window) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\Delta\omega_{min} = \frac{2\pi}{N_{ticks}} f_s$ | speed resolution at sample rate $f_s$ | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| lag $= T/2$ | lag of a raw window estimate (it describes the window's middle) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| lag $= (N-1)T/2$ (plus $T/2$), noise $\div\sqrt N$ | moving average of $N$ samples: added lag and white-noise reduction | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $y_k = y_{k-1} + \alpha\,(x_k - y_{k-1}), \quad 0 < \alpha \le 1$ | first-order low-pass filter (EMA) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\tau_f = T\,\frac{1-\alpha}{\alpha}, \quad \alpha = \frac{T}{\tau_f + T}$ | EMA time constant (≈ its lag) and the factor for a wanted time constant | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| noise $\times\sqrt{\alpha/(2-\alpha)}$ | EMA white-noise reduction | [08.03](../08-control/08.03-wheel-speed-estimation.md), [08.06](../08-control/08.06-derivative-control.md) |
| $\sigma_\omega = \frac{\sqrt 2\,\sigma_n}{T}\cdot\frac{2\pi}{N_{rev}}$ | speed noise from position noise (halve $T$, double the noise) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\sigma_n = 1/\sqrt{12}$ tick | quantisation error of one reading (uniform over one tick) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| ticks per window $= \omega N_{rev} T/2\pi$ | how many ticks arrive per window (want ≥ 5) | [08.03](../08-control/08.03-wheel-speed-estimation.md) |

### PID control
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $u = k_p\,(r - \omega)$ | proportional (P) controller | [08.04](../08-control/08.04-proportional-control.md) |
| $\omega = \frac{K\,(k_p r - u_{db})}{1 + k_p K}, \quad e_{ss} = r - \omega$ | settled speed and error of a P loop on a motor with deadband | [08.04](../08-control/08.04-proportional-control.md) |
| $L = k_p K$ | loop gain of the wheel-speed P loop | [08.04](../08-control/08.04-proportional-control.md) |
| $k_p = \frac{\omega + K u_{db}}{K\,(r - \omega)}$ | P gain needed for a wanted settled speed | [08.04](../08-control/08.04-proportional-control.md) |
| $u(t) = k_p\,e(t) + k_i \int_0^t e(s)\,ds$ | PI controller | [08.05](../08-control/08.05-integral-control.md) |
| $I_k = I_{k-1} + k_i\,e_k\,\Delta t, \quad u_k = \mathrm{clamp}\left(k_p e_k + I_k,\ -1,\ 1\right)$ | discrete PI with integral stored in duty units (Euler) | [08.05](../08-control/08.05-integral-control.md) |
| $I_k - I_{k-1} = k_i e_k \Delta t \Rightarrow e = 0$ in steady state | why the integral removes steady-state error | [08.05](../08-control/08.05-integral-control.md) |
| keep $I_{k-1}$ if $k_p e_k + I_{candidate}$ is beyond ±1 and $e_k$ pushes further | conditional-integration anti-windup | [08.05](../08-control/08.05-integral-control.md) |
| $u = k_p\,e + k_i\!\int e\,dt + k_d\,\frac{de}{dt}, \quad e = r - y$ | PID controller (parallel form) | [08.06](../08-control/08.06-derivative-control.md), [08.07](../08-control/08.07-pid-mathematics.md) |
| $D = -k_d\,\frac{dy}{dt}$ | derivative on measurement (no derivative kick) | [08.06](../08-control/08.06-derivative-control.md) |
| $\frac{de}{dt} = \frac{dr}{dt} - \frac{dy}{dt} = -\frac{dy}{dt}$ when $\frac{dr}{dt} = 0$ | why D on measurement equals D on error for a constant setpoint | [08.06](../08-control/08.06-derivative-control.md) |
| $\dot y_{raw} = \frac{y_k - y_{k-1}}{\Delta t}, \ \alpha = \frac{\Delta t}{\tau_d + \Delta t}, \ \dot y_k = \dot y_{k-1} + \alpha\,(\dot y_{raw} - \dot y_{k-1}), \ D_k = -k_d\,\dot y_k$ | filtered derivative term | [08.06](../08-control/08.06-derivative-control.md) |
| $D_{kick} = k_d\,\frac{\Delta r}{\Delta t}$ | derivative kick of D-on-error after a setpoint step | [08.06](../08-control/08.06-derivative-control.md) |
| $\sigma_D = k_d\,\frac{\sqrt 2\,\sigma}{\Delta t}$ | noise in the unfiltered D term | [08.06](../08-control/08.06-derivative-control.md) |
| $u = k_p\left(e + \frac{1}{T_i}\!\int e\,dt + T_d\,\frac{de}{dt}\right), \quad k_i = \frac{k_p}{T_i},\ k_d = k_p T_d$ | standard (ISA) PID form and conversion | [08.07](../08-control/08.07-pid-mathematics.md) |
| $I_k = I_{k-1} + k_i\,e_k\,\Delta t, \quad u_k = \mathrm{clamp}\!\left(k_p e_k + I_k - k_d\,\frac{y_k - y_{k-1}}{\Delta t}\right)$ | discrete PID, positional form | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\Delta u_k = k_p\,(e_k - e_{k-1}) + k_i\,e_k\,\Delta t - k_d\,\frac{y_k - 2y_{k-1} + y_{k-2}}{\Delta t}, \quad u_k = \mathrm{clamp}(u_{k-1} + \Delta u_k)$ | discrete PID, velocity (incremental) form | [08.07](../08-control/08.07-pid-mathematics.md) |

### Closed-loop dynamics and PI design
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\ddot y + 2\zeta\omega_n \dot y + \omega_n^2 y = \omega_n^2 u$ | standard second-order system | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $M_p = e^{-\pi\zeta/\sqrt{1-\zeta^2}}$ | overshoot of a second-order step response | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $t_p = \frac{\pi}{\omega_n\sqrt{1-\zeta^2}}$ | peak time | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $t_{s,2\%} \approx \frac{4}{\zeta\omega_n}$ ($\approx 5.8/\omega_n$ at $\zeta = 1$) | 2 % settling time | [08.07](../08-control/08.07-pid-mathematics.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\omega_d = \omega_n\sqrt{1-\zeta^2}$ | ringing frequency | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $-\zeta\omega_n \pm j\omega_n\sqrt{1-\zeta^2}$ | poles of a second-order system | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\frac{\tau}{1+L}\dot y + y = \frac{L}{1+L} r, \quad L = K K_p$ | P speed loop: still first order, time constant $\tau/(1+L)$ | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\tau\ddot\theta + \dot\theta + K K_p\,\theta = K K_p\,\theta_r$ | P control of wheel angle: second-order loop | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $\omega_n = \sqrt{\frac{K K_p}{\tau}}, \quad \zeta = \frac{1}{2\sqrt{\tau K K_p}}, \quad \zeta\omega_n = \frac{1}{2\tau}$ | natural frequency and damping of the wheel-angle P loop | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $K_p = 1/(4\tau K \zeta^2)$ | angle-loop P gain for a target damping | [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md) |
| $G(s) = \frac{K}{\tau s + 1}, \quad C(s) = k_p + \frac{k_i}{s}$ | motor and PI transfer functions | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau s^2 + (1 + K k_p)\,s + K k_i = 0$ | closed-loop characteristic equation of PI on a first-order plant | [08.07](../08-control/08.07-pid-mathematics.md) |
| $k_p = \frac{2\zeta\omega_n\tau - 1}{K}, \quad k_i = \frac{\omega_n^2\,\tau}{K}$ | PI gains by pole placement | [08.07](../08-control/08.07-pid-mathematics.md) |
| $k_p = \frac{\tau}{K\,\tau_{cl}}, \quad k_i = \frac{k_p}{\tau}$ | lambda tuning (PI zero cancels the motor pole) | [08.07](../08-control/08.07-pid-mathematics.md) |
| PI zero at $-k_i/k_p$ | where the PI controller's zero sits | [08.07](../08-control/08.07-pid-mathematics.md) |

### Stability, delay and frequency response
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta \approx \frac{\Delta t}{2} + \tau_{filter} + \theta_{transport}$ | total loop dead time | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\omega_c\theta$ | phase (rad) lost to dead time at crossover | [08.07](../08-control/08.07-pid-mathematics.md) |
| $L(s) = \frac{1}{\tau_{cl}s}e^{-\theta s}, \ \omega_c = 1/\tau_{cl}$ | open loop after lambda tuning | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\text{PM} = 90° - 57.3\,\frac{\theta}{\tau_{cl}}$ degrees | phase margin of a lambda-tuned loop | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\tau_{cl} > 1.3\,\theta$ | minimum closed-loop time constant for 45° phase margin | [08.12](../08-control/08.12-control-in-ros2.md) |
| $a_2 s^2 + a_1 s + a_0 = 0$ | characteristic polynomial (try $y = e^{st}$) | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $1 + C\,G\,H = 0$ | closed-loop poles of a feedback loop | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $p = -\frac{1 + KK_p}{\tau}$ | pole of the P speed loop | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $p = -\frac{1}{2\tau} \pm j\sqrt{\frac{KK_p}{\tau} - \frac{1}{4\tau^2}}$ | poles of the wheel-angle P loop | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\tau\tau_f\, s^3 + (\tau + \tau_f)\, s^2 + s + KK_p = 0$ | angle loop with a speed filter (third order) | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $a_2 a_1 > a_3 a_0$ | Routh–Hurwitz stability test for a cubic with positive coefficients | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $K_p < \frac{\tau + \tau_f}{\tau\tau_f K}$, oscillation at $1/\sqrt{\tau\tau_f}$ | critical gain of the filtered angle loop | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| gain $K/\sqrt{1 + (\omega\tau)^2}$, phase $-\arctan(\omega\tau)$ | frequency response of a first-order lag | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| gain 1, phase $-\omega T$ | frequency response of a pure delay $e^{-sT}$ | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| oscillates if $\lvert L(j\omega_{180})\rvert \ge 1$ | oscillation condition (practical Nyquist) | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $\arctan(\omega\tau) + \omega T = \pi$ | frequency $\omega_{180}$ where lag + delay reach −180° | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| $K_{p,crit} = \frac{\sqrt{1 + (\tau\omega_{180})^2}}{K}$ | critical P gain of a first-order loop with delay | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| gain margin $= K_{p,crit}/K_p$ | how much gain could grow before instability | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md) |
| phase margin $= 180° + \angle L(j\omega_c)$ at $\lvert L\rvert = 1$ | extra lag tolerated at crossover | [FCT.04](../optional-foundations/control-theory/FCT.04-stability-intuition.md), [08.07](../08-control/08.07-pid-mathematics.md) |

### Tuning rules
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $R = K/\tau$, $L = \theta$ | Ziegler–Nichols open-loop parameters from a fitted step | [08.08](../08-control/08.08-tuning-pid.md) |
| P: $k_p = 1/(RL) = \tau/(K\theta)$; PI: $0.9/(RL)$, $T_i = L/0.3$; PID: $1.2/(RL)$, $T_i = 2L$, $T_d = 0.5L$ | Ziegler–Nichols open-loop (reaction curve) rules | [08.08](../08-control/08.08-tuning-pid.md) |
| P: $0.5K_u$; PI: $0.45K_u$, $T_i = T_u/1.2$; PID: $0.6K_u$, $T_i = T_u/2$, $T_d = T_u/8$; no overshoot: $0.2K_u$, $T_u/2$, $T_u/3$ | Ziegler–Nichols closed-loop (ultimate gain) rules | [08.08](../08-control/08.08-tuning-pid.md) |
| $u = u_0 \pm d$ | relay (bang-bang) auto-tune controller | [08.08](../08-control/08.08-tuning-pid.md) |
| $K_u \approx \frac{4d}{\pi a}$ | ultimate gain from the relay limit cycle (describing function) | [08.08](../08-control/08.08-tuning-pid.md) |
| $\text{IAE} = \int \lvert e\rvert\,dt, \ \text{ITAE} = \int t\,\lvert e\rvert\,dt, \ \text{ISE} = \int e^2\,dt$ | error-integral scores for comparing tunings | [08.08](../08-control/08.08-tuning-pid.md) |

### Feedforward
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\omega = \frac{v}{r}$ | wheel speed needed for a ground speed | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $u_{ff} = \mathrm{sign}(\omega)\left(u_{db} + \frac{\lvert\omega\rvert}{\omega_{top}(V)}\,(1 - u_{db})\right)$ | feedforward duty from the motor model | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $\omega_{top}(V) = \omega_{max}\frac{V}{V_{nom}}$ | top wheel speed at the current battery voltage | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $u = u_{ff}(r_{sp}, V) + k_p\,e + I, \quad I \mathrel{+}= k_i\,e\,\Delta t$ | feedforward plus PI correction | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| unclamped output $= u_{ff} + k_p e + I_{candidate}$ | value the anti-windup test must check | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $u = u_{ff}(\omega_{sp}) + k_{ff,a}\,\dot\omega_{sp} + \text{PID}$ | acceleration feedforward for a moving setpoint | [08.09](../08-control/08.09-feedforward-exact-speed.md) |

### Heading control and turning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v = \frac{r(\omega_L + \omega_R)}{2}, \quad \omega_z = \frac{r(\omega_R - \omega_L)}{b}$ | robot speed and yaw rate from wheel speeds | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\omega_L = \omega_{fwd} - \frac{\omega_z\,b}{2r}, \quad \omega_R = \omega_{fwd} + \frac{\omega_z\,b}{2r}$ | wheel speeds for a forward speed and yaw rate | [08.10](../08-control/08.10-straight-line-heading-control.md), [08.11](../08-control/08.11-rotate-exactly-90.md) |
| meters_per_tick $= 2\pi r_{nom}/N$ | distance per encoder tick | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\Delta\theta = \frac{\Delta d_R - \Delta d_L}{b}, \quad \Delta d = \Delta\text{ticks} \times \text{meters\_per\_tick}$ | heading change from encoders | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\theta_{k} = \theta_{k-1} + (\omega_{measured} - b_{gyro})\,\Delta t$ | heading from an integrated, de-biased gyro | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $s = \phi\,r_{true}, \quad d_{est} = \phi\,r_{nom} = s\,\frac{r_{nom}}{r_{true}}$ | true vs estimated wheel distance with a wrong radius | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\kappa L^2/2$ | lateral offset after length $L$ with constant heading error rate $\kappa$ | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\omega_{z,cmd} = \mathrm{clamp}\!\left(k_{p,h}\,e_\theta + I,\ \pm\,\omega_{max}\right), \quad e_\theta = \mathrm{wrap}(\theta_{sp} - \theta)$ | heading controller (outer loop) | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| time constant $= 1/k_{p,h}$ | closed loop of P control on an integrator (heading) | [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $e = \mathrm{wrap}(\theta_{target} - \theta_{measured}) \in (-\pi,\ \pi]$ | wrapped angle error | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $v_{peak} = \min\left(v_{max},\ \sqrt{a_{max}\,\lvert d\rvert}\right), \ t_{accel} = \frac{v_{peak}}{a_{max}}, \ t_{cruise} = \frac{\lvert d\rvert - v_{peak}t_{accel}}{v_{peak}}$ | trapezoidal (or triangular) motion profile | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $T = 2t_{accel} + t_{cruise}$ | total profile duration | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $v^2 = 2a(\lvert d\rvert/2)$ | triangular case: half the move spent accelerating | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\tfrac12 a t^2$ (ramp), linear (cruise), $\lvert d\rvert - \tfrac12 a (T-t)^2$ (down) | profile position: exact integral of profile velocity | [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\omega_{z,cmd} = \mathrm{clamp}\Big(\dot\theta_{profile}(t) + k_p\,e + I,\ \pm\,\omega_{max}\Big), \ e = \mathrm{wrap}\big(\theta_{start} + \theta_{profile}(t) - \theta\big)$ | profiled turn: feedforward from the profile plus feedback | [08.11](../08-control/08.11-rotate-exactly-90.md) |

### Differential-drive kinematics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v_w = r\,\omega_w$ | wheel rim speed from wheel angular speed | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{L}$ | forward kinematics: robot speed and yaw rate | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $R = v/\omega$ | turning radius | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $v_L = \omega\left(R - \frac{L}{2}\right), \quad v_R = \omega\left(R + \frac{L}{2}\right)$ | wheel speeds as rotation about the ICC | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $R = \frac{L}{2}\cdot\frac{v_R + v_L}{v_R - v_L}$ | turning radius from wheel speeds | [09.01](../09-odometry/09.01-diff-drive-kinematics.md), [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} r/2 & r/2 \\ -r/L & r/L \end{bmatrix} \begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix}$ | forward kinematics as a matrix (determinant $r^2/L$) | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\dot x = v\cos\theta, \quad \dot y = v\sin\theta, \quad \dot\theta = \omega$ | unicycle model: pose rates in the world frame | [09.01](../09-odometry/09.01-diff-drive-kinematics.md), [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\text{ICC} = (x - R\sin\theta,\; y + R\cos\theta)$ | ICC in world coordinates | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\dot x \sin\theta - \dot y\cos\theta = 0$ | non-holonomic constraint: no sideways motion | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\frac{\delta v}{v} = \frac{\delta r}{r}, \quad \frac{\delta \omega}{\omega} = \frac{\delta r}{r} - \frac{\delta L}{L}$ | how radius and wheelbase errors propagate | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $R(1-\cos(s/R)) \approx s^2/2R$ | sideways drift after driving $s$ on an arc of radius $R$ | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $\Delta\theta = s/R$ | heading turned after arc length $s$ | [08.01](../08-control/08.01-open-vs-closed-loop.md) |
| $\omega_L = \frac{v - \omega L/2}{r}, \quad \omega_R = \frac{v + \omega L/2}{r}$ | inverse kinematics: wheel speeds from cmd_vel | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $\begin{bmatrix} \omega_L \\ \omega_R \end{bmatrix} = \frac{1}{r}\begin{bmatrix} 1 & -L/2 \\ 1 & L/2 \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix}$ | inverse kinematics as a matrix | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $\kappa = \omega / v = 1/R$ | curvature of the commanded path | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $R = \frac{L}{2}\cdot\frac{\omega_R + \omega_L}{\omega_R - \omega_L}$ | turning radius from wheel angular speeds (depends only on their ratio) | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $s = \min\left(1,\ \frac{\omega_{max}}{\max(\lvert\omega_L\rvert, \lvert\omega_R\rvert)}\right), \ (\omega_L, \omega_R) \leftarrow (s\,\omega_L,\ s\,\omega_R)$ | saturation strategy 1: scale both wheels (keeps curvature) | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $m = (\omega_L + \omega_R)/2,\ d = (\omega_R - \omega_L)/2$; clamp $m$ to $\pm(\omega_{max} - \lvert d\rvert)$ | saturation strategy 2: keep rotation | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| step $\le a_{max}\,\Delta t$ per cycle | acceleration ramp of a command | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |

### Dead reckoning (odometry integration)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\Delta s = v\,\Delta t, \quad \Delta\theta = \omega\,\Delta t$ | step distance and heading change from velocities | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $\Delta s = (d_R + d_L)/2, \quad \Delta\theta = (d_R - d_L)/L$ | step distance and heading change from wheel distances | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $x \mathrel{+}= \Delta s\cos\theta, \ y \mathrel{+}= \Delta s\sin\theta$ | Euler update (first order) | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $x \mathrel{+}= \Delta s\cos(\theta + \Delta\theta/2), \ y \mathrel{+}= \Delta s\sin(\theta + \Delta\theta/2)$ | midpoint (RK2) update (second order) | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $x \mathrel{+}= R\,[\sin(\theta+\Delta\theta) - \sin\theta], \ y \mathrel{-}= R\,[\cos(\theta+\Delta\theta) - \cos\theta], \ R = \Delta s/\Delta\theta$ | exact-arc update (straight-line branch when $\lvert\Delta\theta\rvert < 10^{-9}$) | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $\theta \leftarrow \operatorname{atan2}(\sin(\theta+\Delta\theta), \cos(\theta+\Delta\theta))$ | heading update with wrapping | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $x(t+\Delta t) = x + v\cos\theta\,\Delta t - \tfrac12 v\omega\sin\theta\,\Delta t^2 + O(\Delta t^3)$ | Taylor expansion: Euler's per-step error $\tfrac12 v\omega\,\Delta t^2$ | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| total Euler error $\propto \tfrac12 v\omega T\,\Delta t$ | Euler error over a fixed duration is linear in $\Delta t$ | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $x(\Delta t) - x_0 = \int_0^{\Delta t} v\cos(\theta_0 + \omega\tau)\,d\tau = R\left[\sin(\theta_0 + \Delta\theta) - \sin\theta_0\right]$ | derivation of the exact arc | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $\Delta x_{exact} = \Delta s\,\cos\!\left(\theta + \frac{\Delta\theta}{2}\right)\cdot\frac{\sin(\Delta\theta/2)}{\Delta\theta/2}$ | exact arc as midpoint × sinc factor (no division by zero) | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $\frac{\sin u}{u} \approx 1 - u^2/6$ | sinc factor for small angles | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $d = \frac{2\pi r}{N_{rev}}\,\Delta n$ | wheel distance from tick delta | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $\Delta n = \big((c - p) + M/2\big) \bmod M \;-\; M/2$ | counter-rollover-safe tick delta (valid if the true change is below $M/2$) | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |

### Odometry calibration
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $E_d = D_R/D_L, \quad E_b = b_{actual}/b_{nominal}$ | systematic error ratios | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $x_{cg} = \frac{1}{n}\sum_i \varepsilon_{x,i}, \ y_{cg} = \frac{1}{n}\sum_i \varepsilon_{y,i}, \ r_{cg} = \sqrt{x_{cg}^2 + y_{cg}^2}$ | UMBmark cluster centre of gravity | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $E_{max,syst} = \max(r_{cg,cw},\ r_{cg,ccw})$ | UMBmark benchmark number | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\text{SEM} = \sigma/\sqrt{n}$; stop when $E_{max,syst} < 3\,\text{SEM}$ | standard error of the mean; calibration stop rule | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\alpha = \frac{x_{cg,cw} + x_{cg,ccw}}{-4L}, \quad \beta = \frac{x_{cg,cw} - x_{cg,ccw}}{-4L}$ | Borenstein: corner and straight-leg errors | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $R = \frac{L/2}{\sin(\beta/2)}, \ E_d = \frac{R + b/2}{R - b/2}, \ E_b = \frac{\pi/2}{\pi/2 - \alpha}$ | Borenstein: error ratios from $\alpha$, $\beta$ | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $c_L = \frac{2}{E_d + 1}, \ c_R = \frac{2}{1/E_d + 1}, \ r_L \leftarrow c_L\,r_L, \ r_R \leftarrow c_R\,r_R, \ b \leftarrow E_b\,b$ | Borenstein corrections | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $S(r) = \sum_i (d_i - r\varphi_i)^2 \Rightarrow \hat r = \frac{\sum_i \varphi_i d_i}{\sum_i \varphi_i^2}$ | least-squares wheel radius from straight runs | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\hat b = \sum s_i\Delta\theta_i / \sum \Delta\theta_i^2$ | least-squares wheelbase from spins | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\mathbf{e}(p) = \big[\ \hat x_j - x_j,\ \hat y_j - y_j,\ w\cdot\operatorname{wrap}(\hat\theta_j - \theta_j)\ \big]_{j=1..N}$ | stacked end-pose residuals for all runs | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| $\mathbf{e}(p + \delta) \approx \mathbf{e}(p) + J\delta$; solve $J\delta = -\mathbf{e}$, $p \leftarrow p + \delta$ (minimises $\lVert\mathbf{e}(p)\rVert^2$) | Gauss–Newton nonlinear least squares | [09.05](../09-odometry/09.05-calibrating-odometry.md) |
| multiplier $= r_L/r$, $r_R/r$, $b/L$ | ROS diff_drive_controller calibration multipliers | [09.05](../09-odometry/09.05-calibrating-odometry.md) |

### Odometry error growth and ROS conventions
| Formula | Name / what it gives | Lesson |
|---|---|---|
| lateral error $= d\sin\delta\theta \approx d\,\delta\theta$ | lever effect of an initial heading error | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\kappa \approx \frac{\Delta r}{\bar r\,b}$ | heading bias per metre from a wheel-radius mismatch | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\delta\theta(d) = \kappa d, \quad \delta y(d) = \int_0^d \kappa s\,ds = \frac{\kappa d^2}{2}$ | systematic error growth on a straight run | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma^2_{L} = k\,\lvert d_L\rvert, \ \sigma^2_{R} = k\,\lvert d_R\rvert$ | random wheel-slip model: variance grows with distance rolled | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $k = s^2\delta$ | slip constant from per-step slip std $s$ and per-step travel $\delta$ | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma^2_\theta = \frac{\sigma_L^2 + \sigma_R^2}{b^2} = \frac{2k\,d}{b^2}$ | random heading variance on a straight run ($\propto d$, so $\sigma \propto \sqrt d$) | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma^2_{lat} = \frac{2k}{b^2}\int_0^d (d - s)^2\,ds = \frac{2k}{b^2}\cdot\frac{d^3}{3}$ | random lateral variance ($\sigma_{lat} = \sigma_\theta d/\sqrt3 \propto d^{1.5}$) | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\sigma_{along} = \sqrt{k d/2}$ | random error along the direction of travel | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $k = \sigma_\theta^2\, b^2 / (2d)$ | estimate the slip constant from repeated straight runs | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\lvert d_L\rvert = \lvert d_R\rvert = b\,\lvert\Delta\theta\rvert/2$ | wheel travel during a spin in place | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $F = \begin{bmatrix} 1 & 0 & -\Delta s\sin\mu \\ 0 & 1 & \Delta s\cos\mu \\ 0 & 0 & 1\end{bmatrix}$ | odometry Jacobian w.r.t. the pose | [09.06](../09-odometry/09.06-accumulated-error.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $G = \begin{bmatrix} \frac{\cos\mu}{2} + \frac{\Delta s\sin\mu}{2b} & \frac{\cos\mu}{2} - \frac{\Delta s\sin\mu}{2b} \\ \frac{\sin\mu}{2} - \frac{\Delta s\cos\mu}{2b} & \frac{\sin\mu}{2} + \frac{\Delta s\cos\mu}{2b} \\ -\frac1b & \frac1b\end{bmatrix}$ | odometry Jacobian w.r.t. wheel travels | [09.06](../09-odometry/09.06-accumulated-error.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\Sigma' = F\,\Sigma\,F^{\mathsf T} + G\,Q\,G^{\mathsf T}, \quad Q = \begin{bmatrix} k\lvert d_L\rvert & 0 \\ 0 & k\lvert d_R\rvert\end{bmatrix}$ | pose covariance propagation through one odometry step | [09.06](../09-odometry/09.06-accumulated-error.md) |
| yaw $\psi \to (0, 0, \sin\frac\psi2, \cos\frac\psi2)$ | planar yaw as a quaternion $(x, y, z, w)$ | [09.07](../09-odometry/09.07-odometry-in-ros2.md) |
| index $6i + j$ holds $\text{cov}(q_i, q_j)$ | ROS 6×6 row-major covariance layout (diagonal at 0, 7, 14, 21, 28, 35) | [09.07](../09-odometry/09.07-odometry-in-ros2.md) |

### Probability, Gaussians and uncertainty ellipses
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $bel(\mathbf{x}_k) = p(\mathbf{x}_k \mid \mathbf{z}_{1:k}, \mathbf{u}_{1:k})$ | belief: what the robot knows about its state | [10.01](../10-localization/10.01-what-is-localization.md) |
| $p(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k)$ | motion model | [10.01](../10-localization/10.01-what-is-localization.md) |
| $p(\mathbf{z}_k \mid \mathbf{x}_k)$ | measurement model | [10.01](../10-localization/10.01-what-is-localization.md) |
| $P(\lvert x - \mu\rvert < a) = \operatorname{erf}\!\big(a / (\sigma\sqrt2)\big)$ | probability a 1D Gaussian lies within ±a of its mean | [10.01](../10-localization/10.01-what-is-localization.md) |
| $x = l_x - r\cos(\theta + \varphi), \quad y = l_y - r\sin(\theta + \varphi)$ | position fix from range and bearing to a known landmark | [10.01](../10-localization/10.01-what-is-localization.md) |
| fix error $\approx r \cdot \delta\theta$ | position error from a heading error at range $r$ | [10.01](../10-localization/10.01-what-is-localization.md) |
| $P = \begin{bmatrix} \sigma_x^2 & \operatorname{cov}(x,y) & \operatorname{cov}(x,\theta) \\ \operatorname{cov}(x,y) & \sigma_y^2 & \operatorname{cov}(y,\theta) \\ \operatorname{cov}(x,\theta) & \operatorname{cov}(y,\theta) & \sigma_\theta^2 \end{bmatrix}$ | 3×3 pose covariance matrix | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\rho = \operatorname{cov}(x,y)/(\sigma_x\sigma_y)$ | correlation coefficient | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $d^2(\mathbf{p}) = (\mathbf{p} - \boldsymbol\mu)^\top \Sigma^{-1} (\mathbf{p} - \boldsymbol\mu)$ | squared Mahalanobis distance; density $\propto \exp(-\tfrac12 d^2)$ | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\Sigma = V \Lambda V^\top$, $\mathbf{q} = V^\top(\mathbf{p} - \boldsymbol\mu)$, $q_1^2/\lambda_1 + q_2^2/\lambda_2 = k$ | eigen-decomposition turns the ellipse axis-aligned | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| semi-axes $= \sqrt{k\lambda_1},\ \sqrt{k\lambda_2}$, angle $= \operatorname{atan2}(v_{1,y}, v_{1,x})$ | ellipse axes from eigenvalues / eigenvectors | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $\lambda_{1,2} = \frac{a+c}{2} \pm \sqrt{\left(\frac{a-c}{2}\right)^2 + b^2}, \quad \alpha = \tfrac12 \operatorname{atan2}(2b,\ a - c)$ | closed-form eigenvalues and angle of a 2×2 covariance $\begin{bmatrix} a & b \\ b & c \end{bmatrix}$ | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $P(d^2 \le k) = 1 - e^{-k/2} \iff k = -2\ln(1 - p)$ | probability inside a 2D ellipse (chi-square, 2 dof) | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| 95 % thresholds: $k = 3.841$ (1D), $5.991$ (2D), $7.815$ (3D) | chi-square 95 % values for ellipses and gates | [10.02](../10-localization/10.02-uncertainty-gaussians.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| half-width $= \sqrt{k\,\Sigma_{xx}}$ | extent of an ellipse along one axis | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |
| $P_{k+1} = F_k P_k F_k^\top + G_k M_k G_k^\top$ | covariance propagation along an odometry log | [10.02](../10-localization/10.02-uncertainty-gaussians.md) |

### Bayes filter and histogram filter
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\overline{bel}(\mathbf{x}_k) = \sum_{\mathbf{x}_{k-1}} p(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k)\, bel(\mathbf{x}_{k-1})$ | Bayes filter predict step (integral for continuous states) | [10.01](../10-localization/10.01-what-is-localization.md), [10.03](../10-localization/10.03-bayes-filter.md) |
| $bel(\mathbf{x}_k) = \eta\; p(\mathbf{z}_k \mid \mathbf{x}_k)\; \overline{bel}(\mathbf{x}_k)$ | Bayes filter update step | [10.01](../10-localization/10.01-what-is-localization.md), [10.03](../10-localization/10.03-bayes-filter.md) |
| $\overline{bel}(j) = \sum_{o} \kappa(o)\ bel(j - u - o) = (bel * \kappa)(j - u)$ | histogram-filter predict as a convolution | [10.03](../10-localization/10.03-bayes-filter.md) |
| $\eta = \Big(\sum_j p(z_k \mid j)\,\overline{bel}(j)\Big)^{-1}$ | normaliser of the update | [10.03](../10-localization/10.03-bayes-filter.md) |
| $p_{hit}/p_{false}$ and $(1 - p_{hit})/(1 - p_{false})$ | likelihood ratios of a "door" / "wall" reading | [10.03](../10-localization/10.03-bayes-filter.md) |
| $\overline{bel}'(j) = (1 - \varepsilon)\ \overline{bel}(j) + \varepsilon / n$ | uniform floor for kidnap recovery | [10.03](../10-localization/10.03-bayes-filter.md) |
| $bel(\mathbf{x}_k) = \eta\; p(\mathbf{z}_k \mid \mathbf{x}_k) \int p(\mathbf{x}_k \mid \mathbf{x}_{k-1}, \mathbf{u}_k)\, bel(\mathbf{x}_{k-1})\, d\mathbf{x}_{k-1}$ | continuous Bayes filter (one line) | [10.08](../10-localization/10.08-particle-filter-mcl.md) |

### Kalman filter (1D)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x_k = x_{k-1} + u_k + w_k,\ w_k \sim \mathcal{N}(0, Q); \quad z_k = x_k + v_k,\ v_k \sim \mathcal{N}(0, R)$ | 1D motion and measurement models | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $\bar{x}_k = x_{k-1} + u_k, \quad \bar{P}_k = P_{k-1} + Q$ | 1D predict | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $\nu_k = z_k - \bar{x}_k, \quad S_k = \bar{P}_k + R, \quad K_k = \frac{\bar{P}_k}{S_k}$ | innovation, its variance, Kalman gain | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $x_k = \bar{x}_k + K_k\,\nu_k, \quad P_k = (1 - K_k)\,\bar{P}_k$ | 1D update | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $x = (1 - K)\,\bar{x} + K z$ | the update as a weighted average | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $P(K) = (1-K)^2\,\bar{P} + K^2 R \Rightarrow K = \frac{\bar{P}}{\bar{P} + R}$ | minimum-variance derivation of the gain | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $1/P = 1/\bar{P} + 1/R$ | information adds (product of two Gaussians) | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $M = \frac{Q + \sqrt{Q^2 + 4QR}}{2}, \quad K_\infty = \frac{M}{M + R}, \quad P_\infty = K_\infty R$ | steady-state gain and variance (depend only on $Q/R$) | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $Q = q\,\Delta t$ | process noise per step from a random-walk rate | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| reject if $\nu^2 / S > 9$ | 3σ outlier rejection | [10.04](../10-localization/10.04-kalman-filter-1d.md) |

### Kalman filter (multivariate) and consistency
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{x}_k = F\mathbf{x}_{k-1} + B\mathbf{u}_k + \mathbf{w}_k,\ \mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, Q); \quad \mathbf{z}_k = H\mathbf{x}_k + \mathbf{v}_k,\ \mathbf{v}_k \sim \mathcal{N}(\mathbf{0}, R)$ | linear state-space model | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\bar{\mathbf{x}}_k = F\mathbf{x}_{k-1} + B\mathbf{u}_k, \quad \bar P_k = F P_{k-1} F^\top + Q$ | KF predict | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\boldsymbol\nu = \mathbf{z} - H\bar{\mathbf{x}}, \ S = H\bar P H^\top + R, \ K = \bar P H^\top S^{-1}, \ \mathbf{x} = \bar{\mathbf{x}} + K\boldsymbol\nu, \ P = (I - KH)\bar P$ | KF update | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $P = (I - KH)\bar P(I - KH)^\top + K R K^\top$ | Joseph-form covariance update (stays symmetric) | [10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \ G = \begin{bmatrix} \tfrac12\Delta t^2 \\ \Delta t \end{bmatrix}, \ Q = \sigma_a^2 \begin{bmatrix} \tfrac14\Delta t^4 & \tfrac12\Delta t^3 \\ \tfrac12\Delta t^3 & \Delta t^2 \end{bmatrix}$ | constant-velocity model | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $Q = G\,\Sigma_{noise}\,G^\top$ (here $G\sigma_a^2 G^\top$) | process noise from the physical noise source | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\bar{\mathbf{x}} = F\mathbf{x} + G a_{cmd}$ | known acceleration as a control ($B = G$) | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\epsilon = \mathbf{e}^\top P^{-1}\mathbf{e} \sim \chi^2_n$, mean $n$ | NEES consistency test (simulation, needs truth) | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\epsilon_\nu = \boldsymbol\nu^\top S^{-1}\boldsymbol\nu \sim \chi^2_m$, mean $m$ | NIS consistency test (works on the real robot) | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $N\bar\epsilon \sim \chi^2_{Nn}$ | average NEES over $N$ Monte Carlo runs | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| reject if NIS $> \chi^2_m(0.99)$ (6.63 for $m = 1$, 9.21 for $m = 2$) | measurement gating | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |

### Extended Kalman filter
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\bar{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}), \quad \bar P = FPF^\top + GMG^\top$ | EKF predict | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\boldsymbol\nu = \mathbf{z} - h(\bar{\mathbf{x}})$ (bearing wrapped), $H = \partial h/\partial\mathbf{x}$ | EKF innovation | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\mathbf{x} = [x,\ y,\ \theta]^\top, \quad \mathbf{u} = [d_L,\ d_R]^\top$ | EKF pose state and odometry input | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\Delta s = \frac{d_L + d_R}{2}, \ \Delta\theta = \frac{d_R - d_L}{b}, \ \theta_m = \theta + \frac{\Delta\theta}{2}$ | step quantities of the motion model | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix} x + \Delta s\cos\theta_m \\ y + \Delta s\sin\theta_m \\ \mathrm{wrap}(\theta + \Delta\theta)\end{bmatrix}$ | nonlinear odometry motion model | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $h(\mathbf{x}) = \begin{bmatrix} \sqrt{(l_x-x)^2 + (l_y-y)^2} \\ \mathrm{wrap}\big(\mathrm{atan2}(l_y-y,\ l_x-x) - \theta\big)\end{bmatrix}$ | range–bearing measurement model | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $R = \mathrm{diag}(\sigma_r^2,\ \sigma_\varphi^2)$ | range–bearing measurement noise | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\partial\theta_m/\partial d_L = -1/(2b), \ \partial\theta_m/\partial d_R = +1/(2b)$ | why $G$ has the $\Delta s/(2b)$ terms | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $M = \mathrm{diag}\big(k^2 \lvert d_L\rvert,\ k^2\lvert d_R\rvert\big), \quad Q = G\,M\,G^\top$ | process noise from wheel-travel noise | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $H = \begin{bmatrix} -\delta_x/r & -\delta_y/r & 0 \\ \delta_y/q & -\delta_x/q & -1 \end{bmatrix}$ | Jacobian of the range–bearing model | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| accept if $d^2 = \boldsymbol\nu^\top S^{-1}\boldsymbol\nu < 5.991$ | 95 % chi-square gate for a 2D measurement | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\frac{\partial f_i}{\partial x_j} \approx \frac{f_i(\mathbf{x} + \varepsilon \mathbf{e}_j) - f_i(\mathbf{x} - \varepsilon \mathbf{e}_j)}{2\varepsilon}$ | numerical Jacobian check (central difference) | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| expected forward progress $\propto e^{-\sigma_\theta^2/2}$ | true mean shrinks with heading spread (EKF ignores this) | [10.06](../10-localization/10.06-extended-kalman-filter.md) |

### Sensor fusion (IMU and odometry)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta \mathrel{+}= \big[\alpha\,\omega_{gyro} + (1-\alpha)\,\omega_{odom}\big]\,\Delta t$ | complementary filter on heading | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\hat\omega = \frac{\sigma_2^2 z_1 + \sigma_1^2 z_2}{\sigma_1^2 + \sigma_2^2}, \quad \sigma^2 = \frac{\sigma_1^2\sigma_2^2}{\sigma_1^2 + \sigma_2^2}$ | optimal fusion of two measurements (inverse-variance weights) | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| fused $= \omega + w_1b_1 + w_2b_2$ | biases survive fusion as a weighted average | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\hat b_{gyro} = \frac{1}{N}\sum_{i=1}^{N} \omega_{gyro,i}, \quad \sigma_{\hat b} = \frac{\sigma_{gyro}}{\sqrt N}$ | gyro bias from a stationary average and its standard error | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $\omega_{odom} \approx a\,\omega_{true} + c\,v \Rightarrow \omega_{odom}^{cal} = \frac{\omega_{odom} - c\,v}{a}$ | odometry yaw-rate calibration | [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) |
| $z = \omega + b$ (gyro), $z = \omega$ (wheels), $z = (\omega_R r - \omega_L r)/b$ | measurement models for a heading/rate/bias filter | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |

### Particle filter (MCL)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d_L^{(i)} \sim \mathcal N(d_L,\ k^2\lvert d_L\rvert), \ d_R^{(i)} \sim \mathcal N(d_R,\ k^2\lvert d_R\rvert), \ \mathbf{x}^{(i)} \leftarrow f(\mathbf{x}^{(i)}, [d_L^{(i)}, d_R^{(i)}])$ | sampling the motion model per particle | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\mathbf{e}_j^{(i)} = \begin{bmatrix} x_i + r_j\cos(\theta_i + a_j) \\ y_i + r_j\sin(\theta_i + a_j)\end{bmatrix}, \ d_j^{(i)} = \mathrm{dist}(\mathbf{e}_j^{(i)},\ \text{nearest obstacle})$ | beam endpoint in the map and its distance to a wall | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $p_j = z_{hit}\exp\!\left(-\frac{d_j^2}{2\sigma_{hit}^2}\right) + \frac{z_{rand}}{z_{max}}, \quad w_i \propto \prod_{j=1}^{K} p_j$ | likelihood-field measurement model (AMCL) | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $w_i \propto p(\mathbf{z}_k \mid \mathbf{x}^{(i)}_k)$ | particle weight when the proposal is the motion model | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\log w_i = \sum_j \log p_j$, subtract the max, then exponentiate | log-space weights (avoid underflow) | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $u_j = \frac{u_0 + j}{N}, \ j = 0\ldots N-1, \ u_0 \sim \mathcal U[0, 1)$ | systematic (low-variance) resampling teeth | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| copies $= \lfloor Nw\rfloor$ or $\lceil Nw\rceil$ | guarantee of systematic resampling | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $N_{eff} = \frac{1}{\sum_i w_i^2}$; resample when $N_{eff} < N/2$ | effective sample size and resampling rule | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| error $\propto 1/\sqrt N$ | Monte Carlo error shrinks with particle count | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\hat\theta = \mathrm{atan2}\big(\sum_i w_i\sin\theta_i,\ \sum_i w_i\cos\theta_i\big)$ | circular mean of particle headings | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $w_{slow} \mathrel{+}= \alpha_{slow}(w_{avg} - w_{slow})$ (same for $w_{fast}$); inject $\max(0, 1 - w_{fast}/w_{slow})$ | augmented MCL: random-particle injection for recovery | [10.08](../10-localization/10.08-particle-filter-mcl.md) |

### Fiducial landmarks
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T_{map\,base} = T_{map\,tag} \cdot (T_{base\,tag})^{-1}$ | robot pose directly from one tag (tempting shortcut) | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $T_{base\,tag} = T_{base\,optical} \cdot T_{optical\,tag}$; $r = \sqrt{x^2 + y^2}$, $\varphi = \mathrm{atan2}(y, x)$ | range and bearing from a tag detection | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $w_{px} = \frac{s}{r} f_x, \quad f_x = \frac{W/2}{\tan(\text{HFOV}/2)}$ | tag size in pixels; focal length in pixels | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| usable range $\approx s \cdot f_x / 30$ | farthest range at which a tag spans ~30 px | [10.10](../10-localization/10.10-fiducial-landmarks.md) |
| $\sigma_r = a\,r$, $\sigma_\varphi$ constant, $R = \mathrm{diag}(\sigma_r^2, \sigma_\varphi^2)$ | measured detector noise model | [10.10](../10-localization/10.10-fiducial-landmarks.md) |

### Simulation of differential equations
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\dot x = f(x, u, t)$ | first-order ODE (state-space form) | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\dot\omega = \frac{K u - \omega}{\tau}$ | karmel's wheel as an ODE | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $x_{k+1} = x_k + \Delta t\, f(x_k, u_k, t_k)$ | forward Euler step | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\omega_{k+1} - \omega_\infty = \left(1 - \frac{\Delta t}{\tau}\right)(\omega_k - \omega_\infty)$ | Euler on a first-order system: gap shrinks by $a = 1 - \Delta t/\tau$ | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| stable if $\lvert 1 - \Delta t/\tau\rvert < 1 \iff \Delta t < 2\tau$; no ringing if $\Delta t < \tau$; ~1 % if $\Delta t \lesssim \tau/10$ | Euler step-size rules | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\omega_{k+1} = \omega_\infty + e^{-\Delta t/\tau}(\omega_k - \omega_\infty)$ | exact discrete update (stable for any step) | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\dot x = \begin{bmatrix}\omega \\ \frac{K\,\mathrm{sat}(K_p(\theta_r - \theta)) - \omega}{\tau}\end{bmatrix}$ | wheel-angle P loop as a first-order system with saturation | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $\Delta t < 2/\lvert\lambda_{fast}\rvert$ | Euler must respect the fastest eigenvalue (stiff systems) | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| $x_{k+1} = e^{A\Delta t}x_k + \dots$ | exact discretisation via the matrix exponential | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |

### Discrete time and sampling
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $f_s = 1/T$ | sample rate | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $f_{alias} = \left\lvert f - f_s \cdot \mathrm{round}\!\left(\frac{f}{f_s}\right) \right\rvert$ | apparent frequency after sampling (aliasing) | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| Nyquist frequency $= f_s/2$ | highest frequency represented faithfully | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $\frac{v}{r}\cdot\frac{56}{2\pi}$ | motor shaft frequency at ground speed $v$ (56:1 gearbox) | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $\omega_{k+1} = a\,\omega_k + b\,u_k, \quad a = e^{-T/\tau},\ b = K(1 - a)$ | exact zero-order-hold model of the wheel | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $z = e^{pT}$; stable if $\lvert z\rvert < 1$ | continuous pole to discrete pole; unit-circle stability | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $z = a - bK_p$, stable if $K_p < \frac{1+a}{b}$ | P speed loop with no computation delay | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $z^2 - az + bK_p = 0$, $K_p < \frac{1}{b} = \frac{1}{K\left(1 - e^{-T/\tau}\right)}$ | P speed loop with one-sample delay: critical gain | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $b \approx KT/\tau$ for small $T$ | critical gain grows roughly with loop rate | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| effective delay $\approx 1.5\,T$ | ZOH half-period plus one-sample compute delay | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| $f_{bw} = 1/(2\pi\tau_{cl})$; sample at 10–30 × $f_{bw}$ | bandwidth from closed-loop time constant; loop-rate rule | [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |
| outer loop 3–10 × slower than inner (bandwidth) | cascade rule | [08.10](../08-control/08.10-straight-line-heading-control.md), [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md) |

### State space, LQR, MPC and MPPI
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\dot x = f(x, u)$; linear: $\dot x = Ax + Bu$ | state-space model; poles are the eigenvalues of $A$ | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $u = -Kx$ | state feedback | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\dot x = (A - BK)x$ | closed loop under state feedback (poles = eigenvalues of $A - BK$) | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $J = \int_0^\infty \left(x^\top Q x + u^\top R u\right)\,dt$ | LQR cost | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $A^\top P + PA - PBR^{-1}B^\top P + Q = 0, \quad K = R^{-1}B^\top P$ | continuous algebraic Riccati equation and LQR gain | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $K = (R + B^\top P B)^{-1}B^\top P A$ | discrete LQR gain | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\dot x = \begin{bmatrix}0 & 1\\ 0 & -1/\tau\end{bmatrix} x + \begin{bmatrix}0\\ K/\tau\end{bmatrix} u$ | wheel (angle, speed) under duty control | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\dot x = \begin{bmatrix}0&1\\0&0\end{bmatrix}x + \begin{bmatrix}0\\1\end{bmatrix}u$ | double integrator (position, velocity; acceleration input) | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $x_{k+1} = A_d x_k + B_d u_k$; double integrator: $A_d = \begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}$, $B_d = \begin{bmatrix}\Delta t^2/2\\ \Delta t\end{bmatrix}$ | zero-order-hold discretisation | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\min_{u_0\dots u_{N-1}} \sum_{k=1}^{N} x_k^\top Q x_k + R\sum_{k=0}^{N-1} u_k^2$ s.t. $x_{k+1} = A_d x_k + B_d u_k$, $\lvert u_k\rvert \le u_{max}$, $\lvert v_k\rvert \le v_{max}$ | model predictive control (MPC) problem | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $\epsilon^{(m)} \sim \mathcal N(0, \Sigma)$ | MPPI sampled control perturbations | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $w^{(m)} = \exp\!\big(-(S^{(m)} - S_{min})/\lambda\big)$, normalised | MPPI softmax weights | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| $U \leftarrow U + \sum_m w^{(m)}\epsilon^{(m)}$ | MPPI control-sequence update | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |

## SLAM, navigation and computer vision (11–13, FCV)

### Maps and occupancy grids
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $M_{2D} = \left\lceil \tfrac{W}{r} \right\rceil \left\lceil \tfrac{D}{r} \right\rceil b$ | memory of a dense 2D grid | [11.01](../11-slam/11.01-map-representations.md) |
| $M_{3D} = \left\lceil \tfrac{W}{r} \right\rceil \left\lceil \tfrac{D}{r} \right\rceil \left\lceil \tfrac{H}{r} \right\rceil b$ | memory of a dense 3D grid | [11.01](../11-slam/11.01-map-representations.md) |
| $s/2^d$ | leaf cell size of an octree of depth $d$ | [11.01](../11-slam/11.01-map-representations.md) |
| $l = \ln\frac{p}{1-p}$ | log-odds from probability | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p = 1 - \frac{1}{1 + e^{l}}$ | probability from log-odds | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $l_{t,i} = l_{t-1,i} + \ln\frac{p(m_i \mid z_t, x_t)}{1 - p(m_i \mid z_t, x_t)} - l_0$ | log-odds occupancy update (one addition per beam) | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p(m_i \mid z_{1:t}) = \frac{p(z_t \mid m_i)\, p(m_i \mid z_{1:t-1})}{p(z_t \mid z_{1:t-1})}$ | Bayes' rule for one cell | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\frac{p(m_i \mid z_{1:t})}{p(\neg m_i \mid z_{1:t})} = \frac{p(z_t \mid m_i)}{p(z_t \mid \neg m_i)} \cdot \frac{p(m_i \mid z_{1:t-1})}{p(\neg m_i \mid z_{1:t-1})}$ | odds form of the update (normalizer cancels) | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\max(\lvert dx\rvert, \lvert dy\rvert) + 1$ | number of cells a Bresenham beam crosses | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $\text{col} = \lfloor (x - o_x)/r \rfloor,\ \text{row} = \lfloor (y - o_y)/r \rfloor$ | world point → grid cell | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $(x_s + r_k\cos(\theta_s + \alpha_k),\ y_s + r_k\sin(\theta_s + \alpha_k))$ | LiDAR beam end point in the map frame | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |
| $p = (255 - \text{pixel})/255$ | occupancy probability of a map PGM pixel | [11.02](../11-slam/11.02-occupancy-grid-mapping.md) |

### SLAM estimation
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $p(x_{1:t},\, m \mid z_{1:t},\, u_{1:t})$ | full SLAM: whole trajectory and map | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $p(x_t,\, m \mid z_{1:t},\, u_{1:t}) = \int \cdots \int p(x_{1:t},\, m \mid z_{1:t},\, u_{1:t})\; dx_1 \cdots dx_{t-1}$ | online SLAM: current pose and map | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $p(x_{1:t}, m \mid z, u) \propto p(x_0) \prod_{k} p(x_k \mid x_{k-1}, u_k) \prod_{k} p(z_k \mid x_k, m)$ | SLAM posterior factorized into motion and measurement terms | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $x^* = \arg\min_x \sum e_{ij}(x)^\top \,\Omega_{ij}\, e_{ij}(x)$ | graph SLAM as weighted least squares | [11.03](../11-slam/11.03-the-slam-problem.md), [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $3 + 2n$ state, $(3+2n)^2$ covariance entries | EKF-SLAM size with $n$ landmarks | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $r\,\delta\theta$ (exactly $\Delta = r \sin \delta\theta$) | sideways error of a point at range $r$ from heading error | [11.03](../11-slam/11.03-the-slam-problem.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\nu = z - \hat z$ | innovation | [11.03](../11-slam/11.03-the-slam-problem.md) |
| $d^2 = \nu^\top S^{-1} \nu$ | squared Mahalanobis distance for data association (gate 5.99 for 2 DOF, 95%) | [11.03](../11-slam/11.03-the-slam-problem.md) |

### Scan matching (ICP)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $E(R, t) = \sum_i \lVert R p_i + t - q_i \rVert^2$ | point-to-point alignment error | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $t = \bar q - R \bar p$ | optimal translation | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $\bar p = \tfrac{1}{N}\sum p_i,\ \bar q = \tfrac{1}{N}\sum q_i$ | centroids | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $p'_i = p_i - \bar p,\ q'_i = q_i - \bar q$ | centered points | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $H = \sum_i p'_i\, q_i'^\top$ | cross-covariance (source first) | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $H = U S V^\top,\ R = V U^\top$ (if $\det = -1$: $R = V \operatorname{diag}(1, -1)\, U^\top$) | Kabsch rotation via SVD | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $\theta = \operatorname{atan2}(H_{xy} - H_{yx},\ H_{xx} + H_{yy})$ | 2D Kabsch shortcut | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $e_i = n_i^\top (R p_i + t - q_i)$ | point-to-line ICP error | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $R p \approx p + \delta\theta\,(-p_y, p_x)$ | small-angle linearized rotation | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| smallest/largest eigenvalue of $\sum n n^\top$ | degeneracy check (corridor) | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $O(\log n)$ per query | KD-tree nearest neighbour cost | [11.04](../11-slam/11.04-scan-matching-icp.md) |

### Pose graphs
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\hat z_{ij} = X_i^{-1} X_j$ | relative pose predicted by the graph | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $e_{ij} = \operatorname{t2v}\!\left(Z_{ij}^{-1}\,(X_i^{-1} X_j)\right) = \begin{pmatrix} R_z^\top \left(R_i^\top (t_j - t_i) - t_z\right) \\ \operatorname{wrap}(\theta_j - \theta_i - \theta_z) \end{pmatrix}$ | 2D pose-graph edge error | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $F(x) = \sum_{(i,j)} e_{ij}(x)^\top\, \Omega_{ij}\, e_{ij}(x)$ | total graph cost ($\chi^2$) | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\Omega = \Sigma^{-1}$, e.g. $\operatorname{diag}(1/\sigma_x^2, 1/\sigma_y^2, 1/\sigma_\theta^2)$ | information matrix from standard deviations | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $e_{ij}(x + \Delta x) \approx e_{ij} + A_{ij}\,\Delta x_i + B_{ij}\,\Delta x_j$ | linearized edge error | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $A_{ij} = \begin{pmatrix} -R_z^\top R_i^\top & R_z^\top \frac{\partial R_i^\top}{\partial \theta_i} (t_j - t_i) \\ 0 & -1 \end{pmatrix}$, $B_{ij} = \begin{pmatrix} R_z^\top R_i^\top & 0 \\ 0 & 1 \end{pmatrix}$ | edge Jacobians | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $\frac{\partial R^\top}{\partial\theta} = \begin{pmatrix} -\sin\theta & \cos\theta \\ -\cos\theta & -\sin\theta \end{pmatrix}$ | derivative of a transposed 2D rotation | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $H\,\Delta x = -b$ | Gauss-Newton normal equations | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $H_{ii} \mathrel{+}= A^\top \Omega A,\ H_{ij} \mathrel{+}= A^\top \Omega B,\ H_{ji} \mathrel{+}= B^\top \Omega A,\ H_{jj} \mathrel{+}= B^\top \Omega B,\ b_i \mathrel{+}= A^\top \Omega e,\ b_j \mathrel{+}= B^\top \Omega e$ | how each edge fills $H$ and $b$ | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $x \leftarrow x + \Delta x$ | Gauss-Newton update (then wrap angles) | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md) |
| $e^\top \Omega e$ per edge vs $\chi^2$ threshold (7.81 for 3 DOF, 95%) | false-loop-closure check | [11.05](../11-slam/11.05-pose-graphs-loop-closure.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |

### SLAM tuning and troubleshooting
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\Delta\theta = \max(0.3,\ 0.5\,\omega)$ | rotation between slam_toolbox keyframes while turning | [11.06](../11-slam/11.06-slam-toolbox-simulation.md), [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $\Delta x = v\,\delta t,\ \Delta\theta = \omega\,\delta t$ | pose error from timestamp skew | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $r(\theta) = \frac{h}{p \cos\theta}$ (small $p$) | where a pitched LiDAR's beams hit the floor | [11.09](../11-slam/11.09-slam-troubleshooting.md) |
| $a_x = -g\sin p + b$ | IMU forward acceleration at rest on a pitch | [11.09](../11-slam/11.09-slam-troubleshooting.md) |

### Visual SLAM
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\min_{\{T_k\},\,\{X_j\}} \sum_{k}\sum_{j \in \mathcal{V}(k)} \rho\!\left( \left\lVert \pi\!\left(K,\, T_k^{-1} X_j\right) - u_{kj} \right\rVert^2_{\Sigma_{kj}} \right)$ | bundle adjustment | [11.08](../11-slam/11.08-visual-slam.md) |
| $6 \times \text{keyframes} + 3 \times \text{points}$ | number of unknowns in bundle adjustment | [11.08](../11-slam/11.08-visual-slam.md) |
| $\sigma_Z = \frac{Z^2}{fB}\,\sigma_d$ | depth uncertainty from disparity noise | [11.08](../11-slam/11.08-visual-slam.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |

### Navigation basics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d_{\text{stop}} = \frac{v^2}{2a}$, $t = v/a$ | braking distance and time | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $J_{\text{global}}(\sigma) = \int_\sigma \big(1 + w \cdot c(s)\big)\, ds$ | global planner cost | [12.01](../12-navigation/12.01-the-navigation-problem.md) |
| $J_{\text{local}}(u) = w_p\, d_{\text{path}} + w_g\, d_{\text{goal}} + w_o\, \text{obstacle}(u) + w_e\, \lVert u\rVert$ | local planner cost (subject to $\lvert v\rvert \le v_{\max}$, $\lvert \dot v\rvert \le a_{\max}$) | [12.01](../12-navigation/12.01-the-navigation-problem.md) |

### Grid path planning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| relax: if $g(n) + \ell < g(m)$ then $g(m) \leftarrow g(n) + \ell$ | Dijkstra / A* edge update | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $f(n) = g(n) + h(n)$ | A* priority | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h(n) \le h^*(n)$ | admissible heuristic | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h(n) \le \ell(n, m) + h(m)$, $h(\text{goal}) = 0$ | consistent heuristic | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $h_{\text{oct}} = \max(\Delta r, \Delta c) + (\sqrt 2 - 1)\min(\Delta r, \Delta c)$ | octile distance | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $\sqrt{\Delta r^2 + \Delta c^2}$ | Euclidean heuristic | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $\Delta r + \Delta c$ | Manhattan heuristic (not admissible on 8-connected grids) | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $f = g + w\,h$, cost $\le w \times$ optimal | weighted A* | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $O(V \log V)$ | Dijkstra complexity with a binary heap | [12.02](../12-navigation/12.02-grid-path-planning.md) |

### Sampling-based planning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $q_{\text{new}} = q_{\text{near}} + \text{step} \cdot \frac{q_{\text{rand}} - q_{\text{near}}}{\lVert q_{\text{rand}} - q_{\text{near}}\rVert}$ | RRT steer step | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| minimize $\text{cost}(q) + \lVert q - q_{\text{new}}\rVert$ | RRT* choose parent | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| rewire if $\text{cost}(q_{\text{new}}) + \lVert q_{\text{new}} - q\rVert < \text{cost}(q)$ | RRT* rewire | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $r_n = \gamma \left(\frac{\log n}{n}\right)^{1/d}$ | RRT* rewiring radius | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $p = A_{\text{narrow}}/A$ | chance a sample lands in a narrow passage | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $P(\text{miss}) = (1 - p)^n \approx e^{-pn}$ | chance of never sampling the passage | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| $k^d$ | cells in a grid with $k$ cells per axis in $d$ dimensions | [12.03](../12-navigation/12.03-sampling-based-planning.md) |

### Costmaps
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $r_{\text{inscribed}} = \min \text{dist}$, $r_{\text{circumscribed}} = \max \text{dist}$ (over footprint vertices and edges) | footprint radii | [12.04](../12-navigation/12.04-costmaps.md) |
| $r_{\text{in}} = b$, $r_{\text{circ}} = \sqrt{a^2 + b^2}$ | radii of a centered rectangle | [12.04](../12-navigation/12.04-costmaps.md) |
| $\text{cost}(d) = 254$ if $d = 0$; $253$ if $0 < d \le r_{\text{in}}$; $\left\lfloor 252 \cdot e^{-k\,(d - r_{\text{in}})} \right\rfloor$ if $d > r_{\text{in}}$ | Nav2 inflation cost | [12.04](../12-navigation/12.04-costmaps.md), [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $d(c) = r_{\text{in}} + \frac{\ln(252 / c)}{k}$ | distance at which inflation cost falls to $c$ | [12.04](../12-navigation/12.04-costmaps.md) |
| $\ell \cdot (1 + w \cdot \text{cost}_n / 252)$ | cost-aware step cost for A* | [12.04](../12-navigation/12.04-costmaps.md) |
| gap $- 2 r_{\text{in}}$ | free band for the robot's center | [12.04](../12-navigation/12.04-costmaps.md) |

### Local planning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $L = \sqrt{x_r^2 + y_r^2}$ | lookahead distance | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $R = v/\omega$ | arc radius of a constant $(v, \omega)$ command | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\kappa = \frac{1}{R} = \frac{2\,y_r}{L^2}$ | pure pursuit curvature | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\omega = v\,\kappa$ | turn rate from curvature | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $L = \text{clamp}(v \cdot \text{lookahead\_time},\ \text{min},\ \text{max})$ | velocity-scaled lookahead | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $v \leftarrow v \cdot R/R_{\min}$ (if $R < R_{\min}$) | RPP curvature slowdown | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $d = (k\,r_{\text{in}} - \ln c + \ln 253)/k$ | obstacle distance from a cost value (RPP) | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $v \leftarrow v \cdot \text{gain} \cdot d / 0.3$ | RPP proximity slowdown | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $v = v_{\max} \cdot d_{\text{left}} / d_{\text{approach}}$ | RPP approach slowdown near the goal | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $V_d = [v - a_v\Delta t,\ v + a_v\Delta t] \cap [v_{\min}, v_{\max}]$, $\Omega_d = [\omega - a_\omega\Delta t,\ \omega + a_\omega\Delta t] \cap [-\omega_{\max}, \omega_{\max}]$ | dynamic window | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $x(t) = \frac{v}{\omega}\sin\omega t,\ y(t) = \frac{v}{\omega}(1 - \cos\omega t)$ | arc rollout of a constant command | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $J = 2.0\, d_{\text{path}} + 3.0\, d_{\text{carrot}} + \frac{0.1}{\text{clearance}} + 1.0\,\frac{v_{\max} - v}{v_{\max}}$ | course DWA scoring | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $w_k = \frac{\exp(-(S_k - S_{\min})/\lambda)}{\sum_j \exp(-(S_j - S_{\min})/\lambda)}$ | MPPI sample weights | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $U \leftarrow U + \sum_k w_k\,\varepsilon_k$ | MPPI control update | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $L(1 - 1/\sqrt 2) \approx 0.3L$ | pure pursuit corner cut at 90° | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |

### Nav2 on the real robot and safety
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $c = w - 2r$ | free corridor through a door | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $L(1-\cos(\theta/2))$ | corner cut of a pure pursuit arc at corner angle $\theta$ | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $d_{\text{stop}} = v\,(T_{\text{sensor}} + T_{\text{costmap}} + T_{\text{controller}}) + \frac{v^2}{2a}$ | stopping distance with latency | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $\text{tolerance} \ge 2\sigma_{\text{AMCL}} + v\,T_{\text{latency}} + \text{margin}$ | minimum goal tolerance | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $\sigma = \sqrt{C_{xx}}$ | position noise from a pose covariance | [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md) |
| $\text{cmd}_{\text{out}} = \text{cmd}_{\text{in}} \cdot \frac{t_c}{T_{\text{before}}}$ | collision monitor `approach` slowdown | [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md) |

### Pixels and color
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $Y = 0.299\,R + 0.587\,G + 0.114\,B$ | grayscale (BT.601 luma) | [13.01](../13-computer-vision/13.01-images-as-data.md), [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $V = M,\ S = 255 \cdot \frac{C}{M}$ | HSV value and saturation (OpenCV 8-bit) | [13.01](../13-computer-vision/13.01-images-as-data.md) |
| $H_{deg} = 60 \frac{G - B}{C}$ ($M=R$); $120 + 60 \frac{B - R}{C}$ ($M=G$); $240 + 60 \frac{R - G}{C}$ ($M=B$); OpenCV $H = H_{deg}/2$ | HSV hue | [13.01](../13-computer-vision/13.01-images-as-data.md), [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $c_{lin} = \left(\frac{c + 0.055}{1.055}\right)^{2.4}$ ($c > 0.04045$), else $c/12.92$ | undo sRGB gamma | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $L^* = 116 f(Y/Y_n) - 16$, $a^* = 500\,[f(X/X_n) - f(Y/Y_n)]$, $b^* = 200\,[f(Y/Y_n) - f(Z/Z_n)]$, $f(t) = t^{1/3}$ | CIELAB from XYZ | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $\Delta E^*_{76} = \sqrt{\Delta L^{*2} + \Delta a^{*2} + \Delta b^{*2}}$ | color difference | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $h = \operatorname{atan2}(b^*, a^*)$, $C^* = \sqrt{a^{*2} + b^{*2}}$ | LCh hue and chroma | [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md) |
| $255 \times x^{1/2.2}$ | gamma-encoded pixel value for linear light $x$ | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |

### Image sensors
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{SNR} = \frac{N}{\sqrt{N + \sigma_r^2}}$ | signal-to-noise ratio with shot and read noise | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $\text{DR} = 20\log_{10}(\text{full well}/\sigma_r)$ dB $= \log_2(\text{full well}/\sigma_r)$ stops | dynamic range | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $b_{rot} \approx f_{px}\,\omega\,t$ | motion blur from rotation | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md), [11.08](../11-slam/11.08-visual-slam.md) |
| $b_{trans} \approx f_{px}\,v\,t/Z$ | motion blur from sideways translation | [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |

### Filtering and thresholds
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $I'(y, x) = \sum_{i=-k}^{k}\sum_{j=-k}^{k} w(i, j)\, I(y + i, x + j)$ | kernel filtering | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $G(x, y) = \sum_{i}\sum_{j} K(i, j)\; I(x + i,\ y + j)$ | correlation (`cv2.filter2D`) | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $G(x, y) = \sum_{i}\sum_{j} K(i, j)\; I(x - i,\ y - j)$ | true convolution (kernel flipped) | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\sigma' = \sigma \sqrt{\sum_{i,j} w(i,j)^2}$ (box of $N$ pixels: $\sigma/\sqrt{N}$) | noise after a linear filter | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md), [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $e^{-(x^2+y^2)/2\sigma^2}$, size $\approx 6\sigma$ | Gaussian kernel | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $2k$ vs $k^2$ multiplications | cost of a separable vs full $k \times k$ kernel | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\lvert\nabla I\rvert = \sqrt{G_x^2 + G_y^2}$, $\theta = \operatorname{atan2}(G_y, G_x)$ | gradient magnitude and direction | [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md) |
| $\sigma_B^2(t) = \omega_0(t)\,\omega_1(t)\,\big[\mu_0(t) - \mu_1(t)\big]^2$ | Otsu between-class variance | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |
| $T(y, x) = \text{GaussianMean}(y, x) - C$, dark if $I < T$ | adaptive threshold | [13.02](../13-computer-vision/13.02-opencv-preprocessing.md) |

### Shape and blob detection
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $M_{pq} = \sum_{x,y} x^p y^q\, I(x, y)$ | image moments | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $\bar{u} = \frac{M_{10}}{M_{00}},\ \bar{v} = \frac{M_{01}}{M_{00}}$ | centroid | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $c = \frac{4\pi A}{P^2}$ | circularity | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $f = \frac{A}{\pi r_e^2}$ | fill ratio | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $r_{exp} = f R (v - c_y) / (f h)$ | expected ball radius at image row $v$ | [13.03](../13-computer-vision/13.03-classical-detection.md) |
| $A = N,\ \bar u = \frac{1}{N}\sum_i u_i,\ \bar v = \frac{1}{N}\sum_i v_i$ | mask area and centroid | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $\Sigma = \frac{1}{N-1}\sum_i \begin{pmatrix} u_i - \bar u \\ v_i - \bar v \end{pmatrix}\begin{pmatrix} u_i - \bar u & v_i - \bar v \end{pmatrix}$ | mask covariance (shape and orientation) | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $\text{elongation} = \sqrt{\lambda_1/\lambda_2}$ ($= L/W$ for a rectangle; variances $L^2/12$, $W^2/12$) | elongation | [13.11](../13-computer-vision/13.11-segmentation.md) |
| $\lvert M_1 \cap M_2\rvert / \lvert M_1 \cup M_2\rvert$ | mask IoU | [13.11](../13-computer-vision/13.11-segmentation.md) |

### Features and matching
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $E(\Delta x, \Delta y) = \sum_{w} [I(x + \Delta x, y + \Delta y) - I(x, y)]^2$ | window shift error | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $E \approx \begin{bmatrix}\Delta x & \Delta y\end{bmatrix} M \begin{bmatrix}\Delta x \\ \Delta y\end{bmatrix}$, $M = \sum_{w}\begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$ | structure tensor | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $R = \det M - k\,(\operatorname{tr} M)^2 = \lambda_1\lambda_2 - k(\lambda_1 + \lambda_2)^2$ | Harris corner response | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $\min(\lambda_1, \lambda_2)$ | Shi–Tomasi score | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $\frac{d_1}{d_2} < \rho$ | Lowe's ratio test | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $[u', v', 1]^\top \sim H [u, v, 1]^\top$, $H = K R K^{-1}$ | homography of a pure camera rotation | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $R = K^{-1} H K$, yaw $= \operatorname{atan2}(R_{02}, R_{22})$ | rotation from a homography | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| $N = \frac{\log(1 - p)}{\log(1 - w^s)}$ | RANSAC iterations needed | [13.04](../13-computer-vision/13.04-features-and-matching.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |

### Pinhole camera and projection
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $f_x = \frac{W/2}{\tan(\text{HFOV}/2)}$ | focal length in pixels from field of view | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [11.08](../11-slam/11.08-visual-slam.md), [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md) |
| $\text{VFOV} = 2\arctan(c_y/f_y)$, $\text{HFOV} = 2\arctan(W/(2 f_x))$ | field of view from focal length | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$ | intrinsic matrix | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $u = f_x \frac{X}{Z} + c_x,\ v = f_y \frac{Y}{Z} + c_y$ | pinhole projection | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $s [u, v, 1]^\top = K \,[\,R \mid t\,]\, [P_w, 1]^\top$, $[R \mid t]$ = top rows of $T_{world,optical}^{-1}$ | projection with extrinsics | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $X = \frac{(u - c_x)\,Z}{f_x},\ Y = \frac{(v - c_y)\,Z}{f_y}$ | back-projection with known depth | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $d = \left(\frac{u - c_x}{f_x}, \frac{v - c_y}{f_y}, 1\right)$ | ray through a pixel | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $f H / Z$ (image size); $Z \approx \frac{f R}{r}$ | distance from apparent size | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $D = R / \sin(\arctan(r/f))$ | exact distance to a sphere's center | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $Z = \frac{f_y\,(h_c - h_o)}{v - c_y}$ | ground-plane distance from image row | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\left\lvert\frac{\partial Z}{\partial r}\right\rvert = \frac{Z^2}{f R}$, $\left\lvert\frac{\partial Z}{\partial v}\right\rvert = \frac{Z^2}{f\,(h_c - h_o)}$ | distance sensitivity to pixel errors | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $f \tan(\text{pitch})$ | row shift from camera pitch | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $f' = s f$, $c_x' = (c_x + 0.5)\, s - 0.5$ | rescaling $K$ when resizing by factor $s$ | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |
| $\hat{x} = (x_1/\sigma_1^2 + x_2/\sigma_2^2)/(1/\sigma_1^2 + 1/\sigma_2^2)$ | inverse-variance fusion of two estimates | [13.05](../13-computer-vision/13.05-pinhole-camera-model.md) |

### Lens distortion and calibration
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x = X/Z,\ y = Y/Z,\ r^2 = x^2 + y^2$ | normalized image coordinates | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $x_d = x\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 x y + p_2 (r^2 + 2x^2)$ | plumb-bob distortion (x) | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $y_d = y\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2y^2) + 2 p_2 x y$ | plumb-bob distortion (y) | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $u = f_x x_d + c_x,\ v = f_y y_d + c_y$ | distorted pixel | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $\min_{K,\,d,\,\{R_i, t_i\}} \sum_{i}\sum_{j} \lVert \hat{u}_{ij} - \pi(K, d, R_i P_j + t_i) \rVert^2$ | camera calibration objective | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $\sqrt{\frac{1}{N}\sum \lVert\hat{u}_{ij} - \pi(\ldots)\rVert^2}$ | RMS reprojection error | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |
| $f S / Z$ | image size of a checkerboard square | [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md) |

### Fiducial markers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\min_{R,\,t} \sum_{j=1}^{4} \lVert \hat{u}_j - \pi(K, d,\, R P_j + t) \rVert^2$ | marker pose (PnP) | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $P_{1..4} = (\mp\tfrac{L}{2}, \pm\tfrac{L}{2}, 0)$ | marker corner coordinates | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $Z \approx f L / s$, $\Delta Z = Z/s$ per px | range from marker size and its error | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $e_2/e_1$ | pose ambiguity ratio | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $T_{base,tag} = T_{base,optical} \; T_{optical,tag}$ | marker pose in the robot frame | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| $p_{goal} = p_{tag} + s\,\hat{z}_{tag}$, $\psi = \operatorname{atan2}(-\hat{z}_{tag,y}, -\hat{z}_{tag,x})$ | docking goal in front of a tag | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |

### Depth and stereo
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = u_L - u_R = \frac{f\,B}{Z}$, $Z = \frac{f\,B}{d}$ | depth from disparity | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\left\lvert\frac{\partial Z}{\partial d}\right\rvert = \frac{f B}{d^2} = \frac{Z^2}{f B}$, $\Delta Z \approx \frac{Z^2}{f\,B}\,\Delta d$ | stereo depth error | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $Z_{min} = fB/D$ | minimum stereo depth | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $B = \frac{Z^2\,\Delta d}{f\,\Delta Z}$ | baseline needed for a depth accuracy | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\delta = \frac{c_- - c_+}{2(c_- - 2c_0 + c_+)}$ | sub-pixel disparity by parabola fit | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\sigma_Z \approx k Z^2$ | depth camera noise model | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $Z = r / \lVert(x_n, y_n, 1)\rVert$ | depth from Euclidean range | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $p_w = R\,p_c + t$ | transform points to the world frame | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $\mathbf{n} \cdot \mathbf{p} + d = 0$ | plane equation (left side = signed distance) | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |
| $\mathbf{n} = \frac{(\mathbf{p}_2 - \mathbf{p}_1) \times (\mathbf{p}_3 - \mathbf{p}_1)}{\lVert\cdot\rVert},\ d = -\mathbf{n} \cdot \mathbf{p}_1$ | plane from three points | [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md) |

### Projective and epipolar geometry
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{l}^{\mathsf T}\mathbf{p} = 0$ | point lies on line | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $\mathbf{l} = \mathbf{p} \times \mathbf{q}$, $\mathbf{x} = \mathbf{l}_1 \times \mathbf{l}_2$ | line through two points, intersection of two lines | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $[u', v', w']^\top = H [x, y, 1]^\top$, $(u, v) = (u'/w', v'/w')$ | applying a homography | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $\begin{bmatrix} -x & -y & -1 & 0&0&0 & ux & uy & u \\ 0&0&0 & -x & -y & -1 & vx & vy & v \end{bmatrix}\mathbf{h} = \mathbf{0}$ | DLT rows per correspondence ($A\mathbf{h} = 0$, solve by SVD) | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $\mathbf{p} \sim K\,[\,\mathbf{r}_1\ \mathbf{r}_2\ \mathbf{t}\,]\,[X, Y, 1]^\top$ | floor homography | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $v_h = c_y - f\tan(\text{pitch})$ | horizon row | [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md) |
| $E = [\mathbf{t}]_\times R$ | essential matrix | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\hat{\mathbf{x}}_2^{\mathsf T} E\, \hat{\mathbf{x}}_1 = 0$, $\mathbf{x}_2^{\mathsf T} F\, \mathbf{x}_1 = 0$, $F = K_2^{-\mathsf T} E K_1^{-1}$ | epipolar constraint | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |
| $\mathbf{l}_2 = F\mathbf{x}_1$ | epipolar line | [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md) |

### Neural networks
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x'_c = \frac{x_c/255 - \mu_c}{\sigma_c}$ | ImageNet input normalization | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $p_i = \frac{e^{z_i}}{\sum_{j=1}^{K} e^{z_j}}$ | softmax | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $W_{out} = \left\lfloor \frac{W + 2p - k}{s} \right\rfloor + 1$ | convolution output size | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $k \cdot k \cdot C_{in} \cdot C_{out} + C_{out}$ | convolution parameter count | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |
| $t_p < 1/f$ | perception loop-rate budget | [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md) |

### Detection metrics and tracking
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathrm{IoU}(A,B) = \frac{\lvert A \cap B\rvert}{\lvert A\rvert + \lvert B\rvert - \lvert A \cap B\rvert}$ | intersection over union | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $w_\cap = \max(0,\ \min(x_2^A, x_2^B) - \max(x_1^A, x_1^B))$, same for $h_\cap$ | box overlap size | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $\text{precision} = \frac{TP}{TP + FP},\ \text{recall} = \frac{TP}{TP + FN}$ | precision and recall | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $F_1 = \frac{2PR}{P + R}$ | F1 score | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $AP = \sum_k (r_k - r_{k-1})\,p_{interp}(r_k)$ | all-point average precision | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $\text{mAP} = \frac{1}{\lvert C\rvert}\sum_{c} AP_c$ | mean average precision | [13.10](../13-computer-vision/13.10-object-detection-models.md), [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| $\sum (1 - \mathrm{IoU})$ | Hungarian assignment cost | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\mathbf{x}_{k\mid k-1} = F\mathbf{x}_{k-1},\ P_{k\mid k-1} = FPF^\top + Q$ | Kalman predict | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\mathbf{z}_k = H\mathbf{x}_k + \mathbf{v},\ \mathbf{v}\sim\mathcal N(0,R)$ | measurement model | [13.12](../13-computer-vision/13.12-tracking.md) |
| $K = P_{k\mid k-1}H^\top (HP_{k\mid k-1}H^\top + R)^{-1}$ | Kalman gain | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\mathbf{x}_k = \mathbf{x}_{k\mid k-1} + K(\mathbf{z}_k - H\mathbf{x}_{k\mid k-1}),\ P_k = (I - KH)P_{k\mid k-1}$ | Kalman update | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\mathrm{MOTA} = 1 - \frac{\sum_t (FN_t + FP_t + IDSW_t)}{\sum_t GT_t}$ | multi-object tracking accuracy | [13.12](../13-computer-vision/13.12-tracking.md) |
| $\Delta u \approx f\,\omega\,\Delta t$ | image shift while turning | [13.12](../13-computer-vision/13.12-tracking.md) |

### Embeddings and vision-language models
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\cos(\mathbf a, \mathbf b) = \frac{\mathbf a \cdot \mathbf b}{\lVert\mathbf a\rVert\,\lVert\mathbf b\rVert}$ | cosine similarity | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $p_k = \frac{e^{s\,\cos(\mathbf v, \mathbf t_k)}}{\sum_j e^{s\,\cos(\mathbf v, \mathbf t_j)}}$ | CLIP zero-shot probabilities | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $p = \sigma(s\cos + b)$ | SigLIP per-phrase score | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $\mathcal L = -\frac{1}{N}\sum_i \log \frac{e^{s\cos(\mathbf v_i,\mathbf t_i)}}{\sum_j e^{s\cos(\mathbf v_i,\mathbf t_j)}}$ | contrastive training loss | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| $\text{tokens} = \left\lceil \frac{w}{28} \right\rceil \cdot \left\lceil \frac{h}{28} \right\rceil$ | image tokens of a frame | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $c = \frac{(\text{image tokens} + \text{prompt tokens})\cdot p_\text{in} + \text{output tokens} \cdot p_\text{out}}{10^6}$ | cost per VLM question | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| $d = v t$ | distance driven while waiting for an answer | [13.14](../13-computer-vision/13.14-vision-language-models.md) |

### Vision in ROS 2 and mapping detections
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $B = H \times W \times c$, $\text{rate} = B \cdot f$ | image message size and bandwidth | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $\text{age} \approx q \cdot L$, throughput $= \min(f, 1/L)$ | staleness of a detection | [13.15](../13-computer-vision/13.15-vision-in-ros2.md) |
| $e \approx r\,\omega\,\Delta t$ | position error from stale data while turning | [13.15](../13-computer-vision/13.15-vision-in-ros2.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $Z = f_y H / h_{px}$ | known-size distance | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\beta = \arctan((u - c_x)/f_x)$, $Z = r\cos\beta$ | depth from a range sensor at the box bearing | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\frac{dZ}{dv} = -\frac{h f_y}{(v - c_y)^2} = -\frac{Z^2}{h f_y}$ | floor-plane error per pixel | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $Z = h / (\cos\theta \frac{v - c_y}{f_y} + \sin\theta)$ | floor-plane distance with camera pitch | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\frac{\sigma_Z}{Z} = \sqrt{\left(\frac{\sigma_H}{H}\right)^2 + \left(\frac{\sigma_h}{h_{px}}\right)^2}$ | known-size relative error | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $J = \begin{bmatrix} Z/f_x & 0 & (u - c_x)/f_x \\ 0 & Z/f_y & (v - c_y)/f_y \\ 0 & 0 & 1\end{bmatrix}$, $\Sigma_p = J \, \mathrm{diag}(\sigma_u^2, \sigma_v^2, \sigma_Z^2) \, J^{\mathsf T}$ | covariance of a back-projected point | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $d^2 = (p_1 - p_2)^{\mathsf T}(P + R)^{-1}(p_1 - p_2) < \chi^2_{3, \alpha}$ | 3D association gate (7.815 at 95%, 11.345 at 99%) | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $p' = Rp + t$, $\Sigma' = R\,\Sigma\,R^{\mathsf T}$ | transforming a point and its covariance | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |

## Robotic arms and manipulation (14–15, labs)

### Arm geometry and workspace
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $r_{\max} = d_{\text{shoulder}} + L_1 + L_2 + L_3$ | maximum reach: base-to-shoulder distance plus all link lengths | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\lvert l_1 - l_2 \rvert \le r \le l_1 + l_2$ | reachability of a planar 2-link arm (outside: too short; inside the inner circle: cannot fold) | [14.01](../14-robotic-arm/14.01-arm-anatomy.md), [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\text{approach azimuth} = \text{atan2}(a_y, a_x) = -q_1$ | a 5-DOF arm can only approach from the compass direction its base points | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |

### Servos and calibration
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta_{\text{deg}} = (\text{raw} - \text{mid}) \cdot \frac{360}{4095}$ | LeRobot's raw-count to degrees conversion | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $\text{mid} = \frac{\text{range\_{min}} + \text{range\_{max}}}{2}$ | centre of a joint's calibrated range | [14.02](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| $1\ \text{count} = \frac{360°}{4096} = 0.0879°$ | resolution of a 12-bit servo encoder | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\texttt{Goal\_Velocity} = \frac{v\ [\text{deg/s}]}{0.0879}$ | speed register from a speed in deg/s (never 0, which means "no limit") | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\texttt{Acceleration} = \frac{a\ [\text{deg/s}^2]}{0.0879 \cdot 100}$ | acceleration register (1..254; 0 means maximum) | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\texttt{Torque\_Limit} = 10 \cdot p\ [\%]$ | torque register from a percentage of stall | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\tau = p \cdot \tau_{\text{stall}}$ | physical torque allowed by a torque limit | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $\text{value} = -(\text{raw} - 2^{10})$ if bit 10 set, else $\text{raw}$ | Feetech sign-magnitude decoding (1036 at bit 10 is −12) | [14.03](../14-robotic-arm/14.03-controlling-servos.md) |
| $q_{\text{urdf}} = s \cdot \frac{\pi}{180}\left(d_{\text{lerobot}} - o\right), \quad s \in \{+1, -1\}$ | per-joint map from LeRobot degrees to URDF radians | [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md) |

### Forward kinematics and transforms
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T_{a,c} = T_{a,b} \, T_{b,c}$ | composition of transforms: inner indices cancel | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{\text{base},\text{tool}}(q) = T_{\text{base},1}(q_1)\, T_{1,2}(q_2)\, \cdots \, T_{n-1,n}(q_n)\, T_{n,\text{tool}}$ | forward kinematics as a chain (a left fold over the joints) | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $x = l_1\cos q_1 + l_2\cos(q_1 + q_2), \quad y = l_1\sin q_1 + l_2\sin(q_1+q_2)$ | planar 2-link forward kinematics | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\phi = q_1 + q_2$ | tool direction of the planar 2-link arm | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T(q) = \prod_{i} \text{Rot}_z(q_i)\, \text{Trans}_x(l_i)$ | planar chain FK as a product of rotations and translations | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{\text{parent},\text{child}}(q) = T(\texttt{xyz}, \texttt{rpy}) \cdot \text{Rot}(\texttt{axis}, q)$ | URDF joint transform: fixed origin first, then the joint rotation | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $R = R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})$ | URDF/ROS fixed-axis `rpy` convention | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{\text{base},\text{tool}}(q) = \left[\prod_{i=1}^{n} T_i^{\text{origin}} \, \text{Rot}(\hat{a}_i, q_i)\right] T^{\text{tool}}$ | full URDF forward kinematics | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\text{Rot}(\hat a, \theta) = I + \sin\theta\, [\hat a] + (1 - \cos\theta)\, [\hat a]^2$ | Rodrigues' formula: rotation about an axis | [14.04](../14-robotic-arm/14.04-forward-kinematics.md), [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $[\hat a] = \begin{bmatrix} 0 & -a_z & a_y \\ a_z & 0 & -a_x \\ -a_y & a_x & 0 \end{bmatrix}$ | skew-symmetric (cross-product) matrix | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{i-1,i} = \text{Rot}_z(\theta_i)\,\text{Trans}_z(d_i)\,\text{Trans}_x(a_i)\,\text{Rot}_x(\alpha_i)$ | Denavit–Hartenberg joint transform (four numbers per joint) | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $\mathcal{S}_i = (\hat\omega_i, -\hat\omega_i \times p_i)$ | screw axis of a revolute joint | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| $T_{\text{base},\text{tool}}(q) = e^{[\mathcal{S}_1]q_1} e^{[\mathcal{S}_2]q_2} \cdots e^{[\mathcal{S}_n]q_n} M$ | product of exponentials (Lynch & Park) forward kinematics | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |

### Inverse kinematics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $r^2 = x^2 + y^2$ | squared distance from shoulder to target | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\cos q_2 = \frac{r^2 - l_1^2 - l_2^2}{2 l_1 l_2}$ | elbow angle from the law of cosines ($\lvert\cos q_2\rvert > 1$ means unreachable) | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\sin q_2 = \pm\sqrt{1 - \cos^2 q_2}$ | the ± is the elbow-up / elbow-down choice | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $q_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}\!\left(l_2 \sin q_2,\; l_1 + l_2 \cos q_2\right)$ | shoulder angle of the 2-link IK | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $e(q) = x^* - f(q)$ | task-space error for numerical IK | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $e(q + \Delta q) \approx e(q) - J \Delta q \;\Rightarrow\; \Delta q = J^{+} e$ | Newton step (naive, explodes near singularities) | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\min \lVert J\Delta q - e\rVert^2 + \lambda^2\lVert\Delta q\rVert^2$ | regularised (damped) least-squares problem | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\Delta q = J^{T}\left(J J^{T} + \lambda^2 I\right)^{-1} e$ | damped least squares (Levenberg–Marquardt) IK step | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| accept: $\lambda \leftarrow \lambda/2$; reject: $\lambda \leftarrow 4\lambda$ | adaptive damping rule | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| $\lVert p_{\text{target}} - p_{\text{shoulder}}\rVert \le L_1 + L_2 + L_3$ | quick reach check before blaming the solver | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |

### Jacobian, singularities and statics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v_{\text{tool}} = J(q)\,\dot q$ | joint speeds to tool velocity | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\dot q = J^{-1} v$ | tool velocity to joint speeds (square, non-singular $J$ only) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $J_{:,i} = \hat z \times (p_{\text{tool}} - p_i)$, with $\hat z \times r = (-r_y, r_x)$ | planar Jacobian column from geometry | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $J_i = \begin{bmatrix} z_i \times (p_{\text{tool}} - p_i) \\ z_i \end{bmatrix} \in \mathbb{R}^6$ | geometric Jacobian column of a revolute joint (linear part, angular part) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $f \cdot v = \tau \cdot \dot q$ | power conservation (why the transpose maps forces) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\tau = J^T f$ | joint torques needed to hold a tool force (payload only, not the arm's own weight) | [14.06](../14-robotic-arm/14.06-jacobian.md), [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| $\det J = l_1 l_2 \sin q_2$ | determinant of the 2-link Jacobian | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $w = \sqrt{\det(JJ^T)}$ | Yoshikawa manipulability | [14.06](../14-robotic-arm/14.06-jacobian.md), [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $w = l_1 l_2 \lvert\sin q_2\rvert$ | manipulability of the 2-link arm (max at elbow 90°, zero straight or folded) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\min \lVert J\dot q - v\rVert^2 + \lambda^2\lVert\dot q\rVert^2$ | damped least-squares velocity problem | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\dot q = J^{T}\left(JJ^{T} + \lambda^2 I\right)^{-1} v$ | damped least-squares velocity (stays bounded at singularities) | [14.06](../14-robotic-arm/14.06-jacobian.md), [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\frac{1}{\sigma_i} \to \frac{\sigma_i}{\sigma_i^2 + \lambda^2}$ | what damping does to each singular direction (peaks at $1/(2\lambda)$) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\lambda^2 = \left(1 - (\sigma_{\min}/\epsilon)^2\right)\lambda_{\max}^2$ if $\sigma_{\min} < \epsilon$, else 0 | variable damping | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\dot q = J^{+}v + (I - J^{+}J)\dot q_0$ | redundancy resolution: second term moves joints without moving the tool | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $\lVert\dot q\rVert \le \lVert v\rVert/\sigma_{\min}$ | worst-case joint speed for a tool speed (pick $\sigma_{\min}$ threshold from the servo limit) | [14.06](../14-robotic-arm/14.06-jacobian.md) |
| $q \leftarrow q + \dot q\,dt$ | Euler integration of joint velocity (drifts open-loop) | [14.06](../14-robotic-arm/14.06-jacobian.md) |

### Trajectories
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $t_a = v_{\max}/a_{\max}$ | time to reach cruise speed | [14.07](../14-robotic-arm/14.07-trajectories.md), [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $\tfrac12 a_{\max} t_a^2 = v_{\max}^2/(2a_{\max})$ | distance covered while accelerating | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $D = v_{\max}^2/a_{\max}$ | break-even distance: shorter moves are triangles, longer ones trapezoids | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $T = \frac{D}{v_{\max}} + \frac{v_{\max}}{a_{\max}}$ if $D \ge v_{\max}^2/a_{\max}$ | trapezoid (accelerate, cruise, decelerate) minimum duration | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $T = 2\sqrt{D/a_{\max}}$, peak $v = \sqrt{D\,a_{\max}}$ | triangle profile (move too short to reach $v_{\max}$) | [14.07](../14-robotic-arm/14.07-trajectories.md), [08.11](../08-control/08.11-rotate-exactly-90.md) |
| $v = \left(a T - \sqrt{a^2T^2 - 4aD}\right)/2$ | cruise speed that stretches a trapezoid to duration $T$ (smaller root of $D = v(T - v/a)$) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q(t) = a_0 + a_1t + a_2t^2 + a_3t^3$ | cubic trajectory (four boundary conditions) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $q(t) = q_0 + 3\frac{D}{T^2}t^2 - 2\frac{D}{T^3}t^3$ | rest-to-rest cubic | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $\dot q_{\text{peak}} = \frac{3D}{2T}$ at $t = T/2$, $\ddot q_{\text{peak}} = \frac{6D}{T^2}$ at $t = 0, T$ | peaks of the rest-to-rest cubic | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $T \ge \max\left(1.5D/v_{\max},\ \sqrt{6D/a_{\max}}\right)$ | shortest legal cubic duration | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $\dot q_{\text{peak}} = 15D/(8T)$, $\ddot q_{\text{peak}} = (10/\sqrt3)D/T^2$ | peaks of the rest-to-rest quintic (zero end accelerations) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $T \ge \max\left(1.875D/v_{\max},\ \sqrt{5.7735\,D/a_{\max}}\right)$ | shortest legal quintic duration | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $\text{jerk}_{\text{measured}} \approx a_{\max}/dt$ | the trapezoid's "infinite" jerk as seen by sampling (scales as $1/dt$) | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $a_{\text{tool}} \approx J\ddot q + \dot J \dot q$ | tool acceleration from joint motion | [14.07](../14-robotic-arm/14.07-trajectories.md) |
| $R - \sqrt{R^2 - (L/2)^2}$ | sagitta: how far a joint-space arc bulges from the straight chord | [14.07](../14-robotic-arm/14.07-trajectories.md) |

### Motion planning and accuracy
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d_{\text{tool}} = f \times \text{extent} \times \lVert J\rVert_{\max}$ | worst-case tool motion between two collision checks | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| $f \le \frac{d_{\text{tool}}}{\text{extent} \times \lVert J\rVert_{\max}}$ | segment fraction needed to avoid tunnelling through thin objects | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| $\Delta q_{\text{step}} \le r / \lVert J\rVert_{\max}$ | joint step that guarantees no gap larger than clearance $r$ | [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md) |
| $\Delta p_{\text{tool}} = J\Delta q$ | tool error from a joint error | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $\Delta p_{\text{worst}} = \Delta q \sum_i\lVert J_{:,i}\rVert$, $\Delta p_{\text{RSS}} = \Delta q\sqrt{\sum_i \lVert J_{:,i}\rVert^2}$ | error budget: worst case and root-sum-square | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $\text{repeatability} = \max_i \lVert p_i - \bar p\rVert$ | scatter about your own mean | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $\text{accuracy} = \lVert\bar p - p_{\text{commanded}}\rVert$ | offset of the mean from the truth | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $\text{offset} = \overline{p_{\text{cmd}} - p_{\text{meas}}}$ | constant-offset correction | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |
| $p_{\text{corrected}} = A\,p_{\text{meas}} + b$ | affine correction fitted by least squares | [14.10](../14-robotic-arm/14.10-move-gripper-to-position.md) |

### Arm safety
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $v = r\omega$ | tip speed of the whole arm swinging about the base | [14.11](../14-robotic-arm/14.11-arm-safety.md), [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $E = \tfrac12 m v^2$ | kinetic energy at impact (scales with speed squared) | [14.11](../14-robotic-arm/14.11-arm-safety.md) |
| $F = \tau / r$ | pinch force at distance $r$ from a joint (huge near the axis) | [14.11](../14-robotic-arm/14.11-arm-safety.md) |

### Grasping physics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\lvert f_t\rvert \le \mu\, f_n$, $f_n \ge 0$ | Coulomb friction: the contact does not slip | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $F = \mu N$ | high-school friction law | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\alpha = \arctan \mu$ | friction-cone half-angle | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\mu = \tan\theta$ | measuring $\mu$ with a ramp: tilt angle where the object starts to slide | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $n\, \mu\, F \;\ge\; s\, m\, (g + a)$ | friction must carry the weight plus lift acceleration, with margin | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $F = \frac{s\, m\, (g + a)}{n\, \mu}$ | required normal force per pad | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $m_{\max} = \frac{n \mu F}{s(g+a)}$ | heaviest object a given squeeze can hold | [15.01](../15-manipulation/15.01-physics-of-grasping.md), [15.02](../15-manipulation/15.02-grippers.md) |
| $\sqrt{g^2 + a^2}$ in place of $g + a$ | quick estimate for a horizontal acceleration | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\tau = m g d$ | gravity torque when grasping off the centre of mass | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\tau_{\max} = \tfrac{2}{3}\, \mu\, F\, r$ | torsional friction a circular pad patch resists (per pad) | [15.01](../15-manipulation/15.01-physics-of-grasping.md), [15.02](../15-manipulation/15.02-grippers.md) |
| $w = (f_x, f_y, \tau)$, $\tau = (p - c) \times f$ | planar contact wrench | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $(f_x, f_y, \tau/L)$ | unit wrench made commensurate with a characteristic length | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\varepsilon = $ radius of the largest origin-centred ball inside the hull of the cone-edge wrenches | Ferrari–Canny grasp quality; $\varepsilon > 0$ means force closure | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |
| $\mu > \tan(\text{taper})$ | minimum friction for a two-pad grasp on tapered faces | [15.01](../15-manipulation/15.01-physics-of-grasping.md) |

### Grippers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $F = \frac{\text{torque\_limit} \times \tau_{\text{stall}} \times \eta}{\ell}$ | pad force of a servo-driven jaw | [15.02](../15-manipulation/15.02-grippers.md) |
| $1\ \text{kg·cm} = 0.0981\ \text{N·m}$ | datasheet torque unit conversion | [15.02](../15-manipulation/15.02-grippers.md) |
| $F\ [\text{N}] = \text{grams} \times 0.00981$ | kitchen-scale reading to force | [15.02](../15-manipulation/15.02-grippers.md) |
| $F = \Delta p \times A \times \eta, \quad A = \pi \left(\tfrac{d}{2}\right)^2$ | suction-cup holding force (scales as $d^2$; $\Delta p$ at most ~101 kPa) | [15.02](../15-manipulation/15.02-grippers.md) |
| $F_{\text{shear}} = \mu_{\text{lip}} F$ | sideways hold of a suction cup (about half of pull-off) | [15.02](../15-manipulation/15.02-grippers.md) |

### Hand–eye calibration
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $T_{\text{base,gripper}}^{(i)} \; X \; T_{\text{cam,target}}^{(i)} = T_{\text{base,target}}$ | the board did not move: the same pose for every arm pose | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $A X = X B$ | classic hand–eye equation | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $A_{ij} = g_j^{-1} g_i, \quad B_{ij} = c_j c_i^{-1}$ | motion pairs, eye-in-hand | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $A_{ij} = g_j g_i^{-1}$ (same $B_{ij}$) | motion pairs, eye-to-hand | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $R_A = R_X R_B R_X^{-1}$ | the two motions are conjugate rotations | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $\lVert\log R_{A}\rVert = \lVert\log R_{B}\rVert$ | free sanity check per pair (drop pairs that disagree) | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $\alpha_i = R_X \beta_i$ | rotation constraint (a Procrustes problem) | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $M = \sum_i \beta_i \alpha_i^{T}, \quad R_X = (M^{T} M)^{-1/2} M^{T}$ | Park–Martin closed-form rotation (re-orthonormalise with an SVD) | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $R_{A_i} t_X + t_{A_i} = R_X t_{B_i} + t_X$ | translation block of $A_i X = X B_i$ | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $(R_{A_i} - I)\, t_X = R_X t_{B_i} - t_{A_i}$ | linear least-squares system for the translation (singular along the rotation axis) | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $\text{spread of } g_i X c_i$ | consistency residual: field check of a calibration | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |
| $\tfrac12(R + I)$ | symmetric part used to read the axis of a 180° rotation | [15.03](../15-manipulation/15.03-hand-eye-calibration.md) |

### Object pose from depth
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $X = \frac{(u - c_x) Z}{f_x}, \quad Y = \frac{(v - c_y) Z}{f_y}, \quad Z = Z$ | inverted pinhole model: pixel plus depth to 3D point | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $p_{\text{base}} = T_{\text{base,optical}} \; p_{\text{optical}}$ | camera point to base frame | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $T_{\text{base,optical}} = T_{\text{base,gripper}} \cdot X_{\text{hand-eye}} \cdot T_{\text{cam,optical}}$ | camera pose chain for a wrist camera | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $n \cdot p + d = 0$, $\lVert n\rVert = 1$ | plane equation (RANSAC table fit) | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\sigma_Z < \text{threshold} < \text{height of the shortest object}$ | plane-threshold bounds | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\sigma_Z = k Z^2$ | depth noise grows with distance squared | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\text{noise gaps in one surface} < \text{voxel} < \text{smallest gap between objects}$ | voxel-size bounds for clustering | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $C = \frac{1}{N}\sum_i (p_i - \bar p)(p_i - \bar p)^{T}$ | footprint covariance | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $C v_k = \lambda_k v_k$ | PCA: eigenvectors give the object's axes, eigenvalues its spread | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\lambda_1/\lambda_2 = (\ell_1/\ell_2)^2$ | variance scales as length squared | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\theta \in (-90°, 90°]$ | yaw wrapped as an axis (PCA cannot tell $\theta$ from $\theta + 180°$) | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |
| $\sigma_{\bar p} \propto 1/\sqrt{N}$ | averaging $N$ points beats down random noise | [15.04](../15-manipulation/15.04-object-pose-estimation.md) |

### Grasp planning and approach
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{score} = \frac{\sum_k w_k\, t_k}{\sum_k w_k}$, $w = (0.30, 0.30, 0.20, 0.20)$ | grasp score: weighted mean of normalised terms | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $t = \operatorname{clip}\left(\frac{\text{stroke} - \text{width}}{\text{stroke}}, 0, 1\right)$ | width-margin term | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $t = \operatorname{clip}(\varepsilon / 0.3, 0, 1)$ | stability term | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $t = \operatorname{clip}(1 - \text{offset}/0.02, 0, 1)$ | centred term | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $t = \operatorname{clip}(\text{dist} / 16\,\text{mm}, 0, 1)$ | clearance term | [15.05](../15-manipulation/15.05-grasp-planning.md) |
| $\text{slack} = (\text{stroke} - \text{width}) / 2$ | lateral error the jaw tolerates before one pad hits first | [15.07](../15-manipulation/15.07-detect-and-approach.md) |
| $\Delta\text{width} = \text{width}\,(1/\cos\theta - 1)$ | width growth when the jaw closes $\theta$ off the short axis | [15.07](../15-manipulation/15.07-detect-and-approach.md) |
| $\sigma_{\text{total}} = \sqrt{\sum \sigma_i^2}$ | independent errors add in quadrature | [15.07](../15-manipulation/15.07-detect-and-approach.md) |
| $(\text{stroke} - w)/2 \approx 1.5\sigma \;\Rightarrow\; w \approx \text{stroke} - 3\sigma$ | where pick success falls off a cliff | [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md) |
| $P(\lvert\text{error}\rvert < \text{slack})$ | pick success probability for a Gaussian reach error | [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md) |

### Visual servoing
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x = \frac{u - c_x}{f_x}, \quad y = \frac{v - c_y}{f_y}$ | normalised image coordinates | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $(x, y) = (X/Z, Y/Z)$ | perspective projection in normalised coordinates | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L_p = \begin{bmatrix} -\frac{1}{Z} & 0 & \frac{x}{Z} & xy & -(1 + x^2) & y \\ 0 & -\frac{1}{Z} & \frac{y}{Z} & 1 + y^2 & -xy & -x \end{bmatrix}$ | interaction matrix of one point feature | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\dot s = L v$ | feature velocity from camera twist | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $v_c = -\lambda\, L^{+}(s - s^{*})$ | IBVS control law (note the minus) | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L^{+} = L^{T}(L L^{T} + \mu^2 I)^{-1}$ | damped pseudo-inverse of the stacked $2N \times 6$ matrix | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $T_{err} = (T^{*}_{cam,obj})^{-1} T_{cam,obj}$ | PBVS pose error | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $v_c = +\lambda\,(t_{err},\; \theta u_{err})$ | PBVS control law (note the plus) | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\omega_{base} = R_{base,cam}\, \omega_{cam}$ | rotate the angular velocity into the base frame | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $v_{tool} = R_{base,cam} v_{cam} + \omega_{base} \times (p_{tool} - p_{cam})$ | move the twist's reference point from the camera to the tool | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $a = \sqrt{\mu_{20} + \mu_{02}}$ | apparent-size moment feature | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\alpha = \frac12 \arctan\frac{2\mu_{11}}{\mu_{20} - \mu_{02}}$ | orientation moment feature | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $L_{moment} = \frac{\partial m}{\partial s} L_{points}$ | interaction matrix of moment features (chain rule) | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\lambda \times T_{delay} \lesssim 1$ | stability rule of thumb for gain versus loop delay | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| $\Delta X = \Delta u \cdot Z / f$ | metric size of one pixel of jitter at depth $Z$ | [15.06](../15-manipulation/15.06-visual-servoing.md) |

### Mobile manipulation and stability
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathbf{c} = \frac{m_b \mathbf{c}_b + \sum_i m_i \mathbf{c}_i(q) + m_p \mathbf{p}_{tool}(q)}{m_b + \sum_i m_i + m_p}$ | combined centre of mass of base, arm and payload | [15.10](../15-manipulation/15.10-mobile-manipulation.md), [20.02](../20-final-robot/20.02-hardware-integration.md) |
| margin = distance from $\mathbf{c}$ to the nearest support-polygon edge | static stability margin (negative means it tips) | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $a_{tip} = g\,x_c/h$ | acceleration that tips the robot about its support edge | [15.10](../15-manipulation/15.10-mobile-manipulation.md), [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $\text{use the arm} \iff w(q) \ge 0.010 \wedge \text{margin} \ge 30\ \text{mm}$ | decide between reaching and re-parking the base | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |
| $\text{lateral offset} = d \sin(\text{yaw error})$ | sideways miss caused by a heading error at distance $d$ | [15.10](../15-manipulation/15.10-mobile-manipulation.md) |

### Wheels, encoders and drive (labs)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{m/tick} = \frac{2\pi r}{\text{ticks\_per\_rev}}$ | distance per encoder tick (wheel circumference ÷ ticks) | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s = \theta \cdot b/2$ | arc each wheel travels when turning in place by $\theta$ | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s_{L,R} = \theta\,(R \mp b/2)$ | wheel arcs for a turn of radius $R$ | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\omega_{L,R} = (v \mp \omega L/2)/r$ | differential-drive inverse kinematics: body twist to wheel speeds | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md), [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $\omega = v / r$ | wheel speed for a ground speed | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $\theta_{\text{err}} = \text{bias} \times t$ | heading drift of an uncorrected gyro | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| $\text{measured} = \text{gain} \cdot \text{true} + \text{offset}$ | sensor calibration line (invert it to correct readings) | [07.01](../07-sensors/07.01-sensor-fundamentals.md) |
| $t_k = t_0 + k \cdot \text{period}$ | fixed-rate loop deadlines (no drift) | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| $\text{wrap}(a) = ((a + \pi) \bmod 2\pi) - \pi$ | wrap an angle to $(-\pi, \pi]$ | [08.11](../08-control/08.11-rotate-exactly-90.md) |

### Motor control (labs)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $u = k_p (r - \omega)$ | proportional control | [08.04](../08-control/08.04-proportional-control.md) |
| $\omega = \text{slope}\,(u - \text{deadband})$ | motor model above the deadband | [08.04](../08-control/08.04-proportional-control.md) |
| $\omega_{ss} = \frac{\text{slope}\,(k_p r - \text{deadband})}{1 + \text{slope}\, k_p}$ | where a P-only loop settles (always below the setpoint) | [08.04](../08-control/08.04-proportional-control.md) |
| $\omega_{\max} = \text{slope}\,(1 - \text{deadband})$ | saturation speed at duty 1.0 | [08.04](../08-control/08.04-proportional-control.md) |
| $u = k_p e + I, \quad I \leftarrow I + k_i e\,dt$ | PI control (integral stored already multiplied by $k_i$) | [08.05](../08-control/08.05-integral-control.md) |
| $d = -k_d \cdot \dot y_{\text{filtered}}$ | derivative on measurement | [08.06](../08-control/08.06-derivative-control.md) |
| $\alpha = \frac{dt}{\tau + dt}$ | first-order low-pass coefficient of the derivative filter | [08.06](../08-control/08.06-derivative-control.md) |
| $\tau s^2 + (1 + K k_p) s + K k_i$ | closed-loop characteristic polynomial of PI + $K/(\tau s + 1)$ | [08.07](../08-control/08.07-pid-mathematics.md) |
| $k_p = \frac{2\zeta\omega_n\tau - 1}{K}, \quad k_i = \frac{\omega_n^2 \tau}{K}$ | PI pole placement | [08.07](../08-control/08.07-pid-mathematics.md) |
| $k_i / k_p = 1/\tau$, $k_p = \frac{\tau}{K\tau_{cl}}$ | lambda tuning: cancel the motor pole, choose the closed-loop time constant | [08.07](../08-control/08.07-pid-mathematics.md) |
| $t_r = \tau \ln 9$, $t_{s,2\%} = \tau \ln 50$ | rise and 2 % settling time of a first-order response | [08.07](../08-control/08.07-pid-mathematics.md) |
| $t_p = \pi / \omega_d$ | peak time of a second-order response | [08.07](../08-control/08.07-pid-mathematics.md) |
| $n = \frac{y - y_{\text{start}}}{\text{setpoint} - y_{\text{start}}}$ | normalised step response (works for downward steps) | [08.07](../08-control/08.07-pid-mathematics.md) |
| $\omega_{\text{top}} = \omega_{\max} \cdot V_{\text{batt}} / V_{\text{nom}}$ | top speed scales with battery voltage | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $u_{ff} = \operatorname{sign}(\omega)\left(\text{deadband} + \frac{\lvert\omega\rvert}{\omega_{\text{top}}}(1 - \text{deadband})\right)$ | feedforward duty (exactly 0 for a 0 setpoint) | [08.09](../08-control/08.09-feedforward-exact-speed.md) |

### Estimation and mapping (labs)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $K = \frac{P^-}{P^- + R}$ | 1D Kalman gain | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $R = \sigma^2$ | measurement variance from a sensor's standard deviation | [10.04](../10-localization/10.04-kalman-filter-1d.md) |
| $K = P H^T S^{-1}$ | multivariate Kalman gain | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| $\theta_m = \theta + (d_R - d_L)/(2b)$ | midpoint heading of the odometry motion model | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $M = \operatorname{diag}(k^2\lvert d_L\rvert, k^2\lvert d_R\rvert)$ | wheel-travel noise grows with distance | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| $\text{ESS} = 1 / \sum w^2$ | effective sample size of a particle set | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $\bar\theta = \operatorname{atan2}(\sum w \sin\theta, \sum w \cos\theta)$ | circular mean of particle headings | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| $H = (src - \mu_s)^T(dst - \mu_d)$ | cross-covariance used by Kabsch/ICP to find the rotation | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| $\max(dr, dc) + (\sqrt2 - 1)\min(dr, dc)$ | octile distance on an 8-connected grid | [12.02](../12-navigation/12.02-grid-path-planning.md) |
| $\text{cost} = 252 \cdot e^{-k(d - r)}$ | Nav2 inflation cost outside the inscribed radius | [12.04](../12-navigation/12.04-costmaps.md) |
| $\kappa = \frac{2 y_r}{x_r^2 + y_r^2}, \quad \omega = v\kappa$ | pure-pursuit curvature and turn rate | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| $Z = f_y H / h_{px}$ | depth from an object of known height $H$ | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $Z = h f_y / (v - c_y)$ | depth from the ground plane for a level camera at height $h$ | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |
| $\partial X / \partial Z = (u - c_x)/f_x$ | why an off-centre object inherits depth error sideways | [13.16](../13-computer-vision/13.16-detection-to-map-position.md) |

### Learning and evaluation (labs)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $p \pm z\sqrt{p(1-p)/n}$ | textbook success-rate interval (breaks near 0 and 1; use Wilson) | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $y \log p + (1-y)\log(1-p)$ | log-likelihood maximised when fitting a temperature | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $r + \gamma (1 - \text{terminated}) \max_{a'} Q(s', a')$ | TD target (no bootstrap from a terminal state) | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md), [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $G_t = \sum_{k \ge t} \gamma^{k-t} r_k$ | return-to-go | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\nabla_{\text{logits}} \log \pi(a) = \text{onehot}(a) - \pi$ | softmax policy score | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $r = 10 \times \text{progress} - 0.05$ | go-to-goal step reward (±10 on success / collision) | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |

### Wiring, timing and stopping (labs)
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $R_{\text{loop}} = 2 \cdot \text{length} \cdot \Omega/\text{m}$ | wire resistance counts both conductors | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $1.25\, I_{\text{cont}} < I_{\text{fuse}} \le I_{\text{ampacity}}$ | fuse sizing: no nuisance trips, and the fuse is weaker than the wire | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $d_{\text{stop}} = v\, t_{dead} + \frac{v^2}{2a}$ | stopping distance: dead-time travel plus braking | [20.04](../20-final-robot/20.04-safety-case.md), [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $v = -a t + \sqrt{a^2 t^2 + 2 a m}$ | maximum safe speed for a stopping margin $m$ | [20.04](../20-final-robot/20.04-safety-case.md) |
| $d = v \times t_{\text{latency}}$ | distance travelled during a latency | [19.11](../19-llm-robot-agents/19.11-voice-interface.md), [FC.08](../optional-foundations/cpp-and-embedded/FC.08-micro-ros.md) |
| $U = \sum \text{WCET}/\text{period}$ | CPU utilisation of periodic tasks | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $U \le n(2^{1/n} - 1)$ | Liu & Layland rate-monotonic schedulability bound | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $\Delta = ((\text{new} - \text{old} + 2^{29}) \bmod 2^{30}) - 2^{29}$ | signed difference on a counter that wraps at $2^{30}$ | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |

## Machine learning, RL, embodied AI and LLM agents (16–19, FML)

### Learning from data
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\hat y = f_\theta(x)$ | a model's prediction | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\theta^* = \arg\min_\theta \; \frac{1}{N}\sum_{i=1}^{N} L\big(f_\theta(x_i),\, y_i\big)$ | training = minimize the average loss | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md) |
| $\hat y = w_1 d + w_2 V + b$ | linear model with two features | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\hat y = w^\top \phi(x)$ | model linear in parameters, any features | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\hat y = Xw$ | linear model over a batch | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $w^* = (X^\top X)^{-1} X^\top y$ | normal equations (least squares closed form) | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $s = w^\top x + b > 0$ | linear classifier decision rule | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\text{accuracy} = \frac{TP+TN}{N}$ | fraction of all predictions that are right | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\text{precision} = \frac{TP}{TP+FP}$ | fraction of positive calls that are right | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\text{recall} = \frac{TP}{TP+FN}$ | fraction of real positives found | [FML.02](../optional-foundations/machine-learning/FML.02-supervised-learning.md) |
| $\mathbb{E}\big[(y - \hat y)^2\big] = \text{bias}^2 + \text{variance} + \sigma^2$ | bias–variance decomposition of the error | [FML.03](../optional-foundations/machine-learning/FML.03-train-validation-test.md) |

### Loss functions
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $L = (\hat y - y)^2$ | squared error (MSE, L2) | [FML.01](../optional-foundations/machine-learning/FML.01-what-is-ml.md), [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\lvert \hat y - y \rvert$ | absolute error (MAE, L1) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\tfrac12 e^2$ if $\lvert e\rvert\le\delta$, else $\delta(\lvert e\rvert - \tfrac12\delta)$ | Huber loss (robust to outliers) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md), [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $-[y\ln p + (1-y)\ln(1-p)]$ | binary cross-entropy | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $p_k = \frac{e^{z_k}}{\sum_j e^{z_j}}, \quad L = -\ln p_{\text{true}}$ | softmax and softmax cross-entropy | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $p = 1/(1 + e^{-z})$ | sigmoid (two-class softmax) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $L = z + \ln(1 + e^{-z})$ | stable BCE from the logit (label 0) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $-\ln p(y\mid x) = \frac{(y - \hat y)^2}{2\sigma^2} + \tfrac12\ln(2\pi\sigma^2)$ | Gaussian negative log-likelihood (why MSE) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| $\frac{d}{dc}\frac1N\sum(c - y_i)^2 = 0 \Rightarrow c = \bar y$ | MSE's best constant is the mean (MAE's is the median) | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |

### Gradient descent and optimizers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta \leftarrow \theta - \eta\, \nabla_\theta L(\theta)$ | gradient descent step | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\nabla_w L = \frac{2}{N} X^\top (Xw - y)$ | gradient of MSE for a linear model | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $H = \frac2N X^\top X$ | Hessian of the linear-model MSE | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\eta < \frac{2}{\lambda_{\max}(H)}$ | largest stable learning rate | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\lvert 1 - \eta\lambda \rvert$ | per-step error contraction along an eigen-direction | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $v \leftarrow \beta v + \nabla L, \qquad \theta \leftarrow \theta - \eta v$ | momentum | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $m \leftarrow \beta_1 m + (1-\beta_1) g,\quad v \leftarrow \beta_2 v + (1-\beta_2) g^2,\quad \theta \leftarrow \theta - \eta \frac{\hat m}{\sqrt{\hat v} + \epsilon}$ | Adam update | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| $\nabla_w L = \frac1N X^\top(\sigma(Xw) - y)$ | gradient of logistic regression with BCE | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |

### Neural networks and backpropagation
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $a = \phi(w^\top x + b)$ | one neuron | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\mathrm{ReLU}(z) = \max(0, z)$ | ReLU activation | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $h = \phi(W x + b)$ | one dense layer | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $z = W_1 x + b_1,\quad h = \mathrm{ReLU}(z),\quad \hat y = W_2 h + b_2$ | forward pass of a 1-hidden-layer MLP | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $nH + H + H + 1$ | parameter count of that MLP | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $W_2(W_1x + b_1) + b_2 = (W_2W_1)x + (W_2b_1 + b_2)$ | layers without activation collapse to one | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| std $=\sqrt{2/n_{\text{in}}}$ | He initialization for ReLU | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\frac{\partial L}{\partial \theta_i} \approx \frac{L(\theta_i + \epsilon) - L(\theta_i - \epsilon)}{2\epsilon}$ | numerical gradient (central difference) | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| $\frac{\partial L}{\partial w} = \frac{\partial L}{\partial a}\cdot\frac{\partial a}{\partial b}\cdot\frac{\partial b}{\partial w}$ | chain rule | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $Z_1 = XW_1^\top + b_1,\quad H_1 = \mathrm{ReLU}(Z_1),\quad \hat y = H_1W_2^\top + b_2$ | batch forward pass | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\frac{\partial L}{\partial \hat y} = \tfrac2N(\hat y - y)$ | MSE gradient at the output | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\frac{\partial L}{\partial W_2} = \Big(\frac{\partial L}{\partial \hat y}\Big)^{\!\top} H_1,\quad \frac{\partial L}{\partial b_2} = \sum \frac{\partial L}{\partial \hat y}$ | output-layer gradients | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\frac{\partial L}{\partial H_1} = \frac{\partial L}{\partial \hat y}\,W_2,\quad \frac{\partial L}{\partial Z_1} = \frac{\partial L}{\partial H_1}\odot \mathbb{1}[Z_1 > 0]$ | backprop through a layer and a ReLU | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\frac{\partial L}{\partial W_1} = \Big(\frac{\partial L}{\partial Z_1}\Big)^{\!\top} X,\quad \frac{\partial L}{\partial b_1} = \sum_{\text{rows}} \frac{\partial L}{\partial Z_1}$ | first-layer gradients | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |
| $\frac{\lVert g_a - g_n\rVert}{\lVert g_a\rVert + \lVert g_n\rVert}$ | relative error of a gradient check | [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md) |

### Convolutional networks
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $Y_{o,i,j} = b_o + \sum_{c}\sum_{u=0}^{k-1}\sum_{v=0}^{k-1} W_{o,c,u,v}\; X_{c,\,i\cdot s+u-p,\; j\cdot s+v-p}$ | convolution (cross-correlation) layer | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $W_{\text{out}} = \left\lfloor \frac{W_{\text{in}} + 2p - k}{s} \right\rfloor + 1$ | output size of a conv layer | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $C_{\text{out}}\,(C_{\text{in}}\,k^2 + 1)$ | parameter count of a conv layer | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| $r_{\text{out}} = r_{\text{in}} + (k - 1)\,j_{\text{in}},\quad j_{\text{out}} = j_{\text{in}}\cdot s$ | receptive field growth per layer | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |

### Embeddings and attention
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\cos(a, b) = \frac{a \cdot b}{\lVert a\rVert\,\lVert b\rVert}$ | cosine similarity | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $L = -\frac1N\sum_{i=1}^{N} \ln \frac{\exp(u_i\cdot v_i/\tau)}{\sum_{j=1}^{N}\exp(u_i\cdot v_j/\tau)}$ | InfoNCE contrastive loss (also CLIP) | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $\partial L/\partial z = (g - u\,(u\cdot g))/\lVert z\rVert$ | gradient through normalization $u = z/\lVert z\rVert$ | [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md) |
| $\mathrm{Attention}(Q, K, V) = \mathrm{softmax}\!\left(\frac{QK^\top}{\sqrt{d_k}}\right)V$ | scaled dot-product attention | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $Q = XW_Q,\ K = XW_K,\ V = XW_V$ | self-attention projections | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $d_k = d/h$ | per-head dimension in multi-head attention | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $4(d^2 + d)$ | parameters of one attention layer | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $x \leftarrow x + \mathrm{MHA}(\mathrm{LN}(x)),\quad x \leftarrow x + \mathrm{MLP}(\mathrm{LN}(x))$ | pre-norm transformer block | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $n^2$ | attention matrix entries per head per layer (cost) | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |
| $n = (W/P)\times(H/P)$ (+1 CLS) | ViT token count for a $W \times H$ image | [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md) |

### Unsupervised learning
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $J = \sum_{i=1}^{N} \lVert x_i - \mu_{c(i)}\rVert^2$ | k-means objective (inertia) | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $X = U\Sigma V^\top$, variance of component $k = \sigma_k^2/N$ | PCA via SVD | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $\sigma_k^2/\sum_j\sigma_j^2$ | explained-variance ratio | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $z = (x - \bar x)V_k^\top,\quad \hat x = zV_k + \bar x$ | PCA compress and reconstruct | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $z = f_\phi(x),\quad \hat x = g_\theta(z),\quad L = \frac1N\sum_i \lVert x_i - g_\theta(f_\phi(x_i))\rVert^2$ | autoencoder | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| $s(x) = \lVert x - \hat x\rVert$ | anomaly score | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |

### Generative models and diffusion
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $z = \mu + \sigma\,\varepsilon,\ \varepsilon \sim \mathcal N(0, 1)$ | reparameterization trick | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $L = \lVert x - \text{dec}(z)\rVert^2 + \beta\,\mathrm{KL}\big(\mathcal N(\mu, \sigma^2)\,\Vert\,\mathcal N(0, 1)\big)$ | VAE loss | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $\mathrm{KL} = \tfrac12\left(\mu^2 + \sigma^2 - 1 - \ln\sigma^2\right)$ | KL of a Gaussian to the standard normal | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $\alpha_t = 1 - \beta_t,\quad \bar\alpha_t = \prod_{s\le t}\alpha_s$ | diffusion noise schedule | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $a_t = \sqrt{\bar\alpha_t}\,a_0 + \sqrt{1 - \bar\alpha_t}\,\varepsilon$ | DDPM forward process in one jump | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\lVert \varepsilon - \epsilon_\theta(a_t, t, o)\rVert^2$ | DDPM training loss (predict the noise) | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\hat a_0 = (a_t - \sqrt{1-\bar\alpha_t}\,\hat\varepsilon)/\sqrt{\bar\alpha_t}$ | clean action implied by a noise prediction | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $a_{t-1} = \frac{1}{\sqrt{\alpha_t}}\left(a_t - \frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\,\epsilon_\theta(a_t, t, o)\right) + \sqrt{\beta_t}\,z$ | DDPM sampling step | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a_t = (1-t)\,a_0 + t\,a_1,\quad L = \lVert (a_1 - a_0) - v_\theta(a_t, t, o)\rVert^2$ | flow matching path and loss | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $a \leftarrow a + \tfrac1K\,v_\theta(a, k/K, o)$ | flow-matching Euler sampling | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| $v(a, t) = (a_1 - a)/(1 - t)$ | ideal velocity field for a single demonstration | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |

### Perception models on the robot
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\Delta\theta = \omega L$ | bearing error from inference latency while rotating | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md), [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $q f t$ | expected false alarms in $t$ seconds | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $P(\text{no false alarm}) = (1 - q)^{f t}$ | chance of no false alarm | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $h = f_y H / Z$ | pinhole size check: object height in pixels | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $f_y = (\text{image half-size}) / \tan(\text{FOV}/2)$ | focal length in pixels from field of view | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md) |
| $3/n$ | rule of three: 95% upper bound on failure rate after 0 failures in $n$ | [16.01](../16-machine-learning/16.01-where-ml-helps-robots.md), [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md), [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $n_{\text{eff}} = n \, \frac{1 - \rho}{1 + \rho}$ | effective sample size of correlated frames | [16.02](../16-machine-learning/16.02-robot-data.md), [16.03](../16-machine-learning/16.03-fine-tuning-a-detector.md) |
| $(1024 + 1) \times (C + 1)$ and $(1024 + 1) \times 4(C + 1)$ | new classification and box weights of a Faster R-CNN head | [16.03](../16-machine-learning/16.03-fine-tuning-a-detector.md) |
| $x \approx s\,(q - z),\quad s = \frac{x_{\max} - x_{\min}}{255},\quad z = \mathrm{round}\!\left(\frac{-x_{\min}}{s}\right)$ | INT8 affine quantization | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $q = \mathrm{round}(x/s) + z$ | quantize a value | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| $s/2$ | maximum rounding error of quantization | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |

### Learned and grey-box dynamics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\tau \frac{d\omega}{dt} = k \cdot \mathrm{dz}(u) \cdot \frac{V}{V_{\text{nom}}} - \omega$ | first-order motor model with deadband and battery scaling | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\mathrm{dz}(u) = \mathrm{sign}(u)\,\max(\lvert u\rvert - d,\, 0)$ | deadzone function | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\omega_{\infty} = k \cdot \mathrm{dz}(u) \cdot \frac{V}{V_{\text{nom}}}$ | steady-state wheel speed | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $u_{\text{ff}}(\omega^*, V) = \mathrm{sign}(\omega^*)\left(d + \frac{\lvert\omega^*\rvert\,V_{\text{nom}}}{k\,V}\right)$ | inverse model: feed-forward duty | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $x_i = \mathrm{dz}(u_i)\frac{V_i}{V_{\text{nom}}},\quad k^*(d) = \frac{\sum_i x_i \omega_i}{\sum_i x_i^2}$ | least-squares gain for a fixed deadband | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $d^* = \arg\min_d \sum_i (k^*(d)\,x_i - \omega_i)^2$ | deadband by grid search | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $1 - e^{-t/\tau}$ | fraction of final speed reached after $t$ | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |
| $\mathrm{SE}(k) = \frac{\sigma}{\sqrt{n\,\mathrm{Var}(x)}}$ | standard error of a fitted slope | [16.05](../16-machine-learning/16.05-learning-dynamics.md) |

### Evaluation and statistics
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\mathrm{ECE} = \sum_{b} \frac{n_b}{N}\,\bigl\lvert\mathrm{acc}(b) - \mathrm{conf}(b)\bigr\rvert$ | expected calibration error | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\hat{p} = \sigma\!\left(\frac{\mathrm{logit}(p)}{T}\right)$ | temperature scaling | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\mathrm{PSI} = \sum_{b} (c_b - r_b)\,\ln\frac{c_b}{r_b}$ | population stability index (distribution shift) | [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md) |
| $\hat p = k/n$ | observed success rate | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $\hat p \pm 1.96\sqrt{\hat p(1-\hat p)/n}$ | Wald interval (unreliable for small $n$) | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\text{center} = \frac{\hat p + \frac{z^2}{2n}}{1 + \frac{z^2}{n}},\quad \text{half-width} = \frac{z}{1+\frac{z^2}{n}}\sqrt{\frac{\hat p(1-\hat p)}{n} + \frac{z^2}{4n^2}}$ | Wilson score interval | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md), [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md), [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| width $\propto 1/\sqrt{n}$ | interval width vs number of trials | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $\Lambda \leftarrow \Lambda + \ln(p_1/p_0)$ on success, $+\ln\big((1-p_1)/(1-p_0)\big)$ on failure | SPRT log-likelihood update | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| accept $H_1$ if $\Lambda \ge \ln\frac{1-\beta}{\alpha}$, accept $H_0$ if $\Lambda \le \ln\frac{\beta}{1-\alpha}$ | SPRT stopping bounds | [18.10](../18-embodied-ai/18.10-evaluating-learned-policies.md) |
| $n \approx (z_{\alpha/2}+z_\beta)^2\,\bar p(1-\bar p)\,2/\Delta^2$ | trials per arm to tell two success rates apart | [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |

### Reinforcement learning: MDPs and values
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $(\mathcal{S}, \mathcal{A}, P, R, \gamma)$ | Markov decision process | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $P(s_{t+1} \mid s_t, a_t) = P(s_{t+1} \mid s_t, a_t, s_{t-1}, a_{t-1}, \dots, s_0)$ | Markov property | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| $G_t = \sum_{k=0}^{\infty} \gamma^k\, r_{t+k+1}$ | discounted return | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $G_t = r_{t+1} + \gamma\, G_{t+1}$ | return recursion | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $1/(1-\gamma)$ | effective horizon in steps | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V^\pi(s) = \mathbb{E}_\pi[\,G_t \mid s_t = s\,],\quad Q^\pi(s, a) = \mathbb{E}_\pi[\,G_t \mid s_t = s, a_t = a\,]$ | state and action value | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V^\pi(s) = \sum_a \pi(a \mid s) \sum_{s', r} P(s', r \mid s, a)\,\big[r + \gamma V^\pi(s')\big]$ | Bellman expectation equation | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V = (I - \gamma P_\pi)^{-1} r_\pi$ | exact policy value of a finite MDP | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $V^\pi(s) = \sum_a \pi(a\mid s)\,Q^\pi(s,a)$ | value from action values | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $Q^*(s, a) = \sum_{s', r} P(s', r \mid s, a)\,\big[r + \gamma \max_{a'} Q^*(s', a')\big]$ | Bellman optimality equation | [17.02](../17-reinforcement-learning/17.02-value-exploration.md), [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\pi^*(s) = \arg\max_a Q^*(s, a)$ | greedy optimal policy | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $Q(a) \leftarrow Q(a) + \frac{1}{N(a)}\,\big[r - Q(a)\big]$ | incremental mean | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $a = \arg\max_a \big[Q(a) + c\sqrt{\ln t / N(a)}\big]$ | UCB action selection | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |
| $\varepsilon_t = \max(0.01,\ 1/\sqrt{t+1})$ | decaying exploration schedule | [17.02](../17-reinforcement-learning/17.02-value-exploration.md) |

### Reinforcement learning: Q-learning and DQN
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $Q(s, a) \leftarrow Q(s, a) + \alpha\,\big[r + \gamma \max_{a'} Q(s', a') - Q(s, a)\big]$ | Q-learning update (target is $r$ if $s'$ is terminal) | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $\delta = r + \gamma \max_{a'} Q(s', a') - Q(s, a)$ | TD error | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $r + \gamma\, Q(s', a_{\text{next}})$ | SARSA target | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| $y_i = r_i + \gamma\,(1 - d_i)\,\max_{a'} Q_{\theta^-}(s'_i, a')$ | DQN target with terminal mask | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $L(\theta) = \frac{1}{\lvert B\rvert}\sum_{i \in B} \ell\big(y_i - Q_\theta(s_i, a_i)\big)$ | DQN loss | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $y_i = r_i + \gamma\,(1-d_i)\,Q_{\theta^-}\!\big(s'_i,\ \arg\max_{a'} Q_{\theta}(s'_i, a')\big)$ | Double DQN target | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| $\theta^- \leftarrow \tau\theta + (1-\tau)\theta^-$ | soft target-network update | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md), [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\mathbb{E}[\max_a \hat{Q}] \ge \max_a \mathbb{E}[\hat{Q}]$ | overestimation bias of the max | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |

### Reinforcement learning: policy gradients and actor-critic
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $J(\theta) = \mathbb{E}[\,R\,]$ | policy objective | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\nabla \pi = \pi \nabla \log \pi$ | log-derivative trick | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\nabla_\theta \mathbb{E}_{a \sim \pi_\theta}[R(a)] = \mathbb{E}_{a \sim \pi_\theta}\!\big[\,R(a)\, \nabla_\theta \log \pi_\theta(a)\,\big]$ | score-function gradient | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\nabla_\theta \log \pi_\theta(a) = \mathbf{e}_a - \pi_\theta$ | softmax score function | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\nabla_\theta J(\theta) = \mathbb{E}_\tau\!\left[\sum_{t=0}^{T-1} \nabla_\theta \log \pi_\theta(a_t \mid s_t)\,\big(G_t - b(s_t)\big)\right]$ | policy gradient theorem (REINFORCE with baseline) | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $G_t = \sum_{k=t}^{T-1} \gamma^{\,k-t} r_k$ | return-to-go | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\mathbb{E}_{a}\big[\nabla_\theta \log \pi_\theta(a \mid s)\, b(s)\big] = 0$ | a baseline adds no bias | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\theta \leftarrow \theta + \alpha \hat{g}$ | gradient ascent on the return | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\log \pi_\theta(a \mid s) = -\frac{(a - \mu)^2}{2\sigma^2} - \log \sigma - \tfrac{1}{2}\log 2\pi$ | Gaussian policy log-probability | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $\frac{\partial \log \pi}{\partial \mu} = \frac{a - \mu}{\sigma^2},\quad \frac{\partial \log \pi}{\partial (\log \sigma)} = \frac{(a-\mu)^2}{\sigma^2} - 1$ | Gaussian score functions | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $a \sim \mathcal{N}(k\,x,\ \sigma^2)$, reward $-\lvert x_{t+1}\rvert - 0.1\,a^2$ | learned P-controller docking policy | [17.05](../17-reinforcement-learning/17.05-policy-gradients.md) |
| $A_t = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$ | one-step advantage from a critic | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t),\quad \hat{A}^{\text{GAE}}_t = \sum_{l \ge 0} (\gamma\lambda)^l\, \delta_{t+l}$ | generalized advantage estimation | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\hat{A}_t + V(s_t)$ | critic's regression target | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $L^{\text{CLIP}}(\theta) = \mathbb{E}_t\Big[\min\big(\rho_t \hat{A}_t,\ \text{clip}(\rho_t, 1-\epsilon, 1+\epsilon)\,\hat{A}_t\big)\Big]$ | PPO clipped objective | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $\rho_t = \frac{\pi_\theta(a_t\mid s_t)}{\pi_{\theta_\text{old}}(a_t\mid s_t)}$ | PPO probability ratio | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $L = L^{\text{CLIP}} - c_v\,(V_\phi(s_t) - \hat{R}_t)^2 + c_e\,H[\pi_\theta]$ | full PPO objective | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $n_\text{epochs} \times (n_\text{envs} \cdot n_\text{steps}) / \text{batch size}$ | PPO gradient steps per rollout | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $J(\pi) = \mathbb{E}\Big[\sum_t r_t + \alpha\,\mathcal{H}\big(\pi(\cdot \mid s_t)\big)\Big]$ | SAC maximum-entropy objective | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| $Q(s, a) - \alpha \log \pi(a\mid s)$ | SAC actor objective | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |

### Reinforcement learning: environments, rewards, sim-to-real
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $(\cos e, \sin e)$ | wrap-free encoding of an angle for a network | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $r_t = 10\,(d_{t-1} - d_t) - 0.05 + 10\,[\text{success}] - 10\,[\text{collision}]$ | go-to-goal progress reward | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $\sum_t 10(d_{t-1} - d_t) = 10(d_0 - d_{\text{final}})$ | progress reward telescopes | [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md) |
| $F(s, s') = \gamma\,\Phi(s') - \Phi(s)$ | potential-based shaping (keeps optimal policy) | [17.08](../17-reinforcement-learning/17.08-reward-design.md) |
| $\Phi(s) = -10\,d(s)$, $\Phi(s) = -3\lvert e(s)\rvert$ | potentials for the progress and heading terms | [17.08](../17-reinforcement-learning/17.08-reward-design.md) |
| $\omega = (d_\text{right} - d_\text{left})/b$ | yaw rate from wheel travel (wheelbase calibration) | [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md) |

### Imitation learning and action chunking
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\theta^\star = \arg\min_\theta \; \mathbb{E}_{(o,a)\sim\mathcal{D}} \left[ \ell\big(\pi_\theta(o), a\big) \right]$ | behaviour cloning objective | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $J(\pi_\theta) \le J(\pi^*) + \sum_{t=1}^{T} \epsilon\,(T - t) \approx J(\pi^*) + \tfrac{1}{2}\epsilon T^2$ | compounding error bound $O(\epsilon T^2)$ (DAgger: $O(\epsilon T)$) | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $(1 - \epsilon)^T$ | probability of a mistake-free episode | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $\arg\min_{\hat a}\; \mathbb{E}_{a \sim p(a\mid o)}\lVert\hat a - a\rVert^2 = \mathbb{E}[a \mid o]$ | MSE predicts the mean (averages modes) | [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md) |
| $P_\text{clean} = (1-\epsilon)^{T/h}$ | clean-episode probability with chunking | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $a_t = \frac{\sum_{i=0}^{n-1} w_i\, a_t^{(i)}}{\sum_{i=0}^{n-1} w_i},\quad w_i = e^{-m\,i}$ | ACT temporal ensembling ($w_0$ = oldest) | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $N_\text{eff} = (\sum w_i)^2 / \sum w_i^2$ | effective count of a weighted average (noise $\sigma/\sqrt{N_\text{eff}}$) | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| $\mathcal{L} = \frac{1}{k}\sum_{j=0}^{k-1} \lVert \hat a_{t+j} - a_{t+j} \rVert_1 + \beta\; D_\text{KL}\big(q_\phi(z \mid a_{t:t+k}, s_t)\,\Vert\,\mathcal{N}(0, I)\big)$ | ACT CVAE loss ($\beta = 10$) | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |

### Diffusion policies and VLAs
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $A^k = \sqrt{\bar\alpha_k}\, A^0 + \sqrt{1 - \bar\alpha_k}\;\varepsilon$ | forward noising of an action chunk | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\mathcal{L}(\theta) = \mathbb{E}\Big[\big\lVert\varepsilon - \varepsilon_\theta\big(\sqrt{\bar\alpha_k} A^0 + \sqrt{1-\bar\alpha_k}\,\varepsilon,\; k,\; O\big)\big\rVert^2\Big]$ | diffusion policy training loss | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $A^{k-1} = \frac{\sqrt{\bar\alpha_{k-1}}\,\beta_k}{1-\bar\alpha_k}\,\hat A^0 + \frac{\sqrt{\alpha_k}\,(1-\bar\alpha_{k-1})}{1-\bar\alpha_k}\,A^k + \sigma_k z$ | DDPM reverse step | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\sigma_k^2 = \frac{1-\bar\alpha_{k-1}}{1-\bar\alpha_k}\beta_k$ | DDPM step noise variance | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $A^{k-1} = \alpha\,(A^k - \gamma\,\varepsilon_\theta(O, A^k, k) + \mathcal{N}(0, \sigma^2 I))$ | Langevin form of the denoising step | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $A^{k'} = \sqrt{\bar\alpha_{k'}}\,\hat A^0 + \sqrt{1-\bar\alpha_{k'}}\;\hat\varepsilon$ | DDIM deterministic step (can skip levels) | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $\sqrt{1-\bar\alpha}/\sqrt{\bar\alpha}$ | error amplification of noise prediction at a noise level | [18.05](../18-embodied-ai/18.05-diffusion-policies.md) |
| $b = \min\left(255,\ \left\lfloor \frac{a - a_{\min}}{a_{\max} - a_{\min}}\cdot 256 \right\rfloor\right),\quad \hat a = a_{\min} + (b + 0.5)\,\frac{a_{\max} - a_{\min}}{256}$ | binned action tokens | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $x_\tau = (1-\tau)\,\epsilon + \tau A,\quad \mathcal L = \left\lVert v_\theta(x_\tau, \tau, o) - (A - \epsilon)\right\rVert^2$ | flow-matching training for a VLA | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $x_{\tau + 1/K} = x_\tau + \tfrac{1}{K}\, v_\theta(x_\tau, \tau, o)$ | flow-matching Euler inference | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| $\hat a_{d} = \frac{a_d - \mu_d^{(k)}}{\sigma_d^{(k)}},\quad \mathcal L = \frac{1}{\sum_d m_d}\sum_{d} m_d \left\lvert\hat a^\text{pred}_d - \hat a_d\right\rvert$ | per-dataset normalization and masked loss | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |
| $p_i = \frac{n_i^{\alpha}}{\sum_j n_j^{\alpha}}$ | dataset mixture sampling weights | [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md) |

### Compute, training and latency budgets
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $M_\text{infer} \gtrsim 2N \text{ bytes}$ | inference memory floor (half precision) | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $M_\text{train} \gtrsim 16N \text{ bytes}$ | full-training memory floor (mixed precision + AdamW) | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $t_\text{open} = \frac{h}{f_\text{control}}$ | open-loop time per chunk | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $t_\text{inf} - 1/f_\text{control}$ | time the arm is frozen per cycle (synchronous inference) | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $t_\text{open} / (t_\text{open} + t_\text{freeze})$ | fraction of time the arm moves | [18.01](../18-embodied-ai/18.01-embodied-ai-landscape.md) |
| $T = \left\lceil \frac{N_\text{frames}\cdot E}{B} \right\rceil \cdot t_\text{step}$ | training wall-clock time | [18.06](../18-embodied-ai/18.06-train-first-policy-lerobot.md) |
| $S = \text{steps} \times B,\quad \text{epochs} = S / N_\text{frames},\quad \text{wall-clock} = S / r$ | training budget | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $L = \frac{8\,M}{U} + T_\text{inf} + T_\text{rtt} + T_\text{jitter}$ | remote inference latency | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $\text{threshold} \cdot H \ \ge\ \lceil L \cdot f \rceil$ | async queue refill threshold | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |
| $H > 2Lf$ | chunk length with one request in flight | [18.08](../18-embodied-ai/18.08-fine-tuning-a-vla.md) |

### LLM agents: latency, cost and structure
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d_\text{dead} = v \cdot (T_\text{poll} + T_\text{rt})$ | distance driven before a decision takes effect | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $v_\text{max} = \frac{d_\text{margin}}{T_\text{poll} + T_\text{rt}}$ | maximum safe speed for a decision loop | [19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md) |
| $N_\text{in} = \sum_{k=1}^{T}\left(t_\text{tools} + t_\text{sys} + t_\text{transcript}(k)\right)$ | input tokens of a multi-turn agent | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md) |
| $\text{cost} = \frac{N_\text{in}}{10^6}\times \text{price}_\text{in} + \frac{N_\text{out}}{10^6}\times \text{price}_\text{out}$ | LLM cost of a task | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md), [19.06](../19-llm-robot-agents/19.06-task-planning-decomposition.md) |
| $E_\text{extra} = n \cdot k$ | extra transitions for global aborts in a flat state machine | [19.04](../19-llm-robot-agents/19.04-state-machines.md) |
| $n - 1$ | edges of a tree with $n$ nodes | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $N_\text{ticks} = t_\text{task} / T_\text{tick}$ | behaviour-tree ticks per task | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $t_\text{react} \le T_\text{tick} + t_\text{cancel}$ | worst-case reaction time of a behaviour tree | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| $P(\text{false alarm after } n \text{ looks}) = p_\text{miss}^{\,n}$ | verification with repeated looks | [19.07](../19-llm-robot-agents/19.07-perception-action-loops.md) |
| $1 - (1 - p)^{n}$ | probability of at least one false alarm in $n$ steps | [19.07](../19-llm-robot-agents/19.07-perception-action-loops.md) |
| $p(t) = p_0 \cdot 2^{-(t - t_\text{seen}) / T_{1/2}}$ | belief decay with a half-life | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| belief $\times\, 0.45$ per miss | miss penalty when looking and not seeing | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $\mathbb{E}[\text{trust memory}] = d + (1 - p)\,S$ vs $\mathbb{E}[\text{search}] = S$ | expected cost of acting on memory | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $p^\star = \frac{d}{S}$ | belief threshold for trusting memory | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $\text{score} = \sum_\text{words} \text{idf} \cdot \frac{\text{tf}\,(k_1+1)}{\text{tf} + k_1(1 - b + b\cdot\text{len}/\text{avg\_len})}$ | BM25 retrieval score | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| $\text{score} \times (0.5 + 0.5 \cdot 2^{-\text{age}/86400})$ | recency factor (age in s) | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |

### Voice interface
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $L = 20 \log_{10} \sqrt{\tfrac{1}{N}\textstyle\sum_i x_i^2}$ | frame level in dBFS (RMS) | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| $\text{distance} = v \cdot t$ | distance driven during a stop command's latency | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| $T_\text{first word} = T_\text{endpoint} + T_\text{ASR} + T_\text{agent} + T_\text{TTS first audio}$ | voice latency budget | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| $T_\text{ASR} = \text{fixed} + \text{RTF} \times \text{audio seconds}$ | speech-recognition latency model | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |

## Final robot, module 21, projects and other foundations

### System architecture: bandwidth and latency
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $B = \sum_{\text{publishers}} f_i \cdot s_i$ | total bandwidth of all published topics | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $T_\text{worst} = \sum_i \left( \frac{1}{f_i} + c_i \right)$ | worst-case latency of a chain: each step waits up to one period then computes | [20.01](../20-final-robot/20.01-final-architecture.md) |
| $\text{rate} = \text{bytes per message} \times \text{messages per second}$ | raw data rate of a sensor stream | [FL.11](../optional-foundations/linux-and-tools/FL.11-git-for-robotics.md) |
| $\text{level} = \text{STALE} \text{ if } t_\text{now} - t_\text{last} > k \cdot T_\text{report}, \text{ else the reported level}$ | staleness rule: a silent component is declared STALE after $k$ of its own periods | [20.03](../20-final-robot/20.03-software-integration-bringup.md) |

### Power and wiring
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\Delta V = 2 \, L \, \rho_{\text{AWG}} \, I$ | voltage drop in a wire run (current goes out and back) | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $1.25 \, I_\text{continuous} \;\le\; I_\text{fuse} \;\le\; I_\text{ampacity}$ | fuse sizing: above normal draw, below what the wire can carry | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $P_\text{in} = P_\text{out} / \eta, \quad I = P_\text{in} / V$ | input power and current of a switching regulator (constant-power load) | [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $V_{load} = V_{oc} - I \cdot (R_{int} + R_{branch})$ | Thevenin voltage sag seen by a load | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $E = \tfrac12 CV^2$ | energy stored in a capacitor | [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $E = I \cdot V \cdot t$ | electrical energy of a pulse (power times time) | [21.03](../21-war-machine/21.03-releasers-i-latches.md), [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $F_\text{mag} \;\propto\; \Phi^2 \;\propto\; I^2 \;\propto\; V^2$ | solenoid force grows with the square of voltage | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $F = F_\text{rated} \, (V/V_{rated})^2$ | solenoid force at a lower supply voltage | [21.03](../21-war-machine/21.03-releasers-i-latches.md) |

### Stability and tipping
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $x_{cg} = \frac{\sum_i m_i x_i}{\sum_i m_i} \qquad z_{cg} = \frac{\sum_i m_i z_i}{\sum_i m_i}$ | combined centre of gravity from a parts table | [21.01](../21-war-machine/21.01-war-machine-architecture.md), [20.02](../20-final-robot/20.02-hardware-integration.md) |
| $m\,a\,z_\text{cg} > m\,g\,x_\text{cg} \quad\Longrightarrow\quad a_\text{tip} = \frac{g\,x_\text{cg}}{z_\text{cg}}$ | acceleration that tips the robot backwards over the axle | [20.02](../20-final-robot/20.02-hardware-integration.md), [21.01](../21-war-machine/21.01-war-machine-architecture.md), [21.08](../21-war-machine/21.08-weights-power-stability.md) |
| $a = g \cdot (x_{caster} - x_{CG})/z_{CG}$ | braking deceleration that tips the robot nose-down over the caster | [20.02](../20-final-robot/20.02-hardware-integration.md), [21.08](../21-war-machine/21.08-weights-power-stability.md) |

### Stopping distance and safety
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d_{\text{stop}} = v\,t_{\text{reaction}} \;+\; \frac{v^{2}}{2a}$ | stopping distance: travel during reaction plus braking distance | [P01](../projects/P01-manual-drive.md) |
| $d_\text{stop} = v\,t_\text{dead} + \frac{v^2}{2a}$ | stopping distance of a stop mechanism with dead time | [20.04](../20-final-robot/20.04-safety-case.md) |
| $v_\text{max} = -a\,t_\text{dead} + \sqrt{a^2 t_\text{dead}^2 + 2\,a\,d_\text{margin}}$ | highest speed that still stops within the margin | [20.04](../20-final-robot/20.04-safety-case.md) |
| $d = v\,(T_{\text{watchdog}} + T_{\text{control}}) + \frac{v^2}{2a}$ | distance driven before a watchdog stop takes effect and brakes | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $T_c + T_b > W \;\Rightarrow\; T_b > W - T_c$ | longest blocking call before the firmware watchdog trips | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $\text{RPN} = S \times O \times D$ | FMEA risk priority number | [21.09](../21-war-machine/21.09-failure-modes-and-safety.md) |
| $\text{release force} = \text{holding force} \times 3$ | latch release force with a safety factor of 3 | [21.03](../21-war-machine/21.03-releasers-i-latches.md) |

### Mechanics of payloads and launchers
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $E = \tfrac{1}{2} m v^2$ | kinetic energy of a moving payload | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $E = m g h$ | potential energy of a payload dropped from height $h$ | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $v = \sqrt{2gh}$ | speed of a dropped payload at the bottom | [21.02](../21-war-machine/21.02-payloads-and-bom.md) |
| $E = F \times d$ (also $W = F \cdot d$) | work done by a force over a stroke | [21.01](../21-war-machine/21.01-war-machine-architecture.md), [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $J = \Delta p = m v$ | impulse of a launch (momentum given to the payload) | [21.01](../21-war-machine/21.01-war-machine-architecture.md) |
| $v_{kick} = J/M = v \cdot m_{proj}/m_{robot}$ | recoil speed of the robot (momentum conservation) | [21.01](../21-war-machine/21.01-war-machine-architecture.md), [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\tau = F \times r$ | torque a servo needs to push with force $F$ at lever arm $r$ | [21.03](../21-war-machine/21.03-releasers-i-latches.md) |
| $k = F/x$ | spring stiffness from a pull test (Hooke's law) | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $E = \tfrac{1}{2}kx^2$ | energy stored in a stretched spring | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $k = 2E/x^2$ | stiffness needed to store energy $E$ at stretch $x$ | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $v = \sqrt{2E/m}$ | launch speed from the energy delivered to a payload | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\text{range}_{ideal} = v^2/g$ | maximum no-drag range (at 45°) | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\text{range} = v^2 \sin(2\theta)/g$ | no-drag range at launch angle $\theta$ | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $\text{apex} = v^2 \sin^2\theta/(2g)$ | highest point of the trajectory | [21.04](../21-war-machine/21.04-releasers-ii-launch.md), [21.10](../21-war-machine/21.10-integration-and-demo.md) |
| $I = m r^2 + I_{arm}$ | moment of inertia of a throw arm with a payload at radius $r$ | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $E = \tfrac{1}{2} I \omega^2$ | rotational kinetic energy | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $v = \omega r$ | tip speed of a rotating arm (slew rate times radius) | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |
| $A/m$ | area-to-mass ratio, how strongly drag slows a projectile | [21.04](../21-war-machine/21.04-releasers-ii-launch.md) |

### Triggers: RC, vision and sound
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $d = \frac{w - 1000}{2000 - 1000}$ | RC channel value from PPM pulse width ($w$ in µs) | [21.05](../21-war-machine/21.05-triggers-remote.md) |
| $f = \frac{w/2}{\tan(\text{FOV}/2)}$ | camera focal length in pixels from image width and field of view | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $d = \frac{f \cdot s}{p}$ | distance to a marker of known width from its pixel width (pinhole model) | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $p_f^N$ | probability of a false fire when $N$ consecutive frames must all agree (dwell) | [21.06](../21-war-machine/21.06-triggers-vision.md) |
| $\Delta_{dB} = L_\text{event} - L_\text{floor}$ | how far a sound is above the background | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $\text{RMS} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} x_i^2}$ | root-mean-square loudness envelope of an audio window | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $\lambda_\text{false} = R \cdot p_1 \cdot p_2$ | expected false fires per hour of a double-clap trigger | [21.07](../21-war-machine/21.07-triggers-sound.md) |
| $P(\ge 1) = 1 - e^{-\lambda T}$ | probability of at least one random event in time $T$ at rate $\lambda$ (Poisson) | [21.07](../21-war-machine/21.07-triggers-sound.md) |

### Wheels, encoders and odometry
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{metres per tick} = \frac{2\pi r}{N}$ | distance a wheel rolls per encoder tick | [P03](../projects/P03-encoder-movement.md), [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $d = n \cdot \frac{2\pi r}{N}$ | distance from a tick count | [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $\text{ticks for } d = \frac{d}{2\pi r / N}$ | ticks needed to drive distance $d$ | [P03](../projects/P03-encoder-movement.md) |
| $\text{ticks per wheel for a spin of } \theta = \frac{(b/2)\,\theta}{2\pi r / N}$ | ticks each wheel needs (in opposite directions) to turn in place by $\theta$ | [P03](../projects/P03-encoder-movement.md) |
| $s = \frac{L}{2}\,\lvert\theta\rvert$ | arc length each wheel travels when turning in place | [FPY.04](../optional-foundations/python/FPY.04-typing-dataclasses-packaging.md) |
| $\Delta = \big((n - p + 2^{15}) \bmod 2^{16}\big) - 2^{15}$ | tick difference of a 16-bit counter that wraps around | [FPY.01](../optional-foundations/python/FPY.01-python-for-csharp-devs.md) |
| $\omega_k = \frac{n_k - n_{k-1}}{N}\cdot 2\pi \cdot f_s$ | wheel speed from the tick change between two samples | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\Delta\omega = 2\pi f_s / N$ | speed resolution: the speed of one tick per sample | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $v = r(\omega_l + \omega_r)/2$ | robot forward speed from wheel speeds (differential drive) | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega = r(\omega_r - \omega_l)/L$ | robot yaw rate from wheel speeds (differential drive) | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $R = v/\omega$ | turning radius | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $\omega_{L,R} = (v \mp \omega\, b/2)/r$ | wheel speeds needed for a forward speed and yaw rate (inverse kinematics) | [P04](../projects/P04-pid-movement.md) |
| $\theta = (d_{right} \cdot \text{ratio} - d_{left}) / b$ | heading from left and right wheel distances (with radius-ratio correction) | [P04](../projects/P04-pid-movement.md) |
| $\sigma_x(d) \approx k \sqrt{d}$ | odometry error growth when random slip dominates | [P08](../projects/P08-odometry.md) |
| $\sigma_x(d) \approx k\,d$ | odometry error growth when a scale (calibration) error dominates | [P08](../projects/P08-odometry.md) |
| $\sigma = \sqrt{\text{variance}}$ | standard deviation from a covariance diagonal entry | [P08](../projects/P08-odometry.md) |
| $\text{error} \propto t^2$ | position error from double-integrating an accelerometer | [P05](../projects/P05-autonomous-square-path.md), [P08](../projects/P08-odometry.md) |

### Motor control
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\omega(t) = K u\,(1 - e^{-t/\tau})$ | first-order motor step response | [P04](../projects/P04-pid-movement.md) |
| $\omega(t) = \omega_{ss}\left(1 - e^{-(t - t_0)/\tau}\right),\quad t \ge t_0$ | first-order step response starting at time $t_0$; 63.2 % at $\tau$, 95 % at $3\tau$ | [FPY.03](../optional-foundations/python/FPY.03-plotting-matplotlib.md) |
| $u = K_p \cdot e$ | proportional control output | [FL.10](../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md) |
| $T \approx 2\pi/\omega_{BW}$ | oscillation period that points to a too-high inner-loop gain | [P04](../projects/P04-pid-movement.md) |

### Frames, localization, navigation and vision
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $p_i^{robot} = \begin{bmatrix} r_i\cos\theta_i \\ r_i\sin\theta_i \end{bmatrix}$ | LiDAR beam (range, angle) to a point in the robot frame | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $p_i^{world} = R(\psi)\,p_i^{robot} + \begin{bmatrix}x\\y\end{bmatrix}$ | robot-frame point to world frame (rotate, then translate) | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $R(\psi) = \begin{bmatrix}\cos\psi & -\sin\psi\\ \sin\psi & \cos\psi\end{bmatrix}$ | 2D rotation matrix | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $P_{world} = P\,R^{\mathsf T} + t$ | the same transform for many points stored as rows | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $R^{\mathsf T} = R^{-1}$ | the inverse of a rotation is its transpose | [FPY.02](../optional-foundations/python/FPY.02-numpy-essentials.md) |
| $d = \sqrt{(\mathbf{x}_{\text{true}} - \boldsymbol{\mu})^\top \Sigma^{-1} (\mathbf{x}_{\text{true}} - \boldsymbol{\mu})}$ | Mahalanobis distance: how many standard deviations the truth is from the estimate | [P10](../projects/P10-localization.md) |
| $r_{\text{insc}} = \min(\text{half-length}, \text{half-width})$ | inscribed radius of a rectangular footprint | [P11](../projects/P11-autonomous-navigation.md) |
| $r_{\text{circ}} = \sqrt{\text{half-length}^2 + \text{half-width}^2}$ | circumscribed radius of a rectangular footprint | [P11](../projects/P11-autonomous-navigation.md) |
| $\text{cost}(d) = 253$ if $d \le r_{\text{insc}}$; $252 \cdot e^{-k\,(d - r_{\text{insc}})}$ if $r_{\text{insc}} < d \le r_{\text{inflation}}$; $0$ if $d > r_{\text{inflation}}$ | Nav2 inflation-layer cost of a cell at distance $d$ from an obstacle | [P11](../projects/P11-autonomous-navigation.md) |
| $d = w/2$ | distance from a doorway's centre line to each frame | [P11](../projects/P11-autonomous-navigation.md) |
| $r\,\omega\,\Delta t$ | sideways position error from looking up a transform at the wrong time while spinning | [P13](../projects/P13-robot-plus-vision.md) |
| $\sigma_{total} = \sqrt{\sigma_1^2 + \sigma_2^2 + \dots}$ | combining independent error sources in quadrature | [P15](../projects/P15-vision-guided-arm.md) |
| $l = \log\frac{p}{1-p}$ | log-odds of a grid cell being occupied | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |

### 3D printing and CAD
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $C_{material} = m \cdot \frac{P_{kg}}{1000}$ | filament cost of a part ($m$ in grams) | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $C_{own} = C_{printer} + N \cdot \bar{c}_{home}$ | total cost of owning a printer for $N$ prints | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $C_{service} = N \cdot \bar{c}_{service}$ | total cost of a print service for $N$ parts | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $N^* = \frac{C_{printer}}{\bar{c}_{service} - \bar{c}_{home}}$ | break-even number of prints | [F3D.01](../optional-foundations/3d-printing-and-cad/F3D.01-why-3d-printing.md) |
| $\text{DOF}_{free} = \sum \text{DOF}_{geometry} - \sum \text{DOF}_{removed} = 0$ | a sketch is fully constrained when no degrees of freedom are left | [F3D.02](../optional-foundations/3d-printing-and-cad/F3D.02-cad-basics.md) |
| $s = r\left(1 - \cos\frac{\pi}{n}\right)$ | sagitta: worst gap between a circle and its $n$-segment polygon | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $2r\cos(\pi/n)$ | flat-to-flat width of a polygon-exported hole | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $\text{size} = 84 + 50 \times \text{triangles}$ | binary STL file size in bytes | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $V_{tri} = \frac{a \cdot (b \times c)}{6}$ | signed volume per triangle; summed over a closed mesh gives the part volume | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $m \approx \rho\left[\min(V, A t) + f\,\big(V - \min(V, A t)\big)\right]$ | printed mass: solid shell plus partly filled core | [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md) |
| $C = m\,(1 + w)\,\frac{P_{kg}}{1000}$ | part cost including waste | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |
| $e_{max} = \frac{H - F}{2}$ | allowed hole-position error, fixed fastener (screw located by a thread) | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $e_{max} = H - F$ | allowed hole-position error, floating fastener (clearance holes on both parts) | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $\Delta L = k L$ | length lost to shrinkage | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $\text{scale} = 1/(1-k)$ | slicer scale that cancels shrinkage | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_{measured} = g \cdot d_{CAD} + b$ | linear fit of how the printer changes hole sizes | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_{CAD} = \frac{d_{target} - b}{g}$ | hole size to draw to get a target printed size | [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md) |
| $d_{floor} = \frac{h}{\tan(\theta_{FoV}/2 - \alpha)}$ | distance at which a range sensor's cone hits the floor | [F3D.05](../optional-foundations/3d-printing-and-cad/F3D.05-mounting-bracket.md) |
| $e = \frac{s}{\cos 30°}$ | across-corners width of a hex nut from its across-flats width | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $L_{screw} \le t + L_i - 0.5\text{ mm}$ | longest screw that will not bottom out in a heat-set insert | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $d = m\,z \qquad d_a = m\,(z + 2) \qquad d_f = m\,(z - 2.5) \qquad p = \pi m$ | spur gear pitch, outside and root diameters and circular pitch | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $a = \frac{m\,(z_1 + z_2)}{2}$ | centre distance of two meshing gears | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $i = \frac{z_2}{z_1} = \frac{\omega_1}{\omega_2} \approx \frac{T_2}{T_1}$ | gear ratio: slows speed and multiplies torque | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $z_{min} = \frac{2}{\sin^2 20°} = 17.1$ | fewest teeth on a 20° gear before undercut | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $F_t = \frac{2T}{d}$ | tangential force on gear teeth from torque | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $\sigma \approx \frac{F_t}{b\,m\,Y}$ | Lewis formula: bending stress at the tooth root | [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md) |
| $\Delta T = T_{limit} - T_{environment}$ | temperature margin of a material (negative means it softens) | [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md) |

### Embedded and real-time computing
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{wasted} = \text{sizeof(struct)} - \sum_i \text{sizeof(field}_i)$ | bytes lost to struct padding | [FC.03](../optional-foundations/cpp-and-embedded/FC.03-cpp-for-csharp-devs.md) |
| $T_{\text{rebuild}} \approx \frac{n \cdot t_c}{p} + t_{\text{link}}$ | rebuild time after editing a widely included header | [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md) |
| $\text{CPU load} = f \cdot t_h$ | fraction of CPU spent in an interrupt handler | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $t_b < \texttt{stable\_ms} < t_r$ | debounce time window: longer than bounce, shorter than real events | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| $\text{flash} = \text{text} + \text{data}$ | firmware flash usage | [FC.06](../optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md) |
| $\text{static RAM} = \text{data} + \text{bss}$ | firmware static RAM usage | [FC.06](../optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md) |
| $q = \text{round}(v \cdot 2^{16})$, resolution $2^{-16}$ | Q16.16 fixed-point encoding of a real number | [FC.06](../optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md) |
| $U = \sum_{i=1}^{n} \frac{C_i}{T_i}$ | CPU utilisation of periodic tasks | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $U \le n\left(2^{1/n} - 1\right)$ | Liu-Layland rate-monotonic schedulability bound | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $\text{missed control steps} = \left\lceil \frac{T_\text{block}}{T_\text{control}} \right\rceil$ | control steps skipped by a blocking call on a cooperative scheduler | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $R_i = C_i + \sum_{j \in hp(i)} \left\lceil \frac{R_i}{T_j} \right\rceil C_j$ | worst-case response time of a task (iterate until it stops changing) | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| $\text{frames queued} = \lfloor T_b \cdot f_t \rfloor$ | telemetry frames that pile up during a blocking call | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $\text{lateness after } n \text{ iterations} = n\epsilon, \quad f_{real} = 1/(T + \epsilon)$ | drift of a loop timed with relative sleeps | [FPY.05](../optional-foundations/python/FPY.05-asyncio-for-csharp-devs.md) |
| $S = \frac{1}{(1 - f) + \dfrac{f}{s}}$ | Amdahl's law: overall speed-up from speeding up one part | [FPY.06](../optional-foundations/python/FPY.06-python-performance.md) |

### Linux and networking
| Formula | Name / what it gives | Lesson |
|---|---|---|
| $\text{months left} = 12\,(Y_{\text{EOL}} - Y_{\text{now}}) + (M_{\text{EOL}} - M_{\text{now}})$ | months of support left for a ROS or Ubuntu release | [FL.01](../optional-foundations/linux-and-tools/FL.01-why-linux.md) |
| $T_{\text{°C}} = \frac{\text{value}}{1000}$ | CPU temperature from the millidegree value in `/sys` | [FL.04](../optional-foundations/linux-and-tools/FL.04-filesystem.md) |
| $\text{utilization} \approx \text{load} / \text{cores}$ | CPU saturation from the Linux load average | [FL.06](../optional-foundations/linux-and-tools/FL.06-processes-systemd.md) |
| $\text{addresses in subnet} = 2^{32-p}, \qquad \text{usable hosts} = 2^{32-p} - 2$ | IPv4 subnet size from prefix length | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $\text{discovery multicast} = 7400 + 250d$ | DDS discovery multicast port for ROS domain $d$ | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $\text{discovery unicast} = 7400 + 250d + 10 + 2i$ | DDS discovery unicast port for participant $i$ | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $\text{user data multicast} = 7400 + 250d + 1$ | DDS user-data multicast port | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |
| $\text{user data unicast} = 7400 + 250d + 11 + 2i$ | DDS user-data unicast port for participant $i$ | [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md) |

---

## See also

- [Glossary](glossary.md) — every term in words.
- [`curriculum/concept-index.md`](../curriculum/concept-index.md) — every concept and the lesson that owns it.
