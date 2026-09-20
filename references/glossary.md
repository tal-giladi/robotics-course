# Glossary

Every term this course uses, defined in one or two sentences, with the lesson that owns it. It is
for looking things up, not for reading end to end.

- **The definition is the short version.** The link is where the term is actually taught, with the
  numbers, the derivation and the experiment. If a definition is enough, stop reading; if it isn't,
  follow the link.
- **A complete concept → lesson index** (every `teaches` entry in the syllabus, including the ones
  that are skills rather than vocabulary) is [`curriculum/concept-index.md`](../curriculum/concept-index.md).
  `python course.py learn <concept>` searches the same data.
- **Where a term has a .NET or distributed-systems twin**, it is given in half a sentence. The
  analogy is a handhold, not an equivalence — ROS 2 actions really are not `Task<T>`.
- **Units are SI everywhere**: metres, radians, seconds, volts, amps, kilograms. See
  [Units and conventions](#units-and-conventions) below. If a number in your code has no unit in its
  name, that is a bug waiting to happen.

> [!TIP]
> **Ask your teacher:** "Define *innovation*, *residual* and *error* for me side by side — I keep
> mixing them up."

**Jump to:** [Acronyms](#acronyms) · [Units and conventions](#units-and-conventions) ·
[karmel](#karmel-the-course-robot) ·
[A](#a) [B](#b) [C](#c) [D](#d) [E](#e) [F](#f) [G](#g) [H](#h) [I](#i) [J](#j) [K](#k) [L](#l)
[M](#m) [N](#n) [O](#o) [P](#p) [Q](#q) [R](#r) [S](#s) [T](#t) [U](#u) [V](#v) [W](#w)
[X](#x-y-z)

---

## Acronyms

Robotics is dense with acronyms and nobody expands them for you. These are the ones the course uses.

| Acronym | Expansion | One line | Taught in |
|---|---|---|---|
| ADC | Analog-to-Digital Converter | Turns a voltage into an integer; resolution in bits, reference voltage sets the scale. | [FE.08](../optional-foundations/electronics/FE.08-adc.md) |
| AMCL | Adaptive Monte Carlo Localization | The ROS 2 particle filter that localizes the robot in a known map and publishes `map → odom`. | [10.09](../10-localization/10.09-amcl-in-ros2.md) |
| ACT | Action Chunking Transformer | An imitation-learning policy that predicts a chunk of future actions at once instead of one step. | [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md) |
| API | Application Programming Interface | Used in its ordinary software sense throughout. | — |
| BMS | Battery Management System | The protection board on a lithium pack: over-discharge, over-current and balancing. | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| BOM | Bill of Materials | The shopping list for a build, part by part with quantities. | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| BT | Behavior Tree | A tick-driven task structure with sequences, fallbacks and a blackboard; Nav2 and the LLM agent both use one. | [19.05](../19-llm-robot-agents/19.05-behavior-trees.md) |
| CAD | Computer-Aided Design | Parametric 3D modelling for brackets and mounts. | [F3D.02](../optional-foundations/3d-printing-and-cad/F3D.02-cad-basics.md) |
| CAN | Controller Area Network | A differential, multi-drop, error-checked bus used by robot actuators. | [FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md) |
| CLIP | Contrastive Language–Image Pre-training | Puts images and text in one embedding space, which gives you open-vocabulary matching. | [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md) |
| CNN | Convolutional Neural Network | The image network family: local kernels, weight sharing, pooling. | [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md) |
| COBS | Consistent Overhead Byte Stuffing | A framing scheme that makes a byte value impossible inside a packet, so frame boundaries are unambiguous. | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| CPR | Counts Per Revolution | Encoder counts for one full turn; karmel's is 11 × 56 × 4 = 2464 per wheel revolution. | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| CRC | Cyclic Redundancy Check | A checksum that detects corrupted serial frames. | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |
| CUDA | Compute Unified Device Architecture | NVIDIA's GPU compute stack; what Jetson inference runs on. | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| DDS | Data Distribution Service | The pub/sub middleware under ROS 2: discovery, QoS and transport, with no broker process. | [04.01](../04-ros2/04.01-why-middleware.md) |
| DH | Denavit–Hartenberg | A convention for describing a kinematic chain with four parameters per joint. | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| DOF | Degrees Of Freedom | How many independent numbers describe a configuration or a pose. | [14.01](../14-robotic-arm/14.01-arm-anatomy.md) |
| DQN | Deep Q-Network | Q-learning with a neural network, a replay buffer and a target network. | [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md) |
| DWB | Dynamic Window Approach (Nav2's implementation) | A local planner that samples feasible velocity commands and scores the resulting short trajectories. | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| EKF | Extended Kalman Filter | A Kalman filter for nonlinear models: linearize with Jacobians at the current estimate. | [10.06](../10-localization/10.06-extended-kalman-filter.md) |
| EMI | Electromagnetic Interference | Motor and switching noise coupling into your signals. | [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md) |
| FK | Forward Kinematics | Joint angles → end-effector pose. | [14.04](../14-robotic-arm/14.04-forward-kinematics.md) |
| FOV | Field Of View | The angular extent a sensor sees. | [07.04](../07-sensors/07.04-tof-sensors.md) |
| FSM | Finite State Machine | Explicit states and transitions; the robust way to run a multi-step task. | [19.04](../19-llm-robot-agents/19.04-state-machines.md) |
| GIL | Global Interpreter Lock | CPython's lock: threads do not give you CPU parallelism for Python bytecode. | [03.04](../03-robot-software/03.04-timing-and-concurrency.md) |
| GPIO | General-Purpose Input/Output | A pin you can drive high/low or read. | [FE.05](../optional-foundations/electronics/FE.05-digital-analog-gpio.md) |
| IBVS | Image-Based Visual Servoing | Close the loop on pixel error, never reconstructing 3D. | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| ICP | Iterative Closest Point | Align two point sets by repeatedly pairing nearest neighbours and solving for the transform. | [11.04](../11-slam/11.04-scan-matching-icp.md) |
| ICC | Instantaneous Centre of Curvature | The point a differential-drive robot is momentarily rotating about. | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| IK | Inverse Kinematics | End-effector pose → joint angles; often several solutions or none. | [14.05](../14-robotic-arm/14.05-inverse-kinematics.md) |
| IMU | Inertial Measurement Unit | Accelerometers + gyroscopes (+ sometimes a magnetometer) on one chip. | [07.05](../07-sensors/07.05-imu-fundamentals.md) |
| I²C | Inter-Integrated Circuit | Two-wire addressed bus (SDA/SCL) with pull-ups; how most small sensors attach. | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| IoU | Intersection over Union | Overlap metric for boxes and masks; the threshold behind mAP and NMS. | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| ISR | Interrupt Service Routine | The handler an interrupt jumps to; keep it short and touch nothing slow. | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| JSON | JavaScript Object Notation | Used for progress state and LLM tool calls. | [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md) |
| LM | Levenberg–Marquardt | A damped Gauss–Newton solver: interpolates between gradient descent and Newton steps. | [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md) |
| LQR | Linear Quadratic Regulator | Optimal state-feedback gains from a quadratic cost. | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| LiDAR | Light Detection And Ranging | A laser rangefinder that sweeps a plane (2D) or a volume (3D). | [07.07](../07-sensors/07.07-2d-lidar.md) |
| mAP | mean Average Precision | The standard detection score, averaged over classes and IoU thresholds. | [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md) |
| MCAP | (a container format; the name is not a useful expansion) | rosbag2's default storage format since Iron. | [04.14](../04-ros2/04.14-rosbag2.md) |
| MCL | Monte Carlo Localization | Particle-filter localization in a known map. | [10.08](../10-localization/10.08-particle-filter-mcl.md) |
| MCP | Model Context Protocol | A standard way to expose tools to an LLM; used to give an agent typed robot skills. | [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md) |
| MDP | Markov Decision Process | States, actions, transitions, rewards — the formal frame for RL. | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| MLP | Multi-Layer Perceptron | A plain fully connected network. | [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md) |
| MPC | Model Predictive Control | Optimize a short horizon every cycle, apply the first action, repeat. | [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md) |
| MPPI | Model Predictive Path Integral | A sampling-based MPC local planner in Nav2. | [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md) |
| MSE | Mean Squared Error | The default regression loss. | [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md) |
| NEES | Normalized Estimation Error Squared | A consistency check for a filter: is its claimed covariance honest? | [10.05](../10-localization/10.05-kalman-filter-multivariate.md) |
| NMS | Non-Maximum Suppression | Removes duplicate detections that overlap above an IoU threshold. | [13.10](../13-computer-vision/13.10-object-detection-models.md) |
| ODE | Ordinary Differential Equation | The form every dynamics model in this course takes. | [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md) |
| ONNX | Open Neural Network Exchange | A portable model format for deploying a trained network on the robot. | [16.04](../16-machine-learning/16.04-models-on-edge-compute.md) |
| ORB | Oriented FAST and Rotated BRIEF | A fast, free keypoint detector + binary descriptor. | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| PBVS | Position-Based Visual Servoing | Estimate the target's 3D pose and servo in Cartesian space. | [15.06](../15-manipulation/15.06-visual-servoing.md) |
| PCA | Principal Component Analysis | The dominant directions of a point set; used for object principal axes. | [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md) |
| PD / PI / PID | Proportional–Integral–Derivative | The three-term feedback controller; P reacts to error, I removes bias, D damps. | [08.07](../08-control/08.07-pid-mathematics.md) |
| PEP 668 | "Externally managed environment" | Why `pip install` into Ubuntu 24.04's system Python is blocked. | [03.02](../03-robot-software/03.02-environments-and-pinning.md) |
| PIO | Programmable I/O | The RP2040/RP2350 state machines that count encoder edges without the CPU. | [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md) |
| PnP | Perspective-n-Point | Recover a camera pose from known 3D points and their image projections. | [13.07](../13-computer-vision/13.07-fiducial-markers.md) |
| PPO | Proximal Policy Optimization | The default on-policy RL algorithm: clipped policy updates. | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| PWM | Pulse-Width Modulation | Switch a supply on and off fast; the duty cycle sets the average voltage. | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| QoS | Quality of Service | The per-topic DDS contract: reliability, durability, history, depth. Mismatch it and you get silence. | [04.11](../04-ros2/04.11-qos.md) |
| RAII | Resource Acquisition Is Initialization | C++'s deterministic cleanup idiom (`using`/`IDisposable`'s stricter cousin). | [FC.03](../optional-foundations/cpp-and-embedded/FC.03-cpp-for-csharp-devs.md) |
| RAG | Retrieval-Augmented Generation | Give the LLM retrieved facts (here: the robot's own memory of places and objects). | [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md) |
| RANSAC | RANdom SAmple Consensus | Fit a model from minimal samples repeatedly and keep the one with most inliers. | [13.04](../13-computer-vision/13.04-features-and-matching.md) |
| REP | ROS Enhancement Proposal | ROS's standards documents; REP-103 (units/axes) and REP-105 (frames) are load-bearing here. | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| RK2 | Runge–Kutta 2nd order | Midpoint integration; second-order accurate in the step size. | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| RL | Reinforcement Learning | Learning a policy from reward instead of labels. | [17.01](../17-reinforcement-learning/17.01-rl-framing.md) |
| RMS / RMSE | Root Mean Square (Error) | The standard scalar error summary. | [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md) |
| RMW | ROS Middleware interface | The swappable layer between `rclpy`/`rclcpp` and a specific DDS vendor. | [04.01](../04-ros2/04.01-why-middleware.md) |
| ROI | Region Of Interest | A sub-window of an image or a ToF sensor's array. | [07.04](../07-sensors/07.04-tof-sensors.md) |
| ROS | Robot Operating System | Not an OS: a middleware, build system and package ecosystem. ROS 2 is the current generation. | [00.05](../00-orientation/00.05-ros2-in-one-page.md) |
| RPY | Roll, Pitch, Yaw | Three sequential rotations; ROS uses the fixed-axis XYZ convention. | [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md) |
| RRT | Rapidly-exploring Random Tree | Sampling-based planner that grows a tree toward random samples. | [12.03](../12-navigation/12.03-sampling-based-planning.md) |
| RS-485 | (a differential serial standard) | The physical layer under many smart-servo buses. | [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md) |
| RTOS | Real-Time Operating System | A scheduler that guarantees deadlines, not throughput. | [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md) |
| SAC | Soft Actor-Critic | Off-policy, entropy-regularized continuous-control RL. | [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md) |
| SAM | Segment Anything Model | Promptable segmentation: click or box, get a mask. | [13.11](../13-computer-vision/13.11-segmentation.md) |
| SBC | Single-Board Computer | The Raspberry Pi class of machine: full Linux, no hard real-time. | [00.04](../00-orientation/00.04-robot-computers.md) |
| SDF | Simulation Description Format | Gazebo's world and model format (URDF's simulation-side cousin). | [06.03](../06-simulation/06.03-gazebo-basics.md) |
| SE(2) / SE(3) | Special Euclidean group | The set of rigid transforms in the plane / in 3D: rotation plus translation. | [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md) |
| SGD | Stochastic Gradient Descent | Train on minibatches; the basis of every optimizer here. | [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md) |
| SLAM | Simultaneous Localization And Mapping | Build the map and locate yourself in it at the same time. | [11.03](../11-slam/11.03-the-slam-problem.md) |
| SO(3) | Special Orthogonal group | The set of 3D rotation matrices: orthonormal, determinant +1. | [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md) |
| SPI | Serial Peripheral Interface | Fast four-wire bus with a chip-select per device. | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| SVD | Singular Value Decomposition | Factorization behind least squares, pseudo-inverses and point-set alignment. | [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md) |
| STT / TTS | Speech-To-Text / Text-To-Speech | The two ends of a voice interface. | [19.11](../19-llm-robot-agents/19.11-voice-interface.md) |
| TD | Temporal Difference | Learn from the difference between successive value estimates rather than from full returns. | [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md) |
| TF / tf2 | TransForm library | ROS 2's time-indexed tree of coordinate frames, and the API to query it. | [05.07](../05-frames-and-transforms/05.07-tf2-broadcast-listen.md) |
| ToF | Time of Flight | Measure distance from the round-trip time of light. | [07.04](../07-sensors/07.04-tof-sensors.md) |
| UART | Universal Asynchronous Receiver/Transmitter | Plain serial: TX, RX, a common ground and an agreed baud rate. | [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md) |
| UF2 | USB Flashing Format | The drag-and-drop firmware file the Pico's BOOTSEL drive accepts. | [FC.06](../optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md) |
| URDF | Unified Robot Description Format | The XML that describes links, joints, visuals, collisions and inertias. | [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md) |
| USB | Universal Serial Bus | How the Pi talks to the Pico here (`/dev/ttyACM0`). | [01.05](../01-first-robot/01.05-pico-microcontroller-setup.md) |
| VAE | Variational Autoencoder | A latent-variable generative model; ACT uses one. | [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md) |
| VLA | Vision-Language-Action model | A policy that maps images + an instruction directly to robot actions. | [18.07](../18-embodied-ai/18.07-vision-language-action-models.md) |
| VLM | Vision-Language Model | Answers questions about images; the perception half of a VLA. | [13.14](../13-computer-vision/13.14-vision-language-models.md) |
| XRCE-DDS | eXtremely Resource Constrained Environment DDS | The wire protocol micro-ROS uses between an MCU and its agent. | [FC.08](../optional-foundations/cpp-and-embedded/FC.08-micro-ros.md) |
| YAML | YAML Ain't Markup Language | The configuration format for ROS 2 parameters, Nav2, and `karmel.yaml`. | [04.09](../04-ros2/04.09-parameters.md) |

---

## Units and conventions

The course obeys [REP-103](https://www.ros.org/reps/rep-0103.html) and
[REP-105](https://www.ros.org/reps/rep-0105.html) everywhere, in code, in lessons and in
`karmel.yaml`. Taught in [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) and [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md).

| Quantity | Unit | Notes |
|---|---|---|
| Length | metre (m) | Never millimetres in code. Datasheets give mm; convert at the boundary. |
| Angle | radian (rad) | Never degrees in code. Print degrees for humans only. |
| Time | second (s) | ROS stamps are `sec` + `nanosec`; `float` seconds in plain Python. |
| Linear velocity | m/s | `Twist.linear.x`. |
| Angular velocity | rad/s | `Twist.angular.z`, positive = counter-clockwise (left turn). |
| Mass | kilogram (kg) | |
| Force / torque | newton (N) / newton-metre (N·m) | |
| Voltage / current | volt (V) / ampere (A) | Battery capacity is quoted in mAh or Wh — see [FE.13](../optional-foundations/electronics/FE.13-batteries.md). |
| Temperature | celsius (°C) | |

**Axes (right-handed, body frame):** **x forward, y left, z up**. A positive yaw turns the robot to
its left. In diagrams: x red, y green, z blue.

**Optical frames** are the exception you will trip over: a camera's `*_optical_frame` uses
**z forward, x right, y down**, because that is what computer vision libraries assume. The fixed
rotation between `camera_link` and `camera_optical_frame` is the usual reason a detection lands on
the ceiling. → [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [13.05](../13-computer-vision/13.05-pinhole-camera-model.md)

**The standard frame chain** (REP-105): `map → odom → base_link → sensor frames`. Each frame has
exactly one parent. `map → odom` is published by the localizer and corrects drift in jumps;
`odom → base_link` is published by odometry and is smooth but drifts. → [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [09.07](../09-odometry/09.07-odometry-in-ros2.md)

**Sign conventions worth stating out loud:** encoder counts increase when the wheel drives the robot
forward; positive `linear.x` is forward; positive `angular.z` is a left turn; a positive joint angle
follows the right-hand rule about the joint's axis. Most "the robot turns the wrong way" bugs are one
of these four reversed. → [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md), [01.11](../01-first-robot/01.11-drive-and-rotate.md)

---

## karmel, the course robot

**karmel** is the differential-drive robot you build and keep extending: a Raspberry Pi 5 for ROS 2
and perception, a Raspberry Pi Pico 2 for real-time motor control and encoder counting, two geared
DC motors with hall encoders, a ToF and an ultrasonic sensor, an IMU, a camera, a 2D LiDAR, and
later a 5-DOF servo arm. → [01.01](../01-first-robot/01.01-plan-the-build.md)

| Term | What it means here | Where |
|---|---|---|
| `karmel.yaml` | `labs/config/karmel.yaml` — the single source of the robot's physical parameters (wheel radius, wheel separation, ticks per revolution, sensor offsets, software limits). Simulation and hardware both read it; a mismatch here shows up as a constant scale error. | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| `robotlab` | The course's pure-Python package: the HAL, the fakes, the mini-simulator and the auto-graded exercises. Importable without ROS 2 or hardware. | [03.05](../03-robot-software/03.05-hardware-abstraction.md) |
| `robotlab.sim` | The course mini-simulator (the syllabus calls the concept `pysim`): a deterministic, seeded, 2.5-D differential-drive model — a disc on a plane, obstacles as line segments and circles — that runs on the laptop in a second. | [06.02](../06-simulation/06.02-course-mini-simulator.md) |
| Pi ↔ Pico protocol | The line-based serial protocol between the Pi and the Pico: sequence numbers, `A <seq> OK`/`ERR` acknowledgements, a checksum, telemetry, and a 300 ms firmware watchdog. | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| `base_link` | karmel's body frame: origin at the midpoint between the drive wheels, at wheel-axle height, x forward. | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| `base_footprint` | The same point projected onto the ground plane. | [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md) |
| Stage 0–6 | The hardware buying stages: 0 = laptop only, 1 = the first robot, then IMU/camera, LiDAR, optional Jetson/depth, the arm, final integration. | [01.02](../01-first-robot/01.02-buying-hardware-in-israel.md) |
| Watchdog | The firmware rule that stops the motors if no valid command arrives for 300 ms. It is never removed "temporarily". | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| `/dev/karmel` | The stable udev symlink to the Pico, so your code never depends on whether it enumerated as `ttyACM0` or `ttyACM1`. | [03.03](../03-robot-software/03.03-robust-serial-communication.md) |

---

## A

**A\*** — Dijkstra plus an admissible heuristic (usually straight-line distance to the goal), so the
search expands toward the goal instead of in all directions. Optimal as long as the heuristic never
overestimates. → [12.02](../12-navigation/12.02-grid-path-planning.md)

**Absolute maximum ratings** — the datasheet numbers a part must never exceed even briefly; they are
destruction limits, not operating limits. Design against the "recommended operating conditions"
table instead. → [02.01](../02-robot-electronics/02.01-datasheets-and-pinouts.md)

**Acceleration** — rate of change of velocity, m/s². What you limit in software so the robot does not
lurch, tip, or slip its wheels. → [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md)

**Accelerometer** — measures proper acceleration, which at rest is just gravity. That is why it gives
you roll and pitch for free and yaw not at all. → [07.05](../07-sensors/07.05-imu-fundamentals.md)

**Accuracy vs precision** — accuracy is how close you are to the truth (bias); precision is how
repeatable you are (noise). A sensor can be precisely wrong, and calibration only fixes the first. →
[07.01](../07-sensors/07.01-sensor-fundamentals.md)

**Action (ROS 2)** — a long-running request with feedback, a result and cancellation, built on top of
topics and services. Closest .NET analogue: a long-running job with progress reporting and a
`CancellationToken`. → [04.08](../04-ros2/04.08-actions.md)

**Action (RL)** — what the agent emits each step; the thing the policy chooses. → [17.01](../17-reinforcement-learning/17.01-rl-framing.md)

**Action chunking** — predicting a block of future actions in one forward pass instead of one action
per inference, which hides policy latency and reduces jitter. → [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md)

**Action tokenization** — encoding continuous robot actions as discrete tokens so a
language-model-shaped network can emit them. → [18.07](../18-embodied-ai/18.07-vision-language-action-models.md)

**Activation function** — the nonlinearity (ReLU, GELU…) that makes a stack of layers more than one
big matrix. → [FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md)

**Actor-critic** — an RL architecture with a policy (actor) and a value estimator (critic) that
supplies a lower-variance learning signal. → [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md)

**Actuator** — anything that converts electrical energy into motion: motors, servos, grippers. → [00.03](../00-orientation/00.03-actuators-and-sensors-field-guide.md)

**Adam** — the default adaptive optimizer: per-parameter step sizes from running gradient moments. →
[FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md)

**Affine transform** — a linear map plus a translation; in homogeneous coordinates it is one matrix
multiply. → [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md)

**Agent loop** — the LLM agent's cycle: observe → decide → call a skill → observe the result → repeat,
with a termination condition. → [19.03](../19-llm-robot-agents/19.03-tool-calling-sim-robot.md)

**Aliasing** — sampling a signal too slowly, so a fast component shows up as a slow, plausible-looking
one. The reason your loop rate must be well above the dynamics you care about. → [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md)

**AMCL** — see the acronym table. Adaptive Monte Carlo Localization: a particle filter with an
adaptive particle count that corrects odometry drift against a known map by publishing
`map → odom`. → [10.09](../10-localization/10.09-amcl-in-ros2.md)

**Ament / `ament_python`** — the ROS 2 build system convention; `ament_python` packages are ordinary
Python packages with a `setup.py` and a `package.xml`. → [04.05](../04-ros2/04.05-packages-and-workspaces.md)

**Angle wrapping** — folding an angle back into (−π, π] so that 179° and −179° are recognised as 2°
apart. Forgetting it is the classic cause of "the robot spins all the way round". → [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md), [FM.02](../optional-foundations/mathematics/FM.02-angles-and-radians.md)

**Anti-windup** — clamping or back-calculating the integral term when the actuator is saturated, so
the controller does not keep accumulating error it cannot act on. → [08.05](../08-control/08.05-integral-control.md)

**AprilTag / ArUco** — printed square fiducial markers with an encoded id; detecting one gives you a
full 6-DOF pose of the tag relative to the camera. Cheap ground truth. → [13.07](../13-computer-vision/13.07-fiducial-markers.md), [10.10](../10-localization/10.10-fiducial-landmarks.md)

**Attention** — the mechanism that lets a token's representation depend on a weighted mix of all other
tokens; the core of transformers. → [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md)

**Autoencoder** — a network trained to reconstruct its input through a bottleneck, learning a compact
representation. → [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md)

**Autograd** — automatic differentiation: the framework records the forward computation and derives
the gradients. → [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md)

**Autonomy** — the degree to which the robot decides for itself. Not a binary; teleop, assisted,
supervised and fully autonomous are points on a scale. → [00.01](../00-orientation/00.01-what-is-a-robot.md)

**Axis-angle** — a rotation as a unit axis and an angle about it; the compact form behind quaternions
and rotation vectors. → [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md)

**`apt` / apt repository** — Debian/Ubuntu package management, and the signed third-party repository
ROS 2 is installed from. Most "cannot find `ros-jazzy-*`" problems are a wrong distro codename or a
broken signing key. → [FL.07](../optional-foundations/linux-and-tools/FL.07-package-management.md), [04.02](../04-ros2/04.02-installing-ros2.md)

**Attached object** — an object MoveIt treats as part of the robot after a grasp, so the planner stops
trying to avoid what the gripper is holding. → [15.09](../15-manipulation/15.09-collision-avoidance-and-recovery.md)

## B

**Back-EMF** — the voltage a spinning motor generates against its supply, proportional to speed. It is
why a motor draws far more current at stall than at speed. → [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md), [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md)

**Backpropagation** — the chain rule applied efficiently backwards through a network to get every
parameter's gradient. → [FML.07](../optional-foundations/machine-learning/FML.07-backprop-pytorch.md)

**Bag (rosbag2)** — a recording of ROS 2 messages with their timestamps, replayable later. Think
event-sourcing log: record once on the robot, debug and regression-test for ever on the laptop.
Default storage is MCAP. → [04.14](../04-ros2/04.14-rosbag2.md)

**Bandit (multi-armed)** — the simplest RL problem: one state, several actions, and the
exploration/exploitation trade-off in its purest form. → [17.02](../17-reinforcement-learning/17.02-value-exploration.md)

**Baseline (RL)** — a state-dependent value subtracted from the return to cut gradient variance
without biasing the estimate. → [17.05](../17-reinforcement-learning/17.05-policy-gradients.md)

**Baud rate** — serial symbols per second. Both ends must agree exactly, or you get plausible-looking
garbage. → [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md)

**Bayer filter** — the colour mosaic over a camera sensor; demosaicing interpolates it into RGB, which
is why fine colour detail is partly invented. → [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md)

**Bayes filter** — the general recursive estimator: predict with the motion model, correct with the
measurement model, repeat. The Kalman filter and the particle filter are two implementations of it. →
[10.03](../10-localization/10.03-bayes-filter.md)

**Bayes' rule** — posterior ∝ likelihood × prior. The one equation behind all of localization. →
[FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md)

**Behavior cloning** — supervised learning of a policy from demonstrations: state in, expert action
out. Simple, and it compounds its own errors. → [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md)

**Behavior tree** — a tree of sequences, fallbacks, decorators and leaf actions, re-ticked at a fixed
rate. Closest analogue: a workflow engine where every step is re-evaluated each tick rather than run
once. Nav2's navigator and the LLM agent's executor are both behavior trees. → [19.05](../19-llm-robot-agents/19.05-behavior-trees.md), [12.06](../12-navigation/12.06-nav2-architecture.md)

**Belief** — the probability distribution over where the robot thinks it is, as opposed to a single
guess. Localization is the business of maintaining one. → [10.01](../10-localization/10.01-what-is-localization.md)

**Bellman equation** — the recursive identity that a state's value equals the immediate reward plus
the discounted value of the next state. → [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md)

**Bias (sensor)** — a constant offset in a sensor's output. Gyro bias is the one that ruins your
heading if you do not estimate it. → [07.01](../07-sensors/07.01-sensor-fundamentals.md), [07.05](../07-sensors/07.05-imu-fundamentals.md)

**Blackboard** — the shared key/value store a behavior tree's nodes read and write. → [19.05](../19-llm-robot-agents/19.05-behavior-trees.md)

**Block diagram** — the control engineer's wiring diagram: blocks for plant, controller and sensor,
arrows for signals, a summing junction for error. → [FCT.01](../optional-foundations/control-theory/FCT.01-systems-signals-feedback.md)

**Blob detection** — finding connected regions that pass a colour or threshold test, then measuring
their area and centroid. The oldest trick in robot vision and still useful. → [13.03](../13-computer-vision/13.03-classical-detection.md)

**BMS** — the protection board on a lithium pack. It disconnects on over-discharge, over-current and
short, and balances cells during charge. When it "cuts out under load", believe it: your current
spike is real. → [FE.13](../optional-foundations/electronics/FE.13-batteries.md)

**Bounding box** — the axis-aligned rectangle a detector reports, with a class and a confidence. →
[13.10](../13-computer-vision/13.10-object-detection-models.md)

**Brake vs coast** — with both H-bridge low-side switches on, the motor is shorted and brakes hard;
with all switches off, it coasts. Which one your driver does at duty 0 changes how the robot stops. →
[02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md)

**Breadboard** — a solderless prototyping board. Fine for a first circuit, unreliable on a moving
robot: vibration lifts wires. → [FE.04](../optional-foundations/electronics/FE.04-breadboards-and-connectors.md), [02.08](../02-robot-electronics/02.08-from-breadboard-to-perfboard.md)

**Bresenham ray casting** — walking a straight line across grid cells in integers; how an occupancy
grid marks the free space between the sensor and a hit. → [11.02](../11-slam/11.02-occupancy-grid-mapping.md)

**Brownout** — the supply dips below the minimum operating voltage and the computer resets. On karmel
it looks like "the Pi reboots when the motors start", and the cause is almost always shared rails and
inrush current. → [02.02](../02-robot-electronics/02.02-power-budget.md)

**Brushed vs brushless motor** — brushed motors commutate mechanically and are trivially driven by an
H-bridge; brushless motors need an electronic controller but last longer and are more efficient. →
[FE.10](../optional-foundations/electronics/FE.10-dc-motors.md)

**Buck converter** — a switching step-down regulator: efficient, and noisy in ways that show up in
your ADC readings. A linear regulator burns the difference as heat instead. → [FE.15](../optional-foundations/electronics/FE.15-voltage-regulators.md), [01.07](../01-first-robot/01.07-power-system.md)

**Bundle adjustment** — jointly optimizing camera poses and 3D landmark positions to minimise
reprojection error; the back end of visual SLAM. → [11.08](../11-slam/11.08-visual-slam.md)

**Bus-off** — a CAN node that has accumulated too many transmit errors takes itself off the bus.
Repeated bus-off means wiring, termination or a rate mismatch, not software. → [FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md)

## C

**Callback group** — the ROS 2 mechanism that decides which callbacks may run concurrently
(`MutuallyExclusive` or `Reentrant`). Roughly a concurrency limiter around a set of handlers; the
standard fix for "my node deadlocks when a callback calls a service". → [04.12](../04-ros2/04.12-executors-and-callbacks.md)

**Camera calibration** — estimating the intrinsic matrix and distortion coefficients, usually from
checkerboard images, so pixels can be turned into rays. → [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md)

**Camera info** — the `sensor_msgs/CameraInfo` message carrying intrinsics, distortion and the frame
id. Without it, no node can project anything. → [07.08](../07-sensors/07.08-rgb-cameras.md)

**CAN bus** — a differential, multi-drop, arbitrating, error-checked bus. The right answer when a
daisy-chained servo bus starts collecting errors as the robot moves. → [FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md), [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md)

**Capacitor (decoupling / bulk)** — a small capacitor next to an IC supplies the fast current spikes
the wires are too slow to deliver; a bulk capacitor rides out the slower dips when the motors start.
→ [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md), [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md)

**Cartesian coordinates** — position as (x, y) or (x, y, z) on perpendicular axes; the default. →
[FM.04](../optional-foundations/mathematics/FM.04-coordinate-systems.md)

**Cascaded control** — an outer loop (heading) setting the setpoint of inner loops (wheel speeds).
Each loop is tuned separately, inner one first and faster. → [08.10](../08-control/08.10-straight-line-heading-control.md)

**Centre of mass** — the point the robot's mass acts through; keep it low and inside the support
polygon or the arm will tip the base. → [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md), [15.10](../15-manipulation/15.10-mobile-manipulation.md)

**Chain rule** — the derivative of a composition; the machinery behind both Jacobians and
backpropagation. → [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md)

**Checksum** — a value computed over a frame's bytes so the receiver can reject corrupted frames. →
[01.10](../01-first-robot/01.10-pi-pico-protocol.md)

**Clearance fit / press fit** — 3D printing tolerances: a clearance fit slides, a press fit needs
force. Printers over-extrude, so holes come out undersize — compensate. → [F3D.04](../optional-foundations/3d-printing-and-cad/F3D.04-tolerances-fits.md)

**CLIP** — see the acronym table. Image and text embeddings in one space, which is what makes
open-vocabulary detection ("find the blue mug") possible without training a class. → [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md)

**Closed-loop control** — measure the output, compute the error, act on it. The opposite of
open-loop, where you command and hope. → [08.01](../08-control/08.01-open-vs-closed-loop.md)

**Clustering** — grouping unlabelled points; used on point clouds to separate objects on a table. →
[FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md), [15.04](../15-manipulation/15.04-object-pose-estimation.md)

**`cmd_vel`** — the `geometry_msgs/Twist` (on Jazzy, `TwistStamped` for `diff_drive_controller`)
topic that carries the desired body velocity. The universal joint between "navigation" and "the
robot". → [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md)

**COBS** — see the acronym table. Byte stuffing that guarantees the delimiter byte never appears
inside a frame, so resynchronisation after a dropped byte is immediate. → [03.03](../03-robot-software/03.03-robust-serial-communication.md)

**`colcon`** — the ROS 2 workspace build tool: builds every package in `src/`, resolves the order
between them, and generates the setup files you source. Roughly `msbuild` over a solution. →
[04.05](../04-ros2/04.05-packages-and-workspaces.md)

**Collision geometry** — the simplified shapes (boxes, cylinders) used for physics and planning, as
opposed to the detailed visual mesh. Using the visual mesh for collision is how you make Gazebo and
MoveIt crawl. → [06.05](../06-simulation/06.05-robot-in-gazebo.md), [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md)

**Collision monitor** — the Nav2 node that stops or slows the robot from raw sensor data, independent
of the planners. Your last software line of defence. → [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md)

**Complementary filter** — fuse a fast, drifting signal (gyro) with a slow, noisy but unbiased one
(accelerometer) using a single blending constant. A Kalman filter's cheap, intuitive ancestor. →
[10.07](../10-localization/10.07-fusing-imu-and-odometry.md)

**Compliant gripper** — a gripper whose fingers deform, so small pose errors do not become large
forces. Compliance buys you tolerance you would otherwise have to earn with calibration. → [15.02](../15-manipulation/15.02-grippers.md)

**Compounding error** — in behavior cloning, a small action error moves the robot to a state the
expert never visited, where the policy is worse, which moves it further off. DAgger exists because
of this. → [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md)

**Configuration space** — the space of all joint/pose values; planning happens here, where the robot
is a point and obstacles are grown regions. → [12.03](../12-navigation/12.03-sampling-based-planning.md)

**Contours** — the boundaries of binary regions; OpenCV's `findContours` plus area and shape filters
gets you surprisingly far. → [13.03](../13-computer-vision/13.03-classical-detection.md)

**Contrastive learning** — train embeddings by pulling matching pairs together and pushing
non-matching pairs apart. → [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md)

**Controller manager** — the `ros2_control` process that loads, configures, activates and updates
controllers against a hardware interface, at a fixed rate. → [06.06](../06-simulation/06.06-ros2-control.md)

**Convolution** — sliding a small kernel over an image (or signal) and summing products; the basis of
blurring, edge detection and CNNs. → [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md), [FML.08](../optional-foundations/machine-learning/FML.08-cnns.md)

**Coordinate frame** — a named origin plus axes. Every measurement belongs to one; most robotics bugs
are a number used in the wrong frame. → [05.01](../05-frames-and-transforms/05.01-why-frames.md)

**Correspondence** — deciding which measurement matches which model point or landmark. Scan matching
and data association both live or die on this. → [11.04](../11-slam/11.04-scan-matching-icp.md)

**Costmap** — a grid of traversal costs built from the static map plus live sensor layers, with
obstacles inflated by the robot's radius. Nav2 keeps a global and a local one. → [12.04](../12-navigation/12.04-costmaps.md)

**Covariance** — how two variables vary together; as a matrix, the full shape of a multivariate
uncertainty. Its ellipse is the picture. → [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md), [10.02](../10-localization/10.02-uncertainty-gaussians.md)

**CRC** — see the acronym table. A checksum with good detection properties, cheap enough for a
microcontroller to run per frame. → [03.03](../03-robot-software/03.03-robust-serial-communication.md)

**Crimping** — making a gas-tight mechanical connection between wire and contact. Properly crimped
beats badly soldered on a vibrating robot. → [02.07](../02-robot-electronics/02.07-wiring-harness.md)

**Cross-embodiment** — training one policy on data from many different robots, so it transfers to a
body it was not collected on. → [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md)

**Cross-entropy** — the classification loss: the negative log probability the model assigned to the
true class. → [FML.04](../optional-foundations/machine-learning/FML.04-loss-functions.md)

**Cross product** — a × b: a vector perpendicular to both, with magnitude |a||b| sin θ. Where torque
and angular velocity come from. → [FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md)

**Crosstalk** — one ultrasonic sensor hearing another's ping, or one signal coupling into a
neighbouring wire. Fix by staggering triggers or separating wires. → [07.03](../07-sensors/07.03-ultrasonic-sensors.md), [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md)

**`cv_bridge`** — converts between `sensor_msgs/Image` and OpenCV arrays. Remember OpenCV is BGR. →
[13.15](../13-computer-vision/13.15-vision-in-ros2.md)

**CI (for robots)** — continuous integration that runs the tests which do not need hardware, on every
push, so the robot is not your test environment. → [03.11](../03-robot-software/03.11-ci-and-reproducibility.md)

**CMake** — the build-system generator ROS 2 C++ packages use (`ament_cmake`). Most first C++ failures
are linking, not compiling. → [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md)

**Cross-compilation** — building on the laptop for a different target architecture (Pico, or arm64).
→ [FC.06](../optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md)

## D

**DAgger** — Dataset Aggregation: run the learned policy, have the expert label the states it
actually visits, retrain. The standard answer to compounding error. → [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md)

**Damped least squares** — a pseudo-inverse with a damping term, so the IK solver stays finite near a
singularity instead of commanding infinite joint speed. → [14.06](../14-robotic-arm/14.06-jacobian.md)

**Damping ratio (ζ)** — how oscillatory a second-order system is: ζ < 1 overshoots, ζ = 1 is critically
damped, ζ > 1 is sluggish. → [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md)

**Data association** — matching an observation to the right landmark or track. Get it wrong once in
SLAM and the map folds. → [11.03](../11-slam/11.03-the-slam-problem.md)

**Data leakage** — evaluating on information the model should not have had; in robot data it usually
means frames from the same episode in both train and test. → [FML.03](../optional-foundations/machine-learning/FML.03-train-validation-test.md), [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md)

**Dead reckoning** — integrating your own motion (wheel ticks, heading) to estimate position, with no
external reference. Smooth, cheap, and drifting without bound. → [09.03](../09-odometry/09.03-dead-reckoning-integration.md)

**Deadband** — the range of commands that produce no motion, because friction exceeds torque. karmel's
is about 0.12 duty; below it, a P controller looks broken. → [01.08](../01-first-robot/01.08-first-motor-spin.md), [08.02](../08-control/08.02-motor-step-response.md)

**Dead-man switch** — a control that must be held for motion to continue. Teleop uses one. → [01.15](../01-first-robot/01.15-teleop-from-laptop.md)

**Decoupling capacitor** — see *Capacitor*. → [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md)

**Degrees of freedom** — the number of independent motions a mechanism has. karmel's base has 2
(forward, turn) in the plane; the arm has 5 plus a gripper. → [14.01](../14-robotic-arm/14.01-arm-anatomy.md)

**Denavit–Hartenberg** — a four-parameter-per-joint convention for describing a kinematic chain.
Compact, standard, and genuinely fiddly to get right the first time. → [14.04](../14-robotic-arm/14.04-forward-kinematics.md)

**Derivative kick** — the spike a D term produces when the setpoint steps, because the error jumps.
Fix by differentiating the measurement, not the error. → [08.06](../08-control/08.06-derivative-control.md)

**Determinism** — same inputs, same outputs, same timing. Simulations are made deterministic by
seeding; real-time firmware is made deterministic by bounded work per cycle. → [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md), [06.02](../06-simulation/06.02-course-mini-simulator.md)

**DDS** — see the acronym table. The brokerless pub/sub middleware under ROS 2. Nodes discover each
other by multicast, which is why `ROS_DOMAIN_ID`, Wi-Fi multicast and Docker networking all show up
in "my nodes can't see each other". → [04.01](../04-ros2/04.01-why-middleware.md)

**Diff drive (differential drive)** — two independently driven wheels and a passive caster. Turning
comes from a speed difference; it cannot move sideways. → [09.01](../09-odometry/09.01-diff-drive-kinematics.md)

**`diff_drive_controller`** — the `ros2_control` controller that converts `cmd_vel` into wheel
commands and publishes odometry. On Jazzy it subscribes to `TwistStamped`, not `Twist` — a common
"nothing moves" cause. → [06.06](../06-simulation/06.06-ros2-control.md)

**Diffusion model** — a generative model that learns to denoise; as a policy, it represents
multimodal action distributions instead of averaging them into something useless. → [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md),
[18.05](../18-embodied-ai/18.05-diffusion-policies.md)

**Dijkstra** — uniform-cost search: expands the cheapest frontier node until the goal is reached.
Optimal, and it explores in every direction. → [12.02](../12-navigation/12.02-grid-path-planning.md)

**Discount factor (γ)** — how much future reward is worth now; it makes infinite-horizon returns
finite and sets the effective planning horizon. → [17.02](../17-reinforcement-learning/17.02-value-exploration.md)

**Distribution shift** — the deployment data differs from the training data. The reason a policy that
scores well offline fails on the robot. → [16.06](../16-machine-learning/16.06-evaluating-ml-in-robots.md)

**Domain randomization** — randomising simulation parameters (masses, frictions, textures, latencies)
so the policy must be robust to the range that contains reality. → [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md), [06.10](../06-simulation/06.10-sim-to-real-gap.md)

**Dot product** — a · b = |a||b| cos θ: projection, similarity and "how much of a is along b". →
[FM.06](../optional-foundations/mathematics/FM.06-dot-and-cross-product.md)

**Drift** — slow, accumulating error with no bound: gyro heading drift, odometry position drift. The
reason localization and SLAM exist. → [09.06](../09-odometry/09.06-accumulated-error.md)

**Duty cycle** — the fraction of a PWM period the output is high; with a motor, roughly the fraction
of supply voltage it sees. → [FE.07](../optional-foundations/electronics/FE.07-pwm.md)

**Dynamic window** — the set of velocities reachable within one control step given acceleration
limits; the local planner searches inside it. → [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md)

**Debouncing** — filtering the mechanical chatter of a switch so one press is one event. → [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md)

**DHCP** — the protocol that hands your Pi an address. It is also why the Pi's address keeps changing
and why you want mDNS or a reservation. → [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md)

**`dialout` group** — the Linux group that grants access to serial devices. Adding yourself to it (and
logging out and in) is the fix for `Permission denied: '/dev/ttyACM0'`. → [FL.05](../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md)

**Docker (for robotics)** — containers for reproducible ROS 2 environments. Two gotchas dominate:
device passthrough for the Pico, and DDS discovery across the container boundary. → [FL.12](../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)

## E

**Edge detection** — finding intensity discontinuities with gradient operators (Sobel) or Canny. →
[FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md), [13.04](../13-computer-vision/13.04-features-and-matching.md)

**EKF** — see the acronym table. The Kalman filter for nonlinear models: linearize f and h with
Jacobians at the current estimate each step. Cheap, standard, and it diverges when the linearization
is bad. → [10.06](../10-localization/10.06-extended-kalman-filter.md)

**Embedding** — a learned vector representation where distance means similarity. → [FML.09](../optional-foundations/machine-learning/FML.09-embeddings.md)

**Embodiment** — the fact that the robot has a body with mass, friction, latency and limits, and that
this is not a detail you can abstract away. → [00.01](../00-orientation/00.01-what-is-a-robot.md)

**Encoder** — a sensor that counts wheel or motor rotation. Quadrature (two channels, 90° apart) gives
you direction as well as count. → [01.09](../01-first-robot/01.09-reading-encoders.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md)

**Encoder rollover** — a fixed-width tick counter wrapping. Handle it with modular arithmetic on the
delta, or your odometry jumps by kilometres. → [09.04](../09-odometry/09.04-odometry-from-scratch.md)

**End effector** — the business end of the arm: the gripper, or the tool frame you actually command. →
[14.01](../14-robotic-arm/14.01-arm-anatomy.md)

**Epipolar geometry** — the constraint that a point in one camera lies on a line in the other; what
makes stereo matching a 1D search. → [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md)

**ε-greedy** — explore with probability ε, otherwise take the best known action. The simplest
exploration rule. → [17.02](../17-reinforcement-learning/17.02-value-exploration.md)

**Episode** — one run from reset to termination, in RL or in policy evaluation. → [17.01](../17-reinforcement-learning/17.01-rl-framing.md)

**Error (control)** — setpoint minus measurement. Everything a feedback controller does is a function
of it. → [08.01](../08-control/08.01-open-vs-closed-loop.md)

**Error propagation** — how input uncertainties turn into output uncertainty, via the Jacobian:
Σ_out = J Σ_in Jᵀ. This is why odometry covariance grows the way it does. → [09.06](../09-odometry/09.06-accumulated-error.md)

**Euler angles** — three sequential rotations (roll, pitch, yaw). Intuitive, compact, and they suffer
gimbal lock; ROS stores orientation as quaternions for this reason. → [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md)

**Euler integration** — the simplest ODE step: x += ẋ·dt. First-order accurate; halving dt halves the
error. → [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md), [09.03](../09-odometry/09.03-dead-reckoning-integration.md)

**Exact arc integration** — integrating a constant-curvature motion in closed form instead of
approximating it as a straight line. Removes the systematic error that Euler odometry shows in
curves. → [09.03](../09-odometry/09.03-dead-reckoning-integration.md)

**Executor** — the ROS 2 object that spins a node and dispatches its callbacks. Single-threaded by
default; the scheduler you did not know you had. → [04.12](../04-ros2/04.12-executors-and-callbacks.md)

**Exploration vs exploitation** — the trade-off between trying something new and taking the best known
option. → [17.02](../17-reinforcement-learning/17.02-value-exploration.md)

**Exposure** — how long the sensor integrates light. Too long and moving scenes blur; too short and
the image is noisy. Auto-exposure is often what changed between a run that worked and one that did
not. → [07.08](../07-sensors/07.08-rgb-cameras.md), [FCV.01](../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md)

**Extrapolation error** — tf2 asking for a transform at a time outside the buffer's data:
"extrapolation into the future" (you asked too soon, or clocks disagree) or "into the past" (the
buffer is too short, or the publisher stopped). → [05.11](../05-frames-and-transforms/05.11-debugging-tf.md), [05.07](../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)

**Extrinsics** — where a camera (or any sensor) is, relative to another frame, as opposed to its
internal optics. → [13.05](../13-computer-vision/13.05-pinhole-camera-model.md)

**Eye-in-hand vs eye-to-hand** — camera mounted on the arm versus fixed in the workspace. The hand-eye
calibration equation differs between them. → [15.03](../15-manipulation/15.03-hand-eye-calibration.md)

## F

**Fake driver** — an in-memory implementation of the hardware interface used in tests. Exactly a
repository fake: same interface, no device, deterministic. → [03.05](../03-robot-software/03.05-hardware-abstraction.md)

**Fault tree** — a structured "what could make this symptom happen" decomposition, used to order the
cheapest, most informative measurement first. → [02.09](../02-robot-electronics/02.09-electrical-debugging-method.md)

**Feature (vision)** — a repeatably detectable image location (corner, blob) plus a descriptor that
lets you match it across images. → [13.04](../13-computer-vision/13.04-features-and-matching.md)

**Feedforward** — commanding what you already know is needed (the duty that produces this speed), so
feedback only has to correct the remainder. Usually the single biggest tracking improvement
available. → [08.09](../08-control/08.09-feedforward-exact-speed.md)

**Fiducial** — a deliberately designed, easily detected marker (AprilTag, ArUco, checkerboard). →
[13.07](../13-computer-vision/13.07-fiducial-markers.md)

**Filtered derivative** — a D term computed on a low-pass-filtered signal, because raw differentiation
amplifies noise. → [08.06](../08-control/08.06-derivative-control.md)

**Finite state machine** — explicit states, explicit transitions, one current state. The structure
that keeps a multi-step manipulation task debuggable. → [19.04](../19-llm-robot-agents/19.04-state-machines.md), [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md)

**Firmware** — the code running on the microcontroller, bare-metal or on MicroPython, with no OS
between it and the pins. → [FC.01](../optional-foundations/cpp-and-embedded/FC.01-embedded-systems-101.md)

**First-order system** — one state, one time constant τ: the response to a step is an exponential
approach. karmel's wheel speed is one, with τ ≈ 0.08 s. → [08.02](../08-control/08.02-motor-step-response.md), [FCT.02](../optional-foundations/control-theory/FCT.02-first-second-order-systems.md)

**Fixed frame (RViz)** — the frame RViz draws everything relative to. Set it to `map` and the robot
moves; set it to `base_link` and the world moves around a stationary robot. → [05.09](../05-frames-and-transforms/05.09-rviz.md)

**Floating input** — an input pin connected to nothing, which reads noise. Pull-ups and pull-downs
exist to prevent this. → [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md)

**Flow matching** — a continuous-time generative method related to diffusion, used in newer VLA action
heads because it needs fewer sampling steps. → [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md), [18.07](../18-embodied-ai/18.07-vision-language-action-models.md)

**Flyback diode** — a diode across an inductive load (motor, relay coil) that gives the collapsing
current somewhere to go instead of destroying the switching transistor. → [FE.18](../optional-foundations/electronics/FE.18-capacitors-diodes.md)

**Footprint** — the robot's outline used by the costmap. Too small and you clip door frames; too
large and the planner refuses doorways. → [12.04](../12-navigation/12.04-costmaps.md)

**Force closure** — a grasp where the contact forces can resist any external wrench. The formal
version of "it will not be pushed out of the gripper". → [15.01](../15-manipulation/15.01-physics-of-grasping.md)

**Forward kinematics** — joint angles in, end-effector pose out. Always solvable, always unique. →
[14.04](../14-robotic-arm/14.04-forward-kinematics.md)

**Frame** — see *Coordinate frame*.

**Framing** — marking where one serial message ends and the next begins (newline, length prefix,
COBS). Without it, one dropped byte corrupts everything after it. → [01.10](../01-first-robot/01.10-pi-pico-protocol.md), [03.03](../03-robot-software/03.03-robust-serial-communication.md)

**Free-body diagram** — every force on one body, drawn. The first step of any "will it tip / will it
climb / how much torque" question. → [FP.02](../optional-foundations/physics/FP.02-forces-newton.md)

**Friction cone** — the set of contact forces that do not slip, with half-angle arctan μ. A grasp is
safe when the required force lies inside it. → [15.01](../15-manipulation/15.01-physics-of-grasping.md), [FP.03](../optional-foundations/physics/FP.03-friction-traction.md)

**Fuse** — a deliberate weak point in series with the battery positive, sized just above your maximum
current. Non-negotiable on a lithium pack. → [01.07](../01-first-robot/01.07-power-system.md)

**Filesystem hierarchy** — where Linux keeps things: `/dev` for devices, `/etc` for configuration,
`/opt/ros` for ROS 2. Knowing it turns "file not found" into a two-second answer. → [FL.04](../optional-foundations/linux-and-tools/FL.04-filesystem.md)

**Flash vs RAM** — on a microcontroller, program storage versus working memory, both tiny and both
countable. → [FC.01](../optional-foundations/cpp-and-embedded/FC.01-embedded-systems-101.md)

**Fuel models** — Gazebo's online model library. A model that renders but has no collision geometry is
the reason the robot drives straight through it. → [06.08](../06-simulation/06.08-building-worlds.md)

## G

**Gain** — the multiplier in a controller (k_p, k_i, k_d). Tuning is choosing them; the lesson teaches
tuning by logging, not by guessing. → [08.04](../08-control/08.04-proportional-control.md), [08.08](../08-control/08.08-tuning-pid.md)

**Gaussian (normal) distribution** — the bell curve, defined by mean and variance. Kalman filters
assume everything is one, which is both their power and their failure mode. → [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md)

**Gauss–Newton** — iteratively solve a nonlinear least-squares problem by linearizing and solving the
normal equations. → [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md)

**Gazebo** — the physics simulator the course uses (Harmonic release, the `gz` command line). Gazebo
Classic is end-of-life; `ign` commands and `ros_ign_*` packages are the old names. → [06.03](../06-simulation/06.03-gazebo-basics.md)

**Gear ratio** — how much the gearbox multiplies torque and divides speed. karmel's motors are 1:56. →
[FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md)

**Geofence** — a region the agent is forbidden to command the robot into, enforced below the language
model, not by it. → [19.09](../19-llm-robot-agents/19.09-agent-safety-boundaries.md)

**Gimbal lock** — the loss of one rotational degree of freedom when two Euler axes align (pitch at
±90°). → [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md)

**GIL** — see the acronym table. Why you use processes, `asyncio` or C extensions rather than threads
for Python CPU work. → [03.04](../03-robot-software/03.04-timing-and-concurrency.md)

**Global vs local planner** — the global planner finds a route through the whole map (A*, Dijkstra);
the local planner follows it while reacting to what the sensors see now (DWB, MPPI). → [12.01](../12-navigation/12.01-the-navigation-problem.md)

**Gradient** — the vector of partial derivatives: the direction of steepest increase. → [FM.17](../optional-foundations/mathematics/FM.17-derivatives.md)

**Gradient descent** — step against the gradient to minimise a loss. → [FML.05](../optional-foundations/machine-learning/FML.05-gradient-descent.md), [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md)

**Graph optimization** — adjusting all poses in a pose graph to minimise the total constraint error.
SLAM's back end. → [11.05](../11-slam/11.05-pose-graphs-loop-closure.md)

**Grasp quality** — a score for how well a candidate grasp resists disturbance; the ε-metric is the
radius of the largest wrench ball the grasp can resist. → [15.01](../15-manipulation/15.01-physics-of-grasping.md)

**Ground (electrical)** — the common reference all voltages are measured against. Two boards that
exchange signals must share it, or nothing works. → [FE.16](../optional-foundations/electronics/FE.16-grounding.md)

**Ground loop** — two paths to ground with different potentials, so current flows through your signal
return and corrupts readings. Star grounding is the fix. → [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md)

**Gymnasium** — the standard RL environment API (`reset`, `step`, spaces). → [17.07](../17-reinforcement-learning/17.07-robot-gym-environment.md)

**Gyroscope** — measures angular rate. Integrate it for heading, and watch the bias turn into drift. →
[07.05](../07-sensors/07.05-imu-fundamentals.md)

**Git LFS** — Git Large File Storage, for model weights and bags. Clone without it and you get
130-byte pointer files that fail to load. → [FL.11](../optional-foundations/linux-and-tools/FL.11-git-for-robotics.md), [03.11](../03-robot-software/03.11-ci-and-reproducibility.md)

## H

**H-bridge** — four switches that let you apply either polarity across a motor, so it can go both
ways. Never turn both switches on one side on at once. → [FE.12](../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md)

**Hand-eye calibration** — solving AX = XB for the fixed transform between the camera and the gripper
(or the base). Without it, perception and the arm disagree by a constant offset. → [15.03](../15-manipulation/15.03-hand-eye-calibration.md)

**Hardware abstraction layer (HAL)** — an interface over the hardware with a fake implementation for
tests, so the same control code runs on the simulator, the fake and the robot. Exactly the repository
pattern, applied to motors. → [03.05](../03-robot-software/03.05-hardware-abstraction.md)

**Hardware interface (`ros2_control`)** — the plugin that reads state from and writes commands to the
real (or simulated) robot, at the controller manager's rate. Swapping it is how sim and real share one
stack. → [06.06](../06-simulation/06.06-ros2-control.md), [08.12](../08-control/08.12-control-in-ros2.md)

**Header stamp** — the `builtin_interfaces/Time` in a message header. It is the time the *data* was
captured, not when it was published, and everything downstream (tf2, message filters, SLAM) depends
on it being honest. → [07.10](../07-sensors/07.10-sensor-data-in-ros2.md)

**Heading** — the robot's yaw angle. In this course, radians, measured from the map's +x axis,
counter-clockwise positive, wrapped to (−π, π]. → [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md)

**Heat-set insert** — a brass threaded insert melted into a printed part so you can use a metal screw
repeatedly. → [F3D.06](../optional-foundations/3d-printing-and-cad/F3D.06-gears-inserts-fasteners.md)

**Heuristic** — in A*, an estimate of the remaining cost. Admissible (never overestimates) keeps A*
optimal. → [12.02](../12-navigation/12.02-grid-path-planning.md)

**Histogram filter** — a Bayes filter over a discretised state space; the easiest way to see
prediction and correction actually happening. → [10.03](../10-localization/10.03-bayes-filter.md)

**Homogeneous coordinates** — adding a 1 to a point so that rotation and translation are a single 4×4
matrix multiply, and chains compose by multiplication. → [FM.09](../optional-foundations/mathematics/FM.09-homogeneous-coordinates.md), [05.04](../05-frames-and-transforms/05.04-homogeneous-transforms.md)

**Homography** — the 3×3 projective map between two views of a plane. → [FCV.04](../optional-foundations/computer-vision/FCV.04-projective-geometry.md)

**HSV** — hue, saturation, value. Separating colour from brightness makes colour thresholding survive
lighting changes far better than RGB. → [13.01](../13-computer-vision/13.01-images-as-data.md), [FCV.03](../optional-foundations/computer-vision/FCV.03-color-spaces.md)

**Headless** — running the robot with no screen or keyboard: SSH in, and keep long-running processes
alive in `tmux` so a dropped connection does not kill them. → [01.04](../01-first-robot/01.04-raspberry-pi-setup.md), [FL.13](../optional-foundations/linux-and-tools/FL.13-tmux-headless.md)

**Hostname / mDNS** — `karmel.local` resolves through multicast DNS without a DNS server. It is also
the first thing to fail on a guest Wi-Fi that blocks multicast. → [01.04](../01-first-robot/01.04-raspberry-pi-setup.md), [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md)

## I

**I²C** — see the acronym table. Two wires, addressed devices, open-drain with pull-up resistors. A
failing `i2c.scan()` is almost always wiring, address, pull-ups or a shared-address clash. →
[FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md), [02.04](../02-robot-electronics/02.04-buses-i2c-spi-uart.md)

**ICP** — see the acronym table. Pair nearest points, solve for the transform that aligns them, repeat.
The workhorse of scan matching, and it converges to the wrong answer in a featureless corridor. →
[11.04](../11-slam/11.04-scan-matching-icp.md)

**Idempotent skill** — a robot skill that can be retried safely because running it twice has the same
effect as running it once. Essential when an LLM may repeat a call. → [19.02](../19-llm-robot-agents/19.02-robot-skill-api.md)

**Imitation learning** — learning a policy from demonstrations rather than from reward. → [18.02](../18-embodied-ai/18.02-imitation-learning-behavior-cloning.md)

**IMU** — see the acronym table. Accelerometers and gyroscopes, sometimes a magnetometer. Gives you
good short-term rotation and terrible long-term position. → [07.05](../07-sensors/07.05-imu-fundamentals.md)

**Inertia tensor** — the rotational analogue of mass, as a 3×3 matrix. Nonsense inertias are why
simulated robots jitter or launch themselves. → [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md), [06.05](../06-simulation/06.05-robot-in-gazebo.md)

**Inflation layer** — the costmap layer that grows obstacles by the robot's radius plus a decaying
cost, so the planner naturally keeps clearance. → [12.04](../12-navigation/12.04-costmaps.md)

**Innovation** — measurement minus predicted measurement, in a Kalman filter. Its size relative to the
innovation covariance tells you whether to believe the sensor or the filter. → [10.05](../10-localization/10.05-kalman-filter-multivariate.md)

**Instance segmentation** — per-object masks, as opposed to semantic segmentation's per-class masks. →
[13.11](../13-computer-vision/13.11-segmentation.md)

**Integral windup** — the integral term accumulating while the actuator is saturated, so the
controller overshoots badly when the constraint lifts. → [08.05](../08-control/08.05-integral-control.md)

**Interrupt** — a hardware event that pre-empts the main loop and runs a handler. The right way to
count encoder edges; the wrong place to do arithmetic-heavy work. → [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md)

**Intrinsics** — a camera's internal parameters: focal lengths and principal point (plus distortion).
→ [13.05](../13-computer-vision/13.05-pinhole-camera-model.md)

**Inverse kinematics** — end-effector pose in, joint angles out. May have several solutions, or none
if the target is outside the workspace. → [14.05](../14-robotic-arm/14.05-inverse-kinematics.md)

**Inverse sensor model** — the rule that turns one range reading into log-odds updates for the cells
along the beam: free before the hit, occupied at it. → [11.02](../11-slam/11.02-occupancy-grid-mapping.md)

**Infill / walls** — a 3D print's internal density and the number of perimeter shells. Walls carry
most of a part's strength; infill is cheaper than people assume it is. → [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md)

**Inference latency** — how long one model forward pass takes on the machine that will actually run
it. Measure it on the Pi or the Jetson, never on the laptop. → [13.09](../13-computer-vision/13.09-cnns-for-robot-vision.md), [16.04](../16-machine-learning/16.04-models-on-edge-compute.md)

**Interface package** — a ROS 2 package that contains only `.msg`/`.srv`/`.action` definitions, so
publishers and subscribers can depend on the types without depending on each other. → [04.06](../04-ros2/04.06-messages-and-custom-interfaces.md)

**Interrupt service routine** — see *Interrupt*. Keep it short, allocate nothing, and hand the work to
the main loop. → [FC.05](../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md)

## J

**Jacobian** — the matrix of partial derivatives of outputs with respect to inputs. For an arm it maps
joint velocities to end-effector twist; for a filter it is how nonlinearity gets linearized; for
covariance it is how uncertainty propagates. Learn it once, use it everywhere. → [14.06](../14-robotic-arm/14.06-jacobian.md),
[10.06](../10-localization/10.06-extended-kalman-filter.md), [09.06](../09-odometry/09.06-accumulated-error.md)

**Jitter** — variation in loop period. A loop that averages 50 Hz but occasionally takes 40 ms will
trip a 300 ms watchdog eventually, and it will ruin a derivative term long before that. → [03.04](../03-robot-software/03.04-timing-and-concurrency.md),
[FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md)

**Joint** — the connection between two links, with a type (revolute, continuous, prismatic, fixed), an
axis and limits. → [14.01](../14-robotic-arm/14.01-arm-anatomy.md), [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md)

**Joint trajectory controller** — the `ros2_control` controller that executes a timed sequence of joint
positions, and aborts with `PATH_TOLERANCE_VIOLATED` if the arm cannot keep up. → [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

**`journalctl`** — reads the systemd log. Where a service that "did not start" tells you why. →
[FL.06](../optional-foundations/linux-and-tools/FL.06-processes-systemd.md)

**JST-XH** — the balance connector on a lithium pack, and the small polarised connector you will use
for signals. → [02.07](../02-robot-electronics/02.07-wiring-harness.md)

## K

**Kalman filter** — the optimal recursive estimator for linear systems with Gaussian noise: predict,
then correct with a gain that weighs the measurement against the prediction by their covariances. →
[10.05](../10-localization/10.05-kalman-filter-multivariate.md), [10.04](../10-localization/10.04-kalman-filter-1d.md)

**Kalman gain** — how much of the innovation to apply. Large when the sensor is trusted relative to
the prediction, small when it is not. → [10.04](../10-localization/10.04-kalman-filter-1d.md)

**Keepout zone** — an area marked forbidden in a costmap filter, so the robot plans around it even
though nothing physical is there (stairs, the cat's bowl). → [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md)

**Kernel (image)** — the small matrix slid over an image in a convolution. → [FCV.02](../optional-foundations/computer-vision/FCV.02-convolution-kernels.md)

**Kidnapped robot** — the robot is picked up and put somewhere else without being told. The test of
whether your localizer can recover from confident wrongness. → [10.01](../10-localization/10.01-what-is-localization.md)

**Kinematic chain** — links connected by joints, from base to end effector. → [14.04](../14-robotic-arm/14.04-forward-kinematics.md)

**KV rating** — a brushless motor's RPM per volt, unloaded. → [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md)

## L

**Landmark** — a distinguishable feature with a known or estimated position, used to correct drift. →
[10.01](../10-localization/10.01-what-is-localization.md)

**LaserScan** — `sensor_msgs/LaserScan`: ranges on a regular angular grid, with a frame id and a
stamp. Converting it to Cartesian points is polar-to-Cartesian and nothing more. → [07.07](../07-sensors/07.07-2d-lidar.md)

**Latency** — the delay between a thing happening and your code acting on it. Sensor latency, network
latency and inference latency all add, and the sum is what destabilises a controller. → [07.01](../07-sensors/07.01-sensor-fundamentals.md),
[19.01](../19-llm-robot-agents/19.01-why-llms-dont-drive-motors.md)

**Launch file** — a Python (usually) description of which nodes to start, with which parameters,
remappings and arguments. Roughly `docker-compose` for ROS 2 processes. → [04.10](../04-ros2/04.10-launch-files.md)

**Least squares** — minimise the sum of squared residuals. Calibration, SLAM, hand-eye and system
identification are all this. → [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md)

**Level shifter** — a circuit that safely connects a 3.3 V pin to a 5 V device, in the direction(s)
you need. → [02.05](../02-robot-electronics/02.05-logic-levels-and-protection.md)

**Levenberg–Marquardt** — see the acronym table. → [FM.20](../optional-foundations/mathematics/FM.20-optimization-basics.md)

**Lifecycle node** — a ROS 2 node with managed states (unconfigured → inactive → active → finalized),
so a system can be brought up in a defined order. Closest analogue: `IHostedService` with explicit
configure/activate. Nav2 is built from them. → [04.15](../04-ros2/04.15-lifecycle-nodes.md)

**Likelihood** — the probability of the observation given a hypothesis; the term Bayes' rule
multiplies the prior by. → [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md)

**Likelihood field** — a precomputed distance-to-nearest-obstacle grid that makes scoring a particle's
scan fast and smooth. → [10.08](../10-localization/10.08-particle-filter-mcl.md)

**Link** — a rigid body in a robot model, with visual, collision and inertial properties. → [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md),
[14.01](../14-robotic-arm/14.01-arm-anatomy.md)

**Li-ion / LiPo** — the lithium chemistries used here. 3.0–4.2 V per cell, enormous short-circuit
current, and a real fire risk when abused. Never charge unattended. → [FE.13](../optional-foundations/electronics/FE.13-batteries.md), [FE.14](../optional-foundations/electronics/FE.14-lithium-battery-safety.md)

**Log-odds** — the representation occupancy grids use, because Bayesian updates become additions and
the value cannot saturate to exactly 0 or 1. → [11.02](../11-slam/11.02-occupancy-grid-mapping.md)

**Loop closure** — recognising that you are back somewhere you have been, and adding the constraint
that corrects the whole trajectory. The moment a SLAM map snaps into place — or folds, if the
association was wrong. → [11.05](../11-slam/11.05-pose-graphs-loop-closure.md)

**Low-pass filter** — attenuates fast components. An exponential moving average with one coefficient
is usually enough, and it costs you phase lag. → [08.03](../08-control/08.03-wheel-speed-estimation.md)

**LQR** — see the acronym table. → [FCT.06](../optional-foundations/control-theory/FCT.06-state-space-lqr-mpc.md)

**LeRobot dataset** — the standard episode format for robot demonstration data: observations, actions
and metadata per timestep, ready for imitation-learning training. → [18.03](../18-embodied-ai/18.03-teleop-data-collection.md)

**Lifelong mapping** — continuing to update a saved map on later runs instead of rebuilding it, and
the localization mode that uses one without changing it. → [11.07](../11-slam/11.07-mapping-your-room.md)

**Linking** — the step where the compiler's object files are joined; `undefined reference to …` is a
link error, not a compile error. → [FC.04](../optional-foundations/cpp-and-embedded/FC.04-cmake-builds.md)

## M

**Madgwick filter** — a light-weight orientation filter fusing gyro with accelerometer (and
magnetometer); what `imu_filter_madgwick` runs. → [07.06](../07-sensors/07.06-imu-in-ros2.md)

**Magnetometer** — measures the magnetic field, giving an absolute heading reference — until the
robot's own motor currents swamp it. → [07.05](../07-sensors/07.05-imu-fundamentals.md)

**Map → odom** — the transform the localizer publishes to correct accumulated odometry drift. It jumps;
`odom → base_link` does not. Controllers use odom, goals use map. → [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md), [10.09](../10-localization/10.09-amcl-in-ros2.md)

**Map server** — the Nav2 node that loads a saved `.pgm` + `.yaml` map and publishes it on a
transient-local topic. → [10.09](../10-localization/10.09-amcl-in-ros2.md)

**Markov assumption** — the future depends only on the current state, not the whole history. What
makes recursive filters and MDPs tractable. → [17.01](../17-reinforcement-learning/17.01-rl-framing.md), [10.03](../10-localization/10.03-bayes-filter.md)

**Mass vs weight** — mass is kilograms and is the same everywhere; weight is a force in newtons. The
distinction matters the moment you size a motor. → [FP.02](../optional-foundations/physics/FP.02-forces-newton.md)

**MCAP** — rosbag2's default storage format since Iron; self-describing, fast to index. → [04.14](../04-ros2/04.14-rosbag2.md)

**MCP** — see the acronym table. A standard tool-calling protocol; used here to expose validated robot
skills to an LLM agent instead of letting it emit raw commands. → [19.10](../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md)

**Measurement model** — p(observation | state): what the sensor would report if the robot were here.
→ [10.03](../10-localization/10.03-bayes-filter.md)

**Message filters** — the ROS 2 utility that synchronises messages from several topics by their header
stamps (exact or approximate time). → [07.10](../07-sensors/07.10-sensor-data-in-ros2.md)

**Middleware** — the layer that moves data between processes and machines so your nodes do not have to
know about sockets. In ROS 2, DDS. → [00.02](../00-orientation/00.02-the-robotics-stack.md)

**micro-ROS** — ROS 2 on a microcontroller, talking XRCE-DDS to an agent process that bridges it into
the normal ROS 2 graph. → [FC.08](../optional-foundations/cpp-and-embedded/FC.08-micro-ros.md)

**MicroPython** — the Python implementation that runs on the Pico. Fast enough for the protocol and
the control loop; not for arithmetic-heavy work. → [FC.02](../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)

**Moment of inertia** — how hard a body is to angularly accelerate about an axis. → [FP.06](../optional-foundations/physics/FP.06-rotational-motion-inertia.md)

**Monte Carlo localization** — particle-filter localization: represent the belief with weighted
samples, move them with the motion model, weight them by the measurement, resample. → [10.08](../10-localization/10.08-particle-filter-mcl.md)

**Morphology (image)** — erode, dilate, open, close: cheap operations that clean up binary masks. →
[13.02](../13-computer-vision/13.02-opencv-preprocessing.md)

**Motion model** — p(new state | old state, command): how the robot is expected to move, including how
uncertain that is. → [10.03](../10-localization/10.03-bayes-filter.md)

**Motor constant** — the proportionality between duty (or voltage) and steady-state speed, and between
current and torque. The basis of feedforward. → [08.09](../08-control/08.09-feedforward-exact-speed.md)

**MoveIt 2** — the ROS 2 motion-planning framework for arms: planning scene, collision checking,
planners, and trajectory execution. → [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md)

**MPPI** — see the acronym table. Sample many control sequences, weight them by cost, take the
weighted average. Nav2's smoothest local planner, and the most expensive. → [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md)

**Multivariate Gaussian** — a Gaussian over several variables, defined by a mean vector and a
covariance matrix; drawn as an ellipse. → [FM.15](../optional-foundations/mathematics/FM.15-covariance-multivariate-gaussians.md), [10.02](../10-localization/10.02-uncertainty-gaussians.md)

**Microstepping** — driving a stepper with intermediate current ratios so it moves in fractions of a
full step: smoother, quieter, and with less holding torque per microstep. → [FE.11](../optional-foundations/electronics/FE.11-servos-and-steppers.md)

**`mpremote`** — the tool that copies files to the Pico and opens its REPL over USB. → [FC.02](../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)

## N

**Nav2** — the ROS 2 navigation stack: costmaps, planner server, controller server, behavior server,
BT navigator and a lifecycle manager over all of it. → [12.06](../12-navigation/12.06-nav2-architecture.md)

**NEES** — see the acronym table. The test that catches an overconfident filter, which a plot never
will. → [10.05](../10-localization/10.05-kalman-filter-multivariate.md)

**Neural network** — layers of weighted sums and nonlinearities, fitted by gradient descent. →
[FML.06](../optional-foundations/machine-learning/FML.06-neural-networks.md)

**NMS** — see the acronym table. → [13.10](../13-computer-vision/13.10-object-detection-models.md)

**Node (ROS 2)** — a process (usually) that participates in the graph: publishes, subscribes, serves,
and holds parameters. → [04.03](../04-ros2/04.03-nodes-and-cli.md)

**Noise** — the random part of a sensor's error, as opposed to bias. Characterise it with a standard
deviation before you filter it. → [07.01](../07-sensors/07.01-sensor-fundamentals.md)

**Norm** — a vector's length. `‖v‖₂` is the ordinary Euclidean one. → [FM.05](../optional-foundations/mathematics/FM.05-vectors.md)

**Named location** — a human-readable label ("kitchen") bound to a map pose, so missions and the LLM
agent can talk about places instead of coordinates. → [12.09](../12-navigation/12.09-waypoints-and-missions.md), [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md)

**NiMH** — nickel-metal-hydride cells: heavier and lower-density than lithium, and far more forgiving.
→ [FE.13](../optional-foundations/electronics/FE.13-batteries.md)

## O

**Occupancy grid** — a 2D array of cells, each free / occupied / unknown (in ROS: 0–100 and −1), with
a resolution and an origin. The map format everything in Nav2 consumes. → [11.01](../11-slam/11.01-map-representations.md), [11.02](../11-slam/11.02-occupancy-grid-mapping.md)

**Odometry** — the estimate of pose from integrating wheel (and IMU) motion, published as
`nav_msgs/Odometry` plus the `odom → base_link` transform. Smooth, continuous, drifting. →
[09.04](../09-odometry/09.04-odometry-from-scratch.md), [09.07](../09-odometry/09.07-odometry-in-ros2.md)

**Ohm's law** — V = IR. Three quantities, one equation, and most electronics debugging is deciding
which two you can measure. → [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md)

**ONNX** — see the acronym table. → [16.04](../16-machine-learning/16.04-models-on-edge-compute.md)

**Open-drain** — an output that can pull low but not high, so a pull-up resistor sets the idle level
and several devices can share one line. The reason I²C needs pull-ups. → [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md)

**Open-loop control** — command without measuring the result. Works until the battery sags, the floor
changes or a motor differs from its twin. → [08.01](../08-control/08.01-open-vs-closed-loop.md)

**Open-vocabulary detection** — detecting objects named in free text at query time, rather than a
fixed class list, using image/text embeddings. → [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md)

**ORB** — see the acronym table. → [13.04](../13-computer-vision/13.04-features-and-matching.md)

**Orthonormal matrix** — rows and columns are unit length and mutually perpendicular; its inverse is
its transpose. Rotation matrices are orthonormal with determinant +1. → [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md)

**Oscillation** — sustained back-and-forth about the setpoint. Usually too much proportional gain, too
much loop delay, or a derivative term amplifying noise. → [08.07](../08-control/08.07-pid-mathematics.md)

**Outlier** — a measurement that does not come from the model you assumed. RANSAC and robust losses
exist to stop one from dominating a least-squares fit. → [FM.21](../optional-foundations/mathematics/FM.21-statistics-for-experiments.md)

**Overfitting** — fitting the training set's noise rather than its structure, so validation error rises
while training error falls. → [FML.03](../optional-foundations/machine-learning/FML.03-train-validation-test.md)

**Overshoot** — how far past the setpoint the response goes, as a percentage of the step. → [08.07](../08-control/08.07-pid-mathematics.md)

## P

**Parameter (ROS 2)** — a typed, declared, per-node configuration value, settable from YAML, the
command line or a launch file, and changeable at runtime with a callback. Roughly `IOptions<T>` with a
change notification. Undeclared parameters in your YAML are silently ignored — the classic "my value
is not applied". → [04.09](../04-ros2/04.09-parameters.md)

**Particle filter** — a Bayes filter whose belief is a set of weighted samples; handles multimodal
beliefs and nonlinear models, at the cost of particles. → [10.08](../10-localization/10.08-particle-filter-mcl.md)

**PCA** — see the acronym table. → [FML.11](../optional-foundations/machine-learning/FML.11-unsupervised-learning.md)

**PID** — see the acronym table. Proportional acts on present error, integral on accumulated error,
derivative on its rate of change. The course builds it experimentally, one term at a time. →
[08.04](../08-control/08.04-proportional-control.md), [08.05](../08-control/08.05-integral-control.md), [08.06](../08-control/08.06-derivative-control.md), [08.07](../08-control/08.07-pid-mathematics.md)

**Pinhole model** — the camera model where a 3D point projects through one point onto the image plane:
u = f·X/Z + c. Everything else (distortion) is a correction to it. → [13.05](../13-computer-vision/13.05-pinhole-camera-model.md)

**PIO** — see the acronym table. The RP2 state machines that count quadrature edges at full speed
without the CPU noticing. → [01.09](../01-first-robot/01.09-reading-encoders.md)

**Planning scene** — MoveIt's model of the world: the robot, attached objects and collision objects.
Stale scene, wrong plan. → [14.09](../14-robotic-arm/14.09-moveit2-motion-planning.md), [15.09](../15-manipulation/15.09-collision-avoidance-and-recovery.md)

**Point cloud** — a set of 3D points, usually `sensor_msgs/PointCloud2`, from a depth camera or a 3D
LiDAR. → [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md)

**Policy** — the mapping from observation to action. Hand-written, learned, or a mix. → [17.01](../17-reinforcement-learning/17.01-rl-framing.md)

**Policy gradient** — differentiate expected return with respect to the policy parameters and step
uphill. → [17.05](../17-reinforcement-learning/17.05-policy-gradients.md)

**Pose** — position plus orientation. In 2D, (x, y, θ); in 3D, a translation plus a quaternion. →
[05.02](../05-frames-and-transforms/05.02-pose-in-2d.md)

**Pose graph** — nodes are poses, edges are relative-transform constraints with covariances; optimizing
it is SLAM's back end. → [11.05](../11-slam/11.05-pose-graphs-loop-closure.md)

**Posterior** — the belief after incorporating the measurement. → [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md)

**Power** — P = VI, in watts. A power budget adds up every load's worst case and checks the supply can
deliver it. → [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md), [02.02](../02-robot-electronics/02.02-power-budget.md)

**PPO** — see the acronym table. → [17.06](../17-reinforcement-learning/17.06-actor-critic-ppo-sac.md)

**Pre-grasp pose** — a pose offset back along the approach direction from the grasp, so the final
motion is a straight, collision-free line. → [15.07](../15-manipulation/15.07-detect-and-approach.md)

**Precision / recall** — of the things you reported, how many were right (precision); of the things
there were, how many you found (recall). → [FCV.06](../optional-foundations/computer-vision/FCV.06-detection-metrics.md)

**Prior** — the belief before the measurement. → [FM.16](../optional-foundations/mathematics/FM.16-bayes-rule.md)

**Process noise (Q)** — how much you admit your motion model is wrong. Too small and the filter stops
listening to sensors; too large and it tracks noise. → [10.04](../10-localization/10.04-kalman-filter-1d.md)

**Prompt injection (physical)** — text in the robot's environment — a label, a sign, a screen — that
the vision-language stack reads and the agent then obeys. Defend below the model: allowlists,
parameter validation, geofences. → [19.09](../19-llm-robot-agents/19.09-agent-safety-boundaries.md)

**Pseudo-inverse** — the least-squares "inverse" of a non-square matrix, `A⁺`. Behind both
overdetermined calibration fits and redundant-arm IK. → [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md)

**Publisher / subscriber** — ROS 2's many-to-many, fire-and-forget messaging. Closest analogue: a topic
on a message bus, except discovery is peer-to-peer and delivery is governed by QoS. → [04.04](../04-ros2/04.04-topics-publishers-subscribers.md)

**Pull-up / pull-down resistor** — a resistor that gives a floating input a defined idle level. I²C
needs pull-ups on both lines; a button needs one or the other. → [FE.06](../optional-foundations/electronics/FE.06-pull-up-pull-down.md)

**Pure pursuit** — a path follower that steers toward a look-ahead point on the path. Simple, stable,
and it cuts corners. → [12.05](../12-navigation/12.05-local-planning-obstacle-avoidance.md)

**PWM** — see the acronym table. Duty cycle sets average voltage; frequency sets whether the motor
whines audibly and how the driver heats. → [FE.07](../optional-foundations/electronics/FE.07-pwm.md), [02.03](../02-robot-electronics/02.03-motor-drivers-in-depth.md)

**Perfboard** — the soldered, permanent successor to the breadboard. Lay it out so you can still
probe every net. → [02.08](../02-robot-electronics/02.08-from-breadboard-to-perfboard.md)

**PLA / PETG / ASA / TPU** — the printing filaments: PLA for prototypes, PETG for parts that live in a
car in an Israeli summer, ASA for outdoors, TPU for flexible fingers and bumpers. → [F3D.07](../optional-foundations/3d-printing-and-cad/F3D.07-materials-chassis-ordering.md)

**Plane segmentation** — fitting and removing the dominant plane (the table, the floor) from a point
cloud so the objects on it stand out. → [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md), [15.04](../15-manipulation/15.04-object-pose-estimation.md)

**Port (network)** — the number that picks a service on a host; `Connection refused` means nothing is
listening on it. → [FL.09](../optional-foundations/linux-and-tools/FL.09-networking-basics.md)

**Product of exponentials** — an alternative to Denavit–Hartenberg for forward kinematics, built from
screw axes; often easier to set up correctly. → [14.04](../14-robotic-arm/14.04-forward-kinematics.md)

## Q

**Q-learning** — learn the action-value function off-policy from the Bellman optimality equation. →
[17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md)

**QoS** — see the acronym table. Reliability (reliable/best-effort), durability (volatile/transient
local), history and depth. A reliable subscriber will not connect to a best-effort publisher, and the
symptom is silence with everything apparently connected. Sensor data is best-effort; maps and robot
descriptions are transient local. → [04.11](../04-ros2/04.11-qos.md)

**Quadrature** — two encoder channels 90° out of phase, so the order of edges gives direction as well
as count. Counting both edges of both channels gives 4× resolution. → [01.09](../01-first-robot/01.09-reading-encoders.md)

**Quantization** — running a network in 8-bit (or lower) precision to make it fit and run on the
robot, trading a little accuracy for a lot of speed. → [16.04](../16-machine-learning/16.04-models-on-edge-compute.md)

**Quaternion** — a four-number representation of 3D rotation: no gimbal lock, cheap to compose, easy
to interpolate, and impossible to read by eye. ROS stores all orientations this way. → [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md),
[05.05](../05-frames-and-transforms/05.05-rotations-in-3d.md)

## R

**RAG** — see the acronym table. For a robot, the retrieved context is its own semantic memory: where
it has seen objects and rooms. → [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md)

**RANSAC** — see the acronym table. Fit from minimal random samples, count inliers, keep the best. How
you find the table plane in a cloud full of objects. → [13.04](../13-computer-vision/13.04-features-and-matching.md), [13.08](../13-computer-vision/13.08-depth-and-3d-vision.md)

**Rate (loop rate)** — how often a control loop runs. Choose it from the dynamics (well above the
system's bandwidth) and then actually measure whether you achieve it. → [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md), [03.04](../03-robot-software/03.04-timing-and-concurrency.md)

**`rclpy` / `rclcpp`** — the Python and C++ client libraries for ROS 2. → [04.04](../04-ros2/04.04-topics-publishers-subscribers.md)

**Real time** — meeting deadlines, not going fast. A missed deadline is a failure even if the average
is fine. Linux on a Pi is not real time; the Pico is. → [FC.07](../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md), [00.04](../00-orientation/00.04-robot-computers.md)

**Reality gap** — the difference between simulation and the robot, and the reason "it worked in
simulation" is a statement about your model, not your robot. → [06.10](../06-simulation/06.10-sim-to-real-gap.md)

**Recovery behavior** — what Nav2 does when it is stuck: clear the costmaps, spin, back up, wait. If
recoveries run constantly, something upstream is wrong. → [12.10](../12-navigation/12.10-recovery-and-navigation-safety.md)

**Remapping** — renaming a node's topics/services at launch, so a generic node fits your graph without
code changes. → [04.10](../04-ros2/04.10-launch-files.md)

**REP-103 / REP-105** — the two ROS standards this course obeys: SI units with x-forward/y-left/z-up
axes, and the `map → odom → base_link` frame convention. → [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)

**Replay buffer** — the store of past transitions an off-policy algorithm samples from. → [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md)

**Reprojection error** — the pixel distance between where a 3D point lands and where it was observed;
the number camera calibration minimises and the number that tells you whether to trust the result. →
[13.06](../13-computer-vision/13.06-lens-distortion-calibration.md)

**Resampling** — drawing a new particle set in proportion to weights, so particles concentrate where
the belief is. Do it too often and you lose diversity. → [10.08](../10-localization/10.08-particle-filter-mcl.md)

**Residual** — observed minus predicted. The thing every least-squares problem squares and sums. →
[FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md)

**Resolved-rate control** — commanding joint velocities as J⁻¹ times a desired end-effector twist.
Straight-line Cartesian jogging, and it explodes at singularities unless damped. → [14.06](../14-robotic-arm/14.06-jacobian.md)

**Reward hacking** — the policy maximises the reward you wrote instead of the behaviour you meant. →
[17.08](../17-reinforcement-learning/17.08-reward-design.md)

**Reward shaping** — adding intermediate reward to make a sparse problem learnable, at the risk of
teaching the shaping instead of the task. → [17.08](../17-reinforcement-learning/17.08-reward-design.md)

**RMW** — see the acronym table. → [04.01](../04-ros2/04.01-why-middleware.md)

**Robot** — a machine that senses, decides and acts in the physical world, in a closed loop with
physics. Not a program with motors attached. → [00.01](../00-orientation/00.01-what-is-a-robot.md)

**`robot_state_publisher`** — reads the URDF and `/joint_states` and publishes the whole TF tree of the
robot's links. If your movable joints are missing from `/tf`, this is what is not running. →
[05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md)

**`ROS_DOMAIN_ID`** — the integer that partitions DDS discovery. Set it per project/machine, or your
robot will hear somebody else's `cmd_vel`. → [04.02](../04-ros2/04.02-installing-ros2.md)

**Rotation matrix** — a 3×3 orthonormal matrix with determinant +1 that rotates vectors. Composes by
multiplication; its inverse is its transpose. → [FM.10](../optional-foundations/mathematics/FM.10-rotation-matrices.md)

**RRT / RRT\*** — sampling-based planners. RRT finds a path quickly and it is ugly; RRT* keeps
improving it toward optimal. → [12.03](../12-navigation/12.03-sampling-based-planning.md)

**Runge–Kutta 2 (midpoint)** — integrate using the derivative at the midpoint of the step; second-order
accurate, which is why halving dt quarters the error. → [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [FM.18](../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md)

**RViz 2** — the ROS 2 3D visualiser. It shows you what the robot believes, not what is true, which is
exactly why it is the best debugging tool you have. → [05.09](../05-frames-and-transforms/05.09-rviz.md)

**RS-485** — the differential, multi-drop electrical standard under many smart-servo buses. Half
duplex: only one device may talk at a time, and the last node needs termination. → [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md),
[FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md)

## S

**Sampling rate** — how often a sensor is read. Must exceed twice the highest frequency you care
about, and in practice far more. → [07.01](../07-sensors/07.01-sensor-fundamentals.md), [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md)

**Scan matching** — estimating the transform between two laser scans (or a scan and a map) by aligning
them. SLAM's front end. → [11.04](../11-slam/11.04-scan-matching-icp.md)

**SDF** — see the acronym table. Gazebo's format for worlds and models. → [06.03](../06-simulation/06.03-gazebo-basics.md)

**Semantic map** — a map annotated with what things are and where, not just whether a cell is
occupied. What an LLM agent needs in order to act on "put it on the kitchen table". → [19.08](../19-llm-robot-agents/19.08-memory-and-world-model.md),
[13.16](../13-computer-vision/13.16-detection-to-map-position.md)

**Semantic segmentation** — a class label for every pixel. → [13.11](../13-computer-vision/13.11-segmentation.md)

**Sense–think–act** — the loop every robot runs: read sensors, decide, command actuators, repeat. The
rates differ per layer — 1 kHz at the motor, 0.1 Hz at the language planner. → [00.01](../00-orientation/00.01-what-is-a-robot.md)

**Sensor fusion** — combining several sensors so the estimate is better than any one of them, weighted
by how much each is trusted. → [10.07](../10-localization/10.07-fusing-imu-and-odometry.md)

**Service (ROS 2)** — a request/response call between nodes. An RPC, and like every RPC it can time out
after the server already did the work. Never call one synchronously from inside a callback in a
single-threaded executor. → [04.07](../04-ros2/04.07-services.md)

**Servo (motor)** — a motor with integrated position feedback and control. Hobby servos take a PWM
pulse width; smart/bus servos take addressed packets on a serial bus and report back. → [FE.11](../optional-foundations/electronics/FE.11-servos-and-steppers.md),
[14.03](../14-robotic-arm/14.03-controlling-servos.md)

**Setpoint** — the value the controller is trying to make the measurement equal. → [08.01](../08-control/08.01-open-vs-closed-loop.md)

**Settling time** — how long the response takes to stay within a band (commonly ±2 %) of the setpoint.
→ [08.07](../08-control/08.07-pid-mathematics.md)

**Singularity** — an arm configuration where the Jacobian loses rank, so some end-effector direction
needs infinite joint speed. Damp it, or plan around it. → [14.06](../14-robotic-arm/14.06-jacobian.md)

**Sim-to-real** — transferring something developed in simulation to the robot, and the discipline of
measuring the gap rather than hoping. → [06.10](../06-simulation/06.10-sim-to-real-gap.md), [17.09](../17-reinforcement-learning/17.09-rl-sim-to-real.md)

**SLAM** — see the acronym table. A front end (scan matching, loop-closure detection) plus a back end
(pose-graph optimization). → [11.03](../11-slam/11.03-the-slam-problem.md)

**`slam_toolbox`** — the ROS 2 SLAM package the course uses: online async mapping, serialization, and a
localization mode against a previously built map. → [11.06](../11-slam/11.06-slam-toolbox-simulation.md)

**SLERP** — spherical linear interpolation between quaternions: constant angular rate, shortest path.
→ [FM.12](../optional-foundations/mathematics/FM.12-quaternions.md)

**Smart servo bus** — a daisy chain of addressed servos on a half-duplex serial line (often RS-485).
One id clash or one bad connector and the chain stops at that point. → [02.10](../02-robot-electronics/02.10-can-bus-and-smart-actuators.md)

**Specular reflection** — a smooth surface bouncing the ultrasonic ping or the laser away from the
sensor, so the obstacle is invisible. Why ultrasonic sensors miss angled walls and LiDAR misses glass.
→ [07.03](../07-sensors/07.03-ultrasonic-sensors.md), [07.07](../07-sensors/07.07-2d-lidar.md)

**Stall current** — the current a motor draws at zero speed with full voltage, several times its
running current. The number your fuse, driver and battery must survive. → [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md), [02.02](../02-robot-electronics/02.02-power-budget.md)

**Star ground** — one physical point where all grounds meet, so no return current shares a path with a
signal. → [02.06](../02-robot-electronics/02.06-noise-grounding-emi.md)

**State (RL / estimation)** — the variables that summarise everything relevant about the situation. →
[17.01](../17-reinforcement-learning/17.01-rl-framing.md), [FCT.03](../optional-foundations/control-theory/FCT.03-differential-equations-simulation.md)

**Static transform** — a fixed relationship between frames (sensor mount offsets), published once on a
latched topic instead of continuously. → [05.07](../05-frames-and-transforms/05.07-tf2-broadcast-listen.md)

**Steady-state error** — the error that remains after everything settles. Proportional control alone
leaves one; integral action removes it. → [08.04](../08-control/08.04-proportional-control.md)

**Step response** — the output when the input jumps. Reading τ and the final value off it is how the
course identifies karmel's motor model. → [08.02](../08-control/08.02-motor-step-response.md)

**Stereo depth** — depth from the disparity between two cameras a known baseline apart. Fails on
texture-free surfaces. → [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md), [FCV.05](../optional-foundations/computer-vision/FCV.05-stereo-epipolar.md)

**Structured light** — projecting a known pattern and reading its deformation to get depth. Good
indoors, defeated by sunlight. → [07.09](../07-sensors/07.09-depth-cameras-and-point-clouds.md)

**Support polygon** — the convex hull of the ground contact points. The robot tips when the centre of
mass leaves it. → [FP.07](../optional-foundations/physics/FP.07-center-of-mass-stability.md)

**SVD** — see the acronym table. → [FM.19](../optional-foundations/mathematics/FM.19-linear-systems-least-squares.md)

**System identification** — measuring a real system's response and fitting model parameters to it,
rather than trusting the datasheet. → [16.05](../16-machine-learning/16.05-learning-dynamics.md), [06.10](../06-simulation/06.10-sim-to-real-gap.md)

**`systemd`** — how a Linux service is started at boot, restarted on failure and logged. How karmel
brings itself up without a laptop. → [FL.06](../optional-foundations/linux-and-tools/FL.06-processes-systemd.md), [20.03](../20-final-robot/20.03-software-integration-bringup.md)

**`scp` / `rsync`** — copying files to and from the robot; `rsync` only sends what changed, which
matters over Wi-Fi with a bag file. → [FL.08](../optional-foundations/linux-and-tools/FL.08-ssh-remote-dev.md)

**Slicer** — the program that turns a 3D model into printer instructions: layer height, walls, infill,
supports. → [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md)

**Snap** — Ubuntu's containerised package format. Worth knowing because a snap-installed tool has a
different filesystem view from an apt-installed one. → [FL.07](../optional-foundations/linux-and-tools/FL.07-package-management.md)

**SSH key** — public-key authentication for logging into the robot without a password. Generate it
once, copy the public half, and disable password login. → [FL.08](../optional-foundations/linux-and-tools/FL.08-ssh-remote-dev.md)

**STL / STEP / 3MF** — the print file formats: STL is a dumb triangle mesh, STEP keeps real geometry
for further CAD, 3MF carries units and metadata. → [F3D.03](../optional-foundations/3d-printing-and-cad/F3D.03-file-formats-slicers.md)

**Strain relief** — anchoring a cable so movement pulls on the anchor and not on the solder joint. The
single cheapest reliability improvement on a moving robot. → [02.07](../02-robot-electronics/02.07-wiring-harness.md)

**Symlink** — a Linux pointer to another path. `/dev/karmel` is one; so is `colcon build
--symlink-install`, which is why editing a Python file can take effect without rebuilding. →
[FL.04](../optional-foundations/linux-and-tools/FL.04-filesystem.md), [04.05](../04-ros2/04.05-packages-and-workspaces.md)

## T

**Target network** — a slowly updated copy of the Q-network used to compute targets, which stops DQN
from chasing its own tail. → [17.04](../17-reinforcement-learning/17.04-deep-q-networks.md)

**Temporal difference** — learn from the gap between successive value estimates rather than waiting for
the episode's return. → [17.03](../17-reinforcement-learning/17.03-q-learning-gridworld.md)

**Temporal ensembling** — averaging overlapping predicted action chunks so the commanded trajectory is
smooth. → [18.04](../18-embodied-ai/18.04-action-chunking-transformers.md)

**tf2** — see the acronym table. A time-indexed tree of transforms, so any node can ask "where was this
point in that frame at that moment". Every frame has exactly one parent, and asking about a time the
buffer does not cover is the most common error in this course. → [05.07](../05-frames-and-transforms/05.07-tf2-broadcast-listen.md), [05.11](../05-frames-and-transforms/05.11-debugging-tf.md)

**Thresholding** — turning a grayscale image into a binary mask at a cut-off (fixed, Otsu, or
adaptive). → [13.02](../13-computer-vision/13.02-opencv-preprocessing.md)

**Time constant (τ)** — the time a first-order system takes to cover 63 % of a step. karmel's wheel
speed is about 0.08 s. → [08.02](../08-control/08.02-motor-step-response.md)

**Time of flight** — see the acronym table. Distance from the round-trip time of a light pulse.
Accurate, small, and affected by ambient light and surface reflectivity. → [07.04](../07-sensors/07.04-tof-sensors.md)

**Topic** — a named, typed channel in the ROS 2 graph. → [04.04](../04-ros2/04.04-topics-publishers-subscribers.md)

**Topological map** — places and the connections between them, rather than a metric grid. → [11.01](../11-slam/11.01-map-representations.md)

**Torque** — the rotational equivalent of force, N·m. Motor torque times gear ratio divided by wheel
radius is the force at the ground. → [FP.04](../optional-foundations/physics/FP.04-torque-gears-wheels.md)

**Traction** — the friction available between wheel and floor; exceed it and the wheels slip, and
encoders lie. → [FP.03](../optional-foundations/physics/FP.03-friction-traction.md), [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md)

**Trajectory** — a path with timing: positions, velocities and accelerations as functions of time, as
opposed to a path, which is only geometry. → [14.07](../14-robotic-arm/14.07-trajectories.md)

**Transformer** — the attention-based architecture behind modern vision and language models, and the
backbone of ACT and the VLAs. → [FML.10](../optional-foundations/machine-learning/FML.10-transformers-attention.md)

**Transient local** — the QoS durability that makes a publisher keep the last message for subscribers
that join later. Maps and robot descriptions use it; that is why a late-starting node can still get
them. → [04.11](../04-ros2/04.11-qos.md)

**Trapezoidal profile** — accelerate, cruise, decelerate. The simplest motion profile that respects
velocity and acceleration limits. → [08.11](../08-control/08.11-rotate-exactly-90.md)

**Twist** — `geometry_msgs/Twist`: linear and angular velocity. For karmel only `linear.x` and
`angular.z` are meaningful. → [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md)

**TensorRT** — NVIDIA's inference optimiser; the reason a Jetson runs a model several times faster
than plain PyTorch does. → [16.04](../16-machine-learning/16.04-models-on-edge-compute.md)

**Termination (bus)** — the resistors at the physical ends of a CAN or RS-485 bus that stop signal
reflections. Missing termination shows up as errors that grow with cable length and motion. →
[FE.19](../optional-foundations/electronics/FE.19-can-bus-concepts.md)

**Test pyramid (for robots)** — many pure unit tests with fakes, fewer simulation tests, a few
hardware-in-the-loop tests, and a written manual procedure at the top. → [03.08](../03-robot-software/03.08-testing-robot-software.md)

**Tinning** — pre-wetting a wire or an iron tip with solder, so the real joint takes a second. →
[FE.17](../optional-foundations/electronics/FE.17-soldering.md)

**`tmux`** — a terminal multiplexer: your processes survive a dropped SSH connection. Essential for
headless work. → [FL.13](../optional-foundations/linux-and-tools/FL.13-tmux-headless.md)

## U

**UART** — see the acronym table. → [FE.09](../optional-foundations/electronics/FE.09-uart-i2c-spi.md)

**udev rule** — a Linux rule that gives a USB device a stable name (`/dev/karmel`) and the right
permissions, so your code stops guessing at `ttyACM0` vs `ttyACM1`. → [03.03](../03-robot-software/03.03-robust-serial-communication.md), [FL.05](../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md)

**UMBmark** — the standard benchmark for differential-drive odometry calibration: drive a square
clockwise and counter-clockwise and separate the systematic errors from the random ones. → [09.05](../09-odometry/09.05-calibrating-odometry.md)

**Uncertainty** — what you do not know, quantified. In this course that means a variance, a covariance
matrix, or a particle cloud, never a vague feeling. → [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md), [10.02](../10-localization/10.02-uncertainty-gaussians.md)

**Undistortion** — removing lens distortion using the calibrated coefficients, so straight lines in the
world are straight in the image. → [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md)

**URDF** — see the acronym table. XML describing links, joints, visuals, collisions and inertias;
written with `xacro` macros so it is not 2000 lines of repetition. → [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md)

**`use_sim_time`** — the parameter that makes a node take time from `/clock` instead of the wall clock.
Set it on every node when simulating — mixing the two produces tf2 extrapolation errors that look like
a frames bug. → [06.04](../06-simulation/06.04-bridging-gazebo-ros2.md), [06.09](../06-simulation/06.09-same-code-sim-and-real.md)

**`uv`** — the fast Python package manager and lockfile tool the course uses for reproducible
laptop-side environments. → [FL.10](../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md), [03.02](../03-robot-software/03.02-environments-and-pinning.md)

## V

**Value function** — expected return from a state (V) or from a state-action pair (Q). → [17.02](../17-reinforcement-learning/17.02-value-exploration.md)

**Variance** — the square of the standard deviation; the scalar version of covariance. → [FM.14](../optional-foundations/mathematics/FM.14-distributions-gaussians-noise.md)

**VAE** — see the acronym table. → [FML.12](../optional-foundations/machine-learning/FML.12-generative-models-diffusion.md)

**Vector** — magnitude and direction; in robotics, a velocity, a force, a position offset. →
[FM.05](../optional-foundations/mathematics/FM.05-vectors.md)

**Velocity smoother** — the Nav2 component that limits acceleration and jerk on the way to the base, so
a jumpy controller output does not become a jumpy robot. → [12.08](../12-navigation/12.08-nav2-on-the-real-robot.md)

**venv** — an isolated Python environment. On Ubuntu 24.04 (PEP 668) it is not optional for non-apt
packages, and it interacts badly with `colcon` unless you know the rules. → [03.02](../03-robot-software/03.02-environments-and-pinning.md), [FL.10](../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)

**Visual odometry** — estimating motion from camera images alone by tracking features between frames.
→ [11.08](../11-slam/11.08-visual-slam.md)

**Visual servoing** — closing the control loop directly on visual error, either in image space (IBVS)
or in Cartesian space after pose estimation (PBVS). → [15.06](../15-manipulation/15.06-visual-servoing.md)

**VLA** — see the acronym table. Images plus an instruction in, robot actions out, from a single
pretrained model. → [18.07](../18-embodied-ai/18.07-vision-language-action-models.md)

**VLM** — see the acronym table. → [13.14](../13-computer-vision/13.14-vision-language-models.md)

**Voltage divider** — two resistors that scale a voltage down, e.g. so a battery's 12 V fits an ADC's
3.3 V range. → [FE.02](../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md), [01.14](../01-first-robot/01.14-battery-monitoring.md)

**Voxel map** — a 3D occupancy grid of cubes. Needed the moment obstacles have height (a table top
that a 2D scan passes under). → [11.01](../11-slam/11.01-map-representations.md)

## W

**Wake word** — the phrase that starts the voice pipeline, so the robot is not streaming audio
continuously. → [19.11](../19-llm-robot-agents/19.11-voice-interface.md)

**Watchdog** — a timer that fires if it is not fed, stopping the actuators. karmel has them at every
layer: the Pico's 300 ms command timeout, the ROS base node's `cmd_vel` timeout, teleop's dead-man
key. Never remove one "temporarily". → [01.10](../01-first-robot/01.10-pi-pico-protocol.md), [20.04](../20-final-robot/20.04-safety-case.md)

**Waypoint following** — visiting a sequence of goals, deciding for each what "reached" and "failed"
mean. → [12.09](../12-navigation/12.09-waypoints-and-missions.md)

**Wheel separation (track width)** — the distance between the two drive wheels' contact points; the
parameter that converts a wheel-speed difference into an angular velocity. karmel's is 0.200 m, and
the effective value differs from the measured one — calibrate it. → [09.01](../09-odometry/09.01-diff-drive-kinematics.md), [09.05](../09-odometry/09.05-calibrating-odometry.md)

**Wheel slip** — the wheel turns and the robot does not move correspondingly. Encoders cannot see it,
so it is a pure, silent odometry error. → [07.02](../07-sensors/07.02-wheel-encoders-in-depth.md), [FP.03](../optional-foundations/physics/FP.03-friction-traction.md)

**Wire gauge (AWG)** — wire thickness. Thinner is a higher number and a higher resistance; motor and
battery wiring needs thicker wire than signals. → [02.07](../02-robot-electronics/02.07-wiring-harness.md)

**World frame** — the fixed frame everything else is eventually expressed in; in ROS practice, `map`.
→ [05.01](../05-frames-and-transforms/05.01-why-frames.md), [05.06](../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)

**World model (learned)** — a learned predictor of what happens next, which a policy can plan against
without touching the real robot. → [18.09](../18-embodied-ai/18.09-foundation-models-world-models.md)

**Workspace (arm)** — the set of poses the end effector can actually reach. Points outside it make IK
"fail" for no visible reason. → [14.01](../14-robotic-arm/14.01-arm-anatomy.md), [14.11](../14-robotic-arm/14.11-arm-safety.md)

**Workspace overlay** — sourcing one `colcon` workspace on top of another, so your package shadows the
installed one. When "my code change has no effect", this is usually why. → [04.05](../04-ros2/04.05-packages-and-workspaces.md)

**WSL2** — Windows Subsystem for Linux, one of the three supported ways to run Ubuntu 24.04 for this
course (the others: a VM, or a real install). USB passthrough for the Pico needs extra work. →
[FL.02](../optional-foundations/linux-and-tools/FL.02-installing-ubuntu.md)

## X, Y, Z

**`xacro`** — the XML macro preprocessor that turns parameterised, reusable URDF fragments into a full
URDF at launch time. → [05.08](../05-frames-and-transforms/05.08-urdf-and-xacro.md)

**X11 / WSLg** — how a Linux GUI program (RViz, Gazebo) gets a window when it runs in a container or in
WSL. `could not connect to display` is this and nothing else. → [FL.12](../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)

**XRCE-DDS** — see the acronym table. → [FC.08](../optional-foundations/cpp-and-embedded/FC.08-micro-ros.md)

**XT30 / XT60** — the keyed, high-current power connectors used for the battery. Never use Dupont
jumpers for motor power. → [02.07](../02-robot-electronics/02.07-wiring-harness.md)

**Yaw** — rotation about z, the robot's heading in the plane. → [FM.11](../optional-foundations/mathematics/FM.11-euler-angles.md), [05.02](../05-frames-and-transforms/05.02-pose-in-2d.md)

**YOLO** — the real-time single-stage object detector family used in the vision module; check the
licence of the specific weights before shipping anything. → [13.10](../13-computer-vision/13.10-object-detection-models.md)

**Zero-order hold** — holding a digital command constant between samples, which is what a discrete
controller actually does to a continuous plant. → [FCT.05](../optional-foundations/control-theory/FCT.05-discrete-time-sampling.md)

**Zero-shot detection** — detecting a class the detector was never trained on, by matching text and
image embeddings. → [13.13](../13-computer-vision/13.13-embeddings-open-vocabulary.md)

**Ziegler–Nichols** — a classical PID tuning recipe based on the gain and period at which the loop just
oscillates. A starting point, not an answer; this course prefers tuning by logging. → [08.08](../08-control/08.08-tuning-pid.md)

---

## See also

- [`curriculum/concept-index.md`](../curriculum/concept-index.md) — the complete concept → lesson index.
- [troubleshooting/](troubleshooting/README.md) — symptom-driven debugging guides that cut across lessons.
- [resources.md](resources.md) — where to read more, by topic.
- [papers.md](papers.md) — the papers behind the algorithms named here.
- [SAFETY.md](../SAFETY.md) — the ten standing rules.
