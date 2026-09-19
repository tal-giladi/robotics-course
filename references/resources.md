# Learning resources by topic

> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09-16). Every URL below was checked on that date. The
> evidence and the check method are in
> [`research/learning-resources-2026-09.md`](research/learning-resources-2026-09.md),
> [`research/software-versions-2026-09.md`](research/software-versions-2026-09.md) and
> [`research/embodied-ai-landscape-2026-09.md`](research/embodied-ai-landscape-2026-09.md).
> Documentation sites move (docs.ros.org, Nav2 and micro-ROS all moved in 2025–2026). If a link
> is broken, check [Moved and dead links](#moved-and-dead-links) first, then the project's home page.
> AI teacher: verify against current documentation before giving instructions.

This file lists authoritative resources only. Within each topic they are ordered **official docs →
papers → university courses → high-quality tutorials → books**. The papers are listed in full, with
reading guidance, in [papers.md](papers.md). Lessons pick their "Go deeper" links from this file.

**Columns:** *Cost* is Free, Paid, or Free/Paid (for example free to audit, paid certificate).
*Level* is Beg(inner), Int(ermediate) or Adv(anced), measured against robotics knowledge, not
programming. *Modules* are the course modules each resource supports (see the key below).

<details><summary>Module key</summary>

| Id | Module | Id | Foundation track |
|---|---|---|---|
| 00 | Orientation | FM | Mathematics |
| 01 | Your first physical robot | FE | Electronics |
| 02 | Robot electronics in practice | FP | Physics |
| 03 | Software engineering for robots | FL | Linux and tools |
| 04 | ROS 2 | FPY | Python for C# developers |
| 05 | Coordinate frames and transformations | FC | Embedded systems and C/C++ |
| 06 | Simulation | FCT | Control theory |
| 07 | Sensors | FML | Machine learning |
| 08 | Control | FCV | Computer vision |
| 09 | Odometry | F3D | 3D printing and CAD |
| 10 | Localization | | |
| 11 | Mapping and SLAM | | |
| 12 | Navigation | | |
| 13 | Computer vision | | |
| 14 | The robotic arm | | |
| 15 | Manipulation | | |
| 16 | Machine learning for robots | | |
| 17 | Reinforcement learning | | |
| 18 | Modern embodied AI | | |
| 19 | LLMs and robot agents | | |
| 20 | The final robot | | |

The full map is in [COURSE_MAP.md](../COURSE_MAP.md).
</details>

## Contents

1. [Robotics (general)](#1-robotics-general)
2. [ROS 2 and robot middleware](#2-ros-2-and-robot-middleware)
3. [Simulation](#3-simulation)
4. [Mathematics](#4-mathematics)
5. [Electronics and embedded systems](#5-electronics-and-embedded-systems)
6. [Control](#6-control)
7. [Sensors, state estimation and SLAM](#7-sensors-state-estimation-and-slam)
8. [Navigation and planning](#8-navigation-and-planning)
9. [Arms and manipulation](#9-arms-and-manipulation)
10. [Computer vision](#10-computer-vision)
11. [Machine learning and deep learning](#11-machine-learning-and-deep-learning)
12. [Reinforcement learning](#12-reinforcement-learning)
13. [Embodied AI and robot learning](#13-embodied-ai-and-robot-learning)
14. [LLM agents for robots](#14-llm-agents-for-robots)
15. [Linux, Python and developer tools](#15-linux-python-and-developer-tools)
16. [Safety](#16-safety)
17. [3D printing and CAD](#17-3d-printing-and-cad)
18. [Fast-moving technologies: verify before use](#fast-moving-technologies-verify-before-use)
19. [Moved and dead links](#moved-and-dead-links)

---

## 1. Robotics (general)

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| REP-103 Standard Units of Measure and Coordinate Conventions | https://www.ros.org/reps/rep-0103.html | standard | Free | Beg | SI units and right-handed axes (x forward, y left, z up) | 05, all |
| MIT 6.4210/6.4212 Robotic Manipulation (Tedrake) | https://manipulation.csail.mit.edu/ | university course notes | Free | Int–Adv | Perception-to-grasp pipelines with Drake and runnable notebooks | 14, 15, 18 |
| MIT Underactuated Robotics (Tedrake) | https://underactuated.csail.mit.edu/ | university course notes + videos | Free | Adv | Dynamics, trajectory optimization, LQR/MPC, legged systems | 08, FCT, 17 |
| Modern Robotics, Coursera specialization (6 courses) | https://www.coursera.org/specializations/modernrobotics | university course | Free/Paid (audit free, certificate paid) | Int | Structured path through the Modern Robotics book, with quizzes and a capstone | 05, 14, 15 |
| ETH Autonomous Mobile Robots (ASL) | https://asl.ethz.ch/education/lectures/autonomous_mobile_robots.html | university course | Free (course info) | Int | Companion lecture to the Siegwart book | 07, 09–12 |
| QUT Robot Academy (Peter Corke) | https://robotacademy.net.au/ | video course | Free | Beg–Int | Short university-level videos: spatial math, kinematics, vision | 05, 13, 14 |
| Duckietown | https://duckietown.com/ | platform + docs | Free docs; hardware Paid | Beg–Int | A complete autonomy curriculum on small robots. Docs: https://docs.duckietown.com/ | 07–13 |
| Articulated Robotics (Josh Newans) | https://articulatedrobotics.xyz/tutorials/ | tutorial + video | Free | Beg | The best free story of building a real diff-drive ROS 2 robot. **Caveat:** made for Ubuntu 20.04, ROS 2 Foxy and Gazebo Classic. Use it for concepts and the official docs for commands. | 04–06, 11, 12 |
| The Construct | https://www.theconstruct.ai/ | platform | Paid (some free content) | Beg–Int | ROS/Gazebo sandboxes in the browser. Optional; everything here is available free elsewhere. | 04, 06 |
| Modern Robotics (Lynch & Park), book home | https://hades.mech.northwestern.edu/index.php/Modern_Robotics | book + videos | Free preprint PDF | Int | The best single textbook for arm kinematics, dynamics and mobile manipulation. PDF: https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf · code: https://github.com/NxRLab/ModernRobotics | 05, 09, 14, 15 |
| Robotics, Vision and Control, 3rd ed. (Corke) | https://petercorke.com/rvc/ | book | Paid (Springer) | Beg–Int | A hands-on bridge from math to code, with the Robotics Toolbox for Python: https://github.com/petercorke/robotics-toolbox-python | 05, 13, 14 |
| Introduction to Autonomous Mobile Robots, 2nd ed. (Siegwart, Nourbakhsh, Scaramuzza) | https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/ | book | Paid | Int | Locomotion, sensors, localization, planning for wheeled robots | 07, 09–12 |

## 2. ROS 2 and robot middleware

The course pins **ROS 2 Jazzy Jalisco on Ubuntu 24.04** (see
[`curriculum/versions.yaml`](../curriculum/versions.yaml)). The Jazzy docs keep the classic
`Tutorials/` layout. The Lyrical and Rolling docs were restructured, and their old `Tutorials/`
paths return 404.

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| ROS 2 Jazzy installation (Ubuntu, deb packages) | https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html | official docs | Free | Beg | The install you will actually do | 04 |
| ROS 2 distributions page | https://docs.ros.org/en/jazzy/Releases.html | official docs | Free | Beg | Choosing a distro; EOL dates | 04 |
| ROS 2 Jazzy tutorials index | https://docs.ros.org/en/jazzy/Tutorials.html | official docs | Free | Beg–Int | The canonical hands-on path | 04 |
| Beginner: CLI tools | https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html | official tutorial | Free | Beg | Nodes, topics, services, parameters, actions, bags | 04 |
| Beginner: client libraries | https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html | official tutorial | Free | Beg | Workspaces, colcon, publishers and subscribers, interfaces | 04 |
| Intermediate: launch | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html | official tutorial | Free | Int | Python launch files | 04 |
| Intermediate: tf2 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html | official tutorial | Free | Int | Broadcasters, listeners, time travel | 05 |
| Intermediate: URDF | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html | official tutorial | Free | Int | Robot description, xacro, robot_state_publisher | 05, 14 |
| ROS 2 concepts (Jazzy) | https://docs.ros.org/en/jazzy/Concepts.html | official docs | Free | Beg–Int | Concept pages at three levels | 04 |
| About QoS settings | https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html | official docs | Free | Int | Reliability, durability, history; sensor-data profiles | 04, 07 |
| About middleware vendors (DDS/Zenoh RMWs) | https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html | official docs | Free | Int | Choosing an RMW | 04, 03 |
| Lyrical Luth release notes | https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html | official docs | Free | Int | What the next LTS changes (the course's upgrade candidate) | 04 |
| design.ros2.org (all design articles) | https://design.ros2.org/ | design docs | Free | Int–Adv | The "why" behind ROS 2. See especially ROS on DDS: https://design.ros2.org/articles/ros_on_dds.html | 04 |
| REP-105 Coordinate Frames for Mobile Platforms | https://www.ros.org/reps/rep-0105.html | standard | Free | Int | The map / odom / base_link conventions | 05, 09–12 |
| REP-2000 ROS 2 Releases and Target Platforms | https://www.ros.org/reps/rep-2000.html | standard | Free | Int | OS and architecture tiers per distro (not yet updated for Lyrical) | 04 |
| micro-ROS (now at Vulcanexus) | https://micro.vulcanexus.org/ | official docs | Free | Int–Adv | ROS 2 on microcontrollers. Pico port: https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk | FC, 01 |
| Robotics Stack Exchange | https://robotics.stackexchange.com/ | Q&A | Free | All | Searchable ROS Q&A (ROS Answers moved here) | all |
| Open Robotics Discourse | https://discourse.openrobotics.org/ | forum | Free | All | Release announcements and working groups | 04, 06 |
| *Robot Operating System 2: Design, Architecture, and Uses in the Wild* (Macenski et al., 2022) | https://arxiv.org/abs/2211.07752 | paper | Free | Int | The ROS 2 design paper | 04 |
| ETH Programming for Robotics — ROS (RSL, 2026 edition) | https://rsl.ethz.ch/education-students/lectures/ros.html | university course | Free (public ZIP) | Beg–Int | A compact 5-lecture ROS 2 bootcamp | 04, 05 |

## 3. Simulation

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Gazebo Harmonic docs (pairs with Jazzy) | https://gazebosim.org/docs/harmonic/tutorials/ | official docs | Free | Beg–Int | Worlds, SDF, sensors, gz tools | 06 |
| Gazebo: installing with ROS (pairing table) | https://gazebosim.org/docs/latest/ros_installation/ | official docs | Free | Beg | Which Gazebo release goes with which ROS 2 distro | 06 |
| Setting up a robot simulation (ROS 2 + Gazebo) | https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html | official tutorial | Free | Int | ROS 2 ↔ Gazebo integration | 06 |
| ros2_control docs (Jazzy) | https://control.ros.org/jazzy/index.html | official docs | Free | Int | Hardware interfaces and controllers. Demos: https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html | 06, 08, 14 |
| gz_ros2_control | https://github.com/ros-controls/gz_ros2_control | official code + docs | Free | Int | ros2_control hardware simulated in Gazebo | 06 |
| MuJoCo docs | https://mujoco.readthedocs.io/ | official docs | Free | Int | The physics engine used by most robot-learning work | 06, 17, 18 |
| MuJoCo Playground | https://playground.mujoco.org/ | official docs | Free | Int | GPU RL environments that train in minutes on one GPU or Colab | 17 |
| Isaac Lab docs | https://isaac-sim.github.io/IsaacLab/ | official docs | Free (needs an NVIDIA RTX GPU) | Int–Adv | GPU-parallel RL for robots | 17, 18 |
| Isaac Sim requirements | https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html | official docs | Free | Int | Check your GPU before you plan on Isaac Sim | 06, 17 |
| Genesis docs | https://genesis-world.readthedocs.io/ | official docs | Free | Int | Young multi-physics simulator; USABLE-BY-HOBBYIST | 06, 17 |
| ManiSkill3 | https://github.com/haosulab/ManiSkill | official code + docs | Free (assets CC BY-NC) | Int | GPU-parallel manipulation simulation (Linux + NVIDIA) | 17, 18 |
| *Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World* (Tobin et al., 2017) | https://arxiv.org/abs/1703.06907 | paper | Free | Int | The core sim-to-real idea ([papers.md 6.4](papers.md#64-domain-randomization-for-transferring-deep-neural-networks-from-simulation-to-the-real-world)) | 06, 17 |

## 4. Mathematics

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Solà, *Quaternion kinematics for the error-state Kalman filter* | https://arxiv.org/abs/1711.02508 | paper | Free | Adv | The reference for quaternion conventions and IMU filter math | FM, 05, 07, 10 |
| Solà, Deray, Atchuthan, *A micro Lie theory for state estimation in robotics* | https://arxiv.org/abs/1812.01537 | paper | Free | Adv | SO(3)/SE(3) Jacobians made practical | FM, 10, 11 |
| MIT 18.06 Linear Algebra (Strang) | https://ocw.mit.edu/courses/18-06-linear-algebra-spring-2010/ | university course + video | Free | Int | Full university linear algebra. Self-study version: https://ocw.mit.edu/courses/18-06sc-linear-algebra-fall-2011/ | FM |
| 3Blue1Brown, Essence of Linear Algebra | https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab | video | Free | Beg | Geometric intuition for transforms, determinants, eigenvectors | FM, 05 |
| 3Blue1Brown, Essence of Calculus | https://www.youtube.com/playlist?list=PLZHQObOWTQDMsr9K-rj53DwVRMYO3t5Yr | video | Free | Beg | Intuition for derivatives and integrals | FM, 08 |
| Visualizing quaternions (Ben Eater + 3Blue1Brown) | https://eater.net/quaternions | interactive video | Free | Int | The best intuition for quaternions. Companion: https://www.3blue1brown.com/lessons/quaternions-and-3d-rotation | FM, 05 |
| Khan Academy, Trigonometry | https://www.khanacademy.org/math/trigonometry | course | Free | Beg | Filling trig gaps (unit circle, atan2 thinking) | FM, 05 |
| Khan Academy, Probability | https://www.khanacademy.org/math/statistics-probability/probability-library | course | Free | Beg | Probability before Bayes filters | FM, 10 |
| Seeing Theory (Brown University) | https://seeing-theory.brown.edu/ | interactive | Free | Beg | Visual probability and statistics | FM, 07 |
| Immersive Linear Algebra | https://immersivemath.com/ila/index.html | interactive book | Free | Beg–Int | Linear algebra with interactive 3D figures | FM |
| Convex Optimization (Boyd & Vandenberghe) | https://web.stanford.edu/~boyd/cvxbook/ | book | Free PDF | Adv | Foundations for MPC and trajectory optimization | FM, FCT, 12 |

## 5. Electronics and embedded systems

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Raspberry Pi docs: computers (incl. Pi 5 power) | https://www.raspberrypi.com/documentation/computers/raspberry-pi.html | official docs | Free | Beg | Board specs, power (Pi 5: 5 V/5 A USB-C), GPIO header | 01, 02 |
| Raspberry Pi docs: microcontrollers (Pico series) | https://www.raspberrypi.com/documentation/microcontrollers/ | official docs | Free | Beg–Int | Pico / Pico 2 entry point | 01, FC |
| Pico C/C++ SDK docs | https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html | official docs | Free | Int | Native firmware. Examples: https://github.com/raspberrypi/pico-examples | FC |
| MicroPython docs | https://docs.micropython.org/en/latest/ | official docs | Free | Beg | Python on the Pico | 01, FC |
| MicroPython RP2 quick reference | https://docs.micropython.org/en/latest/rp2/quickref.html | official docs | Free | Beg | Pins, PWM, I²C, UART on the Pico | 01, 02, FC |
| MicroPython `rp2` module (PIO) | https://docs.micropython.org/en/latest/library/rp2.html | official docs | Free | Int | PIO state machines for encoder decoding. **`machine.Encoder` does not support the Pico** (ESP32/MIMXRT only). | 01, 07, FC |
| MicroPython `machine.PWM` | https://docs.micropython.org/en/latest/library/machine.PWM.html | official docs | Free | Beg | PWM on the Pico | 01, FE |
| pico-examples: PIO quadrature encoder (C) | https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder | official code | Free | Int | Reliable high-rate encoder counting | 01, FC |
| gpiozero docs | https://gpiozero.readthedocs.io/ | official docs | Free | Beg | Pythonic GPIO on the Pi. **On the Pi 5, only the lgpio backend works; RPi.GPIO does not.** | 01, 02 |
| pinout.xyz | https://pinout.xyz/ | reference | Free | Beg | Interactive map of the 40-pin header | 01, 02 |
| Pololu brushed DC motor drivers | https://www.pololu.com/category/11/brushed-dc-motor-drivers | vendor docs | Free docs | Beg–Int | Driver selection; each product page has pinouts and current limits | 01, 02 |
| Pololu documentation index | https://www.pololu.com/docs | vendor docs | Free | Int | User guides for motor controllers | 02 |
| Arduino docs | https://docs.arduino.cc/ | official docs | Free | Beg | Arduino boards and libraries (not the course path, but common) | FC |
| SparkFun: Voltage, Current, Resistance, and Ohm's Law | https://learn.sparkfun.com/tutorials/voltage-current-resistance-and-ohms-law | tutorial | Free | Beg | First principles | FE |
| SparkFun: Pulse Width Modulation | https://learn.sparkfun.com/tutorials/pulse-width-modulation | tutorial | Free | Beg | Motor speed and servo control | FE, 01 |
| SparkFun: I2C | https://learn.sparkfun.com/tutorials/i2c | tutorial | Free | Beg | The sensor bus | FE, 02 |
| SparkFun: Serial Peripheral Interface (SPI) | https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi | tutorial | Free | Beg | Fast sensor and display buses | FE, 02 |
| SparkFun: Serial Communication | https://learn.sparkfun.com/tutorials/serial-communication | tutorial | Free | Beg | UART between Pi and Pico | FE, 01, 03 |
| SparkFun: Pull-up Resistors | https://learn.sparkfun.com/tutorials/pull-up-resistors | tutorial | Free | Beg | Floating inputs, I²C lines | FE, 02 |
| SparkFun: How to Use a Multimeter | https://learn.sparkfun.com/tutorials/how-to-use-a-multimeter | tutorial | Free | Beg | Debugging hardware safely | FE, 01, 02 |
| SparkFun: How to Solder (through-hole) | https://learn.sparkfun.com/tutorials/how-to-solder-through-hole-soldering | tutorial | Free | Beg | Headers and connectors | FE, 01, 02 |
| Adafruit: Li-Ion & LiPoly Batteries | https://learn.adafruit.com/li-ion-and-lipoly-batteries | tutorial | Free | Beg | Chemistry, charging, protection, care | FE, 01 |
| Adafruit: Motor Selection Guide | https://learn.adafruit.com/adafruit-motor-selection-guide | tutorial | Free | Beg | DC vs. servo vs. stepper; choosing drivers | FE, 01 |
| Adafruit: I2C Addresses and Troublesome Chips | https://learn.adafruit.com/i2c-addresses | reference | Free | Beg–Int | Resolving I²C address conflicts | 02 |
| Ben Eater | https://eater.net/ | video | Free | Beg–Int | Bottom-up intuition for digital electronics | FE |
| learncpp.com | https://www.learncpp.com/ | tutorial | Free | Beg–Int | The best free modern C++ course | FC, 03 |
| cppreference.com | https://en.cppreference.com/ | reference | Free | All | Authoritative C++ standard library reference | FC, 03 |
| Stachniss, Modern C++ for Computer Vision (Uni Bonn) | https://www.ipb.uni-bonn.de/teaching/cpp-2020/index.html | university course | Free | Int | C++ and CMake for robotics and CV engineers | FC, 03 |
| All About Circuits textbook | https://www.allaboutcircuits.com/textbook/ | book | Free | Beg–Int | A deeper theory reference | FE, 02 |
| The Art of Electronics, 3rd ed. (Horowitz & Hill) | https://artofelectronics.net/ | book | Paid | Int–Adv | The practitioner's bible for circuit design | FE, 02 |
| Making Embedded Systems, 2nd ed. (Elecia White) | https://www.oreilly.com/library/view/making-embedded-systems/9781098151539/ | book | Paid | Int | Embedded architecture, motors, debugging patterns | FC, 02 |

## 6. Control

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| ros2_control docs (Jazzy) | https://control.ros.org/jazzy/index.html | official docs | Free | Int | Where control loops run in ROS 2 | 08, 06 |
| Steve Brunton, Control Bootcamp | https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m | university lectures | Free | Int–Adv | State space, LQR, Kalman filter, observability | FCT, 08, 10 |
| Brian Douglas, Classical Control Theory lectures | https://www.youtube.com/playlist?list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk | video | Free | Int | Intuition for root locus, Bode, Nyquist | FCT |
| MATLAB Tech Talks: Understanding PID Control (Brian Douglas) | https://www.mathworks.com/videos/series/understanding-pid-control.html | video | Free | Beg–Int | Anti-windup, derivative filtering, tuning | 08 |
| MATLAB Tech Talks: Control Systems in Practice | https://www.youtube.com/playlist?list=PLn8PRpmsu08pFBqgd_6Bi7msgkWFKL33b | video | Free | Int | Real-world control engineering problems | 08, FCT |
| Brett Beauregard, "Improving the Beginner's PID" | http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ (the site has no working https) | tutorial | Free | Beg–Int | Turning textbook PID into robust embedded code: sample time, derivative kick, windup | 08 |
| Feedback Systems, 2nd ed. (Åström & Murray) | https://www.cds.caltech.edu/~murray/FBS/Second_Edition.html | book | Free online | Int | A rigorous but readable control textbook | FCT, 08 |

## 7. Sensors, state estimation and SLAM

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| REP-145 Conventions for IMU Sensor Drivers | https://www.ros.org/reps/rep-0145.html | standard | Free | Int | IMU message and frame conventions | 07 |
| ROS camera_calibration package docs | https://docs.ros.org/en/jazzy/p/camera_calibration/ | official docs | Free | Int | Calibrating cameras in ROS (CameraInfo) | 07, 13 |
| SLAM Toolbox (repo README = docs) | https://github.com/SteveMacenski/slam_toolbox | official code + docs | Free | Int | The course's 2D SLAM | 11 |
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | official code | Free (GPL-3.0) | Adv | Visual and visual-inertial SLAM reference | 11 |
| Papers: Kalman/EKF, UKF, MCL, KLD-sampling, graph SLAM, SLAM Toolbox, ORB-SLAM | [papers.md Tracks 1–2](papers.md#track-1--probabilistic-state-estimation) | papers | Free/Paid (DOI links may be paywalled) | Int–Adv | The algorithms behind robot_localization, AMCL and slam_toolbox | 10, 11 |
| Stachniss, YouTube playlists | https://www.youtube.com/@CyrillStachniss/playlists | university lectures | Free | Int–Adv | Exceptionally clear lectures on SLAM, ICP, Kalman and particle filters | 10, 11 |
| Stachniss teaching page (Uni Bonn) | https://www.ipb.uni-bonn.de/teaching/ | university course hub | Free | Int–Adv | Index of courses with slides and videos | 10, 11 |
| Stachniss, Mobile Sensing and Robotics 1 (2021) | https://www.ipb.uni-bonn.de/msr1-2021/index.html | university course | Free | Int | Sensors and state estimation basics | 07, 10 |
| Stachniss, Mobile Sensing and Robotics 2 (2021) | https://www.ipb.uni-bonn.de/msr2-2021/index.html | university course | Free | Adv | Graph SLAM, bundle adjustment, localization | 11 |
| Stachniss, Self-Driving Cars (2021) | https://www.ipb.uni-bonn.de/sdc-2021/index.html | university course | Free | Int–Adv | Localization, mapping and planning for vehicles | 10–12 |
| KalmanFilter.net (Alex Becker) | https://kalmanfilter.net/ | tutorial | Free (book Paid) | Beg–Int | A KF/EKF/UKF intro driven by numerical examples | 10 |
| Kalman and Bayesian Filters in Python (Roger Labbe) | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python | book (Jupyter) | Free | Int | Hands-on filters with code; static since Aug 2024 but still the best | 10 |
| Probabilistic Robotics (Thrun, Burgard, Fox) | https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ | book | Paid | Int–Adv | The classic reference for Bayes filters, EKF/UKF/particle filters and SLAM | 10, 11 |

## 8. Navigation and planning

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Nav2 documentation (Jazzy) | https://docs.nav2.org/jazzy/ | official docs | Free | Int | The navigation stack (the root https://docs.nav2.org/ redirects to Rolling) | 12 |
| Nav2 navigation concepts | https://docs.nav2.org/jazzy/getting_started/navigation_concepts/ | official docs | Free | Int | Servers, behavior trees, costmaps, REP-105 frames | 12 |
| Nav2 first-time robot setup guide | https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/ | official tutorial | Free | Int | TF, URDF, odometry, sensors and footprint for your own robot | 12, 20 |
| Nav2 behavior trees | https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/ | official docs | Free | Int | Nav2 BT XML and example trees | 12, 19 |
| BehaviorTree.CPP docs | https://www.behaviortree.dev/ | official docs | Free | Int | The BT engine used by Nav2 | 12, 19 |
| Papers: A*, RRT, DWA, MPPI, Nav2, Smac, behavior trees | [papers.md Track 3](papers.md#track-3--planning-and-navigation) | papers | Free/Paid | Int–Adv | The ideas behind Nav2's planners and controllers | 12 |
| *From the Desks of ROS Maintainers* (Macenski et al., 2023) | https://arxiv.org/abs/2307.15236 | paper | Free | Int | A map of the mobile-robotics algorithms actually deployed in ROS 2 | 11, 12 |
| Stachniss, Self-Driving Cars (2021) | https://www.ipb.uni-bonn.de/sdc-2021/index.html | university course | Free | Int–Adv | Planning lectures | 12 |

## 9. Arms and manipulation

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| MoveIt 2 docs and tutorials (main) | https://moveit.picknik.ai/main/doc/tutorials/tutorials.html | official docs | Free | Int | Motion planning for arms. There is no `/jazzy/` path; use main and the matching branch. | 14, 15 |
| MoveIt 2 quickstart in RViz | https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html | official tutorial | Free | Int | Your first MoveIt session | 14 |
| ros2_control demos (Jazzy) | https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html | official docs | Free | Int | Joint trajectory controllers and hardware interfaces | 14 |
| LeRobot SO-101 arm guide | https://huggingface.co/docs/lerobot/so101 | official docs | Free (hardware Paid) | Int | Building and calibrating a low-cost arm | 14, 18 |
| Papers: MoveIt, grasp synthesis, Dex-Net, Contact-GraspNet | [papers.md Track 4](papers.md#track-4--manipulation-and-kinematics) | papers | Free | Int–Adv | Motion planning and grasping | 14, 15 |
| MIT Robotic Manipulation (Tedrake) | https://manipulation.csail.mit.edu/ | university course notes | Free | Int–Adv | Perception, grasping and learned manipulation, with notebooks | 15, 18 |
| Modern Robotics, Coursera | https://www.coursera.org/specializations/modernrobotics | university course | Free/Paid | Int | Kinematics, Jacobians, trajectories | 14 |
| Robotics Toolbox for Python (Corke) | https://github.com/petercorke/robotics-toolbox-python | library + tutorial | Free | Beg–Int | Quick FK/IK/Jacobian/trajectory experiments | 14 |
| Modern Robotics (Lynch & Park) | https://hades.mech.northwestern.edu/index.php/Modern_Robotics | book | Free PDF | Int | FK/IK, Jacobians, dynamics, grasping | 14, 15 |

## 10. Computer vision

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| OpenCV-Python tutorials | https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html | official docs | Free | Beg–Int | Image processing, features, video | 13, FCV |
| OpenCV camera calibration (Python) | https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html | official tutorial | Free | Int | Intrinsics and distortion from chessboards | 13 |
| Ultralytics YOLO docs | https://docs.ultralytics.com/ | official docs | Free docs (code AGPL-3.0 or commercial license) | Beg–Int | The fastest route to detection, segmentation and pose on robots. Training: https://docs.ultralytics.com/modes/train | 13, 16 |
| Hugging Face Transformers: vision task guides | https://huggingface.co/docs/transformers/tasks/object_detection | official docs | Free | Int | Fine-tuning DETR-style detectors. OWLv2: https://huggingface.co/docs/transformers/model_doc/owlv2 | 13, 16 |
| Papers: YOLO, ViT, DETR, CLIP, Grounding DINO, SAM, DINOv2, Depth Anything V2 | [papers.md Track 5](papers.md#track-5--vision-for-robots) | papers | Free | Int–Adv | Where modern robot perception comes from | 13 |
| First Principles of Computer Vision (Shree Nayar, Columbia) | https://fpcv.cs.columbia.edu/ | university video course | Free | Beg–Int | Beautifully explained classical CV: imaging, features, stereo | 13, FCV |
| Stanford CS231n | https://cs231n.stanford.edu/ | university course | Free materials | Int | CNNs and ViTs from scratch. Notes: https://cs231n.github.io/ | 13, FML |
| Hugging Face Community Computer Vision Course | https://huggingface.co/learn/computer-vision-course/unit0/welcome/welcome | course | Free | Beg–Int | Broad modern CV with HF tooling | 13, 16 |
| Roboflow notebooks | https://github.com/roboflow/notebooks | notebooks | Free | Beg–Int | Copy-paste fine-tuning recipes for the latest detectors. Vendor-oriented and thin on theory: use them as recipes, not as a course. | 16 |
| Szeliski, *Computer Vision: Algorithms and Applications*, 2nd ed. | https://szeliski.org/Book/ | book | Free PDF (personal use) | Int–Adv | A broad modern CV reference | 13, FCV |
| Hartley & Zisserman, *Multiple View Geometry* | https://www.robots.ox.ac.uk/~vgg/hzbook/ | book | Paid | Adv | Epipolar geometry, homographies, bundle adjustment | 13, 11 |

## 11. Machine learning and deep learning

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| PyTorch tutorials | https://docs.pytorch.org/tutorials/ | official docs | Free | Beg–Int | Official PyTorch how-tos. Install selector: https://pytorch.org/get-started/locally/ | FML, 16 |
| Hugging Face Learn (LLM, Deep RL, CV, Robotics) | https://huggingface.co/learn | course hub | Free | Beg–Int | Courses built on the HF ecosystem | FML, 16–19 |
| Stanford CS229 Machine Learning | https://cs229.stanford.edu/ | university course | Free materials | Int–Adv | Classical ML theory | FML |
| Stanford CS224N NLP with Deep Learning | https://web.stanford.edu/class/cs224n/ | university course | Free materials | Int–Adv | Transformers and LLM foundations | FML, 19 |
| Stanford CS336 Language Modeling from Scratch | https://cs336.stanford.edu/ | university course | Free materials | Adv | Building and training LMs end to end | FML |
| fast.ai Practical Deep Learning for Coders | https://course.fast.ai/ | course | Free | Beg–Int | Top-down, practical deep learning | FML, 16 |
| Karpathy, Neural Networks: Zero to Hero | https://karpathy.ai/zero-to-hero.html | video course | Free | Int | From backprop to GPT, from scratch | FML |
| Dive into Deep Learning | https://d2l.ai/ | book | Free | Int | An interactive textbook with PyTorch code | FML |

## 12. Reinforcement learning

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Gymnasium docs (Farama) | https://gymnasium.farama.org/ | official docs | Free | Beg | The standard environment API; you wrap the course simulator in it | 17 |
| Stable-Baselines3 docs | https://stable-baselines3.readthedocs.io/ | official docs | Free | Beg–Int | Reliable PPO/SAC/TD3 baselines | 17 |
| CleanRL docs | https://docs.cleanrl.dev/ | official docs + code | Free | Int | Readable single-file implementations | 17 |
| Papers: DQN, PPO, SAC, domain randomization, legged_gym, HIL-SERL | [papers.md Track 6](papers.md#track-6--reinforcement-learning) | papers | Free/Paid | Int–Adv | The algorithms you train with | 17 |
| Berkeley CS 185/285 Deep RL (Levine) | https://rail.eecs.berkeley.edu/deeprlcourse/ | university course | Free slides and videos | Adv | The deepest deep-RL course, including imitation and model-based RL | 17, 18 |
| Stanford CS 224R Deep Reinforcement Learning (Finn) | https://cs224r.stanford.edu/ | university course | Free slides and videos | Adv | Imitation, offline RL and multi-task RL for robots | 17, 18 |
| David Silver, UCL/DeepMind RL course | https://www.youtube.com/playlist?list=PLqYmG7hTraZDM-OYHWgPebj2MfCFzFObQ | university lectures | Free | Int | Classic RL lectures | 17 |
| Hugging Face Deep RL Course | https://huggingface.co/learn/deep-rl-course | course | Free | Beg–Int | Hands-on with SB3 and CleanRL-style code | 17 |
| OpenAI Spinning Up in Deep RL | https://spinningup.openai.com/ | tutorial | Free | Int | Concise deep-RL theory and a key-papers list. The code is dated; read it for concepts. | 17 |
| Sutton & Barto, *Reinforcement Learning: An Introduction*, 2nd ed. | http://incompleteideas.net/book/the-book-2nd.html (the site has no working https) | book | Free PDF | Int | The foundational text | 17 |

## 13. Embodied AI and robot learning

> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09-16). As of that date LeRobot was at v0.6.1, and its API
> breaks between minor versions. Pin it. Maturity labels come from
> [`research/embodied-ai-landscape-2026-09.md`](research/embodied-ai-landscape-2026-09.md).

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| LeRobot docs | https://huggingface.co/docs/lerobot/index | official docs | Free | Int | Datasets, ACT/Diffusion/VLA policies, real robots (USABLE-BY-HOBBYIST) | 18 |
| LeRobot GitHub | https://github.com/huggingface/lerobot | official code | Free | Int | Source, issues, releases | 18 |
| LeRobot: imitation learning on real-world robots | https://huggingface.co/docs/lerobot/il_robots | official tutorial | Free | Int | Record, train and evaluate a policy | 18 |
| LeRobot compute and hardware guide | https://huggingface.co/docs/lerobot/hardware_guide | official docs | Free | Int | VRAM and training-time estimates per policy | 18, 16 |
| LeRobot SmolVLA docs | https://huggingface.co/docs/lerobot/smolvla | official docs | Free | Int | Your first VLA fine-tune | 18 |
| openpi (π0 / π0.5) | https://github.com/Physical-Intelligence/openpi | official code | Free (Apache-2.0 + Gemma terms) | Adv | Open π0 and π0.5 weights; LoRA needs a 24 GB+ GPU | 18 |
| Isaac GR00T | https://github.com/NVIDIA/Isaac-GR00T | official code | Free (NVIDIA Open Model License weights) | Adv | GR00T N1.7; fine-tuning needs 40 GB+ | 18 |
| *Robot Learning: A Tutorial* (Capuano et al., 2025) | https://arxiv.org/abs/2510.12403 | paper / tutorial | Free | Int | Modern robot learning (RL, BC, VLAs) with LeRobot examples. Web version: https://huggingface.co/spaces/lerobot/robot-learning-tutorial | 18 |
| Papers: DAgger, ACT, Diffusion Policy, LeRobot, RT-2, OpenVLA, π0, SmolVLA, π0.5, GR00T, Gemini Robotics | [papers.md Track 7](papers.md#track-7--imitation-learning-and-vision-language-action-models) | papers | Free | Int–Adv | The methods behind the policies you train | 18 |
| ETH Zurich Robot Learning (Oier Mees, Spring 2026) | https://cvg.ethz.ch/lectures/Robot-Learning/ | university course | Free slides, videos, homework | Int–Adv | **The best fully open current robot-learning course:** IL, RL, generative policies, world models, VLAs | 18, 17 |
| Cornell CS 4756/5756 Robot Learning (Spring 2026) | https://www.cs.cornell.edu/courses/cs4756/2026sp/ | university course | Free slides | Int | IL, RL and MPC for robots | 18, 17 |
| CMU 16-831 Introduction to Robot Learning (Spring 2026) | https://sites.google.com/view/16-831-cmu/home | university course | Free slides | Int–Adv | RL, IL and visual learning for robots | 18, 17 |
| Hugging Face Robotics Course | https://huggingface.co/learn/robotics-course | course | Free | Beg–Int | A LeRobot-based course; units 5–7 (RL, IL, foundation models) were "coming soon" in 2026-09 | 18 |

## 14. LLM agents for robots

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Anthropic Claude: tool use overview | https://platform.claude.com/docs/en/agents-and-tools/tool-use/overview | official docs | Free docs (API Paid) | Int | Tool calling with Claude. Defining tools: https://platform.claude.com/docs/en/agents-and-tools/tool-use/define-tools | 19 |
| OpenAI: function calling | https://developers.openai.com/api/docs/guides/function-calling | official docs | Free docs (API Paid) | Int | Tool calling with OpenAI models | 19 |
| Gemini Robotics-ER overview (Gemini API) | https://ai.google.dev/gemini-api/docs/robotics-overview | official docs | Free docs (API pricing varies) | Int | A first-party "robot brain": pointing, trajectories, function calling (preview) | 19, 18 |
| Model Context Protocol | https://modelcontextprotocol.io/ | official docs | Free | Int | Exposing robot capabilities as MCP tools. Spec: https://modelcontextprotocol.io/specification/latest | 19 |
| py_trees docs | https://py-trees.readthedocs.io/ | official docs | Free | Int | Behavior trees in Python. ROS 2 tutorials: https://py-trees-ros-tutorials.readthedocs.io/ | 19 |
| BehaviorTree.CPP docs | https://www.behaviortree.dev/ | official docs | Free | Int | The BT engine used by Nav2; the Groot editor | 19, 12 |
| ROSA (NASA JPL) | https://github.com/nasa-jpl/rosa | official code | Free (Apache-2.0) | Int | A LangChain agent for ROS introspection | 19 |
| ros-mcp-server | https://github.com/robotmcp/ros-mcp-server | code | Free (Apache-2.0) | Int | MCP ↔ rosbridge. **No documented safety layer**: use it for development, never as a production control surface | 19 |
| RAI (Robotec.ai) | https://github.com/RobotecAI/rai | code | Free (Apache-2.0) | Int–Adv | A multi-agent framework for ROS 2 | 19 |
| Papers: SayCan, Code as Policies, ChatGPT for Robotics, Inner Monologue, RoboPAIR, RoboGuard, ROSA | [papers.md Track 8](papers.md#track-8--llm-agents-for-robots) | papers | Free | Int | Agent patterns and their failure modes; **read RoboPAIR before giving an LLM actuators** | 19 |

## 15. Linux, Python and developer tools

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Ubuntu Server documentation | https://ubuntu.com/server/docs | official docs | Free | Beg–Int | Headless Pi setup, networking, services | FL, 01, 03 |
| Ubuntu: the Linux command line for beginners | https://ubuntu.com/tutorials/command-line-for-beginners | official tutorial | Free | Beg | A quick shell refresher | FL |
| Docker docs | https://docs.docker.com/ | official docs | Free | Beg–Int | Containerized ROS development. Get started: https://docs.docker.com/get-started/ | FL, 03, 04 |
| uv docs (Astral) | https://docs.astral.sh/uv/ | official docs | Free | Beg–Int | Fast Python environments and lockfiles | FL, FPY, 03 |
| Python Packaging User Guide | https://packaging.python.org/ | official docs | Free | Int | pyproject.toml and wheels | FPY, 03 |
| pytest docs | https://docs.pytest.org/ | official docs | Free | Beg–Int | Testing robot software without hardware | 03 |
| MIT Missing Semester (2026 edition) | https://missing.csail.mit.edu/ | university course | Free | Beg–Int | Shell, git, debugging, packaging | FL, 03 |
| Pro Git | https://git-scm.com/book/en/v2 | book | Free | Beg–Int | Git internals and workflows | FL, 03 |
| The Linux Command Line (William Shotts) | https://linuxcommand.org/tlcl.php | book | Free PDF | Beg–Int | A complete shell and scripting book | FL |

## 16. Safety

Read [SAFETY.md](../SAFETY.md) first. The standards below are for context. A home robot is not an
industrial robot cell, but the hazard categories are the same.

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| ISO 10218-1:2025 Robots — safety requirements | https://www.iso.org/standard/73933.html | standard (overview page) | Paid | Adv | Robot-level requirements (replaces the 2011 edition) | 00, 14, 20 |
| ISO 10218-2:2025 Robot applications and cells | https://www.iso.org/standard/73934.html | standard (overview page) | Paid | Adv | Integration safety; **now contains the former ISO/TS 15066 collaborative-robot content** | 14, 20 |
| ROS 2 security: setting up SROS2 | https://docs.ros.org/en/jazzy/Tutorials/Advanced/Security/Introducing-ros2-security.html | official tutorial | Free | Int–Adv | Keystores, enclaves, encrypted DDS | 04, 19, 20 |
| ROS 2 DDS-Security integration (design) | https://design.ros2.org/articles/ros2_dds_security.html | design doc | Free | Adv | Security architecture rationale | 04, 20 |
| OSHA robotics safety | https://www.osha.gov/robotics | guidance | Free | Beg | Hazards and the US regulatory context. Hazards: https://www.osha.gov/robotics/hazards | 00, 14 |
| OSHA Technical Manual: Industrial Robot System Safety | https://www.osha.gov/otm/section-4-safety-hazards/chapter-4 | guidance | Free | Int | Safeguarding and risk assessment | 14, 20 |
| A3: Updated ISO 10218 FAQ | https://www.automate.org/robotics/blogs/updated-iso-10218-faq | article | Free | Int | A plain-language summary of what changed in 2025 | 14, 20 |
| Papers: RoboPAIR, RoboGuard | [papers.md 8.5–8.6](papers.md#85-jailbreaking-llm-controlled-robots-robopair) | papers | Free | Int | Why LLM guardrails must live outside the LLM | 19, 20 |
| Battery University BU-304a: Safety Concerns with Li-ion | https://batteryuniversity.com/article/bu-304a-safety-concerns-with-li-ion | article | Free | Beg | Thermal runaway and safe handling | FE, 01 |
| Adafruit: Li-Ion & LiPoly Batteries | https://learn.adafruit.com/li-ion-and-lipoly-batteries | tutorial | Free | Beg | Handling LiPo batteries at hobby scale | FE, 01 |

## 17. 3D printing and CAD

| Resource | URL | Kind | Cost | Level | Best for | Modules |
|---|---|---|---|---|---|---|
| Onshape Learning Center | https://learn.onshape.com/ | official course | Free | Beg–Int | Structured CAD courses. Fundamentals: https://learn.onshape.com/learn/course/fundamentals-cad | F3D |
| Onshape free plan | https://www.onshape.com/en/products/free | product terms | Free (non-commercial; documents are public) | Beg | CAD in the browser that works on Linux | F3D |
| FreeCAD documentation wiki | https://wiki.freecad.org/ | official docs | Free | Beg–Int | Fully open offline CAD. Getting started: https://wiki.freecad.org/Getting_started | F3D |
| Autodesk Fusion for Personal Use | https://www.autodesk.com/products/fusion-360/personal | product terms | Free for qualifying users | Beg–Int | Home, non-commercial use under US$1,000/yr revenue | F3D |
| onshape-to-robot | https://onshape-to-robot.readthedocs.io/ | official docs | Free | Int | Onshape assembly → URDF / SDF / MuJoCo | F3D, 05, 14 |
| fusion360-urdf-ros2 | https://github.com/runtimerobotics/fusion360-urdf-ros2 | code | Free | Int | The most current Fusion → URDF exporter (Humble/Jazzy) | F3D, 05 |
| Prusa Knowledge Base | https://help.prusa3d.com/ | vendor docs | Free | Beg–Int | Printer troubleshooting and design for printing: https://help.prusa3d.com/article/modeling-with-3d-printing-in-mind_164135 | F3D |
| Bambu Lab Wiki | https://wiki.bambulab.com/ | vendor docs | Free | Beg | Printer operation and the filament table | F3D |
| Printables | https://www.printables.com/ | model repository | Free | Beg | Robot parts, brackets, wheels | F3D |
| Thingiverse | https://www.thingiverse.com/ | model repository | Free | Beg | A large library of older models | F3D |

---

## Fast-moving technologies: verify before use

> [!CAUTION]
> **AI teacher: before giving installation steps, commands, parameter names, model IDs or version
> numbers for anything in this table, open the official documentation linked here and check the
> current version.** Do not rely on training data or on this file alone. These projects rename
> packages, move doc pages and break APIs between releases. If what you find differs from
> [`curriculum/versions.yaml`](../curriculum/versions.yaml), tell the student, say which source
> you trust and why, and record the difference. A maintainer then refreshes the snapshot in
> [`research/`](research/) (see [MAINTAINING.md](../MAINTAINING.md)).
>
> **Student:** when a command from a lesson fails with "package not found", "unknown parameter" or
> "no such file", suspect version drift before suspecting yourself.

"Course pin" is what the course uses as of 2026-09-16, from
[`curriculum/versions.yaml`](../curriculum/versions.yaml). "What moves" is what broke or changed
recently.

| Technology | Official documentation | Course pin (2026-09-16) | What moves |
|---|---|---|---|
| **ROS 2** | https://docs.ros.org/en/jazzy/ · install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html · distros: https://docs.ros.org/en/jazzy/Releases.html · REP-2000: https://www.ros.org/reps/rep-2000.html | Jazzy Jalisco (LTS, EOL May 2029), Ubuntu 24.04, Python 3.12 | Lyrical Luth (May 2026, Ubuntu 26.04) is the next LTS but lacked Nav2 binaries in the main apt repo on 2026-09-16. Lyrical/Rolling docs moved away from `Tutorials/`. Kilted reaches EOL in late 2026. |
| **Gazebo** | https://gazebosim.org/docs/harmonic/tutorials/ · pairing: https://gazebosim.org/docs/latest/ros_installation/ | Harmonic via `ros-jazzy-ros-gz` | Gazebo Classic reached EOL on 2025-01-29, so `gazebo_ros` and `spawn_entity.py` tutorials no longer apply. `ros_ign_*` → `ros_gz_*`; `ign_ros2_control` → `gz_ros2_control`. Lyrical pairs with Jetty. |
| **ros2_control** | https://control.ros.org/jazzy/index.html · https://github.com/ros-controls/gz_ros2_control | ros2_control 4.48.x, ros2_controllers 4.42.x | On Jazzy, `diff_drive_controller` expects `TwistStamped`. Parameters are removed or renamed between distros. |
| **Nav2** | https://docs.nav2.org/jazzy/ · setup guide: https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/ | navigation2 1.3.x | Docs moved to per-distro paths (old deep links 404). BT.CPP v4 XML; plugin names use `::`. `cmd_vel` becomes `TwistStamped` by default after Jazzy. |
| **slam_toolbox** | https://github.com/SteveMacenski/slam_toolbox | 2.8.x | Parameter files change between distros. Cartographer gets no upstream development. |
| **MoveIt 2** | https://moveit.picknik.ai/main/index.html · tutorials: https://moveit.picknik.ai/main/doc/tutorials/tutorials.html | 2.12.x | The docs track Rolling (`main`) and there is no `/jazzy/` doc path, so pick the matching branch in the tutorials. |
| **micro-ROS** | https://micro.vulcanexus.org/ · Pico: https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk | Jazzy branches | micro.ros.org moved. No apt binary for the agent (build it or use Docker). No Lyrical branch of micro_ros_setup yet. |
| **NVIDIA JetPack / Jetson** | https://developer.nvidia.com/embedded/jetpack · archive: https://developer.nvidia.com/embedded/jetpack-archive · Orin Nano quick start: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html | JetPack 7.2.x (Ubuntu 24.04, CUDA 13) on Jetson Orin Nano Super | JetPack 7.2 has no microSD image for the Orin Nano (USB ISO install instead), and older firmware must first be updated through JetPack 6. |
| **NVIDIA Isaac ROS** | https://nvidia-isaac-ros.github.io/getting_started/ · releases: https://nvidia-isaac-ros.github.io/releases/ | Isaac ROS 4.x (targets Jazzy) | Tied to specific JetPack, CUDA and driver versions; runs in dev containers. |
| **NVIDIA Isaac Sim / Isaac Lab** | https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html · https://isaac-sim.github.io/IsaacLab/ · https://github.com/isaac-sim/IsaacLab | Isaac Sim 6.1; Isaac Lab 2.3 (3.0 in beta) | Isaac Lab 3.0 beta changes the install and physics backends. Minimum GPU: RTX 4080 16 GB for Isaac Sim 6.x. The WSL2 ROS path is deprecated. |
| **LeRobot** | https://huggingface.co/docs/lerobot/index · https://github.com/huggingface/lerobot | v0.6.x (pin the exact version) | v0.6.0 was a breaking cleanup (Python 3.12+, new extras, a rebuilt RL stack). CLI names and config keys change between minor versions. |
| **VLA models** | SmolVLA: https://huggingface.co/docs/lerobot/smolvla · π0/π0.5: https://github.com/Physical-Intelligence/openpi and https://huggingface.co/docs/lerobot/pi05 · GR00T: https://github.com/NVIDIA/Isaac-GR00T and https://huggingface.co/docs/lerobot/groot · Gemini Robotics: https://deepmind.google/models/gemini-robotics/ | SmolVLA first; π0.5 or GR00T N1.7 only with a 24–40 GB GPU | New versions every few months (GR00T went N1 → N1.7 in about 13 months). Weights licenses differ per release. Closed models (π0.7, Gemini Robotics 2 VLA) cannot be run. |
| **Detector and segmentation frameworks** | Ultralytics: https://docs.ultralytics.com/ · Grounding DINO: https://github.com/IDEA-Research/GroundingDINO · SAM 3: https://github.com/facebookresearch/sam3 · OWLv2 in Transformers: https://huggingface.co/docs/transformers/model_doc/owlv2 · OpenCV: https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html | Ultralytics YOLO26 or YOLO11 | A new YOLO generation roughly every year; export and TensorRT flags change. Ultralytics is AGPL-3.0. Licenses differ by model size for some models (for example Depth Anything V2). |
| **LLM APIs and agent protocols** | Claude: https://platform.claude.com/docs/en/agents-and-tools/tool-use/overview · OpenAI: https://developers.openai.com/api/docs/guides/function-calling · Gemini Robotics-ER: https://ai.google.dev/gemini-api/docs/robotics-overview · MCP spec: https://modelcontextprotocol.io/specification/latest | None pinned. Use the current model IDs from each provider's docs. | Model IDs are retired on short notice (Gemini Robotics-ER 1.6 preview shut down 2026-08-31). Doc domains moved (docs.claude.com → platform.claude.com, platform.openai.com → developers.openai.com). The MCP spec is versioned by date (latest was 2026-07-28). |

---

## Moved and dead links

These moves were found during verification on 2026-09-16. If you find an old link in a lesson or
elsewhere, replace it.

| Old URL (do not use) | Status on 2026-09-16 | Use instead |
|---|---|---|
| `http://www.probabilistic-robotics.org/` | 404 / connection failure | https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ |
| `https://fbswiki.org/` and the old Caltech `amwiki` | connection failure / 404 | https://www.cds.caltech.edu/~murray/FBS/Second_Edition.html |
| `https://www.cs.cmu.edu/~16831-f24/` (also f23, f25) | 404 | https://sites.google.com/view/16-831-cmu/home |
| `https://docs.nav2.org/behavior_trees/index.html` and other unversioned Nav2 deep links | "Page moved" / 404 | https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/ (and other `docs.nav2.org/jazzy/...` paths) |
| `https://docs.ros.org/en/rolling/Concepts.html`, `/en/rolling/Tutorials.html`, `/en/lyrical/Tutorials.html` | 404 (docs restructured) | The Jazzy paths in §2, or the Lyrical `Get-Started` layout: https://docs.ros.org/en/lyrical/Get-Started.html |
| `https://moveit.picknik.ai/jazzy/index.html` | 404 | https://moveit.picknik.ai/main/index.html |
| `https://micro.ros.org/` (docs paths) | "has moved"; deep links 404 | https://micro.vulcanexus.org/ |
| `https://learn.adafruit.com/lipo-battery-care` | 404 | https://learn.adafruit.com/li-ion-and-lipoly-batteries |
| `https://robotacademy.net.au/masterclass/` | 404 | https://robotacademy.net.au/ |
| `https://petercorke.com/books/robotics-vision-control-python/` | 404 | https://petercorke.com/rvc/ |
| Original UNC URL of Welch & Bishop, *An Introduction to the Kalman Filter* | 404 | https://www.cs.utexas.edu/~pstone/Courses/393Rfall15/readings/Welch+Bishop-TR-95.pdf (course mirror) |
| OpenAI function-calling docs on `platform.openai.com` | redirects | https://developers.openai.com/api/docs/guides/function-calling |
| Anthropic docs on `docs.claude.com` | redirects | https://platform.claude.com/docs/en/agents-and-tools/tool-use/overview |
| `https://discourse.ros.org/` | redirects | https://discourse.openrobotics.org/ |
| MicroPython `machine.Encoder` on the Pico | page is fine, but **rp2 is not supported** | PIO: https://docs.micropython.org/en/latest/library/rp2.html |

Sources that block automated checks but are fine in a browser (verified another way): Khan Academy,
Raspberry Pi documentation, MIT Press pages, ISO pages, Printables, All About Circuits. The OpenAI
"Learning Dexterity" blog and Microsoft's "ChatGPT for Robotics" article returned HTTP 403, so
[papers.md](papers.md) links the arXiv versions instead.
