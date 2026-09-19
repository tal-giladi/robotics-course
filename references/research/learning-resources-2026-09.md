# Learning resources for the robotics course (verified 2026-09-16)

Audience: experienced software/AI engineer. Every URL below was checked on 2026-09-16.

Verification legend (the "Chk" column):

- **OK**: HTTP 200 today, and the page title or content matched.
- **OK\***: the site blocks automated clients (Cloudflare/Anubis/403). I confirmed it another way: a WebFetch that returned the content, or a search-engine result showing that exact URL and title.
- **FAIL**: the URL is broken or moved today. A working replacement is listed where one exists.

Level: Beg = beginner, Int = intermediate, Adv = advanced.

---

## Findings that matter for the syllabus

- **ROS 2 distros (today).** Lyrical Luth was released May 2026. It is LTS, runs on Ubuntu 26.04 and is supported until May 2031. It pairs with Gazebo Jetty.
  - Jazzy Jalisco is also LTS: Ubuntu 24.04, supported until May 2029, paired with Gazebo Harmonic. Gazebo's own docs still recommend "Ubuntu 24.04 + Jazzy + Harmonic" to beginners.
  - Kilted is not LTS and reaches EOL in Nov 2026.
  - REP 2000 has not been updated for Lyrical yet. It lists only Jazzy and Kilted.
- **docs.ros.org was restructured for Lyrical/Rolling.** The old `Tutorials/...` paths return 404 on lyrical/rolling (for example `/en/rolling/Tutorials.html` and `/en/rolling/Concepts.html`). They moved under `Get-Started/`, `ROS-Framework/`, `Developer-Tools/` and `Capabilities/`. The Jazzy docs keep the classic `Tutorials/` layout. This file uses Jazzy links, plus the matching Lyrical links where it matters.
- **Nav2 docs moved to per-distro paths.** The old `docs.nav2.org/behavior_trees/index.html` now returns "Page moved". Use `docs.nav2.org/jazzy/...` or `docs.nav2.org/rolling/...`.
- **micro-ROS moved.** micro.ros.org now says "has moved" and points to https://micro.vulcanexus.org/.
- **MicroPython `machine.Encoder` does NOT support rp2 (Pico).** Its docs list only ESP32 and MIMXRT. On a Pico, read quadrature encoders with PIO: `rp2.StateMachine`, or the C `pico-examples/pio/quadrature_encoder`.
- **ISO/TS 15066 was folded into ISO 10218-2:2025.** ISO 10218-1/-2:2025 replace the 2011 editions.
- **Probabilistic Robotics site is down.** probabilistic-robotics.org returned 404 or a connection failure. Use the MIT Press page.
- **Other moves:**
  - The CMU 16-831 site moved to Google Sites. The old `cs.cmu.edu/~16831-f24/` returns 404.
  - fbswiki.org did not connect. The Caltech mirror of *Feedback Systems* works.
  - The OpenAI function-calling docs now redirect to developers.openai.com.
  - The Anthropic docs are at platform.claude.com. docs.claude.com redirects there.

---

## 1. Robotics (general)

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| Modern Robotics: Mechanics, Planning, and Control (Lynch & Park), book home | https://hades.mech.northwestern.edu/index.php/Modern_Robotics | book + videos | Free preprint PDF | Int | Rigorous screw-theory kinematics/dynamics; the best single textbook for arms and mobile manipulation | OK |
| Modern Robotics, free PDF (v2) | https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf | book | Free | Int | Offline reading. Tablet, large-font and 2-up variants are linked from the home page | OK |
| Modern Robotics, Coursera specialization (6 courses) | https://www.coursera.org/specializations/modernrobotics | course | Audit free; certificate paid | Int | Structured path with quizzes and a mobile-manipulator capstone | OK |
| Modern Robotics code library (Python/MATLAB/Mathematica) | https://github.com/NxRLab/ModernRobotics | code | Free | Int | Reference implementations of FK/IK/dynamics matching the book | OK |
| MIT 6.4210/6.4212 Robotic Manipulation (Tedrake) | https://manipulation.csail.mit.edu/ | course notes | Free | Int-Adv | Perception-to-grasp pipelines with Drake and runnable notebooks; notes being updated for Fall 2026 | OK |
| MIT Underactuated Robotics (Tedrake) | https://underactuated.csail.mit.edu/ | course notes + videos | Free | Adv | Dynamics, trajectory optimization, LQR/MPC, legged systems. Lecture videos: https://www.youtube.com/channel/UChfUOAhz7ynELF-s_1LPpWg | OK |
| Probabilistic Robotics (Thrun, Burgard, Fox), MIT Press page | https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ | book | Paid | Int-Adv | The classic reference for Bayes filters, EKF/UKF/particle filters, SLAM | OK\* |
| probabilistic-robotics.org (book website) | http://www.probabilistic-robotics.org/ | book site | - | - | - | **FAIL** (404 / connection failure) |
| Cyrill Stachniss teaching page (Uni Bonn) | https://www.ipb.uni-bonn.de/teaching/ | course hub | Free | Int-Adv | Index of all courses with slides and videos | OK |
| Stachniss, YouTube channel / playlists | https://www.youtube.com/@CyrillStachniss/playlists | video lectures | Free | Int-Adv | Exceptionally clear 5-minute and full-length lectures on SLAM, ICP, Kalman/particle filters, photogrammetry | OK |
| Stachniss, Mobile Sensing and Robotics 2 (2021), course page | https://www.ipb.uni-bonn.de/msr2-2021/index.html | course | Free | Adv | Graph SLAM, bundle adjustment, localization. Playlist: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6 | OK |
| Stachniss, Mobile Sensing and Robotics 1 (2021) | https://www.ipb.uni-bonn.de/msr1-2021/index.html | course | Free | Int | Sensors, state estimation basics | OK |
| Stachniss, Self-Driving Cars (Uni Bonn) | https://www.ipb.uni-bonn.de/sdc-2021/index.html | course | Free | Int-Adv | Localization, mapping, planning for AVs. Playlist: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQo2Z_ogbonywTg8jxCI9pD | OK |
| Peter Corke, Robotics, Vision and Control (RVC 3e, Python and MATLAB editions) | https://petercorke.com/rvc/ | book | Paid (Springer) | Beg-Int | Very hands-on bridge between math and code. The 3e-Python edition pairs with the toolbox below. Redirects to /rvc2/home/ | OK |
| Robotics Toolbox for Python (Corke) | https://github.com/petercorke/robotics-toolbox-python | library | Free | Beg-Int | Quick FK/IK/Jacobians/trajectories and Swift/PyPlot visualization. Actively maintained (pushed today). Hub: https://petercorke.com/python-toolboxes/ | OK |
| QUT Robot Academy (Corke) | https://robotacademy.net.au/ | video course | Free | Beg-Int | Short university-level video lessons: spatial math, kinematics, vision | OK |
| Introduction to Autonomous Mobile Robots, 2nd ed. (Siegwart, Nourbakhsh, Scaramuzza), MIT Press | https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/ | book | Paid | Int | Locomotion, sensors, localization, planning for wheeled robots. 2011 edition, still the standard intro | OK\* |
| ETH Autonomous Mobile Robots (ASL, Siegwart), course page | https://asl.ethz.ch/education/lectures/autonomous_mobile_robots.html | course | Free info | Int | Companion lecture to the Siegwart book | OK |
| ETH Programming for Robotics - ROS (RSL) | https://rsl.ethz.ch/education-students/lectures/ros.html | course | Free ZIP | Beg-Int | Compact 5-lecture ROS 2 bootcamp (2026 edition). Public ZIP: https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ros2026/ROS_COURSE_material_2026.zip ; the rest of the material is on ETH Moodle | OK |
| Articulated Robotics (Josh Newans) | https://articulatedrobotics.xyz/tutorials/ | tutorial + video | Free | Beg | Best free "build a real diff-drive ROS 2 robot" narrative (URDF, Gazebo, ros2_control, SLAM, Nav2). **Assessment:** excellent pedagogy, but the mobile-robot series was built on Ubuntu 20.04/ROS 2 Foxy with Gazebo Classic. Expect to translate to Jazzy/Lyrical and gz-sim. Use it for concepts, and the official docs for commands. YouTube: https://www.youtube.com/@ArticulatedRobotics | OK |
| The Construct | https://www.theconstruct.ai/ | platform | **Paid** (some free content; pricing page https://www.theconstruct.ai/pricing/) | Beg-Int | Browser-based ROS/ROS 2/Gazebo/Isaac Lab sandboxes and structured masterclasses. Optional; everything here is available free elsewhere | OK |
| Duckietown | https://duckietown.com/ | platform + docs | Free docs; hardware paid | Beg-Int | Complete, affordable autonomy curriculum on small robots. Docs: https://docs.duckietown.com/ ; edX course: https://www.edx.org/learn/technology/eth-zurich-self-driving-cars-with-duckietown | OK |

## 2. ROS 2

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| ROS 2 Distributions page (Lyrical/Kilted/Jazzy/Humble/Rolling) | https://docs.ros.org/en/jazzy/Releases.html | docs | Free | Beg | Choosing a distro; EOL dates | OK |
| Lyrical Luth release notes (LTS to May 2031) | https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html | docs | Free | Int | What changed in the newest LTS | OK |
| ROS 2 Jazzy Tutorials index | https://docs.ros.org/en/jazzy/Tutorials.html | docs | Free | Beg-Int | The canonical hands-on path | OK |
| Beginner: CLI tools (nodes, topics, services, params, actions, bags) | https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html | tutorial | Free | Beg | First week of ROS 2 | OK |
| Beginner: Client libraries (workspace, colcon, pub/sub, services, interfaces) | https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html | tutorial | Free | Beg | Writing Python/C++ nodes | OK |
| Intermediate: tf2 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html | tutorial | Free | Int | Frames, broadcasters/listeners, time travel | OK |
| Intermediate: URDF | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html | tutorial | Free | Int | Robot description, xacro, robot_state_publisher | OK |
| Intermediate: Launch | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html | tutorial | Free | Int | Python launch files | OK |
| Setting up a robot simulation (Gazebo) | https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html | tutorial | Free | Int | ROS 2 and Gazebo integration | OK |
| Lyrical equivalents (new layout): Get Started / tf2 / QoS | https://docs.ros.org/en/lyrical/Get-Started.html ; https://docs.ros.org/en/lyrical/ROS-Framework/client-libraries/Working-with-Client-Libraries/Tf2/Tf2-Main.html ; https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/topics/About-Quality-of-Service-Settings.html | docs | Free | Beg-Int | Use these if the course standardizes on Lyrical | OK |
| ROS 2 Concepts (Jazzy) | https://docs.ros.org/en/jazzy/Concepts.html | docs | Free | Beg-Int | Basic, intermediate and advanced concept pages. The Rolling URL `/en/rolling/Concepts.html` returns 404 | OK |
| About QoS settings | https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html | docs | Free | Int | Reliability, durability, history; sensor-data profiles | OK |
| About middleware vendors (DDS/Zenoh RMWs) | https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html | docs | Free | Int | Choosing an RMW | OK |
| design.ros2.org, all design articles | https://design.ros2.org/ | design docs | Free | Int-Adv | The "why" behind ROS 2 | OK |
| ROS on DDS | https://design.ros2.org/articles/ros_on_dds.html | design doc | Free | Int-Adv | How and why DDS was adopted | OK |
| ROS 2 QoS policies (design) | https://design.ros2.org/articles/qos.html | design doc | Free | Int-Adv | QoS rationale | OK |
| Nav2 documentation (Jazzy) | https://docs.nav2.org/jazzy/ | docs | Free | Int | Navigation stack. Root https://docs.nav2.org/ redirects to /rolling/ | OK |
| Nav2 navigation concepts | https://docs.nav2.org/jazzy/getting_started/navigation_concepts/ | docs | Free | Int | Servers, BTs, costmaps, state estimation (REP-105 frames) | OK |
| Nav2 first-time robot setup guide | https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/ | tutorial | Free | Int | TF, URDF, odometry, sensors, footprint for your own robot | OK |
| MoveIt 2 docs + tutorials (main = Rolling) | https://moveit.picknik.ai/main/doc/tutorials/tutorials.html | docs | Free | Int | Motion planning for arms. `/jazzy/` path returns 404; use main and pick the matching branch | OK |
| MoveIt 2 quickstart in RViz | https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html | tutorial | Free | Int | First MoveIt session | OK |
| ros2_control docs (Jazzy) | https://control.ros.org/jazzy/index.html | docs | Free | Int | Hardware interfaces, controllers. Demos: https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html | OK |
| gz_ros2_control | https://github.com/ros-controls/gz_ros2_control | code/docs | Free | Int | Simulated ros2_control hardware in Gazebo | OK |
| Gazebo docs, Harmonic (for Jazzy) | https://gazebosim.org/docs/harmonic/tutorials/ | docs | Free | Beg-Int | Simulation tutorials. `latest` = Jetty (for Lyrical): https://gazebosim.org/docs/latest/tutorials/ | OK |
| Gazebo, Installing with ROS (version pairing table) | https://gazebosim.org/docs/latest/ros_installation/ | docs | Free | Beg | Humble-Fortress, Jazzy-Harmonic, Kilted-Ionic, Lyrical-Jetty | OK |
| Robotics Stack Exchange | https://robotics.stackexchange.com/ | Q&A | Free | All | Searchable ROS/robotics Q&A (ROS Answers was migrated here) | OK |
| Open Robotics Discourse (formerly discourse.ros.org) | https://discourse.openrobotics.org/ | forum | Free | All | Release announcements, WG discussions. discourse.ros.org redirects here | OK |
| REP-103 Standard Units of Measure and Coordinate Conventions | https://www.ros.org/reps/rep-0103.html | standard | Free | Beg | SI units, right-handed axes (x forward, y left, z up) | OK |
| REP-105 Coordinate Frames for Mobile Platforms | https://www.ros.org/reps/rep-0105.html | standard | Free | Int | map/odom/base_link conventions | OK |
| REP-2000 ROS 2 Releases and Target Platforms | https://www.ros.org/reps/rep-2000.html | standard | Free | Int | OS/arch tiers per distro (not yet updated for Lyrical) | OK |
| REP-145 Conventions for IMU Sensor Drivers | https://www.ros.org/reps/rep-0145.html | standard | Free | Int | IMU message and frame conventions | OK |

## 3. Math

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| 3Blue1Brown, Essence of Linear Algebra | https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab | video | Free | Beg | Geometric intuition for transforms, determinants, eigenvectors. Site: https://www.3blue1brown.com/topics/linear-algebra | OK |
| 3Blue1Brown, Essence of Calculus | https://www.youtube.com/playlist?list=PLZHQObOWTQDMsr9K-rj53DwVRMYO3t5Yr | video | Free | Beg | Derivatives and integrals intuition. Site: https://www.3blue1brown.com/topics/calculus | OK |
| Khan Academy, Trigonometry | https://www.khanacademy.org/math/trigonometry | course | Free | Beg | Filling trig gaps (atan2 thinking, unit circle) | OK\* (JS challenge page; URL confirmed by search) |
| Khan Academy, Probability | https://www.khanacademy.org/math/statistics-probability/probability-library | course | Free | Beg | Probability basics before Bayes filters | OK\* |
| MIT 18.06 Linear Algebra (Strang), Spring 2010 | https://ocw.mit.edu/courses/18-06-linear-algebra-spring-2010/ | course + video | Free | Int | Full university linear algebra | OK |
| MIT 18.06SC (scholar version, with recitations) | https://ocw.mit.edu/courses/18-06sc-linear-algebra-fall-2011/ | course | Free | Int | Self-study-friendly version with problem sets | OK |
| Immersive Linear Algebra | https://immersivemath.com/ila/index.html | interactive book | Free | Beg-Int | Interactive 3D figures | OK |
| Visualizing quaternions (Ben Eater + 3Blue1Brown) | https://eater.net/quaternions | interactive video | Free | Int | Best intuition for quaternions | OK |
| 3Blue1Brown, Quaternions and 3D rotation | https://www.3blue1brown.com/lessons/quaternions-and-3d-rotation | video/lesson | Free | Int | Companion to the interactive series | OK |
| Solà, "Quaternion kinematics for the error-state Kalman filter" (arXiv:1711.02508) | https://arxiv.org/abs/1711.02508 | paper | Free | Adv | The reference for quaternion conventions and IMU ESKF math | OK |
| Solà, Deray, Atchuthan, "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537) | https://arxiv.org/abs/1812.01537 | paper | Free | Adv | SO(3)/SE(3) Jacobians made practical (manif library) | OK |
| KalmanFilter.net (Alex Becker) | https://kalmanfilter.net/ | tutorial | Free (book paid) | Beg-Int | Numerical-example-driven KF/EKF/UKF intro | OK |
| Kalman and Bayesian Filters in Python (Roger Labbe) | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python | book (Jupyter) | Free | Int | Hands-on filters with code (filterpy). Repo static since Aug 2024 but still the best | OK |
| Seeing Theory (Brown University) | https://seeing-theory.brown.edu/ | interactive | Free | Beg | Visual probability and statistics | OK |
| Convex Optimization (Boyd & Vandenberghe) | https://web.stanford.edu/~boyd/cvxbook/ | book | Free PDF (https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf) | Adv | Foundations for MPC and trajectory optimization | OK |

## 4. Electronics

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| SparkFun, Voltage, Current, Resistance, and Ohm's Law | https://learn.sparkfun.com/tutorials/voltage-current-resistance-and-ohms-law | tutorial | Free | Beg | First principles | OK |
| SparkFun, Pulse Width Modulation | https://learn.sparkfun.com/tutorials/pulse-width-modulation | tutorial | Free | Beg | Motor speed and servo control basics | OK |
| SparkFun, I2C | https://learn.sparkfun.com/tutorials/i2c | tutorial | Free | Beg | IMUs and sensors bus | OK |
| SparkFun, Serial Peripheral Interface (SPI) | https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi | tutorial | Free | Beg | Fast sensor/display buses | OK |
| SparkFun, Serial Communication | https://learn.sparkfun.com/tutorials/serial-communication | tutorial | Free | Beg | UART between Pi and MCU | OK |
| SparkFun, Pull-up Resistors | https://learn.sparkfun.com/tutorials/pull-up-resistors | tutorial | Free | Beg | Buttons, I2C lines, floating inputs | OK |
| SparkFun, How to Use a Multimeter | https://learn.sparkfun.com/tutorials/how-to-use-a-multimeter | tutorial | Free | Beg | Debugging hardware safely | OK |
| SparkFun, How to Solder: Through-Hole | https://learn.sparkfun.com/tutorials/how-to-solder-through-hole-soldering | tutorial | Free | Beg | Soldering headers and connectors | OK |
| Adafruit, Li-Ion & LiPoly Batteries | https://learn.adafruit.com/li-ion-and-lipoly-batteries | guide | Free | Beg | Chemistry, voltages, charging, protection, care | OK |
| Adafruit, Motor Selection Guide | https://learn.adafruit.com/adafruit-motor-selection-guide | guide | Free | Beg | DC vs. servo vs. stepper; choosing drivers | OK |
| Adafruit, I2C Addresses and Troublesome Chips | https://learn.adafruit.com/i2c-addresses | reference | Free | Beg-Int | Resolving I2C address conflicts | OK |
| All About Circuits, textbook (DC, AC, Semiconductors, Digital, Reference) | https://www.allaboutcircuits.com/textbook/ | book | Free | Beg-Int | Deeper theory reference (Kuphaldt's Lessons in Electric Circuits) | OK\* |
| Raspberry Pi docs, Raspberry Pi hardware (incl. Pi 5 power: 5 V/5 A USB-C, 3 A mode limits USB peripherals to 600 mA) | https://www.raspberrypi.com/documentation/computers/raspberry-pi.html | docs | Free | Beg | Board specs, power, GPIO header | OK\* (content fetched) |
| gpiozero docs | https://gpiozero.readthedocs.io/ | docs | Free | Beg | Pythonic GPIO on Pi (motors, PWM, buttons) | OK |
| pinout.xyz | https://pinout.xyz/ | reference | Free | Beg | Interactive 40-pin header map | OK |
| Raspberry Pi docs, Microcontrollers (Pico series, MicroPython, C/C++ SDK, Debug Probe) | https://www.raspberrypi.com/documentation/microcontrollers/ | docs | Free | Beg-Int | Pico/Pico 2 (RP2040/RP2350) entry point | OK\* |
| MicroPython docs, machine.PWM | https://docs.micropython.org/en/latest/library/machine.PWM.html | docs | Free | Beg | PWM on Pico | OK |
| MicroPython docs, rp2 module (PIO) | https://docs.micropython.org/en/latest/library/rp2.html | docs | Free | Int | PIO state machines, e.g. encoder decoding | OK |
| MicroPython, RP2 quick reference | https://docs.micropython.org/en/latest/rp2/quickref.html | docs | Free | Beg | Pins, PWM, I2C, UART on Pico | OK |
| MicroPython, machine.Encoder | https://docs.micropython.org/en/latest/library/machine.Encoder.html | docs | Free | Int | **ESP32/MIMXRT only, not rp2.** Use PIO on Pico | OK (caveat) |
| pico-examples, PIO quadrature encoder (C) | https://github.com/raspberrypi/pico-examples/tree/master/pio/quadrature_encoder | code | Free | Int | Reliable high-rate encoder counting on RP2040/RP2350 | OK |
| Pololu, Brushed DC Motor Drivers | https://www.pololu.com/category/11/brushed-dc-motor-drivers | product docs | Free docs | Beg-Int | Driver selection; each product page has pinouts and current limits | OK |
| Pololu, example gearmotor with 48 CPR encoder (25D, 47:1) | https://www.pololu.com/product/4865 | product docs | Free docs | Beg-Int | Encoder wiring, CPR math, dimensions. Resources tab: https://www.pololu.com/product/4865/resources | OK |
| Pololu, Documentation index | https://www.pololu.com/docs | docs | Free | Int | User guides (e.g. motor controllers) | OK |
| Battery University, BU-304a Safety Concerns with Li-ion | https://batteryuniversity.com/article/bu-304a-safety-concerns-with-li-ion | article | Free | Beg | Thermal runaway, safe handling. Also BU-808 on prolonging battery life: https://batteryuniversity.com/article/bu-808-how-to-prolong-lithium-based-batteries | OK |
| The Art of Electronics, 3rd ed. (Horowitz & Hill) | https://artofelectronics.net/ | book | **Paid** | Int-Adv | The practitioner's bible for analog/digital design | OK |
| Ben Eater | https://eater.net/ | video + kits | Free videos | Beg-Int | Bottom-up digital electronics intuition (8-bit breadboard computer, 6502) | OK |

## 5. Control

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| MATLAB Tech Talks, Understanding PID Control (Brian Douglas, 7 videos) | https://www.mathworks.com/videos/series/understanding-pid-control.html | video | Free | Beg-Int | Anti-windup, derivative filtering, tuning. YouTube: https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y | OK\* (content fetched) |
| MATLAB Tech Talks, Control Systems in Practice | https://www.youtube.com/playlist?list=PLn8PRpmsu08pFBqgd_6Bi7msgkWFKL33b | video | Free | Int | Real-world control engineering issues | OK |
| Brian Douglas, Control System Lectures: Classical Control Theory | https://www.youtube.com/playlist?list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk | video | Free | Int | Root locus, Bode, Nyquist intuition. All lectures: https://www.youtube.com/playlist?list=PLUMWjy5jgHK3j74Z5Tq6Tso1fSfVWZC8L | OK |
| Feedback Systems, 2nd ed. (Åström & Murray) | https://www.cds.caltech.edu/~murray/FBS/Second_Edition.html | book | Free online (Princeton UP) | Int | Rigorous but readable control textbook | OK |
| fbswiki.org (original FBS wiki) | https://fbswiki.org/ | book site | - | - | - | **FAIL** (connection failure today; use the Caltech link above) |
| Brett Beauregard, "Improving the Beginner's PID" series | http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ | tutorial | Free | Beg-Int | Turning textbook PID into robust embedded code (sample time, derivative kick, windup, on/off) | OK |
| Steve Brunton, Control Bootcamp | https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m | video | Free | Int-Adv | State space, LQR, Kalman filter, observability | OK |

## 6. Computer vision

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| OpenCV-Python Tutorials | https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html | docs | Free | Beg-Int | Practical image processing, features, video. `4.x` redirects to 4.13.0 | OK |
| OpenCV, Camera Calibration (Python) | https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html | tutorial | Free | Int | Intrinsics and distortion from chessboards | OK |
| ROS camera_calibration package docs | https://docs.ros.org/en/jazzy/p/camera_calibration/ | docs | Free | Int | Calibrating ROS cameras (CameraInfo) | OK |
| Szeliski, Computer Vision: Algorithms and Applications, 2nd ed. | https://szeliski.org/Book/ | book | Free PDF for personal use | Int-Adv | Broad modern CV reference | OK |
| Stanford CS231n, Deep Learning for Computer Vision | https://cs231n.stanford.edu/ | course | Free materials | Int | CNNs/ViTs from scratch. Notes: https://cs231n.github.io/ | OK |
| Hartley & Zisserman, Multiple View Geometry | https://www.robots.ox.ac.uk/~vgg/hzbook/ | book | **Paid** | Adv | Epipolar geometry, homographies, bundle adjustment | OK |
| First Principles of Computer Vision (Shree Nayar, Columbia) | https://fpcv.cs.columbia.edu/ | video course | Free | Beg-Int | Beautifully explained classical CV (imaging, features, stereo, SfM). YouTube: https://www.youtube.com/@firstprinciplesofcomputerv3258 | OK |
| Ultralytics YOLO docs | https://docs.ultralytics.com/ | docs | Free docs (AGPL-3.0 / commercial license) | Beg-Int | Fastest route to detection/segmentation/pose on robots. Training: https://docs.ultralytics.com/modes/train | OK |
| Hugging Face Transformers, vision task guides | https://huggingface.co/docs/transformers/tasks/image_classification | docs | Free | Int | Fine-tuning ViT/DETR etc. Object detection: https://huggingface.co/docs/transformers/tasks/object_detection | OK |
| Hugging Face Community Computer Vision Course | https://huggingface.co/learn/computer-vision-course/unit0/welcome/welcome | course | Free | Beg-Int | Broad modern CV with HF tooling | OK |
| Roboflow notebooks | https://github.com/roboflow/notebooks | notebooks | Free | Beg-Int | **Assessment:** ~9.7k stars, active (Aug 2026). Good copy-paste fine-tuning recipes for the latest detectors and VLMs, but vendor-oriented (pushes the Roboflow platform) and thin on theory. Use as recipes, not as a course | OK |

## 7. ML / Deep learning

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| fast.ai, Practical Deep Learning for Coders | https://course.fast.ai/ | course | Free | Beg-Int | Top-down, practical DL | OK |
| Karpathy, Neural Networks: Zero to Hero | https://karpathy.ai/zero-to-hero.html | video course | Free | Int | Backprop to GPT from scratch. Playlist: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ ; code: https://github.com/karpathy/nn-zero-to-hero | OK |
| Dive into Deep Learning (d2l.ai) | https://d2l.ai/ | book | Free | Int | Interactive textbook with PyTorch code | OK |
| Stanford CS229 Machine Learning | https://cs229.stanford.edu/ | course | Free materials | Int-Adv | Classical ML theory | OK |
| Stanford CS224N NLP with Deep Learning | https://web.stanford.edu/class/cs224n/ | course | Free materials | Int-Adv | Transformers and LLM foundations | OK |
| Stanford CS336 Language Modeling from Scratch | https://cs336.stanford.edu/ | course | Free materials | Adv | Building and training LMs end-to-end | OK |
| PyTorch Tutorials | https://docs.pytorch.org/tutorials/ | docs | Free | Beg-Int | Official PyTorch how-tos | OK |
| Hugging Face Learn (LLM, Deep RL, CV, Robotics courses) | https://huggingface.co/learn | course hub | Free | Beg-Int | Ecosystem-native courses. LLM course: https://huggingface.co/learn/llm-course | OK |

## 8. Reinforcement learning

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| Sutton & Barto, Reinforcement Learning: An Introduction, 2nd ed. | http://incompleteideas.net/book/the-book-2nd.html | book | Free PDF (http://incompleteideas.net/book/RLbook2020.pdf) | Int | The foundational text | OK |
| OpenAI Spinning Up in Deep RL | https://spinningup.openai.com/ | docs/tutorial | Free | Int | Concise deep-RL theory + key papers list. Code is dated (repo last push Aug 2024); read for concepts | OK |
| David Silver, UCL/DeepMind RL course | https://www.youtube.com/playlist?list=PLqYmG7hTraZDM-OYHWgPebj2MfCFzFObQ | video | Free | Int | Classic RL lectures. Slides: https://www.davidsilver.uk/teaching/ (redirects to davidstarsilver.wordpress.com) | OK |
| Hugging Face Deep RL Course | https://huggingface.co/learn/deep-rl-course | course | Free | Beg-Int | Hands-on with SB3, CleanRL-style code, Unity envs | OK |
| Gymnasium docs (Farama) | https://gymnasium.farama.org/ | docs | Free | Beg | Standard env API (successor of OpenAI Gym) | OK |
| Stable-Baselines3 docs | https://stable-baselines3.readthedocs.io/ | docs | Free | Beg-Int | Reliable PPO/SAC/TD3 baselines (active) | OK |
| CleanRL docs | https://docs.cleanrl.dev/ | docs + code | Free | Int | Single-file readable algorithm implementations | OK |
| Berkeley CS 185/285 Deep RL (Levine) | https://rail.eecs.berkeley.edu/deeprlcourse/ | course | Free slides; Fall 2023 videos on YouTube | Adv | The deepest deep-RL course incl. imitation and model-based RL | OK |

## 9. Embodied AI / robot learning

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| LeRobot docs (Hugging Face) | https://huggingface.co/docs/lerobot/index | docs | Free | Int | Datasets, ACT/Diffusion/VLA policies, real robots | OK |
| LeRobot GitHub | https://github.com/huggingface/lerobot | code | Free | Int | Very active (pushed today) | OK |
| LeRobot, SO-101 arm guide | https://huggingface.co/docs/lerobot/so101 | docs | Free (hardware ~$) | Int | Low-cost arm build and teleop | OK |
| LeRobot, Imitation learning on real-world robots | https://huggingface.co/docs/lerobot/il_robots | tutorial | Free | Int | Record, train, evaluate a policy | OK |
| Hugging Face Robotics Course | https://huggingface.co/learn/robotics-course | course | Free | Beg-Int | New LeRobot-based course; classical to robot learning. Units 5-7 (RL, IL, foundation models) marked "coming soon" | OK |
| "Robot Learning: A Tutorial" (Capuano et al., HF, arXiv:2510.12403) | https://arxiv.org/abs/2510.12403 | paper/tutorial | Free | Int | Modern robot learning (RL, BC, VLAs) with LeRobot examples. Web version: https://huggingface.co/spaces/lerobot/robot-learning-tutorial | OK |
| Stanford CS 224R Deep Reinforcement Learning (Finn), Spring 2026 | https://cs224r.stanford.edu/ | course | Free slides; Spring 2025 videos on YouTube | Adv | Imitation, offline RL, multi-task/meta RL for robots | OK |
| ETH Zurich Robot Learning (Oier Mees), Spring 2026 | https://cvg.ethz.ch/lectures/Robot-Learning/ | course | Free slides, videos, homework | Int-Adv | **Best fully open current robot-learning course:** IL, RL, generative policies, world models, VLAs. HW: https://github.com/mees-robot-learning-course/ethz-course-2026 | OK |
| Cornell CS 4756/5756 Robot Learning, Spring 2026 | https://www.cs.cornell.edu/courses/cs4756/2026sp/ | course | Free slides | Int | IL, RL, MPC for robots | OK |
| CMU 16-831 Introduction to Robot Learning (Held), Spring 2026 | https://sites.google.com/view/16-831-cmu/home | course | Slides via Dropbox link | Int-Adv | RL/IL/visual learning for robots. Old `cs.cmu.edu/~16831-f24/` is **FAIL** (404) | OK |
| MIT Robotic Manipulation (Tedrake) | https://manipulation.csail.mit.edu/ | course notes | Free | Int-Adv | Includes learning-based manipulation chapters (see §1) | OK |
| MuJoCo docs | https://mujoco.readthedocs.io/ | docs | Free | Int | Physics sim used by most robot-learning work | OK |
| Isaac Lab docs | https://isaac-sim.github.io/IsaacLab/ | docs | Free (NVIDIA GPU) | Int-Adv | GPU-parallel RL for robots | OK |

## 10. Linux & developer tools

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| Ubuntu Server documentation | https://ubuntu.com/server/docs | docs | Free | Beg-Int | Headless Pi/robot OS setup, networking, services. documentation.ubuntu.com/server redirects here | OK |
| Ubuntu, The Linux command line for beginners | https://ubuntu.com/tutorials/command-line-for-beginners | tutorial | Free | Beg | Quick shell refresher | OK |
| The Linux Command Line (William Shotts) | https://linuxcommand.org/tlcl.php | book | Free PDF (CC BY-NC-ND; 7th Internet Ed. 25.12A) | Beg-Int | Complete shell and scripting book | OK |
| MIT Missing Semester (2026 edition) | https://missing.csail.mit.edu/ | course | Free | Beg-Int | Shell, git, debugging, packaging; 2026 adds agentic coding | OK |
| Docker docs | https://docs.docker.com/ | docs | Free | Beg-Int | Containerized ROS dev. Get started: https://docs.docker.com/get-started/ | OK |
| Pro Git book | https://git-scm.com/book/en/v2 | book | Free | Beg-Int | Git internals and workflows | OK |
| Python Packaging User Guide | https://packaging.python.org/ | docs | Free | Int | pyproject.toml, wheels | OK |
| uv docs (Astral) | https://docs.astral.sh/uv/ | docs | Free | Beg-Int | Fast Python env and dependency management | OK |
| pytest docs | https://docs.pytest.org/ | docs | Free | Beg-Int | Testing robot software | OK |

## 11. Embedded & C++

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| MicroPython docs | https://docs.micropython.org/en/latest/ | docs | Free | Beg | Python on Pico/ESP32 | OK |
| Arduino docs | https://docs.arduino.cc/ | docs | Free | Beg | Arduino boards and libraries. Learn hub: https://docs.arduino.cc/learn/ | OK |
| Raspberry Pi Pico C/C++ SDK docs | https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html | docs | Free | Int | Native firmware for RP2040/RP2350. SDK repo: https://github.com/raspberrypi/pico-sdk ; examples: https://github.com/raspberrypi/pico-examples ; PDF: https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf | OK\* (PDF and repos OK) |
| learncpp.com | https://www.learncpp.com/ | tutorial | Free | Beg-Int | Best free modern C++ course | OK |
| cppreference.com | https://en.cppreference.com/ | reference | Free | All | Authoritative std library reference | OK |
| Stachniss, Modern C++ for Computer Vision (Uni Bonn) | https://www.ipb.uni-bonn.de/teaching/cpp-2020/index.html | course | Free | Int | C++ for robotics/CV engineers (CMake, tooling) | OK |
| Making Embedded Systems, 2nd ed. (Elecia White, O'Reilly 2024) | https://www.oreilly.com/library/view/making-embedded-systems/9781098151539/ | book | **Paid** | Int | Embedded architecture, motors, debugging patterns. Companion repo: https://github.com/eleciawhite/making-embedded-systems | OK\* |
| micro-ROS (now at micro.vulcanexus.org) | https://micro.vulcanexus.org/ | docs | Free | Int-Adv | ROS 2 on microcontrollers. Old https://micro.ros.org/ shows "has moved" | OK |
| micro-ROS for Raspberry Pi Pico SDK | https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk | code | Free | Int-Adv | Pico to ROS 2 bridge starting point | OK |

## 12. Agents & LLMs for robots

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| Anthropic Claude, Tool use overview | https://platform.claude.com/docs/en/agents-and-tools/tool-use/overview | docs | Free docs | Int | Function/tool calling with Claude. Define tools: https://platform.claude.com/docs/en/agents-and-tools/tool-use/define-tools | OK |
| OpenAI, Function calling | https://developers.openai.com/api/docs/guides/function-calling | docs | Free docs | Int | OpenAI tool calling (platform.openai.com URL redirects here) | OK |
| Model Context Protocol, intro | https://modelcontextprotocol.io/ | docs | Free | Int | Exposing robot capabilities as MCP tools | OK |
| MCP Specification (latest = 2026-07-28) | https://modelcontextprotocol.io/specification/latest | spec | Free | Adv | Protocol details (resolves to /specification/2026-07-28) | OK |
| BehaviorTree.CPP docs | https://www.behaviortree.dev/ | docs | Free | Int | BT engine used by Nav2; Groot visual editor | OK |
| py_trees docs | https://py-trees.readthedocs.io/ | docs | Free | Int | Python behavior trees. ROS 2 tutorials: https://py-trees-ros-tutorials.readthedocs.io/ | OK |
| Nav2, Behavior Trees | https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/ | docs | Free | Int | Nav2 BT XML, nodes, example trees (old `/behavior_trees/index.html` path is **FAIL**, "Page moved") | OK |

## 13. Safety

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| ISO 10218-1:2025 Robotics, Safety requirements, Part 1: Industrial robots | https://www.iso.org/standard/73933.html | standard (overview page) | **Paid** standard | Adv | Robot-level safety requirements (replaces 2011 ed.) | OK\* |
| ISO 10218-2:2025 Part 2: Industrial robot applications and robot cells | https://www.iso.org/standard/73934.html | standard (overview page) | **Paid** | Adv | Integration/cell safety; **now contains the former ISO/TS 15066 collaborative-application content** | OK\* |
| ISO/TS 15066:2016 Collaborative robots | https://www.iso.org/standard/62996.html | standard (overview page) | Paid | Adv | Historical; body-region force/pressure limits, superseded by the 10218-2:2025 integration | OK\* |
| A3, Updated ISO 10218 FAQ | https://www.automate.org/robotics/blogs/updated-iso-10218-faq | article | Free | Int | Plain-language summary of what changed in 2025 | OK\* |
| OSHA, Robotics safety topic page | https://www.osha.gov/robotics | guidance | Free | Beg | Hazards, standards, US regulatory context. Hazards: https://www.osha.gov/robotics/hazards | OK |
| OSHA Technical Manual Sec. IV Ch. 4, Industrial Robot Systems and Industrial Robot System Safety | https://www.osha.gov/otm/section-4-safety-hazards/chapter-4 | guidance | Free | Int | Safeguarding, risk assessment (updated 2021) | OK\* |
| Battery University, BU-304a Li-ion safety | https://batteryuniversity.com/article/bu-304a-safety-concerns-with-li-ion | article | Free | Beg | Lithium battery hazards | OK |
| Adafruit, Li-Ion & LiPoly Batteries (care and protection) | https://learn.adafruit.com/li-ion-and-lipoly-batteries | guide | Free | Beg | Hobby-scale LiPo handling. (`learn.adafruit.com/lipo-battery-care` does not exist, 404) | OK |
| ROS 2 security, Setting up security (SROS2) | https://docs.ros.org/en/jazzy/Tutorials/Advanced/Security/Introducing-ros2-security.html | tutorial | Free | Int-Adv | Keystores, enclaves, encrypted DDS. Lyrical path: `Developer-Tools/Introspection-and-analysis/Security/Introducing-ros2-security.html` | OK |
| ROS 2 DDS-Security integration (design) | https://design.ros2.org/articles/ros2_dds_security.html | design doc | Free | Adv | Security architecture rationale. Tool repo: https://github.com/ros2/sros2 | OK |

## 14. 3D printing & CAD

| Title | URL | Type | Cost | Level | Best for | Chk |
|---|---|---|---|---|---|---|
| Onshape Learning Center | https://learn.onshape.com/ | course | Free | Beg-Int | Structured CAD courses. Fundamentals: https://learn.onshape.com/learn/course/fundamentals-cad | OK |
| Onshape Free plan | https://www.onshape.com/en/products/free | product terms | Free (non-commercial; documents are public) | Beg | Browser CAD that works on Linux; pairs with onshape-to-robot | OK |
| FreeCAD documentation wiki | https://wiki.freecad.org/ | docs | Free (open source) | Beg-Int | Offline, fully open CAD. Getting started: https://wiki.freecad.org/Getting_started | OK |
| Autodesk Fusion for Personal Use (compare/terms) | https://www.autodesk.com/products/fusion-360/personal | product terms | Free for qualifying users | Beg-Int | Terms: home-based non-commercial use, under US$1,000/yr revenue, 3-year renewable license, reduced features | OK\* |
| Prusa Knowledge Base | https://help.prusa3d.com/ | docs | Free | Beg-Int | Printer troubleshooting, design for printing: https://help.prusa3d.com/article/modeling-with-3d-printing-in-mind_164135 ; materials: https://help.prusa3d.com/filament-material-guide | OK |
| Bambu Lab Wiki | https://wiki.bambulab.com/ | docs | Free | Beg | Printer ops. Filament/plate/nozzle table: https://wiki.bambulab.com/en/general/filament-guide-material-table | OK |
| Printables (Prusa) | https://www.printables.com/ | model repository | Free | Beg | Robot parts, brackets, wheels | OK\* (Cloudflare) |
| Thingiverse | https://www.thingiverse.com/ | model repository | Free | Beg | Large legacy model library | OK |
| onshape-to-robot docs | https://onshape-to-robot.readthedocs.io/ | docs | Free (open source) | Int | Onshape assembly to URDF / SDF / MuJoCo. Needs Onshape API keys. Actively maintained (repo pushed Aug 2026): https://github.com/Rhoban/onshape-to-robot | OK |
| fusion2urdf (syuntoku14), original | https://github.com/syuntoku14/fusion2urdf | code | Free | Int | **Status:** original exporter, ROS 1-era package output; not archived, last push Mar 2025. Prefer the ROS 2 fork below | OK |
| fusion360-urdf-ros2 (runtimerobotics) | https://github.com/runtimerobotics/fusion360-urdf-ros2 | code | Free | Int | Most current Fusion exporter: tested on Humble and Jazzy, Gazebo Harmonic/Classic; ros2_control and Gazebo plugins must be added by hand. Last push Nov 2025. Derived from syuntoku14 and dheena2k2/fusion2urdf-ros2 (last push Jan 2024) | OK |

---

## Failed verification summary

| URL | Result | Replacement |
|---|---|---|
| http://www.probabilistic-robotics.org/ | 404 / connection failure | https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ |
| https://fbswiki.org/ | connection failure | https://www.cds.caltech.edu/~murray/FBS/Second_Edition.html |
| http://www.cds.caltech.edu/~murray/amwiki/index.php/Main_Page | 404 | same as above |
| https://www.cs.cmu.edu/~16831-f24/ (also f23, f25) | 404 | https://sites.google.com/view/16-831-cmu/home |
| https://docs.nav2.org/behavior_trees/index.html | "Page moved" 404 | https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/ |
| https://docs.ros.org/en/rolling/Concepts.html, /en/rolling/Tutorials.html, /en/lyrical/Tutorials.html | 404 (docs restructured) | Jazzy paths above, or the Lyrical `Get-Started` / `ROS-Framework` paths |
| https://moveit.picknik.ai/jazzy/index.html | 404 | https://moveit.picknik.ai/main/index.html |
| https://micro.ros.org/ (docs paths) | "has moved"; deep links 404 | https://micro.vulcanexus.org/ |
| https://learn.adafruit.com/lipo-battery-care | 404 | https://learn.adafruit.com/li-ion-and-lipoly-batteries |
| https://robotacademy.net.au/masterclass/ | 404 | https://robotacademy.net.au/ |
| https://petercorke.com/books/robotics-vision-control-python/ | 404 | https://petercorke.com/rvc/ |
