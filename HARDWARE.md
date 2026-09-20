# Hardware

What to buy, when, and where, for a student living in **Israel**. The detailed per-item pages are
in [`hardware/`](hardware/README.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT.** Israeli shop prices include 18% VAT.
> Imported items are shown as the foreign list price, and in ₪ **with 18% import VAT but without
> shipping**. 1 USD = ₪3.033 (Bank of Israel, 2026-09-16). Stock and prices change daily, and
> import rules changed three times in 2025–26: **check before you order.**
> Evidence: `references/research/israel-hardware-stage1-2026-09.md`,
> `references/research/israel-hardware-stages2-5-2026-09.md`,
> `references/research/software-versions-2026-09.md`.

> [!CAUTION]
> Stage 1 introduces a lithium-ion battery pack. Read [SAFETY.md](SAFETY.md) before the parts
> arrive: fuse the pack, charge only with the proper charger on a non-flammable surface or in a
> LiPo bag, never unattended. **Buy cells and packs in Israel**; lithium rarely ships here.

## The buying philosophy: don't buy everything at once

1. **Buy per stage, when the lessons need it.** Each stage below lists the lessons it unlocks.
   Everything before those lessons runs on your laptop or in simulation.
2. **Start with the cheapest thing that teaches the concept.** Upgrade only when you've
   *measured* a limitation (Pi 5 too slow, battery runtime too short, gripper too weak).
3. **Buy locally when the price gap is small** (warranty, fast replacements, no import VAT or
   courier risk). Import when the saving is large (the RPLIDAR C1 is ≈ 4× cheaper abroad) or
   there's no local seller (depth cameras, arm kits).
4. **Keep cheap spares of the parts beginners break**: Pico, motor driver, fuses, one motor.
5. **Stage 4 and the 3D printer are optional.** No lesson requires a Jetson or a depth camera.

## Stages at a glance

Labels: **Buy now** = when you start the stage · **Optional** = not required by any lesson, or only
on the recommended path · **Buy later** = wait until the named lesson.

| Stage | Buy now | Optional | Buy later | Unlocks (lessons) |
|---|---|---|---|---|
| **Tools** ([tools.md](hardware/tools.md)) | Multimeter, 80 W soldering iron, solder/flux, strippers, cutters, heat shrink, screwdrivers, hex keys, hot glue, LiPo bag, safety glasses | Helping hands, fume extractor, calipers | JST/Dupont crimper (before [02.07](02-robot-electronics/02.07-wiring-harness.md)) | [01.03](01-first-robot/01.03-workbench-and-tools.md), [01.06](01-first-robot/01.06-mechanical-assembly.md), [01.07](01-first-robot/01.07-power-system.md), [02.07](02-robot-electronics/02.07-wiring-harness.md)–[02.09](02-robot-electronics/02.09-electrical-debugging-method.md), [FE.03](optional-foundations/electronics/FE.03-multimeter.md), [FE.04](optional-foundations/electronics/FE.04-breadboards-and-connectors.md), [FE.17](optional-foundations/electronics/FE.17-soldering.md), [14.02](14-robotic-arm/14.02-buying-and-assembling-the-arm.md) |
| **1 — First robot** ([stage-1](hardware/stage-1-first-robot.md)) | Raspberry Pi 5 (4 GB OK / 8 GB recommended) + Active Cooler + microSD; Pico 2; 2× Yahboom 520 12 V 205 RPM encoder motors; 2× DRV8874; 90 mm wheels, 6 mm hubs, ball caster, chassis plate; 3S Samsung 35E pack + BMS + 12.6 V charger; fuse, switch, XT60; 5 V 5.5 A regulator; INA219; US-100; VL53L1X | 27 W Pi PSU, 8 GB Pi, NVMe HAT, spare Pico/driver/motor | — | [01.04](01-first-robot/01.04-raspberry-pi-setup.md)–[01.15](01-first-robot/01.15-teleop-from-laptop.md), [02.02](02-robot-electronics/02.02-power-budget.md)–[02.09](02-robot-electronics/02.09-electrical-debugging-method.md), [03.03](03-robot-software/03.03-robust-serial-communication.md), [03.10](03-robot-software/03.10-networking-for-robots.md), [04.16](04-ros2/04.16-your-robot-as-ros2-node.md), [06.09](06-simulation/06.09-same-code-sim-and-real.md), [07.01](07-sensors/07.01-sensor-fundamentals.md)–[07.04](07-sensors/07.04-tof-sensors.md), [08.01](08-control/08.01-open-vs-closed-loop.md)–[08.12](08-control/08.12-control-in-ros2.md), [09.05](09-odometry/09.05-calibrating-odometry.md), [09.07](09-odometry/09.07-odometry-in-ros2.md), [FC.02](optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md), [FC.05](optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md), [FC.06](optional-foundations/cpp-and-embedded/FC.06-pico-c-sdk.md), [FC.08](optional-foundations/cpp-and-embedded/FC.08-micro-ros.md); projects P01–P04, P06, P08 |
| **2 — IMU and camera** ([stage-2](hardware/stage-2-imu-and-camera.md)) | BNO055 IMU (ICM-20948 if out of stock); USB webcam (Logitech C920) | Raspberry Pi Camera Module 3 (advanced path: Pi OS + Docker or building libcamera) | Buy the whole stage at module 07 | [07.05](07-sensors/07.05-imu-fundamentals.md), [07.06](07-sensors/07.06-imu-in-ros2.md), [07.08](07-sensors/07.08-rgb-cameras.md), [08.11](08-control/08.11-rotate-exactly-90.md), [10.07](10-localization/10.07-fusing-imu-and-odometry.md), [10.10](10-localization/10.10-fiducial-landmarks.md), [13.06](13-computer-vision/13.06-lens-distortion-calibration.md), [13.07](13-computer-vision/13.07-fiducial-markers.md), [13.15](13-computer-vision/13.15-vision-in-ros2.md), [13.16](13-computer-vision/13.16-detection-to-map-position.md), [15.04](15-manipulation/15.04-object-pose-estimation.md); projects P05, P12 |
| **3 — LiDAR** ([stage-3](hardware/stage-3-lidar.md)) | Slamtec RPLIDAR C1 | T-mini Pro (if C1 out of stock), LDROBOT STL-19P | Buy at [07.07](07-sensors/07.07-2d-lidar.md) | [07.07](07-sensors/07.07-2d-lidar.md), [11.07](11-slam/11.07-mapping-your-room.md), [12.08](12-navigation/12.08-nav2-on-the-real-robot.md); projects P09, P10, P11, P13, P17 |
| **4 — Compute and depth** ([stage-4](hardware/stage-4-compute-and-depth.md)) | — | **Whole stage:** Jetson Orin Nano Super Developer Kit; RealSense D435i (with Jetson) or OAK-D Lite (with Pi 5) | Only if the Pi 5 is the measured bottleneck | None required. Helps [07.09](07-sensors/07.09-depth-cameras-and-point-clouds.md), [13.08](13-computer-vision/13.08-depth-and-3d-vision.md), [16.04](16-machine-learning/16.04-models-on-edge-compute.md), module 18 on-robot inference |
| **5 — Robotic arm** ([stage-5](hardware/stage-5-robotic-arm.md)) | SO-101 leader + follower kit (LeRobot) | RoArm-M3 (alternative); LeKiwi mobile-manipulator path; spare servos | Order ≈ 1 month before module 14 | [14.02](14-robotic-arm/14.02-buying-and-assembling-the-arm.md), [14.03](14-robotic-arm/14.03-controlling-servos.md), [14.08](14-robotic-arm/14.08-arm-urdf-and-ros2-control.md), [14.10](14-robotic-arm/14.10-move-gripper-to-position.md), [14.11](14-robotic-arm/14.11-arm-safety.md), [15.03](15-manipulation/15.03-hand-eye-calibration.md), [15.06](15-manipulation/15.06-visual-servoing.md)–[15.08](15-manipulation/15.08-pick-and-place-pipeline.md), [15.10](15-manipulation/15.10-mobile-manipulation.md), [18.03](18-embodied-ai/18.03-teleop-data-collection.md), [18.06](18-embodied-ai/18.06-train-first-policy-lerobot.md), [18.08](18-embodied-ai/18.08-fine-tuning-a-vla.md); projects P14–P16 |
| **6 — Final robot** ([stage-6](hardware/stage-6-final-robot.md)) | Latching mushroom e-stop + relay/contactor; INA226; mounting parts | Bigger battery (3S LiPo 5000 mAh or Li-ion 3S2P) if runtime is too short; Bambu Lab A1 mini printer | E-stop can be bought earlier, at [14.11](14-robotic-arm/14.11-arm-safety.md) | [20.02](20-final-robot/20.02-hardware-integration.md), [20.04](20-final-robot/20.04-safety-case.md); final project P18 |

Projects live in [projects/](projects/README.md).

## Cost per stage

**Prices checked 2026-09-16, approximate, including VAT.** "Shipping" means Israeli store
delivery fees (estimated) or international shipping (not verified).

| Stage | Cheapest path ≈ ₪ | ≈ USD | Recommended path ≈ ₪ | ≈ USD |
|---|---|---|---|---|
| Tools (buy now) | 638 | 210 | 868 | 286 |
| Tools (buy later) | — (ruler, window + fan, pre-made cables) | — | 375 | 124 |
| 1 — First robot (all-in incl. ≈ ₪150 store shipping) | 1,775 | 585 | 2,250 (Pi 5 4 GB) · **2,550 (Pi 5 8 GB)** | 742 · 841 |
| 2 — IMU and camera | 301 | 99 | 380 | 125 |
| 3 — LiDAR | ≈ 300 (import from DFRobot, estimate) | ≈ 100 | 1,080 (local) | 356 |
| 4 — Compute and depth (optional) | 0 (skip) · 965 (OAK-D Lite) + shipping | 0 · 317 | ≈ 3,050 (Jetson + NVMe + D435i) + shipping | 1,005 |
| 5 — Robotic arm | ≈ 810 + shipping | 268 | ≈ 1,030 + shipping | 339 |
| 6 — Final robot | 171 | 56 | 1,005 | 331 |
| **Core course: tools + Stages 1–3** | **≈ 3,015** | **≈ 994** | **≈ 4,580** (4 GB) · **4,880** (8 GB) | **1,509 · 1,608** |
| **Everything except Stage 4** | **≈ 4,000** | **≈ 1,317** | **≈ 6,990** (4 GB) | **≈ 2,304** |
| Everything including Stage 4 | — | — | ≈ 10,040 (4 GB) | ≈ 3,310 |

## Cheapest path vs recommended path

| Decision | Cheapest path | Recommended path | Why the recommended choice |
|---|---|---|---|
| Pi 5 RAM | 4 GB (₪500) | 8 GB (₪800) if budget allows | Headroom for rviz, vision models and Nav2 together |
| Motor driver | 1× Pololu TB6612FNG (₪30.30) | 2× Pololu DRV8874 (₪121.20) | 520 motors stall at 3–4 A; TB6612 is 1.2 A continuous and can thermal-trip |
| 5 V regulator | UBEC 5 A (₪75) | Pololu D42V55F5 5.5 A (₪170) | Stable 5 V under Pi 5 + USB load prevents brown-out reboots |
| Cells | 3× LG HJ2 (₪119.70) | 4× Samsung 35E (₪196, one spare) | Higher capacity, known rating, a matched spare |
| Battery monitor | Resistor divider into the Pico ADC | INA219 (₪40) | Measures current as well as voltage |
| ToF sensor | Deferred to Stage 2 (Adafruit ₪80) | Pololu VL53L1X in Stage 1 (₪107.80) | Lesson 01.13 uses both range sensors |
| Bench PSU | None | Official 27 W PSU (₪70) | Desk work without the battery |
| IMU | Waveshare ICM20948 module (₪81) | BNO055 (₪135, restock) / SparkFun ICM-20948 (₪152) | BNO055 has an apt ROS 2 driver and on-chip fusion. Bosch marks it **"not recommended for new designs"** (checked 2026-09-20); the course uses it knowingly — see [stage-2](hardware/stage-2-imu-and-camera.md) for why and for the replacement path |
| LiDAR | RPLIDAR C1 imported ($69) | RPLIDAR C1 at Hackstore (₪1,050) | Local stock, warranty, no courier risk; import if you can wait and shipping works |
| Arm | SO-101 WowRobo Package 1 ($199) | SO-101 WowRobo Package 2 ($259) | Unassembled leader + follower **with camera**, clearer contents |
| Final power | Keep the Stage 1 pack; 10 A relay | 3S 5000 mAh LiPo + balance charger + 20 A relay | Runtime and current headroom for base + arm |
| Tools | DT-9205M, no helping hands | UT33A+, helping hands, crimper, calipers, fume extractor | Safer measurements, cleaner harnesses |

## Sim-only path: what you can do with zero hardware

You can start today with only a laptop (Ubuntu 24.04, or Windows 11 + WSL2; see
`curriculum/versions.yaml`). **148 of the 218 main-path lessons list no hardware**, including:

| Area | Lessons with no hardware |
|---|---|
| Orientation | All of module 00 |
| Software, ROS 2, frames | Most of modules 03 and 04, all of module 05 |
| Simulation | [06.02 Course mini-simulator](06-simulation/06.02-course-mini-simulator.md), [06.03 Gazebo basics](06-simulation/06.03-gazebo-basics.md), [06.05 Your robot in Gazebo](06-simulation/06.05-robot-in-gazebo.md), [06.07 Simulated sensors](06-simulation/06.07-simulated-sensors-and-noise.md), and the rest of module 06 except 06.09 |
| Estimation and autonomy | Most of modules 09–12: Kalman and particle filters, AMCL, [11.06 slam_toolbox in simulation](11-slam/11.06-slam-toolbox-simulation.md), [12.07 Nav2 in simulation](12-navigation/12.07-nav2-in-simulation.md) |
| Vision and AI | Most of module 13, all of modules 16, 17 and 19 |
| Projects | [P07 The simulated twin](projects/P07-simulated-robot.md) |

The labs include a pure-Python simulator and a ROS 2 Jazzy + Gazebo Harmonic workspace that
model karmel from `labs/config/karmel.yaml`, so the code you write in simulation is the code that
runs on the real robot (lesson [06.09](06-simulation/06.09-same-code-sim-and-real.md)). A
reasonable plan: study modules 00, 03–06 in simulation while the Stage 1 parts arrive.

What simulation can't teach: soldering, battery safety, motor deadband and real noise, which is
why Stage 1 is worth its price. (If you try Isaac Sim, it needs an RTX 4080-class GPU; the core course
uses Gazebo, which doesn't.)

## Detailed pages

| Page | Contents |
|---|---|
| [hardware/README.md](hardware/README.md) | Index |
| [hardware/tools.md](hardware/tools.md) | Workbench tools BOM |
| [hardware/stage-1-first-robot.md](hardware/stage-1-first-robot.md) | Every Stage 1 part, with alternatives, links, compatibility, spares, seller questions |
| [hardware/stage-2-imu-and-camera.md](hardware/stage-2-imu-and-camera.md) | IMU and camera |
| [hardware/stage-3-lidar.md](hardware/stage-3-lidar.md) | RPLIDAR C1 and alternatives |
| [hardware/stage-4-compute-and-depth.md](hardware/stage-4-compute-and-depth.md) | Jetson, RealSense, OAK-D (optional) |
| [hardware/stage-5-robotic-arm.md](hardware/stage-5-robotic-arm.md) | SO-101, RoArm-M3, LeKiwi |
| [hardware/stage-6-final-robot.md](hardware/stage-6-final-robot.md) | E-stop, relay, battery v2, mounting |
| [hardware/suppliers-israel.md](hardware/suppliers-israel.md) | Verified Israeli suppliers, and names that didn't check out |
| [hardware/importing-to-israel.md](hardware/importing-to-israel.md) | $75 VAT threshold, couriers, AliExpress lithium, plugs, warranty |
| [hardware/kits-vs-custom.md](hardware/kits-vs-custom.md) | Yahboom, Waveshare, Pololu Romi, TurtleBot 4 Lite, LeKiwi vs building karmel |
| [hardware/3d-printing-options.md](hardware/3d-printing-options.md) | Print services, makerspaces, printers and when to buy one |

For lesson authors: lessons reference only the catalog ids `tools`, `robot-base`, `imu`, `camera`,
`lidar`, `jetson`, `depth-camera`, `arm`, `estop` ([AUTHORING.md](AUTHORING.md) §9).
