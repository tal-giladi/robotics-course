# Stage 3 — 2D LiDAR

Catalog id: `lidar`. Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT** for Israeli shops. USD prices are
> foreign list prices without shipping or VAT. 1 USD = ₪3.033. Evidence:
> `references/research/israel-hardware-stages2-5-2026-09.md`.

**When to buy:** when you reach lesson [07.07](../07-sensors/07.07-2d-lidar.md), the first lesson
that uses a real LiDAR. Until then, simulated LiDAR (module 06) covers the concepts.

## What this stage unlocks

Lessons [07.07](../07-sensors/07.07-2d-lidar.md), [11.07 Map your real room](../11-slam/11.07-mapping-your-room.md),
[12.08 Nav2 on the real robot](../12-navigation/12.08-nav2-on-the-real-robot.md); projects
[P09](../projects/P09-mapping.md), [P10](../projects/P10-localization.md), [P11](../projects/P11-autonomous-navigation.md),
[P13](../projects/P13-robot-plus-vision.md), [P17](../projects/P17-llm-controlled-robot.md).

## Totals

| Path | Contents | ≈ ₪ | ≈ USD |
|---|---|---|---|
| **Recommended** | RPLIDAR C1 from Hackstore (in stock, local warranty) + upper-deck standoffs | **₪1,080** | **$356** |
| **Cheapest** | RPLIDAR C1 from DFRobot ($69) + shipping (not verified) | **≈ ₪300** (estimate) | ≈ $100 |

The local price is about 4× the DFRobot price. Importing saves ≈ ₪750 if shipping to Israel
works when you order. A $69 order is under the $75 VAT-free limit only if the goods value stays
under it (sources disagree on whether shipping counts; see [importing-to-israel.md](importing-to-israel.md)).

---

## RPLIDAR C1 — Buy now (at Stage 3)

| Field | Detail |
|---|---|
| Exact component | **Slamtec RPLIDAR C1** (DTOF 360° 2D LiDAR, USB adapter included) |
| Alternatives | **LDROBOT STL-19P / D500 kit** $119 at DFRobot, https://www.dfrobot.com/product-2610.html (DTOF, brushless, 60 klux; community Nav2-style driver, no apt binaries). **Local fallback: YDLIDAR/EAI T-mini Pro ₪950** at Hackstore (in stock), https://hackstore.co.il/product/%d7%9e%d7%95%d7%a6%d7%90-%d7%98%d7%95%d7%95%d7%97-%d7%a1%d7%95%d7%a8%d7%a7-360-lidar-t-mini-pro-12m-uart/ , only if the C1 is out of stock (its driver needs YDLidar-SDK). **Avoid:** LD19/D300 (discontinued), LD06 (poor in sunlight), RPLIDAR A1M8 ($99, older belt-driven triangulation, worse and dearer), YDLIDAR X4 Pro (belt-driven, fiddly setup, price unverified). |
| Why | Laser scans for SLAM (slam_toolbox), AMCL localization and Nav2 obstacle avoidance. It's the sensor that turns the robot into an autonomous navigator. |
| Price (≈, 2026-09-16) | **₪1,050** in stock (Hackstore) · **$69** (DFRobot) |
| Buy in Israel | https://hackstore.co.il/product/%d7%9e%d7%95%d7%a6%d7%90-%d7%98%d7%95%d7%95%d7%97-%d7%a1%d7%95%d7%a8%d7%a7-360-lidar-rplidar-c1-12m-uart/ |
| International | DFRobot: https://www.dfrobot.com/product-2803.html (check the courier status first; courier service to Israel was disrupted in March–April 2026) |
| ROS 2 driver | **Two drivers, and for the C1 the difference matters** (checked **2026-09-20**). **`rplidar_ros` *is* an apt binary** on Jazzy — `sudo apt install ros-jazzy-rplidar-ros`, version **2.1.0-4noble**, built for **both amd64 and arm64** (packages.ros.org `dists/noble`; [ros/rosdistro `jazzy/distribution.yaml`](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml) pins the release to `2.1.0-4`). **But that release predates C1 support:** `rplidar_c1_launch.py` and the SDK update that adds "RPLIDAR C1 support" are still under **"Forthcoming"** in the upstream changelog, so the apt package ships no C1 launch file. https://index.ros.org/p/rplidar_ros/ · https://github.com/Slamtec/rplidar_ros/blob/ros2/CHANGELOG.rst . **`sllidar_ros2` is source-only** — it has no binary in any ROS 2 distro, so clone it into `labs/ros2_ws/src` and `colcon build`. It does have the C1 launch file today: `ros2 launch sllidar_ros2 sllidar_c1_launch.py` (prefix `view_` for the variant that also opens RViz). https://github.com/Slamtec/sllidar_ros2 |
| Driver defaults (C1) | From [`launch/sllidar_c1_launch.py`](https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_c1_launch.py): `serial_port` **`/dev/ttyUSB0`**, `serial_baudrate` **`460800`**, `frame_id` **`laser`**, `scan_mode` `Standard`, `angle_compensate` `true`. `laser` is already karmel's scan frame (`labs/TESTED.md`: `/scan frame_id = laser`), so nothing needs remapping — but if you switch to a driver that defaults to something else, set `frame_id: laser` or the transform tree breaks. |
| Driver, what to do | Build `sllidar_ros2` from source for the C1. Re-check `rplidar_ros` at the next course revision ([MAINTAINING.md](../MAINTAINING.md)): once a release past 2.1.4 is bloomed for Jazzy the apt package becomes the simpler path, because a C1 without a C1 launch file is just a serial port that doesn't answer. |
| Specs | 12 m range (white targets) / 6 m (black), 5 kHz sample rate, IP54, 110 g, 460800 baud UART over the included USB adapter, Class 1 laser |
| Compatibility | USB to the Pi 5. Draws power from USB: **set `usb_max_current_enable=1`** when the Pi runs from the 5 V regulator (Stage 1 notes), or the LiDAR motor may stall or reboot the Pi. It shows up as `/dev/ttyUSB0`. The **Pico keeps `/dev/ttyACM0`**, so add a udev rule so the device names can't swap. |
| Mounting | Put it on the **upper deck, centered, above everything else** so nothing blocks the 360° view (`karmel.yaml`: x = 0, z = 0.12 m). Keep cables below the scan plane. |
| Cables / connectors | USB adapter and cable included. You need M2.5 screws and 30–40 mm standoffs for the upper deck (estimate ≈ ₪30). |
| Tools | Drill for the deck holes, screwdriver |
| Spares | None. Keep the USB adapter board; it's the part people lose. |
| Safety | Class 1 eye-safe, but never stare into or disassemble the emitter ([SAFETY.md](../SAFETY.md)). Navigation tests: ≤ 0.3 m/s indoors, hand on the switch. |
| Ask the seller | Hackstore: "Is this the C1 with the USB adapter board and cable included?" DFRobot: "Do you currently ship to Israel, with which courier, and what does it cost?" |

Next: [Stage 4 — compute and depth (optional)](stage-4-compute-and-depth.md), or go straight
to [Stage 5 — robotic arm](stage-5-robotic-arm.md).
