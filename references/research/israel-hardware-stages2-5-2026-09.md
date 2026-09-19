# Israel hardware research — Stages 2–5 (perception, LiDAR, compute/depth, arm) — 2026-09

All prices were checked on **2026-09-16** unless marked otherwise. ₪ = Israeli shop price as listed on the page (Israeli consumer prices normally include 18% VAT; most pages don't say so explicitly). USD = foreign store list price, without shipping, VAT or duty.

## 0. Import/shipping context for Israel (read first)

| Topic | Finding | Source |
|---|---|---|
| VAT-free limit for personal imports | Went back to **USD 75** on 2026-06-02 (it was USD 150 from Dec 2025 to Jun 2026). Orders over $75 pay 18% VAT. | https://www.vatupdate.com/2026/06/10/israel-restores-75-vat-exemption-threshold-for-personal-imports/ |
| Courier disruption | Seeed Studio posted on 2026-03-02 that DHL/FedEx/UPS shipments to Israel and the UAE were suspended because Middle East airspace was closed. FedEx said on 2026-04-09 that service in the Middle East was back to normal. I could not confirm Seeed's status for Sept 2026, so **email order@seeed.io before ordering**. | https://www.cep-research.com/2026/03/03/middle-east-conflict-continues-to-disrupt-global-logistics/ , https://www.freightwaves.com/news/ups-flights-to-israel-still-suspended-fedex-and-dhl-operate |
| Raspberry Pi approved reseller (IL) | The raspberrypi.com reseller finder lists only **PiiTel** (Netanya) for Israel. | https://www.raspberrypi.com/resellers/?q=&country=Israel , https://piitel.co.il |
| Main local maker shops | **4project.co.il** (Yehud; SparkFun/Pololu distributor), **hackstore.co.il**, **piitel.co.il** (sells Waveshare, Arducam and Adafruit parts) | pages below |
| Batteries | Buy LiPo/Li-ion packs **locally**. Air shipping of loose lithium packs is restricted. | general |

---

## Stage 2 — Perception add-ons

| Part | Rec / Alt | Price + source (2026-09-16) | ROS 2 driver | Notes |
|---|---|---|---|---|
| **Raspberry Pi Camera Module 3 (standard, 75°)** | **Recommended** | **₪145**, in stock, PiiTel https://piitel.co.il/shop/raspberry-pi-camera-module-3/ (variant SKU 73522) | `camera_ros` 0.7.0, binaries for Humble/Jazzy/Kilted/Lyrical/Rolling https://index.ros.org/p/camera_ros/ (repo https://github.com/christianrauch/camera_ros) | IMX708 12 MP autofocus. The index page warns that the bloomed ROS `libcamera` may not fully support all Pi camera modules. Building the RPi fork (https://github.com/raspberrypi/libcamera) may be needed on Pi 5. |
| **Camera Module 3 Wide (120°)** | Recommended for navigation/SLAM FOV | **₪200**, in stock, PiiTel, same page (SKU 73523) | same | The wider FOV helps visual odometry and obstacle detection. |
| **Pi 5 camera cable (22-pin → 15-pin)** | **Required** for either camera | **₪15** Official RPi 200 mm https://piitel.co.il/shop/official-raspberry-pi-csi-fpc-flexible-cable-200mm-for-raspberry-pi-5-22pin-to-15pin-suitable-for-csi-camera-modules/ ; **₪22** 500 mm third-party https://piitel.co.il/shop/csi-fpc-flexible-cable-for-raspberry-pi-5-22pin-to-15pin-500mm-suitable-for-csi-camera-modules/ | — | The 4project "Raspberry Pi camera FFC 60 cm" (₪8.70) is **15→15-pin (Pi 4 style) and will NOT fit a Pi 5**. Buy the 22→15 cable. |
| USB webcam alternative | Alt | Logitech C920 **from ₪245** (18 stores), C920E **₪220**, on Zap https://www.zap.co.il/search.aspx?keyword=logitech+c920 ; Arducam "USB UVC Camera Module 3" (IMX708, 102°) **₪270** https://piitel.co.il/shop/12mp-imx708-usb-uvc-102-wide-angle-fixed-focus-camera-module-3/ | `usb_cam` / `v4l2_camera` (UVC). I did not re-check their index status today. | Plug-and-play on Pi or Jetson with no libcamera trouble. Uses more CPU (MJPEG decode). LeRobot also uses cheap USB cams. |
| **IMU: BNO055** | **Recommended for ROS 2** | Hackstore generic BNO055 **₪135 (out of stock)** https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-bno055-%d7%97%d7%99%d7%99%d7%a9%d7%9f-9-%d7%a6%d7%99%d7%a8%d7%99%d7%9d-%d7%aa%d7%90%d7%95%d7%a6%d7%94-%d7%aa%d7%a0%d7%95%d7%a2%d7%94-%d7%96%d7%95%d7%99%d7%95%d7%aa/ ; Adafruit BNO055 **₪250 (out of stock)** https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-adafruit-%d7%97%d7%99%d7%99%d7%a9%d7%9f-%d7%aa%d7%a0%d7%95%d7%a2%d7%94-9-%d7%a6%d7%99%d7%a8%d7%99%d7%9d-%d7%9e%d7%91%d7%95%d7%a1%d7%a1-bno055/ | **`bno055` (flynneva) v0.5.0, released binaries for Humble/Jazzy/Kilted/Lyrical/Rolling, I2C or UART** https://index.ros.org/p/bno055/ , https://github.com/flynneva/bno055 | Fuses orientation on the chip and publishes `sensor_msgs/Imu` with `apt install`, which makes it the least-friction option for `robot_localization`. Stock is the problem: both local listings are out of stock, so order abroad or ask Hackstore. |
| IMU: BNO085/BNO086 | Alt (better sensor, weaker ROS 2 support) | SparkFun BNO086 Qwiic **₪234.63 (pre-order)**, 4project https://www.4project.co.il/product/sparkfun-vr-imu-breakout-bno086-qwiic | `bno08x_driver` **UNRELEASED** (build from source, I2C) https://index.ros.org/p/bno08x_driver/ , https://github.com/bnbhat/bno08x_ros2_driver ; ros2_control plugin https://github.com/RbSCR/bno08x_hardware_interface | Fusion and dynamic calibration are better than the BNO055. Choose it if the course is fine with a source build, or read it on the MCU and publish from there. |
| IMU: ICM-20948 | Local in-stock fallback | SparkFun ICM-20948 Qwiic **₪152, in stock**, 4project https://www.4project.co.il/product/sparkfun-9dof-imu-breakout-icm-20948-qwiic ; Waveshare 10-DOF (ICM20948+LPS22HB) **₪81, in stock** https://piitel.co.il/shop/10-dof-imu-sensor-module-for-raspberry-pi-pico-onboard-icm20948-and-lps22hb-chip/ | No mainstream released driver found. Publish raw data and fuse with `imu_filter_madgwick` (imu_tools). | Raw 9-DOF only, so the course has to cover filtering (a good lesson, but more work). |
| IMU: MPU-6050 | Not recommended | 4project listing is **Discontinued** https://www.4project.co.il/product/mpu-6050-imu-breakout ; cheap GY-521 clones at Hackstore/Hipertronics (no price captured) | Community only | 6-DOF with no magnetometer, so yaw drifts. It's a legacy part. |
| **ToF VL53L1X** | Recommended (cliff/bumper ranging) | Adafruit VL53L1X **₪80, in stock**, PiiTel https://piitel.co.il/shop/adafruit-vl53l1x-time-of-flight-distance-sensor-4000mm-range/ ; Pololu VL53L1X **₪107.80, in stock** https://www.4project.co.il/product/vl53l1x-distance-sensor-regulated-pololu ; SparkFun Qwiic **₪190.90** https://www.4project.co.il/product/sparkfun-distance-sensor-breakout-vl53l1x-qwiic | No mature ROS 2 package. Read it on the MCU or Pi I2C and publish `sensor_msgs/Range`. | Up to 4 m. The multizone upgrade is VL53L5CX (SparkFun, ₪192.50, 4project). |

**IMU recommendation:** The **BNO055** is the course default because it has the only IMU driver here with apt binaries on every current ROS 2 distro, and it outputs orientation directly. Use the **BNO085/086** when the students can build from source and want better heading. The **ICM-20948 (₪152, in stock at 4project)** is the local fallback and pairs well with a Madgwick-filter lesson.

---

## Stage 3 — 2D LiDAR

| Part | Rec / Alt | Price + source (2026-09-16) | ROS 2 driver (maturity) | Notes |
|---|---|---|---|---|
| **Slamtec RPLIDAR C1** | **Recommended** | **Israel: ₪1,050, in stock, Hackstore** https://hackstore.co.il/product/%d7%9e%d7%95%d7%a6%d7%90-%d7%98%d7%95%d7%95%d7%97-%d7%a1%d7%95%d7%a8%d7%a7-360-lidar-rplidar-c1-12m-uart/ ; **Abroad: $69, DFRobot** https://www.dfrobot.com/product-2803.html | **High.** Official `rplidar_ros` has apt binaries for Humble/Jazzy (2.1.4) and Kilted/Rolling (2.1.0); C1 support and a launch file are in the changelog https://index.ros.org/p/rplidar_ros/ . Official `sllidar_ros2` lists C1 (launch `view_sllidar_c1_launch.py`) https://github.com/Slamtec/sllidar_ros2 | DTOF, 12 m (white) / 6 m (black), 5 kHz, IP54, 110 g, 460800 baud, USB adapter included. The local price is about 4× DFRobot's; the import is cheaper but pays 18% VAT (over $75 with shipping) and depends on courier status. |
| LDROBOT **STL-19P / D500 kit** (LD19 successor) | Alt | **$119, DFRobot** https://www.dfrobot.com/product-2610.html | **Medium-high.** Official https://github.com/ldrobotSensorTeam/ldlidar_ros2 (and ldlidar_stl_ros2). Community lifecycle/Nav2-style driver for Humble/Jazzy (LD19 tested, LD06 untested, 104★) https://github.com/Myzhar/ldrobot-lidar-ros2 . No apt binaries found. | DTOF, 12 m, 10 Hz, 60 klux ambient-light tolerance, 47 g, 5 V/180 mA, brushless motor (no belt). |
| LDROBOT **LD19 / D300** | Avoid for new purchases | Waveshare LD19 kit **$95.99, marked DISCONTINUED**, D500 suggested instead https://www.waveshare.com/dtof-lidar-ld19.htm | same as above | Existing units still work with the drivers above. |
| LDROBOT LD06 | Not recommended | Amazon listings only (no price captured) | Official ldlidar_ros2 | Poor outdoor/sunlight performance per the Myzhar README. |
| Slamtec RPLIDAR A1M8-R6 | Not recommended | **$99, DFRobot** https://www.dfrobot.com/product-1125.html | High (same drivers) | Older triangulation unit with a belt drive. It costs more than the C1 and is worse. |
| YDLIDAR X4 Pro | Budget alt | youyeetoo Shopify returned "542.00" in an unstated currency (probably geo-localized, **unverified**) https://youyeetoo.com/products/ydlidar-x4pro-lidar ; Amazon listing https://www.amazon.com/Triangular-Scanning-Obstacle-Avoidance-Navigation/dp/B0CQLWBT52 (price not readable) | **Medium.** Official https://github.com/YDLIDAR/ydlidar_ros2_driver (`humble` branch, used for Jazzy too). Needs YDLidar-SDK. X4 Pro reports as S2PRO. Jazzy fork https://github.com/fordft168/ydlidar_ros2_driver_jazzy | Triangulation, 10 m, belt-driven. Setup is fiddly (SDK plus parameter quirks). |
| Slamtec-compatible local alt: YDLIDAR/EAI **T-mini Pro** | Local alt | **₪950, in stock, Hackstore** https://hackstore.co.il/product/%d7%9e%d7%95%d7%a6%d7%90-%d7%98%d7%95%d7%95%d7%97-%d7%a1%d7%95%d7%a8%d7%a7-360-lidar-t-mini-pro-12m-uart/ | ydlidar_ros2_driver | Only worth it if the C1 is out of stock locally. |

**LiDAR recommendation:** Get the **RPLIDAR C1**. It's the only LiDAR here with an official vendor driver that ships as apt binaries on Humble/Jazzy. It is DTOF (no belt), rated IP54, and sold locally in stock (Hackstore ₪1,050). Importing from DFRobot at $69 is much cheaper if shipping works. The alternative is the **LDROBOT STL-19P (D500)** with the Myzhar lifecycle driver.

---

## Stage 4 — Better compute and depth

| Part | Rec / Alt | Price + source (2026-09-16) | ROS 2 driver | Notes |
|---|---|---|---|---|
| **NVIDIA Jetson Orin Nano Super Dev Kit (8 GB)** | **Recommended** compute upgrade | **Israel: ₪1,652, OUT OF STOCK, PiiTel** https://piitel.co.il/shop/nvidia-jetson-orin-nano-super-developer-kit/ . Distributor **C.R.G. Electronics** (B2B, no public price) https://crg.co.il/catalog_category/nvidia-jetson/ . Abroad: usually listed at $249 MSRP (search snippets; one retailer at $399) https://www.sparkfun.com/nvidia-jetson-orin-nano-developer-kit.html | JetPack 6.x (Ubuntu 22.04 → ROS 2 Humble natively; other distros via Docker/Isaac ROS) | NVIDIA's product page gives no price; "where to buy" now redirects to https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/ (timed out). **SKU note:** 945-13766-**0000**-000 is the US/CA/CN/JP/PH region kit (RS Online); outside those regions look for **-0005-000**. 67 TOPS, 7–25 W. |
| **Intel/RealSense D435i** | **Recommended** depth camera (with Jetson) | **$334**, RealSense store https://store.realsenseai.com/ | **Most mature:** `realsense2_camera` 4.58.4 binaries on Humble/Jazzy/Kilted/Lyrical/Rolling https://index.ros.org/p/realsense2_camera/ , https://github.com/IntelRealSense/realsense-ros | **RealSense spun out of Intel on 2025-07-11** as an independent company (CEO Nadav Orbach, HQ Cupertino, $50M Series A). The new store is store.realsenseai.com. D400 support continues. https://www.realsenseai.com/news-insights/news/realsense-completes-spin-out-from-intel-raises-50-million-to-accelerate-ai-powered-vision-for-robotics-and-biometrics/ . Has an IMU. The old store.intelrealsense.com URLs still show up in search. |
| RealSense D405 | Alt (close range / manipulation) | **$272**, same store | same | Short-range (~7 cm–50 cm sweet spot). Good wrist camera for the arm stage. D435 is $314 (OOS), D455 $419, D555 $499 (OOS). |
| **Luxonis OAK-D Lite** | Alt (if staying on Pi 5 — on-device NN) | **$269** (90 in stock; AF and FF variants) https://shop.luxonis.com/products/oak-d-lite-1 | `depthai-ros`: index.ros.org shows binaries for **Kilted/Lyrical 3.4.0 (2026-08-19)**. Humble/Jazzy showed "no version" today, **verify** https://index.ros.org/p/depthai_ros_driver/ , https://github.com/luxonis/depthai-ros | Runs the neural net on the camera (4 TOPS), so it suits a Pi 5 without a GPU. Price has gone up (was ~$149 at launch). Luxonis lists **no Israel/Middle East distributor** https://www.luxonis.com/distributors (Mouser Israel lists OAK-D). |
| Luxonis OAK-D Pro / Pro W | Alt | **$429 / $529** https://shop.luxonis.com/collections/oak-cameras-col | same | Active IR, works in low light. |
| **Orbbec Gemini 2** | Alt (cheapest stereo) | **$234** https://store.orbbec.com/collections/gemini-2-series (Gemini 215 $218, Gemini 2L $319) | `OrbbecSDK_ROS2` v2 supports Foxy/Humble/Jazzy; apt `ros-humble-orbbec-camera` https://github.com/orbbec/OrbbecSDK_ROS2 | The Gemini 335 ($264) is **out of stock** https://store.orbbec.com/products/gemini-335 . The README doesn't state Jetson/ARM support, so verify. Astra 2 is also supported. |
| Israel stock for depth cameras | — | PiiTel: **no** OAK/RealSense/Orbbec results (searches 2026-09-16) | — | Plan to import. |

**Compute/depth recommendation:** Get the **Jetson Orin Nano Super** once students need GPU perception or Isaac ROS. It's out of stock at PiiTel (₪1,652), so ask C.R.G. or import the -0005 SKU. For depth, pick the **RealSense D435i ($334)**: it has the best ROS 2 packaging (binaries on every distro), a built-in IMU and the largest Nav2/Isaac ROS tutorial base. If the robot stays on a Pi 5, pick the **OAK-D Lite ($269)** instead, because it runs detection on the camera.

---

## Stage 5 — Robotic arm

| Arm | Rec / Alt | Price + source (2026-09-16) | ROS 2 / MoveIt 2 | LeRobot | Payload / repeatability | Power | Mobile-base fit | Notes |
|---|---|---|---|---|---|---|---|---|
| **SO-101 (TheRobotStudio / HF LeRobot)** | **Primary** | **WowRobo** https://shop.wowrobo.com/products/so-arm101-diy-kit-assembled-version-1 : Pkg 1 printed parts + 12 servos **$199**; Pkg 2 unassembled leader+follower+camera **$259**; Pkg 3 assembled **$299**. **Seeed** servo kit Pro **$249.90** (no printed parts/camera) + printed parts **$29.90** https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html . **PartaBot** (US) electronics-only $329 / full kit $399 / assembled $479; follower-only $249/$299 https://partabot.com/products/so-arm101 . DIY BOM ≈ **$229.88** for 2 arms, **$121.94** for one follower https://github.com/TheRobotStudio/SO-ARM100 | Community stacks, no official vendor package: https://github.com/ros-physical-ai/ros2_so_arm (description + feetech_ros2_driver + MoveIt config), https://github.com/legalaspro/so101-ros-physical-ai (ros2_control + MoveIt 2 + teleop + recording), https://github.com/adityakamath/so_arm_ros2 | **Flagship** robot in LeRobot docs https://huggingface.co/docs/lerobot/so101 | Payload and repeatability are not published by the maintainers (**unverified**). Low; hobby servos. | Follower: 6× STS3215 7.4 V (1/345). Leader mixes 1/191, 1/345, 1/147. 5 V PSU for the 7.4 V version, or a 12 V version (30 kg·cm vs 16.5 kg·cm stall). | Yes. It's the arm on LeKiwi and XLeRobot. | Best for imitation learning (ACT, SmolVLA, π₀). Parts can be 3D-printed locally. PiiTel sells 30 kg serial-bus servos (ST3215-class, 12 V) at **₪107** https://piitel.co.il/shop/30kg-serial-bus-servo-high-precision-and-torque-with-programmable-360-degrees-magnetic-encoder/ and 4project sells the Feetech FE-URT-1 bus adapter at **₪59.40** https://www.4project.co.il/product/feetech-fe-urt-1-serial-servo-adapter , useful for spares. |
| **Waveshare RoArm-M3 (S / Pro)** | **Alternative** (official ROS 2 + MoveIt demos) | **$239.99–$339.99** https://www.waveshare.com/roarm-m3.htm | Official ROS 2 workspace + URDF + MoveIt 2 MTC demo https://github.com/waveshareteam/roarm_ws , https://www.waveshare.com/wiki/RoArm-M3_Moveit_MTC_Demonstration | Waveshare claims LeRobot support; not in LeRobot's main docs (**verify**) | Up to 1 kg claimed at the end effector (wiki also says 200 g @ 0.5 m); ≈ ±5 mm | 12 V 5 A; accepts a 3S Li-ion battery | Good. It runs on 12 V/3S and has ESP32 WiFi control. | The Pro version uses metal ST3235 servos. RoArm-M2-S (4-DOF) costs $179.99–$299.99, 7–13 V, ±4 mm https://www.waveshare.com/roarm-m2-s.htm . PiiTel carries other Waveshare robots but had **no RoArm listing**. |
| Elephant Robotics **myCobot 280 Pi** | Premium alt | **$799** official https://shop.elephantrobotics.com/collections/mycobot-280 (Arduino $599, M5 $649, Jetson Nano $849, RDK X5 $830); €905 at Welectron | Official https://github.com/elephantrobotics/mycobot_ros2 (branches foxy/galactic/**humble**; MoveIt 2 not confirmed in README) | Not in LeRobot core docs | 250 g payload; repeatability not captured | 12 V adapter (not verified today) | Possible, but its Pi 4B controller duplicates the robot's compute. | Polished product, but the ROS 2 support stops at Humble. |
| Hiwonder **ArmPi Ultra** | Not primary | Starter w/o Pi **$299.99** up to Standard w/ Pi 5 16 GB **$809.99** (Shopify JSON) https://www.hiwonder.com/products/armpi-ultra | ROS 2 (vendor images/tutorials) | No | 25 kg bus servos; payload not captured | — | Stationary kit | Closed-ish vendor tutorials, includes a 3D camera. |
| Hiwonder **xArm 1S / xArm ESP32 / LeArm AI** | Not recommended for ROS 2 | xArm 1S **$199.99** unassembled / **$239.99** assembled https://www.hiwonder.com/products/xarm-1s ; xArm ESP32 **$229.99** https://www.hiwonder.com/products/xarm-esp32 ; LeArm AI **$199.99–$319.99** https://www.hiwonder.com/products/learm-ai | No official ROS 2 | No | Toy-class | — | — | Education toys (MicroPython/Arduino). |
| Trossen **PincherX 100 / WidowX 250 S** | Not available | **Both DISCONTINUED** per Trossen pages https://www.trossenrobotics.com/pincherx100 , https://www.trossenrobotics.com/widowx-250 | Interbotix ROS 2 (Galactic/Humble/Rolling) | ALOHA heritage | PX100 50 g / 5 mm; WX250S 250 g / 1 mm | — | — | Replacement **WidowX AI**: base **$2,995**, leader **$4,685.95**, 1.5 kg payload https://store.trossenrobotics.com/products/widowx-ai-base-1 (way over budget). |
| **UFACTORY Lite 6** | Not for a small mobile base | **$3,500** https://www.ufactory.us/product/lite-6 | Official xarm_ros2 (vendor marks ROS 2 "Beta") https://robots.ros.org/xarm/ | No | 600 g, 0.5 mm, 440 mm reach | **24 V DC 16.5 A**, 7.2 kg arm | No: too heavy and power-hungry | Real 6-axis arm, desk-mounted only. |

### Mobile manipulators

| Platform | BOM / price | Source | Notes |
|---|---|---|---|
| **LeKiwi** (3-omni-wheel base + SO-101, Pi 5) | **12 V complete $482** / 5 V complete $499; base-only $248 (5 V) / $251.5 (12 V); wired version $184. Includes Pi 5 4GB ($60), 2 USB cams ($12.98 each), 3× 4" omni wheels ($9.99 each). Power: 12 V 5 A Li-ion battery or 65 W laptop power bank. Seeed LeKiwi 12V kit (base + printed parts + battery) **€270.55** at OpenELAB https://openelab.io/products/seeed-studio-lekiwi-kit12v-version-mobile-base-with-3d-printed-parts-and-battery ; Seeed full kit https://www.seeedstudio.com/LeKiwi-Full-Kit-12V-Verision.html | https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/BOM.md , https://huggingface.co/docs/lerobot/lekiwi | Official LeRobot robot. The follower arm doesn't count toward base-only prices. |
| **XLeRobot** (dual SO-101 on a cart base) | BOM **from ~$660**; WowRobo dev kit **$579 worldwide** (¥3699 CN), **excluding battery, IKEA cart, 3D printing, tools and shipping**. RealSense head upgrade +$220 | https://github.com/Vector-Wangel/XLeRobot | Built on LeRobot. Laptop compute by default. ROS 2 only via sim/control layers. |

**Arm recommendation:**
- **Primary: SO-101 leader+follower.** Get WowRobo Package 2 ($259, unassembled, with camera) or Seeed's kit plus locally printed parts. It is LeRobot's flagship arm (imitation learning is the point of this stage) and costs the least. It is also the arm on LeKiwi, which gives a clear route to mobile manipulation. ROS 2/MoveIt 2 comes from community packages (`ros2_so_arm`, `so101-ros-physical-ai`). Choose the 12 V servo version if it will run from the robot battery.
- **Alternative: Waveshare RoArm-M3-Pro** ($239.99–$339.99). It has an official ROS 2 workspace and MoveIt 2 demos, runs on 12 V/3S, and claims more payload. The trade-off is weaker LeRobot integration.
- For a mobile manipulator, build **LeKiwi 12 V (~$482 BOM)** on the course's own base, or on the LeKiwi base when the omni-wheel design is wanted.

---

## 3D printing in Israel

| Item | Price / detail (2026-09-16) | Source | Notes |
|---|---|---|---|
| **Bambu Lab A1 mini** (official importer 3DbotX) | **₪990**; **A1 mini Combo (AMS lite) ₪1,890** | https://www.3dbotx.co.il/product-page/bambulab-a1-mini-3d-printer , https://www.3dbotx.co.il/product-page/bambulab-a1-mini-combo-3d-printer | 180 mm³ build volume, big enough for SO-101 parts. 3DbotX warns that grey-import units may be region-locked. |
| Bambu Lab A1 mini (Zap) | **₪890** at Bug (Eilat in-store ₪755); importer not stated | https://www.zap.co.il/model.aspx?modelid=1243156 | Check that it's an official-importer unit before buying. |
| **Bambu Lab A1** | **₪1,550** (3DbotX) | https://www.3dbotx.co.il/product-page/bambulab-a1-3d-printer | 256 mm³. Best all-rounder for a robot course. |
| Prusa CORE One | **₪6,460** at Caliber (official Prusa partner; page shows several prices, **verify**) | https://shop.caliber.co.il/items/7537329-Prusa-CORE-One | Also at Laserline, ProMaker, 3dny (Yizmako). Prusa MK4S not priced. |
| Creality Ender 3 V3 / V3 KE | **₪1,790 / ₪1,990** at 3DbotX (from search snippet, page not fetched) | https://www.3dbotx.co.il/product-page/creality-ender-3-v3 , https://www.3dbotx.co.il/product-page/creality-ender-3-v3-ke | Spider3D also imports Creality https://www.spider3d.co.il/product-tag/creality/ |
| Filament PLA | ₪100–200 per kg (service article); spools from ₪69 (Spider3D) | https://www.3d3.co.il/post/discover-cost , https://www.spider3d.co.il/product-category/%D7%A4%D7%99%D7%9C%D7%9E%D7%A0%D7%98%D7%99%D7%9D/ | — |
| **Print services, typical cost** | 3D Print Orel: small simple parts **from ₪30–50**, medium **₪100–300**. 3D3: simple ₪10–20, small figure ~₪30, large prototype >₪500; modelling ₪100–400/h. **Rough estimate for a full SO-101 print set: a few hundred ₪ (unverified).** Buying Seeed's printed set ($29.90) or a WowRobo package with parts is cheaper. | https://3dprintorel.co.il/3d-printing-price/ , https://www.3d3.co.il/post/discover-cost | Other services: 3D Factory (Jaffa) https://www.3dfactory.co.il/ , AMP Jerusalem https://amp3d.co.il/ , Center3D https://www.centerd3.co.il/ , Indus3design https://www.indus3design.com/3d-printing-cost/ |
| **MakeLab Israel (Yehud)** | Free community lab; 3D printers, laser cutter, CNC, PCB; Sun–Thu 09:00–21:00; operator approval needed. 28 Kdoshei Mitsrayim St. | https://fablabs.io/labs/makelabisrael , https://www.makelab.org.il/ | fablabs.io says "Active" (listing date unknown). |
| **Makerz network** (makerspace.co.il) | Membership/punch card; 3D printers (PLA/PETG/ABS + 2 Stratasys), laser, CNC; paid manufacturing service. Locations listed: Mateh Asher maker center, Emek Harod (Ein Harod) entrepreneurship center, Idan (Emek HaMaayanot), Tal Makers, MonkeyMakers | https://makerspace.co.il/ , https://makerspace.co.il/makerspace/ | Mostly northern/periphery sites. No prices online. |
| Impact Labs (Tel Aviv) | 1,000 m² product-development lab with many printers (search snippet) | https://www.impactlabs.tech/?lang=en | Public access and pricing not verified. |
| FabLab IL (Holon) / MadaTech FabLab (Haifa) / Tel Aviv Makers | Only 2014-era articles found | https://www.fablabs.io/labs/fablabil , https://3dprint.com/36673/largest-fab-lab-opens-in-haifa/ | **Current status unverified.** University labs (Technion, TAU, HIT) are usually for students only. |

---

## Power, safety and monitoring (bigger robot)

| Part | Rec | Price + source (2026-09-16) | Notes |
|---|---|---|---|
| 3S LiPo 11.1 V 5000 mAh | Recommended for a 12 V robot + SO-101 12 V / RoArm | FullymaX 30C **₪285**, 45C **₪250**; 3S 3300 mAh **₪225**, Skyline (Kav HaOfek) https://www.skylineairline.co.il/%D7%A1%D7%95%D7%9C%D7%9C%D7%95%D7%AA-lipo | Needs a balance charger and a LiPo bag. Run the Pi/Jetson through a buck converter. |
| 4S LiPo 14.8 V 4400 mAh 70C XT90 | For a 12–19 V Jetson + motors | **₪520**, Skyline, same page | The Jetson dev kit takes 9–20 V in (verify on the carrier); use a regulator. |
| DIY Li-ion pack | Alt (safer than LiPo, longer runtime) | LG HJ2 18650 2900 mAh **₪39.90/cell**, 4project https://www.4project.co.il/product/18650-lg-hj2-battery-3.7v-3000mah | 3S2P or 4S2P plus a BMS. LeKiwi's BOM uses a ready-made 12 V 5 A Li-ion pack. |
| **E-stop mushroom button** | Recommended | Erco "לחצן פטריה בקופסא אדום ננעל 1NC" (latching, in enclosure) **₪67.20** https://www.erco.co.il/b2c/9922516.html ; key-release 1NO+1NC **₪169.20** https://www.erco.co.il/b2c/command-and-control/command-devices-22mm/62115020640.html | Wire the NC contact in series with the **coil of a relay/contactor that cuts motor + arm power** (not the computer's power). Use the NO contact, or sense the line on a GPIO, so ROS knows the e-stop is pressed. |
| Relay for e-stop cut | Recommended | SPDT sealed relay 12 V coil 10 A **₪3.90** https://www.4project.co.il/product/spdt-relay-12v-coil-10a ; 5 V coil 20 A **₪27.60** https://www.4project.co.il/product/spdt-relay-5v-coil-20a | For higher currents use an automotive 40 A relay or DC contactor (not priced) and add a fuse. |
| **INA226 power monitor** | Recommended | **₪55, in stock**, Hackstore https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-%d7%97%d7%99%d7%99%d7%a9%d7%9f-%d7%9e%d7%aa%d7%97-%d7%95%d7%96%d7%a8%d7%9d-ina226/ | I2C (to 36 V), bus voltage + shunt current. Publish `sensor_msgs/BatteryState` from a small node. 4project only has the analog INA169 (₪91.60). |

---

## Recommendations (summary)

1. **Stage 2:** Camera Module 3 Wide (₪200) plus the Pi 5 22→15 cable (₪15) from PiiTel; the standard camera (₪145) also works. Use `camera_ros`. Take the **BNO055** IMU (apt driver on every distro); if it's out of stock locally, buy the ICM-20948 at 4project (₪152). Add **VL53L1X** from PiiTel (₪80).
2. **Stage 3:** **RPLIDAR C1**: Hackstore ₪1,050 (in stock) or DFRobot $69. Official `rplidar_ros`/`sllidar_ros2`. Alternative: LDROBOT STL-19P ($119).
3. **Stage 4:** **Jetson Orin Nano Super** (PiiTel ₪1,652, currently OOS; ~$249 abroad) with a **RealSense D435i ($334, realsenseai store)**. On Pi 5 only, use the **OAK-D Lite ($269)** instead.
4. **Stage 5:** **SO-101** leader+follower (WowRobo $259 kit / $299 assembled; Seeed $249.90 + $29.90 parts). **Alternative: Waveshare RoArm-M3** ($239.99–$339.99). Mobile manipulation: **LeKiwi 12 V (~$482 BOM)**.
5. **Fabrication:** **Bambu Lab A1 mini ₪990 / A1 ₪1,550** (3DbotX, official). Free printing at MakeLab Yehud.
6. **Safety/power:** 3S LiPo 5000 mAh (₪250–285), latching e-stop (₪67.20) driving a relay/contactor, **INA226 (₪55)**.
7. **Logistics:** foreign orders over $75 pay 18% VAT (since 2026-06-02). Check courier status with Seeed (Israel shipments were suspended in March 2026) before ordering.

---

## Could not verify

- Seeed Studio / WowRobo / PartaBot / Luxonis / RealSense **shipping to Israel as of Sept 2026** (Seeed's March 2026 suspension notice exists, but current status is unconfirmed; WowRobo shipping-policy URL returned 404).
- Current **Jetson Orin Nano Super** USD price (NVIDIA page shows none; marketplace page timed out), ILS price at DigiKey/Mouser Israel (403/blocked), and C.R.G. Electronics pricing (B2B).
- **depthai-ros** Humble/Jazzy binaries (index.ros.org showed only Kilted/Lyrical/older today).
- OrbbecSDK_ROS2 Jetson/ARM support and Jazzy apt package name.
- **SO-101 payload/repeatability** (not published by TheRobotStudio/HF).
- RoArm-M3 LeRobot support depth (Waveshare claim only).
- myCobot 280 Pi repeatability, PSU spec, and MoveIt 2 support in `mycobot_ros2`.
- YDLIDAR X4 Pro real USD price (youyeetoo returned 542.00 in an unstated/geo-localized currency; Amazon price unreadable).
- Whether 4project/PiiTel/Hackstore prices include VAT (not stated on the pages; assumed yes).
- Hiwonder Shopify prices assumed USD (store base currency not confirmed).
- Prusa CORE One exact Caliber price (page shows several ₪ figures; ₪6,460 is from structured data) and Prusa MK4S price in Israel.
- Creality Ender 3 V3 / V3 KE prices (search snippet only).
- Current status of FabLab IL Holon, MadaTech FabLab Haifa, Tel Aviv Makers, Impact Labs public access, and university fablabs.
- Typical cost to print a full SO-101 part set at an Israeli service (no published per-gram rates).
- LiFePO4 12 V pack prices at Israeli shops (only Amazon results).
- `usb_cam`/`v4l2_camera` current index status (not re-checked today).
