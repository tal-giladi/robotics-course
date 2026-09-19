# Stage 2 — IMU and camera

Catalog ids: `imu`, `camera`. Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT** for Israeli shops. USD prices are
> foreign list prices without shipping or VAT. 1 USD = ₪3.033. Evidence:
> `references/research/israel-hardware-stages2-5-2026-09.md` and `software-versions-2026-09.md`.

**When to buy:** when you reach module 07 (sensors). Nothing in modules 01–06 needs these parts.

## What this stage unlocks

| Part | Lessons | Projects |
|---|---|---|
| IMU | [07.05](../07-sensors/07.05-imu-fundamentals.md), [07.06](../07-sensors/07.06-imu-in-ros2.md), [08.11](../08-control/08.11-rotate-exactly-90.md), [10.07](../10-localization/10.07-fusing-imu-and-odometry.md) | [P05](../projects/P05-autonomous-square-path.md), [P10](../projects/P10-localization.md) (with the LiDAR) |
| Camera | [07.08](../07-sensors/07.08-rgb-cameras.md), [10.10](../10-localization/10.10-fiducial-landmarks.md), [13.06](../13-computer-vision/13.06-lens-distortion-calibration.md), [13.07](../13-computer-vision/13.07-fiducial-markers.md), [13.15](../13-computer-vision/13.15-vision-in-ros2.md), [13.16](../13-computer-vision/13.16-detection-to-map-position.md), [15.04](../15-manipulation/15.04-object-pose-estimation.md) | [P12](../projects/P12-camera-object-detection.md), [P13](../projects/P13-robot-plus-vision.md) |

## Totals

| Path | Contents | ≈ ₪ | ≈ USD |
|---|---|---|---|
| **Recommended** | BNO055 (Hackstore, when restocked) + Logitech C920 | **₪380** | **$125** |
| Recommended, in stock today | SparkFun ICM-20948 + Logitech C920 | ₪397 | $131 |
| **Cheapest** | Waveshare 10-DOF ICM20948 module + Logitech C920E | **₪301** | **$99** |
| Optional add-on | Camera Module 3 Wide + Pi 5 camera cable | ₪215 | $71 |
| Deferred from Stage 1 (cheapest path only) | VL53L1X (Adafruit, Piitel) | ₪80 | $26 |

---

## 1. IMU

### 1a. BNO055 9-DOF IMU with on-chip fusion — Buy now (at Stage 2)

| Field | Detail |
|---|---|
| Exact component | Bosch **BNO055** breakout (Adafruit BNO055 or a generic GY-BNO055) |
| Alternative | **ICM-20948** (local, in stock), see 1b. BNO085/BNO086 (better fusion, but its ROS 2 driver `bno08x_driver` is unreleased and must be built from source): SparkFun BNO086 Qwiic ₪234.63 (pre-order) at 4Project, https://www.4project.co.il/product/sparkfun-vr-imu-breakout-bno086-qwiic . **Not recommended:** MPU-6050 (no magnetometer, so yaw drifts; the 4Project listing is discontinued). |
| Why | Heading and angular rate for exact 90° turns (08.11) and for fusing with wheel odometry in `robot_localization` (10.07). The BNO055 fuses on the chip and has an **apt-installable ROS 2 driver** (`bno055`, flynneva, binaries for Humble/Jazzy and newer, I²C or UART), so it's the lowest-friction path. |
| Price (≈, 2026-09-16) | Generic BNO055 **₪135** · Adafruit BNO055 **₪250**. **Both were out of stock at Hackstore.** |
| Buy in Israel | Generic: https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-bno055-%d7%97%d7%99%d7%99%d7%a9%d7%9f-9-%d7%a6%d7%99%d7%a8%d7%99%d7%9d-%d7%aa%d7%90%d7%95%d7%a6%d7%94-%d7%aa%d7%a0%d7%95%d7%a2%d7%94-%d7%96%d7%95%d7%99%d7%95%d7%aa/ · Adafruit: https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-adafruit-%d7%97%d7%99%d7%99%d7%a9%d7%9f-%d7%aa%d7%a0%d7%95%d7%a2%d7%94-9-%d7%a6%d7%99%d7%a8%d7%99%d7%9d-%d7%9e%d7%91%d7%95%d7%a1%d7%a1-bno055/ |
| International | Adafruit BNO055 (STEMMA QT) **$29.95**, in stock on 2026-09-16: https://www.adafruit.com/product/4646 . Under $75, so VAT-free if the order stays under the threshold (shipping cost not checked). |
| Compatibility | 3–5 V logic, I²C address 0x28 (or 0x29). ROS 2 driver docs: https://index.ros.org/p/bno055/ , source https://github.com/flynneva/bno055 . Connect it to the **Pi's I²C** (for the ROS driver) or to the Pico's I²C (then publish from the Pico bridge). Mount it near the robot's center, flat, away from motors and the battery wires (magnetometer). |
| Cables / connectors | STEMMA QT / Qwiic (JST-SH 4-pin) to Dupont cable, or solder headers; 4 short wires (3V3, GND, SDA, SCL) |
| Tools | Soldering iron for the headers, double-sided foam tape or M2.5 standoffs |
| Spares | None needed |
| **Ask the seller (Hackstore)** | "When will the BNO055 (generic or Adafruit) be back in stock?" If there's no date, buy the ICM-20948 below or order from Adafruit. |

### 1b. ICM-20948 9-DOF IMU — local in-stock alternative

| Field | Detail |
|---|---|
| Exact component | **SparkFun ICM-20948 Qwiic** (recommended alternative) or **Waveshare 10-DOF ICM20948 + LPS22HB** module (cheapest) |
| Why choose it | In stock locally. There's no mainstream released ROS 2 driver: you publish raw data and fuse it with `imu_filter_madgwick` (apt package `imu-tools`). That's more work, but it's a good filtering lesson. |
| Price | SparkFun **₪152** (4Project, in stock) · Waveshare **₪81** (Piitel, in stock) |
| Buy in Israel | https://www.4project.co.il/product/sparkfun-9dof-imu-breakout-icm-20948-qwiic · https://piitel.co.il/shop/10-dof-imu-sensor-module-for-raspberry-pi-pico-onboard-icm20948-and-lps22hb-chip/ |
| Compatibility | 3.3 V I²C. The Waveshare board is a Pico-format module; check the pin-out before stacking it. |
| Cables | Qwiic-to-Dupont cable for SparkFun (price not checked) |

---

## 2. Camera

> [!IMPORTANT]
> **Version-sensitive** (verified 2026-09-16). On **Ubuntu 24.04** the Raspberry Pi camera
> stack (libcamera) **isn't operational with the stock packages** (Ubuntu's docs: only from
> 25.04). That's why the course's **main path is a USB webcam** with `v4l2_camera`, and the
> Camera Module 3 is an optional advanced path. See `curriculum/versions.yaml`.

### 2a. USB webcam, Logitech C920 — Buy now (at Stage 2)

| Field | Detail |
|---|---|
| Exact component | **Logitech C920** (or the C920E business variant) |
| Alternative | Arducam "USB UVC Camera Module 3" (IMX708, 102° wide, fixed focus) **₪270** at Piitel, https://piitel.co.il/shop/12mp-imx708-usb-uvc-102-wide-angle-fixed-focus-camera-module-3/ . It uses the Camera Module 3 sensor over plain USB, so it avoids the libcamera issue and gives a wider view. Any UVC webcam works with `v4l2_camera`. |
| Why | Lessons 07.08 and module 13: images in ROS 2, calibration, AprilTags, detection. It's plug-and-play on the Pi 5 and later on a Jetson. LeRobot (Stage 5) also uses USB cameras, so it's reused there. |
| Price | C920 **from ₪245** (18 stores on Zap) · C920E **₪220** |
| Buy in Israel | Compare on Zap: https://www.zap.co.il/search.aspx?keyword=logitech+c920 (consumer shops such as KSP, Ivory and Bug carry Logitech) |
| International | Not worth it; local price is competitive and warranty is simpler |
| Compatibility | UVC, so the driver is `v4l2_camera` (apt `ros-jazzy-v4l2-camera` 0.7.3 on arm64) or `usb_cam`. Uses more Pi CPU than a CSI camera (MJPEG decode); use 640×480 (the `karmel.yaml` default). **Pi 5 USB current:** when the Pi is fed by the buck regulator, set `usb_max_current_enable=1` or USB devices may brown out. |
| Cables / connectors | Attached USB-A cable. A short right-angle USB extension helps cable routing (estimate ₪20). |
| Tools | A 1/4"-20 screw or a 3D-printed/cut bracket to fix it to the chassis front (x = 0.10 m, z = 0.10 m in `karmel.yaml`) |
| Spares | None |

### 2b. Raspberry Pi Camera Module 3 — Optional (advanced path)

| Field | Detail |
|---|---|
| Exact component | **Camera Module 3 Wide (120°)**, or the standard (75°) version |
| Alternative | The Arducam USB version above (same IMX708 sensor, no libcamera trouble) |
| Why | Lower CPU use and latency than USB, and a wide FOV for visual odometry. It needs extra work on the course OS: (1) build camera_ros against the `raspberrypi/libcamera` fork on Ubuntu 24.04, or (2) run Raspberry Pi OS on the host with ROS 2 Jazzy in Docker (`ros:jazzy-ros-base`). **Which of these works reliably end-to-end is unverified**; expect debugging. |
| Price | Wide **₪200** · Standard **₪145** (Piitel, in stock) |
| Buy in Israel | https://piitel.co.il/shop/raspberry-pi-camera-module-3/ |
| Compatibility | ROS 2 node `camera_ros` (https://index.ros.org/p/camera_ros/). Its apt build pulls upstream libcamera, which "may not contain full support for all Raspberry Pi camera modules". |
| **Required cable** | **Pi 5 camera cable, 22-pin → 15-pin.** Official 200 mm **₪15**, https://piitel.co.il/shop/official-raspberry-pi-csi-fpc-flexible-cable-200mm-for-raspberry-pi-5-22pin-to-15pin-suitable-for-csi-camera-modules/ · third-party 500 mm **₪22**, https://piitel.co.il/shop/csi-fpc-flexible-cable-for-raspberry-pi-5-22pin-to-15pin-500mm-suitable-for-csi-camera-modules/ . **The 4Project "Raspberry Pi camera FFC 60 cm" (₪8.70) is 15→15-pin and will NOT fit a Pi 5.** |
| Spares | A second ribbon cable (₪15). The connectors are fragile; power off before inserting. |

---

## 3. If you took the cheapest Stage 1 path

Buy the ToF sensor now if you skipped it: **Adafruit VL53L1X ₪80** at Piitel (in stock),
https://piitel.co.il/shop/adafruit-vl53l1x-time-of-flight-distance-sensor-4000mm-range/ , needed for
[07.04](../07-sensors/07.04-tof-sensors.md).

Next: [Stage 3 — LiDAR](stage-3-lidar.md).
