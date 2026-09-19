# Stage 4 (optional) — better compute and a depth camera

Catalog ids: `jetson`, `depth-camera`. Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT** for Israeli shops. USD prices are
> foreign list prices; "≈ ₪ with VAT" adds 18% import VAT (orders over $75) at 1 USD = ₪3.033,
> **without shipping**. Evidence: `references/research/israel-hardware-stages2-5-2026-09.md`,
> `software-versions-2026-09.md`.

**This whole stage is Optional.** No lesson in the course *requires* a Jetson or a depth camera:
every lesson runs on the Pi 5 with the USB webcam, or in simulation. Buy this stage only when
the Pi 5 is clearly the bottleneck, for example when you want real-time detection models, depth
point clouds or Isaac ROS.

## Lessons that benefit (none require it)

[00.04 Robot computers](../00-orientation/00.04-robot-computers.md),
[07.09 Depth cameras and point clouds](../07-sensors/07.09-depth-cameras-and-point-clouds.md),
[13.08 Depth and 3D vision](../13-computer-vision/13.08-depth-and-3d-vision.md),
[16.04 Models on edge compute](../16-machine-learning/16.04-models-on-edge-compute.md),
[15.04 Object pose for grasping](../15-manipulation/15.04-object-pose-estimation.md), and running
learned policies on the robot in module 18.

## Totals

| Path | Contents | ≈ ₪ | ≈ USD |
|---|---|---|---|
| **Recommended** (GPU) | Jetson Orin Nano Super (Piitel, when restocked) + NVMe SSD (estimate ₪200) + RealSense D435i (imported, with VAT) | **≈ ₪3,050** + shipping | **≈ $1,005** |
| **Cheapest** (stay on Pi 5) | Luxonis OAK-D Lite (imported, with VAT) | **≈ ₪965** + shipping | **≈ $317** |
| Skip it | Keep the Pi 5 and the USB webcam; run heavy models on your laptop over the network | ₪0 | $0 |

---

## 1. NVIDIA Jetson Orin Nano Super Developer Kit — Optional (Buy later)

| Field | Detail |
|---|---|
| Exact component | **NVIDIA Jetson Orin Nano Super Developer Kit (8 GB)**. Outside the US/CA/CN/JP/PH regions look for SKU **945-13766-0005-000**; -0000-000 is the US-region kit. |
| Alternative | Keep the Pi 5 and add an OAK-D Lite (the camera runs the neural net itself). Or offload inference to a laptop GPU over Wi-Fi (latency permitting). |
| Why | 67 TOPS GPU for real-time detection/segmentation, Isaac ROS (4.x targets Jazzy) and on-robot policy inference, at 7–25 W. |
| Price (≈, 2026-09-16) | **₪1,652** at Piitel, **out of stock** · abroad usually listed at **$249 MSRP** (search snippets; one retailer at $399), ≈ ₪890 with VAT before shipping. That's under $500, so VAT only, no duty. |
| Buy in Israel | Piitel: https://piitel.co.il/shop/nvidia-jetson-orin-nano-super-developer-kit/ · NVIDIA distributor **C.R.G. Electronics** (B2B, no public price): https://crg.co.il/catalog_category/nvidia-jetson/ |
| International | SparkFun: https://www.sparkfun.com/nvidia-jetson-orin-nano-developer-kit.html · NVIDIA marketplace: https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/ (timed out when checked on 2026-09-16) |
| Compatibility | **Version-sensitive:** the course pins **JetPack 7.2.x** (Ubuntu 24.04, CUDA 13) so ROS 2 Jazzy installs natively. From JetPack 7.2 there's **no microSD image**: you write a USB installer and install to microSD/NVMe, and a board with older firmware needs the JetPack 6 firmware update path first. Isaac ROS on Jetson needs a **128 GB+ NVMe**. |
| Power on the robot | The dev kit takes a DC barrel input (research says 9–20 V; **verify on your carrier board**). On the robot, feed it from a regulated supply, not straight from the pack. A 4S pack is one option ([Stage 6](stage-6-final-robot.md)). |
| Cables / connectors | NVMe M.2 2280 SSD (≥128 GB; not priced in the research, estimate ₪150–250 at KSP/Ivory/Bug), USB-C data cable for flashing, DC power cable |
| Tools | None beyond the basic set |
| Spares | None |
| Ask the seller | Piitel: "When is the Orin Nano Super back in stock, which SKU (-0005?), and is the power adapter included with an EU/IL plug?" C.R.G.: "Do you sell single dev kits to individuals, and at what price?" |

## 2. Depth camera

### 2a. RealSense D435i — Optional (recommended depth camera with a Jetson)

| Field | Detail |
|---|---|
| Exact component | **RealSense D435i** (stereo depth + RGB + IMU) |
| Alternative | **RealSense D405** $272 (short range ≈ 7–50 cm, good as a wrist camera for the arm). D455 $419. Orbbec Gemini 2 $234 (cheapest stereo; `OrbbecSDK_ROS2`; Jetson/ARM support not stated, so verify), https://store.orbbec.com/collections/gemini-2-series |
| Why | The most mature ROS 2 depth driver (`realsense2_camera` 4.58.4, apt binaries on Jazzy), a built-in IMU, and the largest Nav2/Isaac ROS tutorial base |
| Price | **$334** · ≈ **₪1,195** with VAT, before shipping |
| Buy in Israel | **No local stock found** (Piitel had no RealSense/OAK/Orbbec results on 2026-09-16). Plan to import. |
| International | RealSense store: https://store.realsenseai.com/ . RealSense spun out of Intel in July 2025 as an independent company; old `store.intelrealsense.com` links still appear in search. |
| ROS 2 | https://index.ros.org/p/realsense2_camera/ , https://github.com/IntelRealSense/realsense-ros |
| Compatibility | Needs a **USB 3** port (blue) for full depth streams. Works on the Jetson; on the Pi 5 it runs, but the CPU load of depth processing is high. |
| Cables / connectors | USB-C to USB-A **3.x** cable (use the supplied one; long or cheap cables drop to USB 2), 1/4"-20 tripod screw mount |
| Spares | None |
| Import note | $334 is over $75 but under $500: 18% VAT, no customs duty. **Don't put the Jetson and the camera in one order** from the same store if that pushes it over $500 (duty and purchase tax may apply). Don't split an order just to dodge tax. |

### 2b. Luxonis OAK-D Lite — Optional (recommended depth camera if you stay on the Pi 5)

| Field | Detail |
|---|---|
| Exact component | **Luxonis OAK-D Lite** (AF or FF variant) |
| Alternative | OAK-D Pro $429 / Pro W $529 (active IR, works in low light), https://shop.luxonis.com/collections/oak-cameras-col |
| Why | Runs the neural network **on the camera** (4 TOPS), so a Pi 5 without a GPU can do detection with depth. TurtleBot 4 Lite ships with this camera. |
| Price | **$269** (90 in stock; up from ≈ $149 at launch) · ≈ **₪965** with VAT, before shipping |
| Buy in Israel | No Israel/Middle East distributor listed by Luxonis (https://www.luxonis.com/distributors); Mouser Israel lists OAK-D products (https://www.mouser.co.il/, price not checked) |
| International | https://shop.luxonis.com/products/oak-d-lite-1 |
| ROS 2 | `depthai-ros`: on 2026-09-16 the apt index had `depthai-ros-driver` 2.12.2 for Jazzy (v2 API); the v3 driver is a separate package. **Verify before relying on it.** https://index.ros.org/p/depthai_ros_driver/ , https://github.com/luxonis/depthai-ros |
| Compatibility | USB-C. Needs enough USB current from the Pi 5 (`usb_max_current_enable=1`). |
| Cables / connectors | USB-C to USB-A 3.x cable (included per usual practice; verify) |
| Spares | None |

Next: [Stage 5 — robotic arm](stage-5-robotic-arm.md).
