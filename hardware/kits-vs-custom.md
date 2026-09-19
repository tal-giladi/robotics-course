# Ready-made kits vs the custom robot

Back to [HARDWARE.md](../HARDWARE.md) · [Stage 1 BOM](stage-1-first-robot.md)

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate.** Israeli prices include VAT; USD/EUR prices are
> foreign list prices without shipping or Israeli import VAT (+18% above $75). 1 USD = ₪3.033.
> Evidence: `references/research/israel-hardware-stage1-2026-09.md` (kit section) and
> `israel-hardware-stages2-5-2026-09.md` (LeKiwi); WAVE ROVER, MicroROS-Pi5 and TurtleBot 4 Lite pages re-checked 2026-09-16.

## Comparison

| Kit | Price / availability (2026-09-16) | Pros | Cons |
|---|---|---|---|
| **Custom karmel (this course)** | ≈ ₪1,775–2,550 all-in, **all parts sold in Israel** ([stage-1-first-robot.md](stage-1-first-robot.md)) | You learn power, encoders and motor control. True 2WD differential geometry for ROS 2 odometry. Every part is replaceable locally. Grows into LiDAR, arm and e-stop stages. | More work: drilling, soldering, wiring, debugging. The price includes a Pi 5 that most kits leave out. |
| **Yahboom MicroROS-Pi5** | **₪1,416** incl. VAT at Piitel, **out of stock**; **Pi 5 not included** (+₪500–800). https://piitel.co.il/shop/microros%E2%80%91pi5-ros2-robot-car-kit-for-raspberry-pi-5/ | ESP32 micro-ROS board, 4 encoder motors, MS200 LiDAR, 2 MP camera gimbal, 7.4 V battery, ROS 2 Humble course material. Closest Israel-sourced pre-built option. | Out of stock. 4WD/mecanum odometry is harder than 2WD. Material targets Humble (the course uses Jazzy). You learn less about power and wiring. A bundled battery is fine locally but a problem if imported. |
| **Waveshare UGV Rover** (6-wheel, ROS 2) | **₪2,200** at Piitel, **out of stock**. https://piitel.co.il/shop/waveshare-ugv-rover-ros-2-6-wheel-4wd-ai-robot-kit/ | Full ROS 2 platform, pan-tilt, rugged | Expensive; Pi/Jetson not included; out of stock; skid-steer odometry |
| **Waveshare WAVE ROVER** (4WD metal) | **$89.99** at waveshare.com (checked 2026-09-16), ≈ ₪322 with VAT before shipping. Resellers $81–112. Not at Piitel. https://www.waveshare.com/wave-rover.htm | Aluminium body, ESP32 driver board with JSON serial commands for a Pi or Jetson, built-in 3S 18650 UPS (cells not included; buy locally), cheapest metal base | **4× N20 12 V 200 RPM motors; no wheel encoders in the spec**, so odometry comes only from the IMU or open-loop commands. Skid-steer 4WD. Over $75, so VAT applies. |
| Waveshare UGV01 (tracked) | $169.99, https://www.waveshare.com/ugv01.htm | Rugged, ESP32 slave, expandable | Tracks slip, so odometry is poor |
| **Pololu Romi** chassis + encoder pair | $39.95 + $9.95 at Pololu (https://www.pololu.com/product/3500 , https://www.pololu.com/product/3542), ≈ ₪151. Not found at 4Project. | Excellent 2WD differential geometry. Encoders give ≈ 1,440 counts per wheel revolution (estimate). Under $75 if shipping is cheap. | 6× AA NiMH holder, plastic gear motors, needs a Romi 32U4 board or your own driver, small deck for a Pi 5 + LiDAR. Shipping cost not checked. |
| **TurtleBot 4 Lite** ("if money is no object" reference) | MSRP **$1,095** at launch (2022, https://www.openrobotics.org/blog/2022/5/3/introducing-the-turtlebot-4); **€1,699 incl. EU VAT, pre-order** at Elektor on 2026-09-16 (https://www.elektor.com/products/clearpath-robotics-turtlebot-4-lite). No Israeli seller found; buy through Clearpath's distributors (https://clearpathrobotics.com/turtlebot-4/). | The reference ROS 2 education robot: iRobot Create 3 base (encoders, IMU, cliff/bump sensors, dock), RPLIDAR A1, OAK-D Lite, official Nav2/SLAM tutorials, simulator (`turtlebot4-simulator` on Jazzy) | Over $500, so VAT **plus possible customs duty and purchase tax**. Ships with a **Raspberry Pi 4B 4 GB** (older). Top speed 0.31 m/s. You learn nothing about motors, drivers or power, and repairs mean importing parts. The course uses TurtleBot **in simulation** for free. |
| **LeKiwi** (omni-wheel base + SO-101 arm) | Community BOM: 12 V complete ≈ $482, base-only ≈ $251.50; Seeed LeKiwi 12 V kit €270.55 at OpenELAB. https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/BOM.md · https://huggingface.co/docs/lerobot/lekiwi | Official LeRobot mobile manipulator: data collection and learned policies out of the box. Uses the same Pi 5 and SO-101 as this course. | Holonomic 3-wheel base with less classic ROS 2/Nav2 material. Printed parts. Battery kits may not ship to Israel. Best as an **optional Stage 5 path** ([stage-5-robotic-arm.md](stage-5-robotic-arm.md)), not a first robot. |

## Why the course builds a custom robot

1. **The course's learning goals need it.** Stage 1 must teach you to power a robot safely,
   drive motors, read encoders, drive a square and monitor the battery. A sealed kit hides exactly those parts.
2. **Right geometry for odometry.** Two driven wheels plus a caster is the textbook
   differential drive that `diff_drive_controller`, odometry calibration (UMBmark) and Nav2 assume.
   Skid-steer 4WD, tracks and mecanum all add slip.
3. **Encoders are non-negotiable.** Closed-loop speed control (module 08) and odometry (module 09)
   need wheel encoders; the cheapest metal kit (WAVE ROVER) appears to have none.
4. **Israeli availability and repairs.** Every karmel part is sold by Piitel, 4Project,
   Hackstore or Sollan, so a burnt driver is a ₪60 local fix. The kits above were out of stock
   locally or import-only, often above the $75 VAT line, with batteries that can't be shipped.
5. **Cost.** The custom Stage 1 costs about the same as a MicroROS-Pi5 kit plus its Pi 5, and
   a fraction of a TurtleBot 4 Lite, while exceeding both as a learning platform.
6. **It grows.** The upper deck, 5 V 5.5 A rail and 3S pack are sized for the LiDAR, camera, arm and e-stop stages.

**When a kit is the better choice:** if you want to skip electronics entirely and start at ROS 2,
buy the Yahboom MicroROS-Pi5 when Piitel restocks it.
If budget is irrelevant and you want a documented reference platform, a TurtleBot 4 Lite works,
but expect import tax and slow repairs.
