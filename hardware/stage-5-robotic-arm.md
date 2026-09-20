# Stage 5 — robotic arm

Catalog id: `arm` (uses the Stage 2 `camera` too). Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate.** Israeli shop prices include VAT. USD prices are
> foreign list prices; "≈ ₪ with VAT" adds 18% import VAT at 1 USD = ₪3.033, **without shipping**.
> Evidence: `references/research/israel-hardware-stages2-5-2026-09.md`.

**When to buy:** at module 14. Shipping from China/US vendors can take weeks, so order about a
month ahead. No complete arm kit was found in stock in Israel; plan to import.

## What this stage unlocks

Lessons [14.02 Buying and assembling the arm](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md),
[14.03](../14-robotic-arm/14.03-controlling-servos.md), [14.08](../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md),
[14.10](../14-robotic-arm/14.10-move-gripper-to-position.md), [14.11 Arm safety](../14-robotic-arm/14.11-arm-safety.md),
[15.03](../15-manipulation/15.03-hand-eye-calibration.md), [15.06](../15-manipulation/15.06-visual-servoing.md),
[15.07](../15-manipulation/15.07-detect-and-approach.md), [15.08](../15-manipulation/15.08-pick-and-place-pipeline.md),
[15.10 Mobile manipulation](../15-manipulation/15.10-mobile-manipulation.md),
[18.03 Teleop data collection](../18-embodied-ai/18.03-teleop-data-collection.md),
[18.06 Train your first LeRobot policy](../18-embodied-ai/18.06-train-first-policy-lerobot.md),
[18.08 Fine-tuning a VLA](../18-embodied-ai/18.08-fine-tuning-a-vla.md); projects
[P14](../projects/P14-robotic-arm.md), [P15](../projects/P15-vision-guided-arm.md), [P16](../projects/P16-pick-and-place.md).

## Totals

| Path | Contents | ≈ ₪ | ≈ USD |
|---|---|---|---|
| **Recommended** | SO-101 leader + follower, WowRobo Package 2 (unassembled, with camera) + clamps/PSU/cables allowance (estimate ₪100) | **≈ ₪1,030** + shipping | **≈ $339** |
| **Cheapest** | WowRobo Package 1 (printed parts + 12 servos) + ₪100 allowance for anything not included (**verify contents**) | **≈ ₪810** + shipping | **≈ $268** |
| Alternative | Waveshare RoArm-M3 (S / Pro) | ≈ ₪860–1,220 with VAT + shipping | $239.99–$339.99 |
| Optional add-on | LeKiwi mobile-manipulator base (see below) | see below | |

> [!CAUTION]
> Arms move faster than you can react, and servos pinch. Before powering the arm read
> [SAFETY.md](../SAFETY.md) and lesson [14.11](../14-robotic-arm/14.11-arm-safety.md): torque and
> speed limits in software first, face out of the workspace, power switch within reach.

> [!IMPORTANT]
> **The arm eats karmel's forward tipping margin, and the front caster is what pays for it.**
> karmel's ball caster sits **ahead** of the wheel axle (`labs/config/karmel.yaml`:
> `caster_offset_x_m: +0.10`), because even without an arm the centre of mass is already ≈ 1.7 mm
> forward of the axle — see [stage-1-first-robot.md](stage-1-first-robot.md) B5 and `labs/TESTED.md`
> §3.1, where a *rear* caster left the robot resting 5.6° nose-down. A 700 g arm reaching forward
> moves the centre of mass further forward still, and the support polygon's front edge is that
> caster. Two consequences when you mount the arm ([15.10](../15-manipulation/15.10-mobile-manipulation.md),
> Stage 6): **mount it low and as close to the axle as the workspace allows**, and **move the
> battery rearward** to counterweight it. When you bench-test a table-mounted arm this does not
> matter; the first time the arm extends on the moving base, it is the whole problem.

---

## 1. SO-101 leader + follower kit (LeRobot) — Buy now (at Stage 5)

| Field | Detail |
|---|---|
| Exact component | **SO-101 (SO-ARM101) leader + follower arm kit**, the flagship arm of Hugging Face LeRobot. Recommended purchase: **WowRobo Package 2** (unassembled leader + follower + camera). |
| Alternatives (same arm) | WowRobo Package 1 $199 (printed parts + 12 servos) · WowRobo Package 3 $299 (assembled) · **Seeed** servo kit Pro $249.90 (no printed parts or camera) + printed parts $29.90, https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html · **PartaBot** (US) electronics-only $329 / full kit $399 / assembled $479, https://partabot.com/products/so-arm101 · **DIY BOM** ≈ $229.88 for 2 arms or $121.94 for one follower, printing the parts yourself, https://github.com/TheRobotStudio/SO-ARM100 |
| Why | Stage 5 is about manipulation **and imitation learning**. With a leader arm you teleoperate the follower, record demonstrations and train ACT/SmolVLA-style policies (module 18). SO-101 is the cheapest arm with first-class LeRobot support, and it's also the arm on LeKiwi. |
| Price (≈, 2026-09-16) | Package 2 **$259** ≈ **₪927** with VAT · Package 1 $199 ≈ ₪712 · Seeed kit + parts $279.80 ≈ ₪1,001 |
| Buy in Israel | **No complete kit found in Israel.** Local spares: Piitel 30 kg serial-bus servo (ST3215-class, 12 V) **₪107**, https://piitel.co.il/shop/30kg-serial-bus-servo-high-precision-and-torque-with-programmable-360-degrees-magnetic-encoder/ · 4Project Feetech FE-URT-1 serial servo adapter **₪59.40**, https://www.4project.co.il/product/feetech-fe-urt-1-serial-servo-adapter . 3D-print parts locally: [3d-printing-options.md](3d-printing-options.md). |
| International | WowRobo: https://shop.wowrobo.com/products/so-arm101-diy-kit-assembled-version-1 · Seeed and PartaBot links above. Docs: https://huggingface.co/docs/lerobot/so101 |
| Compatibility | Follower: 6× Feetech STS3215 servos at 7.4 V (1/345 gearing). The leader mixes 1/191, 1/345 and 1/147 gearing. The 7.4 V version runs from a 5 V PSU; **a 12 V version** exists (30 kg·cm vs 16.5 kg·cm stall). **Choose the 12 V servos if the arm will later run from the robot's 3S battery** (Stage 6). ROS 2 / MoveIt 2 support is community-maintained (no vendor package): https://github.com/ros-physical-ai/ros2_so_arm , https://github.com/legalaspro/so101-ros-physical-ai . Payload and repeatability are **not published** (hobby servos, low payload). |
| Cables / connectors | USB-C data cables from each servo bus board to the computer (one per arm), the servo daisy-chain cables (included in kits), a DC PSU matching the servo voltage, and 2 **table clamps** (C-clamps) to hold the arms down |
| Tools | Small Phillips screwdrivers, hex keys; assembly takes an afternoon or two per arm (lesson 14.02) |
| Spares | **1–2 spare servos of the follower's type.** The wrist and gripper servos are the ones that stall and die. Local: Piitel ₪107 (12 V ST3215-class, so check voltage and gearing match) or order extras with the kit. Keep a spare set of printed gripper parts (cheap to reprint). |
| **Ask the seller** | WowRobo/Seeed: "Do you ship to Israel now, with which courier, and how much? Does the package include the servo driver boards, power supplies (with which plug) and USB cables? 7.4 V or 12 V servos?" Seeed posted a suspension of shipments to Israel in March 2026; **email order@seeed.io** to confirm current status. |
| Mains power | Israel is 230 V 50 Hz. Kit PSUs rated 100–240 V work with a plug adapter; **never use a 110 V-only supply**. |

## 2. Waveshare RoArm-M3 — Alternative

| Field | Detail |
|---|---|
| Exact component | **Waveshare RoArm-M3-S** or **RoArm-M3-Pro** (Pro uses metal ST3235 servos) |
| Alternative | RoArm-M2-S (4-DOF) $179.99–$299.99, https://www.waveshare.com/roarm-m2-s.htm |
| Why choose it | **Official** ROS 2 workspace, URDF and MoveIt 2 demos (https://github.com/waveshareteam/roarm_ws , https://www.waveshare.com/wiki/RoArm-M3_Moveit_MTC_Demonstration). It runs on **12 V 5 A** and accepts a 3S Li-ion battery, so it fits a mobile base. Claimed payload up to 1 kg (the wiki also says 200 g at 0.5 m), ≈ ±5 mm. |
| Trade-off | LeRobot support is a Waveshare claim only, not in LeRobot's main docs. No leader arm, so imitation learning (module 18) is harder. |
| Price | **$239.99–$339.99** ≈ ₪860–1,220 with VAT, before shipping |
| Buy in Israel | Piitel carries other Waveshare robots but had **no RoArm listing** on 2026-09-16. Ask Piitel to order one. |
| International | https://www.waveshare.com/roarm-m3.htm (Waveshare ships worldwide; local taxes are the buyer's responsibility) |
| Cables / Spares | 12 V 5 A PSU (check included plug), USB cable; spare ST3235/ST3215 servos |

## 3. Not recommended for this course

| Arm | Why not |
|---|---|
| Elephant Robotics myCobot 280 Pi ($799) | Polished, but ROS 2 support stops at Humble, and its Pi 4B duplicates the robot's compute |
| Hiwonder ArmPi Ultra ($299.99–$809.99), xArm 1S / xArm ESP32 / LeArm AI | Vendor tutorials, no LeRobot, toy-class for the cheaper ones |
| Trossen PincherX 100 / WidowX 250 S | Discontinued. The replacement WidowX AI starts at $2,995. |
| UFACTORY Lite 6 ($3,500) | A real arm, but 7.2 kg and 24 V 16.5 A: desk-mounted only |

## 4. Optional path — LeKiwi mobile manipulator

| Field | Detail |
|---|---|
| What | **LeKiwi**: a 3-omni-wheel base carrying an SO-101 follower and a Pi 5, an official LeRobot robot |
| Label | **Optional**. The course's main path mounts the arm on karmel instead (lesson 15.10, Stage 6). Choose LeKiwi if you specifically want omni-wheel mobile manipulation with the LeRobot stack. |
| Price | Community BOM: **12 V complete ≈ $482** (includes a Pi 5 4 GB at $60 and 2 USB cameras, which you already own, so subtract them) · base-only 12 V ≈ $251.50 · Seeed LeKiwi 12 V kit (base + printed parts + battery) **€270.55** at OpenELAB. EUR→₪ was not converted in the research. |
| Where | BOM: https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/BOM.md · docs: https://huggingface.co/docs/lerobot/lekiwi · Seeed full kit: https://www.seeedstudio.com/LeKiwi-Full-Kit-12V-Verision.html · OpenELAB (EU): https://openelab.io/products/seeed-studio-lekiwi-kit12v-version-mobile-base-with-3d-printed-parts-and-battery |
| Battery warning | A kit that **includes a Li-ion battery** may not ship to Israel (lithium restrictions). Prefer the version without a battery and buy the pack locally ([Stage 6](stage-6-final-robot.md)). |

Next: [Stage 6 — final robot](stage-6-final-robot.md).
