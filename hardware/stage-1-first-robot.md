# Stage 1 — the first robot (karmel v1)

Catalog ids: `robot-base`, plus `tools` (see [tools.md](tools.md)). Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT** (18% Israeli VAT is included in
> Israeli shop prices). USD conversions use 1 USD = ₪3.033 (Bank of Israel rate, 2026-09-16).
> Stock at Piitel, 4Project and Hackstore changes daily; check before you order. Evidence:
> `references/research/israel-hardware-stage1-2026-09.md`.

Labels: **Buy now** = needed for the Stage 1 lessons. **Optional** = nice to have, or only on the
recommended (not cheapest) path. **Buy later** = wait until the lesson that needs it.

## What this stage unlocks

Lessons [01.04](../01-first-robot/01.04-raspberry-pi-setup.md) to
[01.15](../01-first-robot/01.15-teleop-from-laptop.md), most of module 02 (from
[02.02](../02-robot-electronics/02.02-power-budget.md)), [03.03](../03-robot-software/03.03-robust-serial-communication.md),
[03.10](../03-robot-software/03.10-networking-for-robots.md), [04.16](../04-ros2/04.16-your-robot-as-ros2-node.md),
[06.09](../06-simulation/06.09-same-code-sim-and-real.md), [07.01](../07-sensors/07.01-sensor-fundamentals.md)–[07.04](../07-sensors/07.04-tof-sensors.md),
module 08 control (except the IMU part of 08.11), [09.05](../09-odometry/09.05-calibrating-odometry.md),
[09.07](../09-odometry/09.07-odometry-in-ros2.md), the Pico foundations FC.02/FC.05/FC.06/FC.08, and
projects P01–P04, P06, P08. The shopping lesson itself is
[01.02](../01-first-robot/01.02-buying-hardware-in-israel.md).

## Architecture in one picture

```text
3S Li-ion pack (Samsung 35E ×3, 9.9–12.6 V) + 3S BMS
   └─[XT60]─[10 A fuse]─[switch]─┬─▶ 2× DRV8874 motor drivers ─▶ 2× Yahboom 520 12 V encoder motors
                                 ├─▶ 5 V 5.5 A regulator ─▶ Raspberry Pi 5 (Ubuntu Server 24.04)
                                 └─▶ INA219 (voltage/current, I²C)
Raspberry Pi 5 ──USB serial──▶ Raspberry Pi Pico 2 (MicroPython): PWM, encoders (PIO), US-100, VL53L1X, INA219
```

Physical defaults (wheel radius, gear ratio 1:56, ticks per revolution 2,464, pin map) live in
`labs/config/karmel.yaml`. If you buy different parts, edit that file.

## Safety first

> [!CAUTION]
> This stage introduces a lithium-ion pack that can deliver tens of amps into a short circuit.
> Read [SAFETY.md](../SAFETY.md) rules 4, 5 and 9 before the parts arrive, and do lesson
> [FE.14 Lithium battery safety](../optional-foundations/electronics/FE.14-lithium-battery-safety.md)
> before you assemble the pack. Charge only with the 12.6 V pack charger, on a non-flammable
> surface or in a LiPo-safe bag, never unattended, never overnight. Fuse the pack before the switch.

---

## Bill of materials (summary)

| # | Item | Label | Store | ≈ ₪ |
|---|---|---|---|---|
| 1 | Raspberry Pi 5 4 GB (8 GB recommended if budget allows, ₪800) | Buy now | Piitel | 500 |
| 2 | Official Active Cooler | Buy now | Piitel | 30 |
| 3 | microSD 64 GB (Kingston Canvas Select Plus) | Buy now | KSP | 57 |
| 4 | Official 27 W USB-C PSU (bench power) | Buy now (Optional on cheapest path) | Piitel | 70 |
| 5 | Raspberry Pi Pico 2 with headers | Buy now | Piitel | 35 |
| 6 | 2× Yahboom 520 12 V encoder motor, 205 RPM (1:56) | Buy now | Piitel | 130 |
| 7 | 2× JGB37-520 metal motor bracket | Buy now | Hackstore | 40 |
| 8 | Pololu 6 mm aluminium hub, 2-pack | Buy now | 4Project | 60.90 |
| 9 | Pololu 90×10 mm wheel pair | Buy now | 4Project | 50 |
| 10 | Pololu 3/4" metal ball caster | Buy now | 4Project | 21.10 |
| 11 | 2WD acrylic chassis plate kit | Buy now | Piitel | 50 |
| 12 | 2× Pololu DRV8874 carrier (budget: 1× TB6612FNG ₪30.30) | Buy now | 4Project | 121.20 |
| 13 | 4× Samsung INR18650-35E (3 + 1 spare) | Buy now, **locally** | Sollan | 196 |
| 14 | 3×18650 holder | Buy now | Hackstore | 16 |
| 15 | 3S 12.6 V 40 A BMS board | Buy now | Hackstore | 45 |
| 16 | 12.6 V 2 A 3S Li-ion pack charger | Buy now | Hackstore | 80 |
| 17 | Pololu D42V55F5 5 V 5.5 A regulator (budget: UBEC 5 A ₪75) | Buy now | Hackstore | 170 |
| 18 | 5×20 mm fuse holder + 10 A fuse | Buy now | Hackstore | 25 |
| 19 | DPST rocker switch (≥10 A DC) | Buy now | Hackstore | 25 |
| 20 | 2 pairs XT60 | Buy now | 4Project | 19.80 |
| 21 | INA219 module (budget: resistor divider) | Buy now (Optional on cheapest path) | Hackstore | 40 |
| 22 | US-100 ultrasonic sensor | Buy now | Hackstore | 40 |
| 23 | Pololu VL53L1X ToF carrier | Buy now (Buy later on cheapest path) | 4Project | 107.80 |
| 24 | 830-point breadboard | Buy now | 4Project | 10.90 |
| 25 | Jumper wire sets | Buy now | Piitel | 39 |
| 26 | M2.5 standoffs and screws | Buy now | any (estimate) | 30 |
| 27 | Wire, terminals, heat shrink, bulk capacitors | Buy now | any (estimate) | 60 |
| 28 | Micro-USB data cable (Pi → Pico), microSD reader if your laptop lacks one | Buy now | any (estimate) | 30 |
| | **Parts total, Pi 5 4 GB** | | | **≈ ₪2,100** |
| | Shipping from about 5 stores (estimate, not verified: ₪25–40 each) | | | ≈ ₪150 |
| | **Recommended path, all-in, Pi 5 4 GB** | | | **≈ ₪2,250 ≈ $742** |
| | **Recommended path, all-in, Pi 5 8 GB** | | | **≈ ₪2,550 ≈ $841** |
| | **Cheapest path, all-in** (changes below) | | | **≈ ₪1,775 ≈ $585** |

Cheapest-path changes (from the research budget variant):

| Change | Saving |
|---|---|
| 1× Pololu TB6612FNG dual carrier (₪30.30) instead of 2× DRV8874 | −₪90.90 |
| Hackstore UBEC 5 A (₪75) instead of the D42V55F5 | −₪95 |
| Move the VL53L1X to Stage 2 (lesson 01.13 works with the US-100 alone) | −₪107.80 |
| Skip the 27 W PSU (run the Pi from the robot pack on the bench) | −₪70 |
| 3× LG HJ2 18650 at 4Project (₪119.70) instead of 4× Samsung 35E | −₪76.30 |
| 100 k / 22 k resistor divider into a Pico ADC instead of the INA219 | −₪35 |

Workbench tools are extra: see [tools.md](tools.md) (≈ ₪870 recommended, ≈ ₪640 cheapest).

---

## A. Compute

### A1. Raspberry Pi 5 — Buy now

| Field | Detail |
|---|---|
| Exact component | Raspberry Pi 5, **4 GB** (OK) or **8 GB** (recommended if the budget allows) |
| Alternative | 4Project Pi 5 8 GB ₪784 (4 GB ₪504.90 was out of stock on 2026-09-16). Pi 5 16 GB ₪1,390 at Piitel is overkill. The 2 GB (₪320) is too small for ROS 2 + Nav2. |
| Why | The robot's main computer: Ubuntu Server 24.04, ROS 2 Jazzy, SLAM, Nav2, camera processing. 8 GB leaves room for on-board vision models and rviz. |
| Price (≈, 2026-09-16) | 4 GB **₪500** · 8 GB **₪800** (Piitel, in stock) |
| Buy in Israel | Piitel, the only official Raspberry Pi Approved Reseller in Israel: https://piitel.co.il/shop/raspberry-pi-5/ · 4Project 8 GB: https://www.4project.co.il/product/raspberry-pi5-8g |
| International | Rarely worth it. Importing saves money only if the order stays under the $75 VAT-free limit (see [importing-to-israel.md](importing-to-israel.md)); delivery is slow, and the local official reseller gives you an easy warranty path. |
| Compatibility | Ubuntu Server 24.04 LTS arm64 is supported on the Pi 5 (22.04 is not). **RPi.GPIO doesn't work on a Pi 5**; the course uses lgpio/gpiozero. A buck regulator feeding 5 V has no USB-PD negotiation, so the Pi limits USB ports to 600 mA until you set `usb_max_current_enable=1` in `config.txt` (lesson 01.07). |
| Cables / connectors | USB-C for bench power. Micro-HDMI only if you ever attach a monitor (the course is headless, lesson 01.04). |
| Tools | None |
| Spares | No spare. Protect it instead: fuse, correct regulator polarity, standoffs so nothing shorts underneath. |

### A2. Official Active Cooler — Buy now

| Field | Detail |
|---|---|
| Exact component | Raspberry Pi Active Cooler for Pi 5 |
| Alternative | None recommended |
| Why | Sustained ROS 2 load throttles a Pi 5 without a fan, which shows up as a slow, jittery robot. |
| Price | **₪30** (Piitel) |
| Buy in Israel | https://piitel.co.il/shop/active-cooler/ |
| International | Not needed |
| Compatibility | Pi 5 only. It clips into the two mounting holes and plugs into the fan header. |
| Cables / Tools / Spares | None / none / none |

### A3. microSD card — Buy now

| Field | Detail |
|---|---|
| Exact component | Kingston Canvas Select Plus 64 GB |
| Alternative | SanDisk Extreme Pro A2 128 GB ₪140 (KSP item 214036), or SanDisk High Endurance 64 GB ₪79. Official Raspberry Pi A2 32 GB ₪26 at Piitel (out of stock 2026-09-16). **Later upgrade:** NVMe SSD on an M.2 HAT+ (Piitel ₪70, out of stock), Optional. |
| Why | Boot disk for Ubuntu and your workspaces. |
| Price | **₪57** (KSP) |
| Buy in Israel | https://ksp.co.il/web/item/399645 |
| International | Not worth it |
| Compatibility | Any A1/A2 microSD works. Write it with Raspberry Pi Imager from your laptop. |
| Cables / connectors | A USB microSD reader if your laptop has no slot (estimate ≈ ₪20–30). |
| Spares | Keep a second card with a known-good image. When a card dies or an upgrade breaks something, you swap cards in a minute. |

### A4. Official 27 W USB-C power supply — Buy now (Optional on the cheapest path)

| Field | Detail |
|---|---|
| Exact component | Raspberry Pi 27 W USB-C PSU, EU plug |
| Alternative | Skip it and power the Pi from the robot pack while on a stand. A generic USB-C PD charger won't negotiate the Pi 5's 5 V 5 A profile, so the Pi limits USB current. |
| Why | Desk work (lessons 01.04–01.05) without charging and wiring a battery first. |
| Price | **₪70** (Piitel) |
| Buy in Israel | Piitel, https://piitel.co.il/ (search "27W"; the research recorded the price but not the product URL) |
| Compatibility | The EU (Type C) plug fits Israeli Type H sockets; 230 V input. |
| Spares | None |

### A5. Raspberry Pi Pico 2 — Buy now

| Field | Detail |
|---|---|
| Exact component | Raspberry Pi Pico 2 (RP2350) **with pre-soldered headers**, running MicroPython |
| Alternative | 4Project Pico 2 without headers ₪33.20 (you solder the headers, good practice for FE.17). The original Pico (RP2040) also works. ESP32-S3 at Hackstore ₪105–115 only if you want Wi-Fi micro-ROS without the Pi. |
| Why | The real-time controller: PWM for the motor drivers, quadrature encoder decoding in PIO hardware, ultrasonic timing, I²C sensors, and a 300 ms motor watchdog that works even if Linux hangs. |
| Price | **₪35** (Piitel). Hackstore sells it at ₪80, so avoid that listing. |
| Buy in Israel | https://piitel.co.il/shop/raspberry-pi-pico-2-with-headers/ |
| International | Not needed |
| Compatibility | **3.3 V logic.** US-100 and VL53L1X are 3.3 V safe. An HC-SR04's 5 V echo needs a 1 k / 2 k divider. |
| Cables / connectors | **Micro-USB data cable** to the Pi (some cheap cables are charge-only). Dupont jumpers to the breadboard. |
| Spares | **Buy 2.** At ₪35, a spare Pico saves a week when a wiring mistake kills a GPIO pin. |

---

## B. Drivetrain

### B1. Encoder gear motors ×2 — Buy now

| Field | Detail |
|---|---|
| Exact component | **Yahboom 520 DC gear motor with Hall encoder, 12 V, 205 RPM (1:56)** |
| Alternative | Same motor at 333 RPM (1:30): faster and less torque (set `gear_ratio: 30.0` in `karmel.yaml`). Hackstore JGA25-371 12 V 400 RPM with encoder ₪105 (too fast, 4 mm shaft). Makeblock 25D 185 RPM 9 V optical encoder ₪100.90 on sale at 4Project. Premium: Pololu 25D 280 RPM encoder motor ₪346.90 at 4Project. **Avoid the 550 RPM (1:19) version**: too fast for indoor navigation. |
| Why | Motion plus odometry. The encoders let you drive exact distances (01.12), close speed loops (module 08) and compute odometry (module 09). |
| Price | **₪65 each** (Piitel), ₪130 for two |
| Buy in Israel | https://piitel.co.il/shop/520-dc-gear-motor-with-encoder-550rpm/ |
| International | Yahboom direct: $7.90 bare, **$9.90 with bracket + coupling + tire**, https://category.yahboom.net/products/md520 (shipping cost to Israel not checked). RobotShop also stocks Yahboom 520 motors (buyer pays duties on delivery). |
| Compatibility | From Yahboom's spec: 12 V rated (11–16 V OK), 0.3 A rated, **stall 3 A (1:30) / 4 A (1:56)**, 6 mm D eccentric shaft, 11-line magnetic Hall AB encoder, **3.3–5 V encoder supply with built-in pull-ups** (safe to power at 3.3 V from the Pico), **PH2.0 6-pin connector**. At 1:56: 616 pulses per wheel revolution per channel, **2,464 counts with 4× decoding**. At 90 mm wheels, 205 RPM is ≈ 0.97 m/s no-load top speed. |
| Cables / connectors | PH2.0 6-pin cable to screw terminals / Dupont. If none is included, buy PH2.0 pigtails or crimp your own (crimper: [tools.md](tools.md)). |
| Tools | Hex key for the hub set screws, small screwdriver |
| Spares | Stall current is what kills gearboxes and drivers. One spare motor (₪65) is cheap insurance, and **buy it in the same batch** so the gear ratio matches. |
| **Ask the seller (Piitel)** | The page title lists 205, 333 and 550 RPM but shows one price and **no variant selector**. Ask before paying: *"Which gear ratio ships for ₪65? I need the 205 RPM (1:56) version. Is the PH2.0 6-pin encoder cable included?"* |

### B2. Motor brackets ×2 — Buy now

| Field | Detail |
|---|---|
| Exact component | JGB37-520 metal motor bracket |
| Alternative | The Yahboom bracket (bundled when ordering direct from Yahboom), or a 3D-printed bracket (F3D.05) |
| Why | Bolts the 37 mm gearbox to the chassis plate square and rigid. Wheel misalignment ruins odometry. |
| Price | **₪20 each** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%9E%D7%97%D7%96%D7%99%D7%A7-%D7%9E%D7%A0%D7%95%D7%A2-jgb37-520-%D7%9E%D7%AA%D7%9B%D7%AA/ |
| Compatibility | Uses M3 screws into the gearbox face. The Yahboom 520 is a JGB37-style gearbox, but **the hole pattern is unverified**. Compare the bracket photo with the motor face, or ask Hackstore. |
| Cables / connectors | None. You need M3 screws (usually included) and M3 bolts and nuts to fix it to the plate. |
| Tools | Drill (3.2 mm) to drill the chassis plate, screwdriver |
| Spares | None |

### B3. Wheel hubs — Buy now

| Field | Detail |
|---|---|
| Exact component | Pololu universal aluminium mounting hub, 6 mm shaft, M3 holes, 2-pack |
| Alternative | The coupling bundled with Yahboom's direct-order motor kit, or an AliExpress JGB37 wheel kit |
| Why | Joins the 6 mm D shaft to the wheel with a set screw |
| Price | **₪60.90** for 2 (4Project) |
| Buy in Israel | https://www.4project.co.il/product/aluminum-mounting-hub-6mm-shaft-m3-holes-2pack |
| Compatibility | 6 mm shaft. Tighten the set screw on the flat of the D shaft. Fits the Pololu wheels below. |
| Tools | Small hex key (usually included) |
| Spares | Set screws loosen. Use a drop of removable thread-locker and re-check after the first runs. |

### B4. Wheels — Buy now

| Field | Detail |
|---|---|
| Exact component | Pololu 90×10 mm wheel pair (black) |
| Alternative | Pololu 70×8 mm pair ₪39.30 (slower, lower). 65 mm rubber wheels from an AliExpress JGB37 kit. |
| Why | 90 mm diameter gives `wheel_radius_m: 0.045` in `karmel.yaml` and good ground clearance. |
| Price | **₪50** per pair (4Project) |
| Buy in Israel | https://www.4project.co.il/product/500 |
| International | Pololu: https://www.pololu.com/product/1439 |
| Compatibility | Pololu says these wheels have M3 holes at 12.7 and 19.1 mm spacing, compatible with the universal hubs. The 10 mm width is fine indoors. |
| Spares | None needed |

### B5. Ball caster — Buy now

| Field | Detail |
|---|---|
| Exact component | Pololu 3/4" metal ball caster |
| Alternative | Pololu 1/2" metal ball caster ₪13.90 |
| Why | Third contact point for a 2-wheel differential drive |
| Price | **₪21.10** (4Project) |
| Buy in Israel | https://www.4project.co.il/product/986 |
| Compatibility | **Match heights**: the wheel axle sits ≈ 45 mm above the floor minus the bracket offset. You'll likely need M3 spacers under the caster so the robot sits level. A caster that is too high makes the robot rock; too low lifts a drive wheel. |
| Tools | Calipers or a ruler |
| Spares | None |

### B6. Chassis plate — Buy now

| Field | Detail |
|---|---|
| Exact component | Piitel "Smart Robot Car Chassis Kit, 2 wheels" (use the acrylic plates; keep its TT motors as spares for experiments) |
| Alternative | Piitel "Robot Car Chassis 2WD" ₪122. **DIY 3–4 mm plywood, acrylic or aluminium plate**, cut, laser-cut at a makerspace, or 3D-printed ([3d-printing-options.md](3d-printing-options.md)). A **two-deck** plate with 30–40 mm standoffs leaves room for the LiDAR in Stage 3. |
| Why | Holds everything. Target ≈ 250 × 200 mm (`karmel.yaml` chassis). |
| Price | **≈ ₪50** (from Piitel's search listing; the product page didn't show a price) |
| Buy in Israel | https://piitel.co.il/shop/smart-robot-car-chassis-kit-2-wheels/ |
| Compatibility | TT-motor plates **aren't drilled for 37 mm motors, so expect to drill**. Plan the upper deck now. |
| Tools | Drill and bits (3 mm, 3.2 mm), file, marker |
| Ask the seller | Confirm the price and the plate dimensions. |

### B7. Motor drivers — Buy now

| Field | Detail |
|---|---|
| Exact component | **2× Pololu DRV8874 single brushed DC motor driver carrier** (4.5–37 V, 2.1 A continuous, current-sense output) |
| Alternative | **Budget: 1× Pololu TB6612FNG dual carrier** ₪30.30 at 4Project, https://www.4project.co.il/product/1145 (Hackstore ₪35–50). It works for a light robot, but at 1.2 A continuous / 3.2 A peak a motor stall can trip its thermal shutdown. Also 2× Pololu TB9051FTG ₪65.70 each. Cytron MDD3A is good value but no Israeli stockist was found (RobotShop). **Avoid the DRV8833** (motor supply max 10.8 V, below a full 3S pack) and the **L298N** (drops ≈ 2–5 V as heat, poor with a 3.3 V Pico). |
| Why | The Pico's pins can't drive motors. An H-bridge switches pack voltage to the motor under PWM control. |
| Price | **₪60.60 each**, ₪121.20 for two (4Project) |
| Buy in Israel | https://www.4project.co.il/product/drv8874-single-brushed-dc-motor-driver-carrier |
| International | Pololu direct (FedEx). 4Project is Pololu's listed Israeli distributor, so buy locally. |
| Compatibility | 3.3 V logic OK. The course wires them in **IN/IN mode** (PMODE and nSLEEP tied high; Pico GP2/GP3 left, GP6/GP7 right, `karmel.yaml`). IPROPI current sense can go to a Pico ADC (GP27) for stall detection. Put a **100–470 µF electrolytic capacitor** across VIN near the drivers. |
| Cables / connectors | Header pins (solder them), 18 AWG to the motors and power, Dupont to the Pico |
| Tools | Soldering iron (headers), wire stripper |
| Spares | **Buy 3** if you can (₪60.60). A reversed battery or a shorted motor lead kills a driver instantly. |

---

## C. Power

> [!CAUTION]
> **Buy lithium cells and packs in Israel.** Parcels with lithium cells often fail to ship to
> Israel from AliExpress (see [importing-to-israel.md](importing-to-israel.md)), and loose cells
> of unknown origin are a fire risk. Chargers, BMS boards and holders contain no cells and import normally.

### C1. 18650 cells ×4 — Buy now (locally)

| Field | Detail |
|---|---|
| Exact component | **Samsung INR18650-35E**, 3,500 mAh, new and matched: 3 for the pack + 1 spare |
| Alternative | LG HJ2 18650 at 4Project ₪39.90 each (discharge rating not checked), https://www.4project.co.il/product/18650-lg-hj2-battery-3.7v-3000mah . Cell-Tec lists Samsung 35E at ₪29 (search snippet only; the product URL redirected, **unverified**). |
| Why | The 3S pack (9.9–12.6 V) powers the 12 V motors directly and the Pi through the regulator. The 35E capacity gives long sessions. |
| Price | **₪98 per pair** (sale), ₪196 for 4 (Sollan) |
| Buy in Israel | https://www.sollan.co.il/product/%D7%96%D7%95%D7%92-%D7%A1%D7%95%D7%9C%D7%9C%D7%95%D7%AA-%D7%9C%D7%99%D7%AA%D7%99%D7%95%D7%9D-samsung-inr18650-35e-%D7%9E%D7%A7%D7%A6%D7%95%D7%A2%D7%99%D7%95%D7%AA-%D7%91%D7%94%D7%A1%D7%A4%D7%A7-%D7%90/ |
| Compatibility | Sollan's page says "20 A". Samsung's datasheet rates the 35E around **8 A continuous** (research figure from memory, verify). Budget ≈ 3 A typical and up to 8 A with both motors stalled. Use **flat-top or button-top to match your holder**. |
| Spares | 1 spare cell from the **same batch**. Never mix old and new, or different brands, in one pack. |
| Ask the seller | "Are these genuine Samsung cells from an authorized source, and what is the manufacture date?" Reject cells with torn wraps or dents. |

### C2. 3×18650 holder — Buy now

| Field | Detail |
|---|---|
| Exact component | 3-cell series 18650 holder |
| Alternative | A spot-welded 3S pack from a local pack builder (for example LaBatteria, https://www.labatteria.co.il/ ; whether they build small packs and what it costs is **unverified**) |
| Price | **₪16** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%9E%D7%97%D7%96%D7%99%D7%A7-3-%D7%A1%D7%95%D7%9C%D7%9C%D7%95%D7%AA-18650/ |
| Compatibility | Spring contacts add resistance. They're fine at a few amps, but touch-check them for heat after a stall test. |
| Spares | None |

### C3. 3S BMS (protection board) — Buy now

| Field | Detail |
|---|---|
| Exact component | 3S 12.6 V 40 A charge/balance protection board |
| Alternative | Hackstore 3S 25 A (out of stock 2026-09-16) |
| Why | Over-discharge, overcharge and short-circuit protection for the pack |
| Price | **₪45** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%91%D7%A7%D7%A8-%D7%98%D7%A2%D7%99%D7%A0%D7%94-%D7%95%D7%90%D7%99%D7%96%D7%95%D7%9F-%D7%A2%D7%93-3s-12-6v-40a/ |
| Compatibility | Charge and discharge through the BMS P+ / P− terminals. Wire the balance taps B1/B2 exactly as printed. **Check each tap voltage with the multimeter before connecting the board.** |
| Cables / connectors | 18 AWG silicone wire, XT60 on the output |
| Tools | Soldering iron, multimeter, heat shrink |
| Spares | None. Replace the board if it ever gets hot or stops cutting off. |

### C4. Pack charger — Buy now

| Field | Detail |
|---|---|
| Exact component | 12.6 V 2 A charger for 3-cell Li-ion packs (230 V input) |
| Alternative | Hackstore 12.6 V 3 A "medical standard" ₪220. For LiPo packs (Stage 6): Hackstore B3 PLUS 2S/3S balance charger ₪115, or RCZone RCToolkit C3 ₪220 (https://rczone.co.il/). |
| Why | A Li-ion pack must be charged with a CC/CV charger that stops at 4.2 V per cell (12.6 V). **Never use a bench supply or laptop charger.** |
| Price | **≈ ₪80** (Hackstore search listing) |
| Buy in Israel | https://hackstore.co.il/product/%D7%A1%D7%A4%D7%A7-%D7%9B%D7%95%D7%97-%D7%9E%D7%98%D7%A2%D7%9F-%D7%9E%D7%90%D7%A8%D7%96-%D7%A1%D7%95%D7%9C%D7%9C%D7%95%D7%AA-%D7%9C%D7%99%D7%AA%D7%99%D7%95%D7%9D-12-6v-2a/ |
| Compatibility | Israeli 230 V 50 Hz. Output is a DC barrel jack; one listing says 3.5 × 1.35 mm (**verify**) and fit a matching panel jack on the robot. |
| Safety | [SAFETY.md](../SAFETY.md) rule 5: non-flammable surface or LiPo-safe bag ([tools.md](tools.md)), never unattended, never overnight. |
| Ask the seller | "What is the output connector size and polarity (center positive?)" |

### C5. 5 V regulator for the Pi 5 — Buy now

| Field | Detail |
|---|---|
| Exact component | **Pololu D42V55F5**, 5 V 5.5 A step-down regulator |
| Alternative | Pololu D36V50F5 (5 V 5.5 A) ₪202.60 at 4Project. **Budget: Hackstore UBEC 5 A ₪75.** Not recommended: Sollan 5 V / 5 A buck (9–35 V in) ₪198, whose 9 V minimum input is too close to a discharged 3S pack. |
| Why | A Pi 5 under load plus USB devices (LiDAR, camera later) needs up to 5 A at a stable 5 V. A weak regulator causes brown-out reboots when the motors start (lesson 02.02). |
| Price | **₪170** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%9E%D7%95%D7%A8%D7%99%D7%93-%D7%9E%D7%AA%D7%97-d42v55f5-%D7%9E%D7%99%D7%A6%D7%91-5v-%D7%A2%D7%93-6a/ |
| Compatibility | Feed the Pi through its USB-C or the GPIO 5 V pins. The GPIO pins **bypass the Pi's input protection**, so the fuse and correct polarity matter. Set `usb_max_current_enable=1`. Short, thick leads; common ground with the drivers and Pico. |
| Cables / connectors | A USB-C pigtail or 20 AWG to the GPIO header |
| Tools | Soldering iron, multimeter. **Measure 5.0–5.2 V before connecting the Pi.** |
| Spares | None |

### C6. Fuse, switch and connectors — Buy now

| Field | Detail |
|---|---|
| Exact component | 5×20 mm panel fuse holder ₪20 + 10 A fuse ₪5 · DPST rocker switch 29×21 mm ₪25 · 2 pairs of XT60 (male ₪5.10 + female ₪4.80) |
| Alternative | Inline automotive blade-fuse holder (car-parts store) · Hackstore XT60 inline switch ₪65 · Hackstore XT60 pair ₪18 |
| Why | [SAFETY.md](../SAFETY.md) rules 2 and 4: a fuse right at the battery positive, **before** the switch, and a switch you can reach in one second. XT60 makes the battery removable for charging. |
| Buy in Israel | Fuse holder: https://hackstore.co.il/product/%D7%91%D7%99%D7%AA-%D7%A0%D7%AA%D7%99%D7%9A-5x20-%D7%9E%D7%9E-%D7%9C%D7%A9%D7%A7%D7%A2-%D7%A4%D7%90%D7%A0%D7%9C/ · XT60: https://www.4project.co.il/product/xt60-male-connector · switch: Hackstore (search "rocker"/"מתג") |
| Compatibility | **The current rating isn't shown on the Hackstore switch and fuse-holder listings.** Choose parts rated **≥ 10 A DC**. |
| Tools | Soldering iron (XT60 is soldered), heat shrink |
| Spares | **Buy 5 fuses** (₪5 each); keep the spares taped inside the chassis. |
| Ask the seller | "What is the DC current rating of this switch and this fuse holder?" |

### C7. INA219 voltage/current sensor — Buy now (Optional on the cheapest path)

| Field | Detail |
|---|---|
| Exact component | INA219 I²C module |
| Alternative | INA226 module ₪55 at Hackstore (better resolution, but only ≈ 0.8 A with its 0.1 Ω shunt; used in Stage 6). A 100 k / 22 k resistor divider into Pico ADC0 (GP26), ≈ ₪5, for voltage only. |
| Why | Lesson [01.14](../01-first-robot/01.14-battery-monitoring.md): battery state and low-battery shutdown (warning 10.5 V, cutoff 9.9 V in `karmel.yaml`) |
| Price | **₪40** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%9E%D7%95%D7%93%D7%95%D7%9C-%D7%97%D7%99%D7%99%D7%A9%D7%9F-%D7%9E%D7%AA%D7%97-%D7%95%D7%96%D7%A8%D7%9D-ina219/ |
| Compatibility | I²C at 3.3 V (Pico GP4/GP5). With the usual 0.1 Ω shunt it reads to ≈ 3.2 A. That's fine for bus voltage, but whole-robot current can exceed it; for that, swap the shunt (0.01 Ω) or measure only a branch. |
| Tools | Soldering iron for the headers |

---

## D. Sensors

### D1. US-100 ultrasonic sensor — Buy now

| Field | Detail |
|---|---|
| Exact component | US-100 ultrasonic range sensor |
| Alternative | HC-SR04 ₪11.90 at 4Project. It needs 5 V and a **1 k / 2 k divider** on echo into the 3.3 V Pico (lesson 02.05). |
| Why | Lessons [01.13](../01-first-robot/01.13-distance-sensors-stop.md) and [07.03](../07-sensors/07.03-ultrasonic-sensors.md): stop before hitting things |
| Price | **₪40** (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%D7%97%D7%99%D7%99%D7%A9%D7%9F-%D7%9E%D7%A8%D7%97%D7%A7-%D7%90%D7%95%D7%9C%D7%98%D7%A8%D7%90%D7%A1%D7%95%D7%A0%D7%99-us-100/ |
| Compatibility | Runs at 3.3 V, so no level shifting. Pulse mode on GP14/GP15 (`karmel.yaml`). It also has a UART mode (remove the jumper). |
| Spares | An HC-SR04 at ₪11.90 is a cheap second sensor for comparison experiments. |

### D2. VL53L1X time-of-flight sensor — Buy now (Buy later on the cheapest path)

| Field | Detail |
|---|---|
| Exact component | Pololu VL53L1X carrier with voltage regulator |
| Alternative | **Adafruit VL53L1X ₪80 at Piitel** (in stock), https://piitel.co.il/shop/adafruit-vl53l1x-time-of-flight-distance-sensor-4000mm-range/ · Pololu VL53L0X ₪93.70 (≈ 2 m range) · Hackstore GY-53-L1X ₪95 (UART/PWM module) · SparkFun Qwiic VL53L1X ₪190.90 |
| Why | Lessons 01.13 and [07.04](../07-sensors/07.04-tof-sensors.md): narrow-beam optical ranging up to ≈ 4 m, which complements the wide ultrasonic cone |
| Price | **₪107.80** (4Project) |
| Buy in Israel | https://www.4project.co.il/product/vl53l1x-distance-sensor-regulated-pololu |
| Compatibility | I²C, shared bus with the INA219 (different addresses). No mature ROS 2 package: the Pico reads it and publishes `sensor_msgs/Range` via the Pi. |
| Tools | Soldering iron for the headers |

---

## E. Prototyping and consumables — Buy now

| Item | Exact component / where | ≈ ₪ | Notes |
|---|---|---|---|
| Breadboard | 830-point, 4Project https://www.4project.co.il/product/breadboard-830-points | 10.90 | Hackstore 830-point ₪15. For first wiring only; lesson 02.08 moves you to perfboard. |
| Jumpers | Piitel "Premium Jumper Wires 40×150 mm" ₪16 + 40× female–female 200 mm ₪23 (https://piitel.co.il/) | 39 | Also get a male–female set; the type on the premium set is unverified. |
| Standoffs | M2.5 nylon or brass kit for the Pi and Pico (estimate) | 30 | Not found at Piitel or Hackstore in quick searches. Hackstore stocks M3/M4 nylon spacers at ₪1–4 each. |
| Wire and terminals | 18 AWG silicone red/black (power), 24 AWG (signals), screw terminals, 100–470 µF 25 V capacitors (estimate) | 60 | Hackstore heat shrink ₪6–14 per size |
| Cables | Micro-USB data cable Pi → Pico (estimate) | 15–30 | Test that it carries data, not only charge. |

**Spare-parts strategy for Stage 1:** keep 1 extra Pico 2, 1 extra DRV8874, 1 extra motor from the
same batch, 5 fuses, 1 spare 35E cell, a second microSD with a known-good image, and a bag of
M3/M2.5 screws. That's ≈ ₪250 and covers the failures beginners actually cause (reversed
polarity, stalled motors, shorted GPIO).

## What to do next

1. Order the tools first ([tools.md](tools.md)) and read [SAFETY.md](../SAFETY.md).
2. Send Piitel the gear-ratio question before ordering the motors.
3. Continue with [Stage 2](stage-2-imu-and-camera.md) only when you reach module 07.
