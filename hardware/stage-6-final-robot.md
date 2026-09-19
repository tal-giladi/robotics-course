# Stage 6 — the final robot: e-stop, power v2, mounting

Catalog id: `estop`. Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT.** 1 USD = ₪3.033. Evidence:
> `references/research/israel-hardware-stages2-5-2026-09.md` (power, safety, 3D printing).
> Items marked "estimate" were not priced in the research.

**When to buy:** at module 20. By then you know how much current the base + arm + compute
actually draw (you measured it with the INA219/INA226), so you can size the battery and relay correctly.

## What this stage unlocks

[20.02 Hardware integration](../20-final-robot/20.02-hardware-integration.md),
[20.04 The safety case](../20-final-robot/20.04-safety-case.md), and the
[final project P18](../projects/P18-final-autonomous-ai-robot.md). The e-stop also protects the
arm lessons from [14.11](../14-robotic-arm/14.11-arm-safety.md) onward, so you may buy it earlier.

## Totals

| Path | Contents | ≈ ₪ | ≈ USD |
|---|---|---|---|
| **Recommended** | E-stop ₪67.20 + 5 V-coil 20 A relay ₪27.60 + INA226 ₪55 + 3S 5000 mAh LiPo ₪285 + balance charger ₪220 + mounting hardware (estimate ₪150) + printed brackets via a service (estimate ₪200) | **≈ ₪1,005** | **≈ $331** |
| **Cheapest** | E-stop ₪67.20 + 12 V-coil 10 A relay ₪3.90 + mounting hardware (estimate ₪100); keep the Stage 1 pack | **≈ ₪171** | **≈ $56** |
| Optional | Bambu Lab A1 mini 3D printer (official importer) | ₪990 | $326 |

---

## 1. Emergency stop button — Buy now (at Stage 6)

| Field | Detail |
|---|---|
| Exact component | Erco "לחצן פטריה בקופסא אדום ננעל 1NC": red **latching mushroom button, 1NC, in an enclosure** |
| Alternative | Erco key-release e-stop 1NO+1NC **₪169.20**, https://www.erco.co.il/b2c/command-and-control/command-devices-22mm/62115020640.html . The extra NO contact lets the Pico sense the e-stop state. |
| Why | [SAFETY.md](../SAFETY.md) rule 2 and "The emergency stop": remove actuator energy **in hardware**, independent of software, while the computers stay powered to log and shut down cleanly |
| Price | **₪67.20** (Erco) |
| Buy in Israel | https://www.erco.co.il/b2c/9922516.html |
| Compatibility | Wire the NC contact **in series with the relay/contactor coil** that feeds the motor drivers and arm servos, not the Pi/Pico supply. Releasing the button must **not** restart motion; software requires an explicit re-enable. To let ROS know the e-stop state, sense the line on a GPIO (with a 1NO+1NC button, use the NO contact). |
| Cables / connectors | 22 AWG to the coil circuit, cable gland or grommet, ferrules or spade terminals |
| Tools | Drill for mounting, crimper or soldering iron, multimeter (continuity test NC/NO) |
| Spares | None. **Test it at the start of every autonomous session.** |

## 2. Relay or contactor in the actuator supply — Buy now (at Stage 6)

| Field | Detail |
|---|---|
| Exact component | **SPDT relay, 5 V coil, 20 A**, powered from the 5 V rail through the e-stop NC contact |
| Alternative | Cheapest: SPDT sealed relay **12 V coil, 10 A** ₪3.90, https://www.4project.co.il/product/spdt-relay-12v-coil-10a . It only suits the base alone: two 520 motors can stall at 4 A each. For base + arm, use an **automotive 40 A relay or a DC contactor** (car-parts store; not priced, estimate ₪30–60), with a fuse. |
| Why | The e-stop button contacts shouldn't carry motor current directly. A relay rated above the worst-case actuator current does. |
| Price | **₪27.60** (4Project) |
| Buy in Israel | https://www.4project.co.il/product/spdt-relay-5v-coil-20a |
| Compatibility | Size it for **stall current of every actuator on that supply** plus margin. Add a flyback diode across the coil if the module doesn't have one. A bare relay coil needs more current than a GPIO pin can supply; the e-stop NC contact switches it directly. |
| Spares | One spare relay (cheap) |

## 3. Power monitor v2: INA226 — Buy now (at Stage 6)

| Field | Detail |
|---|---|
| Exact component | INA226 I²C module |
| Alternative | Keep the Stage 1 INA219. 4Project only has the analog INA169 (₪91.60). |
| Why | Bus voltage (to 36 V) and shunt current for a `sensor_msgs/BatteryState` publisher and low-battery shutdown on the bigger robot |
| Price | **₪55** in stock (Hackstore) |
| Buy in Israel | https://hackstore.co.il/product/%d7%9e%d7%95%d7%93%d7%95%d7%9c-%d7%97%d7%99%d7%99%d7%a9%d7%9f-%d7%9e%d7%aa%d7%97-%d7%95%d7%96%d7%a8%d7%9d-ina226/ |
| Compatibility | The common 0.1 Ω shunt limits it to ≈ 0.8 A. For whole-robot current, fit a smaller shunt (for example 0.01 Ω) and set the calibration register. |

## 4. Bigger battery — Optional (only if runtime is too short)

Measure first: log pack current with the INA219/INA226 during a real session and divide the
Stage 1 pack's 3.5 Ah by it (runtime was not measured in the research). An arm or a Jetson
shortens runtime a lot. Upgrade only if you measure a problem.

| Option | Details | ≈ ₪ (2026-09-16) | Source |
|---|---|---|---|
| **3S LiPo 11.1 V 5000 mAh** (recommended when the 12 V arm shares the pack) | FullymaX 30C ₪285 or 45C ₪250. Needs a **LiPo balance charger** and a LiPo bag. | 250–285 | Skyline (Kav HaOfek): https://www.skylineairline.co.il/%D7%A1%D7%95%D7%9C%D7%9C%D7%95%D7%AA-lipo |
| 3S LiPo 3300 mAh | Smaller step up | 225 | same |
| 4S LiPo 14.8 V 4400 mAh 70C XT90 | For a Jetson + motors (always through a regulator; check the Jetson's input range) | 520 | same |
| **DIY Li-ion 3S2P** (safer chemistry than LiPo, longer runtime) | 6× LG HJ2 18650 (₪39.90 each) + a 3S BMS; reuses the Stage 1 12.6 V charger | ≈ 240 for the cells + BMS | 4Project: https://www.4project.co.il/product/18650-lg-hj2-battery-3.7v-3000mah |
| LiPo balance charger | RCZone RCToolkit C3 2–3S (₪250, on sale ₪220), or Hackstore B3 PLUS 2S/3S ₪115 | 115–220 | https://rczone.co.il/ · https://hackstore.co.il/ |

> [!CAUTION]
> LiPo packs are less forgiving than protected Li-ion packs: no built-in BMS, and a punctured
> or over-discharged pack can burn. Balance-charge only, in a LiPo bag, never unattended; store at
> storage voltage; keep the software low-battery warning and cutoff (`karmel.yaml`: 10.5 V / 9.9 V for 3S). Read
> [SAFETY.md](../SAFETY.md) rule 5. **Buy all packs in Israel** (lithium rarely ships here).
> Fire service in Israel: 102.

A spot-welded pack from a local pack builder (LaBatteria, https://www.labatteria.co.il/) is
another option; whether they build small 3S2P packs, and the cost, is **unverified**, so ask them.

## 5. Mounting parts — Buy now (at Stage 6)

| Item | Exact component / source | ≈ ₪ | Notes |
|---|---|---|---|
| Standoffs and screws | M3 nylon/brass spacers (Hackstore ₪1–4 each) + M2.5 kit | 50 (estimate) | Upper deck for LiDAR, arm base plate, Jetson |
| Arm base plate | 4–6 mm aluminium or plywood plate bolted through the chassis | 50 (estimate) | The arm's reaction torque tips a light robot: mount it low, and move the battery to counterweight it |
| Printed brackets | Camera mount, e-stop bracket, cable clips, LiDAR riser | 100–300 (service estimate) | See [3d-printing-options.md](3d-printing-options.md): print services charge from ≈ ₪30–50 for small simple parts |
| Cable management | Spiral wrap, zip ties, adhesive tie mounts | 50 (estimate) | Keep cables out of the LiDAR plane and away from wheels |
| Connectors | XT60 for each power branch, fuse per branch | 50 (estimate) | 4Project XT60 ₪5.10/₪4.80 |

## 6. 3D printer — Optional (Buy later, only if worthwhile)

Buy a printer when you expect more than a few print jobs (SO-101 parts, several brackets,
iterations). Otherwise use a service or a makerspace. **Bambu Lab A1 mini ₪990** at the
official importer 3DbotX, https://www.3dbotx.co.il/product-page/bambulab-a1-mini-3d-printer ; the
full comparison and makerspaces are in [3d-printing-options.md](3d-printing-options.md).
