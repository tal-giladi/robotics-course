# Workbench tools

Catalog id: `tools`. Back to [HARDWARE.md](../HARDWARE.md).

> [!IMPORTANT]
> **Prices checked 2026-09-16, approximate, including VAT.** 1 USD = ₪3.033. "Estimate" means
> the item wasn't priced in an Israeli shop during research; any hardware store (for example a
> local "כלי עבודה" shop) sells it. Evidence: `references/research/israel-hardware-stage1-2026-09.md`;
> calipers, fume extractor and crimper prices were checked on 2026-09-16.

## What the tools unlock

[01.03 Workbench and tools](../01-first-robot/01.03-workbench-and-tools.md),
[01.06 Mechanical assembly](../01-first-robot/01.06-mechanical-assembly.md),
[01.07 Power system](../01-first-robot/01.07-power-system.md),
[02.07 Wiring harness](../02-robot-electronics/02.07-wiring-harness.md),
[02.08 Breadboard to perfboard](../02-robot-electronics/02.08-from-breadboard-to-perfboard.md),
[02.09 Electrical debugging](../02-robot-electronics/02.09-electrical-debugging-method.md),
[14.02 Assembling the arm](../14-robotic-arm/14.02-buying-and-assembling-the-arm.md), and the foundations
[FE.03 Multimeter](../optional-foundations/electronics/FE.03-multimeter.md),
[FE.04 Breadboards and connectors](../optional-foundations/electronics/FE.04-breadboards-and-connectors.md),
[FE.17 Soldering](../optional-foundations/electronics/FE.17-soldering.md).

## Totals

| Path | ≈ ₪ | ≈ USD |
|---|---|---|
| **Recommended, buy now** | **₪868** | **$286** |
| **Cheapest, buy now** (DT-9205M multimeter, no helping hands) | **₪638** | **$210** |
| Buy later (crimper, calipers, fume extractor) | ₪375 | $124 |

## Tools BOM

| Tool | Label | Recommended (where) | ≈ ₪ | Alternative | Why / notes |
|---|---|---|---|---|---|
| Multimeter | **Buy now** | **UNI-T UT33A+**, Hackstore https://hackstore.co.il/product/%D7%A8%D7%91-%D7%9E%D7%95%D7%93%D7%93-%D7%93%D7%99%D7%92%D7%99%D7%98%D7%90%D7%9C%D7%99-ut33a/ | 250 | DT-9205M at Hackstore ₪105 (cheapest path). 4Project digital multimeter ₪116.37 (sale). Avoid KSP's ₪38 Omega (toy-grade). | The most important tool: pack voltage, regulator output **before** connecting the Pi, continuity, polarity ([SAFETY.md](../SAFETY.md) rule 9) |
| Soldering iron | **Buy now** | **908S 80 W adjustable pencil iron, 230 V**, Hackstore https://hackstore.co.il/ (search listing ₪120 in stock; the product URL returned 404, so ask) | 120 | **Pinecil V2** $25.99, https://pine64.com/product/pinecil-smart-mini-portable-soldering-iron/ (needs a USB-C PD ≥ 45 W charger, not priced; shipping to Israel not checked). 4Project Bakon SBK936B 65 W station ₪399.90. **Avoid** 30 W unregulated irons (KSP Bolt ₪45, 4Project ₪43.40): too weak for XT60 and 18 AWG. | Headers, XT60, BMS, power wiring |
| Solder | **Buy now** | 63/37 0.8 mm 100 g, Hackstore | 65 | 4Project leaded 0.8 mm 100 g ₪156.90. Lead-free (SAC305) is kinder to health but harder for beginners (price not checked). | With leaded solder: wash hands, no food at the bench (SAFETY rule 10) |
| Flux pen or paste | **Buy now** | Any electronics shop (estimate) | 30 | | Clean joints on XT60 and thick wire |
| Tip cleaner (brass wool) | **Buy now** | Hackstore | 15 | Wet sponge | |
| Wire strippers | **Buy now** | Self-adjusting stripper, any hardware store (estimate; not found at KSP, Hackstore or 4Project) | 60 | Fixed-gauge stripper | 18–26 AWG |
| Flush cutters | **Buy now** | Any hardware or electronics shop (estimate) | 30 | | Leads, zip ties. Wear glasses: clipped leads fly. |
| Heat shrink assortment | **Buy now** | Hackstore 1.5 / 3 / 5 mm (₪6–14 per size) | 24 | | Insulate every power joint; cover exposed battery leads |
| Helping hands | **Buy now** (skip on cheapest path) | 2-arm workstation with iron stand, Hackstore | 85 | Hackstore 2-arm clamp ₪45 (out of stock 2026-09-16) | Holds XT60 and wires while soldering |
| Precision screwdrivers | **Buy now** | Arctic Precision Screwdriver Toolkit, KSP https://ksp.co.il/web/item/398521 | 59 | Bolt SE101 10-pc ₪89 (KSP) | |
| Hex keys (metric 1.5–5 mm) | **Buy now** | Any hardware store (estimate) | 40 | | Hub set screws, M3 socket screws |
| Hot glue gun | **Buy now** | Hunter 78 W, KSP https://ksp.co.il/web/item/207722 | 35 | | Strain relief, temporary sensor mounts |
| LiPo / Li-ion safety bag | **Buy now** | C-Hobby 20×10 cm, https://c-hobby.co.il/en/product/lipo-safe-bag/ (price from page data; may be list price) | 35 | Skyline or Amazon bags (not verified) | Charge the pack inside it, on a non-flammable surface ([SAFETY.md](../SAFETY.md) rule 5) |
| Safety glasses | **Buy now** | Any hardware store (estimate) | 20 | | Soldering, clipping, drilling |
| Drill + 3 / 3.2 mm bits | **Buy now** (borrow if you can) | Household cordless drill (not priced) | — | A makerspace ([3d-printing-options.md](3d-printing-options.md)) | Drilling the chassis plate for 37 mm motor brackets (01.06). Not included in the totals. |
| **Crimper for JST-PH / XH and Dupont** | **Buy later** (before [02.07](../02-robot-electronics/02.07-wiring-harness.md)) | **IWISS SN-2549 ratcheting crimper** (AWG 28–18; JST ZH/PH/XH/VH, Dupont, Molex), $20.99 on 2026-09-16, https://www.icrimptools.com/products/iwiss-sn-2549-ratcheting-wire-crimping-tools-for-jst-zh-1-5mm-ph-2-0mm-xh-2-5mm-vh-3-96mm-jwps-4-0mm-pitch-dupont-2-54mm-pitch-open-barrel-terminals-awg28-18-0-08-1-0mm | ≈ 70 with shipping (estimate) | SN-28B-type crimpers on AliExpress/Amazon. **No crimper was found at Hackstore** (searched 2026-09-16); ask 4Project. Pre-crimped JST cables: Hackstore JST SM leads ₪6–12. | Stage 1 works with pre-made jumpers and screw terminals. Crimping makes a harness that survives vibration. |
| Digital calipers 150 mm | **Buy later** (Optional; useful from 01.06) | Hackstore, in stock 2026-09-16, https://hackstore.co.il/product/%D7%A7%D7%9C%D7%99%D7%91%D7%A8-%D7%93%D7%99%D7%92%D7%99%D7%98%D7%9C%D7%99-150-%D7%9E%D7%9E/ | 110 | A steel ruler | Wheel diameter for odometry calibration ([09.05](../09-odometry/09.05-calibrating-odometry.md)), caster height, 3D-print fits |
| Fume extractor | **Buy later** (Optional) | Desktop fume extractor model 493, Hackstore, in stock 2026-09-16, https://hackstore.co.il/product/%d7%a9%d7%95%d7%90%d7%91-%d7%a2%d7%a9%d7%9f-%d7%a9%d7%95%d7%9c%d7%97%d7%a0%d7%99-%d7%93%d7%92%d7%9d-493/ (replacement carbon filter ₪20) | 195 | Until then: an open window and a small fan blowing fumes **away** from your face | SAFETY rule 10 |

## What can wait

- **Crimper**: until the wiring-harness lesson 02.07. Use Dupont jumpers and screw terminals before that.
- **Calipers**: a ruler is enough to start; buy them before odometry calibration or 3D design work.
- **Fume extractor**: ventilate instead for the first few soldering sessions; buy it if you solder a lot.
- **A bench power supply** isn't needed at all in this course: the 27 W Pi PSU and the robot pack
  cover everything. Never charge lithium from a bench supply.
