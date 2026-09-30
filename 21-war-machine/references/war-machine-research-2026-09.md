# War machine — shop verification log (September 2026)

> Dated evidence behind [../hardware/war-machine-bom.md](../hardware/war-machine-bom.md) and the
> "Go deeper" links of module 21. Course convention (see
> [MAINTAINING.md](../../MAINTAINING.md)): prices and stock are *verified* facts with a date;
> anything without a date is an estimate. Re-verify before a purchase that matters — Israeli
> RC and airsoft prices move with the exchange rate and with the season (the cold-smoke and
> sparkler rows are **in-season** items).

**Verified 2026-09-23** (BOM price check) and **2026-09-24** (stock re-check, lesson links).

**Rate used for cross-checking imports:** 1 USD = ₪3.033 (2026-09-16). Orders under $75 USD
avoid import VAT (threshold raised 2026-06-02) — this is what makes the AliExpress latching
solenoid and the ELRS modules viable, and it is why the BOM marks them "import".

## Shops (the OTC set)

| Shop | What it covers in this module | Verified 2026-09-24 | Notes |
|---|---|---|---|
| [4Project](https://4project.co.il) (Yehud) | The Pololu distributor: the 20 A e-stop relay (₪27.60, course BOM), fuses, XT60/Deans, the 30 A relay class | stock + price | the course's own course BOM shop; the weapon branch hardware (fuses, XT60) lives here |
| [Hackstore](https://hackstore.co.il) (Rehovot) | Servos (MG996R 180°/360°), solenoids (42 N and 85 N, 35 mm, 12 V), linear actuators (120 N/100 mm, 150 N/150 mm, 150 N/200 mm, 600 N/120 mm), keypads, the INMP441, JST-PH wiring, the XL6009 12 V modules | product pages verified | **JS-rendered prices** — the page confirms the item is in stock but the ₪ price could not be read automatically (the BOM's "TBC" rows); browse the link before ordering |
| [Piitel](https://piitel.co.il) | the official Raspberry Pi reseller: the Pi 5 (the course's compute) | course BOM | the war machine adds no new Pi-class parts; the Pi runs the vision trigger (21.06) |
| [Sollan](https://www.sollan.co.il) | general electronics: the INA219/INA226 current monitors (21.08-E3), capacitors (the 470 µF bulk), wire | course BOM | the measurement hardware of the sag campaign |
| [KSP](https://www.ksp.co.il) (all branches) | M3 hardware kits, blade fuses + holders, steel ball bearings, rubber bands, stationery (glue sticks, paintballs shelf), zip ties | shelf items, OTC | the "any hardware store" rows; the bearing shelf is where the 5 mm chrome balls come from |
| [Erco](https://www.erco.co.il) | the e-stop button (₪67.20, course BOM) | course BOM | the weapon rail reuses the course's e-stop (21.01 domain v3) |
| [Maof RC / טיסן הקריות](https://www.maof-rc.co.il) | the STX2 2-channel 2.4 GHz FHSS transmitter + SRX200 receiver (21.05) | [product page](https://www.maof-rc.co.il/store/stx2-2-channel-24ghz-fhss-with-srx200) verified in stock | the off-board trigger; 2 channels = 2 weapons; the ELRS import is the alt (the BOM marks it import) |
| [Fox Paintball](https://www.foxpaintball.co.il) | paintballs (0.25 g, 1 kg bags), the 12 g CGS CO2 inserts, cold-smoke sticks (in season) | verified in stock | the paintball shop — the 12 g cartridge is the only part of the BOM under real gas pressure (21.09 Level 3) |
| [Fantom / fantompepper.co.il](https://fantompepper.co.il) | the CYMA CO2 blowback BB magazine (₪180, verified), the 6 mm airsoft BBs (10 kg bags) | verified in stock | the highest-energy OTC weapon in the BOM (21.04 Level 5); the 10 kg BB bags are the "15,000 rounds" row |
| [Perfectreef](https://perfectreef.konimbo.co.il) | the 95 g disposable CO2 cartridges (the aquarium shop) | verified in stock | the Siege-grade CO2 (21.02); a regulator is needed for most guns — the CYMA takes 8/12 g only |
| [Ron Maor](https://ronmaor.co.il) | the AGF 1/5 scale 78 kg·cm servo (₪1,247, verified) | verified | the premium row, only for the "siege" package (21.04 Level 4 — the lobber) |
| [Hauzattahel](https://www.hauzattahel.co.il) | the 20× piezo buzzers ([product page](https://www.hauzattahel.co.il/Bargain_29322_Global.htm)) | verified in stock | the cheap sound sensor (21.07) and the arm-warning sounder |
| [DreamDigital](https://dreamdigital.co.il) | the 30×5.5 mm piezo disc elements ([store page](https://dreamdigital.co.il/boutique_18955-Store.html)) | verified in stock | the disc + the 10 kΩ divider is the 21.07 Level 2 circuit |
| [3DbotX](https://3dbotx.co.il) | the weapon rail print (Bambu Lab printers + filament) | course BOM | the rail is printed, not bought (the FreeCAD profile is the course's) |
| [Zap](https://www.zap.co.il) | the bouncy balls (4 cm), the toy-store rows | shelf items, OTC | the cannon's soft round (21.04 Level 3) |
| [wango-caravans.com](https://wango-caravans.com) | the neodymium magnets (25×10 mm and 50×15 mm discs, N52) | verified | the "sticky mine" payload (21.02); keep the deck magnet-free (21.09 row 13) |

## Cross-border (the import rows — under $75 → no VAT, 2026-06-02 threshold)

| Item | Where | Verified | Notes |
|---|---|---|---|
| Latching solenoid, 12 V, 20 N | AliExpress | not verified (the class is standard) | the only BOM row that is not a walk-in purchase; the 85 N Hackstore pair is the OTC substitute (push-pull, holding current is the price) |
| ELRS 2.4 GHz TX + RX | AliExpress / the IL ELRS community | not verified | the STX2/SRX200 at Maof RC is the OTC substitute (21.05) |

## What "TBC" means (the BOM's price column)

A "TBC" row is a **product page that was verified in stock on the date above** whose price is
rendered by JavaScript and could not be read automatically. The item is there; the ₪ is not
in the log. Browse the link before ordering, and update this file with the price and a new
date when you check (that is the MAINTAINING.md convention — the date moves, the fact does
not disappear).

## External references used in the lessons (the non-shop links)

| URL | Used in | What it is |
|---|---|---|
| [ISO 13849-1 overview](https://www.iso.org/standard/73934.html) | 21.01, 21.09, 21.10 | the safety-related control standard — the e-stop-plus-relay argument in industrial vocabulary |
| [Pololu distributor list](https://www.pololu.com/distributors/0J118) | 21.01 | where the Pololu parts ship from; 4Project is the Israeli distributor |
| [OpenCV ArUco documentation (4.10.0)](https://docs.opencv.org/4.10.0/d4/d17/namespacecv_1_1aruco.html) | 21.06 | the `detectMarkers` API; Version-sensitive (the module layout moved between OpenCV 4.5 and 4.8; the `4.x` path 404s — this version-pinned one is live, checked 2026-09-24) |
| [García-Hidalgo et al., ArUco 2017](https://arxiv.org/abs/1701.04191) | 21.06 | the marker design and the detection/pose pipeline |
| [The YOLO project](https://github.com/ultralytics/ultralytics) | 21.06 | the current small detectors; Version-sensitive (the model names and the Pi 5 runtime move) |
| [Battery University — BU-802a, rising internal resistance](https://www.batteryuniversity.com/article/bu-802a-how-does-rising-internal-resistance-affect-performance/) | 21.08 | why the R_int is a measured parameter that ages (BU-703 was renumbered; checked 2026-09-24) |
| [Battery University — BU-902, how to measure internal resistance](https://www.batteryuniversity.com/article/bu-902-how-to-measure-internal-resistance/) | 21.08 | the 21.08-E3 measurement method |
| [FEMA's FMEA method](https://www.fema.gov/evacuation-and-sheltering/fema-failure-mode-and-effects-analysis-fmea) | 21.09 | the method's origin and the RPN's limits |
| [Wikipedia — Pulse-position modulation](https://en.wikipedia.org/wiki/Pulse-position_modulation) | 21.05 | the PPM frame the RC RX decodes |
| [Wikipedia — Schmitt trigger](https://en.wikipedia.org/wiki/Schmitt_trigger) | 21.07 | the hysteresis the sound gate uses (the 15/8 dB pair) |

## Corrections logged against the BOM (the model's numbers, not the store's)

The BOM's energy column is the *store's* stored energy; the model (21.04,
`code/projectile_range.py`) computes the *projectile's* energy. The corrections, documented
in 21.04:

| BOM row | BOM's number | The model's number | The lesson |
|---|---|---|---|
| Slingshot, 2 bands, 120 mm | ~4 J | 0.86 J stored (k ≈ 120 N/m, the 13 mm bands' measured class) → 0.60 J to the projectile at 70% | 21.04 Level 2 |
| Cannon, 120 N × 100 mm | ~12 J | 12.0 J work → 6.0 J to the projectile at 50% | 21.04 Level 3 |
| AGF heavy throw | ~4 J | 0.40 J at the real slew (the tip speed, not the torque, sets the range — the lobber) | 21.04 Level 4 |

The BOM's "0.05 kg bouncy ball leaves at ~7 m/s" (the cannon row) is conservative; the model
says 15.5 m/s, and 21.10's range clearing (25 m) is set by the model ×2, not by the 7 m/s.
