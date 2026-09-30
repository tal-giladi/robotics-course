# War machine — Israeli over-the-counter BOM

> Bill of materials for adding weapons to karmel (module 20). Every row is something you can
> buy over the counter in Israel without a licence or a customs form.
>
> **Prices** are in ₪ including 18% VAT, checked **2026-09-23** unless a row says otherwise.
> "TBC" = the shop has the item in stock (product page verified) but the page is JavaScript-
> rendered and the price could not be read automatically — browse the link before ordering.
> "(estimate)" = no shop page verified; typical street price for that class of item.
>
> **Rate used for cross-checking imports:** 1 USD = ₪3.033 (2026-09-16). Orders under
> $75 USD avoid import VAT (threshold raised 2026-06-02) — relevant for AliExpress latching
> solenoids and ELRS modules, marked as such.
>
> The base robot's parts (Pi 5, Pico 2, Yahboom 520 motors, 3S pack, RPLIDAR C1, camera,
> 20 A e-stop relay ₪27.60 at 4Project, Erco e-stop button ₪67.20) are the course BOM in
> [`hardware/stage-6-final-robot.md`](../../hardware/stage-6-final-robot.md) — listed here only
> where a weapon reuses them.

## How to read this file

Each weapon system has a table. Columns: **Exact component** (what to order) · **Alt** (cheaper
or easier substitute) · **Why** · **Price** · **Buy in Israel** (verified link) · **Compat**
(what it plugs into on karmel, and the gotcha).

Payloads are split by *release method*, not by what they are — a stink bomb and a steel ball
are both "things that get dropped". The release mechanism determines which subsystem you buy:

| Release class | Mechanism | Lesson |
|---|---|---|
| **Drop** | gravity bin, servo flap, solenoid pin, latch | [21.03](../21.03-releasers-i-latches.md) |
| **Throw / launch** | servo arm, rubber-band slingshot, linear actuator cannon, CO2 air gun | [21.04](../21.04-releasers-ii-launch.md) |
| **Trigger** | manual button, 2.4 GHz RC, vision, sound | [21.05](../21.05-triggers-remote.md), [21.06](../21.06-triggers-vision.md), [21.07](../21.07-triggers-sound.md) |

---

## A. Shared platform (all weapon systems)

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| 3D-printed weapon rail: aluminium extrusion look-alike, 20×20 mm, PETG/ABS, 300 mm | 3D-printed from a FreeCAD profile | One common mounting interface for every weapon; print on your own or at a local farm | print material ~₪50–80 (estimate) | [3DbotX](https://3dbotx.co.il) (Bambu Lab printers + filament) or any local print farm | bolts to deck 1/2 with M3; keep everything out of the LiDAR plane (z = 0.12 ± 0.02 m) |
| M3 hardware kit: screws, nuts, 8 mm/12 mm standoffs, 40 pcs | M4 if you print thicker | The only fastener size the rail uses | ~₪30 (estimate) | KSP (KSP.co.il, all branches) | — |
| 12 V boost converter, e.g. XL6009 module set to 12.0 V, 3 A | LM2596 buck **only if** you accept ≤7.6 V output at empty pack (see [21.08](../21.08-weights-power-stability.md) Level 2) | **The solenoid tap.** 3S Li-ion is 9.0–12.6 V: actuators and servos run fine directly on the switched rail at 9.9–12.6 V, but a *buck* cannot output 12 V when the pack is at 9.9 V, and solenoid force scales with V². The boost tap guarantees 12.0 V for the solenoids even at empty pack | TBC, ~₪30–60 (estimate) | [Hackstore](https://hackstore.co.il) (XL6009 12 V modules in stock) | input on the switched rail (behind the e-stop relay); output feeds only the solenoid branches; 3 A |
| 5 A blade fuse + holder, per weapon 12 V branch | 2 A for mic/servo-only branches | each weapon is its own branch, its own fuse — one stuck solenoid must not brown out the others | ~₪10 (estimate) | KSP / [4Project](https://4project.co.il) | fuse before every branch, after the boost |
| XT60 + Deans cable 30 cm, 18 AWG | 16 AWG if the branch exceeds 8 A | power delivery to the weapon 12 V bus | TBC | [4Project](https://4project.co.il) (course harness already uses XT60) | — |
| JST-PH 4-pin connectors + wire, 20 pcs | JST-SH for the micro-servo signals | signal wiring for servos/solenoids/mics | ~₪20 (estimate) | [Hackstore](https://hackstore.co.il) | servo PWM on Pico 2 GPIO; mic I²S on Pico 2 |
| Zip ties + hot melt + epoxy (small) | cable ties 3 mm | mechanical retention, vibration | ~₪25 (estimate) | KSP / any hardware store | every weapon is also secured for the moment it is not firing |

**Weight and power added by subsystem A alone:** rail + hardware + converter + fuses ≈ **0.35 kg
(estimate)**, ~4 W continuous (converter quiescent + losses).

---

## B. Payloads — drop class (static, released and they fall)

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| Stink bomb, 100 ml aerosol, "stink bomb" / spray deodorant trick | deodorant stick (less messy, weaker) | classic low-energy payload: no trigger, no battery, just gets dropped or squeezed | ₪10–20 (estimate) | any supermarket / KSP (shelf item, OTC) | drop bin slot; ~0.15 kg each |
| UHU glue stick, 50 g | hot glue cartridge | sticks to whatever it hits; OTC stationery | ₪20–30 (estimate) | stationery (KSP, Express, any paper shop) | drop bin slot or squeeze-release with a servo plunger; ~0.06 kg |
| Water-filled balloon, 100–150 ml, pack of 25 | gel-filled balloon (reusable, no leak) | the original robot payload; the *releaser* is the engineering | ~₪15 per 25 (estimate) | any supermarket | drop bin slot; ~0.15 kg; the bin must be sealed against the floor |
| Paintballs, 0.25 g, 1 kg bag | airsoft BB 6 mm (see launch class) | visible, low energy, OTC sports item | TBC, ~₪40–70 per kg (estimate) | [Fox Paintball](https://www.foxpaintball.co.il) (1 kg 0.25 g bags verified in stock) | drop in a paper-cup sleeve, or the CO2 gun below |
| Neodymium magnet, 25×10 mm disc, N52 | 20×8 mm | "sticky mine": drop it and it holds to any steel target | TBC | [wango-caravans.com](https://wango-caravans.com) (magnet supplier, verified) or KSP | drop bin slot; ~0.04 kg each; keep away from the LiDAR (ferrous? no — but a dropped one on the deck is a phantom obstacle) |
| Confetti / glitter pouch: 3D-printed zippered pouch + zip tie, 30 g | biodegradable rice, dyed | area denial / visual; the pouch *is* the payload, the zip tie is the release | print ~₪5 + zip ties ₪2 | print locally; dyed rice at any grocery store | drop bin slot; ~0.05 kg |
| Cold-smoke stick, 8 s burn, white | sparkler (סמרטוט) as a smoke substitute | visible cloud, OTC holiday item | ₪5–15 per stick (estimate, **in season**; verify at [Fox Paintball](https://www.foxpaintball.co.il) or a gas station) | gas stations / toy + fireworks shops during holidays (OTC, no licence) | **its own slot with a flame shield** — it burns; ~0.03 kg |
| Sparkler, 20–30 cm, 2–4 s | — | light + hiss + tiny sparks; the cheapest "flashbang" | ₪3–10 (estimate, in season) | same as above | own slot, flame shield, treat like a candle; ~0.01 kg |
| Bouncy ball, 4 cm | steel ball (below) | kinetic drop with a bounce; kid-safe | ~₪5 (estimate) | Zap / any toy store | drop bin slot; ~0.05 kg |

**Drop-class payload mass budget:** 3-slot bin loaded ≈ **0.4–0.6 kg of payload (estimate)**
on top of the ~0.25 kg bin + releaser (subsystem D).

---

## C. Payloads — launch class (kinetic, given energy by the weapon)

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| Rubber bands, heavy "tractor" 13 mm × 200 mm, pack | 2 bands in parallel (2× the force, 2× the stretch needed) | the slingshot's energy storage; OTC stationery | ~₪15 per pack (estimate) | KSP / stationery | slingshot frame (subsystem D); energy ≈ 2–8 J with 2 bands at 120 mm stretch |
| Steel ball bearing, 5 mm, chrome, bag of 100 | 6 mm bouncy ball (safer at range) | slingshot / cannon projectile; dense and cheap | ~₪20 per bag (estimate) | KSP / any hardware store (bearing shelf) | slingshot or cannon; 0.004 kg each; **eye protection** |
| Bouncy ball 4 cm, mixed, bag | — | cannon projectile with low muzzle velocity and a bounce | ~₪15 per bag (estimate) | Zap / toy store | cannon; 0.05 kg; the cannon's *soft* round |
| CYMA CO2 blowback BB magazine, 6 mm airsoft | full CYMA airsoft pistol (bigger, ₪150–300 estimate) | a *complete* OTC air gun: CO2 8 g power, ~25 BB mag, 100+ fps class — the highest-energy weapon you can buy without a licence | ₪180 (verified at [Fantom](https://fantompepper.co.il)) | [Fantom / fantompepper.co.il](https://fantompepper.co.il) (CYMA range verified) | trigger pulled by a servo (subsystem D) or a 12 V linear actuator; the whole magazine is the payload rack |
| Airsoft BBs, 6 mm, 0.20–0.25 g, 10 kg | 0.12 g (weaker, safer indoors) | ammunition for the CYMA; 10 kg ≈ 15,000 rounds | TBC (10 kg bags verified in stock at [Fantom](https://fantompepper.co.il)) | [Fantom / fantompepper.co.il](https://fantompepper.co.il) | — |
| CO2 cartridge, 12 g paintball | 8 g (smaller, cheaper, more frequent swap) | powers the CYMA and any CO2 gun; OTC at paintball shops | TBC, ~₪8–15 each (estimate) | [Fox Paintball](https://www.foxpaintball.co.il) (12 g CGS inserts verified) | screw in a 12 g cartridge; **the cartridge is the only part of the whole BOM under real gas pressure** |
| CO2 cartridge, 95 g disposable | 12 g × 8 (cheaper total, heavier rack) | for larger CO2 guns or long demo days; one cartridge = ~8× the shots | TBC, ~₪40–60 (estimate) | [Perfectreef](https://perfectreef.konimbo.co.il) (aquarium shop, 95 g disposables verified) | needs a regulator for most guns — the CYMA magazine takes 8/12 g only |
| Neodymium magnet 50×15 mm (heavy mine) | — | for the AGF heavy-throw: drops and *sticks*, 0.15 kg of payload with a 0.3 m arm | TBC | [wango-caravans.com](https://wango-caravans.com) | heavy-throw arm only; the recoil on a 2.66 kg robot is measurable |

**Launch-class energy budget (the numbers [21.08](../21.08-weights-power-stability.md) computes):**

| Launcher | Stored/shot energy | Peak current (12 V) | Duty |
|---|---|---|---|
| Slingshot, 2 bands, 120 mm | ~4 J | ~0.3 A (servo, 0.2 s) | one shot per 3 s |
| Cannon, 120 N actuator, 100 mm | ~12 J | ~10–15 A (estimate) | one shot per 5 s |
| CYMA CO2 magazine | ~3–5 J per BB (CO2, not electrical) | ~1–2 A (servo trigger) | one shot per 2 s |
| AGF heavy throw, 0.15 kg at 0.3 m | ~4 J | ~2 A (78 kg-class servo) | one shot per 10 s |

---

## D. Releasers — the mechanisms that let go (or let fly)

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| MG996R 180° digital servo, with feedback | SG90 micro (0.15 N·m — fine for a light flap, too weak for a latch) | the default latch/flap actuator: 10 kg·cm, PWM on a Pico GPIO | TBC, ~₪80–130 (estimate) | [Hackstore](https://hackstore.co.il) (in stock, page verified) | 5 V rail, PWM on Pico 2; **behind the e-stop relay** (it moves things) |
| MG996R 360° continuous-rotation | the 180° version with a detent | the throw arm and the slingshot's cocking servo | TBC, ~₪80–130 (estimate) | [Hackstore](https://hackstore.co.il) (in stock, page verified) | same as above; the 360° version has no hard end-stops — the software must stop it |
| Linear solenoid, 35 mm stroke class, 12 V, 42 N | the 85 N version (same module family, more force, ~2× the current) | the pin-push release: pushes a 2 mm steel pin out of the payload's hole, gravity takes over | TBC, ~₪80–150 (estimate) | [Hackstore](https://hackstore.co.il) (42 N and 85 N 35 mm 12 V verified in stock) | 12 V weapon bus, 5 A fuse; pulse it 100–300 ms (a held solenoid is a heater: 42 N at 12 V ≈ 3–5 W continuous) |
| Latching solenoid, 12 V, 20 N, import | two 85 N solenoids in push-pull | one pulse holds, one pulse releases — zero holding current, ideal for a latch you want to hold for minutes | ~₪40–60 (estimate, **import** under $75 → no VAT) | AliExpress (unverified IL shop; the 85 N Hackstore pair is the OTC substitute) | 12 V weapon bus; the import item — the only row in this BOM that is not a walk-in purchase |
| Steel pin 2 mm × 40 mm + torsion spring, 2 pcs | a bent paper clip (bench test) | the pin-and-spring drop: zero electronics, fails open when the spring is wound | ~₪15 (estimate) | KSP / hardware store | the pin goes through the payload's printed sleeve; the spring holds it, the solenoid or a servo tab withdraws it |
| 3D-printed gravity drop bin, 3 slots, servo flap | a cardboard box for the bench test | the passive releaser: payload falls when the flap opens; the MG996R holds the flap shut | print ₪5–10 | print locally | on the front of the rail, payload exits over the front caster — **watch the LiDAR plane** |
| 3D-printed slingshot frame, Y-frame 150 mm | balsa Y (cheaper, less repeatable) | holds the two rubber bands; the servo cocks a hook that releases the pouch | print ₪5–10 | print locally | the band's recoil is ~40 N for 0.1 s (estimate) — bolt it, don't glue it |
| 3D-printed throw arm, 250 mm, with counterweight pocket | a wooden dowel arm | the MG996R-360 swings it; the counterweight keeps the empty arm light enough to swing at 20°/ms class speeds | print ₪5–10 | print locally | MG996R-360 on the rail; arm tip speed ~2–3 m/s at 250 mm radius (estimate) |
| Linear actuator, 12 V, 120 N, 100 mm stroke, 28 V motor class | 150 N / 150 mm (longer throw, same bus) | the cannon: a 0.05 kg bouncy ball leaves at ~7 m/s (estimate, computed in [21.04](../21.04-releasers-ii-launch.md)) | TBC, ~₪150–400 (estimate) | [Hackstore](https://hackstore.co.il) (120 N/100 mm, 150 N/150 mm, 150 N/200 mm, 600 N/120 mm all verified in stock) | 12 V weapon bus, **its own 5 A fuse + 18 AWG branch** (10–15 A peak); the heaviest single weapon part (~0.8 kg) |
| AGF 1/5 scale 78 kg·cm servo (heavy RC servo) | 2× MG996R-180 in a geared pair (weaker, cheaper) | the heavy-throw: enough torque to swing a 0.3 m arm with a 0.15 kg payload | ₪1,247 (verified at [Ron Maor](https://ronmaor.co.il)) | [Ron Maor (ronmaor.co.il)](https://ronmaor.co.il) | 6–8 V RC supply — run it from the 12 V bus through its own 2 A fuse (it is a 1/5 RC servo, 6–8 V nominal; 12 V is the *fast* setting, measure its draw); the premium row, only for the "siege" package |

**Releaser selection rule** (the full argument is in [21.03](../21.03-releasers-i-latches.md)):

| Payload mass | Drop it? | Launch it? | Releaser |
|---|---|---|---|
| < 0.1 kg (BB, glue stick, confetti) | yes | yes | gravity bin / pin+spring |
| 0.1–0.25 kg (stink bomb, balloon, magnet) | yes | no | servo flap or solenoid pin |
| 0.25–0.6 kg (bouncy ball, heavy magnet) | marginal | yes | cannon or slingshot |
| > 0.6 kg | no | yes, heavy | AGF arm |

---

## E. Triggers — how the weapons know to fire

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| USB membrane keypad, 4×4 | 4× push buttons + 4 kΩ pull-downs on Pico 2 GPIO | the manual trigger: one key per weapon, zero wireless | TBC, ~₪30–50 (estimate) | [Hackstore](https://hackstore.co.il) (keypads in stock) | USB on the Pi (compute domain — the manual trigger works even when the e-stop is in, by design) |
| STX2 2-channel 2.4 GHz FHSS transmitter + SRX200 receiver | ELRS 2.4 GHz TX + RX modules (import) | the off-board trigger: a person with a hand-held unit stands 50 m away; 2 channels = 2 weapons | TBC, ~₪150–300 (estimate) | [Maof RC / טיסן הקריות (maof-rc.co.il)](https://www.maof-rc.co.il/store/stx2-2-channel-24ghz-fhss-with-srx200) (verified in stock) | RX signal to Pico 2 GPIO (compute domain); the *switch* it closes is on the weapon 12 V bus (actuator domain) |
| INMP441 I²S MEMS microphone module | USB condenser mic on the Pi | the sound trigger: clap/whistle detection, 3.3 V, I²S on the Pico 2 | TBC, ~₪30–50 (estimate) | [Hackstore](https://hackstore.co.il) (page verified) | I²S pins on Pico 2; 2 A-class branch (mA draw, 5 V rail — compute domain: the mic is a sensor, it cannot hurt anyone, same argument as the LiDAR in [20.02](../../20-final-robot/20.02-hardware-integration.md)) |
| Piezo disc element, 30×5.5 mm + buzzer 20-pack | the INMP441 (better, more expensive) | the cheap sound sensor: a 10 kΩ + the disc as a voltage divider, one ADC pin | TBC (20-pack buzzers and 30 mm discs both verified in stock) | [Hauzattahel](https://www.hauzattahel.co.il/Bargain_29322_Global.htm) (20× piezo buzzers), [DreamDigital](https://dreamdigital.co.il/boutique_18955-Store.html) (30×5.5 mm discs) | one Pico 2 ADC pin; the buzzer pack is also the *sounder* for the "arm warning" |
| IR LED 940 nm + phototransistor pair, 2 pcs | a TV remote + the phototransistor (bench test) | the coded trigger: a hand-held IR beacon transmits a pattern; the robot fires on the pattern, not on any light | ~₪15 (estimate) | [Hackstore](https://hackstore.co.il) / KSP | LED on a hand-held battery unit, phototransistor on Pico 2 ADC; 940 nm is invisible to the camera |
| ArUco markers, A4 prints, any of the 6×6 dictionary set | coloured tape crosses (worse) | the vision trigger target: print, stick on the wall, the Pi detects it and fires | print free | any print shop / home printer | no hardware — the camera is already on karmel; see [21.06](../21.06-triggers-vision.md) |

**Trigger domain rule:** the *sensor* (mic, keypad, RX, camera, phototransistor) lives on the
always-on compute domain — it cannot hurt anyone. The *switch it closes* (solenoid, servo,
actuator) lives on the switched actuator domain behind the e-stop relay. A trigger that fires a
weapon while the e-stop is in is a bug, and [21.09](../21.09-failure-modes-and-safety.md) has the
interlock test for it.

---

## F. Power — the weapon rail

| Exact component | Alt | Why | Price | Buy in Israel | Compat |
|---|---|---|---|---|---|
| 20 A relay, DPST, 5 V coil | the course's existing relay (reuse it and re-size) | the e-stop boundary for the whole weapon bus — the course's 20 A relay is sized for the drive motors; the weapons add ~15 A peak, so the *cannon branch* gets its own relay or the main relay goes to 30 A | ₪27.60 (verified, course BOM) | [4Project](https://4project.co.il) | behind the Erco e-stop NC contact, exactly as in [20.02](../../20-final-robot/20.02-hardware-integration.md) |
| 30 A relay, DPST, 5 V coil (whole weapon bus) | keep the 20 A and limit the cannon's current in software (weaker launch) | a 120 N actuator peaks at ~5–6 A; add the CYMA trigger servo, a second solenoid and the latches in the same instant and the weapon bus wants ~12–15 A — the course's 20 A relay already carries the drive motors' 8.85 A stall, so the weapons get their own relay with real margin | TBC, ~₪40–80 (estimate) | [4Project](https://4project.co.il) / [Hackstore](https://hackstore.co.il) | its own e-stop NC feed, in series with the whole weapon bus |
| XT60 pigtail, 30 cm, 16 AWG | — | the bus tap | TBC | [4Project](https://4project.co.il) | — |
| Fuse, 5 A blade × 4, 2 A blade × 2, 10 A blade × 1 | — | per-branch protection | ~₪20 (estimate) | KSP / [4Project](https://4project.co.il) | every weapon branch |

**The weapon rail, end to end** (drawn in [21.01](../21.01-war-machine-architecture.md)):

```
3S pack ─[10 A]─[main switch]─┬─▶ 5 V regulator ─▶ Pi 5, Pico 2, sensors, keypad, RC RX   (compute)
                              │
                              ├─[e-stop NC]─▶ relay 1 (20 A, course) ─▶ drive motors + arm bus
                              │
                              └─[e-stop NC]─▶ relay 2 (30 A, weapons) ─┬─[5 A]─▶ cannon (120 N actuator)
                                                                       ├─[2 A]─▶ CYMA trigger servo
                                                                       ├─[2 A]─▶ MG996R latches + slingshot
                                                                       └─▶ 12 V boost (XL6009, 12.0 V)
                                                                              ├─[2 A]─▶ solenoid 85 N
                                                                              └─[2 A]─▶ solenoid 42 N
```

---

## G. Total build packages

| Package | Contents | Added mass (estimate) | Added peak power | Total robot mass |
|---|---|---|---|---|
| **Scout** | rail, boost, drop bin + 2 MG996R, slingshot, keypad, INMP441, 3-slot payload rack | 1.50 kg | 5.3 A weapon-bus peak | 4.16 kg |
| **Assault** | Scout + cannon (120 N actuator) + CYMA CO2 magazine + 2.4 GHz RC + 30 A cannon relay | 2.85 kg | 13.3 A weapon-bus peak | 5.51 kg |
| **Siege** | Assault + AGF heavy-throw arm + 95 g CO2 + latching solenoids + IR beacon | 3.80 kg | 15.8 A weapon-bus peak | 6.46 kg |

The mass column is what [21.08](../21.08-weights-power-stability.md) feeds into the CG and
tipping model — every entry there is (estimate) until you weigh your own build.
