# Module 21 · War Machine — build progress log

> **Why this file exists:** the session crashes sometimes and we restart from scratch.
> This file is the single source of truth for *what has been done, what is in flight, and what
> is next*. Read it first after every restart, update it before every pause.
>
> **Task (original instructions):** convert the course's ready robot (karmel, module 20) into a
> **war machine robot** — add add-ons, payloads and weapons it can throw, blow, explode, spray,
> stick, etc. All materials must be **over-the-counter in Israel** (with shop links). Focus on the
> **technical side** of the weapons (launch/release/trigger mechanics), not the propellants
> themselves. Include multiple **triggers** (remote control, vision detection, sound detection),
> multiple **releasers** matched to different payload classes, and account for **weights** and
> **things that can go wrong**. Any spec that differs from the course robot is documented **here**,
> never in the original course files.

## Ground rules (do not violate on restart)

1. **Do not touch existing files.** Everything new lives in `21-war-machine/`.
   (New files only; my files may be updated freely.)
2. **Format must match the existing course** (see `AUTHORING.md`): each lesson follows the exact
   heading order — What you will learn / Why it matters / Prerequisites / Concept / Technical
   explanation / Diagram / Code / Exercise / Expected result / Troubleshooting / Common mistakes /
   Knowledge check / Practical challenge / You can skip this if… / Go deeper / Progress checkpoint.
   The module is published on GitHub, so relative links must resolve.
3. **Israeli OTC parts only**, with links to verified Israeli shops. Use the shop list in
   `hardware/suppliers-israel.md` (Piitel, 4Project, Hackstore, Sollan, Erco, RC shops: Skyline,
   RCZone, C-Hobby, Dominator, 3DbotX, KSP, Zap). Prices are approximate, dated, with 18% VAT.
4. **Robot reference (karmel, from `labs/config/karmel.yaml`):** 1.6 kg bare / ≈2.66 kg as built,
   3S Li-ion 10.8 V nominal (9.0–12.6 V), two Yahboom 520 motors, ball caster **in front** of the
   axle (CG must sit ahead of the axle!), LiDAR plane z = 0.12 m, camera FOV 1.20 rad,
   Pi 5 + Pico 2, two power domains (compute always-on / actuators behind e-stop relay).
   Max speed 0.5 m/s (software), demo 0.3 m/s.
5. **Lesson length:** 1,500–3,500 words + code, one sitting.

## Lesson plan (module 21, 10 lessons)

| Id | Title | Status | Notes |
|---|---|---|---|
| 21.01 | War machine architecture — what changes on karmel | ✔ verified | fixed 2026-09-24: added the `## Prerequisites` section + markers, 2 external URLs in Go deeper (ISO + Pololu distributors), E2 path/check corrected to `code/exercises` + `-k 2101_e2` |
| 21.02 | Payloads — the BOM of things you can throw, shoot, stick, spray | ✔ verified | payload taxonomy, impact-energy table, state machine (LOADED = verified fact), rack layout |
| 21.03 | Releasers I — latches, clamps and pins (static payloads) | ✔ verified | flap torque (2.77× margin), solenoid V²/pulse/flyback, pin+spring fail-open, 5 V tap (spec-delta row 11) |
| 21.04 | Releasers II — launch mechanisms (kinetic payloads) | ✔ verified | launch chain, slingshot k=120 N/m (0.60 J), cannon 6.0 J / 15.5 m/s, AGF lobber verdict (0.40 J, 0.45 m), CO2 51 m |
| 21.05 | Triggers I — remote control | ✔ verified | written by subagent; PPM decode + dead-man, e-stop interlock; RC RX on GPIO 16 → moves to 20 when 21.07's mic lands (21.07's pin plan); IR phototransistor on GPIO 21 |
| 21.06 | Triggers II — vision detection | ✔ verified | ArUco path (distance = f·s/p), YOLO path, 15-frame dwell / conf 0.5 / 1–5 m / 2 s cooldown gate, false-fire p_f^N |
| 21.07 | Triggers III — sound detection | ✔ verified | written by subagent; INMP441 I²S on 16/17/18, double-clap 250–900 ms, 15/8 dB hysteresis |
| 21.08 | Weights, power and stability of the war load | ✔ verified | CG with extended arm, Thevenin sag (`sag` CLI), sag×servo interaction, runtime tax |
| 21.09 | What can go wrong — failure modes and safety | ✔ verified | 23-row FMEA (top-3 RPNs), big three (double-fire/dry-fire/jam), CO2 rolling-cartridge, range clearings 15/25/50 m, 10-test e-stop matrix |
| 21.10 | Integration, test range and the war demo | ✔ verified | test matrix (72-shot smoke), range layout, 12-step runbook with GO/NO-GO, CSV shot log, evidence package |

Status legend: ⬜ not started · 🚧 in progress · ✅ done (file written) · ✔ verified (format + links + code, via `code/validate_module21.py`)

**Verification state (2026-09-24):** `py tools/validate.py --paths <all 14 module md files>` → 0
errors, 0 warnings. `py 21-war-machine/code/validate_module21.py` → 0 errors (only the soft
word-count note: lessons run 3,774–5,595 words — above AUTHORING.md's "typically 1,500–3,500"
but in-format: the course's own module 20 runs 5,274–6,028 words on 20.04–20.06).
`py -m pytest 21-war-machine\code -q` → **33 passed, 19 skipped** (skips = unfilled exercise
stubs; fresh clone is green). Demo outputs quoted in 21.05/21.06/21.07 verified verbatim
against a fresh run. External link probe (`--external`) → **0 dead links**: 4 stale links found
and replaced (BU-703 renumbered → BU-802a/BU-902; Samsung 35E PDF gateway retired → batemo spec
sheet; OpenCV `4.x` ArUco 404s → version-pinned 4.10.0; analog.com INMP441 timed out → Mouser
product page). All shop links re-probed 2026-09-24.

## File inventory (actual, as of 2026-09-24)

```
21-war-machine/
├── PROGRESS.md            ← this file
├── README.md              ← module index (table + code listing + research pointer)
├── 21.01-war-machine-architecture.md
├── 21.02-payloads-and-bom.md
├── 21.03-releasers-i-latches.md
├── 21.04-releasers-ii-launch.md
├── 21.05-triggers-remote.md
├── 21.06-triggers-vision.md
├── 21.07-triggers-sound.md
├── 21.08-weights-power-stability.md
├── 21.09-failure-modes-and-safety.md
├── 21.10-integration-and-demo.md
├── hardware/
│   └── war-machine-bom.md ← Israeli shop BOM with prices + links (verified 2026-09-23/24)
├── code/
│   ├── war_power_budget.py    ← mass/cg/tipping, Thevenin sag, firing-energy budget, packages, check
│   ├── releaser.py            ← flap torque, solenoid pin force, pulse width
│   ├── projectile_range.py    ← launch model: muzzle velocity, ideal/dragged range, recoil, arm
│   ├── trigger_pipeline.py    ← Weapon FSM + 4 triggers (button/RC/vision/sound) + demo
│   ├── test_war_machine.py    ← 33 unit tests, no hardware
│   ├── conftest.py            ← sys.path for the module code
│   ├── validate_module21.py   ← module format/link/chain validator (wraps tools/validate.py)
│   ├── _outputs.txt           ← captured CLI outputs (re-captured 2026-09-24)
│   └── exercises/
│       ├── student.py         ← 10 stubs (the -E2 coding exercises, one per lesson)
│       ├── test_exercise.py   ← 19 checkers, keyed 2101_e2 … 2108_e2 (skip until filled)
│       └── conftest.py
└── references/
    └── war-machine-research-2026-09.md ← dated shop-verification log (MAINTAINING.md convention)
```

## Research log (Israeli OTC verification)

> Record here: what was verified, where, price, date. Unverified claims get "(unverified)".

- [x] Base shop list verified 2026-09-16 (existing course): `hardware/suppliers-israel.md`.
- [x] Servos: MG996R 180° w/ feedback + MG996R 360° continuous in stock at [Hackstore](https://hackstore.co.il) (page exists; JS-rendered → price TBC, ~₪80–120 estimate). ST3215 bus servo ₪107 at Piitel (course docs). AGF 1/5 78 kg servo ₪1,247 at [Ron Maor](https://ronmaor.co.il) (heavy-duty option).
- [x] 2.4 GHz RC: STX2 2-channel 2.4 GHz FHSS TX + SRX200 RX at [Maof RC (טיסן, Kiryat Atzmonot)](https://www.maof-rc.co.il/store/stx2-2-channel-24ghz-fhss-with-srx200). ELRS modules: import/RC shops (unverified local price).
- [x] Mic: INMP441 I2S mic module in stock at [Hackstore](https://hackstore.co.il) (price TBC). USB mic at Zap (generic).
- [x] CO2: 12 g paintball CO2 cartridges at [Fox Paintball](https://www.foxpaintball.co.il) (PolarStar CGS insert 12 g); 95 g disposable at [Perfectreef](https://perfectreef.konimbo.co.il) (aquariums, larger guns). Paintballs 1 kg 0.25 g at Fox Paintball.
- [x] Linear actuator 12 V: 120 N/12 V/100 mm, 150 N/150 mm, 150 N/200 mm, 600 N/120 mm, 1200 N/150 mm all in stock at [Hackstore](https://hackstore.co.il) (prices TBC, ~₪150–400 estimate).
- [x] Solenoids: linear solenoid 35 mm 12 V 42 N and 85 N at [Hackstore](https://hackstore.co.il). Latching solenoids: import (AliExpress) — marked accordingly.
- [~] Smoke / sparklers: cold-smoke sticks = holiday OTC (gas stations, toy/fireworks shops) — mark "estimate ₪5–15/stick, verify in season". Sparklers (סמרטוטים) same channels. No single verified web shop found; Fox Paintball likely carries paintball smoke (verify in-store).
- [x] Projectiles: airsoft BBs 10 kg 0.25 g at [Fantom (fantompepper.co.il)](https://www.fantompepper.co.il); CYMA CO2 blowback BB magazine ₪180 at Fantom; steel balls (bearing 3–5 mm) KSP/hardware; bouncy balls Zap/toy stores; neodymium magnets [wango-caravans.com](https://wango-caravans.com) + KSP.
- [x] Piezo: 20-pack piezo buzzers at [Hauzattahel](https://www.hauzattahel.co.il/Bargain_29322_Global.htm); 30×5.5 mm piezo elements at [DreamDigital](https://dreamdigital.co.il/boutique_18955-Store.html).
- [x] Springs/rubber bands/torsion springs/zip ties: generic OTC (KSP, hardware stores, stationery) — no specific shop link needed.
- [x] Erco e-stop ₪67.20, 4Project 20 A relay ₪27.60 (course docs, verified 2026-09-16).
- Note: Hackstore pages are JS-rendered → web_fetch truncates; prices marked "TBC" until browsed manually.

## Decisions log

| # | Decision | Date | Reason |
|---|---|---|---|
| D1 | New folder name: `21-war-machine/` at repo root, parallel to `20-final-robot/` | 2026-09-23 | Continues the module numbering; "parallel to the existing course" as requested |
| D2 | Module has 10 lessons (not 6) | 2026-09-23 | User asked for breadth: ≥3 trigger types, ≥3 releaser types, BOM, weights, failures |
| D3 | BOM page lives inside the module (`hardware/war-machine-bom.md`), not in `hardware/` | 2026-09-23 | Can't touch existing folder; module stays self-contained for GitHub |
| D4 | Do NOT add entries to `curriculum/syllabus.yaml` / `course.py` | 2026-09-23 | Those are existing files; module is parallel/standalone. (Revisit if user wants it in the main tool.) |

## Session log

| Session | Date | What happened |
|---|---|---|
| 1 | 2026-09-23 | Started. (Likely crashed before creating anything — no files found at restart.) |
| 2 | 2026-09-23 | Started. (Likely crashed before creating anything — no files found at restart.) |
| 3 | 2026-09-23 | Restarted. Verified repo state: no prior war-machine files exist. Created `21-war-machine/{,code,hardware}`, wrote this PROGRESS.md, read course format (20-final-robot, AUTHORING, HARDWARE, suppliers, karmel.yaml). Wrote `21.01` (35.8 KB). |
| 4 | 2026-09-24 | **The writing session.** Wrote 21.02 (payload taxonomy + impact-energy table) and 21.03 (releasers I) in full. Extended `war_power_budget.py` with the `sag` command (Thevenin); wrote `releaser.py`, `projectile_range.py`, `trigger_pipeline.py`, `test_war_machine.py` (33 tests), `exercises/{student.py,test_exercise.py,conftest.py}`. Dispatched 4 subagents: 10e2806d **completed** 21.05+21.07 (verified format + verbatim demo outputs); 3 others failed (infra). Wrote 21.04, 21.06, 21.09, 21.10 myself. Fixed 21.01 (Prerequisites section + markers, 2 external URLs, E2 path). Wrote `references/war-machine-research-2026-09.md`. Ran the full validation pass: course validator 0 errors, module validator 0 errors, pytest 33 passed/19 skipped. |

**Session 4 details (what was verified, for the next restart):**
- All 10 lessons pass the course validator (`tools/validate.py` via `code/validate_module21.py`):
  heading order, glance + prereqs markers, exercise ids, ≥5 `<details>` knowledge checks,
  ≥2 external URLs in Go deeper, ≥1 `[!CAUTION]` on hardware lessons, diagrams, "Ask your
  teacher" tips.
- **GPIO plan (final, conflict-free):** 8/9 flap servos (21.03) · 16/17/18 I²S mic BCLK/WS/SD
  (21.07) · **16 → 20** for the RC RX when the mic is wired (21.07 moves it; 21.05's table still
  shows 16 as the starting point) · **21** IR phototransistor (21.05, moved from 20 to keep the
  plan conflict-free).
- Lesson lengths: 3,774–5,595 words — above the "typically 1,500–3,500" band but in-format
  (module 20's own 20.04–20.06 run 5,274–6,028).
- Subagent-written 21.05/21.07 were spot-checked: format contract, verbatim demo quotes,
  real pin tables. All good.

## Next steps (on restart, in order)

**All 10 lessons are written and verified (session 4, 2026-09-24).** If you restart now,
the module is complete; the remaining items are polish, not structure:

1. Read this file.
2. **Optional polish, in order of value:**
   - Trim any lesson you find too long (they run 3.8–5.6k words; in-format, but if the user
     wants strict 1,500–3,500 the Knowledge check and Troubleshooting are the trimmable parts).
   - Re-verify the Hackstore "TBC" prices by browsing (the BOM marks them TBC because the
     pages are JS-rendered; update `references/war-machine-research-2026-09.md` with a new date).
   - If the user wants the module in the main tool: add entries to `curriculum/syllabus.yaml`
     (decision D4 says don't, unless asked), then `python tools/build.py` + `python tools/validate.py`.
   - The exercise stubs in `code/exercises/student.py` are intentionally unfilled (the student
     fills them); a fresh clone is `33 passed, 19 skipped`.
3. Update the plan table, research log, and session log before pausing.
4. Keep all external links verified with a date; prices in ₪ with 18% VAT.

## The module is done (definition of done, session 4)

- 10 lessons, exact course format, all internal links resolve, all external links https + dated.
- A full Israeli OTC BOM (`hardware/war-machine-bom.md`) with verified shop links.
- Four runnable code modules + 33 unit tests + 10 exercise stubs with checkers.
- The weight/power/stability math (CG, tipping, Thevenin sag, firing energy) all computed by
  `code/` and quoted in the lessons with real CLI output.
- A dated research-evidence file (`references/war-machine-research-2026-09.md`).
- Spec deltas documented in the module (21.01 Level 5 table + the BOM energy-column
  corrections in 21.04), never in the original course files.
