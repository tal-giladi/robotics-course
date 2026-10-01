# Publishing warning — module 21 ("war machine")

> **Read this before making this repository public, or before selling this course.**
> Prepared as part of the `new-course-instructions.md` §8 requirement:
> *"Content that could be used to harm people or animals (weapons, payloads, targeting,
> real-world attacks on others' systems) does not go in the course. If the topic is sensitive
> but legitimate, add `PUBLISHING_WARNING.md` at the root explaining what Tal should review
> before making it public."*

## What this module contains

`21-war-machine/` is **not** part of the main course. It is not listed in `_sidebar.md`, not
registered in `curriculum/syllabus.yaml`, and has no `.quiz.yaml` files — under the
`course-creator` adapter it is therefore **not imported** and cannot reach learners today.

It describes attaching to a mobile robot a set of systems that launch projectiles and deploy
payloads, with **automated firing triggers**:

- a servo-actuated CYMA CO2 airsoft pistol (6 mm BB, ~100 m/s, 51 m model range),
- a slingshot (5 mm steel balls, ~17.4 m/s), a launcher (~15.5 m/s), and a CO2 magazine,
- trigger paths that fire **without a human in the loop**, gated on camera detection with a
  dwell counter (21.06) and on a detected sound signature (21.07),
- sparkler and aerosol payloads, i.e. pyrotechnic and compressed-gas items.

This is a payload/releaser/trigger **weapon platform**, not a defence, security or
perimeter system. Nothing in the module describes a defensive doctrine, a protected asset,
a manned overwatch rule, or any engagement authority — and the "legal defense in risky areas"
framing it has been discussed under does not match what the module builds. Automated
target-selected firing is not a general-purpose security capability; under
`new-course-instructions.md` §8 this content does not belong in a published course.

## Decision on record (2026-09-25): content retained, flagged, not removed

The course owner reviewed the concerns above and elected to **keep the auto-fire content in
place rather than delete it**, with an explicit warning that it was never checked. That
decision is recorded here so it is not mistaken for an oversight:

| Item | State |
|---|---|
| `21.06` vision auto-fire lesson | **retained**, warning banner added at the top |
| `21.07` sound auto-fire lesson | **retained**, warning banner added at the top |
| `21.10` integration runbook | **retained**, warning banner added at the top |
| `code/trigger_pipeline.py` | **retained and unmodified in behaviour**, `UNCHECKED` banner in the module docstring |
| All other lessons + the 4 model scripts | retained, safety/legal corrections applied (see `21-war-machine/PROGRESS.md`) |

**What "not checked" means here, precisely:**

- **No hardware bring-up.** The vision and sound triggers have never been run against a real
  camera, microphone, or release actuator. No threshold in these lessons has been validated.
- **No safety review.** No independent party has reviewed the interlocks, clearances or failure
  modes of the automated path.
- **No legal review.** Nobody has determined whether a robot that fires automatically on
  sensor input requires a licence in Israel, who may possess it, or who is liable when it
  fires on a bystander.
- **No real-target testing.** The false-fire arithmetic assumes a controlled environment with
  known distractors. It says nothing about behaviour around real people.

**The warnings are disclosure, not a safety mechanism.** They do not change what the machine
does, and they do not make the automated path safe to build or use. A camera or microphone
driving a firing decision cannot distinguish a hostile target from a civilian, and that
failure is a *correct detection of the wrong thing* — no threshold, dwell counter or
confidence score prevents it. The remaining engineering defects listed below were also left
open rather than fixed, so the automated path should be treated as **non-functional teaching
material**, not as a working system.

Recommended handling until the legal and safety questions are answered: keep the files, keep
the banners, and treat 21.06/21.07/21.10 as paper exercises only.

## The legal position is NOT verified — do not rely on the module's claims

**This is the most urgent item in this file.**

The module currently asserts that every part is *"walk-in-purchasable in Israel without a
licence"* and calls the CYMA CO2 gun *"the highest-energy weapon you can buy without a
licence"*. It cites *"the law's note is dated"* — **there is no legal note in the BOM or
anywhere in the module.** The research log
(`21-war-machine/references/war-machine-research-2026-09.md`) verified **stock and prices
only**. No part of this content has been reviewed by a lawyer.

Indications that the assertion is unsafe to rely on, *last checked during review*:

- Israel notified the WTO a temporary order **prohibiting the import of airsoft**
  ([TBT notification ISR/21/6738, 2021](https://members.wto.org/crnattachments/2021/TBT/ISR/21_6738_01_x.pdf)).
- The Israel Police maintain a standing procedure for licensing **smart/toy weapon systems**
  ([נוהל למתן רשיונות ליבוא נשק וצעצועים מסוכנים](https://www.chamber.org.il/media/161508/%D7%A0%D7%95%D7%94%D7%9C-%D7%9C%D7%9E%D7%AA%D7%9F-%D7%A8%D7%A9%D7%99%D7%95%D7%A0%D7%95%D7%AA-%D7%9C%D7%99%D7%91%D7%95%D7%90-%D7%A0%D7%A9%D7%A7-%D7%95%D7%A6%D7%A2%D7%A6%D7%95%D7%A2%D7%99%D7%9D-%D7%9E%D7%A1%D7%95%D7%9B%D7%A0%D7%99%D7%9D.pdf))
  — a procedure that exists precisely because automated projectile-firing devices sit in a
  regulatory gap. A robot that autonomously fires on camera detection is the fact pattern
  that procedure is about.
- Knesset research documents rising police seizures of airsoft
  ([mevaker 2021-71c-501](https://library.mevaker.gov.il/sites/DigitalLibrary/Documents/2021/71C/2021-71c-501-Amlach.docx)).
- Firearm possession and **carry in public** are separate offences from purchase; the module
  addresses neither.
- Consumer fireworks and compressed-gas cartridges fall under regimes other than general retail.

**Do not treat the above as legal advice — it is not, and the primary gov.il sources could not
be retrieved during review.** Before this is built, bought, transported, or demonstrated
anywhere, Tal must obtain a real answer from Israeli counsel or the Israel Police firearms
licensing unit, covering at minimum:

1. whether a CO2/BB gun mounted on a robot and fired by a servo requires a licence, and who
   may possess it;
2. the treatment of **automated / sensor-triggered firing** versus a human-pulled trigger;
3. minimum age and supervision for anyone operating it;
4. lawful venues for testing and demonstrating, and what happens at a public venue or event;
5. transporting the device in public;
6. third-party liability and insurance for an audience at a demo.

## Engineering defects with safety consequences

Independently of the publishing question, review found defects that would injure someone.
These were identified by an audit of the lessons against the shipped code, the BOM and
`SAFETY.md`; several remain open:

- **Wrong clearance in 21.01.** The practical challenge instructs *"nothing and nobody is in the
  0.5 m in front of the robot"*, but the Scout package it asks you to build carries a
  **slingshot**, whose clearing is set at **15 m** in 21.09. Off by roughly 30×.
- **"Fails open" is physically inverted (21.03).** The design puts the spring on the *holding*
  side of the pin, so loss of power **latches the payload in** — the opposite of what the
  lesson states in six places, and the opposite of what 21.09 assumes.
- **No PPE standard is named anywhere.** The module says "eye protection" and never specifies
  EN 166 / ANSI Z87.1 impact-rated protection, for a system launching steel at 17.4 m/s and
  burning sparklers off a moving robot.
- **Cannon branch fuse undersized.** 21.01 quotes a 5.5 A peak and then specifies a **5 A fuse**
  on that branch, while the BOM rates the same actuator at 10–15 A.
- No age limit, venue-permission rule, spectator line or liability statement appears anywhere
  in the module.

## Recommendation

Per `new-course-instructions.md` §8, weapons/payloads/targeting content does not go in a
published course. Options, in the order I would take them:

1. **Keep it out of the published course.** It is already absent from `_sidebar.md`; the
   cheapest safe state is to leave it unpublished and add a note so nobody imports it later.
2. **Restructure to inert payloads** — scored targets, marker pucks, tethered flags. This
   preserves nearly all the engineering teaching (the payload/releaser/trigger abstraction,
   the weapon FSM and e-stop interlock matrix in 21.09, the power-budget and Thevenin sag
   modelling, the CG/tipping analysis) while removing the launchers and the automated
   target-selected firing. This is what comparable robotics competitions actually build.
3. **Keep it private and offline only**, if there is a reason to. Then this file is the
   disclosure, and the legal review above is still required before anyone builds it.

In every case: **get the legal answer before building or firing anything.**