# Safety

Small robots feel harmless. They are not: a shorted 18650 pack can deliver tens of amps and
start a fire, a 30:1 gear motor will happily crush a finger or chew a cable, a robot arm moves
faster than you can react, and an autonomous robot does exactly what its code says — including
driving off the stairs. Safety in this course is not a chapter at the end. It is a set of
habits applied in every hardware lesson, and every lesson repeats the relevant rule at the
moment you need it.

> [!CAUTION]
> These rules are for hobby-scale robots (≤ 25 V, ≤ 20 A, arms with payloads under ~1 kg).
> Mains voltage (230 V in Israel) is never part of this course: use certified chargers and
> power supplies only, and never open or modify them.

## The ten standing rules

1. **Wheels in the air for first tests.** Any new motor code, wiring change or controller gain
   runs first with the robot on a stand (a mug or a box under the chassis).
2. **A power switch you can reach in one second**, cutting motor power. From Stage 6 this
   becomes a latching emergency-stop button wired to a relay/contactor in the motor supply.
3. **Watchdogs everywhere.** Motors stop if commands stop: the Pico firmware stops motors when
   no valid command arrives for 300 ms; the ROS base node stops on `cmd_vel` timeout; teleop
   uses a dead-man key. Never remove a watchdog "temporarily".
4. **Fuse the battery.** An inline fuse (blade or ATC, sized just above your maximum current)
   as close to the battery positive as possible, before the switch.
5. **Lithium batteries are treated like a fuel tank.** Charge only with a proper charger, on a
   non-flammable surface or in a LiPo-safe bag, never unattended, never overnight, never in a
   hot car (Israeli summer car interiors exceed 60 °C). Stop using any swollen, dented,
   punctured or hot cell. Store partially charged (~3.8 V/cell). Never short the leads: cover
   exposed connectors, insulate before cutting wires.
6. **Software limits before hardware limits.** Cap velocity, acceleration, arm joint ranges,
   speed and torque in software first — then test in simulation — then on hardware at low
   limits — then raise the limits gradually.
7. **Clear the test area.** Nothing fragile, no cables across the floor, stairs blocked, and
   **people, children and pets out of the robot's path**. Pets are fascinated by moving robots
   and cables; small parts (screws, M3 nuts, cells) are choking hazards.
8. **One hand for the stop.** When running autonomous code the first time, keep a hand on the
   switch (or the e-stop / the `Ctrl+C` terminal). Be ready before you press Enter.
9. **Power off before rewiring.** Disconnect the battery (not just the switch) before changing
   wiring. Check polarity with a multimeter before connecting anything new.
10. **Soldering and tools:** ventilate or use a fume extractor, wear safety glasses when
    clipping leads and soldering, park the iron in its stand, wash hands after handling leaded
    solder, unplug the iron when you leave the bench.

## Hazards by stage

| Stage | New hazard | Controls | Taught in |
|---|---|---|---|
| 1 — first robot | Li-ion shorts and fires; motors pinching fingers; runaway robot | Fuse, switch, BMS pack, wheels up, firmware watchdog, low-voltage cutoff | [01.07](01-first-robot/01.07-power-system.md), [01.10](01-first-robot/01.10-pi-pico-protocol.md), [01.14](01-first-robot/01.14-battery-monitoring.md), [FE.14](optional-foundations/electronics/FE.14-lithium-battery-safety.md) |
| 1 — soldering | Burns, fumes, flying leads, lead exposure | Stand, ventilation, glasses, lead-free solder, hand washing | [FE.17](optional-foundations/electronics/FE.17-soldering.md), [01.03](01-first-robot/01.03-workbench-and-tools.md) |
| 2 — sensors | Mostly low risk; 5 V signals damaging 3.3 V pins | Level shifting, check before connecting | [02.05](02-robot-electronics/02.05-logic-levels-and-protection.md) |
| 3 — LiDAR | Laser light | Consumer 2D LiDARs used here are Class 1 eye-safe; still never stare into or disassemble the emitter | [07.07](07-sensors/07.07-2d-lidar.md) |
| 3–4 — autonomy | Robot driving into people, pets, stairs, cables | Speed limits (≤ 0.3 m/s indoors), collision monitor, keepout zones, supervised tests | [12.10](12-navigation/12.10-recovery-and-navigation-safety.md) |
| 5 — robotic arm | Pinch points, fast unexpected motion, servo stall heating | Torque and speed limits, joint limits, workspace boundaries, torque-off on fault, start at low speed, keep face out of the workspace | [14.11](14-robotic-arm/14.11-arm-safety.md), [14.03](14-robotic-arm/14.03-controlling-servos.md) |
| 5 — learned policies | Unpredictable actions from ML policies | Action clipping, workspace envelopes, human with e-stop, evaluate in sim first | [18.10](18-embodied-ai/18.10-evaluating-learned-policies.md) |
| 6 — LLM agents | Language model issuing unsafe or injected commands | Allowlisted skills only, parameter validation, geofences, human confirmation for risky actions, deterministic low-level controllers, prompt-injection awareness | [19.09](19-llm-robot-agents/19.09-agent-safety-boundaries.md) |
| 6 — final robot | Everything combined, higher energy | Hardware e-stop, layered watchdogs, safe shutdown, written test procedure | [20.04](20-final-robot/20.04-safety-case.md) |

## The emergency stop (Stage 6)

A real e-stop removes energy from actuators **in hardware**, independent of software:

```text
Battery + ──[fuse]──[main switch]──┬───────────────────────────▶ 5 V buck → Raspberry Pi / Pico (stay powered, log the event)
                                   │
                                   └──[E-STOP NC contact]──[relay/contactor coil path]──▶ motor driver V+ and arm servo supply
```

- Normally-closed, latching mushroom button: pressing it opens the actuator supply; twisting
  releases it. Releasing must **not** restart motion — software requires an explicit re-enable.
- The computers stay powered so the robot can log what happened and shut down cleanly.
- Test the e-stop at the start of every session in which the robot moves autonomously.

## Safe testing procedure (use for every new behavior)

1. Code review your own change: limits, units (m vs mm, rad vs deg), sign conventions.
2. Run it in simulation (course mini-simulator or Gazebo) and look at the plots.
3. On the robot: wheels in the air / arm at low torque and speed; watch telemetry.
4. On the floor: open area, slow limits, hand on the stop.
5. Increase limits one at a time. Record what you changed.

## If something goes wrong

- **Smoke, a hot smell, a hissing or swelling battery:** disconnect power if you can do so
  safely, move away, and if a lithium cell is venting or burning, do not use water on a large
  pack fire indoors if you can avoid it — get out, close the door, and call the fire service
  (102 in Israel). For a small cell that has stopped venting, move it outdoors onto sand or
  concrete with non-conductive tongs once it is cool.
- **Burn:** cool under running water for 20 minutes.
- **Robot runaway:** hit the switch/e-stop; don't try to grab moving wheels or arms.
- Afterwards, find the root cause before powering on again (the Troubleshooting sections teach
  systematic debugging).

## Further reading

- Battery University — BU-304a Safety Concerns with Li-ion: https://batteryuniversity.com/article/bu-304a-safety-concerns-with-li-ion
- Adafruit — Li-ion & LiPoly batteries guide: https://learn.adafruit.com/li-ion-and-lipoly-batteries
- ROS 2 security (SROS2) documentation: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Security/Introducing-ros2-security.html
- ISO 10218 (industrial robot safety; ISO/TS 15066 collaborative guidance is now folded into ISO 10218-2:2025) — overview: https://www.iso.org/standard/73934.html
