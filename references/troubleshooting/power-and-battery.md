# Troubleshooting: power and battery

Power faults masquerade as software faults. A robot that "randomly reboots", "loses the Pico", "reads
the IMU wrong when driving" or "works on the bench and dies on the floor" is usually a power problem,
and no amount of reading your Python will find it.

> [!CAUTION]
> **Safety:** disconnect the battery — not just the switch — before changing any wiring, and check
> polarity with a multimeter before you reconnect. Never leave a lithium pack charging unattended.
> Stop using any cell that is swollen, dented, punctured or hot. → [SAFETY.md](../../SAFETY.md),
> [FE.14](../../optional-foundations/electronics/FE.14-lithium-battery-safety.md)

**Have these numbers before you start:** pack voltage at rest, pack voltage under load, the regulated
5 V rail under load, and your motors' stall current. Everything below is a comparison against them.
→ [FE.03](../../optional-foundations/electronics/FE.03-multimeter.md), [02.02](../../02-robot-electronics/02.02-power-budget.md)

---

### Symptom: the Pi reboots (or the Pico drops off `/dev`) as soon as the motors start

The single most common hardware fault in this course. It is a voltage collapse, not a software crash.

1. **Confirm it is a brownout, not a crash.** On the Pi, `vcgencmd get_throttled` — anything other
   than `0x0` means the supply dipped. A kernel panic would leave something in `journalctl`; a
   brownout leaves a clean boot log. → [01.04](../../01-first-robot/01.04-raspberry-pi-setup.md)
2. **Measure the 5 V rail while a motor starts**, at the Pi's connector, not at the regulator. A dip
   below ~4.7 V is your answer. Use a multimeter's min-hold if it has one, or watch the reading as a
   helper commands a step.
3. **Check whether the motors and the computer share a rail.** They must not. Separate buck
   converters (or at minimum separate rails from one supply with bulk capacitance on each) is the
   architecture the course uses. → [02.02](../../02-robot-electronics/02.02-power-budget.md), [01.07](../../01-first-robot/01.07-power-system.md)
4. **Look at the wire, not just the converter.** Thin wire and a long run to the motor driver drop
   volts under a 2–3 A inrush. Measure the voltage at *both* ends of the motor supply wire while
   driving. → [02.07](../../02-robot-electronics/02.07-wiring-harness.md)
5. **Add bulk capacitance across the motor driver's V+** (hundreds of µF, correct polarity and voltage
   rating) to absorb the inrush spike. → [FE.18](../../optional-foundations/electronics/FE.18-capacitors-diodes.md), [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
6. **Only then suspect the converter**: a buck rated 3 A that browns out at 1.5 A is either a
   counterfeit or is current-limiting because its input sagged. Check its *input* voltage under load
   too. → [FE.15](../../optional-foundations/electronics/FE.15-voltage-regulators.md)

Related: the same collapse is what resets the Pico ([02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)) and what makes the battery reading jump
only while driving ([01.14](../../01-first-robot/01.14-battery-monitoring.md)).

### Symptom: the BMS cuts the whole robot off during reversals or hard acceleration

The protection board is doing its job; the current spike is real.

1. **Reversal current is roughly twice stall current**, because back-EMF adds to the supply. Compute
   it: two motors, stall current each, times two. Compare with the BMS's over-current threshold. →
   [FE.13](../../optional-foundations/electronics/FE.13-batteries.md), [02.02](../../02-robot-electronics/02.02-power-budget.md)
2. **If the number exceeds the threshold, limit the rate of change of the command** (acceleration
   limits in software) before you buy a bigger BMS. The robot does not need to reverse in 50 ms. →
   [08.12](../../08-control/08.12-control-in-ros2.md)
3. **If it does not exceed it**, suspect a bad cell: measure each cell's voltage under load. One weak
   cell hits the low-voltage cut-off first and takes the pack down. → [FE.14](../../optional-foundations/electronics/FE.14-lithium-battery-safety.md)
4. **Check the pack's temperature and age.** Internal resistance rises with cycles; an old pack sags
   more for the same current. → [FE.13](../../optional-foundations/electronics/FE.13-batteries.md)

### Symptom: the fuse blows at switch-on, never while driving

That is inrush, not overload.

1. **Confirm the inrush hypothesis**: if it blows only at the instant of connection, capacitors are
   charging through the fuse. → [01.07](../../01-first-robot/01.07-power-system.md)
2. **Check the fuse rating against your real maximum current**, not against the average. Sized "just
   above maximum" means above stall, not above cruise. → [01.07](../../01-first-robot/01.07-power-system.md)
3. **If the fuse blows every time and instantly, stop and look for a short** before assuming inrush:
   with the battery disconnected, continuity between the battery positive rail and ground should not
   beep. → [FE.03](../../optional-foundations/electronics/FE.03-multimeter.md), [02.08](../../02-robot-electronics/02.08-from-breadboard-to-perfboard.md)
4. Reversed polarity on a buck converter or an electrolytic capacitor will also do this, once. →
   [FE.18](../../optional-foundations/electronics/FE.18-capacitors-diodes.md)

### Symptom: the regulator outputs the battery voltage, or nothing

1. **Nothing out:** check the input first. No input, no output — and an enable pin held low, or a
   shorted output, will also give you zero. → [FE.15](../../optional-foundations/electronics/FE.15-voltage-regulators.md)
2. **Battery voltage out** on an adjustable buck means the potentiometer was never set, or you have
   the module wired input-to-output. Set the output *before* connecting the load, with the load
   disconnected. → [01.07](../../01-first-robot/01.07-power-system.md)
3. **Hot linear regulator:** it is burning (V_in − V_out) × I as heat, which is exactly why the course
   uses a switching buck for the 5 V rail. → [FE.15](../../optional-foundations/electronics/FE.15-voltage-regulators.md)

### Symptom: the BMS output (`P+` to `P−`) reads 0 V

1. Many BMS boards latch off after an over-discharge or a short and need a charger connected briefly
   to reset. Try that before suspecting the board. → [01.07](../../01-first-robot/01.07-power-system.md), [FE.13](../../optional-foundations/electronics/FE.13-batteries.md)
2. If it stays at 0 V, measure the cells directly (carefully, one at a time). A cell below the
   under-voltage threshold keeps the BMS off. → [FE.14](../../optional-foundations/electronics/FE.14-lithium-battery-safety.md)
3. A pack that will not switch on after sitting for weeks is almost always this: self-discharge took
   a cell under the limit. Store packs partially charged. → [FE.14](../../optional-foundations/electronics/FE.14-lithium-battery-safety.md)

### Symptom: the battery voltage reads consistently 0.3–0.5 V low (or high)

1. **Check the divider resistors' actual values** with the meter, not their printed values. Tolerance
   on cheap resistors is 5 %, and the divider ratio inherits it. → [01.14](../../01-first-robot/01.14-battery-monitoring.md), [FE.02](../../optional-foundations/electronics/FE.02-ohms-law-series-parallel.md)
2. **Check the ADC reference**, not just the code: a Pico ADC referenced to a noisy 3.3 V rail reads
   proportionally wrong. → [FE.08](../../optional-foundations/electronics/FE.08-adc.md)
3. **Calibrate against the multimeter** at two points and store the correction in `karmel.yaml` rather
   than hard-coding it. → [03.06](../../03-robot-software/03.06-configuration-and-calibration.md), [01.14](../../01-first-robot/01.14-battery-monitoring.md)
4. If the reading only misbehaves while driving, it is not calibration — it is noise and ground
   offset. → [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)

### Symptom: the robot runs far shorter than the runtime calculation says

1. **Recheck the assumed average current.** Runtime estimates are usually made with cruise current;
   real driving is full of accelerations and stalls. Log actual current with the INA219 for one real
   session instead of estimating. → [01.14](../../01-first-robot/01.14-battery-monitoring.md), [FE.01](../../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md)
2. **Subtract the usable capacity, not the nameplate capacity.** You stop at the low-voltage cut-off,
   not at 0 V, and cheap cells are optimistically rated. → [FE.13](../../optional-foundations/electronics/FE.13-batteries.md)
3. **Account for the converters' efficiency** (85–90 % is typical) and the idle draw of the Pi, which
   runs whether or not the robot moves. → [FE.15](../../optional-foundations/electronics/FE.15-voltage-regulators.md), [02.02](../../02-robot-electronics/02.02-power-budget.md)

### Symptom: everything measures correctly on the bench, and the robot misbehaves on the floor

1. Bench supplies are stiff; batteries sag. Repeat the same measurement on battery power. →
   [02.02](../../02-robot-electronics/02.02-power-budget.md)
2. On the floor the robot draws more (carpet, slopes, real load) and vibrates (intermittent
   connections). Wiggle-test every connector with the robot powered and watch for glitches. →
   [02.07](../../02-robot-electronics/02.07-wiring-harness.md)
3. If a connection works when you press on it, it is a bad crimp or a lifted breadboard wire. Fix it
   properly; do not re-seat it and hope. → [02.07](../../02-robot-electronics/02.07-wiring-harness.md), [02.08](../../02-robot-electronics/02.08-from-breadboard-to-perfboard.md)

## Where to go next

- The systematic electrical method, with a fault tree: [02.09](../../02-robot-electronics/02.09-electrical-debugging-method.md)
- Power architecture and the power budget: [02.02](../../02-robot-electronics/02.02-power-budget.md)
- Grounding, decoupling and motor EMI: [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
- Battery chemistry, ratings and safety: [FE.13](../../optional-foundations/electronics/FE.13-batteries.md), [FE.14](../../optional-foundations/electronics/FE.14-lithium-battery-safety.md)
- Symptoms that continue into the motor driver: [Motors and drivers](motors-and-drivers.md)
