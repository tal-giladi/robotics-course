# Troubleshooting: motors and drivers

> [!CAUTION]
> **Safety:** every test on this page runs with the **wheels in the air** — the robot on a stand, a
> mug or a box — and a power switch within reach. A motor test that surprises you on the floor is a
> robot in the wall. → [SAFETY.md](../../SAFETY.md)

The chain is short and you can bisect it with a multimeter: **command → firmware pin → driver input →
driver output → motor**. Measure in the middle first.

```text
  Pico GPIO ──▶ driver IN1/IN2/PWM ──▶ [H-bridge] ──▶ OUT1/OUT2 ──▶ motor
      │                                     ▲
   measure                              motor V+ (from the battery, NOT the 5 V rail)
```

---

### Symptom: the motor does not move at all

Work the chain from the cheap end.

1. **Is motor power present?** Measure V+ at the driver's power terminals, with the battery connected
   and the switch on. Zero volts here ends the investigation. → [01.08](../../01-first-robot/01.08-first-motor-spin.md), [FE.12](../../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md)
2. **Is the driver enabled?** Many carriers have a standby/enable pin that must be pulled high. It is
   the most commonly forgotten wire. → [FE.12](../../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md)
3. **Is the Pico actually driving the pins?** Set a direction pin high in the REPL and measure it.
   3.3 V means firmware is fine and the problem is downstream. → [01.05](../../01-first-robot/01.05-pico-microcontroller-setup.md), [01.08](../../01-first-robot/01.08-first-motor-spin.md)
4. **Is the duty above the deadband?** Below about 0.12 duty, karmel's motors do not turn at all.
   Command 0.5 for the test. This alone explains a large fraction of "the wheel does not move". →
   [01.08](../../01-first-robot/01.08-first-motor-spin.md), [08.02](../../08-control/08.02-motor-step-response.md)
5. **Measure the driver's output** across OUT1/OUT2 while commanding. Voltage there and no motion
   means the motor or its connector; no voltage means the driver or its inputs.
6. **Bypass the driver as the final test:** briefly connect the motor directly to a safe supply. If
   it hums but does not turn, it is mechanically jammed or one winding is open. → [FE.10](../../optional-foundations/electronics/FE.10-dc-motors.md)

### Symptom: the motor only brakes and coasts, and never reverses

1. **Check that both direction inputs are wired and both are being driven.** Only one connected means
   you have forward and stop, not forward and reverse. → [FE.12](../../optional-foundations/electronics/FE.12-h-bridges-motor-drivers.md)
2. **Check the firmware's truth table against the driver's datasheet** — IN1/IN2 vs PWM/DIR are
   different conventions and cheap carriers mix them. → [02.01](../../02-robot-electronics/02.01-datasheets-and-pinouts.md), [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
3. **If it reverses in the REPL but not from the protocol**, the sign is being lost in the command
   path. Log the value that reaches the PWM call. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)

### Symptom: one wheel spins the wrong way

1. **Do not fix this in three places.** Pick one: swap the motor's two leads, or invert the sign in
   the firmware, or set the per-wheel invert flag in `karmel.yaml`. Then write down which. →
   [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)
2. **Verify with the encoder, not your eyes:** drive forward and check that both wheels' counts
   *increase*. Motor direction and encoder sign are two separate bugs that hide each other. →
   [01.09](../../01-first-robot/01.09-reading-encoders.md), [09.02](../../09-odometry/09.02-inverse-kinematics-cmd-vel.md)
3. The sign convention: forward motion increases counts, positive `angular.z` turns left. → [09.01](../../09-odometry/09.01-diff-drive-kinematics.md)

### Symptom: the motor whines, or the sound changes with duty

1. **Audible whine is PWM frequency inside hearing range.** Raise it (typically to ~20 kHz) and the
   whine goes away. → [FE.07](../../optional-foundations/electronics/FE.07-pwm.md), [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
2. **But check the driver's specification first:** many H-bridge carriers cannot switch cleanly at
   20 kHz and will get hot instead. Trade audibility against switching loss deliberately. →
   [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
3. A motor that whines *and* does not turn at low duty is the deadband, not the frequency. →
   [08.02](../../08-control/08.02-motor-step-response.md)

### Symptom: the motor stutters rhythmically under load after 20–60 seconds

This is thermal shutdown cycling, and the timing is the clue.

1. **Touch the driver carrier** (carefully). Hot means the thermal protection is tripping,
   recovering, and tripping again. → [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
2. **Compare your continuous current against the carrier's continuous rating**, not its peak rating.
   Peak ratings are for milliseconds. → [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md), [FE.10](../../optional-foundations/electronics/FE.10-dc-motors.md)
3. **Add a heatsink or airflow, reduce the current limit, or raise the gear ratio** so the motor does
   less work. Do not just live with it: it will fail mid-run.
4. If the *motor* is hot rather than the driver, it is stalling or over-geared. → [FP.04](../../optional-foundations/physics/FP.04-torque-gears-wheels.md)

### Symptom: the wheel does not move below ~40 % duty, and the velocity PID oscillates

1. **Measure the deadband properly**: sweep duty from 0 upward and record the first duty that
   produces motion, for each wheel and each direction. They differ. → [08.02](../../08-control/08.02-motor-step-response.md)
2. **Feed the deadband into the controller** as a feedforward offset instead of asking the integral
   term to climb through it. Windup through a deadband is exactly what makes the loop oscillate. →
   [08.09](../../08-control/08.09-feedforward-exact-speed.md), [08.05](../../08-control/08.05-integral-control.md)
3. A deadband much larger than 0.12 on karmel points at supply sag under load or a tight gearbox. →
   [02.02](../../02-robot-electronics/02.02-power-budget.md)

### Symptom: the current-sense reading is noisy, or reads full scale

1. **Full scale at rest** means the sense pin is floating or mis-wired — check the connection before
   believing the number. → [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
2. **Noisy while driving** is normal: current sense sees the PWM chopping. Low-pass filter it, and
   synchronise the sample away from the switching edge if you can. → [FE.08](../../optional-foundations/electronics/FE.08-adc.md), [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
3. Route the sense wire away from the motor leads, and make sure it returns to the star ground point.
   → [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)

### Symptom: the driver dies when you disconnect the battery to "stop quickly"

1. Do not do this. A spinning motor is a generator; with its supply removed, the back-EMF has nowhere
   to go and the H-bridge takes it. → [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md), [FE.18](../../optional-foundations/electronics/FE.18-capacitors-diodes.md)
2. Stop in software (duty 0, brake mode), then switch off. The e-stop, which removes actuator power
   through a contactor, is designed for the case where you cannot. → [20.04](../../20-final-robot/20.04-safety-case.md)

### Symptom: the Pico resets when the motor reverses

1. This is the power-collapse family: reversal draws roughly twice stall current. See
   [Power and battery](power-and-battery.md).
2. Also check grounding: the Pico must share ground with the driver, on a short, dedicated wire, and
   the motor return current must not flow through that wire. → [FE.16](../../optional-foundations/electronics/FE.16-grounding.md), [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
3. Add bulk capacitance at the driver and a decoupling capacitor at the Pico. → [FE.18](../../optional-foundations/electronics/FE.18-capacitors-diodes.md)

### Symptom: the motors stutter or stop every few hundred milliseconds

Almost never the motors. It is the 300 ms firmware watchdog expiring because commands are late or
being rejected. → [Serial link to the Pico](serial-and-pico.md), [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)

## Where to go next

- H-bridges, PWM frequency, current and braking in depth: [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md)
- The motor's own model (deadband, time constant, steady-state gain): [08.02](../../08-control/08.02-motor-step-response.md)
- Motor physics, torque and sizing: [FE.10](../../optional-foundations/electronics/FE.10-dc-motors.md), [FP.04](../../optional-foundations/physics/FP.04-torque-gears-wheels.md)
- Why the computer reboots when motors start: [Power and battery](power-and-battery.md)
- Counts, signs and "it curves": [Encoders, odometry and control](control-and-odometry.md)
