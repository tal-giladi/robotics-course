# Troubleshooting: encoders, odometry and control

The band between "the motor turns" and "the robot goes where I asked". Almost every fault here is one
of four things: **a sign**, **a calibration constant**, **a deadband**, or **a timing problem** — and
they produce recognisably different patterns.

| Pattern | Look at |
|---|---|
| Wrong *direction* | a sign: encoder polarity, motor wiring, or ω convention |
| Consistently wrong by the same *percentage* | a constant: wheel radius, ticks per revolution, wheel separation |
| Right in simulation, wrong on the floor | deadband, slip, latency, battery sag |
| Fine at low speed, wrong at high speed | missed counts, loop rate, saturation |

→ [09.05](../../09-odometry/09.05-calibrating-odometry.md), [08.02](../../08-control/08.02-motor-step-response.md), [01.12](../../01-first-robot/01.12-encoder-distance-and-square.md)

> [!CAUTION]
> **Safety:** wheels in the air for any new gain or new controller. A P gain that is too high does not
> fail gently. → [SAFETY.md](../../SAFETY.md)

---

### Symptom: the robot curves when commanded to drive straight

1. **Open loop?** Then it is supposed to curve — two motors are never identical. Confirm by looking at
   both wheels' tick counts over the same run. → [01.11](../../01-first-robot/01.11-drive-and-rotate.md), [08.01](../../08-control/08.01-open-vs-closed-loop.md)
2. **Closed loop on wheel speed and still curving?** Check that both wheels actually reach their
   setpoints; one at its duty limit will curve regardless of gains. → [08.10](../../08-control/08.10-straight-line-heading-control.md)
3. **Curving consistently one way** = a calibration difference (wheel radii, tick counts). Curving
   randomly = slip or a loose wheel. → [09.05](../../09-odometry/09.05-calibrating-odometry.md), [01.06](../../01-first-robot/01.06-mechanical-assembly.md)
4. **The tick counts match and it still curves on the floor** — that is slip and floor irregularity,
   which encoders cannot see. Close the loop on heading with the gyro. → [08.10](../../08-control/08.10-straight-line-heading-control.md), [07.02](../../07-sensors/07.02-wheel-encoders-in-depth.md)
5. **The gyro run curves consistently in one direction** → residual gyro bias. Re-measure it at rest.
   → [07.05](../../07-sensors/07.05-imu-fundamentals.md), [08.10](../../08-control/08.10-straight-line-heading-control.md)

### Symptom: the robot turns the opposite way from `cmd_vel`

1. The convention is **positive `angular.z` = left (counter-clockwise)**, x forward, y left. Fix the
   sign once, in the inverse kinematics, and write down that you did. → [09.02](../../09-odometry/09.02-inverse-kinematics-cmd-vel.md), [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
2. Then verify odometry *separately*: a robot that turns correctly and reports the turn with the wrong
   sign has two bugs that were cancelling. → [09.04](../../09-odometry/09.04-odometry-from-scratch.md)
3. "`rotate-left` turns right" at the open-loop stage is the same bug, earlier. → [01.11](../../01-first-robot/01.11-drive-and-rotate.md)

### Symptom: every side of the square is long or short by the same percentage

A pure scale error, and there are only two candidates.

1. **Wheel radius** — measure the *effective* rolling radius under load, not the moulded diameter.
   Drive 2 m against a tape measure and solve for it. → [01.12](../../01-first-robot/01.12-encoder-distance-and-square.md), [09.05](../../09-odometry/09.05-calibrating-odometry.md)
2. **Ticks per revolution** — for karmel, 11 × 56 × 4 = 2464. A different motor variant (1:30 rather
   than 1:56) changes it and nothing else. → [01.09](../../01-first-robot/01.09-reading-encoders.md), [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)
3. Put the corrected value in `karmel.yaml`, not in the code. → [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)

### Symptom: the sides are right and the corners are wrong

That is wheel separation, which only affects rotation.

1. Spin in place N full turns and solve for the effective wheel separation from the tick counts. It
   will not equal the number you measured with a ruler, because of contact-patch width and slip. →
   [09.05](../../09-odometry/09.05-calibrating-odometry.md)
2. Use UMBmark — the same square clockwise and counter-clockwise — to separate the systematic error
   from the random one. If you skip this, you will "calibrate" noise. → [09.05](../../09-odometry/09.05-calibrating-odometry.md)
3. If CW and CCW clusters overlap and the spread is large, your *random* error dominates and there is
   nothing systematic to correct yet. Fix slip and the floor first. → [09.05](../../09-odometry/09.05-calibrating-odometry.md)

### Symptom: odometry is fine driving straight and drifts in curves

1. You are integrating with Euler (straight-line) steps over a constant-curvature motion. Use exact
   arc integration or RK2. → [09.03](../../09-odometry/09.03-dead-reckoning-integration.md)
2. Verify numerically: halve the time step and see whether the error halves (Euler, first order) or
   quarters (RK2, second order). If a shorter step fixes it, the integrator is the cause. →
   [09.03](../../09-odometry/09.03-dead-reckoning-integration.md), [FM.18](../../optional-foundations/mathematics/FM.18-integrals-numerical-integration.md)

### Symptom: odometry jumps by metres

1. **Encoder rollover.** Compute deltas with modular arithmetic against the counter width. →
   [09.04](../../09-odometry/09.04-odometry-from-scratch.md)
2. **A missed telemetry frame** turned into a huge delta. Reject implausible deltas rather than
   integrating them. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md), [09.04](../../09-odometry/09.04-odometry-from-scratch.md)
3. **Two publishers of `odom → base_link`** — your node and `diff_drive_controller`, for example. →
   [TF and frames](tf-and-frames.md), [09.07](../../09-odometry/09.07-odometry-in-ros2.md)

### Symptom: the wheel does not move at all under a P controller, or goes to full speed backwards

1. **Does not move** → the output is inside the deadband, or the sign of the feedback is inverted so
   the controller is fighting itself. Log setpoint, measurement, error and output on the same
   timeline; the answer is always visible in that plot. → [08.04](../../08-control/08.04-proportional-control.md), [08.02](../../08-control/08.02-motor-step-response.md)
2. **Full speed backwards** → positive feedback: the encoder sign and the motor sign disagree. Fix the
   sign, not the gain. → [08.04](../../08-control/08.04-proportional-control.md), [01.09](../../01-first-robot/01.09-reading-encoders.md)

### Symptom: the speed estimate is 0 most of the time and spikes occasionally

Quantization. At low speed, few ticks arrive per control period.

1. Compute the speed from ticks per *elapsed time between actual samples*, not per nominal period. →
   [08.03](../../08-control/08.03-wheel-speed-estimation.md)
2. Below a few ticks per period, use a longer window or measure the time between edges instead of
   counting edges per window. → [08.03](../../08-control/08.03-wheel-speed-estimation.md)
3. "Roughly twice the real speed now and then" is the same artefact plus a timing jitter in the
   denominator. → [08.03](../../08-control/08.03-wheel-speed-estimation.md), [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)

### Symptom: the loop oscillates

1. **Reduce the gain and see whether the period changes.** A period that stays the same as you lower
   k_p points at delay (loop rate, filter lag), not at gain. → [08.07](../../08-control/08.07-pid-mathematics.md), [FCT.04](../../optional-foundations/control-theory/FCT.04-stability-intuition.md)
2. **Adding D made it worse** → D is amplifying quantization noise. Filter the derivative, or
   differentiate the measurement instead of the error. → [08.06](../../08-control/08.06-derivative-control.md)
3. **Slow rolling oscillation that P alone did not have** → integral gain too high, or windup. →
   [08.05](../../08-control/08.05-integral-control.md)
4. **Fine at one setpoint and oscillating at another** → the plant is nonlinear (deadband, saturation,
   gearbox backlash); gain-schedule or add feedforward. → [08.07](../../08-control/08.07-pid-mathematics.md), [08.09](../../08-control/08.09-feedforward-exact-speed.md)
5. **A gain that was fine yesterday** → battery voltage. A full pack has more loop gain than a flat
   one; feedforward scaled by the measured voltage fixes it properly. → [08.09](../../08-control/08.09-feedforward-exact-speed.md), [08.04](../../08-control/08.04-proportional-control.md)

### Symptom: the speed creeps to the setpoint very slowly, or never quite arrives

1. Steady-state error with P alone is expected; it is exactly what integral action is for. →
   [08.04](../../08-control/08.04-proportional-control.md), [08.05](../../08-control/08.05-integral-control.md)
2. If the integral term is present and the error persists, check for anti-windup clamping it, or for
   the output sitting at a limit. → [08.05](../../08-control/08.05-integral-control.md)

### Symptom: the robot lunges, or keeps pushing after being blocked or lifted

Integral windup. The integral accumulated while the wheel could not move, and it discharges the moment
it can. Clamp the integral, or stop integrating while saturated. → [08.05](../../08-control/08.05-integral-control.md)

### Symptom: a click or a current spike every time the setpoint changes

Derivative kick: the error stepped, so its derivative was momentarily enormous. Differentiate the
measurement, not the error. → [08.06](../../08-control/08.06-derivative-control.md)

### Symptom: the robot always runs a few percent slow, whatever the setpoint

1. That is a gain error, which feedforward fixes and feedback only partly compensates. Identify the
   motor's steady-state gain from a step response and use it. → [08.09](../../08-control/08.09-feedforward-exact-speed.md), [08.02](../../08-control/08.02-motor-step-response.md)
2. "Accurate when the battery is full and slow when it is low" is the same thing, plus the voltage
   dependency. Scale the feedforward by the measured pack voltage. → [08.09](../../08-control/08.09-feedforward-exact-speed.md), [01.14](../../01-first-robot/01.14-battery-monitoring.md)

### Symptom: the turn overshoots, or the last degree takes forever

1. Overshoot with a trapezoidal profile means the deceleration phase starts too late — check the
   profile against the actual angular velocity, not the commanded one. → [08.11](../../08-control/08.11-rotate-exactly-90.md)
2. "The last degree takes forever" is the deadband again: the correction is too small to move the
   wheel. Accept a tolerance band and stop, or add a minimum effective command. → [08.11](../../08-control/08.11-rotate-exactly-90.md),
   [08.02](../../08-control/08.02-motor-step-response.md)
3. "Drifts a little further after finished" is momentum plus a coast-mode stop. Brake instead. →
   [02.03](../../02-robot-electronics/02.03-motor-drivers-in-depth.md), [08.11](../../08-control/08.11-rotate-exactly-90.md)
4. "The robot turns the long way round" is angle wrapping. → [05.02](../../05-frames-and-transforms/05.02-pose-in-2d.md)

### Symptom: the numbers are right in simulation and wrong on the robot

The list of things the simulator does not have, in the order they usually matter: motor deadband,
latency, wheel slip, battery sag, encoder quantization, gearbox backlash, floor texture. Measure the
gap rather than re-tuning blind. → [06.10](../../06-simulation/06.10-sim-to-real-gap.md), [06.09](../../06-simulation/06.09-same-code-sim-and-real.md)

### Symptom: the robot stops after half a second, repeatedly

Watchdog. → [Serial link to the Pico](serial-and-pico.md), [01.10](../../01-first-robot/01.10-pi-pico-protocol.md), [08.12](../../08-control/08.12-control-in-ros2.md)

## Where to go next

- Building PID experimentally, one term at a time: [08.04](../../08-control/08.04-proportional-control.md), [08.05](../../08-control/08.05-integral-control.md), [08.06](../../08-control/08.06-derivative-control.md), [08.07](../../08-control/08.07-pid-mathematics.md)
- Tuning by logging rather than by feel: [08.08](../../08-control/08.08-tuning-pid.md)
- Odometry calibration with UMBmark: [09.05](../../09-odometry/09.05-calibrating-odometry.md)
- How uncertainty accumulates, and why drift is unavoidable: [09.06](../../09-odometry/09.06-accumulated-error.md)
- Control inside ROS 2 (`ros2_control`, rates, limits): [08.12](../../08-control/08.12-control-in-ros2.md)
