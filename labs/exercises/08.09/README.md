# 08.09 — Feedforward + PI: drive at exactly 0.5 m/s

Lesson: [08.09 Feedforward + PID: drive at exactly 0.5 m/s](../../../08-control/08.09-feedforward-exact-speed.md)

Stop making feedback do the whole job. Invert the motor model you measured in
[08.02](../../../08-control/08.02-motor-step-response.md) to guess the duty *before* any error
exists, and leave the PI only the part the model got wrong.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `wheel_speed_for(ground_speed_m_s, wheel_radius_m)` | `v = r w`, solved for `w` |
| `FeedforwardModel.top_speed(battery_v)` | `max_wheel_speed_rad_s * battery_v / nominal_v`, or the nominal value when `use_battery` is False or no reading is available |
| `FeedforwardModel.duty(setpoint_rad_s, battery_v)` | `sign(w) * (deadband + |w| / top_speed * (1 - deadband))`, clamped to ±1; **exactly 0 for a setpoint of exactly 0** |
| `FeedforwardPI.update(setpoint, measured, dt, battery_v)` | `feedforward + kp*e + integral`, clamped, with the 08.05 anti-windup (integral in duty units, fill the headroom, then clamp). The feedforward counts in the anti-windup check. |
| `FeedforwardPI.reset()` | clears integral, feedforward and output |

## Check

```bash
python course.py check 08.09
python course.py check 08.09 --solution
```

Tests:

* the duty by hand: 11.111 rad/s → 0.695 at 10.8 V, 0.613 at 12.6 V, 0.748 at 9.9 V;
* a zero setpoint sends zero duty (a deadband duty would make the robot creep);
* `use_battery=False` ignores the voltage, and no reading means "assume nominal";
* every term of the output visible on one non-saturating step, and the integral refusing to wind up
  when the setpoint is out of reach;
* on the realistic simulator (the **right** wheel, whose motor is 6 % weaker than the model):
  feedforward alone settles a few per cent low, feedforward + PI settles at 0.5 m/s within 2 % and
  is above 95 % of the setpoint within 0.4 s;
* on a 15 %-charged battery: ignoring the voltage loses more than 7 % of speed, using the measured
  voltage gets closer, and the integral closes the rest.

## Hints

* The model is a straight line with an offset. Feedforward is that line read backwards.
* Put the feedforward into `others` *before* the anti-windup test, or the integral will wind up
  while the feedforward alone is already saturating the motor.
* Don't be clever about `setpoint == 0`: test it explicitly and return 0.
