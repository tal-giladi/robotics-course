# 08.05 — Integral control: kill the steady-state error, then stop windup

Lesson: [08.05 Integral control — killing steady-state error (and integral windup)](../../../08-control/08.05-integral-control.md)

Add memory to your controller so it reaches the setpoint exactly, then make sure that memory can't
fill up with garbage while the motor is saturated.

## What to implement (`student.py`)

| Method | Does |
|---|---|
| `PIController.update(setpoint, measured, dt)` | `kp * e + integral`, clamped; `integral += ki * e * dt` (stored already multiplied by ki, like the Pico firmware); with `anti_windup=True`: conditional integration, then clamp the integral to the output range |
| `PIController.reset()` | clears `integral` and `output` |

The formulation is the one in `labs/firmware/pico/velocity.py`, so the same code runs on the Pico in 08.12.

## Check

```bash
python course.py check 08.05
python course.py check 08.05 --solution
```

Tests: hand-computed P + integral steps; changing `ki` mid-run must not make the output jump;
no integration while saturated in the error's direction; the integral clamped to ±1; unwinding
allowed while saturated; with `anti_windup=False` the integral must run away (to demonstrate the bug).
Then the realistic simulator, kp = 0.1, ki = 2.0 at 50 Hz:

* holding 8 rad/s: mean speed over the last second within **±0.1 rad/s** (P alone gives ~4.6);
* 25 rad/s (unreachable) for 2 s, then 6 rad/s: with anti-windup the wheel is within ±0.5 rad/s of
  6 for good in **under 0.6 s**; without anti-windup it takes more than **twice** as long.

## Hints

* Compute `candidate = integral + ki * error * dt` first, then decide whether to keep it.
* "Saturated in the error's direction" = unclamped output above the max **and** error positive (or below the min and error negative).
* Clamp the output **after** deciding about the integral.
