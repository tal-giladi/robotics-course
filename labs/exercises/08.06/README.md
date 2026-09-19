# 08.06 — A full PID: derivative on the measurement, filtered

Lesson: [08.06 Derivative control — damping, noise and derivative kick](../../../08-control/08.06-derivative-control.md)

Extend your 08.05 PI controller with a derivative term that doesn't kick when the setpoint jumps
and doesn't turn measurement noise into motor chatter.

## What to implement (`student.py`)

| Method | Does |
|---|---|
| `PIDController.update(setpoint, measured, dt)` | `kp * e + integral + d_term`, clamped. `d_term = -kd * rate`, where `rate` is d(measured)/dt passed through a first-order low-pass with `alpha = dt / (derivative_filter_tau_s + dt)`. No derivative on the very first update. Integral and anti-windup exactly as in 08.05, with `d_term` included in the unclamped output. |
| `PIDController.reset()` | clears the integral, the filtered rate, the last measurement, `d_term` and `output` |

## Check

```bash
python course.py check 08.06
python course.py check 08.06 --solution
```

Tests:

* P and I behave as in 08.05; anti-windup is still there;
* D sign and size by hand (a measurement rising 1 rad/s in 10 ms with kd = 0.1 gives d_term = −10);
* the filter formula by hand (tau = dt → alpha = 0.5);
* **no derivative kick**: a setpoint step with a constant measurement changes the output by exactly the P and I contributions;
* a filtered derivative of a ramp settles to −kd × slope within 1%;
* **noise attenuation**: with white measurement noise (σ = 0.1 rad/s, dt = 20 ms, kd = 0.002) the unfiltered D output has σ ≈ 0.0141 and a 40 ms filter at least halves it;
* the realistic simulator with 20 ms of extra delay: PI(0.1, 2.0) overshoots past 9.6 rad/s on a 0 → 8 step; your PID with kd = 0.002 and a 10 ms filter stays below 9.2 and settles at 8 ± 0.1.

## Hints

* Remember the last *measurement*, not the last error.
* `tau = 0` must mean "no filtering": `alpha = dt / (0 + dt) = 1`.
* Keep the filtered rate between calls; `d_term` is recomputed from it every update.
