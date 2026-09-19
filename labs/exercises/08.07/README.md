# 08.07 — Step-response metrics, and PI gains from the motor model

Lesson: [08.07 PID mathematically — continuous, discrete, and what stability means](../../../08-control/08.07-pid-mathematics.md)

Two things every later lesson uses: a function that turns a logged step response into the four
numbers people actually argue about (rise, overshoot, settling, steady-state error), and two ways
of computing PI gains from the first-order motor model of [08.02](../../../08-control/08.02-motor-step-response.md)
instead of guessing them.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `step_metrics(t, y, t_step, y_start, setpoint, settle_band, final_window_s)` | `StepMetrics(rise_time, peak_time, overshoot_percent, settling_time, steady_state_error, final_value)` from a logged response. Normalise by the step so downward steps work. Interpolate the 10 % and 90 % crossings between samples. The settling band is around the **setpoint**, so a loop with steady-state error never settles (`inf`). Raise `ValueError` when `setpoint == y_start`. |
| `pi_gains_pole_placement(K, tau, zeta, omega_n)` | `(kp, ki)` that put the closed-loop poles of PI + `K/(tau s + 1)` at damping `zeta` and natural frequency `omega_n`. Characteristic polynomial: `tau s^2 + (1 + K kp) s + K ki`. |
| `pi_gains_lambda(K, tau, tau_cl)` | `(kp, ki)` that cancel the motor pole with the PI zero (`ki/kp = 1/tau`) and leave a first-order closed loop with time constant `tau_cl`. |

## Check

```bash
python course.py check 08.07
python course.py check 08.07 --solution
```

Tests:

* a six-sample series computed by hand (rise 1.467 s, overshoot 10 %, settling 4.0 s);
* an exact first-order response: rise must come out as `tau ln 9`, 2 % settling as `tau ln 50`;
* an exact second-order response with `zeta = 0.5`: overshoot 16.3 %, peak time `pi / omega_d`;
* a downward step (8 → 2 rad/s) — the normalisation is what makes this work;
* a P-only response stuck below the setpoint: rise and settling are both `inf`;
* pole placement by hand (`K` 20, `tau` 0.1, `zeta` 1, `omega_n` 20 → kp 0.15, ki 2.0), and a check
  that the poles really land where you asked;
* lambda tuning by hand, then end to end: design gains for `tau_cl = 100 ms`, run them on the
  realistic simulator and measure zero steady-state error, under 10 % overshoot and a rise time
  near `2.2 tau_cl`.

## Hints

* Work on the normalised response `n = (y - y_start) / (setpoint - y_start)`: 0 at the start, 1 at
  the setpoint, for both directions.
* "Settled" means *stays* inside the band: find the **last** sample outside it, not the first inside.
* Both gain formulas are two lines each. Write the characteristic polynomial down first and match
  coefficients — that is all pole placement is.
