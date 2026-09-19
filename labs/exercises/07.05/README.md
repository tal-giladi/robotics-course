# 07.05 — Gyro bias and heading drift

Lesson: [07.05 IMU — accelerometer, gyroscope, magnetometer, bias and drift](../../../07-sensors/07.05-imu-fundamentals.md)

A gyroscope reports the turn rate plus a bias. Integrate it and the bias becomes a heading error that
grows every second. Measure the bias while the robot stands still, subtract it, and the same gyro
holds heading to within a degree over a sequence of turns. You build exactly that, then run it on a
log from the course simulator.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `heading_drift_deg(bias_rad_s, seconds)` | how wrong an uncorrected heading gets |
| `seconds_until_error(bias_rad_s, max_error_deg)` | how long you can trust it |
| `estimate_gyro_bias(rates)` | mean of still samples |
| `detect_still(rates, window, threshold_rad_s)` | find the still samples from the gyro alone (range test) |
| `bias_from_still(rates, still)` | bias from the flagged samples |
| `integrate_heading(times, rates, bias, initial_heading)` | trapezoidal integration, wrapped output |

## Check

```bash
python course.py check 07.05              # your code
python course.py check 07.05 --solution   # the reference
```

Hand-computed cases first. Then the simulator: karmel (realistic preset, gyro bias 0.01–0.02 rad/s,
noise 0.005 rad/s) stands still for 4 s and turns four times by about 90°. With your bias
correction the final heading must be within **1°** of the truth; without it the error is 7–16°.

## Hints

* A still gyro doesn't read zero: it reads the bias. Test the **spread** in a window, not the value.
* Accumulate the heading unwrapped and wrap only what you return, or ten turns become one.
* Use each sample's own time step; don't assume a constant `dt`.
