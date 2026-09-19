# 07.01 — Characterize a distance sensor from a static test

Lesson: [07.01 Sensor fundamentals: accuracy, precision, noise, bias, latency, rate](../../../07-sensors/07.01-sensor-fundamentals.md)

`range_samples.csv` is a static test of a ToF-like distance sensor: a flat target at six
tape-measured distances (0.2 m to 3 m), 150 readings each. Some readings are missing (dropouts),
some are wild (outliers), and the sensor has a bias that grows with distance. Your job: turn the
readings into numbers you can trust and a calibration line that removes the bias.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `load_samples(path, sensor=None)` | CSV → `{true_m: [reading or None, …]}` |
| `is_outlier(values, k=3.5)` | robust outlier flags with the median and the MAD |
| `summarize(samples, true_m)` | counts, dropout and outlier rate, mean, σ and bias of the inliers |
| `fit_calibration_line(true_m, mean_measured_m)` | least-squares `measured = gain · true + offset` |
| `correct_reading(measured_m, gain, offset_m)` | invert the line |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 07.01              # your code
python course.py check 07.01 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (the arithmetic is in the comments), then run your code on
`range_samples.csv` and compare with the hidden truth that generated it (`make_samples.py`): bias
within 1.5 mm at every distance, σ within 25 %, the calibration gain within 0.004 and the offset within
3 mm, and calibrated means within 3 mm of the tape measure.

## Hints

* Compute the median and the MAD on the **valid** readings only (drop `None` first).
* `statistics.stdev` and `np.std(x, ddof=1)` are the sample standard deviation; `np.std(x)` is not.
* Fit the line through the per-distance **means of inliers**, not through all 900 raw readings: the
  outliers would drag it.
