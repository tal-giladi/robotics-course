# 10.08 — Particle filters and Monte Carlo localization

Lesson: [10.08 Particle filters and Monte Carlo localization](../../../10-localization/10.08-particle-filter-mcl.md)

Localize karmel in the simulated apartment **without** a Gaussian: a cloud of weighted pose
hypotheses, moved by sampling the odometry model and reweighted by how well each one explains the
LiDAR scan. This is the algorithm inside Nav2's AMCL ([10.09](../../../10-localization/10.09-amcl-in-ros2.md)).

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `low_variance_resample(weights, u0)` | systematic resampling from ONE random number; `floor(Nw)`/`ceil(Nw)` copies per particle |
| `effective_sample_size(weights)` | `1 / sum(w²)`: how many particles are actually doing work |
| `sample_motion(particles, u, b, k, rng)` | every particle draws its own wheel travels, then the 10.06 midpoint model |
| `scan_endpoints(particles, ranges, angles)` | `(N, K, 2)` world points, vectorised |
| `likelihood_field_weights(...)` | AMCL's default sensor model, in log space so 60 beams don't underflow |
| `beam_weights(...)` | the ray-casting model: one `world.raycast` call for all N×K rays |
| `initialize_uniform / initialize_gaussian` | global localization vs pose tracking |
| `ParticleFilter` | `.predict`, `.update`, `.resample` (ESS-gated), `.estimate` (circular mean), `.covariance`, `.inject_random` |

Provided: `mcl_sim.py` — `build_likelihood_field(world)` (the obstacle-distance grid, precomputed
once), `subsample_scan(scan, beams)`, and `drive_tour` / `wheel_travels` re-exported from
[10.06](../10.06/tour_sim.py).

## Check

```bash
python course.py check 10.08
python course.py check 10.08 --solution
```

The tests pin down the resampling properties (identity on equal weights, sorted indices, the
floor/ceil copy count, zero-weight particles never surviving, and measurably lower variance than
`numpy`'s multinomial sampler), the motion model's mean and spread, both sensor models ranking the
true pose first, and the circular mean. Then three end-to-end runs on the 50-second apartment tour:
tracking from a known start (mean error < 10 cm, at least 5× better than dead reckoning), **global
localization from 3000 uniform particles** (converges in under 10 s, settles below 8 cm), and a
kidnapping at t = 20 s that only the 5 % random-injection filter recovers from.

## Hints

* `np.searchsorted(cumsum, positions, side="right")` is the whole of `low_variance_resample`.
* Weights are products over 60 beams: `exp(sum(log p) - max)` or you will get a cloud of zeros.
* `world.raycast` takes one origin per ray — `np.repeat` the particle positions K times and
  `np.tile`/broadcast the angles, one call, then reshape to `(N, K)`.
* `estimate()` must use `atan2(Σw sin θ, Σw cos θ)`. Averaging +179° and −179° must give 180°, not 0.
* If the filter tracks but then suddenly jumps: you are probably resampling every step and losing
  diversity. That is what the ESS test is for.
