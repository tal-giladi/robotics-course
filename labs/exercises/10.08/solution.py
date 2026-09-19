"""10.08 — Monte Carlo localization: reference solution.

Notation: 10-localization/code/NOTATION.md. Particles are an ``(N, 3)`` array of poses
``[x, y, theta]`` in the map frame; ``w`` is an ``(N,)`` array of weights summing to 1.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from robotlab.sim import World

Array = NDArray[np.floating]


def wrap_angle(a):
    """Wrap to (-pi, pi]; works on scalars and arrays."""
    return math.pi - (math.pi - np.asarray(a)) % (2.0 * math.pi)


# --- resampling ----------------------------------------------------------------------------------
def low_variance_resample(weights: ArrayLike, u0: float) -> NDArray[np.int64]:
    """Systematic ("low-variance") resampling: one random number, N evenly spaced comb teeth.

    Tooth j sits at (u0 + j) / N on the cumulative weight axis, 0 <= u0 < 1, and selects the
    first particle whose cumulative weight EXCEEDS that position (so a zero-weight particle can
    never be selected). A particle of weight w therefore gets exactly floor(N w) or ceil(N w) copies.
    """
    w = np.asarray(weights, dtype=float).reshape(-1)
    n = w.size
    if not 0.0 <= u0 < 1.0:
        raise ValueError(f"u0 must be in [0, 1), got {u0}")
    positions = (u0 + np.arange(n)) / n
    return np.searchsorted(np.cumsum(w), positions, side="right").clip(0, n - 1).astype(np.int64)


def effective_sample_size(weights: ArrayLike) -> float:
    """ESS = 1 / sum(w^2): N when all weights are equal, 1 when one particle holds everything."""
    w = np.asarray(weights, dtype=float).reshape(-1)
    return float(1.0 / np.sum(w * w))


# --- motion --------------------------------------------------------------------------------------
def sample_motion(particles: ArrayLike, u: ArrayLike, b: float, k: float,
                  rng: np.random.Generator) -> Array:
    """Push every particle through the odometry model with ITS OWN sampled wheel travels.

    Each particle draws d_L ~ N(dL, k^2|dL|) and d_R ~ N(dR, k^2|dR|), then applies the same
    midpoint model as 10.06. No Jacobians, no covariance: the spread of the cloud IS the covariance.
    """
    p = np.asarray(particles, dtype=float).reshape(-1, 3)
    dl, dr = (float(v) for v in np.asarray(u, dtype=float).reshape(2))
    n = p.shape[0]
    dls = dl + rng.normal(0.0, k * math.sqrt(abs(dl)), n)
    drs = dr + rng.normal(0.0, k * math.sqrt(abs(dr)), n)
    ds, dth = 0.5 * (dls + drs), (drs - dls) / b
    th_m = p[:, 2] + 0.5 * dth
    return np.column_stack([p[:, 0] + ds * np.cos(th_m),
                            p[:, 1] + ds * np.sin(th_m),
                            wrap_angle(p[:, 2] + dth)])


# --- measurement ---------------------------------------------------------------------------------
def scan_endpoints(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike) -> Array:
    """(N, K, 2) world points each beam would hit if the robot were at each particle."""
    p = np.asarray(particles, dtype=float).reshape(-1, 3)
    r = np.asarray(ranges, dtype=float).reshape(-1)
    a = np.asarray(angles, dtype=float).reshape(-1)
    world_angles = p[:, 2, None] + a[None, :]
    return np.stack([p[:, 0, None] + r[None, :] * np.cos(world_angles),
                     p[:, 1, None] + r[None, :] * np.sin(world_angles)], axis=-1)


def likelihood_field_weights(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike, field,
                             sigma_hit: float = 0.2, z_hit: float = 0.5, z_rand: float = 0.5,
                             max_range: float = 12.0) -> Array:
    """Normalised weights from the likelihood-field model (the one AMCL uses by default).

    For each beam: project the measured endpoint into the map, look up its distance ``d`` to the
    nearest obstacle, and score it
        p = z_hit * exp(-d^2 / (2 sigma_hit^2)) + z_rand / max_range.
    Beams are assumed independent, so the particle's weight is the product — computed as a sum of
    logs, with the maximum subtracted before exponentiating, or 60 beams will underflow to zero.
    """
    pts = scan_endpoints(particles, ranges, angles)
    n, kbeams = pts.shape[0], pts.shape[1]
    d = np.asarray(field.query(pts.reshape(-1, 2))).reshape(n, kbeams)
    p = z_hit * np.exp(-0.5 * (d / sigma_hit) ** 2) + z_rand / max_range
    log_w = np.log(p).sum(axis=1)
    log_w -= log_w.max()
    w = np.exp(log_w)
    return w / w.sum()


def beam_weights(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike, world: World,
                 sigma_hit: float = 0.2, z_rand: float = 0.05, max_range: float = 12.0) -> Array:
    """Normalised weights from the beam model: ray cast from EVERY particle and compare ranges.

    ``world.raycast`` takes one origin per ray, so all N x K rays go in a single vectorised call.
    Physically the most faithful model, and 100x slower than the likelihood field.
    """
    p = np.asarray(particles, dtype=float).reshape(-1, 3)
    r = np.asarray(ranges, dtype=float).reshape(-1)
    a = np.asarray(angles, dtype=float).reshape(-1)
    n, kbeams = p.shape[0], r.size
    origins = np.repeat(p[:, :2], kbeams, axis=0)
    ray_angles = (p[:, 2, None] + a[None, :]).reshape(-1)
    expected = world.raycast(origins, ray_angles, max_range).reshape(n, kbeams)
    expected = np.where(np.isfinite(expected), expected, max_range)
    err = r[None, :] - expected
    prob = (1.0 - z_rand) * np.exp(-0.5 * (err / sigma_hit) ** 2) + z_rand / max_range
    log_w = np.log(prob).sum(axis=1)
    log_w -= log_w.max()
    w = np.exp(log_w)
    return w / w.sum()


# --- initialisation ------------------------------------------------------------------------------
def initialize_uniform(world: World, n: int, rng: np.random.Generator, clearance: float = 0.12) -> Array:
    """``n`` particles spread over every free pose of the map: global localization, no prior."""
    x0, x1, y0, y1 = world.bounds
    out = np.empty((0, 3))
    while out.shape[0] < n:
        batch = np.column_stack([rng.uniform(x0, x1, 4 * n), rng.uniform(y0, y1, 4 * n),
                                 rng.uniform(-math.pi, math.pi, 4 * n)])
        free = world.distance_to_obstacles(batch[:, :2]) > clearance
        out = np.vstack([out, batch[free]])
    return out[:n]


def initialize_gaussian(pose: ArrayLike, std: ArrayLike, n: int, rng: np.random.Generator) -> Array:
    """``n`` particles around a known pose: pose tracking, the AMCL "2D Pose Estimate" case."""
    mu = np.asarray(pose, dtype=float).reshape(3)
    sd = np.asarray(std, dtype=float).reshape(3)
    p = mu + rng.normal(0.0, 1.0, (n, 3)) * sd
    p[:, 2] = wrap_angle(p[:, 2])
    return p


# --- the filter ----------------------------------------------------------------------------------
class ParticleFilter:
    """Monte Carlo localization. ``particles`` (N, 3), ``weights`` (N,) summing to 1."""

    def __init__(self, particles: ArrayLike, weights: ArrayLike | None = None) -> None:
        self.particles = np.asarray(particles, dtype=float).reshape(-1, 3).copy()
        n = self.particles.shape[0]
        if weights is None:
            self.weights = np.full(n, 1.0 / n)
        else:
            w = np.asarray(weights, dtype=float).reshape(-1).copy()
            if w.size != n:
                raise ValueError(f"{w.size} weights for {n} particles")
            self.weights = w / w.sum()
        self.resampled = 0

    @property
    def n(self) -> int:
        return self.particles.shape[0]

    def predict(self, u, b: float, k: float, rng: np.random.Generator) -> None:
        """Sample every particle forward. Weights are unchanged (the proposal IS the motion model)."""
        self.particles = sample_motion(self.particles, u, b, k, rng)

    def update(self, ranges, angles, field, **kwargs) -> float:
        """Reweight with the likelihood field and return the effective sample size."""
        if np.size(ranges) == 0:
            return effective_sample_size(self.weights)
        w = self.weights * likelihood_field_weights(self.particles, ranges, angles, field, **kwargs)
        total = w.sum()
        if not np.isfinite(total) or total <= 0.0:  # every particle explained the scan equally badly
            self.weights = np.full(self.n, 1.0 / self.n)
        else:
            self.weights = w / total
        return effective_sample_size(self.weights)

    def resample(self, rng: np.random.Generator, ess_ratio: float = 0.5) -> bool:
        """Resample only when ESS falls below ``ess_ratio * N``. Returns whether it resampled.

        Resampling every step throws away diversity for nothing; resampling never lets one particle
        take all the weight. The ESS test is the standard compromise.
        """
        if effective_sample_size(self.weights) >= ess_ratio * self.n:
            return False
        idx = low_variance_resample(self.weights, float(rng.uniform(0.0, 1.0)))
        self.particles = self.particles[idx]
        self.weights = np.full(self.n, 1.0 / self.n)
        self.resampled += 1
        return True

    def estimate(self) -> Array:
        """Weighted mean pose, with a CIRCULAR mean for the heading."""
        x = float(self.weights @ self.particles[:, 0])
        y = float(self.weights @ self.particles[:, 1])
        s = float(self.weights @ np.sin(self.particles[:, 2]))
        c = float(self.weights @ np.cos(self.particles[:, 2]))
        return np.array([x, y, math.atan2(s, c)])

    def covariance(self) -> Array:
        """Weighted 2x2 position covariance of the cloud — AMCL's /amcl_pose covariance."""
        mu = self.estimate()[:2]
        d = self.particles[:, :2] - mu
        return (self.weights[:, None] * d).T @ d

    def inject_random(self, world: World, fraction: float, rng: np.random.Generator) -> int:
        """Replace the ``fraction`` lowest-weight particles with uniform ones: the "A" in AMCL's
        recovery. Costs accuracy while localized, and is the only way back from a kidnapping."""
        m = int(round(fraction * self.n))
        if m <= 0:
            return 0
        worst = np.argsort(self.weights)[:m]
        self.particles[worst] = initialize_uniform(world, m, rng)
        self.weights[worst] = self.weights.mean()
        self.weights /= self.weights.sum()
        return m
