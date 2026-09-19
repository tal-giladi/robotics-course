"""10.08 — Monte Carlo localization with a particle filter.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 10.08``.
Only numpy, math and ``robotlab.sim.World`` are needed.

Notation (10-localization/code/NOTATION.md):

    particles   (N, 3) array of poses [x, y, theta] in the map frame, theta wrapped to (-pi, pi]
    weights     (N,)   non-negative, summing to 1
    u           [d_left, d_right] wheel travels since the last predict (m), as in 10.06
    b           wheel separation (m);  k  the wheel-noise constant (m^0.5), as in 10.06
    ranges, angles   one LiDAR scan, already subsampled to valid beams (mcl_sim.subsample_scan)
    field       a LikelihoodField from mcl_sim.build_likelihood_field: field.query(points) -> (N,)
                distance from each world point to the nearest obstacle, clipped at field.max_dist

The LiDAR sits at the robot's origin (karmel.yaml: lidar x = y = 0), so a particle's pose is also
its laser pose — no extra transform.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from robotlab.sim import World

Array = NDArray[np.floating]


def wrap_angle(a):
    """Wrap to (-pi, pi]; works on scalars and arrays. Provided."""
    return math.pi - (math.pi - np.asarray(a)) % (2.0 * math.pi)


# --- resampling ----------------------------------------------------------------------------------
def low_variance_resample(weights: ArrayLike, u0: float) -> NDArray[np.int64]:
    """Systematic ("low-variance") resampling. Return the (N,) array of chosen particle indices.

    Draw ONE number u0 in [0, 1) and lay N evenly spaced comb teeth on the cumulative-weight axis
    at positions (u0 + j) / N for j = 0..N-1. Tooth j selects the first particle whose cumulative
    weight EXCEEDS its position (use ``side="right"``, so a zero-weight particle is never picked).

    Raise ValueError if u0 is outside [0, 1). Hint: np.cumsum + np.searchsorted, no Python loop.
    """
    # TODO(student)
    raise NotImplementedError("low_variance_resample")


def effective_sample_size(weights: ArrayLike) -> float:
    """Return ESS = 1 / sum(w^2): N when the weights are equal, 1 when one particle has all of it."""
    # TODO(student)
    raise NotImplementedError("effective_sample_size")


# --- motion --------------------------------------------------------------------------------------
def sample_motion(particles: ArrayLike, u: ArrayLike, b: float, k: float,
                  rng: np.random.Generator) -> Array:
    """Move every particle through the odometry model with ITS OWN sampled wheel travels.

    Particle i draws  d_L ~ N(dL, k^2 |dL|)  and  d_R ~ N(dR, k^2 |dR|)  — the same noise model as
    10.06's ``wheel_noise`` — then applies the midpoint model:

        ds = (dL + dR)/2,  dth = (dR - dL)/b,  th_m = theta + dth/2
        x' = x + ds cos(th_m),  y' = y + ds sin(th_m),  theta' = wrap(theta + dth)

    Return the new (N, 3) array. Vectorised: draw all N pairs at once, no loop.
    """
    # TODO(student)
    raise NotImplementedError("sample_motion")


# --- measurement ---------------------------------------------------------------------------------
def scan_endpoints(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike) -> Array:
    """Return the (N, K, 2) world points each of the K beams would hit, seen from each of the
    N particles: endpoint = (x + r cos(theta + a), y + r sin(theta + a))."""
    # TODO(student)
    raise NotImplementedError("scan_endpoints")


def likelihood_field_weights(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike, field,
                             sigma_hit: float = 0.2, z_hit: float = 0.5, z_rand: float = 0.5,
                             max_range: float = 12.0) -> Array:
    """Return normalised (N,) weights from the likelihood-field model — what AMCL uses by default.

    For each beam of each particle: look the endpoint's obstacle distance d up in ``field``, then

        p = z_hit * exp(-d^2 / (2 sigma_hit^2)) + z_rand / max_range

    Beams are treated as independent, so a particle's weight is the PRODUCT over beams. With 60
    beams that product underflows to 0.0 in float64: sum the logs instead, subtract the maximum,
    then exponentiate and normalise.
    """
    # TODO(student)
    raise NotImplementedError("likelihood_field_weights")


def beam_weights(particles: ArrayLike, ranges: ArrayLike, angles: ArrayLike, world: World,
                 sigma_hit: float = 0.2, z_rand: float = 0.05, max_range: float = 12.0) -> Array:
    """Return normalised (N,) weights from the beam model: ray cast from every particle.

    ``world.raycast(origins, angles, max_range)`` accepts ONE ORIGIN PER RAY, so build
    ``origins`` of shape (N*K, 2) and ``angles`` of shape (N*K,) and make a single call; reshape the
    result to (N, K). Treat a non-finite expected range as ``max_range``. Score each beam with

        p = (1 - z_rand) * exp(-(r_measured - r_expected)^2 / (2 sigma_hit^2)) + z_rand / max_range

    and combine over beams in log space, as above.
    """
    # TODO(student)
    raise NotImplementedError("beam_weights")


# --- initialisation ------------------------------------------------------------------------------
def initialize_uniform(world: World, n: int, rng: np.random.Generator, clearance: float = 0.12) -> Array:
    """Return (n, 3) particles spread over the FREE poses of the map (global localization).

    Sample x, y inside ``world.bounds`` and theta in (-pi, pi], reject any position with
    ``world.distance_to_obstacles`` <= clearance, and keep sampling until you have n.
    """
    # TODO(student)
    raise NotImplementedError("initialize_uniform")


def initialize_gaussian(pose: ArrayLike, std: ArrayLike, n: int, rng: np.random.Generator) -> Array:
    """Return (n, 3) particles drawn around ``pose`` with per-axis standard deviations ``std``
    (pose tracking — AMCL's "2D Pose Estimate"). Wrap the headings."""
    # TODO(student)
    raise NotImplementedError("initialize_gaussian")


# --- the filter ----------------------------------------------------------------------------------
class ParticleFilter:
    """Monte Carlo localization: ``self.particles`` (N, 3), ``self.weights`` (N,) summing to 1."""

    def __init__(self, particles: ArrayLike, weights: ArrayLike | None = None) -> None:
        # TODO(student): store float COPIES; weights default to 1/N and are normalised otherwise;
        #   raise ValueError if len(weights) != len(particles); set self.resampled = 0.
        raise NotImplementedError("ParticleFilter.__init__")

    @property
    def n(self) -> int:
        """Number of particles."""
        return self.particles.shape[0]

    def predict(self, u, b: float, k: float, rng: np.random.Generator) -> None:
        """Sample every particle forward with ``sample_motion``. Weights are NOT touched."""
        # TODO(student)
        raise NotImplementedError("ParticleFilter.predict")

    def update(self, ranges, angles, field, **kwargs) -> float:
        """Multiply the weights by ``likelihood_field_weights``, renormalise, return the new ESS.

        Two edge cases the tests check: an empty scan changes nothing, and if the weights sum to
        zero or a non-finite value (every particle explained the scan equally badly) reset them to
        uniform rather than producing NaNs.
        """
        # TODO(student)
        raise NotImplementedError("ParticleFilter.update")

    def resample(self, rng: np.random.Generator, ess_ratio: float = 0.5) -> bool:
        """Resample with ``low_variance_resample`` ONLY when ESS < ess_ratio * N.

        Return True if it resampled. After resampling, weights are uniform again and
        ``self.resampled`` is incremented. Draw u0 from ``rng.uniform(0.0, 1.0)``.
        """
        # TODO(student)
        raise NotImplementedError("ParticleFilter.resample")

    def estimate(self) -> Array:
        """Weighted mean pose (3,). The heading needs a CIRCULAR mean:
        atan2(sum w sin(theta), sum w cos(theta)) — averaging +179 and -179 must not give 0."""
        # TODO(student)
        raise NotImplementedError("ParticleFilter.estimate")

    def covariance(self) -> Array:
        """Weighted 2x2 position covariance of the cloud (what AMCL reports in /amcl_pose)."""
        # TODO(student)
        raise NotImplementedError("ParticleFilter.covariance")

    def inject_random(self, world: World, fraction: float, rng: np.random.Generator) -> int:
        """Replace the ``fraction`` LOWEST-weight particles with uniform ones and renormalise;
        return how many were replaced (0 if fraction rounds to 0). This is the recovery that lets
        the filter come back from a kidnapping."""
        # TODO(student)
        raise NotImplementedError("ParticleFilter.inject_random")
