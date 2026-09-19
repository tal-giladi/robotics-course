"""Checker for 10.08 — Particle filters and Monte Carlo localization.

Run: ``python course.py check 10.08`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from robotlab.config import load_config
from robotlab.sim import World

approx = pytest.approx
HERE = Path(__file__).resolve().parent


def _mcl_sim():
    """Import the provided ``mcl_sim.py`` next to this test (pytest runs in importlib mode)."""
    name = "course_exercise_10_08_mcl_sim"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "mcl_sim.py")
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


build_likelihood_field = _mcl_sim().build_likelihood_field
drive_tour = _mcl_sim().drive_tour
subsample_scan = _mcl_sim().subsample_scan
wheel_travels = _mcl_sim().wheel_travels


# --- low_variance_resample -----------------------------------------------------------------------
def test_uniform_weights_with_u0_zero_is_the_identity(impl):
    idx = impl.low_variance_resample(np.full(8, 1 / 8), 0.0)
    assert list(idx) == list(range(8)), "systematic resampling of equal weights must keep every particle"


def test_indices_are_sorted_and_the_right_length(impl):
    w = np.array([0.4, 0.1, 0.3, 0.15, 0.05])
    idx = impl.low_variance_resample(w, 0.37)
    assert idx.shape == (5,)
    assert list(idx) == sorted(idx), "the comb sweeps the cumulative axis once, so indices come out sorted"
    assert idx.min() >= 0 and idx.max() < 5


def test_u0_must_be_in_the_unit_interval(impl):
    with pytest.raises(ValueError):
        impl.low_variance_resample(np.full(4, 0.25), 1.0)
    with pytest.raises(ValueError):
        impl.low_variance_resample(np.full(4, 0.25), -0.1)


def test_hand_computed_case(impl):
    # N = 4, w = [0.1, 0.6, 0.2, 0.1], cumsum = [0.1, 0.7, 0.9, 1.0], u0 = 0.5
    # teeth at 0.125, 0.375, 0.625, 0.875 -> particles 1, 1, 1, 2
    assert list(impl.low_variance_resample([0.1, 0.6, 0.2, 0.1], 0.5)) == [1, 1, 1, 2]


def test_copy_count_is_floor_or_ceil_of_n_times_weight(impl):
    """THE defining property: a particle of weight w gets floor(Nw) or ceil(Nw) copies, always."""
    rng = np.random.default_rng(0)
    n = 50
    for _ in range(20):
        w = rng.random(n)
        w /= w.sum()
        counts = np.bincount(impl.low_variance_resample(w, float(rng.uniform(0, 1))), minlength=n)
        lo, hi = np.floor(n * w), np.ceil(n * w)
        assert np.all(counts >= lo) and np.all(counts <= hi), "multinomial resampling would violate this"


def test_zero_weight_particles_never_survive(impl):
    w = np.array([0.0, 0.5, 0.0, 0.5])
    for u0 in np.linspace(0.0, 0.99, 25):
        counts = np.bincount(impl.low_variance_resample(w, float(u0)), minlength=4)
        assert counts[0] == 0 and counts[2] == 0


def test_lower_variance_than_multinomial(impl):
    """The name is a promise: measure it against numpy's multinomial sampler."""
    rng = np.random.default_rng(3)
    n = 200
    w = rng.random(n)
    w /= w.sum()
    lv, mn = [], []
    for _ in range(60):
        lv.append(np.bincount(impl.low_variance_resample(w, float(rng.uniform(0, 1))), minlength=n))
        mn.append(rng.multinomial(n, w))
    lv_var = float(np.mean(np.var(np.array(lv, dtype=float) - n * w, axis=0)))
    mn_var = float(np.mean(np.var(np.array(mn, dtype=float) - n * w, axis=0)))
    assert lv_var < 0.25 * mn_var, f"low-variance {lv_var:.3f} should be well under multinomial {mn_var:.3f}"


# --- effective_sample_size -----------------------------------------------------------------------
def test_ess_extremes_and_a_hand_case(impl):
    assert impl.effective_sample_size(np.full(100, 0.01)) == approx(100.0)
    w = np.zeros(100)
    w[7] = 1.0
    assert impl.effective_sample_size(w) == approx(1.0)
    # w = [0.5, 0.5, 0, 0] -> 1 / (0.25 + 0.25) = 2
    assert impl.effective_sample_size([0.5, 0.5, 0.0, 0.0]) == approx(2.0)


# --- sample_motion -------------------------------------------------------------------------------
def test_sample_motion_mean_matches_the_deterministic_model(impl):
    rng = np.random.default_rng(1)
    start = np.tile([1.0, 2.0, 0.3], (20_000, 1))
    out = impl.sample_motion(start, [0.10, 0.14], 0.2, 0.01, rng)
    assert out.shape == (20_000, 3)
    ds, dth = 0.12, 0.04 / 0.2
    th_m = 0.3 + dth / 2
    expected = [1.0 + ds * math.cos(th_m), 2.0 + ds * math.sin(th_m), 0.3 + dth]
    assert out.mean(axis=0) == approx(expected, abs=2e-3)


def test_sample_motion_spread_grows_with_k_and_distance(impl):
    rng = np.random.default_rng(2)
    start = np.zeros((20_000, 3))
    small = impl.sample_motion(start, [1.0, 1.0], 0.2, 0.01, rng)
    large = impl.sample_motion(start, [1.0, 1.0], 0.2, 0.04, rng)
    # sigma of each wheel travel is k*sqrt(d) = 0.01; dth = (dR - dL)/b -> sigma 0.01*sqrt(2)/0.2
    assert small[:, 2].std() == approx(0.01 * math.sqrt(2) / 0.2, rel=0.1)
    assert large[:, 2].std() == approx(4 * small[:, 2].std(), rel=0.1)
    assert small[:, 0].std() < 0.02, "forward travel is far more certain than heading"


def test_sample_motion_wraps_the_heading(impl):
    rng = np.random.default_rng(3)
    out = impl.sample_motion(np.tile([0.0, 0.0, 3.0], (500, 1)), [-0.05, 0.05], 0.2, 0.0, rng)
    assert np.all(out[:, 2] > -math.pi) and np.all(out[:, 2] <= math.pi)
    assert out[0, 2] == approx(3.5 - 2 * math.pi, abs=1e-9)


# --- the measurement models ----------------------------------------------------------------------
def test_scan_endpoints_by_hand(impl):
    pts = impl.scan_endpoints([[1.0, 2.0, math.pi / 2]], [2.0, 1.0], [0.0, -math.pi / 2])
    assert pts.shape == (1, 2, 2)
    assert pts[0, 0] == approx([1.0, 4.0], abs=1e-9)  # straight ahead = +y
    assert pts[0, 1] == approx([2.0, 2.0], abs=1e-9)  # 90 deg right = +x


@pytest.fixture(scope="module")
def apartment():
    return World.apartment()


@pytest.fixture(scope="module")
def field(apartment):
    return build_likelihood_field(apartment, resolution=0.05, max_dist=2.0)


@pytest.fixture(scope="module")
def one_scan():
    log = drive_tour(seed=5, observe_every=1000, scan_every=200)
    step, scan = log.scans[1]
    ranges, angles = subsample_scan(scan, 60)
    return log.truth[step], ranges, angles


@pytest.mark.parametrize("model", ["likelihood_field", "beam"])
def test_the_true_pose_gets_the_highest_weight(impl, apartment, field, one_scan, model):
    truth, ranges, angles = one_scan
    offsets = np.array([[0, 0, 0], [0.3, 0, 0], [-0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.35], [1.2, -0.8, 1.0]], dtype=float)
    particles = truth + offsets
    if model == "likelihood_field":
        w = impl.likelihood_field_weights(particles, ranges, angles, field)
    else:
        w = impl.beam_weights(particles, ranges, angles, apartment)
    assert w.sum() == approx(1.0), "weights must be normalised"
    assert np.all(np.isfinite(w)), "60 beams underflow unless you work in log space"
    assert int(np.argmax(w)) == 0, f"the true pose should win with the {model} model, got {w.round(4)}"
    assert w[0] > 100 * w[5], "a pose 1.4 m and 57 deg away should be crushed"


def test_likelihood_field_is_much_faster_than_ray_casting(impl, apartment, field, one_scan):
    """Not a benchmark — just a sanity check that both models rank the same poses the same way."""
    truth, ranges, angles = one_scan
    particles = truth + np.array([[0, 0, 0], [0.15, 0, 0], [0.5, 0.3, 0.2]], dtype=float)
    lf = impl.likelihood_field_weights(particles, ranges, angles, field)
    bm = impl.beam_weights(particles, ranges, angles, apartment)
    assert list(np.argsort(-lf)) == list(np.argsort(-bm))


# --- initialisation ------------------------------------------------------------------------------
def test_initialize_uniform_lands_in_free_space(impl, apartment):
    p = impl.initialize_uniform(apartment, 500, np.random.default_rng(0))
    assert p.shape == (500, 3)
    x0, x1, y0, y1 = apartment.bounds
    assert np.all((p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1))
    assert np.all(apartment.distance_to_obstacles(p[:, :2]) > 0.12), "particles must not start inside walls"
    assert np.all((p[:, 2] > -math.pi) & (p[:, 2] <= math.pi))
    assert p[:, 2].std() > 1.5, "headings must cover the whole circle, not a corner of it"


def test_initialize_gaussian_statistics(impl):
    p = impl.initialize_gaussian([1.0, 2.0, 0.0], [0.2, 0.2, 0.1], 20_000, np.random.default_rng(1))
    assert p.mean(axis=0) == approx([1.0, 2.0, 0.0], abs=0.01)
    assert p.std(axis=0) == approx([0.2, 0.2, 0.1], rel=0.05)


# --- ParticleFilter ------------------------------------------------------------------------------
def test_constructor(impl):
    pf = impl.ParticleFilter(np.zeros((10, 3)))
    assert pf.n == 10 and pf.weights == approx(np.full(10, 0.1))
    pf = impl.ParticleFilter(np.zeros((4, 3)), [1.0, 1.0, 2.0, 4.0])
    assert pf.weights == approx([0.125, 0.125, 0.25, 0.5]), "weights must be normalised"
    with pytest.raises(ValueError):
        impl.ParticleFilter(np.zeros((4, 3)), [1.0, 1.0])


def test_estimate_uses_a_circular_mean(impl):
    pf = impl.ParticleFilter([[0.0, 0.0, math.pi - 0.1], [2.0, 4.0, -math.pi + 0.1]])
    est = pf.estimate()
    assert est[:2] == approx([1.0, 2.0])
    assert abs(est[2]) == approx(math.pi, abs=1e-9), "averaging +179 and -179 degrees must give 180, not 0"


def test_covariance_of_a_known_cloud(impl):
    rng = np.random.default_rng(4)
    pts = rng.multivariate_normal([1.0, -2.0], [[0.04, 0.01], [0.01, 0.09]], 50_000)
    pf = impl.ParticleFilter(np.column_stack([pts, np.zeros(len(pts))]))
    assert pf.covariance() == approx(np.array([[0.04, 0.01], [0.01, 0.09]]), abs=0.005)


def test_resample_only_below_the_ess_threshold(impl):
    rng = np.random.default_rng(5)
    pf = impl.ParticleFilter(np.random.default_rng(0).normal(size=(100, 3)))
    assert pf.resample(rng) is False, "uniform weights have ESS = N; nothing to do"
    assert pf.resampled == 0
    w = np.full(100, 1e-9)
    w[:3] = 1.0
    pf = impl.ParticleFilter(np.random.default_rng(0).normal(size=(100, 3)), w)
    assert pf.resample(rng) is True
    assert pf.resampled == 1
    assert pf.weights == approx(np.full(100, 0.01)), "weights are uniform again after resampling"
    assert len(np.unique(pf.particles[:, 0])) <= 3, "only the three heavy particles should survive"


def test_update_edge_cases(impl, field, one_scan):
    truth, ranges, angles = one_scan
    pf = impl.ParticleFilter(np.tile(truth, (20, 1)))
    before = pf.weights.copy()
    assert pf.update([], [], field) == approx(20.0)
    assert pf.weights == approx(before), "an empty scan must change nothing"
    lost = impl.ParticleFilter(np.tile([0.0, 0.0, 0.0], (20, 1)))  # all particles inside a wall
    lost.update(ranges, angles, field)
    assert np.all(np.isfinite(lost.weights)) and lost.weights.sum() == approx(1.0)


def test_inject_random_replaces_the_worst(impl, apartment):
    rng = np.random.default_rng(6)
    w = np.linspace(1.0, 100.0, 100)
    pf = impl.ParticleFilter(np.tile([1.0, 1.3, 0.0], (100, 1)), w)
    assert pf.inject_random(apartment, 0.0, rng) == 0
    n = pf.inject_random(apartment, 0.1, rng)
    assert n == 10
    assert pf.weights.sum() == approx(1.0)
    moved = np.abs(pf.particles[:, 0] - 1.0) > 1e-9
    assert moved.sum() == 10 and not moved[-1], "the heaviest particle must survive"


# --- end to end ----------------------------------------------------------------------------------
def run_mcl(impl, log, field, particles, k=0.02, beams=40, seed=0, inject=0.0, world=None):
    """Drive the whole tour through the filter. Returns (estimates, position errors, filter)."""
    rng = np.random.default_rng(seed)
    b = load_config().drive.wheel_separation_m
    travels = wheel_travels(log.ticks)
    pf = impl.ParticleFilter(particles)
    scans = dict(log.scans)
    est, err = [], []
    for i in range(1, len(log.t)):
        pf.predict(travels[i - 1], b, k, rng)
        scan = scans.get(i)
        if scan is not None:
            ranges, angles = subsample_scan(scan, beams)
            pf.update(ranges, angles, field)
            if inject:
                pf.inject_random(world, inject, rng)
            pf.resample(rng)
        x = pf.estimate()
        est.append(x)
        err.append(math.hypot(x[0] - log.truth[i, 0], x[1] - log.truth[i, 1]))
    return np.array(est), np.array(err), pf


@pytest.fixture(scope="module")
def tour():
    return drive_tour(seed=4, observe_every=1000, scan_every=25)  # scans at 2 Hz


def test_tracking_from_a_known_start(impl, tour, field):
    p0 = impl.initialize_gaussian(tour.truth[0], [0.05, 0.05, 0.05], 500, np.random.default_rng(1))
    est, err, pf = run_mcl(impl, tour, field, p0, seed=1)
    assert err.mean() < 0.10, f"tracking should stay within 10 cm on average, got {err.mean():.3f} m"
    assert err.max() < 0.35, f"worst-case error {err.max():.3f} m is too large"
    assert err[-1] < 0.15
    heading_err = abs(float(impl.wrap_angle(est[-1, 2] - tour.truth[-1, 2])))
    assert heading_err < math.radians(10), f"final heading off by {math.degrees(heading_err):.1f} deg"
    assert pf.resampled > 5, "a filter that never resamples is not resampling correctly"
    assert math.sqrt(pf.covariance()[0, 0]) < 0.25, "the cloud should have collapsed around the robot"


def test_it_beats_dead_reckoning(impl, tour, field):
    p0 = impl.initialize_gaussian(tour.truth[0], [0.05, 0.05, 0.05], 500, np.random.default_rng(2))
    _, err, _ = run_mcl(impl, tour, field, p0, seed=2)
    b = load_config().drive.wheel_separation_m
    pose = np.array(tour.truth[0], dtype=float)
    dr = []
    for d in wheel_travels(tour.ticks):
        ds, dth = 0.5 * (d[0] + d[1]), (d[1] - d[0]) / b
        th = pose[2] + dth / 2
        pose = np.array([pose[0] + ds * math.cos(th), pose[1] + ds * math.sin(th), pose[2] + dth])
        dr.append(pose.copy())
    dr = np.array(dr)
    dr_err = np.hypot(dr[:, 0] - tour.truth[1:, 0], dr[:, 1] - tour.truth[1:, 1])
    assert err.mean() < 0.2 * dr_err.mean(), (
        f"MCL {err.mean():.3f} m should beat dead reckoning {dr_err.mean():.3f} m by at least 5x")


def test_global_localization_converges(impl, tour, field, apartment):
    """No initial pose at all: 4000 particles over the whole apartment must find the robot."""
    p0 = impl.initialize_uniform(apartment, 3000, np.random.default_rng(3))
    est, err, pf = run_mcl(impl, tour, field, p0, seed=3, beams=40)
    t = tour.t[1:]
    converged = np.flatnonzero((err < 0.20) & (t > 0))
    assert converged.size > 0, "the filter never found the robot"
    first = next(i for i in converged if np.all(err[i:] < 0.40))
    assert t[first] < 10.0, f"global localization took {t[first]:.1f} s of the 50 s tour"
    assert err[-20:].mean() < 0.08, f"settled error {err[-20:].mean():.3f} m is too large"


def test_recovery_from_a_kidnapping(impl, field, apartment):
    """Picked up at t = 20 s and put down in the bedroom. Without injection the filter is lost for
    good; with it, the cloud finds the robot again."""
    log = drive_tour(seed=4, observe_every=1000, scan_every=25, kidnap_at_s=20.0, kidnap_to=(5.0, 4.2, 0.0))
    p0 = impl.initialize_gaussian(log.truth[0], [0.05, 0.05, 0.05], 1500, np.random.default_rng(7))
    _, err_plain, _ = run_mcl(impl, log, field, p0.copy(), seed=7)
    _, err_inject, _ = run_mcl(impl, log, field, p0.copy(), seed=7, inject=0.05, world=apartment)
    late = log.t[1:] > 40.0
    assert err_plain[late].mean() > 0.8, "a plain particle filter cannot recover from a kidnapping"
    assert err_inject[late].mean() < 0.20, (
        f"with 5 % random injection the filter should recover; got {err_inject[late].mean():.2f} m")
