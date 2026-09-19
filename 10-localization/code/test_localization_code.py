"""Smoke and number tests for the module-10 experiment scripts (reference solutions, headless).

    py -m pytest 10-localization/code
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import belief_cloud
import histogram_corridor
import kf1d_wall
import kf_multivariate
import loc_common as lc
import uncertainty_ellipses


def _png(path) -> bool:
    return path.exists() and path.stat().st_size > 10_000 and path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_chi_square_helpers():
    assert lc.chi2_2dof_threshold(0.95) == pytest.approx(5.991, abs=1e-3)
    major, minor, angle = lc.ellipse_axes(np.diag([0.04, 0.01]), 4.0)  # 2-sigma of sigma 20 cm x 10 cm
    assert (major, minor, angle) == pytest.approx((0.4, 0.2, 0.0))
    rng = np.random.default_rng(0)
    pts = rng.multivariate_normal([0, 0], [[0.05, 0.02], [0.02, 0.03]], 20_000)
    assert lc.fraction_inside(pts, [0, 0], np.array([[0.05, 0.02], [0.02, 0.03]]), 5.991) == pytest.approx(0.95, abs=0.01)


def test_belief_cloud(tmp_path):
    res = belief_cloud.main(["--out", str(tmp_path)])
    rows = res["rows"]
    assert all(r["truth_inside_95"] for r in rows), "the true pose should be inside the 95% cloud ellipse"
    assert rows[-1]["sigma_y_m"] > 5 * rows[0]["sigma_y_m"], "uncertainty grows with distance driven"
    kid = res["kidnap"]
    assert kid["after_range_m"] > 5 * kid["before_range_m"], "kidnapping shows up as large landmark residuals"
    assert _png(tmp_path / "belief_cloud.png") and _png(tmp_path / "kidnapped.png")


def test_uncertainty_ellipses(tmp_path):
    res = uncertainty_ellipses.main(["--out", str(tmp_path)])
    ex = res["example"]
    assert ex["sigma_major"] == pytest.approx(0.2525, abs=5e-4)
    assert ex["sigma_minor"] == pytest.approx(0.0568, abs=5e-4)
    assert ex["angle_deg"] == pytest.approx(-29.6, abs=0.2)
    cov = {name: (p1, p2) for name, _, p1, p2 in res["coverage"]}
    assert cov["1 sigma"] == pytest.approx((0.6827, 0.3935), abs=1e-4)
    assert cov["2 sigma"] == pytest.approx((0.9545, 0.8647), abs=1e-4)
    growth = res["growth"]
    assert growth[-1]["major95"] > 20 * growth[0]["major95"]
    assert all(0.92 < r["in_95"] < 0.97 for r in growth)
    assert _png(tmp_path / "ellipses_example.png") and _png(tmp_path / "ellipses_growth.png")


def test_histogram_corridor(tmp_path):
    res = histogram_corridor.main(["--solution", "--out", str(tmp_path)])
    g = res["global"]
    assert g["peak"][-1] == g["true"][-1] and g["p_true_final"] > 0.7
    kid = res["kidnap"]
    assert kid[0.0]["recovered_step"] is None, "a pure Bayes filter never recovers from this kidnapping"
    assert kid[0.05]["recovered_step"] is not None
    assert kid[0.2]["p_before"] < kid[0.0]["p_before"], "injected probability costs confidence"
    k = res["kernels"]
    assert k["blurry [.25, .5, .25]"]["settled_step"] is None
    assert k["matched [.05, .9, .05]"]["settled_step"] < k["sharp [0, 1, 0]"]["settled_step"]
    assert _png(tmp_path / "histogram_global.png")


def test_kf1d_wall(tmp_path):
    res = kf1d_wall.main(["--solution", "--out", str(tmp_path)])
    run = res["run"]
    assert run["kf_rmse"] < 0.3 * run["meas_rmse"]
    assert run["odom_short_rmse"] > 10 * run["odom_rmse"]
    q = res["q_sweep"]
    short = q["short"][0]
    best = int(np.argmin(short))
    assert 0 < best < len(short) - 1, "with biased odometry the RMSE-vs-Q curve is U-shaped"
    assert abs(q["short"][1][0]) > 0.02, "Q = 0 with biased odometry: innovations average more than 2 cm"
    r_rows = res["r_sweep"]
    assert r_rows[0][3] < 0.5, "assuming a 3 mm sensor makes the filter overconfident"
    assert r_rows[2][3] > 0.9
    assert _png(tmp_path / "kf1d_run.png") and _png(tmp_path / "kf1d_q_sweep.png")


def test_kf_multivariate(tmp_path):
    res = kf_multivariate.main(["--solution", "--out", str(tmp_path)])
    by_hand = res["by_hand"]
    assert by_hand["x"] == pytest.approx([1.0900200, 0.5009980], rel=1e-6)
    sim = res["sim"]
    assert sim["fused"]["pos_rmse"] < sim["tof_only"]["pos_rmse"] < sim["tof_only"]["tof_rmse"]
    assert sim["fused"]["vel_rmse"] < 0.5 * sim["tof_only"]["vel_rmse"]
    nees = res["nees"]
    assert nees[0.01]["mean"] > 10 and 1.48 < nees[1.0]["mean"] < 2.59 and nees[100.0]["mean"] < 1.48
    enc = res["encoder_std"]
    assert enc[0.01]["nees"] > 5.99 and enc[0.01]["nis_enc"] < 0.5, "NIS looks calm while NEES says overconfident"
    assert math.isfinite(enc[0.03]["nees"]) and enc[0.03]["nees"] < 3
    assert _png(tmp_path / "kf_multivariate_run.png") and _png(tmp_path / "kf_nees.png")
