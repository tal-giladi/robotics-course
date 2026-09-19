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
from robotlab.sim import OccupancyGrid


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


# --- 10.06-10.10 --------------------------------------------------------------------------------
def test_ekf_landmarks(tmp_path):
    import ekf_landmarks

    res = ekf_landmarks.main(["--solution", "--seeds", "2", "--out", str(tmp_path)])
    assert max(res["numbers"]["jacobian_errors"].values()) < 1e-7, "analytic Jacobians must match central differences"
    assert res["numbers"]["nis"] == pytest.approx(0.319, abs=1e-3)
    tour = res["tour"]
    assert tour["ekf5"]["rmse"] < 0.05 and tour["ekf5"]["max"] < 0.12
    assert tour["dr_rmse"] > 5 * tour["ekf5"]["rmse"], "the EKF must beat dead reckoning by a wide margin"
    assert 1.0 < tour["ekf5"]["nis"] < 3.5 and tour["ekf5"]["nees"] < 6.0
    sweep = res["k_sweep"]
    assert sweep[0.002]["nees"] > 10, "too little process noise -> badly overconfident"
    assert sweep[0.05]["nees"] < 2.0, "too much process noise -> underconfident"
    assert res["nowrap"]["nis"] > 100, "an unwrapped bearing innovation must blow the NIS up"
    assert res["association"]["good start, ids ignored"]["wrong"] == 0
    assert res["association"]["start 36 cm / 11 deg off, P0 honest, no ids"]["wrong"] > 10
    assert res["linearization"]["mc_x"] < 0.95 and res["linearization"]["inside95"] < 0.5, (
        "with 30 deg of heading uncertainty the EKF's ellipse should NOT cover 95% of the truth")
    assert _png(tmp_path / "ekf_tour.png") and _png(tmp_path / "ekf_sigma.png")


def test_imu_odom_fusion(tmp_path):
    import imu_odom_fusion

    res = imu_odom_fusion.main(["--seeds", "1", "--out", str(tmp_path)])
    assert res["bias_hat"] == pytest.approx(0.01, abs=3 * res["bias_se"] + 1e-4)
    assert res["scale"] == pytest.approx(1.04, abs=0.02), "the fit should recover wheel_separation_scale"
    raw, cal = np.array(res["raw_sweep"]), np.array(res["cal_sweep"])
    assert raw.max() / raw.min() > 5, "with two biases the alpha sweep has a deep, lucky minimum"
    assert cal.max() < 1.5 and cal.max() / cal.min() < 5, "after calibration every alpha is fine"
    assert res["best_flip"][0] != res["best_raw"][0], "flipping the gyro bias must move the best alpha"
    assert res["rmse"]["rmse_kf"] < 0.2 * res["rmse"]["rmse_odom"]
    assert res["bias_final"] == pytest.approx(0.01, abs=0.003), "the KF must find the gyro bias"
    assert _png(tmp_path / "imu_odom_run.png") and _png(tmp_path / "imu_odom_alpha.png")


def test_mcl_apartment(tmp_path):
    import mcl_apartment

    res = mcl_apartment.main(["--solution", "--quick", "--out", str(tmp_path)])
    track = res["tracking"][100]
    assert track["rmse"] < 0.05 and res["dead_reckoning_rmse"] > 20 * track["rmse"]
    conv = res["global"][1000]
    assert conv["converged_s"] is not None and conv["converged_s"] < 15.0
    assert conv["settled"] < 0.10
    sym = res["symmetry"]
    # In a plain rectangle the two 180-degree-apart poses are indistinguishable, so the cloud
    # collapses onto ONE of them; which one is chance (seed, particle count, run length).
    assert sym["near_truth"] + sym["mirrored"] > 0.8, "the cloud must settle on one of the two poses"
    assert (sym["final"] > 1.0) == (sym["mirrored"] > 0.5), (
        "a 2 m error means it picked the mirror pose, and vice versa")
    r = res["resampling"]
    assert r["mn_var"] > 3 * r["lv_var"], "multinomial resampling adds variance for nothing"
    assert r["depletion"]["every scan"][0] > 5 * r["depletion"]["when ESS < N/2"][0]
    kid = res["kidnap"]
    assert kid["plain_late"] > 0.8 > kid["inject_late"], "only random injection survives a kidnapping"
    assert kid["inject_before"] > kid["plain_before"], "... and it costs accuracy while localized"
    models = res["models"]
    assert models["beam_s"] > 2 * models["lf_s"], "ray casting is much more expensive"
    assert _png(tmp_path / "mcl_runs.png") and _png(tmp_path / "mcl_kidnap.png")


def test_export_replay(tmp_path):
    import export_replay

    r = export_replay.main(["--out", str(tmp_path)])
    assert r["map_cells"] == (132, 112) and r["resolution"] == 0.05
    assert r["occupied"] > 1500 and r["free"] > 10_000
    assert (tmp_path / "apartment.yaml").exists() and (tmp_path / "apartment.pgm").exists()
    grid = OccupancyGrid.load(tmp_path / "apartment.yaml")   # map_server format, round trip
    assert grid.data.shape == (112, 132) and grid.origin == pytest.approx((-0.3, -0.3))
    data = np.load(tmp_path / "replay.npz")
    assert data["ranges"].shape == (r["scans"], 360) and data["truth"].shape[1] == 3
    assert r["dr_final"] > 0.5, "the odometry the ROS nodes replay must actually drift"


def test_tag_localization(tmp_path):
    import tag_localization

    res = tag_localization.main(["--solution", "--quick", "--repeats", "1", "--out", str(tmp_path)])
    single = res["single"]
    assert abs(single["range_err"]) < 0.03 and abs(single["bearing_err"]) < math.radians(1.0)
    fit = res["fit"]
    assert 0.0 < fit["sigma_rel_measured"] < 0.10 and fit["detection_rate"] > 0.4
    assert fit["sigma_bearing"] >= tag_localization.SIGMA_BEARING_FLOOR, "simulated sigmas must be floored"
    assert res["visibility"]["fraction"] > 0.3, "ten tags should be visible a good part of the tour"
    ekf = res["ekf"]
    assert ekf["rmse"] < 0.03 and 0.5 < ekf["nis"] < 3.5
    assert res["dead_reckoning"]["rmse"] > 10 * ekf["rmse"]
    ungated = res["maperror_no_gate"]
    gated = res["maperror_chi-square_95%_gate"]
    assert ungated["rmse"] > 2 * gated["rmse"], "a mis-measured tag must hurt when it is not gated"
    assert ungated["nis"] > 4.0 and gated["rejected"] > 10
    assert _png(tmp_path / "tag_map.png")
