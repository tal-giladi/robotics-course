"""Checker for 13.16 — from a detection box to a 3D position in the map frame.

Run: ``python course.py check 13.16`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.config import load_config

CAMERA_HEIGHT_M = 0.145            # karmel.yaml: camera z 0.10 above base_link + wheel radius 0.045


def karmel_camera(impl, width: int = 320, height: int = 240):
    """karmel's camera at the requested stream size, from labs/config/karmel.yaml."""
    cfg = load_config().sensors.camera
    return impl.Intrinsics.from_hfov(width, height, cfg.horizontal_fov_rad)


def perfect_box(impl, intr, depth_m: float, lateral_m: float, height_m: float,
                diameter_m: float = 0.07):
    """The box a perfect detector draws around an object of known height standing on the floor."""
    top_v = intr.cy + intr.fy * (CAMERA_HEIGHT_M - height_m) / depth_m
    bottom_v = intr.cy + intr.fy * CAMERA_HEIGHT_M / depth_m
    u = intr.cx + intr.fx * lateral_m / depth_m
    half_w = intr.fx * (diameter_m / 2) / depth_m
    return impl.Box(u - half_w, top_v, u + half_w, bottom_v)


# --- intrinsics ---------------------------------------------------------------------------------
def test_karmel_intrinsics_are_what_the_lesson_says(impl):
    intr = karmel_camera(impl, 640, 480)
    assert intr.fx == pytest.approx(467.7, abs=0.1), "fx = (W/2) / tan(hfov/2)"
    assert (intr.cx, intr.cy) == (320.0, 240.0)


def test_downscaling_halves_every_intrinsic(impl):
    half = impl.scale_intrinsics(karmel_camera(impl, 640, 480), 320, 240)
    assert (half.fx, half.fy, half.cx, half.cy) == pytest.approx((233.87, 233.87, 160.0, 120.0), abs=0.01), \
        "a 2x downscale scales fx, fy, cx and cy by 0.5 — not just the principal point"


def test_non_uniform_resize_scales_the_axes_independently(impl):
    intr = impl.Intrinsics(400.0, 400.0, 320.0, 240.0, 640, 480)
    out = impl.scale_intrinsics(intr, 320, 480)
    assert (out.fx, out.fy, out.cx, out.cy) == pytest.approx((200.0, 400.0, 160.0, 240.0))


# --- back-projection ----------------------------------------------------------------------------
def test_back_project_matches_the_hand_calculation(impl):
    intr = karmel_camera(impl)                      # fx = fy = 233.87, cx = 160, cy = 120
    p = impl.back_project(intr, 208.0, 133.5, 1.15)
    # x = (208-160)*1.15/233.87 = 0.2360, y = (133.5-120)*1.15/233.87 = 0.0664
    assert p == pytest.approx([0.2360, 0.0664, 1.15], abs=1e-3)


def test_the_principal_point_back_projects_straight_ahead(impl):
    intr = karmel_camera(impl)
    assert impl.back_project(intr, intr.cx, intr.cy, 2.0) == pytest.approx([0.0, 0.0, 2.0])


# --- the four depth methods ---------------------------------------------------------------------
@pytest.mark.parametrize("depth_m,lateral_m", [(0.6, 0.0), (1.0, 0.17), (2.5, -0.9)])
def test_all_four_methods_agree_on_a_perfect_box(impl, depth_m, lateral_m):
    """They are not approximations of each other: same geometry, different input."""
    intr = karmel_camera(impl)
    box = perfect_box(impl, intr, depth_m, lateral_m, height_m=0.24)
    assert impl.depth_from_known_size(box, intr, 0.24) == pytest.approx(depth_m, rel=1e-6)
    assert impl.depth_from_ground_plane(box, intr, CAMERA_HEIGHT_M) == pytest.approx(depth_m, rel=1e-6)
    slant = math.hypot(depth_m, lateral_m)
    assert impl.depth_from_range_sensor(slant, box, intr) == pytest.approx(depth_m, rel=1e-6)


def test_known_size_is_inversely_proportional_to_box_height(impl):
    intr = karmel_camera(impl)
    box = impl.Box.from_center_size(160, 120, 20, 56.13)          # fy * 0.24 / 1.0 m
    assert impl.depth_from_known_size(box, intr, 0.24) == pytest.approx(1.0, rel=1e-3)
    taller = impl.Box.from_center_size(160, 120, 20, 2 * 56.13)
    assert impl.depth_from_known_size(taller, intr, 0.24) == pytest.approx(0.5, rel=1e-3)


def test_ground_plane_refuses_rays_at_or_above_the_horizon(impl):
    intr = karmel_camera(impl)
    above = impl.Box(150, 60, 170, intr.cy - 5)
    assert impl.depth_from_ground_plane(above, intr, CAMERA_HEIGHT_M) is None
    on_horizon = impl.Box(150, 60, 170, intr.cy)
    assert impl.depth_from_ground_plane(on_horizon, intr, CAMERA_HEIGHT_M) is None


def test_ground_plane_matches_the_closed_form_for_a_level_camera(impl):
    intr = karmel_camera(impl)
    box = impl.Box(150, 100, 170, 140.0)                           # bottom edge at v = 140
    expected = CAMERA_HEIGHT_M * intr.fy / (140.0 - intr.cy)        # 0.145 * 233.87 / 20 = 1.696 m
    assert impl.depth_from_ground_plane(box, intr, CAMERA_HEIGHT_M) == pytest.approx(expected, rel=1e-9)
    assert expected == pytest.approx(1.696, abs=0.01)


def test_one_degree_of_unmeasured_pitch_is_a_nineteen_percent_error(impl):
    intr = karmel_camera(impl)
    box = perfect_box(impl, intr, 2.0, 0.0, height_m=0.24)
    pitched = impl.depth_from_ground_plane(box, intr, CAMERA_HEIGHT_M, pitch_rad=math.radians(1.0))
    assert pitched == pytest.approx(1.611, abs=0.01), \
        "a camera pitched down sees the same pixel as a CLOSER floor point"


def test_range_sensor_uses_the_forward_component_not_the_slant(impl):
    intr = karmel_camera(impl)
    box = impl.Box.from_center_size(intr.cx + intr.fx, 120, 10, 20)   # bearing exactly 45 degrees
    assert impl.depth_from_range_sensor(1.0, box, intr) == pytest.approx(math.sqrt(0.5), rel=1e-6)


def test_depth_image_takes_the_median_of_the_central_region(impl):
    depth = np.full((240, 320), 3.5)                               # a far wall everywhere
    box = impl.Box(150, 100, 190, 180)
    inner = box.shrunk(0.5)
    depth[int(inner.y1):int(inner.y2), int(inner.x1):int(inner.x2)] = 1.2
    assert impl.depth_from_depth_image(depth, box) == pytest.approx(1.2), \
        "sample the central half of the box, or the background wins"


def test_depth_image_ignores_zeros_and_nans(impl):
    depth = np.full((240, 320), 1.2)
    depth[100:140, 150:170] = 0.0                                  # holes, as every RGB-D camera leaves
    depth[140:150, 150:170] = np.nan
    assert impl.depth_from_depth_image(depth, impl.Box(150, 100, 190, 180)) == pytest.approx(1.2)


def test_depth_image_returns_none_when_there_is_nothing_to_measure(impl):
    assert impl.depth_from_depth_image(np.zeros((240, 320)), impl.Box(150, 100, 190, 180)) is None


def test_depth_image_clips_the_window_to_the_image(impl):
    depth = np.full((240, 320), 0.8)
    box = impl.Box(-40, -40, 60, 60)                               # half outside the frame
    assert impl.depth_from_depth_image(depth, box) == pytest.approx(0.8)


# --- uncertainty --------------------------------------------------------------------------------
def test_covariance_of_a_centred_box_is_dominated_by_the_depth_error(impl):
    intr = karmel_camera(impl)
    box = impl.Box.from_center_size(intr.cx, intr.cy, 20, 30)
    cov = impl.position_covariance(box, intr, 2.0, 0.02, sigma_pixel=2.0)
    assert cov.shape == (3, 3)
    assert math.sqrt(cov[2, 2]) == pytest.approx(0.02), "depth noise passes into Z untouched"
    # sigma_x = Z/fx * sigma_u = 2.0/233.87*2 = 0.0171 m, with no depth term for a centred box
    assert math.sqrt(cov[0, 0]) == pytest.approx(0.0171, abs=1e-3)


def test_depth_error_leaks_sideways_for_off_centre_boxes(impl):
    intr = karmel_camera(impl)
    centred = impl.Box.from_center_size(intr.cx, intr.cy, 20, 30)
    off = impl.Box.from_center_size(intr.cx + 80, intr.cy, 20, 30)
    sx_centred = math.sqrt(impl.position_covariance(centred, intr, 2.0, 0.47)[0, 0])
    sx_off = math.sqrt(impl.position_covariance(off, intr, 2.0, 0.47)[0, 0])
    assert sx_off > 5 * sx_centred, "an off-centre object inherits the depth error laterally"


def test_covariance_is_symmetric_and_positive_definite(impl):
    intr = karmel_camera(impl)
    cov = impl.position_covariance(impl.Box(200, 130, 230, 190), intr, 1.5, 0.05)
    assert cov == pytest.approx(cov.T)
    assert np.all(np.linalg.eigvalsh(cov) > 0)


# --- the plausibility gate ----------------------------------------------------------------------
def test_gate_accepts_a_real_bottle(impl):
    intr = karmel_camera(impl)
    box = perfect_box(impl, intr, 1.0, 0.0, height_m=0.24)
    ok, implied = impl.plausible_size("bottle", box, intr, 1.0)
    assert ok and implied == pytest.approx(0.24, abs=0.01)


def test_gate_rejects_a_bottle_on_a_poster(impl):
    """Same box, but the range sensor hit the wall 3.1 m away behind the poster."""
    intr = karmel_camera(impl)
    box = perfect_box(impl, intr, 1.0, 0.0, height_m=0.24)
    ok, implied = impl.plausible_size("bottle", box, intr, 3.1)
    assert not ok
    assert implied == pytest.approx(0.744, abs=0.01)


def test_gate_rejects_a_cup_that_would_be_vase_sized(impl):
    intr = karmel_camera(impl)
    box = perfect_box(impl, intr, 1.0, 0.0, height_m=0.10, diameter_m=0.09)
    assert impl.plausible_size("cup", box, intr, 1.0)[0]
    assert not impl.plausible_size("cup", box, intr, 3.0)[0]


# --- association and fusion ---------------------------------------------------------------------
def test_mahalanobis_uses_both_covariances(impl):
    cov = np.diag([0.0625, 0.0625, 0.0625])                        # sigma = 0.25 m
    p1, p2 = np.array([1.0, 1.0, 0.1]), np.array([1.3, 1.0, 0.1])
    # d^2 = 0.3^2 / (0.0625 + 0.0625) = 0.72  ->  well inside the 99 % gate
    assert impl.mahalanobis2(p1, cov, p2, cov) == pytest.approx(0.72, abs=1e-6)
    assert impl.mahalanobis2(p1, cov, p2, cov) < impl.GATE_99
    tight = np.diag([4e-4, 4e-4, 4e-4])                            # sigma = 0.02 m (a depth camera)
    assert impl.mahalanobis2(p1, tight, p2, tight) > impl.GATE_99  # the same 0.3 m is now two objects


def test_mahalanobis_is_zero_for_identical_positions(impl):
    cov = np.diag([0.01, 0.01, 0.01])
    p = np.array([2.0, 1.0, 0.1])
    assert impl.mahalanobis2(p, cov, p, cov) == pytest.approx(0.0)


def test_fusing_two_equal_measurements_gives_their_mean(impl):
    cov = np.diag([0.01, 0.01, 0.01])
    p, c = impl.fuse(np.array([2.0, 1.0, 0.1]), cov, np.array([2.2, 1.0, 0.1]), cov)
    assert p == pytest.approx([2.1, 1.0, 0.1])
    assert np.all(np.diag(c) <= np.diag(cov) + 1e-12), "fusing must not increase the uncertainty"


def test_fusion_weights_the_more_certain_measurement_more(impl):
    loose = np.diag([0.09, 0.09, 0.09])                            # sigma 0.30 m
    tight = np.diag([0.01, 0.01, 0.01])                            # sigma 0.10 m
    p, _ = impl.fuse(np.array([0.0, 0.0, 0.0]), loose, np.array([1.0, 0.0, 0.0]), tight)
    assert p[0] == pytest.approx(0.9, abs=1e-6), "inverse-variance weighting: 0.09/(0.09+0.01)"


def test_fusion_never_claims_more_precision_than_the_floor(impl):
    cov = np.diag([1e-4, 1e-4, 1e-4])
    p, c = np.array([2.0, 1.0, 0.1]), np.diag([1e-4, 1e-4, 1e-4])
    for _ in range(50):
        p, c = impl.fuse(p, c, np.array([2.0, 1.0, 0.1]), cov, min_sigma_m=0.02)
    assert np.all(np.diag(c) == pytest.approx(0.02 ** 2)), \
        "without a floor, a fused object rejects its own next detection and the map grows a twin"


def test_a_fused_object_still_associates_with_the_next_detection(impl):
    """The end-to-end property the floor exists for."""
    rng = np.random.default_rng(3)
    truth = np.array([2.05, 1.70, 0.12])
    meas = np.diag([0.04, 0.04, 0.02]) ** 2
    p, c = truth.copy(), meas.copy()
    for _ in range(40):
        z = truth + rng.multivariate_normal(np.zeros(3), meas)
        assert impl.mahalanobis2(p, c, z, meas) < impl.GATE_99
        p, c = impl.fuse(p, c, z, meas)
    # the floored covariance keeps a little slack: the estimate tracks, it does not converge to a point
    assert np.linalg.norm(p - truth) < 0.05
