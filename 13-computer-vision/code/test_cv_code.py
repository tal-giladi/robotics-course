"""Tests for the 13.01-13.08 code. No camera needed.

  py -m pytest 13-computer-vision/code/test_cv_code.py -q        (from the repository root)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aruco_compat import detect_markers, estimate_marker_pose, get_dictionary  # noqa: E402
from camera_model import (PinholeCamera, ground_point_from_pixel, inv_T, karmel_camera,  # noqa: E402
                          karmel_T_base_optical, transform_points)
from synthetic import (BALL_RADIUS_M, ball_scene, checkerboard_image, checkerboard_object_points,  # noqa: E402
                       distort_normalized, random_board_poses, tabletop, tabletop_camera_pose,
                       undistort_normalized)


def test_focal_length_from_fov():
    cam = karmel_camera()
    assert cam.fx == pytest.approx(467.7, abs=0.1)


def test_project_back_project_roundtrip():
    cam = karmel_camera()
    p = np.array([[0.2, -0.1, 1.5], [-0.5, 0.3, 3.0]])
    uv, ok = cam.project(p)
    assert ok.all()
    assert np.allclose(cam.back_project(uv, p[:, 2]), p)


def test_project_matches_opencv():
    cam = karmel_camera()
    p = np.array([[0.2, -0.1, 1.5]])
    uv, _ = cam.project(p)
    uv_cv, _ = cv2.projectPoints(p, np.zeros(3), np.zeros(3), cam.K, None)
    assert np.allclose(uv, uv_cv.reshape(1, 2))


def test_undistort_inverts_distort():
    dist = (-0.28, 0.11, 0.0008, -0.0006, -0.02)
    x = np.linspace(-0.7, 0.7, 50)
    y = np.linspace(0.5, -0.5, 50)
    xd, yd = distort_normalized(x, y, dist)
    xu, yu = undistort_normalized(xd, yd, dist)
    assert np.abs(xu - x).max() < 1e-4 and np.abs(yu - y).max() < 1e-4


def test_ball_truth_and_ground_ray():
    img, truth = ball_scene(np.random.default_rng(0), ball_xy=(1.1, 0.1))
    assert img.shape == (480, 640, 3) and img.dtype == np.uint8
    assert truth.radius_px == pytest.approx(karmel_camera().fx * BALL_RADIUS_M / 1.0, rel=1e-6)
    g = ground_point_from_pixel(karmel_camera(), karmel_T_base_optical(), (truth.u, truth.v), BALL_RADIUS_M)
    assert np.allclose(g, truth.ball_base, atol=1e-9)


def test_find_ball_beats_naive():
    from find_ball import find_ball, find_ball_naive
    img, truth = ball_scene(np.random.default_rng(5), ball_xy=(1.2, 0.1))
    det = find_ball(img)
    assert det is not None and math.hypot(det.u - truth.u, det.v - truth.v) < 2.0
    naive = find_ball_naive(img)
    assert math.hypot(naive.u - truth.u, naive.v - truth.v) > 20      # the box wins


def test_morphology_removes_speckles():
    from preprocessing import clean_mask, count_blobs, naive_orange_mask
    img, _ = ball_scene(np.random.default_rng(2), ball_xy=(0.9, -0.05), noise_sigma=14.0)
    mask = naive_orange_mask(img)
    assert count_blobs(clean_mask(mask)) < count_blobs(mask) / 10


def test_orb_homography_recovers_rotation():
    from features import match_orb, poster, rotation_homography, second_view
    rng = np.random.default_rng(4)
    img1 = poster(rng)
    H = rotation_homography(karmel_camera().K, math.radians(8), math.radians(3), math.radians(4))
    img2 = second_view(img1, H, rng)
    cv2.setRNGSeed(0)
    r = match_orb(img1, img2)
    assert len(r["inliers"]) > 100
    pts = np.float32([r["kp1"][m.queryIdx].pt for m in r["inliers"]]).reshape(-1, 1, 2)
    err = np.linalg.norm(cv2.perspectiveTransform(pts, r["H"]) - cv2.perspectiveTransform(pts, H), axis=2)
    assert err.mean() < 1.0


def test_rendered_checkerboard_corners_are_exact():
    from calibrate import TRUE_CAM, TRUE_DIST, find_corners
    rng = np.random.default_rng(1)
    T = random_board_poses(TRUE_CAM, rng, 1)[0]
    img = checkerboard_image(TRUE_CAM, TRUE_DIST, T, rng)
    corners = find_corners(img)
    assert corners is not None
    obj = checkerboard_object_points().astype(np.float64)
    truth, _ = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], TRUE_CAM.K, TRUE_DIST)
    assert np.linalg.norm(truth.reshape(-1, 2) - corners.reshape(-1, 2), axis=1).mean() < 0.3


def test_calibration_recovers_intrinsics():
    from calibrate import TRUE_CAM, TRUE_DIST, calibrate
    rng = np.random.default_rng(21)
    poses = random_board_poses(TRUE_CAM, rng, 12)
    res = calibrate([checkerboard_image(TRUE_CAM, TRUE_DIST, T, rng) for T in poses])
    assert res["rms"] < 0.3
    assert res["K"][0, 0] == pytest.approx(TRUE_CAM.fx, rel=0.01)
    assert res["K"][0, 2] == pytest.approx(TRUE_CAM.cx, abs=3.0)
    assert res["dist"][0] == pytest.approx(TRUE_DIST[0], abs=0.03)


def test_marker_pose():
    from markers import DOCK_TAG_ID, TAG_DICT, TAG_SIZE_M, detect_and_estimate, dock_tag_pose
    from synthetic import marker_scene
    cam = karmel_camera()
    T_bo = karmel_T_base_optical()
    truth = dock_tag_pose(1.0, 0.1, 0.15, math.radians(-25))
    img = marker_scene(cam, inv_T(T_bo) @ truth, np.random.default_rng(0), DOCK_TAG_ID, TAG_SIZE_M, TAG_DICT)
    found = detect_and_estimate(img, cam.K, None)
    assert [f[0] for f in found] == [DOCK_TAG_ID]
    Tb = T_bo @ found[0][1]
    assert np.linalg.norm(Tb[:3, 3] - truth[:3, 3]) < 0.02


def test_ippe_degenerate_case_falls_back():
    cam = karmel_camera()
    square = np.array([[294.0, 214.0], [345.0, 215.0], [345.0, 265.0], [294.0, 266.0]])
    T, errs, _ = estimate_marker_pose(square, 0.10, cam.K, None)
    assert errs[0] < 2.0 and T[2, 3] == pytest.approx(0.90, abs=0.04)


def test_depth_pipeline_finds_table_and_objects():
    from depth3d import depth_to_points, objects_on_plane, ransac_iterations, ransac_plane
    cam = karmel_camera()
    T_wc = tabletop_camera_pose()
    z, _ = tabletop(cam, T_wc)
    pts = transform_points(T_wc, depth_to_points(z, cam, stride=2))
    near = pts[pts[:, 2] > -0.5]
    plane = ransac_plane(near, 0.005, ransac_iterations(0.6), np.random.default_rng(0))
    assert abs(abs(plane.normal[2]) - 1) < 1e-3
    objs = objects_on_plane(near, plane, min_points=20)
    assert len(objs) == 2
    assert max(o.height_m for o in objs) == pytest.approx(0.10, abs=0.01)


def test_stereo_depth():
    from depth3d import disparity_sgbm, disparity_to_depth
    from synthetic import stereo_pair
    cam = karmel_camera()
    left, right, z = stereo_pair(cam, tabletop_camera_pose(), 0.075, np.random.default_rng(0))
    zs = disparity_to_depth(disparity_sgbm(left, right), cam.fx, 0.075)
    valid = (zs > 0) & (z > 0)
    assert valid.mean() > 0.6
    assert np.median(np.abs(zs - z)[valid]) < 0.01


def test_aruco_dictionary_available():
    d = get_dictionary()
    corners, ids, _ = detect_markers(np.full((100, 100), 255, np.uint8), d)
    assert ids is None or len(ids) == 0


def test_pinhole_camera_scaled_keeps_geometry():
    cam = PinholeCamera(500, 500, 320, 240, 640, 480)
    s = cam.scaled(2)
    uv, _ = cam.project(np.array([[0.1, 0.05, 1.0]]))
    uv2, _ = s.project(np.array([[0.1, 0.05, 1.0]]))
    assert np.allclose((uv2 + 0.5) / 2 - 0.5, uv)
