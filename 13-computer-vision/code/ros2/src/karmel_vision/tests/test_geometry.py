"""Tests for the 13.16 geometry and object map. No ROS, no camera:

    python -m pytest 13-computer-vision/code/ros2/src/karmel_vision/tests
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from karmel_vision.geometry import (Box, Intrinsics, depth_from_depth_image, depth_from_ground_plane,  # noqa: E402
                                    depth_from_known_size, depth_from_range_sensor, known_size_depth_sigma,
                                    plausible_size, position_covariance, position_from_box, synthetic_box)
from karmel_vision.objects import SemanticObjectMap  # noqa: E402
from karmel_vision.scene import RobotPose, SceneObject, depth_map, to_optical, visible_box  # noqa: E402

INTR = Intrinsics.from_hfov(640, 480, 1.20)
CAMERA_H = 0.145


def test_intrinsics_match_the_course_numbers():
    assert INTR.fx == pytest.approx(467.7, abs=0.1)
    assert (INTR.cx, INTR.cy) == (320.0, 240.0)


def test_resizing_the_image_scales_every_intrinsic():
    half = INTR.scaled_to(320, 240)
    assert (half.fx, half.fy, half.cx, half.cy) == pytest.approx((233.87, 233.87, 160.0, 120.0), abs=0.01)
    # a pixel that was at the image centre is still at the image centre
    assert half.project(np.array([0.0, 0.0, 2.0])) == pytest.approx((160.0, 120.0))


def test_project_and_back_project_are_inverses():
    p = np.array([0.17, -0.05, 1.3])
    u, v = INTR.project(p)
    assert INTR.back_project(u, v, p[2]) == pytest.approx(p)


@pytest.mark.parametrize("depth_m,lateral_m", [(0.6, 0.0), (1.0, 0.17), (2.5, -0.9)])
def test_all_four_depth_methods_agree_on_a_perfect_box(depth_m, lateral_m):
    box = synthetic_box(INTR, depth_m, lateral_m)
    assert depth_from_known_size(box, INTR, 0.24) == pytest.approx(depth_m, rel=1e-6)
    assert depth_from_ground_plane(box, INTR, CAMERA_H) == pytest.approx(depth_m, rel=1e-6)
    slant = math.hypot(depth_m, lateral_m)
    assert depth_from_range_sensor(slant, box, INTR) == pytest.approx(depth_m, rel=1e-6)


def test_ground_plane_returns_none_above_the_horizon():
    above = Box(300, 100, 340, 200)                       # bottom edge above cy
    assert depth_from_ground_plane(above, INTR, CAMERA_H) is None


def test_ground_plane_with_a_pitched_camera_sees_further_down():
    box = synthetic_box(INTR, 1.0, 0.0)
    level = depth_from_ground_plane(box, INTR, CAMERA_H)
    pitched = depth_from_ground_plane(box, INTR, CAMERA_H, pitch_rad=math.radians(10))
    assert level == pytest.approx(1.0)
    assert pitched < level                                # the same pixel now points closer to the robot


def test_depth_image_median_ignores_background_and_holes():
    depth = np.full((480, 640), 3.5, np.float32)          # a far wall everywhere
    box = Box(300, 200, 340, 300)
    inner = box.shrunk(0.5)
    depth[int(inner.y1):int(inner.y2), int(inner.x1):int(inner.x2)] = 1.2
    depth[int(inner.y1):int(inner.y1) + 5, :] = 0.0       # a strip of invalid pixels, as a real sensor gives
    assert depth_from_depth_image(depth, box) == pytest.approx(1.2)


def test_depth_image_returns_none_when_everything_is_invalid():
    assert depth_from_depth_image(np.zeros((480, 640), np.float32), Box(300, 200, 340, 300)) is None


def test_known_size_uncertainty_is_dominated_by_the_object_not_the_pixels():
    box = synthetic_box(INTR, 1.0, 0.0)
    sigma = known_size_depth_sigma(1.0, box, "bottle")
    assert 0.15 < sigma < 0.30                            # about 20 % at 1 m, because bottles vary
    assert known_size_depth_sigma(2.0, box, "bottle") == pytest.approx(2 * sigma, rel=0.2)


def test_covariance_grows_with_depth_error_and_off_centre_boxes():
    centred = synthetic_box(INTR, 2.0, 0.0)
    off = synthetic_box(INTR, 2.0, 1.2)
    sx_centred = math.sqrt(position_covariance(centred, INTR, 2.0, 0.10)[0, 0])
    sx_off = math.sqrt(position_covariance(off, INTR, 2.0, 0.10)[0, 0])
    assert sx_off > 3 * sx_centred                        # depth error leaks sideways for off-centre objects


def test_plausibility_gate_rejects_a_bottle_on_a_poster():
    box = synthetic_box(INTR, 1.0, 0.0)                   # a real 0.24 m bottle 1 m away
    assert plausible_size("bottle", box, INTR, 1.0)[0]
    ok, implied = plausible_size("bottle", box, INTR, 3.1)   # the LiDAR hit the wall behind the poster
    assert not ok and implied == pytest.approx(0.744, abs=0.01)


def test_position_uses_the_box_centre_not_the_bottom():
    box = synthetic_box(INTR, 1.0, 0.0)
    p = position_from_box(box, INTR, 1.0)
    assert p[1] == pytest.approx(CAMERA_H - 0.24 / 2, abs=1e-6)     # centre of a 0.24 m bottle


# --------------------------------------------------------------------------------- the scene
def test_scene_geometry_matches_the_hand_calculation():
    """The 05.10 chain: a bottle at map (2.05, 1.70) seen by a robot at (2.0, 1.0) facing +y."""
    pose = RobotPose(2.0, 1.0, math.pi / 2)
    p = to_optical(pose, np.array([2.05, 1.70, 0.165]))
    assert p == pytest.approx([0.05, -0.02, 0.60], abs=1e-9)        # right, down, forward


def test_visible_box_and_depth_map_are_consistent():
    pose = RobotPose(0.0, 0.0, 0.0)
    obj = SceneObject("bottle", 1.5, 0.0)
    box = visible_box(pose, obj, INTR)
    assert box is not None
    depth = to_optical(pose, obj.centre_map)[2]                     # 1.5 m minus the camera's 0.10 m offset
    assert depth == pytest.approx(1.4)
    assert depth_from_known_size(box, INTR, obj.height_m) == pytest.approx(depth, rel=1e-6)
    assert depth_from_depth_image(depth_map(pose, [obj], INTR), box) == pytest.approx(depth, abs=0.01)


def test_objects_behind_the_robot_are_not_visible():
    assert visible_box(RobotPose(0.0, 0.0, math.pi), SceneObject("cup", 1.0, 0.0), INTR) is None


# --------------------------------------------------------------------------------- object map
def test_repeated_detections_become_one_confirmed_object():
    world = SemanticObjectMap(min_hits=3)
    cov = np.diag([0.04, 0.04, 0.02]) ** 2
    rng = np.random.default_rng(1)
    truth = np.array([2.05, 1.70, 0.12])
    for k in range(10):
        world.observe("bottle", truth + rng.multivariate_normal(np.zeros(3), cov), cov, 0.9, k * 0.1)
    assert len(world.objects) == 1
    obj = world.confirmed()[0]
    assert obj.hits == 10
    assert np.linalg.norm(obj.position - truth) < 0.05
    # fusing ten looks shrinks the uncertainty from 3.5 cm to the 2 cm floor, and no further
    assert obj.sigma_m == pytest.approx(world.min_sigma_m)


def test_a_split_object_is_merged_again():
    """One outlier detection starts a twin; after both have seen the object they agree."""
    world = SemanticObjectMap()
    cov = np.diag([0.04, 0.04, 0.02]) ** 2
    world.observe("bottle", np.array([2.05, 1.70, 0.12]), cov, 0.9, 0.0)
    world.objects.append(type(world.objects[0])(99, "bottle", np.array([2.08, 1.73, 0.13]), cov,
                                                hits=1, first_seen_s=0.1, last_seen_s=0.1, score_sum=0.8))
    assert len(world.objects) == 2
    assert world.merge() == 1
    assert world.objects[0].hits == 2


def test_a_far_away_detection_starts_a_second_object():
    world = SemanticObjectMap()
    cov = np.diag([0.05, 0.05, 0.03]) ** 2
    world.observe("bottle", np.array([2.0, 1.7, 0.12]), cov, 0.9, 0.0)
    world.observe("bottle", np.array([4.0, 0.2, 0.12]), cov, 0.9, 0.1)
    assert len(world.objects) == 2


def test_a_vague_detection_is_rejected_instead_of_polluting_the_map():
    world = SemanticObjectMap(max_sigma_m=0.6)
    huge = np.diag([1.0, 1.0, 1.0])
    assert world.observe("bottle", np.array([2.0, 1.7, 0.12]), huge, 0.9, 0.0) is None
    assert world.objects == []


def test_a_single_detection_is_never_confirmed_and_is_forgotten():
    world = SemanticObjectMap(min_hits=3, forget_after_s=5.0)
    cov = np.diag([0.05, 0.05, 0.03]) ** 2
    world.observe("cup", np.array([1.2, 2.4, 0.05]), cov, 0.6, 0.0)
    assert world.confirmed() == []
    assert world.prune(t_s=6.0) == 1


def test_different_classes_never_merge():
    world = SemanticObjectMap()
    cov = np.diag([0.05, 0.05, 0.03]) ** 2
    p = np.array([1.0, 1.0, 0.1])
    world.observe("bottle", p, cov, 0.9, 0.0)
    world.observe("cup", p, cov, 0.9, 0.1)
    assert len(world.objects) == 2
