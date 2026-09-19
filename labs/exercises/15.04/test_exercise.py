"""Checker for 15.04 — tabletop perception: RANSAC plane, clustering, and a graspable pose.

Run: ``python course.py check 15.04`` (or ``--solution`` to see the reference pass).
The clouds here are small and synthetic so every expected number can be checked by hand.
"""

from __future__ import annotations

import math

import numpy as np
import pytest


# --- synthetic clouds ---------------------------------------------------------------------------
def table_points(rng, n=8000, half=0.35, z=0.0, noise=0.0005, tilt_rad=0.0):
    """A level (or tilted) tabletop patch centred on the origin."""
    xy = rng.uniform(-half, half, size=(n, 2))
    zz = np.full(n, z) + xy[:, 0] * math.tan(tilt_rad) + rng.normal(0.0, noise, n)
    return np.column_stack([xy, zz])


def box_points(rng, centre_xy, size, yaw=0.0, n=1500, table_z=0.0, noise=0.0005):
    """Points on the top face and the four side faces of an upright box standing on the table."""
    sx, sy, sz = size
    top = np.column_stack([rng.uniform(-sx / 2, sx / 2, n), rng.uniform(-sy / 2, sy / 2, n),
                           np.full(n, sz)])
    m = n // 2
    side_a = np.column_stack([np.full(m, sx / 2), rng.uniform(-sy / 2, sy / 2, m),
                              rng.uniform(0.0, sz, m)])
    side_b = np.column_stack([rng.uniform(-sx / 2, sx / 2, m), np.full(m, sy / 2),
                              rng.uniform(0.0, sz, m)])
    local = np.vstack([top, side_a, side_b])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    xy = local[:, :2] @ R.T + np.asarray(centre_xy, dtype=float)
    return np.column_stack([xy, local[:, 2] + table_z]) + rng.normal(0.0, noise, (len(local), 3))


def rectangle_footprint(long_m=0.060, short_m=0.030, yaw=0.0, height=0.040, centre=(0.24, 0.06)):
    """The eight outline points of lesson 15.04, Level 4 — checkable entirely by hand."""
    pts2 = np.array([[-long_m / 2, -short_m / 2], [long_m / 2, -short_m / 2],
                     [long_m / 2, short_m / 2], [-long_m / 2, short_m / 2],
                     [0.0, -short_m / 2], [0.0, short_m / 2],
                     [-long_m / 2, 0.0], [long_m / 2, 0.0]])
    c, s = math.cos(yaw), math.sin(yaw)
    xy = pts2 @ np.array([[c, -s], [s, c]]).T + np.asarray(centre, dtype=float)
    return np.column_stack([xy, np.full(len(xy), height)])


def level_plane(impl, z=0.0):
    return impl.Plane((0.0, 0.0, 1.0), -z, 0)


# --- fit_plane_ransac ---------------------------------------------------------------------------
def test_fits_a_level_table(impl):
    rng = np.random.default_rng(0)
    plane = impl.fit_plane_ransac(table_points(rng), rng=np.random.default_rng(1))
    assert abs(plane.height) < 0.002, f"table height {plane.height * 1000:.2f} mm, expected ~0"
    assert abs(float(np.asarray(plane.normal) @ np.array([0, 0, 1]))) > 0.999


def test_the_normal_is_a_unit_vector(impl):
    rng = np.random.default_rng(0)
    plane = impl.fit_plane_ransac(table_points(rng), rng=np.random.default_rng(1))
    assert float(np.linalg.norm(plane.normal)) == pytest.approx(1.0, abs=1e-9)


def test_the_normal_points_up_with_an_up_hint(impl):
    rng = np.random.default_rng(0)
    plane = impl.fit_plane_ransac(table_points(rng), rng=np.random.default_rng(1))
    assert plane.normal[2] > 0.0, "with up_hint=(0,0,1) the returned normal must point up"


def test_fits_a_table_at_a_non_zero_height(impl):
    rng = np.random.default_rng(0)
    plane = impl.fit_plane_ransac(table_points(rng, z=0.120), rng=np.random.default_rng(1))
    assert plane.height == pytest.approx(0.120, abs=0.002)


def test_fits_a_tilted_table(impl):
    rng = np.random.default_rng(0)
    tilt = math.radians(8.0)
    plane = impl.fit_plane_ransac(table_points(rng, tilt_rad=tilt), rng=np.random.default_rng(1))
    angle = math.degrees(math.acos(min(1.0, abs(float(np.asarray(plane.normal) @ np.array([0, 0, 1]))))))
    assert angle == pytest.approx(8.0, abs=1.0)


def test_the_refit_beats_a_three_point_fit_on_average(impl):
    """With refit_iterations=0 the plane comes from 3 noisy points; the refit uses all 4000.

    One seed can get lucky, so this averages over ten. The refit is the cheapest accuracy in the
    whole pipeline — do not skip it.
    """
    rng = np.random.default_rng(0)
    pts = table_points(rng, n=4000, noise=0.003)

    def mean_height_error(refits):
        return float(np.mean([abs(impl.fit_plane_ransac(pts, rng=np.random.default_rng(s),
                                                        refit_iterations=refits).height)
                              for s in range(10)]))

    raw, fit = mean_height_error(0), mean_height_error(2)
    assert fit < raw, f"refit height error {fit * 1000:.3f} mm vs raw {raw * 1000:.3f} mm"


def test_survives_thirty_percent_outliers(impl):
    rng = np.random.default_rng(3)
    table = table_points(rng, n=7000)
    junk = rng.uniform(-0.35, 0.35, size=(3000, 3)) * np.array([1.0, 1.0, 0.3])
    plane = impl.fit_plane_ransac(np.vstack([table, junk]), rng=np.random.default_rng(4))
    assert abs(plane.height) < 0.004, "RANSAC exists precisely to ignore the 30% that is not the table"


def test_the_up_hint_rejects_a_wall(impl):
    """A big vertical wall has more points than the table; the up hint must still pick the table."""
    rng = np.random.default_rng(5)
    table = table_points(rng, n=3000)
    wall_y = rng.uniform(-0.35, 0.35, size=9000)
    wall_z = rng.uniform(0.0, 0.5, size=9000)
    wall = np.column_stack([np.full(9000, 0.30), wall_y, wall_z])
    plane = impl.fit_plane_ransac(np.vstack([table, wall]), rng=np.random.default_rng(6))
    assert abs(plane.normal[2]) > 0.95, f"picked a wall: normal {np.round(plane.normal, 3)}"


def test_inlier_count_is_reported(impl):
    rng = np.random.default_rng(0)
    plane = impl.fit_plane_ransac(table_points(rng, n=5000), rng=np.random.default_rng(1))
    assert plane.n_inliers > 4500


# --- cluster_points -----------------------------------------------------------------------------
def test_an_empty_cloud_has_no_clusters(impl):
    assert impl.cluster_points(np.zeros((0, 3))) == []


def test_two_well_separated_blobs_are_two_clusters(impl):
    rng = np.random.default_rng(7)
    a = rng.normal(0.0, 0.005, (400, 3))
    b = rng.normal(0.0, 0.005, (400, 3)) + np.array([0.20, 0.0, 0.0])
    out = impl.cluster_points(np.vstack([a, b]), voxel_m=0.010, min_points=50)
    assert len(out) == 2


def test_clusters_come_back_largest_first(impl):
    rng = np.random.default_rng(8)
    big = rng.normal(0.0, 0.005, (600, 3))
    small = rng.normal(0.0, 0.005, (200, 3)) + np.array([0.20, 0.0, 0.0])
    out = impl.cluster_points(np.vstack([small, big]), voxel_m=0.010, min_points=50)
    assert [len(c) for c in out] == sorted([len(c) for c in out], reverse=True)
    assert len(out[0]) == 600


def test_small_clusters_are_dropped(impl):
    rng = np.random.default_rng(9)
    big = rng.normal(0.0, 0.005, (600, 3))
    speck = rng.normal(0.0, 0.002, (12, 3)) + np.array([0.20, 0.0, 0.0])
    out = impl.cluster_points(np.vstack([big, speck]), voxel_m=0.010, min_points=50)
    assert len(out) == 1


def test_the_voxel_size_decides_whether_a_gap_separates(impl):
    """Two 30 mm blocks with a 12 mm gap: 4 mm keeps them apart, 20 mm merges them (lesson 15.04)."""
    rng = np.random.default_rng(10)
    a = box_points(rng, (0.24, +0.021), (0.060, 0.030, 0.040), n=800)
    b = box_points(rng, (0.24, -0.021), (0.060, 0.030, 0.040), n=800)
    cloud = np.vstack([a, b])
    assert len(impl.cluster_points(cloud, voxel_m=0.004, min_points=100)) == 2
    assert len(impl.cluster_points(cloud, voxel_m=0.020, min_points=100)) == 1


def test_every_point_lands_in_exactly_one_cluster(impl):
    rng = np.random.default_rng(11)
    a = rng.normal(0.0, 0.005, (400, 3))
    b = rng.normal(0.0, 0.005, (400, 3)) + np.array([0.20, 0.0, 0.0])
    out = impl.cluster_points(np.vstack([a, b]), voxel_m=0.010, min_points=50)
    assert sum(len(c) for c in out) == 800


# --- wrap_axis_angle ----------------------------------------------------------------------------
def test_an_axis_and_its_opposite_are_the_same(impl):
    for deg in (0, 15, 30, 45, 80, 89, 120, 179, 210, 350):
        a = math.radians(deg)
        assert impl.wrap_axis_angle(a) == pytest.approx(impl.wrap_axis_angle(a + math.pi), abs=1e-12)


def test_the_lesson_example_wraps_to_plus_thirty(impl):
    assert math.degrees(impl.wrap_axis_angle(math.radians(-150.0))) == pytest.approx(30.0, abs=1e-9)


def test_the_output_is_in_the_half_open_interval(impl):
    rng = np.random.default_rng(12)
    for a in rng.uniform(-20.0, 20.0, 500):
        out = impl.wrap_axis_angle(float(a))
        assert -math.pi / 2 < out <= math.pi / 2 + 1e-12


def test_both_right_angles_map_to_plus_ninety(impl):
    assert impl.wrap_axis_angle(math.pi / 2) == pytest.approx(math.pi / 2, abs=1e-9)
    assert impl.wrap_axis_angle(-math.pi / 2) == pytest.approx(math.pi / 2, abs=1e-9)


def test_wrapping_is_idempotent(impl):
    rng = np.random.default_rng(13)
    for a in rng.uniform(-10.0, 10.0, 200):
        once = impl.wrap_axis_angle(float(a))
        assert impl.wrap_axis_angle(once) == pytest.approx(once, abs=1e-12)


# --- pose_from_cluster --------------------------------------------------------------------------
def test_the_hand_computed_rectangle(impl):
    """Lesson 15.04, Level 4: a 60 x 30 mm rectangle at 30 deg, centre (0.24, 0.06), top 40 mm."""
    pts = rectangle_footprint(yaw=math.radians(30.0))
    pose = impl.pose_from_cluster(pts, level_plane(impl))
    assert math.degrees(pose.yaw) == pytest.approx(30.0, abs=0.01)
    assert pose.extents[0] == pytest.approx(0.060, abs=1e-6)
    assert pose.extents[1] == pytest.approx(0.030, abs=1e-6)
    assert pose.centre[0] == pytest.approx(0.240, abs=1e-6)
    assert pose.centre[1] == pytest.approx(0.060, abs=1e-6)
    assert pose.top_z == pytest.approx(0.040, abs=1e-6)
    assert pose.centre[2] == pytest.approx(0.020, abs=1e-6), "centre z is half the visible height"


def test_extents_are_widths_not_variances(impl):
    """The eigenvalue ratio here is 4 while the width ratio is 2 — a variance is not a width."""
    pts = rectangle_footprint(yaw=math.radians(30.0))
    pose = impl.pose_from_cluster(pts, level_plane(impl))
    assert pose.extents[0] / pose.extents[1] == pytest.approx(2.0, abs=1e-6)


def test_grasp_width_is_the_short_axis(impl):
    pts = rectangle_footprint(yaw=math.radians(-40.0))
    pose = impl.pose_from_cluster(pts, level_plane(impl))
    assert pose.grasp_width == pytest.approx(pose.extents[1], abs=1e-12)
    assert pose.grasp_width == pytest.approx(0.030, abs=1e-6)


def test_yaw_is_recovered_at_many_angles(impl):
    for deg in (-85.0, -40.0, -5.0, 0.0, 12.0, 55.0, 89.0):
        pts = rectangle_footprint(yaw=math.radians(deg))
        pose = impl.pose_from_cluster(pts, level_plane(impl))
        expected = math.degrees(impl.wrap_axis_angle(math.radians(deg)))
        assert math.degrees(pose.yaw) == pytest.approx(expected, abs=0.01), f"input {deg} deg"


def test_a_box_on_a_raised_table(impl):
    rng = np.random.default_rng(14)
    pts = box_points(rng, (0.30, -0.05), (0.050, 0.025, 0.060), yaw=math.radians(20.0),
                     n=1200, table_z=0.100, noise=0.0002)
    pose = impl.pose_from_cluster(pts, level_plane(impl, z=0.100))
    assert pose.top_z == pytest.approx(0.160, abs=0.002), "top_z is absolute, not above the table"
    assert pose.extents[2] == pytest.approx(0.060, abs=0.002), "extents[2] is the height above it"
    assert pose.centre[0] == pytest.approx(0.300, abs=0.003)
    assert pose.centre[1] == pytest.approx(-0.050, abs=0.003)


def test_too_few_points_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.pose_from_cluster(np.zeros((2, 3)), level_plane(impl))


def test_n_points_is_reported(impl):
    pts = rectangle_footprint()
    assert impl.pose_from_cluster(pts, level_plane(impl)).n_points == 8


# --- segment_tabletop ---------------------------------------------------------------------------
def test_the_whole_pipeline_on_three_boxes(impl):
    rng = np.random.default_rng(15)
    cloud = np.vstack([
        table_points(rng, n=9000),
        box_points(rng, (0.24, 0.10), (0.060, 0.030, 0.040), yaw=math.radians(25.0), n=1200),
        box_points(rng, (0.30, -0.12), (0.040, 0.040, 0.080), n=1200),
        box_points(rng, (0.10, 0.00), (0.030, 0.030, 0.050), yaw=math.radians(-15.0), n=1200),
    ])
    table, poses = impl.segment_tabletop(cloud, rng=np.random.default_rng(16), min_points=200)
    assert abs(table.height) < 0.003
    assert len(poses) == 3, f"expected 3 objects, got {len(poses)}"
    by_x = sorted(poses, key=lambda p: p.centre[0])
    assert by_x[0].centre[0] == pytest.approx(0.10, abs=0.006)
    assert by_x[2].centre[0] == pytest.approx(0.30, abs=0.006)


def test_a_flat_object_is_swallowed_by_a_large_plane_threshold(impl):
    """The upper bound on the threshold, made concrete (lesson 15.04, Level 2)."""
    rng = np.random.default_rng(17)
    cloud = np.vstack([
        table_points(rng, n=9000),
        box_points(rng, (0.24, 0.10), (0.060, 0.030, 0.040), n=1200),   # 40 mm tall
        box_points(rng, (0.10, -0.10), (0.150, 0.072, 0.010), n=1200),  # 10 mm "phone"
    ])
    _, few = impl.segment_tabletop(cloud, plane_threshold_m=0.006,
                                   rng=np.random.default_rng(18), min_points=200)
    _, many = impl.segment_tabletop(cloud, plane_threshold_m=0.025,
                                    rng=np.random.default_rng(18), min_points=200)
    assert len(few) == 2
    assert len(many) == 1, "a 25 mm threshold eats a 10 mm-thick object"
