"""Checker for 15.05 — grasp planning: generate, filter, rank.

Run: ``python course.py check 15.05`` (or ``--solution`` to see the reference pass).
The footprints here are hand-checkable rectangles and polygons.
"""

from __future__ import annotations

import math

import numpy as np
import pytest


# --- hand-checkable clusters --------------------------------------------------------------------
def block_points(long_m=0.060, short_m=0.030, height=0.040, yaw=0.0, centre=(0.24, 0.06),
                 n=40, table_z=0.0):
    """A rectangular block's top face, sampled on a grid, then yawed and placed."""
    u = np.linspace(-long_m / 2, long_m / 2, n)
    v = np.linspace(-short_m / 2, short_m / 2, max(3, n // 2))
    uu, vv = np.meshgrid(u, v)
    local = np.column_stack([uu.ravel(), vv.ravel()])
    c, s = math.cos(yaw), math.sin(yaw)
    xy = local @ np.array([[c, -s], [s, c]]).T + np.asarray(centre, dtype=float)
    return np.column_stack([xy, np.full(len(xy), table_z + height)])


def disc_points(radius=0.020, height=0.050, centre=(0.24, 0.0), n=120, table_z=0.0):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    r = np.sqrt(np.linspace(0.05, 1.0, 12))[:, None] * radius
    xy = np.column_stack([(r * np.cos(a)).ravel(), (r * np.sin(a)).ravel()]) + np.asarray(centre)
    return np.column_stack([xy, np.full(len(xy), table_z + height)])


def pose_for(points, impl, yaw=0.0):
    pts = np.asarray(points, dtype=float)
    top = float(pts[:, 2].max())
    return impl.ObjectPose((float(pts[:, 0].mean()), float(pts[:, 1].mean()), top / 2),
                           yaw, (0.060, 0.030, top), top, len(pts))


def jaw(impl, stroke=0.080, force=30.0):
    return impl.JawGripper("test jaw", stroke_m=stroke, max_force_n=force, pad_mu=0.6,
                           pad_radius_m=0.008, finger_length_m=0.06)


def square_hull():
    """A 100 x 100 mm axis-aligned square, centred on the origin, as hull vertices."""
    return np.array([[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]])


# --- footprint_hull -----------------------------------------------------------------------------
def test_the_hull_of_a_rectangle_has_four_vertices(impl):
    hull = impl.footprint_hull(block_points(n=30))
    assert hull.shape[1] == 2
    assert len(hull) == 4, f"a filled rectangle's hull is its 4 corners, got {len(hull)}"


def test_the_hull_drops_the_z_column(impl):
    hull = impl.footprint_hull(block_points())
    assert hull.ndim == 2 and hull.shape[1] == 2


def test_interior_points_do_not_reach_the_hull(impl):
    pts = np.vstack([block_points(n=20), np.array([[0.24, 0.06, 0.04]] * 50)])
    assert len(impl.footprint_hull(pts)) == 4


# --- width_along --------------------------------------------------------------------------------
def test_width_of_an_axis_aligned_block(impl):
    hull = impl.footprint_hull(block_points(yaw=0.0, n=30))
    w_x, _, _ = impl.width_along(hull, [1.0, 0.0])
    w_y, _, _ = impl.width_along(hull, [0.0, 1.0])
    assert w_x == pytest.approx(0.060, abs=1e-6)
    assert w_y == pytest.approx(0.030, abs=1e-6)


def test_width_of_the_square_along_its_diagonal(impl):
    # a 100 mm square projected on (1,1)/sqrt(2) spans 100 * sqrt(2) = 141.42 mm
    d = np.array([1.0, 1.0]) / math.sqrt(2.0)
    w, lo, hi = impl.width_along(square_hull(), d)
    assert w == pytest.approx(0.1 * math.sqrt(2.0), abs=1e-9)
    assert hi - lo == pytest.approx(w, abs=1e-12)


def test_width_is_the_same_along_d_and_minus_d(impl):
    hull = impl.footprint_hull(block_points(yaw=math.radians(37.0), n=30))
    for deg in (0, 23, 61, 145):
        d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
        assert impl.width_along(hull, d)[0] == pytest.approx(impl.width_along(hull, -d)[0], abs=1e-12)


def test_the_narrowest_direction_is_perpendicular_to_the_long_axis(impl):
    yaw = math.radians(25.0)
    hull = impl.footprint_hull(block_points(yaw=yaw, n=40))
    widths = {deg: impl.width_along(hull, [math.cos(math.radians(deg)), math.sin(math.radians(deg))])[0]
              for deg in range(0, 180, 5)}
    best = min(widths, key=widths.get)
    assert best == pytest.approx(115, abs=5), f"expected ~115 deg (25 + 90), got {best}"
    assert widths[best] == pytest.approx(0.030, abs=0.001)


# --- contact_points -----------------------------------------------------------------------------
def test_both_contacts_lie_on_the_closing_line(impl):
    hull = impl.footprint_hull(block_points(yaw=math.radians(30.0), n=30))
    centre = hull.mean(axis=0)
    for deg in (0, 35, 70, 155):
        d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
        perp = np.array([-d[1], d[0]])
        c1, c2 = impl.contact_points(hull, centre, d)
        assert abs(float((c1 - centre) @ perp)) < 1e-12, "slide each contact onto the closing line"
        assert abs(float((c2 - centre) @ perp)) < 1e-12


def test_the_contacts_span_the_width(impl):
    hull = impl.footprint_hull(block_points(yaw=0.0, n=30))
    centre = hull.mean(axis=0)
    d = np.array([1.0, 0.0])
    c1, c2 = impl.contact_points(hull, centre, d)
    assert float((c2 - c1) @ d) == pytest.approx(0.060, abs=1e-6)


def test_the_low_contact_comes_first(impl):
    hull = square_hull()
    d = np.array([1.0, 0.0])
    c1, c2 = impl.contact_points(hull, np.zeros(2), d)
    assert c1[0] < c2[0], "return (low side along -d, high side along +d) in that order"


# --- hull_normal_at -----------------------------------------------------------------------------
def test_the_normal_of_the_right_edge_of_a_square_points_left(impl):
    n = impl.hull_normal_at(square_hull(), [0.05, 0.0])
    assert np.allclose(n, [-1.0, 0.0], atol=1e-9), "the normal must point INTO the hull"


def test_the_normal_of_the_bottom_edge_points_up(impl):
    n = impl.hull_normal_at(square_hull(), [0.0, -0.05])
    assert np.allclose(n, [0.0, 1.0], atol=1e-9)


def test_the_normal_is_a_unit_vector(impl):
    hull = impl.footprint_hull(block_points(yaw=math.radians(17.0), n=30))
    for p in hull:
        assert float(np.linalg.norm(impl.hull_normal_at(hull, p))) == pytest.approx(1.0, abs=1e-9)


def test_normals_of_a_rotated_block_are_rotated_too(impl):
    yaw = math.radians(30.0)
    hull = impl.footprint_hull(block_points(yaw=yaw, n=30))
    centre = hull.mean(axis=0)
    d = np.array([math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)])
    c1, c2 = impl.contact_points(hull, centre, d)
    n1, n2 = impl.hull_normal_at(hull, c1), impl.hull_normal_at(hull, c2)
    assert float(n1 @ n2) < -0.99, "on a rectangle's two long faces the normals are opposed"


# --- grasp_candidates ---------------------------------------------------------------------------
def test_no_candidate_ever_exceeds_the_stroke(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    g = jaw(impl, stroke=0.045)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), g)
    assert grasps, "a 30 mm-wide block must be graspable by a 45 mm jaw"
    for gr in grasps:
        assert gr.width + 0.010 <= g.stroke_m + 1e-12, "the filter is a filter, not a low score"


def test_a_too_wide_object_yields_nothing(impl):
    pts = block_points(long_m=0.150, short_m=0.072, n=40)
    assert impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl, stroke=0.045)) == []


def test_a_wider_jaw_finds_every_yaw(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl, stroke=0.120))
    assert len(grasps) == 36, "0 to 180 degrees in 5 degree steps"


def test_candidates_come_back_sorted(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl))
    scores = [g.score for g in grasps]
    assert scores == sorted(scores, reverse=True)


def test_the_best_grasp_closes_across_the_short_axis(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    best = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl))[0]
    assert math.degrees(best.yaw) == pytest.approx(117.5, abs=7.5), "25 + 90 = 115 deg"
    assert best.width == pytest.approx(0.030, abs=0.006)


def test_every_grasp_carries_its_four_terms(impl):
    pts = block_points(n=30)
    for g in impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl)):
        assert set(g.terms) == {"width", "stab", "centre", "clear"}
        assert all(0.0 <= v <= 1.0 for v in g.terms.values())
        assert 0.0 <= g.score <= 1.0


def test_the_stability_term_matches_epsilon_quality(impl):
    """stab = clip(epsilon / 0.3, 0, 1) on the same two contacts, relative to their midpoint."""
    pts = block_points(yaw=math.radians(25.0), n=40)
    for g in impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl, stroke=0.120))[:6]:
        c1, c2 = (np.asarray(c, dtype=float) for c in g.contacts)
        mid = (c1 + c2) / 2.0
        hull = impl.footprint_hull(pts)
        eps = impl.epsilon_quality(
            [impl.Contact(tuple(c1 - mid), tuple(impl.hull_normal_at(hull, c1)), 0.6),
             impl.Contact(tuple(c2 - mid), tuple(impl.hull_normal_at(hull, c2)), 0.6)],
            com=(0.0, 0.0), length_scale_m=0.05)
        assert g.terms["stab"] == pytest.approx(min(1.0, eps / 0.3), abs=1e-9)


def test_corner_grasps_have_zero_stability(impl):
    """A block at 25 deg: closing along 0 deg lands both pads on CORNERS, where the hull normals
    diverge so far that the grasp line leaves a friction cone — epsilon = 0. Closing at 115 deg
    (25 + 90) lands them on the two flat long faces. Lesson 15.05, Level 2."""
    pts = block_points(yaw=math.radians(25.0), n=40)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl, stroke=0.120))
    by_yaw = {round(math.degrees(g.yaw)): g for g in grasps}
    assert by_yaw[60].terms["stab"] == pytest.approx(0.0, abs=1e-9), "corner contacts: no closure"
    assert by_yaw[165].terms["stab"] == pytest.approx(0.0, abs=1e-9), "corner contacts again"
    assert by_yaw[25].terms["stab"] > 0.8, "the two short faces, perfectly opposed"
    assert by_yaw[115].terms["stab"] > 0.4, "the two long faces — the grasp a 45 mm jaw can use"


def test_higher_friction_never_lowers_the_stability_term(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    pose = pose_for(pts, impl)
    best = [impl.grasp_candidates(pts, pose, jaw(impl), mu=mu)[0].terms["stab"]
            for mu in (0.1, 0.25, 0.4, 0.6, 0.9)]
    assert best == sorted(best), f"stability must be monotone in mu, got {best}"


def test_a_narrower_jaw_scores_the_same_grasp_lower(impl):
    """The width term is relative to the stroke, so scores are NOT comparable across grippers."""
    pts = block_points(yaw=math.radians(25.0), n=40)
    pose = pose_for(pts, impl)
    narrow = impl.grasp_candidates(pts, pose, jaw(impl, stroke=0.045))[0]
    wide = impl.grasp_candidates(pts, pose, jaw(impl, stroke=0.080))[0]
    assert math.degrees(narrow.yaw) == pytest.approx(math.degrees(wide.yaw), abs=10.0)
    assert wide.score > narrow.score


def test_the_closing_height_is_below_the_top_and_above_the_table(impl):
    pts = block_points(height=0.040, n=30)
    pose = pose_for(pts, impl)
    g = impl.grasp_candidates(pts, pose, jaw(impl), table_z=0.0, grasp_depth_m=0.015)[0]
    assert g.position[2] == pytest.approx(0.025, abs=1e-9), "top_z 0.040 - depth 0.015"


def test_a_short_object_clamps_the_closing_height_above_the_table(impl):
    pts = block_points(height=0.008, n=30)
    pose = pose_for(pts, impl)
    g = impl.grasp_candidates(pts, pose, jaw(impl), table_z=0.0, grasp_depth_m=0.015)[0]
    assert g.position[2] == pytest.approx(0.004, abs=1e-9), "clamped to table_z + 4 mm"


def test_the_pre_grasp_is_80mm_above(impl):
    pts = block_points(n=30)
    g = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl))[0]
    assert g.pre_grasp[:2] == pytest.approx(g.position[:2])
    assert g.pre_grasp[2] == pytest.approx(g.position[2] + 0.08)


def test_an_obstacle_beside_a_pad_drops_only_the_clearance_term(impl):
    pts = block_points(long_m=0.060, short_m=0.030, yaw=0.0, centre=(0.24, 0.00), n=40)
    pose = pose_for(pts, impl)
    neighbour = block_points(long_m=0.060, short_m=0.030, yaw=0.0, centre=(0.24, 0.036), n=40)

    free = {round(math.degrees(g.yaw)): g
            for g in impl.grasp_candidates(pts, pose, jaw(impl, stroke=0.120))}
    blocked = {round(math.degrees(g.yaw)): g
               for g in impl.grasp_candidates(pts, pose, jaw(impl, stroke=0.120),
                                              obstacles=[neighbour])}
    # closing along y (yaw 90) drives a pad toward the neighbour 21 mm away
    assert free[90].terms["clear"] == pytest.approx(1.0)
    assert blocked[90].terms["clear"] < 1.0, "the neighbour must cost the clearance term"
    for key in ("width", "stab", "centre"):
        assert blocked[90].terms[key] == pytest.approx(free[90].terms[key], abs=1e-12), (
            f"an obstacle must not change the {key} term")


def test_a_distant_obstacle_costs_nothing(impl):
    pts = block_points(centre=(0.24, 0.00), n=40)
    far = block_points(centre=(0.24, 0.30), n=40)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl), obstacles=[far])
    assert all(g.terms["clear"] == pytest.approx(1.0) for g in grasps)


def test_a_round_footprint_has_no_preferred_yaw(impl):
    pts = disc_points(radius=0.020, n=180)
    grasps = impl.grasp_candidates(pts, pose_for(pts, impl), jaw(impl))
    widths = [g.width for g in grasps]
    assert max(widths) - min(widths) < 0.002, "every direction across a disc is a diameter"


def test_weights_change_the_ranking(impl):
    pts = block_points(yaw=math.radians(25.0), n=40)
    pose = pose_for(pts, impl)
    wide = jaw(impl, stroke=0.120)
    by_width = impl.grasp_candidates(pts, pose, wide,
                                     weights=impl.ScoreWeights(1.0, 0.0, 0.0, 0.0))[0]
    by_stab = impl.grasp_candidates(pts, pose, wide,
                                    weights=impl.ScoreWeights(0.0, 1.0, 0.0, 0.0))[0]
    assert by_width.width < by_stab.width, (
        "width-only picks the narrowest direction; stability-only picks the flat faces, "
        "which on a 60 x 30 block means the LONG axis")
