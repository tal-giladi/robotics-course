"""Checker for 12.04 — The inflation layer, exactly like Nav2.

Run: ``python course.py check 12.04`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.config import load_config
from robotlab.sim import World

approx = pytest.approx


def nav2_compute_cost(distance_cells: float, resolution: float, inscribed_radius: float, cost_scaling_factor: float) -> int:
    """Transcription of nav2_costmap_2d InflationLayer::computeCost (Jazzy):

        if (distance == 0) cost = LETHAL_OBSTACLE;
        else if (distance * resolution_ <= inscribed_radius_) cost = INSCRIBED_INFLATED_OBSTACLE;
        else { factor = exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
               cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor); }
    """
    if distance_cells == 0:
        return 254
    if distance_cells * resolution <= inscribed_radius:
        return 253
    return int((253 - 1) * math.exp(-1.0 * cost_scaling_factor * (distance_cells * resolution - inscribed_radius)))


# --- footprint_radii -------------------------------------------------------------------------------
def test_karmel_chassis_rectangle(impl):
    cfg = load_config()
    hl, hw = cfg.chassis.length_m / 2, cfg.chassis.width_m / 2
    inscribed, circumscribed = impl.footprint_radii([(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)])
    assert inscribed == approx(min(hl, hw))
    assert circumscribed == approx(cfg.chassis.footprint_radius_m), "the circumscribed circle is karmel.yaml's footprint radius"


def test_nav2_params_footprint_with_and_without_padding(impl):
    fp = [(0.125, 0.105), (0.125, -0.105), (-0.125, -0.105), (-0.125, 0.105)]  # karmel_bringup/config/nav2_params.yaml
    assert impl.footprint_radii(fp) == approx((0.105, math.hypot(0.125, 0.105)))
    padded = [(0.135, 0.115), (0.135, -0.115), (-0.135, -0.115), (-0.135, 0.115)]  # footprint_padding: 0.01
    assert impl.footprint_radii(padded) == approx((0.115, 0.17734), abs=1e-5)


def test_triangle_uses_edge_distance_not_just_vertices(impl):
    # Edge (0.2, 0) -> (-0.1, 0.15): distance from the origin = |0.2 * 0.15| / hypot(0.3, 0.15) = 0.08944
    inscribed, circumscribed = impl.footprint_radii([(0.2, 0.0), (-0.1, 0.15), (-0.1, -0.15)])
    assert inscribed == approx(0.03 / math.hypot(0.3, 0.15))
    assert circumscribed == approx(0.2)


# --- inflation_cost ------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("distance", "inscribed", "csf", "expected"),
    [
        (0.0, 0.115, 5.0, 254),
        (0.05, 0.115, 5.0, 253),
        (0.115, 0.115, 5.0, 253),  # exactly at the inscribed radius: still 253
        (0.20, 0.115, 5.0, 164),  # 252 * exp(-0.425) = 164.73 -> truncated, not rounded
        (0.25, 0.115, 5.0, 128),  # 252 * exp(-0.675) = 128.35
        (0.15, 0.115, 10.0, 177),  # Nav2 default cost_scaling_factor
        (0.35, 0.115, 10.0, 24),
        (0.70, 0.115, 10.0, 0),  # 252 * exp(-5.85) = 0.73
    ],
)
def test_inflation_cost_table(impl, distance, inscribed, csf, expected):
    assert impl.inflation_cost(distance, inscribed, csf) == expected


def test_inflation_cost_matches_nav2_everywhere(impl):
    radius = load_config().chassis.footprint_radius_m
    for resolution in (0.025, 0.05, 0.1):
        for csf in (1.0, 3.0, 5.0, 10.0):
            for i in range(0, 30):
                for j in range(0, i + 1):
                    d = math.hypot(i, j)
                    got = impl.inflation_cost(d * resolution, radius, csf)
                    want = nav2_compute_cost(d, resolution, radius, csf)
                    assert got == want, f"d={d * resolution:.4f} m, csf={csf}: got {got}, Nav2 gives {want}"
                    assert isinstance(got, int)


# --- distance_for_cost --------------------------------------------------------------------------
@pytest.mark.parametrize("csf", [2.0, 5.0, 10.0])
def test_distance_for_cost_inverts_the_decay(impl, csf):
    for cost in (252, 200, 128, 50, 10, 1):
        d = impl.distance_for_cost(cost, 0.115, csf)
        assert 252 * math.exp(-csf * (d - 0.115)) == approx(cost)
    assert impl.distance_for_cost(128, 0.115, 5.0) == approx(0.25048, abs=1e-5)  # 0.115 + ln(252/128) / 5


# --- inflate ---------------------------------------------------------------------------------------
def test_single_obstacle_cell(impl):
    res, inscribed, csf, radius = 0.05, 0.1, 5.0, 0.35  # 7-cell inflation radius
    master = np.zeros((21, 21), np.uint8)
    master[10, 10] = 254
    before = master.copy()
    out = impl.inflate(master, res, inscribed, csf, radius)
    assert np.array_equal(master, before), "inflate must not modify its input"
    assert out.dtype == np.uint8 and out.shape == master.shape
    for r in range(21):
        for c in range(21):
            d = math.hypot(r - 10, c - 10)
            want = nav2_compute_cost(d, res, inscribed, csf) if d <= 7 else 0
            assert out[r, c] == want, f"cell offset {(r - 10, c - 10)} (d = {d:.2f} cells): got {out[r, c]}, want {want}"
    assert out[10, 12] == 253  # 2 cells = 0.10 m = the inscribed radius
    assert out[10, 13] == 196  # 3 cells = 0.15 m: 252 * exp(-5 * 0.05) = 196.25
    assert out[13, 14] == 119  # offset (3, 4) = 5 cells = 0.25 m: 252 * exp(-0.75) = 119.04
    assert out[10, 17] == 72 and out[10, 18] == 0  # 7 cells = 0.35 m -> 72; 8 cells is beyond the radius


def test_nearest_obstacle_wins(impl):
    res, inscribed, csf, radius = 0.05, 0.1, 5.0, 0.35
    master = np.zeros((15, 30), np.uint8)
    master[:, 5] = 254  # a wall
    master[7, 20] = 254  # a post
    out = impl.inflate(master, res, inscribed, csf, radius)
    for c in range(30):
        d = min(abs(c - 5), math.hypot(0, c - 20))
        want = nav2_compute_cost(d, res, inscribed, csf) if d <= 7 else 0
        assert out[7, c] == want, f"column {c}: got {out[7, c]}, want {want}"


def test_inflation_radius_rounds_up_to_whole_cells(impl):
    # 0.33 m / 0.05 m = 6.6 cells -> Nav2 inflates 7 cells, so the cell 0.35 m away gets a cost.
    master = np.zeros((1, 12), np.uint8)
    master[0, 0] = 254
    out = impl.inflate(master, 0.05, 0.1, 5.0, 0.33)
    assert out[0, 7] == nav2_compute_cost(7, 0.05, 0.1, 5.0) > 0
    assert out[0, 8] == 0


def test_existing_costs_and_unknown_space(impl):
    res, inscribed, csf, radius = 0.05, 0.1, 5.0, 0.35
    master = np.zeros((1, 20), np.uint8)
    master[0, 0] = 254
    master[0, 1] = 255  # unknown, 1 cell = 0.05 m away: inside the inscribed radius -> 253
    master[0, 4] = 255  # unknown, 0.20 m away: inflation would give 152 < 253 -> stays unknown
    master[0, 5] = 230  # an existing higher cost is kept (max)
    master[0, 6] = 10  # an existing lower cost is raised
    master[0, 15] = 99  # outside the inflation radius: untouched
    out = impl.inflate(master, res, inscribed, csf, radius)
    assert out[0, 0] == 254
    assert out[0, 1] == 253
    assert out[0, 4] == 255
    assert out[0, 5] == 230
    assert out[0, 6] == nav2_compute_cost(6, res, inscribed, csf)
    assert out[0, 15] == 99


def test_apartment_lethal_band_is_the_configuration_space(impl):
    """With inscribed = karmel's footprint radius, cost >= 253 marks exactly where the robot center may not go."""
    cfg = load_config()
    grid = World.apartment().to_occupancy_grid(0.1)
    master = np.where(grid.data == 100, 254, 0).astype(np.uint8)
    out = impl.inflate(master, grid.resolution, cfg.chassis.footprint_radius_m, 5.0, 0.55)
    rows, cols = np.nonzero(master == 254)
    lethal = np.column_stack([rows, cols])
    for r in range(0, grid.height, 3):
        for c in range(0, grid.width, 3):
            d = float(np.min(np.hypot(lethal[:, 0] - r, lethal[:, 1] - c)))
            want = nav2_compute_cost(d, grid.resolution, cfg.chassis.footprint_radius_m, 5.0) if d <= 6 else 0
            assert out[r, c] == want, f"cell {(r, c)}: got {out[r, c]}, want {want}"
