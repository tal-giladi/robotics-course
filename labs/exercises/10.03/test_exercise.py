"""Checker for 10.03 — The Bayes filter: predict and update on a 1D grid.

Run: ``python course.py check 10.03`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

approx = pytest.approx
HERE = Path(__file__).resolve().parent


def _corridor_sim():
    name = "course_exercise_10_03_corridor_sim"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "corridor_sim.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


# --- normalize ----------------------------------------------------------------------------------
def test_normalize_scales_to_one(impl):
    assert impl.normalize(np.array([1.0, 3.0])) == approx([0.25, 0.75])


def test_normalize_does_not_modify_input(impl):
    b = np.array([2.0, 2.0])
    impl.normalize(b)
    assert b.tolist() == [2.0, 2.0], "normalize must return a new array"


def test_normalize_rejects_zero_belief(impl):
    with pytest.raises(ValueError):
        impl.normalize(np.zeros(4))


# --- predict ------------------------------------------------------------------------------------
KERNEL = [0.1, 0.8, 0.1]


def test_predict_shifts_and_blurs(impl):
    out = impl.predict(np.array([0, 1, 0, 0, 0.0]), 1, KERNEL, wrap=True)
    assert out == approx([0.0, 0.1, 0.8, 0.1, 0.0]), "10% undershoot, 80% exact, 10% overshoot"


def test_predict_wraps_in_a_circular_corridor(impl):
    # From cell 4: moved 0 -> cell 4 (0.1), moved 1 -> cell 0 (0.8), moved 2 -> cell 1 (0.1)
    out = impl.predict(np.array([0, 0, 0, 0, 1.0]), 1, KERNEL, wrap=True)
    assert out == approx([0.8, 0.1, 0.0, 0.0, 0.1])


def test_predict_piles_up_at_a_wall(impl):
    # From cell 3: moved 0 -> 3 (0.1), moved 1 -> 4 (0.8), moved 2 -> "5" = the wall, stays in 4 (0.1)
    out = impl.predict(np.array([0, 0, 0, 1.0, 0]), 1, KERNEL, wrap=False)
    assert out == approx([0.0, 0.0, 0.0, 0.1, 0.9]), "a robot cannot drive through the end wall"


def test_predict_backwards_and_bigger_moves(impl):
    # From cell 4, trying to move -3: really moved -4 (0.2), -3 (0.6) or -2 (0.2) -> cells 0, 1, 2
    out = impl.predict(np.array([0, 0, 0, 0, 1.0, 0, 0]), -3, [0.2, 0.6, 0.2], wrap=False)
    assert out == approx([0.2, 0.6, 0.2, 0.0, 0.0, 0.0, 0.0])


def test_predict_is_a_convolution_on_a_circle(impl):
    rng = np.random.default_rng(3)
    b = rng.random(12)
    b /= b.sum()
    kernel = [0.05, 0.15, 0.6, 0.15, 0.05]
    out = impl.predict(b, 2, kernel, wrap=True)
    expected = sum(p * np.roll(b, 2 + i - 2) for i, p in enumerate(kernel))
    assert out == approx(expected)
    assert out.sum() == approx(1.0), "prediction moves probability around; it never creates or destroys it"


def test_predict_does_not_modify_input(impl):
    b = np.array([0.5, 0.5, 0.0])
    impl.predict(b, 1, KERNEL, wrap=True)
    assert b.tolist() == [0.5, 0.5, 0.0]


def test_uniform_belief_stays_uniform_on_a_circle(impl):
    assert impl.predict(np.full(8, 0.125), 3, KERNEL, wrap=True) == approx(np.full(8, 0.125))


# --- measurement model and update ---------------------------------------------------------------
def test_door_likelihood(impl):
    doors = np.array([True, False, True, False])
    assert impl.door_likelihood(doors, True, 0.9, 0.2) == approx([0.9, 0.2, 0.9, 0.2])
    assert impl.door_likelihood(doors, False, 0.9, 0.2) == approx([0.1, 0.8, 0.1, 0.8])


def test_update_matches_fm16_hallway(impl):
    # FM.16: 5 cells, uniform prior, doors at 1 and 3, likelihood 0.8 / 0.3
    # unnormalized [0.06, 0.16, 0.06, 0.16, 0.06], sum 0.5
    doors = np.array([False, True, False, True, False])
    post = impl.update(np.full(5, 0.2), impl.door_likelihood(doors, True, 0.8, 0.3))
    assert post == approx([0.12, 0.32, 0.12, 0.32, 0.12])


def test_update_with_impossible_reading_raises(impl):
    with pytest.raises(ValueError):
        impl.update(np.array([1.0, 0.0]), np.array([0.0, 1.0]))


# --- the filter loop ----------------------------------------------------------------------------
def test_run_filter_by_hand(impl):
    doors = np.array([True, False, False, True, False])
    history = impl.run_filter(np.ones(5), doors, [1, 1, 1], [True, None, False], KERNEL, 0.8, 0.3, wrap=True)
    assert len(history) == 3
    # Step 1: uniform stays uniform; z=door -> [0.16, .06, .06, .16, .06] / 0.5
    assert history[0] == approx([0.32, 0.12, 0.12, 0.32, 0.12])
    # Step 2: predict only. e.g. cell 1 = 0.1*0.12 + 0.8*0.32 + 0.1*0.12 = 0.28
    assert history[1] == approx([0.16, 0.28, 0.14, 0.14, 0.28])
    # Step 3: predict again -> [0.254, 0.184, 0.254, 0.154, 0.154]; e.g. cell 0 = .1*.16 + .8*.28 + .1*.14
    #         then z=no door, likelihood [0.2, 0.7, 0.7, 0.2, 0.7]
    #         -> unnormalized [0.0508, 0.1288, 0.1778, 0.0308, 0.1078], sum 0.496
    assert history[2] == approx(np.array([0.0508, 0.1288, 0.1778, 0.0308, 0.1078]) / 0.496)


def test_run_filter_rejects_mismatched_lengths(impl):
    with pytest.raises(ValueError):
        impl.run_filter(np.ones(5), np.zeros(5, dtype=bool), [1, 1], [True], KERNEL, 0.8, 0.3)


def test_global_localization_on_a_synthetic_circular_corridor(impl):
    """A robot that doesn't know where it starts must find itself from doors + motion."""
    doors = _corridor_sim().door_map()
    n = doors.size
    kernel = [0.05, 0.9, 0.05]
    successes = 0
    for seed in range(20):
        rng = np.random.default_rng(seed)
        cell = int(rng.integers(n))
        moves, readings, truth = [], [], []
        for _ in range(60):
            cell = (cell + 1 + rng.choice([-1, 0, 1], p=kernel)) % n
            p_door = 0.85 if doors[cell] else 0.1
            moves.append(1)
            readings.append(bool(rng.random() < p_door))
            truth.append(cell)
        history = impl.run_filter(np.ones(n), doors, moves, readings, kernel, 0.85, 0.1, wrap=True)
        error = (int(np.argmax(history[-1])) - truth[-1] + n // 2) % n - n // 2  # signed, around the circle
        successes += abs(error) <= 1
    # The reference solution gets 15/20: noisy motion keeps a few runs ambiguous even after 60 cells.
    assert successes >= 13, f"the belief peak should be within one cell of the truth in most runs; got {successes}/20"


# --- against the simulator ----------------------------------------------------------------------
@pytest.mark.parametrize(("seed", "start_cell"), [(1, 0), (2, 5), (3, 11)])
def test_localizes_the_simulated_robot_in_the_corridor(impl, seed, start_cell):
    """karmel (realistic motors, LiDAR noise) drives 5 m with no idea where it started."""
    cs = _corridor_sim()
    run = cs.drive_corridor(start_cell, 20, seed=seed, realistic=True)
    doors = cs.door_map()
    # Encoders measure one 0.25 m cell to within millimetres, so the motion kernel can be sharp.
    history = impl.run_filter(np.ones(cs.N_CELLS), doors, run.moves, run.readings, [0.05, 0.9, 0.05], 0.9, 0.1, wrap=False)
    final, true_cell = history[-1], run.true_cells[-1]
    near = final[max(0, true_cell - 1): true_cell + 2].sum()
    assert abs(int(np.argmax(final)) - true_cell) <= 1, "after 20 cells the belief peak should be at the true cell (+/-1)"
    assert near > 0.8, f"at least 80% of the belief should be within one cell of the truth, got {near:.2f}"
