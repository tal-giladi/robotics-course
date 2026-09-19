"""The closed-form formulas of 05.03 agree with 3x3 matrices and robotlab.geometry.SE2."""

import math

import numpy as np
import pytest

import frames_math as fm
import transforms_2d_demo as demo

rng = np.random.default_rng(3)


def test_by_hand_matches_matrices() -> None:
    for _ in range(100):
        a = (*rng.uniform(-4, 4, 2), rng.uniform(-math.pi, math.pi))
        b = (*rng.uniform(-4, 4, 2), rng.uniform(-math.pi, math.pi))
        p = rng.uniform(-3, 3, 2)
        assert demo.compose_by_hand(a, b) == pytest.approx(fm.se2_params(fm.se2(*a) @ fm.se2(*b)))
        assert demo.invert_by_hand(a) == pytest.approx(fm.se2_params(fm.se2_inverse(fm.se2(*a))))
        assert demo.apply_by_hand(a, p) == pytest.approx(fm.transform_points(fm.se2(*a), p))


def test_demo_runs(capsys) -> None:
    demo.main()
    out = capsys.readouterr().out
    assert "(+2.0000, +1.1000, +90.00 deg)" in out
    assert "(-1.0000, +2.0000, -90.00 deg)" in out
