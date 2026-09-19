"""05.06 demo numbers."""

import math

import pytest

import frames_math as fm
import rep105_demo


def test_map_to_odom_worked_example() -> None:
    T = rep105_demo.map_to_odom(fm.se2(3.8, 0.3, math.radians(5)), fm.se2(4.0, 0.0, 0.0))
    x, y, th = fm.se2_params(T)
    assert (x, y, math.degrees(th)) == pytest.approx((-0.1848, -0.0486, 5.0), abs=1e-4)
    assert fm.se2_params(T @ fm.se2(4.0, 0.0, 0.0)) == pytest.approx((3.8, 0.3, math.radians(5)))


def test_demo_runs(capsys) -> None:
    rep105_demo.main()
    assert "(+4.2981, +0.3436, +5.00 deg)" in capsys.readouterr().out
