"""05.04 demo: the full 3D chain and the batched scan transform."""

import numpy as np
import pytest

import frames_math as fm
import homogeneous_demo as demo


def test_chain_numbers() -> None:
    T = demo.karmel_bottle_chain()
    bottle = [0.05, -0.02, 0.60]
    assert fm.transform_points(T["T_map_optical"], bottle) == pytest.approx([2.05, 1.70, 0.165])
    assert T["T_map_optical"][:3, 3] == pytest.approx([2.0, 1.1, 0.145])
    assert fm.transform_points(fm.se3_inverse(T["T_map_optical"]), [0, 0, 0]) == pytest.approx([-2.0, 0.145, -1.1])


def test_scan_batch() -> None:
    T = demo.karmel_bottle_chain()
    pts = fm.transform_points(T["T_map_base_link"] @ T["T_base_link_laser"], demo.fake_scan())
    assert pts.shape == (360, 3)
    assert pts[0] == pytest.approx([2.0, -1.0, 0.165])
    assert pts[180] == pytest.approx([2.0, 3.0, 0.165])
    assert np.linalg.norm(pts[:, :2] - [2.0, 1.0], axis=1) == pytest.approx(np.full(360, 2.0))
