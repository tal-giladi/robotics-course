from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.geometry import SE2, angle_diff, deg2rad, rad2deg, rotation_matrix, wrap_angle


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi), (1.5 * math.pi, -0.5 * math.pi),
     (-7.0, -7.0 + 2 * math.pi), (2 * math.pi, 0.0)],
)
def test_wrap_angle_scalar(angle, expected):
    result = wrap_angle(angle)
    assert isinstance(result, float)
    assert result == pytest.approx(expected, abs=1e-12)


def test_wrap_angle_vectorized_range():
    angles = np.linspace(-20, 20, 1001)
    wrapped = wrap_angle(angles)
    assert wrapped.shape == angles.shape
    assert np.all(wrapped > -math.pi) and np.all(wrapped <= math.pi)
    assert np.allclose(np.cos(wrapped), np.cos(angles)) and np.allclose(np.sin(wrapped), np.sin(angles))


def test_angle_diff_takes_short_way():
    assert angle_diff(math.radians(-170), math.radians(170)) == pytest.approx(math.radians(20))


def test_degree_helpers():
    assert deg2rad(180.0) == pytest.approx(math.pi)
    assert rad2deg(math.pi / 2) == pytest.approx(90.0)
    assert np.allclose(deg2rad([90, -90]), [math.pi / 2, -math.pi / 2])


def test_se2_normalizes_theta_and_unpacks():
    x, y, theta = SE2(1, 2, 3 * math.pi)
    assert (x, y, theta) == pytest.approx((1.0, 2.0, math.pi))


def test_compose_moves_in_local_frame():
    pose = SE2(1.0, 0.0, math.pi / 2) @ SE2(1.0, 0.0, math.pi / 2)
    assert pose.as_tuple() == pytest.approx((1.0, 1.0, math.pi))


def test_compose_matches_matrix_product():
    a, b = SE2(0.3, -1.2, 0.7), SE2(-2.0, 0.5, -2.9)
    assert np.allclose((a @ b).as_matrix(), a.as_matrix() @ b.as_matrix())


def test_inverse_and_between():
    a, b = SE2(0.3, -1.2, 0.7), SE2(-2.0, 0.5, -2.9)
    assert (a @ a.inverse()).as_tuple() == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert (a @ a.between(b)).as_tuple() == pytest.approx(b.as_tuple())


def test_matrix_round_trip():
    pose = SE2(4.0, -3.0, -2.5)
    assert SE2.from_matrix(pose.as_matrix()).as_tuple() == pytest.approx(pose.as_tuple())
    with pytest.raises(ValueError):
        SE2.from_matrix(np.eye(2))


def test_apply_single_point_and_batch():
    pose = SE2(1.0, 2.0, math.pi / 2)
    assert pose.apply([1.0, 0.0]) == pytest.approx([1.0, 3.0])
    pts = np.array([[1.0, 0.0], [0.0, 1.0], [2.0, -1.0]])
    out = pose.apply(pts)
    assert out.shape == (3, 2)
    homogeneous = (pose.as_matrix() @ np.column_stack([pts, np.ones(3)]).T).T[:, :2]
    assert np.allclose(out, homogeneous)


def test_from_tuple_and_distance():
    assert SE2.from_tuple((1, 2, 0.5)) == SE2(1.0, 2.0, 0.5)
    assert SE2(0, 0, 0).distance_to(SE2(3, 4, 1)) == pytest.approx(5.0)
    assert np.allclose(rotation_matrix(math.pi / 2) @ [1.0, 0.0], [0.0, 1.0])
