"""05.05 demo runs and its key claims hold."""

import pytest

pytest.importorskip("scipy")

import rotations_demo  # noqa: E402


def test_demo_output(capsys) -> None:
    rotations_demo.main()
    out = capsys.readouterr().out
    assert "scipy from_euler('xyz', [30, 45, 90]) == ROS rpy(30, 45, 90)? True" in out
    assert "scipy from_euler('XYZ', [30, 45, 90]) == ROS rpy(30, 45, 90)? False" in out
    assert "[-0.5  0.5 -0.5  0.5]" in out
    assert "rpy(10, 90, 20) == rpy(0, 90, 10)? True" in out
