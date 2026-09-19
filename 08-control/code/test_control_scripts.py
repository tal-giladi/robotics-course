"""The module-08 experiment scripts run headless and produce the numbers the lessons quote.

    py -m pytest 08-control/code          (about a minute; the --fake test runs in real time)
"""

from __future__ import annotations

import math

import pytest

import motor_step_response
import open_vs_closed_loop
import p_control
import pi_control
import pid_control
import wheel_speed_estimation


def test_open_vs_closed_loop(tmp_path):
    png = tmp_path / "ovc.png"
    r = open_vs_closed_loop.main(["--plot", str(png)])
    assert png.stat().st_size > 10_000
    open_loop = r["open loop, duty 0.5"]
    assert open_loop["right_ticks"] / open_loop["left_ticks"] == pytest.approx(0.94, abs=0.01)
    assert open_loop["heading_deg"] < -8.0, "the weaker right motor turns the robot right"
    assert abs(r["sync feedback on ticks"]["right_ticks"] - r["sync feedback on ticks"]["left_ticks"]) < 30
    assert abs(r["firmware speed loop"]["heading_deg"]) < 4.0
    assert r["open loop, battery at 10%"]["left_rad_s"] < 0.8 * open_loop["left_rad_s"]


def test_motor_step_response(tmp_path):
    png, csv = tmp_path / "step.png", tmp_path / "step.csv"
    r = motor_step_response.main(["--quick", "--plot", str(png), "--csv", str(csv)])
    assert png.exists() and csv.exists()
    assert r["left_deadband"] == pytest.approx(0.12, abs=0.02)
    assert r["tau_ticks"] == pytest.approx(0.08, abs=0.01)
    assert r["tau_63"] > r["tau_ticks"], "the filtered speed estimate adds its own lag"
    assert r["validation_rms"] < 0.8


def test_wheel_speed_estimation(tmp_path):
    png = tmp_path / "est.png"
    r = wheel_speed_estimation.main(["--plot", str(png)])
    assert png.exists()
    assert r["raw 1 ms"]["noise"] > 5 * r["raw 10 ms"]["noise"]
    assert r["raw 50 ms"]["lag_ms"] == pytest.approx(25, abs=3)
    assert r["MA 10"]["lag_ms"] > r["MA 2"]["lag_ms"]
    assert r["dt from the robot's stamps"]["rms"] < 0.2 < r["dt = nominal 20 ms"]["rms"]


def test_p_control(tmp_path):
    r = p_control.main(["--solution", "--plot", str(tmp_path / "p.png")])
    assert r[(0.1, 0)]["settled"] == pytest.approx(r[(0.1, 0)]["predicted"], abs=0.15)
    assert r[(0.1, 0)]["wobble"] < 0.2 and r[(0.8, 0)]["wobble"] > 0.5
    assert r[(0.1, 1)]["wobble"] < 0.2 and r[(0.2, 1)]["wobble"] > 0.5, "delay lowers the gain that oscillates"


def test_pi_control(tmp_path):
    r = pi_control.main(["--solution", "--plot", str(tmp_path / "pi.png")])
    assert r["ki=2.0"]["settled"] == pytest.approx(8.0, abs=0.05)
    assert r["ki=0.0"]["settled"] < 5.0
    assert r["anti_windup=False"]["recovery"] > 3 * r["anti_windup=True"]["recovery"]


def test_pid_control(tmp_path):
    r = pid_control.main(["--solution", "--plot", str(tmp_path / "pid.png")])
    assert r["kick: the error"]["d_term_at_step"] > 0.9
    assert abs(r["kick: the measurement"]["d_term_at_step"]) < 0.05
    assert r["damping: PID, no filter"]["overshoot"] < 0.5 * r["damping: PI (kd = 0)"]["overshoot"]
    assert r["noise: PID, no filter"]["chatter"] > 1.5 * r["noise: PI (kd = 0)"]["chatter"]


def test_p_control_over_the_serial_stack():
    """--fake: the same script through SerialBase, the protocol and a fake Pico, in real time."""
    r = p_control.main(["--solution", "--fake", "--kp", "0.1", "--delays", "0"])
    settled = r[(0.1, 0)]["settled"]
    assert math.isfinite(settled) and 3.0 < settled < 6.5
