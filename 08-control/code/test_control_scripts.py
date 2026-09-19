"""The module-08 experiment scripts run headless and produce the numbers the lessons quote.

    py -m pytest 08-control/code          (about a minute; the --fake test runs in real time)
"""

from __future__ import annotations

import math

import pytest

import control_in_ros2
import feedforward_speed
import motor_step_response
import open_vs_closed_loop
import p_control
import pi_control
import pid_control
import pid_math
import pid_tuning
import rotate_90
import straight_line
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


def test_pid_math(tmp_path):
    r = pid_math.main(["--solution", "--plot", str(tmp_path / "math.png")])
    assert r["forms_max_diff"] < 1e-9, "the positional and velocity forms are the same controller"
    assert r["windup: positional, no anti-windup"]["recovery"] > 1.0
    assert r["windup: positional, fill (firmware)"]["recovery"] < 0.3
    assert r["windup: positional, fill (firmware)"]["speed_2_1"] > 18.0, "fills the headroom: full speed"
    assert r["rate: 25 Hz"].overshoot_percent > 2 * r["rate: 100 Hz"].overshoot_percent
    assert r["rate: 100 Hz, ki*e without dt*"].overshoot_percent > 15.0
    assert r["delay: 0"]["pm"] > 50.0 > r["delay: 2"]["pm"]
    assert r["delay: 3"]["wobble"] > 10 * r["delay: 0"]["wobble"], "negative phase margin oscillates"


def test_pid_tuning(tmp_path):
    r = pid_tuning.main(["--solution", "--plot", str(tmp_path / "tuning.png")])
    fit = r["fit"]
    assert fit["K"] == pytest.approx(19.0, rel=0.1), "gain of the deadband-compensated plant"
    assert 0.07 < fit["tau"] < 0.12 and 0.005 < fit["theta"] < 0.05
    assert r["design: lambda = 0.5 x tau"]["itae"] < r["design: lambda = 2 x tau"]["itae"]
    assert r["design: Ziegler-Nichols PI"]["overshoot"] > 5 * r["design: lambda = 1 x tau"]["overshoot"]
    assert 0.4 < r["ultimate"]["ku"] < 2.0 and 0.05 < r["ultimate"]["tu"] < 0.2
    assert r["relay"]["ku"] == pytest.approx(r["ultimate"]["ku"], rel=0.5), "relay estimate, same ballpark"
    assert r["search"]["overshoot"] < 10.0
    assert r["search"]["itae"] <= r["design: lambda = 1 x tau"]["itae"]


def test_feedforward_speed(tmp_path):
    r = feedforward_speed.main(["--solution", "--plot", str(tmp_path / "ff.png")])
    assert r["fresh: feedforward + PI"]["right_error_pct"] == pytest.approx(0.0, abs=1.0)
    assert r["fresh: feedforward only"]["right_error_pct"] < -3.0, "the 6 % weak motor stays 6 % slow"
    assert r["fresh: feedforward + PI"]["right_peak_feedback"] < \
        0.6 * r["fresh: PI only (kp 0.02, ki 0.3)"]["right_peak_feedback"]
    assert r["model: no deadband term"]["right_error_pct"] < -10.0
    assert r["tired: feedforward only, nominal V"]["right_error_pct"] < -10.0
    assert abs(r["tired: feedforward + PI"]["right_error_pct"]) < 1.0
    assert r["tracking: feedforward + PI"]["rms_rad_s"] < \
        0.6 * r["tracking: PI only (kp 0.02, ki 0.3)"]["rms_rad_s"]
    assert r["floor: feedforward + PI"]["true_speed"] == pytest.approx(0.5, abs=0.02)


def test_straight_line(tmp_path):
    r = straight_line.main(["--plot", str(tmp_path / "straight.png")])
    assert abs(r["open loop (inner loops only)"]["offset_cm"]) > 15.0
    assert abs(r["encoder heading"]["offset_cm"]) > 15.0, \
        "equalising wheel rotation is what the inner loops already do"
    calibrated = next(k for k in r if k.startswith("encoder heading, calibrated"))
    assert abs(r[calibrated]["offset_cm"]) < 5.0
    assert abs(r["gyro heading, raw"]["true_heading_deg"]) > 2.0, "an uncorrected bias bends the path"
    assert abs(r["gyro heading, bias removed"]["offset_cm"]) < 6.0
    assert abs(r["gyro heading, bias removed"]["offset_cm"]) < \
        0.25 * abs(r["open loop (inner loops only)"]["offset_cm"])
    assert r["gyro heading, bias removed"]["travelled_m"] == pytest.approx(3.0, abs=0.15)


def test_rotate_90(tmp_path):
    r = rotate_90.main(["--solution", "--trials", "5", "--plot", str(tmp_path / "rot.png")])
    assert abs(r["profile + PI"]["error"]) < 2.0
    assert r["profile + PI"]["overshoot"] < 2.0
    assert r["profile + PI"]["wheel_jump"] < 0.2 * r["P only, kp = 2"]["wheel_jump"]
    assert r["P only, kp = 15"]["overshoot"] > 2.0, "a step target with a hot gain overshoots"
    assert abs(r["source: encoder difference"]["error"]) > 2.0, "the wheelbase error shows up in a turn"
    assert abs(r["source: gyro, bias removed"]["error"]) < 1.0
    assert r["repeatability"]["worst"] <= 2.0 and r["repeatability"]["std"] < 1.0
    assert r["wrap"]["end_deg"] == pytest.approx(-100.0, abs=2.0)
    assert abs(r["wrap"]["swept_deg"]) < 180.0, "the short way round"


def test_control_in_ros2(tmp_path):
    r = control_in_ros2.main(["--plot", str(tmp_path / "ros2.png")])
    assert r["Pi, 50 Hz"]["wobble"] < 0.2
    assert r["ROS 2 node, 20 Hz + 30 ms latency"]["wobble"] > 10 * r["Pi, 50 Hz"]["wobble"]
    assert r["latency"]["command_path_ms"] == pytest.approx(83.55, abs=0.1)
    assert r["limits"]["reach_time_s"] == pytest.approx(0.5, abs=0.02)
    assert r["watchdog"]["flag_s"] == pytest.approx(0.3, abs=0.02)
    assert r["watchdog"]["stopped_s"] < 0.7


def test_p_control_over_the_serial_stack():
    """--fake: the same script through SerialBase, the protocol and a fake Pico, in real time."""
    r = p_control.main(["--solution", "--fake", "--kp", "0.1", "--delays", "0"])
    settled = r[(0.1, 0)]["settled"]
    assert math.isfinite(settled) and 3.0 < settled < 6.5
