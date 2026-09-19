"""The module-01 measurement scripts run headless and produce the numbers the lessons quote.

    py -m pytest 01-first-robot/code       (about a minute: the fake robot runs in real time)
"""

from __future__ import annotations

import math

import pytest

import duty_calibration
import link_latency
import square_repeatability

FAST = ["--settle", "0.4", "--measure", "0.8", "--duties", "0.1", "0.2", "0.4", "0.6", "0.8", "1.0"]


def test_fit_line_recovers_a_known_line():
    gain, offset = duty_calibration.fit_line([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0])
    assert gain == pytest.approx(2.0)
    assert offset == pytest.approx(1.0)


def test_duty_calibration_finds_the_deadband_and_matching_motors():
    result = duty_calibration.main(["--fake", *FAST])
    # karmel.yaml: duty_deadband 0.12, max_wheel_speed 17 rad/s * 0.045 m = 0.765 m/s at duty 1.
    assert result["left_deadband"] == pytest.approx(0.12, abs=0.02)
    assert result["right_deadband"] == pytest.approx(0.12, abs=0.02)
    assert result["gain_ratio"] == pytest.approx(1.0, abs=0.01), "ideal motors are identical"
    assert abs(result["yaw_rate_at_full_rad_s"]) < 0.02


def test_duty_calibration_measures_the_mismatch_that_makes_the_robot_curve():
    result = duty_calibration.main(["--fake", "--realistic", *FAST])
    # DiffDriveParams.realistic(): the right motor is 6 % weaker than the left one.
    assert result["gain_ratio"] == pytest.approx(0.94, abs=0.02)
    assert math.degrees(result["yaw_rate_at_full_rad_s"]) > 8.0, "a weak right motor turns right"


def test_duty_calibration_writes_a_plot(tmp_path):
    png = tmp_path / "duty.png"
    duty_calibration.main(["--fake", "--plot", str(png), *FAST])
    assert png.stat().st_size > 10_000


def test_closure_error():
    distance, heading = square_repeatability.closure_error((0.03, -0.04, math.radians(-5)))
    assert distance == pytest.approx(0.05)
    assert math.degrees(heading) == pytest.approx(-5.0)


def test_square_repeatability_ideal_robot_comes_back():
    results = square_repeatability.main(["--fake", "--runs", "1", "--side", "0.3", "--speed", "0.25"])
    assert len(results) == 1
    truth = results[0].truth
    assert truth is not None, "the simulator must supply ground truth"
    assert square_repeatability.closure_error(truth)[0] < 0.03


def test_square_repeatability_realistic_robot_drifts_but_odometry_does_not_notice():
    results = square_repeatability.main(
        ["--fake", "--realistic", "--runs", "2", "--side", "0.3", "--speed", "0.25"]
    )
    truth_error = [square_repeatability.closure_error(r.truth)[0] for r in results]
    odom_error = [square_repeatability.closure_error(r.odom)[0] for r in results]
    assert min(truth_error) > 2 * max(odom_error), (
        "the point of the lesson: the robot's own ticks say it closed the square, reality disagrees"
    )


def test_percentiles():
    values = [float(v) for v in range(1, 101)]
    assert link_latency.percentile(values, 50) == 50.0
    assert link_latency.percentile(values, 99) == 99.0
    assert link_latency.percentile(values, 100) == 100.0


def test_link_latency_measures_a_local_link():
    result = link_latency.main(["--fake", "--pings", "10", "--listen", "1.5", "--move", "--moves", "2"])
    assert result["round_trip"]["mean_ms"] < 200
    # The fake Pico streams telemetry at 50 Hz, like the firmware default.
    assert result["telemetry_interval"]["mean_ms"] == pytest.approx(20, abs=6)
    # Command to motion is link latency + the motor time constant (80 ms in karmel.yaml).
    assert 10 < result["command_to_motion"]["mean_ms"] < 500
