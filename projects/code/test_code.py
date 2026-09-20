"""Tests for the project helpers. No hardware, no ROS, runs in under a second.

    py -m pytest projects/code
"""

import math

import pytest

import path_error
import rate_check
import report
import safety_envelope


# ---------------------------------------------------------------- report.py


def test_criterion_parses_both_operators():
    assert report.Criterion.parse("abs_mean<=15") == report.Criterion("abs_mean", "<=", 15.0)
    assert report.Criterion.parse(" n >= 5 ") == report.Criterion("n", ">=", 5.0)


@pytest.mark.parametrize("text", ["abs_mean<15", "wobble<=3", "sd==2", "sd<=x"])
def test_criterion_rejects_nonsense(text):
    with pytest.raises(ValueError):
        report.Criterion.parse(text)


def test_summarize_separates_bias_from_spread():
    # A robot that always ends 10 cm short, +-1 cm: big bias, small spread.
    stats = report.summarize([-9.0, -10.0, -11.0, -10.0])
    assert stats["mean"] == pytest.approx(-10.0)
    assert stats["abs_mean"] == pytest.approx(10.0)
    assert stats["sd"] == pytest.approx(0.8165, abs=1e-3)
    assert stats["abs_max"] == pytest.approx(11.0)
    assert stats["range"] == pytest.approx(2.0)
    assert stats["n"] == 4


def test_summarize_sign_cancelling_is_visible_in_abs_mean():
    # Sometimes left, sometimes right: no bias to calibrate away, but the robot is still wrong.
    stats = report.summarize([-10.0, 10.0, -10.0, 10.0])
    assert stats["mean"] == pytest.approx(0.0)
    assert stats["abs_mean"] == pytest.approx(10.0)


def test_sd_of_one_trial_is_zero_not_an_error():
    assert report.summarize([3.0])["sd"] == 0.0


def test_summarize_rejects_no_trials():
    with pytest.raises(ValueError):
        report.summarize([])


def test_percentile_abs_interpolates():
    assert report.percentile_abs([0.0, 1.0, 2.0, 3.0, 4.0], 50.0) == pytest.approx(2.0)
    assert report.percentile_abs([0.0, 10.0], 95.0) == pytest.approx(9.5)
    assert report.percentile_abs([-7.0], 95.0) == pytest.approx(7.0)


def test_evaluate_reports_pass_and_fail_in_order():
    values = [12.0, -14.0, 16.0, 11.0, -13.0]
    criteria = [report.Criterion.parse(c) for c in ("abs_mean<=15", "abs_max<=15", "n>=5")]
    results = report.evaluate(values, criteria)
    assert [r.passed for r in results] == [True, False, True]
    assert results[1].value == pytest.approx(16.0)


def test_markdown_table_marks_failures():
    values = [30.0, 31.0]
    results = report.evaluate(values, [report.Criterion.parse("abs_mean<=15")])
    table = report.markdown_table(values, results, unit="cm")
    assert "**FAIL**" in table
    assert "n = 2 trials" in table
    assert "| Criterion | Measured | Verdict |" in table


def test_read_column_skips_blanks_and_reports_missing_column(tmp_path):
    csv_path = tmp_path / "trials.csv"
    csv_path.write_text("run,closing_cm\n1,12.0\n2,\n3,-9.5\n", encoding="utf-8")
    assert report.read_column(csv_path, "closing_cm") == [12.0, -9.5]
    with pytest.raises(KeyError):
        report.read_column(csv_path, "heading_deg")


def test_cli_exit_code_is_the_verdict(tmp_path, capsys):
    csv_path = tmp_path / "trials.csv"
    csv_path.write_text("closing_cm\n5\n-6\n7\n", encoding="utf-8")
    ok = report.main([str(csv_path), "--column", "closing_cm", "--criterion", "abs_max<=10", "--unit", "cm"])
    bad = report.main([str(csv_path), "--column", "closing_cm", "--criterion", "abs_max<=3"])
    assert (ok, bad) == (0, 1)
    assert "pass" in capsys.readouterr().out


# ------------------------------------------------------------ path_error.py


def test_wrap_angle_takes_the_short_way():
    assert path_error.wrap_angle(math.radians(260)) == pytest.approx(math.radians(-100))
    assert path_error.wrap_angle(math.pi) == pytest.approx(math.pi)
    assert path_error.wrap_angle(-math.pi) == pytest.approx(math.pi)


def test_heading_error_crosses_the_wrap():
    # measured +171 deg, wanted -179 deg: 10 degrees apart, not 350.
    assert path_error.heading_error_deg(math.radians(171), math.radians(-179)) == pytest.approx(-10.0)


def test_cross_track_is_positive_to_the_left():
    # Segment along +x; a point at y = +0.05 is 5 cm to the left of the direction of travel.
    assert path_error.cross_track_m((1.0, 0.05), (0.0, 0.0), (2.0, 0.0)) == pytest.approx(0.05)
    assert path_error.cross_track_m((1.0, -0.05), (0.0, 0.0), (2.0, 0.0)) == pytest.approx(-0.05)
    # Same point, segment reversed: the sign flips because "left" flipped.
    assert path_error.cross_track_m((1.0, 0.05), (2.0, 0.0), (0.0, 0.0)) == pytest.approx(-0.05)


def test_cross_track_on_a_diagonal():
    # Segment along y = x; the point (1, 0) is sqrt(2)/2 to the right of it.
    value = path_error.cross_track_m((1.0, 0.0), (0.0, 0.0), (2.0, 2.0))
    assert value == pytest.approx(-math.sqrt(2) / 2)


def test_zero_length_segment_is_an_error():
    with pytest.raises(ValueError):
        path_error.cross_track_m((1.0, 1.0), (0.0, 0.0), (0.0, 0.0))


def test_distance_to_segment_clamps_at_the_ends():
    # Beyond the end of the segment the nearest point is the endpoint, not the infinite line.
    assert path_error.distance_to_segment_m((3.0, 0.0), (0.0, 0.0), (2.0, 0.0)) == pytest.approx(1.0)
    assert path_error.distance_to_segment_m((1.0, 0.5), (0.0, 0.0), (2.0, 0.0)) == pytest.approx(0.5)


def test_closing_error_and_path_length_of_a_square():
    side = 2.0
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.10, 0.05)]
    poses = [path_error.Pose(t=float(i), x=x, y=y) for i, (x, y) in enumerate(corners)]
    assert path_error.path_length_m(poses) == pytest.approx(3 * side + math.hypot(0.10, side - 0.05))
    assert path_error.closing_error_m(poses) == pytest.approx(math.hypot(0.10, 0.05))


def test_closing_error_needs_two_poses():
    with pytest.raises(ValueError):
        path_error.closing_error_m([path_error.Pose(0.0, 0.0, 0.0)])


def test_segment_reports_find_a_constant_offset():
    # The robot drives parallel to the line, 4 cm to the left of it, the whole way.
    waypoints = [(0.0, 0.0), (3.0, 0.0)]
    poses = [path_error.Pose(t=i * 0.1, x=i * 0.1, y=0.04) for i in range(31)]
    (segment,) = path_error.segment_reports(poses, waypoints)
    assert segment.samples == 31
    assert segment.mean_m == pytest.approx(0.04)
    assert segment.abs_max_m == pytest.approx(0.04)


def test_segment_reports_assign_every_pose_of_an_L_path():
    waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    poses = [path_error.Pose(t=0.0, x=0.5, y=0.0), path_error.Pose(t=1.0, x=1.0, y=0.5)]
    reports = path_error.segment_reports(poses, waypoints)
    assert [r.samples for r in reports] == [1, 1]


def test_waypoint_misses():
    waypoints = [(0.0, 0.0), (2.0, 0.0)]
    poses = [path_error.Pose(t=0.0, x=0.0, y=0.0), path_error.Pose(t=1.0, x=1.90, y=0.05)]
    misses = path_error.waypoint_misses_m(poses, waypoints)
    assert misses[0] == pytest.approx(0.0)
    assert misses[1] == pytest.approx(math.hypot(0.10, 0.05))


def test_parse_waypoints():
    assert path_error.parse_waypoints("0,0; 2,0 ; 2,2") == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    with pytest.raises(ValueError):
        path_error.parse_waypoints("0,0")
    with pytest.raises(ValueError):
        path_error.parse_waypoints("0,0; 1")


def test_cli_reads_a_pose_log(tmp_path, capsys):
    csv_path = tmp_path / "pose.csv"
    csv_path.write_text("t,x,y,theta\n0,0,0,0\n1,2,0,0\n2,2,2,1.5708\n3,0.05,0.02,3.1416\n",
                        encoding="utf-8")
    assert path_error.main([str(csv_path), "--waypoints", "0,0; 2,0; 2,2; 0,0",
                            "--final-heading-deg", "180"]) == 0
    out = capsys.readouterr().out
    assert "closing error" in out
    assert "worst XTE" in out


# ------------------------------------------------------- safety_envelope.py


def test_braking_distance_is_quadratic_in_speed():
    # Doubling the speed quadruples the braking distance. This is the whole point.
    slow = safety_envelope.braking_distance_m(0.20, 1.0)
    fast = safety_envelope.braking_distance_m(0.40, 1.0)
    assert slow == pytest.approx(0.02)
    assert fast == pytest.approx(4.0 * slow)


def test_stopping_distance_splits_blind_and_braking():
    e = safety_envelope.envelope(0.30, reaction_s=0.16, decel_m_s2=1.0)
    assert e.reaction_distance_m == pytest.approx(0.048)
    assert e.braking_distance_m == pytest.approx(0.045)
    assert e.stopping_distance_m == pytest.approx(0.093)
    assert e.fits_in(0.10) and not e.fits_in(0.05)


def test_max_safe_speed_inverts_stopping_distance():
    for clearance in (0.10, 0.35, 1.0):
        v = safety_envelope.max_safe_speed_m_s(clearance, reaction_s=0.16, decel_m_s2=1.0)
        assert safety_envelope.stopping_distance_m(v, 0.16, 1.0) == pytest.approx(clearance)


def test_max_safe_speed_with_no_latency_is_the_textbook_root():
    # r = 0 collapses to v = sqrt(2 a d).
    assert safety_envelope.max_safe_speed_m_s(0.5, 0.0, 1.0) == pytest.approx(math.sqrt(1.0))


def test_reaction_time_adds_the_parts():
    # 30 ms link p99 + one 20 Hz period + 80 ms motor + 100 ms blocking sensor read.
    assert safety_envelope.reaction_time_s(0.030, 20.0, 0.08, 0.10) == pytest.approx(0.26)


def test_decel_from_coast_matches_braking_distance():
    a = safety_envelope.decel_from_coast_m_s2(0.30, 0.045)
    assert a == pytest.approx(1.0)
    assert safety_envelope.braking_distance_m(0.30, a) == pytest.approx(0.045)


@pytest.mark.parametrize("call", [
    lambda: safety_envelope.stopping_distance_m(-0.1, 0.2, 1.0),
    lambda: safety_envelope.stopping_distance_m(0.1, -0.2, 1.0),
    lambda: safety_envelope.stopping_distance_m(0.1, 0.2, 0.0),
    lambda: safety_envelope.max_safe_speed_m_s(-1.0, 0.2, 1.0),
    lambda: safety_envelope.decel_from_coast_m_s2(0.3, 0.0),
    lambda: safety_envelope.reaction_time_s(0.01, 0.0),
])
def test_safety_envelope_rejects_impossible_inputs(call):
    with pytest.raises(ValueError):
        call()


def test_safety_envelope_cli_verdict(capsys):
    ok = safety_envelope.main(["--speeds", "0.10,0.20", "--reaction", "0.16", "--clearance", "0.15"])
    bad = safety_envelope.main(["--speeds", "0.10,0.50", "--reaction", "0.16", "--clearance", "0.15"])
    assert (ok, bad) == (0, 1)
    out = capsys.readouterr().out
    assert "**no**" in out
    assert "fastest speed that stops within 15 cm" in out


# ------------------------------------------------------------ rate_check.py


def test_gaps_of_a_steady_stream():
    times = [i * 0.05 for i in range(21)]
    gaps = rate_check.gaps_ms(times)
    assert len(gaps) == 20
    assert max(gaps) == pytest.approx(50.0)


def test_gaps_need_two_timestamps():
    with pytest.raises(ValueError):
        rate_check.gaps_ms([1.0])


def test_out_of_order_arrivals_are_sorted_not_negative():
    gaps = rate_check.gaps_ms([0.0, 0.10, 0.05, 0.15])
    assert min(gaps) > 0.0
    assert gaps == pytest.approx([50.0, 50.0, 50.0])


def test_a_good_mean_rate_can_hide_a_stall():
    # 40 Hz for 2 s, then nothing for 2 s: 20 Hz on average, and unusable.
    times = [i * 0.025 for i in range(81)] + [4.0]
    rate, gaps = rate_check.summarize("/odom", times)
    assert rate.mean_hz == pytest.approx(20.25, abs=0.1)
    assert rate.max_gap_ms == pytest.approx(2000.0)
    assert rate.gaps_over(150.0, gaps) == 1


def test_expectation_parsing():
    assert rate_check.Expectation.parse("/odom=20") == rate_check.Expectation("/odom", 20.0, None)
    assert rate_check.Expectation.parse(" /battery_state = 1:2000 ") == \
        rate_check.Expectation("/battery_state", 1.0, 2000.0)


@pytest.mark.parametrize("text", ["/odom", "=20", "/odom=0", "/odom=20:0", "/odom=x"])
def test_expectation_rejects_nonsense(text):
    with pytest.raises(ValueError):
        rate_check.Expectation.parse(text)


def test_evaluate_passes_a_healthy_topic_and_fails_a_stalling_one():
    steady = [i * 0.05 for i in range(101)]
    rate, gaps = rate_check.summarize("/odom", steady)
    expectation = rate_check.Expectation.parse("/odom=20")
    verdicts = rate_check.evaluate(rate, gaps, expectation, default_max_gap_ms=150.0)
    assert [v.passed for v in verdicts] == [True, True]

    stalling = steady + [6.0]
    rate, gaps = rate_check.summarize("/odom", stalling)
    verdicts = rate_check.evaluate(rate, gaps, expectation, default_max_gap_ms=150.0)
    assert [v.passed for v in verdicts] == [False, False]


def test_evaluate_skips_the_gap_check_when_no_limit_is_given():
    rate, gaps = rate_check.summarize("/odom", [i * 0.05 for i in range(21)])
    verdicts = rate_check.evaluate(rate, gaps, rate_check.Expectation.parse("/odom=20"))
    assert len(verdicts) == 1


def test_read_stamps_groups_by_topic_and_checks_columns(tmp_path):
    csv_path = tmp_path / "stamps.csv"
    csv_path.write_text("topic,t\n/odom,0.0\n/odom,0.05\n/battery_state,0.01\n,\n", encoding="utf-8")
    stamps = rate_check.read_stamps(csv_path)
    assert stamps["/odom"] == [0.0, 0.05]
    assert stamps["/battery_state"] == [0.01]

    wrong = tmp_path / "wrong.csv"
    wrong.write_text("name,time\na,1\n", encoding="utf-8")
    with pytest.raises(KeyError):
        rate_check.read_stamps(wrong)


def test_rate_check_cli_reports_a_missing_topic_as_a_failure(tmp_path, capsys):
    csv_path = tmp_path / "stamps.csv"
    rows = ["topic,t"] + [f"/odom,{i * 0.05:.3f}" for i in range(101)]
    csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    ok = rate_check.main([str(csv_path), "--expect", "/odom=20", "--max-gap", "150"])
    missing = rate_check.main([str(csv_path), "--expect", "/scan=10"])
    assert (ok, missing) == (0, 1)
    out = capsys.readouterr().out
    assert "not published" in out
    assert "| Topic | n | Window | Mean rate |" in out
