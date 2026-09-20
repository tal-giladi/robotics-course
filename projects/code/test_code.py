"""Tests for the project helpers. No hardware, no ROS, runs in under a second.

    py -m pytest projects/code
"""

import math

import pytest

import path_error
import report


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
