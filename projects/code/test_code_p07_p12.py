"""Tests for the P07-P12 project helpers. No hardware, no ROS, runs in under a second.

    py -m pytest projects/code
"""

import math

import pytest

import detect_eval
import map_check
import mission_report
import traj_compare
from path_error import Pose


# ------------------------------------------------------------ traj_compare.py


def straight_line(n=11, speed=0.2, dt=0.5, theta=0.0, t0=0.0, x0=0.0, y0=0.0):
    """A robot driving straight at a constant speed, sampled every ``dt``."""
    return [Pose(t=t0 + i * dt,
                 x=x0 + i * dt * speed * math.cos(theta),
                 y=y0 + i * dt * speed * math.sin(theta),
                 theta=theta)
            for i in range(n)]


def test_rebase_pose_puts_the_run_at_the_origin_facing_x():
    # The same straight 1 m drive, recorded in a world rotated 90 deg and offset by (5, -3).
    poses = straight_line(theta=math.pi / 2, t0=1000.0, x0=5.0, y0=-3.0)
    rebased = traj_compare.rebase(poses, "pose")
    assert rebased[0] == Pose(0.0, 0.0, 0.0, 0.0)
    assert rebased[-1].x == pytest.approx(1.0)
    assert rebased[-1].y == pytest.approx(0.0, abs=1e-9)
    assert rebased[-1].theta == pytest.approx(0.0)


def test_rebase_time_keeps_the_coordinates():
    poses = straight_line(t0=1000.0, x0=5.0)
    rebased = traj_compare.rebase(poses, "time")
    assert rebased[0].t == 0.0
    assert rebased[0].x == 5.0


def test_rebase_rejects_an_unknown_alignment():
    with pytest.raises(ValueError):
        traj_compare.rebase(straight_line(), "magnetic-north")


def test_interpolate_between_samples_and_across_the_wrap():
    poses = [Pose(0.0, 0.0, 0.0, math.radians(170.0)),
             Pose(1.0, 2.0, 0.0, math.radians(-170.0))]
    middle = traj_compare.interpolate(poses, 0.5)
    assert middle.x == pytest.approx(1.0)
    # 170 -> -170 is +20 deg the short way, so the midpoint is 180, not 0.
    assert abs(math.degrees(middle.theta)) == pytest.approx(180.0)


def test_interpolate_outside_the_log_is_an_error():
    with pytest.raises(ValueError):
        traj_compare.interpolate(straight_line(), 999.0)


def test_identical_runs_have_zero_divergence():
    d = traj_compare.compare(straight_line(), straight_line(), hz=10.0)
    assert d.max_m == pytest.approx(0.0, abs=1e-9)
    assert d.final_m == pytest.approx(0.0, abs=1e-9)
    assert d.path_a_m == pytest.approx(1.0)


def test_a_slower_twin_diverges_along_the_path():
    # Sim drives 0.2 m/s for 5 s (1.00 m); the real robot manages 0.18 m/s (0.90 m).
    sim = straight_line(speed=0.20)
    real = straight_line(speed=0.18)
    d = traj_compare.compare(sim, real, hz=10.0)
    assert d.final_m == pytest.approx(0.10, abs=1e-6)
    assert d.max_m == pytest.approx(0.10, abs=1e-6)
    assert d.heading_max_deg == pytest.approx(0.0, abs=1e-9)
    assert d.final_percent_of_path == pytest.approx(10.0, abs=0.01)


def test_heading_difference_is_reported_in_degrees():
    a = straight_line(theta=0.0)
    b = [Pose(p.t, p.x, p.y, math.radians(12.0)) for p in straight_line(theta=0.0)]
    # --align pose would rotate that 12 deg away, so compare in the raw frame.
    d = traj_compare.compare(a, b, hz=10.0, alignment="time")
    assert d.heading_max_deg == pytest.approx(12.0)


def test_logs_that_do_not_overlap_in_time_are_an_error():
    # Raw clocks 500 s apart: this is what you get if you forget to rebase the time.
    a = straight_line(t0=0.0)
    b = straight_line(t0=500.0)
    with pytest.raises(ValueError):
        traj_compare.compare(a, b, hz=10.0, alignment="none")
    # ... and rebasing fixes it.
    assert traj_compare.compare(a, b, hz=10.0, alignment="time").max_m == pytest.approx(0.0)


def test_check_reports_each_limit_in_order():
    d = traj_compare.compare(straight_line(speed=0.20), straight_line(speed=0.18), hz=10.0)
    # check() reports rms, worst, final, heading in that order, whatever order they were passed.
    rows = traj_compare.check(d, max_final_m=0.15, max_separation_m=0.05)
    assert [detail.split()[0] for detail, _, _ in rows] == ["worst", "final"]
    assert [passed for _, _, passed in rows] == [False, True]


def test_traj_compare_cli_exit_code_is_the_verdict(tmp_path, capsys):
    def write(name, poses):
        path = tmp_path / name
        path.write_text("t,x,y,theta\n" + "".join(
            f"{p.t},{p.x},{p.y},{p.theta}\n" for p in poses), encoding="utf-8")
        return path

    a = write("sim.csv", straight_line(speed=0.20))
    b = write("real.csv", straight_line(speed=0.18))
    assert traj_compare.main([str(a), str(b), "--max-final", "0.15"]) == 0
    assert traj_compare.main([str(a), str(b), "--max-final", "0.05"]) == 1
    assert "final separation" in capsys.readouterr().out


# --------------------------------------------------------------- map_check.py


def write_map(tmp_path, rows, resolution=0.05, origin=(0.0, 0.0), name="m", free_thresh=0.196):
    """Write a P5 PGM + YAML from a list of strings: '#' occupied, '.' free, '?' unknown."""
    values = {"#": 0, ".": 254, "?": map_check.UNKNOWN_GREY}
    height, width = len(rows), len(rows[0])
    data = bytes(values[c] for row in rows for c in row)
    pgm = tmp_path / f"{name}.pgm"
    pgm.write_bytes(f"P5\n# test map\n{width} {height}\n255\n".encode() + data)
    yaml_path = tmp_path / f"{name}.yaml"
    yaml_path.write_text(
        f"image: {name}.pgm\nresolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: {free_thresh}\n", encoding="utf-8")
    return yaml_path


ROOM = [
    "########",
    "#......#",
    "#......#",
    "#..??..#",
    "#......#",
    "########",
]


def test_load_classifies_the_three_states(tmp_path):
    grid = map_check.OccupancyMap.load(write_map(tmp_path, ROOM))
    counts = grid.counts()
    assert grid.width == 8 and grid.height == 6
    assert counts["unknown"] == 2
    assert counts["occupied"] == 8 + 8 + 4 * 2       # two full rows plus the side walls
    assert counts["free"] == 48 - counts["unknown"] - counts["occupied"]
    assert grid.unknown_percent() == pytest.approx(100.0 * 2 / 48)
    assert grid.misread_unknown_cells() == 0


def test_a_rounder_free_thresh_silently_turns_unknown_into_floor(tmp_path):
    grid = map_check.OccupancyMap.load(write_map(tmp_path, ROOM, free_thresh=0.25))
    assert grid.counts()["unknown"] == 0
    assert grid.misread_unknown_cells() == 2
    assert "free_thresh" in map_check.threshold_warning(grid)


def test_negate_inverts_the_meaning(tmp_path):
    yaml_path = write_map(tmp_path, ROOM)
    yaml_path.write_text(yaml_path.read_text(encoding="utf-8").replace("negate: 0", "negate: 1"),
                         encoding="utf-8")
    grid = map_check.OccupancyMap.load(yaml_path)
    # With negate, the black wall pixels read as free and the white floor as occupied.
    assert grid.at(0, 0) == map_check.FREE


def test_world_to_cell_puts_row_zero_at_the_top(tmp_path):
    grid = map_check.OccupancyMap.load(write_map(tmp_path, ROOM, resolution=0.10))
    # The map is 0.8 x 0.6 m with its origin at (0, 0), so y = 0.55 is the top row.
    assert grid.world_to_cell(0.05, 0.55) == (0, 0)
    assert grid.world_to_cell(0.05, 0.05) == (0, 5)


def test_area_uses_the_resolution(tmp_path):
    grid = map_check.OccupancyMap.load(write_map(tmp_path, ROOM, resolution=0.10))
    assert grid.area_m2(map_check.FREE) == pytest.approx(grid.counts()["free"] * 0.01)


def test_distance_to_open_is_zero_on_the_surface_of_a_wall(tmp_path):
    grid = map_check.OccupancyMap.load(write_map(tmp_path, [
        "????????",
        "?######?",
        "?######?",
        "????????"], name="slab"))
    d = map_check.distance_to_open(grid)
    assert d[0] == 0                       # a non-occupied cell has no distance to itself
    # Every cell of a two-cell-thick slab touches open space, so none is further than one step.
    assert max(d) == 1
    assert map_check.cell_thickness(grid) == [1] * 12


def test_wall_thickness_sees_a_smeared_wall(tmp_path):
    # A one-cell wall in a 5 cm map is 5 cm thick; entering the same room three times smears it.
    crisp = ["????????",
             "?######?",
             "?#....#?",
             "?#....#?",
             "?######?",
             "????????"]
    smeared = ["????????",
               "?######?",
               "?######?",
               "?######?",
               "?######?",
               "????????"]
    crisp_median, crisp_p95, n = map_check.wall_thickness_cm(map_check.OccupancyMap.load(
        write_map(tmp_path, crisp, name="crisp")))
    smeared_median, smeared_p95, _ = map_check.wall_thickness_cm(map_check.OccupancyMap.load(
        write_map(tmp_path, smeared, name="smeared")))
    assert crisp_median == pytest.approx(5.0) and crisp_p95 == pytest.approx(5.0)
    assert n == 6 * 4 - 4 * 2          # every occupied cell of the ring
    # The outside of the smeared block is still one cell, so only the p95 moves.
    assert smeared_median == pytest.approx(5.0)
    assert smeared_p95 == pytest.approx(15.0)


def test_wall_thickness_of_a_map_with_no_measurable_walls_is_zero(tmp_path):
    empty = ["........"] * 4
    assert map_check.wall_thickness_cm(map_check.OccupancyMap.load(
        write_map(tmp_path, empty, name="empty"))) == (0.0, 0.0, 0)


def test_measurement_scale_error():
    m = map_check.Measurement.parse("0,0; 4.08,0 = 4.00")
    assert m.map_m == pytest.approx(4.08)
    assert m.error_m == pytest.approx(0.08)
    assert m.scale_error_percent == pytest.approx(2.0)


@pytest.mark.parametrize("text", ["0,0; 1,1", "0,0 = 1.0", "0,0; 1,1; 2,2 = 1.0", "0; 1 = 1.0"])
def test_measurement_rejects_nonsense(text):
    with pytest.raises(ValueError):
        map_check.Measurement.parse(text)


def test_map_check_cli_verdict(tmp_path, capsys):
    yaml_path = write_map(tmp_path, ROOM)
    assert map_check.main([str(yaml_path), "--measure", "0,0; 4.02,0 = 4.00",
                           "--max-scale-error", "1"]) == 0
    assert map_check.main([str(yaml_path), "--measure", "0,0; 4.20,0 = 4.00",
                           "--max-scale-error", "1"]) == 1
    assert "scale error" in capsys.readouterr().out


def test_map_check_rejects_a_file_that_is_not_a_pgm(tmp_path):
    yaml_path = write_map(tmp_path, ROOM)
    (tmp_path / "m.pgm").write_bytes(b"\x89PNG\r\n\x1a\n")
    with pytest.raises(ValueError):
        map_check.OccupancyMap.load(yaml_path)


# ---------------------------------------------------------- mission_report.py


def attempts(outcomes, goal="kitchen", **columns):
    return [mission_report.Attempt(trial=str(i), group=goal, outcome=o,
                                   duration_s=columns.get("durations", [None] * len(outcomes))[i],
                                   min_clearance_m=columns.get(
                                       "clearances", [None] * len(outcomes))[i],
                                   error_code=columns.get("codes", [""] * len(outcomes))[i])
            for i, o in enumerate(outcomes)]


def test_wilson_is_honest_about_a_perfect_run():
    low, high = mission_report.wilson_interval(20, 20)
    assert high == pytest.approx(1.0)
    assert low == pytest.approx(0.839, abs=0.002)


def test_wilson_never_leaves_zero_to_one():
    low, high = mission_report.wilson_interval(0, 5)
    assert low == 0.0
    assert 0.0 < high < 1.0


@pytest.mark.parametrize("successes,n", [(-1, 5), (6, 5), (0, 0)])
def test_wilson_rejects_impossible_counts(successes, n):
    with pytest.raises(ValueError):
        mission_report.wilson_interval(successes, n)


def test_summarize_counts_successes_and_names_failures():
    report = mission_report.summarize(attempts(
        ["success", "success", "aborted", "success", "aborted"],
        codes=["", "", "105", "", "105"]))
    assert report.successes == 3
    assert report.success_rate == pytest.approx(0.6)
    assert report.failures["aborted (105)"] == 2


def test_durations_come_from_successes_only():
    # A 92 s abort must not be averaged in as a slow arrival.
    report = mission_report.summarize(attempts(
        ["success", "success", "aborted"], durations=[40.0, 50.0, 92.0]))
    assert report.median_duration_s == pytest.approx(45.0)
    assert report.p95_duration_s == pytest.approx(49.5)


def test_worst_clearance_includes_the_failures():
    report = mission_report.summarize(attempts(
        ["success", "collision"], clearances=[0.25, 0.00]))
    assert report.worst_clearance_m == pytest.approx(0.0)


def test_by_group_adds_an_overall_row():
    rows = attempts(["success", "aborted"], goal="kitchen") + attempts(["success"], goal="door")
    reports = mission_report.by_group(rows)
    assert [r.group for r in reports] == ["kitchen", "door", "ALL"]
    assert reports[-1].n == 3


def test_check_evaluates_every_declared_limit():
    report = mission_report.summarize(attempts(
        ["success"] * 18 + ["aborted", "collision"],
        clearances=[0.20] * 19 + [0.0], codes=[""] * 19 + ["203"]))
    rows = mission_report.check(report, min_success_rate=0.9, min_clearance_m=0.10,
                                forbid=["collision"], min_trials=20)
    assert [passed for _, _, passed in rows] == [True, True, False, False]


def test_read_attempts_requires_an_outcome_column(tmp_path):
    path = tmp_path / "goals.csv"
    path.write_text("trial,result\n1,success\n", encoding="utf-8")
    with pytest.raises(KeyError):
        mission_report.read_attempts(path)


def test_mission_report_cli(tmp_path, capsys):
    path = tmp_path / "goals.csv"
    rows = ["trial,goal,outcome,duration_s,min_clearance_m,error_code"]
    rows += [f"{i},kitchen,success,{40 + i},0.20," for i in range(18)]
    rows += ["19,kitchen,aborted,90,0.12,105", "20,door,success,35,0.18,"]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    assert mission_report.main([str(path), "--group", "goal", "--min-success-rate", "0.9",
                                "--min-trials", "20"]) == 0
    assert mission_report.main([str(path), "--min-success-rate", "1.0"]) == 1
    assert "aborted (105)" in capsys.readouterr().out


# ------------------------------------------------------------- detect_eval.py


def box(image="f1.jpg", label="bottle", x=0.0, y=0.0, w=10.0, h=10.0, score=1.0):
    return detect_eval.Box(image, label, x, y, w, h, score)


def test_iou_of_identical_boxes_is_one():
    assert detect_eval.iou(box(), box()) == pytest.approx(1.0)


def test_iou_of_disjoint_boxes_is_zero():
    assert detect_eval.iou(box(), box(x=100.0)) == 0.0


def test_iou_of_a_half_overlap():
    # Two 10x10 boxes offset by 5 in x: intersection 50, union 150.
    assert detect_eval.iou(box(), box(x=5.0)) == pytest.approx(50.0 / 150.0)


def test_a_zero_sized_box_is_rejected():
    with pytest.raises(ValueError):
        box(w=0.0)


def test_two_boxes_on_one_object_are_one_tp_and_one_fp():
    truths = [box()]
    detections = [box(score=0.9), box(x=1.0, score=0.8)]
    matches, false_positives, missed = detect_eval.match(truths, detections, 0.5)
    assert len(matches) == 1 and len(false_positives) == 1 and missed == []
    assert matches[0][0].score == 0.9      # the confident box claimed the truth


def test_a_box_below_the_iou_threshold_is_a_false_positive():
    matches, false_positives, missed = detect_eval.match([box()], [box(x=8.0)], 0.5)
    assert matches == [] and len(false_positives) == 1 and len(missed) == 1


def test_evaluate_separates_labels_and_totals_them():
    truths = [box(label="bottle"), box(image="f2.jpg", label="cup")]
    detections = [box(label="bottle", score=0.9), box(image="f2.jpg", label="chair", score=0.7)]
    results = detect_eval.evaluate(truths, detections, 0.5)
    by_label = {r.label: r for r in results}
    assert by_label["bottle"].tp == 1 and by_label["bottle"].fp == 0
    assert by_label["cup"].fn == 1
    assert by_label["chair"].fp == 1
    assert by_label["ALL"].tp == 1 and by_label["ALL"].fp == 1 and by_label["ALL"].fn == 1
    assert by_label["ALL"].precision == pytest.approx(0.5)
    assert by_label["ALL"].recall == pytest.approx(0.5)
    assert by_label["ALL"].f1 == pytest.approx(0.5)


def test_raising_the_score_threshold_trades_recall_for_precision():
    truths = [box(image=f"f{i}.jpg") for i in range(4)]
    detections = ([box(image=f"f{i}.jpg", score=0.9) for i in range(2)]
                  + [box(image=f"f{i}.jpg", score=0.2) for i in range(2, 4)]
                  + [box(image="f9.jpg", score=0.2)])
    loose = detect_eval.evaluate(truths, detections, 0.5, min_score=0.0)[-1]
    strict = detect_eval.evaluate(truths, detections, 0.5, min_score=0.5)[-1]
    assert loose.recall == pytest.approx(1.0)
    assert loose.precision == pytest.approx(4 / 5)
    assert strict.recall == pytest.approx(0.5)
    assert strict.precision == pytest.approx(1.0)


def test_an_empty_evaluation_has_no_credit():
    result = detect_eval.evaluate([], [], 0.5)[-1]
    assert (result.tp, result.fp, result.fn) == (0, 0, 0)
    assert result.precision == 0.0 and result.recall == 0.0 and result.f1 == 0.0


def test_latency_reports_the_tail_and_the_sustainable_rate():
    latency = detect_eval.summarize_latency([80.0] * 95 + [400.0] * 5)
    assert latency.p50_ms == pytest.approx(80.0)
    assert latency.p95_ms >= 80.0
    assert latency.max_ms == pytest.approx(400.0)
    assert latency.sustained_hz == pytest.approx(1000.0 / latency.p95_ms)


def test_detect_eval_cli(tmp_path, capsys):
    truth = tmp_path / "truth.csv"
    truth.write_text("image,label,x,y,w,h\nf1.jpg,bottle,0,0,10,10\n"
                     "f2.jpg,bottle,0,0,10,10\n", encoding="utf-8")
    detections = tmp_path / "det.csv"
    detections.write_text("image,label,x,y,w,h,score\nf1.jpg,bottle,1,1,10,10,0.9\n",
                          encoding="utf-8")
    latency = tmp_path / "lat.csv"
    latency.write_text("ms\n" + "".join("90\n" for _ in range(20)), encoding="utf-8")

    assert detect_eval.main(["--truth", str(truth), "--detections", str(detections),
                             "--min-precision", "0.9", "--latency", str(latency),
                             "--max-p95-latency", "150"]) == 0
    assert detect_eval.main(["--truth", str(truth), "--detections", str(detections),
                             "--min-recall", "0.9"]) == 1
    assert "Precision" in capsys.readouterr().out
