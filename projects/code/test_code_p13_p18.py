"""Tests for the P13-P18 project helpers. No hardware, no ROS, runs in under a second.

    py -m pytest projects/code
"""

import math

import pytest

import arm_accuracy
import object_map_eval
import outcome_audit
import reach_budget
import redteam_report


# ------------------------------------------------------------ object_map_eval.py


def obj(label, x, y, z=0.0, hits=0, sigma=0.0):
    return object_map_eval.MapObject(label, x, y, z, hits, sigma)


def test_a_clean_map_matches_everything():
    truth = [obj("bottle", 2.05, 1.70, 0.12), obj("cup", 1.20, 0.35, 0.05)]
    estimates = [obj("cup", 1.20, 0.348, 0.048, hits=50, sigma=0.02),
                 obj("bottle", 2.054, 1.698, 0.114, hits=39, sigma=0.02)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.6)
    assert len(report.matches) == 2
    assert report.missed == () and report.phantoms == ()
    assert report.recall == 1.0
    assert report.stat("worst") == pytest.approx(0.0081, abs=1e-3)


def test_two_entries_on_one_real_object_are_one_match_and_one_phantom():
    # The duplicate-object failure of 13.16: a gate that is too tight splits one bottle in two.
    truth = [obj("bottle", 2.00, 1.00)]
    estimates = [obj("bottle", 2.05, 1.00, hits=9), obj("bottle", 1.90, 1.00, hits=4)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.5)
    assert len(report.matches) == 1
    assert report.matches[0].estimate.hits == 9       # the closer one claimed it
    assert len(report.phantoms) == 1
    assert report.recall == 1.0                       # recall is fine; the phantom is the bug


def test_matching_never_crosses_labels():
    truth = [obj("bottle", 1.0, 1.0)]
    estimates = [obj("cup", 1.0, 1.0)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.5)
    assert report.matches == ()
    assert len(report.missed) == 1 and len(report.phantoms) == 1
    assert report.recall == 0.0


def test_an_object_outside_the_gate_is_a_miss_not_a_bad_match():
    truth = [obj("bottle", 2.0, 1.0)]
    estimates = [obj("bottle", 2.0, 2.0)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.6)
    assert report.matches == () and len(report.missed) == 1 and len(report.phantoms) == 1


def test_sigma_ratio_catches_a_dishonest_covariance():
    # 24 cm out while claiming +/- 2 cm: the classic known-size estimate with a depth camera's
    # covariance copied in. Every consumer downstream believes the 2 cm.
    truth = [obj("bottle", 2.05, 1.70)]
    estimates = [obj("bottle", 1.81, 1.70, sigma=0.02)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.6)
    assert report.matches[0].sigma_ratio == pytest.approx(12.0, abs=0.1)
    assert report.stat("max_sigma_ratio") == pytest.approx(12.0, abs=0.1)


def test_no_sigma_published_is_reported_as_none_rather_than_zero_division():
    truth = [obj("cup", 1.0, 1.0)]
    estimates = [obj("cup", 1.05, 1.0)]
    report = object_map_eval.evaluate(truth, estimates, gate_m=0.5)
    assert report.matches[0].sigma_ratio is None
    assert report.stat("max_sigma_ratio") == 0.0


def test_gate_must_be_positive():
    with pytest.raises(ValueError):
        object_map_eval.evaluate([obj("a", 0, 0)], [obj("a", 0, 0)], gate_m=0.0)


# ---------------------------------------------------------------- arm_accuracy.py


def test_accuracy_and_repeatability_are_independent():
    # Repeatable to 1 mm, wrong by 10 mm: the good case, because a consistent error is fixable.
    target = (250.0, 0.0, 120.0)
    measured = [(240.0, 0.0, 120.0), (241.0, 0.0, 120.0), (239.0, 0.0, 120.0)]
    accuracy, repeatability, mean = arm_accuracy.accuracy_and_repeatability(measured, target)
    assert accuracy == pytest.approx(10.0)
    assert repeatability == pytest.approx(1.0)
    assert mean == pytest.approx((240.0, 0.0, 120.0))


def test_scatter_with_no_bias_is_repeatability_not_accuracy():
    target = (200.0, 0.0, 100.0)
    measured = [(203.0, 0.0, 100.0), (197.0, 0.0, 100.0)]
    accuracy, repeatability, _ = arm_accuracy.accuracy_and_repeatability(measured, target)
    assert accuracy == pytest.approx(0.0)
    assert repeatability == pytest.approx(3.0)


def biased_pairs(scale=1.02, offset=(4.0, -2.0, 1.5)):
    """A measured grid produced by a known affine error, which an affine fit must undo exactly."""
    targets = [(x, y, z)
               for x in (150.0, 200.0, 250.0, 300.0)
               for y in (-100.0, 0.0, 100.0)
               for z in (20.0, 120.0)]
    pairs = []
    for t in targets:
        measured = tuple(t[i] * scale + offset[i] for i in range(3))
        pairs.append((measured, t))
    return pairs


def test_a_constant_offset_removes_a_constant_error_exactly():
    pairs = [(tuple(t[i] + 5.0 for i in range(3)), t)
             for t in [(150.0, 0.0, 20.0), (250.0, 50.0, 120.0), (200.0, -50.0, 60.0)]]
    offset = arm_accuracy.fit_offset(pairs)
    assert offset == pytest.approx((-5.0, -5.0, -5.0))
    assert max(arm_accuracy.residuals(pairs, lambda p: arm_accuracy.apply_offset(p, offset))) \
        == pytest.approx(0.0, abs=1e-9)


def test_an_affine_fit_undoes_scale_and_skew_that_an_offset_cannot():
    pairs = biased_pairs()
    offset = arm_accuracy.fit_offset(pairs)
    after_offset = max(arm_accuracy.residuals(pairs, lambda p: arm_accuracy.apply_offset(p, offset)))
    affine = arm_accuracy.fit_affine(pairs)
    after_affine = max(arm_accuracy.residuals(pairs, lambda p: arm_accuracy.apply_affine(p, affine)))
    assert after_offset > 2.0            # a scale error is not a translation
    assert after_affine == pytest.approx(0.0, abs=1e-6)


def test_a_real_correction_generalises_to_held_out_targets():
    pairs = biased_pairs()
    fit, test = arm_accuracy.holdout_split(pairs)
    affine = arm_accuracy.fit_affine(fit)
    held_out = max(arm_accuracy.residuals(test, lambda p: arm_accuracy.apply_affine(p, affine)))
    assert held_out == pytest.approx(0.0, abs=1e-6)


def test_fitting_noise_does_not_generalise():
    # Random-looking but deterministic per-target errors: an affine fit can chase them on the
    # points it saw and must do worse on the ones it did not. This is why you hold half out.
    targets = [(x, y, z)
               for x in (150.0, 200.0, 250.0, 300.0)
               for y in (-100.0, 0.0, 100.0)
               for z in (20.0, 120.0)]
    pairs = []
    for i, t in enumerate(targets):
        wobble = (6.0 * math.sin(i * 2.3), 6.0 * math.cos(i * 1.7), 6.0 * math.sin(i * 0.9))
        pairs.append((tuple(t[j] + wobble[j] for j in range(3)), t))
    fit, test = arm_accuracy.holdout_split(pairs)
    affine = arm_accuracy.fit_affine(fit)
    fitted = sum(arm_accuracy.residuals(fit, lambda p: arm_accuracy.apply_affine(p, affine))) / len(fit)
    held_out = sum(arm_accuracy.residuals(test, lambda p: arm_accuracy.apply_affine(p, affine))) / len(test)
    assert held_out > fitted


def test_the_holdout_split_does_not_line_up_with_the_grid():
    # Every other row of this grid has z = 20: an "every other target" split would hand the fit
    # a constant axis and twelve parameters. The permuted split must not.
    pairs = biased_pairs()
    fit, test = arm_accuracy.holdout_split(pairs)
    assert len(fit) == len(test) == len(pairs) // 2
    assert len({p[1][2] for p in fit}) > 1
    assert not set(map(id, fit)) & set(map(id, test))


def test_affine_needs_enough_points():
    with pytest.raises(ValueError):
        arm_accuracy.fit_affine([((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))])


def test_a_degenerate_grid_is_refused_rather_than_silently_wrong():
    # Every target on one line: twelve parameters, no information about the other two axes.
    pairs = [((x, 0.0, 0.0), (x + 1.0, 0.0, 0.0)) for x in (150.0, 200.0, 250.0, 300.0)]
    with pytest.raises(ValueError):
        arm_accuracy.fit_affine(pairs)


def test_per_target_grouping_keeps_the_order_and_counts_the_repeats():
    rows = [((150.0, 0.0, 20.0), (151.0, 0.0, 20.0)),
            ((150.0, 0.0, 20.0), (149.0, 0.0, 20.0)),
            ((250.0, 0.0, 20.0), (250.0, 0.0, 20.0))]
    reports = arm_accuracy.per_target(rows)
    assert [r.target[0] for r in reports] == [150.0, 250.0]
    assert [r.n for r in reports] == [2, 1]
    assert reports[0].dominant == "repeatability"


# ----------------------------------------------------------------- reach_budget.py


def test_errors_add_in_quadrature_not_by_addition():
    sources = [reach_budget.Source("a", 3.0), reach_budget.Source("b", 4.0)]
    assert reach_budget.quadrature_mm(sources) == pytest.approx(5.0)


def test_halving_the_smaller_source_barely_helps():
    # 15.07's arithmetic: 3 and 4 mm -> 5 mm; halving the 3 gives 4.27, a 15 % improvement.
    sources = [reach_budget.Source("small", 3.0), reach_budget.Source("big", 4.0)]
    assert reach_budget.total_if_halved(sources, "small") == pytest.approx(4.272, abs=1e-3)
    assert reach_budget.total_if_halved(sources, "big") == pytest.approx(3.606, abs=1e-3)
    assert reach_budget.share(sources, "big") == pytest.approx(16 / 25)


def test_lateral_tolerance_is_half_the_free_stroke():
    assert reach_budget.lateral_tolerance_mm(45.0, 30.0) == pytest.approx(7.5)
    assert reach_budget.lateral_tolerance_mm(45.0, 40.0) == pytest.approx(2.5)


def test_the_widest_object_a_45_mm_jaw_grasps_open_loop():
    # p90 of 9 mm needs 18 mm of free stroke, so 27 mm - which is why the course block is 30 mm
    # and why it works four times in five rather than every time.
    assert reach_budget.max_graspable_width_mm(45.0, 9.0) == pytest.approx(27.0)


def test_a_hopeless_reach_reports_a_negative_width_rather_than_pretending():
    assert reach_budget.max_graspable_width_mm(45.0, 30.0) < 0.0


def test_measure_uses_magnitudes_so_left_and_right_misses_do_not_cancel():
    m = reach_budget.measure([-8.0, 8.0, -8.0, 8.0])
    assert m.median_mm == pytest.approx(8.0)
    assert m.max_mm == pytest.approx(8.0)


def test_source_parsing_and_its_refusals():
    assert reach_budget.Source.parse(" hand-eye = 3.2 ") == reach_budget.Source("hand-eye", 3.2)
    for bad in ("hand-eye", "=3.2", "hand-eye=lots"):
        with pytest.raises(ValueError):
            reach_budget.Source.parse(bad)


# ---------------------------------------------------------------- outcome_audit.py


def attempt(reported, verified, phase="", duration=None, group="all"):
    return outcome_audit.Attempt("1", group, reported, verified, phase, duration)


def test_the_four_cells_of_the_agreement_table():
    assert attempt("success", "success").cell == "true success"
    assert attempt("success", "failure").cell == "false success"
    assert attempt("failure", "success").cell == "false failure"
    assert attempt("aborted", "failure").cell == "true failure"


def test_success_words_are_case_and_spelling_tolerant_but_nothing_else_counts():
    assert attempt("SUCCEEDED", "Ok").cell == "true success"
    assert attempt("done", "success").cell == "false failure"     # 'done' is not success


def test_a_run_that_grades_its_own_homework_is_visible_in_the_gap():
    attempts = [attempt("success", "success")] * 15 + [attempt("success", "failure")] * 5
    report = outcome_audit.audit(list(attempts))[0]
    assert report.reported_rate == pytest.approx(1.0)
    assert report.verified_rate == pytest.approx(0.75)
    assert report.count("false success") == 5


def test_wilson_interval_keeps_a_small_sample_honest():
    attempts = [attempt("success", "success")] * 16 + [attempt("failure", "failure")] * 4
    low, high = outcome_audit.audit(list(attempts))[0].interval
    assert low == pytest.approx(0.58, abs=0.02)
    assert high == pytest.approx(0.92, abs=0.02)


def test_failures_with_no_phase_are_counted_as_unrecorded_not_dropped():
    attempts = [attempt("failure", "failure", phase="grasp"),
                attempt("failure", "failure", phase=""),
                attempt("success", "success", phase="")]
    phases = outcome_audit.audit(list(attempts))[0].failure_phases
    assert phases["grasp"] == 1 and phases["unrecorded"] == 1
    assert sum(phases.values()) == 2


def test_grouping_splits_a_hidden_single_bad_object_out_of_the_average():
    attempts = ([outcome_audit.Attempt(str(i), "block", "success", "success") for i in range(15)]
                + [outcome_audit.Attempt(str(i), "can", "success", "failure") for i in range(5)])
    audits = outcome_audit.audit(attempts, by_group=True)
    by_name = {a.group: a for a in audits}
    assert by_name["all"].verified_rate == pytest.approx(0.75)
    assert by_name["block"].verified_rate == pytest.approx(1.0)
    assert by_name["can"].verified_rate == pytest.approx(0.0)


# --------------------------------------------------------------- redteam_report.py


def row(attack, rule, baseline=1, isolated="DENIED", full_stack=None):
    return redteam_report.AttackRow(attack, rule, baseline, isolated,
                                    rule if full_stack is None else full_stack)


def test_a_suite_where_every_rule_is_load_bearing_passes():
    report = redteam_report.build([row("note", "goal_lock"), row("bedroom", "geofence")],
                                  ["goal_lock", "geofence"])
    assert report.isolated_ok == 2 and report.stack_ok == 2
    assert report.not_load_bearing == () and report.got_through == ()
    assert report.untested_rules == ()


def test_a_rule_that_only_works_with_the_others_armed_is_flagged():
    # The whole point of 19.09's only(): the stack held, but not because of this rule.
    report = redteam_report.build([row("note", "goal_lock", isolated="ALLOWED")], ["goal_lock"])
    assert report.stack_ok == 1
    assert [r.attack for r in report.not_load_bearing] == ["note"]


def test_an_attack_that_does_nothing_unguarded_proves_nothing():
    report = redteam_report.build([row("harmless", "mode", baseline=0)], ["mode"])
    assert [r.attack for r in report.weak_baselines] == ["harmless"]


def test_an_attack_that_gets_through_the_full_stack_is_the_headline():
    report = redteam_report.build([row("new_one", "budget", full_stack="ALLOWED")], ["budget"])
    assert [r.attack for r in report.got_through] == ["new_one"]


def test_a_rule_with_no_attack_against_it_is_untested():
    report = redteam_report.build([row("note", "goal_lock")],
                                  ["goal_lock", "geofence", "budget"])
    assert report.untested_rules == ("geofence", "budget")


def test_being_caught_earlier_by_another_layer_is_a_note_not_a_failure():
    # 19.09's own measured result: the geofence refuses the drive before the goal lock is asked.
    r = row("note_on_the_wall", "goal_lock", isolated="GOAL_VIOLATION", full_stack="geofence")
    report = redteam_report.build([r], ["goal_lock", "geofence"])
    assert r.stopped_by_another_rule
    assert report.not_load_bearing == () and report.got_through == ()
