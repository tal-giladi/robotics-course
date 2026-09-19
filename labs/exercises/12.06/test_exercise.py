"""Checker for 12.06 — Nav2's four behavior-tree control nodes.

Run: ``python course.py check 12.06`` (or ``--solution`` to see the reference pass).

The tests tick hand-built trees of scripted leaves, then run Nav2 Jazzy's real default tree,
``navigate_to_pose_w_replanning_and_recovery.xml``, through your nodes against fake servers.
"""

from __future__ import annotations

import pytest

S, F, R = "SUCCESS", "FAILURE", "RUNNING"


def st(impl, name: str):
    return getattr(impl.Status, name)


def leaf(impl, name: str, results: list[str]):
    return impl.ScriptedLeaf(name, [st(impl, r) for r in results])


def tick_n(impl, tree, n: int, dt: float = 0.01):
    ctx = impl.Context()
    out = []
    for _ in range(n):
        out.append(tree.execute(ctx))
        ctx.tick_index += 1
        ctx.now += dt
    return ctx, [s.value for s in out]


# --- PipelineSequence -----------------------------------------------------------------------------
def test_pipeline_reticks_the_earlier_children(impl):
    plan, follow = leaf(impl, "plan", [S]), leaf(impl, "follow", [R, R, R, S])
    ctx, out = tick_n(impl, impl.PipelineSequence("pipe", [plan, follow]), 4)
    assert out == [R, R, R, S]
    assert ctx.names().count("plan") == 4, "the planner keeps being ticked while FollowPath runs"


def test_pipeline_fails_when_a_child_fails(impl):
    ctx, out = tick_n(impl, impl.PipelineSequence("pipe", [leaf(impl, "a", [S]), leaf(impl, "b", [R, F])]), 2)
    assert out == [R, F]


def test_pipeline_succeeds_only_when_the_last_child_succeeds(impl):
    a, b, c = leaf(impl, "a", [S]), leaf(impl, "b", [R, S]), leaf(impl, "c", [R, R, S])
    ctx, out = tick_n(impl, impl.PipelineSequence("pipe", [a, b, c]), 4)
    assert out == [R, R, R, S]
    assert ctx.names() == ["a", "b"] + ["a", "b", "c"] * 3


def test_pipeline_halts_its_children_when_it_finishes(impl):
    a, b = leaf(impl, "a", [S]), leaf(impl, "b", [S])
    pipe = impl.PipelineSequence("pipe", [a, b])
    tick_n(impl, pipe, 1)
    assert pipe.last_child_ticked == 0
    assert a.status is st(impl, "IDLE") and b.status is st(impl, "IDLE")


# --- RecoveryNode ---------------------------------------------------------------------------------
def test_recovery_runs_the_recovery_and_retries_in_the_same_tick(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [F, S]), leaf(impl, "rec", [S])], number_of_retries=1)
    ctx, out = tick_n(impl, node, 1)
    assert out == [S]
    assert ctx.names() == ["main", "rec", "main"]


def test_recovery_returns_success_without_touching_the_recovery(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [S]), leaf(impl, "rec", [S])], number_of_retries=6)
    ctx, out = tick_n(impl, node, 1)
    assert out == [S] and ctx.names() == ["main"]


def test_recovery_gives_up_after_number_of_retries(impl):
    for retries in (0, 1, 3):
        node = impl.RecoveryNode("rec", [leaf(impl, "main", [F]), leaf(impl, "rec", [S])],
                                 number_of_retries=retries)
        ctx, out = tick_n(impl, node, 1)
        assert out == [F]
        assert ctx.names().count("rec") == retries
        assert ctx.names().count("main") == retries + 1


def test_recovery_fails_at_once_when_the_recovery_fails(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [F]), leaf(impl, "rec", [F])], number_of_retries=6)
    ctx, out = tick_n(impl, node, 1)
    assert out == [F] and ctx.names() == ["main", "rec"]


def test_recovery_passes_running_through_and_keeps_its_place(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [R, R, S]), leaf(impl, "rec", [S])])
    ctx, out = tick_n(impl, node, 3)
    assert out == [R, R, S] and ctx.names() == ["main", "main", "main"]


def test_recovery_waits_for_a_running_recovery(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [F, F, S]), leaf(impl, "rec", [R, R, S])],
                             number_of_retries=2)
    ctx, out = tick_n(impl, node, 3)
    assert out == [R, R, S], "the recovery is ticked until it finishes, then the main child retries"


def test_recovery_resets_its_retry_count_after_success(impl):
    node = impl.RecoveryNode("rec", [leaf(impl, "main", [F, S]), leaf(impl, "rec", [S])], number_of_retries=1)
    tick_n(impl, node, 1)
    assert node.retry_count == 0 and node.index == 0


# --- RoundRobin -----------------------------------------------------------------------------------
def test_round_robin_hands_out_one_child_per_success(impl):
    children = [leaf(impl, n, [S]) for n in "abc"]
    ctx, out = tick_n(impl, impl.RoundRobin("rr", children), 4)
    assert out == [S, S, S, S]
    assert ctx.names() == ["a", "b", "c", "a"]


def test_round_robin_skips_failures_within_one_tick(impl):
    ctx, out = tick_n(impl, impl.RoundRobin("rr", [leaf(impl, "a", [F]), leaf(impl, "b", [S]),
                                                   leaf(impl, "c", [S])]), 1)
    assert out == [S] and ctx.names() == ["a", "b"]


def test_round_robin_fails_only_when_every_child_failed(impl):
    rr = impl.RoundRobin("rr", [leaf(impl, n, [F]) for n in "abc"])
    ctx, out = tick_n(impl, rr, 1)
    assert out == [F] and ctx.names() == ["a", "b", "c"]
    assert rr.index == 0 and rr.failed == 0, "a failed RoundRobin halts itself"


def test_round_robin_does_not_move_on_while_a_child_is_running(impl):
    rr = impl.RoundRobin("rr", [leaf(impl, "a", [R, R, S]), leaf(impl, "b", [S])])
    ctx, out = tick_n(impl, rr, 4)
    assert out == [R, R, S, S]
    assert ctx.names() == ["a", "a", "a", "b"]


def test_round_robin_index_survives_a_parents_shallow_reset(impl):
    rr = impl.RoundRobin("rr", [leaf(impl, n, [S]) for n in "abc"])
    tick_n(impl, rr, 1)
    impl.BTNode.halt_child(rr)                    # finished child: shallow reset
    assert rr.index == 1, "this is what makes the recovery subtree cycle"
    rr.status = st(impl, "RUNNING")
    impl.BTNode.halt_child(rr)                    # running child: deep halt
    assert rr.index == 0


# --- RateController -------------------------------------------------------------------------------
def test_rate_controller_throttles_to_hz(impl):
    child = leaf(impl, "plan", [S])
    ctx, out = tick_n(impl, impl.RateController("rate", [child], hz=1.0), 250)
    assert out[0] == S and out[1] == R
    assert ctx.names().count("plan") == 3, "ticked at t = 0.00, 1.00 and 2.00 s"


def test_rate_controller_at_5_hz(impl):
    child = leaf(impl, "plan", [S])
    ctx, _ = tick_n(impl, impl.RateController("rate", [child], hz=5.0), 100)
    assert ctx.names().count("plan") == 5


def test_rate_controller_never_cuts_off_a_running_child(impl):
    child = leaf(impl, "plan", [R, R, R, S])
    ctx, out = tick_n(impl, impl.RateController("rate", [child], hz=1.0), 4)
    assert out == [R, R, R, S] and ctx.names().count("plan") == 4


def test_rate_controller_passes_failure_through(impl):
    ctx, out = tick_n(impl, impl.RateController("rate", [leaf(impl, "plan", [F])], hz=1.0), 1)
    assert out == [F]


# --- the whole Nav2 tree --------------------------------------------------------------------------
def navigate_tree(impl, plan_results: list[str], follow_results: list[str], retries: int = 6):
    """Nav2's navigate_to_pose_w_replanning_and_recovery.xml, built with the student's nodes.

    Only the parts that matter for the control flow: the RateController + ComputePathToPose
    pipeline stage, FollowPath with its costmap-clearing context recovery, and the RoundRobin
    recovery subtree.
    """
    plan = leaf(impl, "ComputePathToPose", plan_results)
    follow = leaf(impl, "FollowPath", follow_results)
    pipeline = impl.PipelineSequence("NavigateWithReplanning", [
        impl.RateController("RateController", [
            impl.RecoveryNode("ComputePathToPose", [plan, leaf(impl, "ClearGlobal-Context", [S])],
                              number_of_retries=1)], hz=1.0),
        impl.RecoveryNode("FollowPath", [follow, leaf(impl, "ClearLocal-Context", [S])],
                          number_of_retries=1),
    ])
    recovery = impl.Sequence("RecoverySequence", [
        impl.ReactiveFallback("RecoveryFallback", [
            leaf(impl, "GoalUpdated", [F]),
            impl.RoundRobin("RecoveryActions", [
                leaf(impl, "ClearingActions", [S]),
                leaf(impl, "Spin", [R] * 3 + [S]),
                leaf(impl, "Wait", [R] * 3 + [S]),
                leaf(impl, "BackUp", [R] * 3 + [S]),
            ]),
        ]),
    ])
    return impl.RecoveryNode("NavigateRecovery", [pipeline, recovery], number_of_retries=retries)


def test_nominal_navigation_replans_once_a_second(impl):
    tree = navigate_tree(impl, [S], [R] * 299 + [S])
    ctx = impl.Context()
    assert impl.run(tree, ctx).value == S
    assert ctx.tick_index == 299
    assert ctx.names().count("ComputePathToPose") == 3, "1 Hz over 3 s of driving"


def test_navigation_recovers_a_controller_failure_by_clearing_the_local_costmap(impl):
    tree = navigate_tree(impl, [S], [R] * 50 + [F] + [R] * 50 + [S])
    ctx = impl.Context()
    assert impl.run(tree, ctx).value == S
    assert "ClearLocal-Context" in ctx.names()
    assert "Spin" not in ctx.names(), "the context recovery handled it; no motion recovery needed"


def test_unreachable_goal_cycles_the_recovery_subtree_then_aborts(impl):
    tree = navigate_tree(impl, [F], [S])
    ctx = impl.Context()
    assert impl.run(tree, ctx).value == F
    order = [n for _, n, s in ctx.trace
             if s.value == S and n in {"ClearingActions", "Spin", "Wait", "BackUp"}]
    assert order == ["ClearingActions", "Spin", "Wait", "BackUp", "ClearingActions", "Spin"], \
        "six retries: the RoundRobin walks its four recoveries and starts again"
    assert ctx.names().count("ComputePathToPose") == 14, "7 attempts, each retried once in context"


@pytest.mark.parametrize("retries", [0, 1, 2, 4])
def test_number_of_retries_controls_how_many_recoveries_run(impl, retries):
    tree = navigate_tree(impl, [F], [S], retries=retries)
    ctx = impl.Context()
    assert impl.run(tree, ctx).value == F
    motions = [n for _, n, s in ctx.trace
               if s.value == S and n in {"ClearingActions", "Spin", "Wait", "BackUp"}]
    assert len(motions) == retries
