"""Tests for 12.06 — the behavior-tree engine and Nav2's default navigation tree."""

from __future__ import annotations

import behavior_tree as bt
import pytest


def scripted(name: str, results: list[bt.Status]) -> bt.Leaf:
    """A leaf that returns ``results`` one per tick, repeating the last one forever.

    The counter lives in the closure, not in ``Leaf.state``, so it survives the resets a parent
    performs — these tests are about the control nodes, not about action bookkeeping.
    """
    ticks = [0]

    def fn(ctx: bt.Context, node: bt.Leaf) -> bt.Status:
        i = ticks[0]
        ticks[0] += 1
        return results[min(i, len(results) - 1)]

    return bt.Leaf(name, fn)


def tick_n(tree: bt.BTNode, n: int, dt: float = 0.01) -> tuple[bt.Context, list[bt.Status]]:
    ctx = bt.Context()
    out = []
    for _ in range(n):
        out.append(tree.execute(ctx))
        ctx.tick_index += 1
        ctx.now += dt
    return ctx, out


S, F, R = bt.Status.SUCCESS, bt.Status.FAILURE, bt.Status.RUNNING


# --- the standard nodes ---------------------------------------------------------------------------
def test_sequence_has_memory_and_fails_fast():
    a, b = scripted("A", [S]), scripted("B", [R, R, S])
    seq = bt.Sequence("seq", [a, b])
    ctx, out = tick_n(seq, 3)
    assert out == [R, R, S]
    # A is ticked once, not on every tick: Sequence resumes at the RUNNING child
    assert [n for _, n, _ in ctx.trace] == ["A", "B", "B", "B"]


def test_fallback_returns_success_on_first_succeeding_child():
    seq = bt.Fallback("fb", [scripted("A", [F]), scripted("B", [S]), scripted("C", [S])])
    _, out = tick_n(seq, 1)
    assert out == [S]


def test_reactive_fallback_rechecks_the_condition_every_tick():
    cond, action = scripted("cond", [F, F, S]), scripted("action", [R, R, R])
    node = bt.ReactiveFallback("rf", [cond, action])
    ctx, out = tick_n(node, 3)
    assert out == [R, R, S]
    assert [n for _, n, _ in ctx.trace].count("cond") == 3, "no memory: the condition is re-ticked"


def test_pipeline_sequence_keeps_ticking_earlier_children():
    plan, follow = scripted("plan", [S]), scripted("follow", [R, R, R, S])
    pipe = bt.PipelineSequence("pipe", [plan, follow])
    ctx, out = tick_n(pipe, 4)
    assert out == [R, R, R, S]
    assert [n for _, n, _ in ctx.trace].count("plan") == 4, "the pipeline re-ticks the planner"


def test_pipeline_sequence_fails_when_any_child_fails():
    pipe = bt.PipelineSequence("pipe", [scripted("a", [S]), scripted("b", [R, F])])
    _, out = tick_n(pipe, 2)
    assert out == [R, F]


def test_recovery_node_ticks_recovery_and_main_in_the_same_tick():
    main, recovery = scripted("main", [F, S]), scripted("recovery", [S])
    node = bt.RecoveryNode("rec", [main, recovery], number_of_retries=1)
    ctx, out = tick_n(node, 1)
    assert out == [S]
    assert [n for _, n, _ in ctx.trace] == ["main", "recovery", "main"]


def test_recovery_node_gives_up_after_number_of_retries():
    main, recovery = scripted("main", [F]), scripted("recovery", [S])
    node = bt.RecoveryNode("rec", [main, recovery], number_of_retries=2)
    ctx, out = tick_n(node, 1)
    assert out == [F]
    assert [n for _, n, _ in ctx.trace].count("recovery") == 2, "two retries, then failure"


def test_recovery_node_fails_immediately_if_the_recovery_fails():
    node = bt.RecoveryNode("rec", [scripted("main", [F]), scripted("recovery", [F])], number_of_retries=6)
    _, out = tick_n(node, 1)
    assert out == [F]


def test_recovery_node_requires_two_children():
    with pytest.raises(ValueError):
        bt.RecoveryNode("rec", [scripted("only", [S])])


def test_round_robin_advances_one_child_per_success():
    children = [scripted(n, [S]) for n in "abc"]
    node = bt.RoundRobin("rr", children)
    ctx, out = tick_n(node, 4)
    assert out == [S, S, S, S]
    assert [n for _, n, _ in ctx.trace] == ["a", "b", "c", "a"]


def test_round_robin_skips_to_the_next_child_on_failure_within_one_tick():
    node = bt.RoundRobin("rr", [scripted("a", [F]), scripted("b", [S]), scripted("c", [S])])
    ctx, out = tick_n(node, 1)
    assert out == [S]
    assert [n for _, n, _ in ctx.trace] == ["a", "b"]


def test_round_robin_fails_only_when_every_child_failed():
    node = bt.RoundRobin("rr", [scripted(n, [F]) for n in "abc"])
    _, out = tick_n(node, 1)
    assert out == [F]


def test_round_robin_index_survives_a_shallow_reset_but_not_a_halt():
    node = bt.RoundRobin("rr", [scripted(n, [S]) for n in "abc"])
    tick_n(node, 1)                 # ran "a", index now 1
    bt.BTNode.halt_child(node)      # parent resets a finished child: shallow
    assert node.index == 1
    node.status = bt.Status.RUNNING
    bt.BTNode.halt_child(node)      # parent halts a RUNNING child: deep
    assert node.index == 0


def test_rate_controller_ticks_its_child_at_the_configured_rate():
    child = scripted("child", [S])
    node = bt.RateController("rate", [child], hz=1.0)
    ctx, out = tick_n(node, 250, dt=0.01)      # 2.5 s at 100 Hz
    assert out[0] is S and out[1] is R
    assert [n for _, n, _ in ctx.trace].count("child") == 3, "t=0.00, 1.00, 2.00 s"


def test_rate_controller_keeps_ticking_a_running_child():
    child = scripted("child", [R, R, R, S])
    node = bt.RateController("rate", [child], hz=1.0)
    ctx, out = tick_n(node, 4, dt=0.01)
    assert out == [R, R, R, S]
    assert [n for _, n, _ in ctx.trace].count("child") == 4


def test_inverter():
    assert tick_n(bt.Inverter("inv", [scripted("a", [S])]), 1)[1] == [F]
    assert tick_n(bt.Inverter("inv", [scripted("a", [F])]), 1)[1] == [S]


# --- the XML and the real Nav2 tree ---------------------------------------------------------------
def test_xml_round_trip_of_the_default_tree():
    tree = bt.load_tree(bt.DEFAULT_TREE.read_text(encoding="utf-8"), bt.FakeNav2().leaves())
    assert isinstance(tree, bt.RecoveryNode)
    assert tree.name == "NavigateRecovery"
    assert tree.number_of_retries == 6
    assert isinstance(tree.children[0], bt.PipelineSequence)


def test_unknown_leaf_is_a_clear_error():
    with pytest.raises(KeyError, match="no fake implementation"):
        bt.load_tree('<root BTCPP_format="4"><BehaviorTree ID="MainTree">'
                     '<Sequence><Frobnicate/></Sequence></BehaviorTree></root>', {})


def run_default(nav2: bt.FakeNav2) -> tuple[bt.Status, bt.Context]:
    tree = bt.load_tree(bt.DEFAULT_TREE.read_text(encoding="utf-8"), nav2.leaves())
    ctx = bt.Context()
    return bt.run(tree, ctx), ctx


def test_nominal_run_replans_once_per_second():
    nav2 = bt.FakeNav2(follow_ticks=300)
    result, ctx = run_default(nav2)
    assert result is S
    assert ctx.tick_index == 299                   # 3.00 s at 100 Hz
    assert nav2.counters["plan"] == 3, "RateController hz=1.0 over 3 s of driving"


def test_controller_failure_is_recovered_by_clearing_the_local_costmap():
    nav2 = bt.FakeNav2(follow_outcomes=[False, True], follow_ticks=200, follow_fail_ticks=150)
    result, ctx = run_default(nav2)
    assert result is S
    names = [n for _, n, s in ctx.trace if s is S]
    assert "ClearLocalCostmap-Context" in names
    assert "ClearLocalCostmap-Subtree" not in names, "the context recovery handles it; no spin needed"


def test_unreachable_goal_cycles_the_recovery_subtree_then_aborts():
    nav2 = bt.FakeNav2(plan_outcomes=[False])
    result, ctx = run_default(nav2)
    assert result is F
    order = [n for _, n, s in ctx.trace
             if s is S and n in {"ClearGlobalCostmap-Subtree", "Spin", "Wait", "BackUp"}]
    assert order == ["ClearGlobalCostmap-Subtree", "Spin", "Wait", "BackUp", "ClearGlobalCostmap-Subtree", "Spin"]
    assert nav2.counters["plan"] == 14, "7 planning attempts x 2 (context recovery retries each)"


def test_goal_updated_short_circuits_the_recovery_subtree():
    nav2 = bt.FakeNav2(plan_outcomes=[False], goal_was_updated=True)
    result, ctx = run_default(nav2)
    assert result is F
    assert not [n for _, n, s in ctx.trace if n in {"Spin", "Wait", "BackUp"}], \
        "a new goal arrived: ReactiveFallback returns SUCCESS before the RoundRobin"


def test_scenarios_all_terminate():
    for _, nav2 in bt.scenarios():
        result, _ = run_default(nav2)
        assert result in (S, F)
