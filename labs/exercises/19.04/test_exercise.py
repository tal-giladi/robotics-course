"""Checker for 19.04 — the state-machine engine.

Run: ``python course.py check 19.04`` (``--solution`` runs the reference).
No robot: the states here are plain callbacks, so the tests are about the engine only.
"""

from __future__ import annotations

import pytest


def counter_state(impl, outcomes, sequence):
    """A state that returns the given outcomes in order, then repeats the last one."""
    box = {"i": 0}

    def fn(bb):
        out = sequence[min(box["i"], len(sequence) - 1)]
        box["i"] += 1
        bb.setdefault("ran", []).append(out)
        return out

    return impl.CallbackState(outcomes, fn)


def linear(impl, name="M"):
    """START --go--> MIDDLE --done--> succeeded, and MIDDLE --oops--> failed."""
    m = impl.StateMachine(name, ("succeeded", "failed"))
    m.add_state("START", counter_state(impl, ("go",), ["go"]), {"go": "MIDDLE"})
    m.add_state("MIDDLE", counter_state(impl, ("done", "oops"), ["done"]),
                {"done": "succeeded", "oops": "failed"})
    return m


# --- construction -----------------------------------------------------------------------------
def test_the_first_state_added_is_the_start(impl):
    m = linear(impl)
    assert m.start == "START"


def test_add_state_chains_and_refuses_duplicates(impl):
    m = impl.StateMachine("M", ("done",))
    assert m.add_state("A", counter_state(impl, ("x",), ["x"]), {"x": "done"}) is m, "add_state returns self"
    with pytest.raises(impl.InvalidMachine, match="duplicate state A"):
        m.add_state("A", counter_state(impl, ("x",), ["x"]), {"x": "done"})


# --- validate ---------------------------------------------------------------------------------
def test_an_empty_machine_has_no_start_state(impl):
    with pytest.raises(impl.InvalidMachine, match="no start state"):
        impl.StateMachine("M", ("done",)).validate()


def test_an_unmapped_outcome_is_refused(impl):
    m = impl.StateMachine("FETCH", ("succeeded", "failed"))
    m.add_state("PICK", counter_state(impl, ("holding", "out_of_reach"), ["holding"]),
                {"holding": "succeeded"})
    with pytest.raises(impl.InvalidMachine, match=r"FETCH.PICK: outcome 'out_of_reach' has no transition"):
        m.validate()


def test_a_transition_for_an_undeclared_outcome_is_refused(impl):
    m = impl.StateMachine("FETCH", ("succeeded",))
    m.add_state("PICK", counter_state(impl, ("holding",), ["holding"]),
                {"holding": "succeeded", "hodling": "succeeded"})
    with pytest.raises(impl.InvalidMachine, match="transition for undeclared outcome 'hodling'"):
        m.validate()


def test_a_transition_to_nowhere_is_refused(impl):
    m = impl.StateMachine("FETCH", ("succeeded",))
    m.add_state("APPROACH", counter_state(impl, ("arrived",), ["arrived"]), {"arrived": "CONFRIM"})
    with pytest.raises(impl.InvalidMachine, match=r"'arrived' goes to unknown 'CONFRIM'"):
        m.validate()


def test_an_unreachable_state_is_refused(impl):
    m = linear(impl)
    m.add_state("RECHARGE", counter_state(impl, ("docked",), ["docked"]), {"docked": "succeeded"})
    with pytest.raises(impl.InvalidMachine, match=r"unreachable states \['RECHARGE'\]"):
        m.validate()


def test_a_self_loop_is_reachable_and_legal(impl):
    m = impl.StateMachine("M", ("done",))
    m.add_state("LOOK", counter_state(impl, ("again", "found"), ["again", "again", "found"]),
                {"again": "LOOK", "found": "done"})
    m.validate()
    assert m.execute({}) == "done"
    assert [e.outcome for e in m.trace] == ["again", "again", "found"]


def test_a_sub_machine_is_validated_too(impl):
    inner = impl.StateMachine("FIND", ("found", "lost"))
    inner.add_state("LOOK", counter_state(impl, ("found", "lost"), ["found"]), {"found": "found"})
    outer = impl.StateMachine("FETCH", ("succeeded", "failed"))
    outer.add_state("FIND", inner, {"found": "succeeded", "lost": "failed"})
    with pytest.raises(impl.InvalidMachine, match=r"FIND.LOOK: outcome 'lost' has no transition"):
        outer.validate()


# --- execute ----------------------------------------------------------------------------------
def test_a_linear_machine_runs_and_traces(impl):
    m = linear(impl)
    bb = {}
    assert m.execute(bb) == "succeeded"
    assert bb["ran"] == ["go", "done"]
    assert [(e.machine, e.state, e.outcome, e.next) for e in m.trace] == [
        ("M", "START", "go", "MIDDLE"), ("M", "MIDDLE", "done", "succeeded")]


def test_execute_validates_first(impl):
    m = impl.StateMachine("M", ("done",))
    m.add_state("A", counter_state(impl, ("x", "y"), ["x"]), {"x": "done"})
    with pytest.raises(impl.InvalidMachine):
        m.execute({})


def test_a_state_that_returns_an_undeclared_outcome_raises_at_once(impl):
    m = impl.StateMachine("M", ("done",))
    m.add_state("A", impl.CallbackState(("x",), lambda bb: "boom"), {"x": "done"})
    with pytest.raises(impl.InvalidMachine, match="M.A returned undeclared outcome 'boom'"):
        m.execute({})


def test_nested_machines_share_one_timeline(impl):
    inner = impl.StateMachine("FIND", ("found",))
    inner.add_state("LOOK", counter_state(impl, ("found",), ["found"]), {"found": "found"})
    outer = impl.StateMachine("FETCH", ("succeeded",))
    outer.add_state("FIND", inner, {"found": "PLACE"})
    outer.add_state("PLACE", counter_state(impl, ("placed",), ["placed"]), {"placed": "succeeded"})
    assert outer.execute({}) == "succeeded"
    assert [(e.machine, e.state) for e in outer.trace] == [
        ("FIND", "LOOK"), ("FETCH", "FIND"), ("FETCH", "PLACE")], "inner entries come first, in one list"
    assert inner.trace is outer.trace


def test_the_clock_is_used_for_trace_timestamps(impl):
    now = {"t": 0.0}
    m = impl.StateMachine("M", ("done",), clock=lambda: now["t"])

    def tick(bb):
        now["t"] += 1.234
        return "x"

    m.add_state("A", impl.CallbackState(("x",), tick), {"x": "done"})
    m.execute({})
    assert m.trace[0].t == pytest.approx(1.23), "timestamps are rounded to 2 decimals"


# --- preemption -------------------------------------------------------------------------------
def test_preemption_aborts_before_the_next_state(impl):
    bb = {"stop": False}
    m = impl.StateMachine("M", ("done", "preempted"),
                          preempt=lambda b: "preempted" if b["stop"] else None)

    def first(b):
        b["stop"] = True          # something (an e-stop handler) fires while this state runs
        return "go"

    m.add_state("A", impl.CallbackState(("go",), first), {"go": "B"})
    m.add_state("B", impl.CallbackState(("go",), lambda b: "go"), {"go": "done"})
    assert m.execute(bb) == "preempted"
    assert [e.outcome for e in m.trace] == ["go", "preempted"], "B must never run"
    assert m.trace[-1].next == "preempted"


def test_preemption_propagates_out_of_a_sub_machine(impl):
    bb = {"stop": True}
    stop = lambda b: "preempted" if b["stop"] else None  # noqa: E731
    inner = impl.StateMachine("FIND", ("found", "preempted"), preempt=stop)
    inner.add_state("LOOK", impl.CallbackState(("found",), lambda b: "found"), {"found": "found"})
    outer = impl.StateMachine("FETCH", ("succeeded", "preempted"), preempt=stop)
    outer.add_state("FIND", inner, {"found": "succeeded", "preempted": "preempted"})
    assert outer.execute(bb) == "preempted"


def test_a_preemption_outcome_the_machine_does_not_declare_is_a_bug(impl):
    m = impl.StateMachine("M", ("done",), preempt=lambda b: "halted")
    m.add_state("A", impl.CallbackState(("x",), lambda b: "x"), {"x": "done"})
    with pytest.raises(impl.InvalidMachine, match="'halted' is not an outcome of this machine"):
        m.execute({})


# --- the bound --------------------------------------------------------------------------------
def test_a_loop_without_a_counter_is_bounded(impl):
    m = impl.StateMachine("M", ("done",), max_transitions=25)
    m.add_state("A", impl.CallbackState(("again",), lambda b: "again"), {"again": "A"})
    with pytest.raises(RuntimeError, match="more than 25 transitions"):
        m.execute({})
    assert len(m.trace) == 25
