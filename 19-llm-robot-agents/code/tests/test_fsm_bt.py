"""State machine (19.04) and behavior tree (19.05) executives over the same skill API."""

from __future__ import annotations

import pytest

from robot_agent.fsm import CallbackState, FetchConfig, InvalidMachine, StateMachine, run_fetch
from robot_agent.sim_world import ArmParams, HomeWorld
from robot_agent.skills import RobotSkills


# --- FSM engine --------------------------------------------------------------------------------
def test_validate_catches_unmapped_outcomes_and_unknown_targets():
    sm = StateMachine("M", ("done",))
    sm.add_state("A", CallbackState(("ok", "err"), lambda bb: "ok"), {"ok": "done"})
    with pytest.raises(InvalidMachine, match="'err' has no transition"):
        sm.validate()
    sm = StateMachine("M", ("done",))
    sm.add_state("A", CallbackState(("ok",), lambda bb: "ok"), {"ok": "B"})
    with pytest.raises(InvalidMachine, match="unknown 'B'"):
        sm.validate()


def test_validate_catches_unreachable_states():
    sm = StateMachine("M", ("done",))
    sm.add_state("A", CallbackState(("ok",), lambda bb: "ok"), {"ok": "done"})
    sm.add_state("B", CallbackState(("ok",), lambda bb: "ok"), {"ok": "done"})
    with pytest.raises(InvalidMachine, match="unreachable"):
        sm.validate()


def test_nested_machine_and_preemption():
    inner = StateMachine("INNER", ("finished",))
    inner.add_state("X", CallbackState(("go",), lambda bb: bb.__setitem__("n", bb["n"] + 1) or "go"), {"go": "finished"})
    outer = StateMachine("OUTER", ("done", "stopped"), preempt=lambda bb: "stopped" if bb["n"] >= 3 else None)
    outer.add_state("LOOP", inner, {"finished": "LOOP"})
    bb = {"n": 0}
    assert outer.execute(bb) == "stopped" and bb["n"] == 3
    assert [e.machine for e in outer.trace].count("INNER") == 3, "nested transitions share one trace"


def test_undeclared_outcome_is_a_bug():
    sm = StateMachine("M", ("done",))
    sm.add_state("A", CallbackState(("ok",), lambda bb: "typo"), {"ok": "done"})
    with pytest.raises(InvalidMachine, match="undeclared outcome 'typo'"):
        sm.execute({})


# --- fetch FSM on the simulated home ------------------------------------------------------------------
def test_fetch_fsm_succeeds():
    home = HomeWorld(seed=1)
    outcome, bb, fsm = run_fetch(RobotSkills(home), FetchConfig("water bottle", "kitchen_table"))
    assert outcome == "succeeded"
    assert home.objects["bottle-1"].on == "kitchen_table"
    states = [e.state for e in fsm.trace]
    assert states[-4:] == ["CONFIRM", "PICK", "DELIVER", "PLACE"]


def test_fetch_fsm_gives_up_after_max_grasp_attempts():
    home = HomeWorld(seed=1, arm=ArmParams(grasp_success_p=0.0))
    outcome, bb, fsm = run_fetch(RobotSkills(home), FetchConfig("water bottle", "kitchen_table", max_grasp_attempts=3))
    assert outcome == "failed"
    assert [e.outcome for e in fsm.trace if e.state == "PICK"] == ["grasp_failed", "grasp_failed", "failed"]
    assert home.holding is None


def test_fetch_fsm_estop_preempts():
    home = HomeWorld(seed=1)
    skills = RobotSkills(home)
    original = skills.call

    def call_and_press_estop(name, args=None, call_id=None, cancel=None):
        r = original(name, args, call_id, cancel)
        if name == "pick" and r.ok:
            home.estop = True  # someone presses the e-stop while the robot holds the bottle
        return r

    skills.call = call_and_press_estop  # type: ignore[method-assign]
    outcome, bb, fsm = run_fetch(skills, FetchConfig("water bottle", "kitchen_table"))
    assert outcome == "preempted"
    assert fsm.trace[-1].state == "DELIVER"


def test_fetch_fsm_deadline():
    home = HomeWorld(seed=1)
    outcome, _, fsm = run_fetch(RobotSkills(home), FetchConfig("water bottle", "kitchen_table", deadline_s=15.0))
    assert outcome == "timeout"


def test_fetch_fsm_reports_not_found():
    home = HomeWorld(seed=1)
    outcome, _, _ = run_fetch(RobotSkills(home), FetchConfig("umbrella", "kitchen_table"))
    assert outcome == "not_found"


# --- behavior tree (py_trees) --------------------------------------------------------------------
py_trees = pytest.importorskip("py_trees")
from robot_agent.bt import FetchTreeConfig, build_fetch_tree, run_tree  # noqa: E402

Status = py_trees.common.Status


def test_bt_fetch_succeeds():
    home = HomeWorld(seed=1)
    skills = RobotSkills(home)
    root, bb = build_fetch_tree(skills, FetchTreeConfig("water bottle", "kitchen_table"))
    run = run_tree(root, skills)
    assert run.status == Status.SUCCESS
    assert home.objects["bottle-1"].on == "kitchen_table"
    assert bb.object["object_id"] == "bottle-1"


def test_bt_estop_halts_the_running_action_reactively():
    home = HomeWorld(seed=1)
    skills = RobotSkills(home)
    root, _ = build_fetch_tree(skills, FetchTreeConfig("water bottle", "kitchen_table"))
    run = run_tree(root, skills, on_tick=lambda tick, _: setattr(home, "estop", tick >= 100))
    assert run.status == Status.FAILURE and run.ticks == 100, "the guard is re-checked on the very next tick"
    last = skills.log[-1]
    assert (last.skill, last.result.code) == ("navigate_to", "CANCELED"), "the running goal was canceled, not abandoned"


def test_bt_deadline_uses_sim_time():
    home = HomeWorld(seed=1)
    skills = RobotSkills(home)
    root, _ = build_fetch_tree(skills, FetchTreeConfig("water bottle", "kitchen_table", deadline_s=15.0))
    run = run_tree(root, skills)
    assert run.status == Status.FAILURE and 15.0 <= run.sim_time_s < 16.0


def test_bt_retries_grasps_then_fails():
    home = HomeWorld(seed=1, arm=ArmParams(grasp_success_p=0.0))
    skills = RobotSkills(home)
    root, _ = build_fetch_tree(skills, FetchTreeConfig("water bottle", "kitchen_table", grasp_attempts=3))
    run = run_tree(root, skills)
    assert run.status == Status.FAILURE
    assert 1 <= sum(1 for c in skills.log if c.skill == "pick") <= 3
    assert home.holding is None
