"""Checker for 19.03 — the agent loop.

Run: ``python course.py check 19.03`` (``--solution`` runs the reference).
Everything here is offline: the "model" is a script or the course's deterministic mock planner.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "19-llm-robot-agents" / "code", _REPO / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robot_agent.llm import (  # noqa: E402
    AssistantMessage,
    FetchPolicyLLM,
    ScriptedLLM,
    ToolCall,
    ToolResultsMessage,
    UserMessage,
    call,
    say,
)
from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402

TASK = "Find the water bottle and put it on the kitchen table."


@pytest.fixture
def skills():
    return RobotSkills(HomeWorld(seed=0))


def watch_stops(skills):
    """Record the simulated time of every base.stop(); the last one must be at the very end."""
    stops: list[float] = []
    original = skills.world.base.stop

    def stop():
        stops.append(skills.world.t)
        original()

    skills.world.base.stop = stop
    return stops


# --- the happy path ---------------------------------------------------------------------------
def test_the_agent_finishes_the_fetch_task(impl, skills):
    run = impl.run_agent(FetchPolicyLLM(), skills, TASK, impl.Budget())
    assert run.outcome == "completed"
    assert skills.world.objects["bottle-1"].on == "kitchen_table", "judge success from the world, not the text"
    assert skills.world.holding is None
    assert (run.llm_turns, len(run.tool_log)) == (14, 13), "seed 0 is deterministic"


def test_the_robot_is_stopped_on_every_path(impl, skills):
    stops = watch_stops(skills)
    impl.run_agent(ScriptedLLM([call("navigate_to", "c1", place="kitchen"), say("done")]), skills, TASK)
    assert stops, "the loop must stop the robot before returning"
    assert stops[-1] == pytest.approx(skills.world.t), "the last stop must be the last thing that happens"


def test_an_ambiguous_request_costs_no_robot_time(impl, skills):
    run = impl.run_agent(FetchPolicyLLM(), skills, "Find the water bottle and put it on the table.")
    assert run.outcome == "completed" and len(run.tool_log) == 1
    assert skills.world.t == 0.0, "asking a question must not move the robot"


# --- transcript shape -------------------------------------------------------------------------
def test_all_results_of_one_turn_go_back_in_one_message(impl, skills):
    """Two calls in a turn -> one ToolResultsMessage holding two results, in order."""
    two = AssistantMessage("", (ToolCall("a", "list_places", {}), ToolCall("b", "get_robot_state", {})),
                           stop_reason="tool_use")
    run = impl.run_agent(ScriptedLLM([two, say("ok")]), skills, TASK)
    blocks = [m for m in run.transcript if isinstance(m, ToolResultsMessage)]
    assert len(blocks) == 1, f"expected one results message per turn, got {len(blocks)}"
    assert [r.call_id for r in blocks[0].results] == ["a", "b"]


def test_a_failed_call_is_still_reported_with_is_error(impl, skills):
    run = impl.run_agent(ScriptedLLM([call("navigate_to", "c1", place="garage"), say("sorry")]), skills, TASK)
    results = [m for m in run.transcript if isinstance(m, ToolResultsMessage)][0].results
    assert len(results) == 1 and results[0].is_error is True
    assert "INVALID_ARGUMENTS" in results[0].content
    assert json.loads(results[0].content)["hint"], "the model needs the hint to recover"


def test_the_transcript_starts_with_the_task(impl, skills):
    run = impl.run_agent(ScriptedLLM([say("nothing to do")]), skills, TASK)
    assert isinstance(run.transcript[0], UserMessage) and run.transcript[0].text == TASK
    assert run.final_text == "nothing to do" and run.outcome == "completed"


def test_the_call_id_is_the_idempotency_key(impl, skills):
    """The same tool-call id twice must not drive the robot twice."""
    again = ScriptedLLM([call("navigate_to", "same-id", place="kitchen"),
                         call("navigate_to", "same-id", place="kitchen"),
                         say("done")])
    impl.run_agent(again, skills, TASK)
    assert len(skills.log) == 2
    assert skills.log[1].cached is True, "pass call_id=tc.id to skills.call"
    assert skills.log[1].sim_t_start == skills.log[1].sim_t_end, "the cached call must consume no robot time"


# --- budgets ----------------------------------------------------------------------------------
def test_turn_limit(impl, skills):
    llm = ScriptedLLM([call("list_places", f"c{i}") for i in range(10)])
    run = impl.run_agent(llm, skills, TASK, impl.Budget(max_llm_turns=3))
    assert run.outcome == "turn_limit" and run.llm_turns == 3
    assert len(run.tool_log) == 3


def test_tool_limit_and_a_well_formed_transcript(impl, skills):
    """When the budget runs out mid-turn, the remaining calls still get a result block."""
    turn = AssistantMessage("", tuple(ToolCall(f"c{i}", "list_places", {}) for i in range(3)),
                            stop_reason="tool_use")
    run = impl.run_agent(ScriptedLLM([turn, say("x")]), skills, TASK, impl.Budget(max_tool_calls=2))
    assert run.outcome == "tool_limit" and len(run.tool_log) == 2
    results = [m for m in run.transcript if isinstance(m, ToolResultsMessage)][0].results
    assert len(results) == 3, "every tool_use id needs a tool_result, even when the budget is gone"
    assert json.loads(results[2].content)["code"] == "BUDGET_EXCEEDED"


def test_robot_time_limit(impl, skills):
    llm = ScriptedLLM([call("navigate_to", f"c{i}", place=p)
                       for i, p in enumerate(["kitchen", "study", "bedroom", "living_room", "desk"])])
    run = impl.run_agent(llm, skills, TASK, impl.Budget(max_robot_time_s=20.0))
    assert run.outcome == "robot_time_limit"
    assert skills.world.t >= 20.0 and len(run.tool_log) < 5


def test_error_limit_counts_consecutive_failures(impl, skills):
    bad = [call("navigate_to", f"c{i}", place="garage") for i in range(6)]
    run = impl.run_agent(ScriptedLLM(bad), skills, TASK, impl.Budget(max_consecutive_errors=3))
    assert run.outcome == "error_limit" and len(run.tool_log) == 3


def test_a_success_resets_the_error_counter(impl, skills):
    mixed = [call("navigate_to", "c1", place="garage"), call("navigate_to", "c2", place="garage"),
             call("list_places", "c3"), call("navigate_to", "c4", place="garage"),
             call("navigate_to", "c5", place="garage"), say("giving up")]
    run = impl.run_agent(ScriptedLLM(mixed), skills, TASK, impl.Budget(max_consecutive_errors=3))
    assert run.outcome == "completed", "two failures, a success, two failures is not three in a row"
    assert len(run.tool_log) == 5


# --- stop reasons -----------------------------------------------------------------------------
def test_a_refusal_ends_the_run(impl, skills):
    llm = ScriptedLLM([AssistantMessage("I can't help with that.", (), stop_reason="refusal")])
    run = impl.run_agent(llm, skills, TASK)
    assert run.outcome == "refusal" and skills.world.t == 0.0


def test_max_tokens_is_not_completion(impl, skills):
    llm = ScriptedLLM([AssistantMessage("I was about to", (), stop_reason="max_tokens")])
    run = impl.run_agent(llm, skills, TASK)
    assert run.outcome == "max_tokens", "a truncated answer is not a finished task"


# --- the log ----------------------------------------------------------------------------------
def test_the_tool_log_is_enough_to_debug_a_run(impl, skills):
    run = impl.run_agent(ScriptedLLM([call("navigate_to", "c1", place="kitchen"), say("done")]), skills, TASK)
    record = run.tool_log[0]
    assert {"turn", "call_id", "skill", "args", "ok", "code", "message",
            "robot_elapsed_s", "robot_t_s"} <= set(record)
    assert record["skill"] == "navigate_to" and record["args"] == {"place": "kitchen"}
    assert record["ok"] is True and record["code"] == "SUCCEEDED"
    assert record["robot_elapsed_s"] > 0 and record["robot_t_s"] == pytest.approx(skills.world.t, abs=0.05)
    json.dumps(run.tool_log)  # one JSON line per call, or it is not a log
