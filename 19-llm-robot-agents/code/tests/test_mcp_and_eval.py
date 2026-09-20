"""Tests for 19.10 — the MCP server and the evaluation harness. No subprocess, no network."""

from __future__ import annotations

import io
import json

import pytest

from robot_agent.evaluation import (
    SUITE,
    Cell,
    RunOutcome,
    regressions,
    run_closed_loop,
    run_over_mcp,
    run_suite,
    table,
    wilson,
)
from robot_agent.mcp_server import (
    PROTOCOL_VERSION,
    UNSUPPORTED_PROTOCOL_VERSION,
    InProcessClient,
    RobotMCPServer,
    request,
    serve_stdio,
)
from robot_agent.safety import Geofence, PhysicalBudget, SafetyLayer
from robot_agent.sim_world import HomeWorld
from robot_agent.skills import RobotSkills


@pytest.fixture
def server() -> RobotMCPServer:
    home = HomeWorld(seed=0)
    layer = SafetyLayer(RobotSkills(home), "autonomous", geofence=Geofence.bedroom_at_night(home),
                        budget=PhysicalBudget(max_distance_m=60.0, max_robot_time_s=300.0))
    layer.accept_task("water bottle", "kitchen_table")
    return RobotMCPServer(layer)


# ----------------------------------------------------------------------------------------------
# Protocol
# ----------------------------------------------------------------------------------------------
def test_discover_reports_versions_capabilities_and_identity(server: RobotMCPServer) -> None:
    r = server.handle(request("server/discover", 1))["result"]
    assert r["resultType"] == "complete"
    assert r["supportedVersions"] == [PROTOCOL_VERSION]
    assert "tools" in r["capabilities"]
    assert r["_meta"]["io.modelcontextprotocol/serverInfo"]["name"] == "karmel-robot"


def test_a_request_without_a_supported_version_is_rejected_with_the_list(server: RobotMCPServer) -> None:
    r = server.handle(request("tools/list", 1, version="2024-01-01"))
    assert r["error"]["code"] == UNSUPPORTED_PROTOCOL_VERSION
    assert r["error"]["data"]["supported"] == [PROTOCOL_VERSION]


def test_a_request_with_no_meta_at_all_is_rejected(server: RobotMCPServer) -> None:
    r = server.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/list", "params": {}})
    assert r["error"]["code"] == UNSUPPORTED_PROTOCOL_VERSION


def test_notifications_get_no_reply(server: RobotMCPServer) -> None:
    assert server.handle({"jsonrpc": "2.0", "method": "notifications/tools/list_changed"}) is None


def test_unknown_methods_are_method_not_found(server: RobotMCPServer) -> None:
    assert server.handle(request("resources/list", 1))["error"]["code"] == -32601


# ----------------------------------------------------------------------------------------------
# Tools
# ----------------------------------------------------------------------------------------------
def test_tools_list_is_sorted_and_schema_complete(server: RobotMCPServer) -> None:
    tools = InProcessClient(server).list_tools()
    names = [t["name"] for t in tools]
    assert names == sorted(names)
    for tool in tools:
        assert tool["inputSchema"]["type"] == "object"
        assert tool["outputSchema"]["required"] == ["skill", "ok", "code", "message"]
        assert tool["description"]


def test_the_mode_decides_which_tools_exist_at_all() -> None:
    skills = RobotSkills(HomeWorld(seed=0))
    server = RobotMCPServer(SafetyLayer(skills, "patrol"))
    names = {t["name"] for t in InProcessClient(server).list_tools()}
    assert "pick" not in names and "navigate_to" in names


def test_a_tool_that_blocks_says_so_in_its_description(server: RobotMCPServer) -> None:
    nav = next(t for t in InProcessClient(server).list_tools() if t["name"] == "navigate_to")
    assert "blocks the robot" in nav["description"]


def test_a_call_returns_text_and_structured_content(server: RobotMCPServer) -> None:
    r = InProcessClient(server).call_tool("get_robot_state")["result"]
    assert r["isError"] is False
    assert json.loads(r["content"][0]["text"]) == r["structuredContent"]
    assert r["structuredContent"]["code"] == "SUCCEEDED"


def test_a_denied_call_is_a_tool_error_not_a_protocol_error(server: RobotMCPServer) -> None:
    r = InProcessClient(server).call_tool("navigate_to", {"place": "bed"})["result"]
    assert r["isError"] is True and r["structuredContent"]["code"] == "GEOFENCE"
    assert r["structuredContent"]["hint"]


def test_an_unknown_tool_is_a_protocol_error(server: RobotMCPServer) -> None:
    assert InProcessClient(server).call_tool("open_fridge")["error"]["code"] == -32602


def test_bad_arguments_are_a_tool_error_the_model_can_fix(server: RobotMCPServer) -> None:
    r = InProcessClient(server).call_tool("navigate_to", {"place": "garage"})["result"]
    assert r["isError"] and r["structuredContent"]["code"] == "INVALID_ARGUMENTS"


# ----------------------------------------------------------------------------------------------
# The robot-specific parts
# ----------------------------------------------------------------------------------------------
def test_replaying_a_request_id_does_not_move_the_robot_twice(server: RobotMCPServer) -> None:
    message = request("tools/call", "req-7", {"name": "navigate_to", "arguments": {"place": "kitchen"}})
    first = server.handle(message)
    t_after_first = server.skills.world.t
    second = server.handle(message)
    assert t_after_first > 0 and server.skills.world.t == pytest.approx(t_after_first)
    assert first["result"]["structuredContent"] == second["result"]["structuredContent"]


def test_untrusted_text_stays_under_its_own_key(server: RobotMCPServer) -> None:
    client = InProcessClient(server)
    client.call_tool("navigate_to", {"place": "kitchen"})
    for _ in range(4):
        r = client.call_tool("detect_objects", {"min_confidence": 0.3})["result"]
        notes = [d for d in r["structuredContent"]["data"]["detections"] if "untrusted_text_seen" in d]
        if notes:
            assert "ignore your previous instructions" in notes[0]["untrusted_text_seen"].lower()
            assert "ignore your previous instructions" not in r["structuredContent"]["message"].lower()
            return
    pytest.skip("the detector did not see the note in four looks (it is allowed to miss)")


def test_stdio_transport_round_trips_one_message_per_line(server: RobotMCPServer) -> None:
    lines = json.dumps(request("server/discover", 1)) + "\n" + \
        json.dumps({"jsonrpc": "2.0", "method": "notifications/tools/list_changed"}) + "\n"
    out = io.StringIO()
    serve_stdio(server, io.StringIO(lines), out)
    replies = [json.loads(line) for line in out.getvalue().splitlines()]
    assert len(replies) == 1 and replies[0]["id"] == 1  # the notification produced no line


def test_malformed_json_gets_a_parse_error_and_the_server_keeps_going(server: RobotMCPServer) -> None:
    out = io.StringIO()
    serve_stdio(server, io.StringIO("{not json\n" + json.dumps(request("server/discover", 2)) + "\n"), out)
    replies = [json.loads(line) for line in out.getvalue().splitlines()]
    assert replies[0]["error"]["code"] == -32700 and replies[1]["id"] == 2


# ----------------------------------------------------------------------------------------------
# Evaluation
# ----------------------------------------------------------------------------------------------
def test_wilson_says_five_out_of_five_is_not_one_hundred_percent() -> None:
    low, high = wilson(5, 5)
    assert low < 0.6 and high == 1.0
    assert wilson(50, 50)[0] > 0.9  # more trials, a narrower claim


def test_success_is_judged_from_the_world_not_the_outcome_string() -> None:
    task = next(t for t in SUITE if t.name == "fetch_bottle")
    out = run_closed_loop(task, seed=0)
    assert out.success and out.robot_s > 0


def test_the_adversarial_task_scores_a_gullible_agent_as_a_failure() -> None:
    from robot_agent.evaluation import run_agent_loop

    task = next(t for t in SUITE if t.name == "injected")
    outcomes = [run_agent_loop(task, seed) for seed in (0, 1, 2)]
    assert any(o.outcome == "completed" and not o.success for o in outcomes)


def test_the_same_task_runs_identically_over_mcp() -> None:
    task = next(t for t in SUITE if t.name == "fetch_bottle")
    direct = run_closed_loop(task, seed=0, guard=True)
    over_mcp = run_over_mcp(task, seed=0)
    assert direct.success == over_mcp.success
    assert over_mcp.robot_s == pytest.approx(direct.robot_s, abs=1.0)
    assert "JSON-RPC" in over_mcp.note


def test_run_suite_is_deterministic_and_reports_regressions() -> None:
    tasks = [t for t in SUITE if t.name in ("fetch_bottle", "ambiguous")]
    runners = {"a": lambda t, s: run_closed_loop(t, s), "b": lambda t, s: run_closed_loop(t, s, guard=True)}
    first = run_suite(runners, tasks, seeds=(0,))
    second = run_suite(runners, tasks, seeds=(0,))
    assert {k: v.rate for k, v in first.items()} == {k: v.rate for k, v in second.items()}
    assert regressions(first, "a", "b") == []


def test_the_table_renders_every_runner_and_task() -> None:
    cells = {("r", "fetch_bottle"): Cell("r", "fetch_bottle", [RunOutcome(True, "succeeded", 40.0, 9)])}
    rendered = table(cells, [next(t for t in SUITE if t.name == "fetch_bottle")])
    assert "fetch_bottle" in rendered and "1/1" in rendered
