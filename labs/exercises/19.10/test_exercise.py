"""Checker for 19.10 — the MCP handler and the evaluation scoring. No transport, no robot."""

from __future__ import annotations

import json

import pytest

TOOLS = [
    {"name": "navigate_to", "description": "drive", "inputSchema": {"type": "object"}},
    {"name": "detect_objects", "description": "look", "inputSchema": {"type": "object"}},
]


def make(impl, calls: list | None = None):
    calls = calls if calls is not None else []

    def call(name: str, args: dict) -> dict:
        calls.append((name, args))
        if name == "navigate_to" and args.get("place") == "garage":
            return {"skill": name, "ok": False, "code": "INVALID_ARGUMENTS", "message": "no such place"}
        return {"skill": name, "ok": True, "code": "SUCCEEDED", "message": "done", "data": {}}

    return impl.Handler(TOOLS, call, instructions="drive carefully"), calls


def req(impl, method: str, request_id=1, params: dict | None = None, version: str | None = "USE") -> dict:
    p = dict(params or {})
    if version is not None:
        p["_meta"] = {impl.META_VERSION: impl.PROTOCOL_VERSION if version == "USE" else version}
    return {"jsonrpc": "2.0", "id": request_id, "method": method, "params": p}


# ----------------------------------------------------------------------------------------------
# Versioning
# ----------------------------------------------------------------------------------------------
def test_discover_reports_versions_capabilities_and_identity(impl) -> None:
    handler, _ = make(impl)
    r = handler.handle(req(impl, "server/discover"))["result"]
    assert r["resultType"] == "complete"
    assert r["supportedVersions"] == [impl.PROTOCOL_VERSION]
    assert r["capabilities"]["tools"]["listChanged"] is True
    assert r["_meta"][impl.META_SERVER]["name"] == "karmel-robot"
    assert r["instructions"] == "drive carefully"


def test_an_old_version_is_rejected_with_the_supported_list(impl) -> None:
    handler, _ = make(impl)
    r = handler.handle(req(impl, "tools/list", version="2025-06-18"))
    assert r["error"]["code"] == impl.UNSUPPORTED_PROTOCOL_VERSION
    assert r["error"]["data"] == {"supported": [impl.PROTOCOL_VERSION], "requested": "2025-06-18"}


def test_a_request_with_no_meta_is_rejected(impl) -> None:
    handler, _ = make(impl)
    assert handler.handle(req(impl, "tools/list", version=None))["error"]["code"] == \
        impl.UNSUPPORTED_PROTOCOL_VERSION


def test_every_method_is_version_checked_not_just_the_first(impl) -> None:
    handler, _ = make(impl)
    handler.handle(req(impl, "server/discover"))
    bad = handler.handle(req(impl, "tools/call", 2, {"name": "navigate_to", "arguments": {}},
                             version="2024-01-01"))
    assert bad["error"]["code"] == impl.UNSUPPORTED_PROTOCOL_VERSION


# ----------------------------------------------------------------------------------------------
# Dispatch
# ----------------------------------------------------------------------------------------------
def test_notifications_get_no_reply(impl) -> None:
    handler, _ = make(impl)
    assert handler.handle({"jsonrpc": "2.0", "method": "notifications/tools/list_changed"}) is None


def test_an_unknown_method_is_method_not_found(impl) -> None:
    handler, _ = make(impl)
    assert handler.handle(req(impl, "resources/list"))["error"]["code"] == impl.METHOD_NOT_FOUND


def test_tools_are_listed_in_a_deterministic_order(impl) -> None:
    handler, _ = make(impl)
    names = [t["name"] for t in handler.handle(req(impl, "tools/list"))["result"]["tools"]]
    assert names == sorted(names)


# ----------------------------------------------------------------------------------------------
# Calling
# ----------------------------------------------------------------------------------------------
def test_a_successful_call_returns_text_and_structured_content(impl) -> None:
    handler, _ = make(impl)
    r = handler.handle(req(impl, "tools/call", 3, {"name": "detect_objects", "arguments": {}}))["result"]
    assert r["isError"] is False
    assert json.loads(r["content"][0]["text"]) == r["structuredContent"]
    assert r["structuredContent"]["code"] == "SUCCEEDED"


def test_a_skill_failure_is_a_tool_error_not_a_protocol_error(impl) -> None:
    handler, _ = make(impl)
    r = handler.handle(req(impl, "tools/call", 4,
                           {"name": "navigate_to", "arguments": {"place": "garage"}}))
    assert "error" not in r
    assert r["result"]["isError"] is True and r["result"]["structuredContent"]["code"] == "INVALID_ARGUMENTS"


def test_an_unknown_tool_is_a_protocol_error(impl) -> None:
    handler, _ = make(impl)
    r = handler.handle(req(impl, "tools/call", 5, {"name": "open_fridge", "arguments": {}}))
    assert r["error"]["code"] == impl.INVALID_PARAMS and "open_fridge" in r["error"]["message"]


def test_bad_params_are_rejected_before_the_robot_is_touched(impl) -> None:
    handler, calls = make(impl)
    r = handler.handle(req(impl, "tools/call", 6, {"name": 42, "arguments": {}}))
    assert r["error"]["code"] == impl.INVALID_PARAMS and calls == []


def test_missing_arguments_default_to_empty(impl) -> None:
    handler, calls = make(impl)
    handler.handle(req(impl, "tools/call", 7, {"name": "detect_objects"}))
    assert calls == [("detect_objects", {})]


def test_replaying_a_request_id_does_not_call_the_robot_twice(impl) -> None:
    handler, calls = make(impl)
    message = req(impl, "tools/call", "req-9", {"name": "navigate_to", "arguments": {"place": "kitchen"}})
    first = handler.handle(message)
    second = handler.handle(message)
    assert len(calls) == 1
    assert first["result"]["structuredContent"] == second["result"]["structuredContent"]


def test_a_different_id_is_a_different_call(impl) -> None:
    handler, calls = make(impl)
    for request_id in ("a", "b"):
        handler.handle(req(impl, "tools/call", request_id,
                           {"name": "navigate_to", "arguments": {"place": "kitchen"}}))
    assert len(calls) == 2


# ----------------------------------------------------------------------------------------------
# Scoring
# ----------------------------------------------------------------------------------------------
def test_five_out_of_five_is_not_a_hundred_percent(impl) -> None:
    low, high = impl.wilson(5, 5)
    assert 0.4 < low < 0.6 and high == 1.0


def test_more_trials_narrow_the_claim(impl) -> None:
    assert impl.wilson(50, 50)[0] > impl.wilson(5, 5)[0]


def test_no_trials_is_no_information(impl) -> None:
    assert impl.wilson(0, 0) == (0.0, 1.0)


def test_half_of_ten_is_centred_on_a_half(impl) -> None:
    low, high = impl.wilson(5, 10)
    assert low < 0.5 < high and (low + high) / 2 == pytest.approx(0.5, abs=0.02)


def runs(*rows):
    return [{"success": s, "robot_s": t, "outcome": o} for s, t, o in rows]


def test_score_reports_the_median_and_the_worst_case(impl) -> None:
    cell = impl.score(runs((True, 40.0, "succeeded"), (True, 44.0, "succeeded"),
                           (True, 90.0, "succeeded")))
    assert cell["median_s"] == 44.0 and cell["worst_s"] == 90.0 and cell["rate"] == 1.0


def test_score_averages_the_middle_two_when_n_is_even(impl) -> None:
    cell = impl.score(runs((True, 10.0, "s"), (True, 20.0, "s"), (True, 30.0, "s"), (True, 40.0, "s")))
    assert cell["median_s"] == pytest.approx(25.0)


def test_score_counts_runs_the_agent_wrongly_called_a_success(impl) -> None:
    cell = impl.score(runs((False, 47.0, "completed"), (True, 44.0, "completed"),
                           (False, 20.0, "failed")))
    assert cell["false_success"] == 1 and cell["successes"] == 1


def test_an_empty_cell_does_not_divide_by_zero(impl) -> None:
    cell = impl.score([])
    assert cell["n"] == 0 and cell["rate"] == 0.0 and cell["median_s"] == 0.0


def test_regressions_list_only_what_got_worse(impl) -> None:
    base = {"a": impl.score(runs((True, 1.0, "s"))), "b": impl.score(runs((True, 1.0, "s")))}
    cand = {"a": impl.score(runs((False, 1.0, "s"))), "b": impl.score(runs((True, 1.0, "s")))}
    assert impl.regressions(base, cand) == ["a: 100% -> 0%"]


def test_a_task_the_baseline_never_ran_is_not_a_regression(impl) -> None:
    assert impl.regressions({}, {"a": impl.score(runs((False, 1.0, "s")))}) == []
