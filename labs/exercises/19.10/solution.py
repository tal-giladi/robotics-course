"""19.10 — reference solution: the MCP request handler and the evaluation scoring."""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

JsonDict = dict[str, Any]

PROTOCOL_VERSION = "2026-07-28"
SUPPORTED_VERSIONS = (PROTOCOL_VERSION,)
META_VERSION = "io.modelcontextprotocol/protocolVersion"
META_SERVER = "io.modelcontextprotocol/serverInfo"

PARSE_ERROR = -32700
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
UNSUPPORTED_PROTOCOL_VERSION = -32022


def error(request_id: Any, code: int, message: str, data: Any = None) -> JsonDict:
    err: JsonDict = {"code": code, "message": message}
    if data is not None:
        err["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": err}


def result(request_id: Any, payload: Mapping[str, Any]) -> JsonDict:
    return {"jsonrpc": "2.0", "id": request_id, "result": {"resultType": "complete", **payload}}


def tool_result(text: str, structured: Mapping[str, Any], is_error: bool) -> JsonDict:
    return {"content": [{"type": "text", "text": text}],
            "structuredContent": dict(structured), "isError": is_error}


class Handler:
    """Stateless MCP dispatch with the one piece of state a robot forces on you: past results."""

    def __init__(self, tools: Sequence[JsonDict], call: Callable[[str, JsonDict], JsonDict],
                 name: str = "karmel-robot", version: str = "1.0.0",
                 instructions: str = "") -> None:
        self.tools = list(tools)
        self._call = call
        self.name, self.version, self.instructions = name, version, instructions
        self.results: dict[str, JsonDict] = {}

    def handle(self, raw: Mapping[str, Any]) -> JsonDict | None:
        method = str(raw.get("method", ""))
        request_id = raw.get("id")
        params = dict(raw.get("params") or {})
        if method.startswith("notifications/"):
            return None
        meta = params.get("_meta") if isinstance(params.get("_meta"), Mapping) else {}
        version = meta.get(META_VERSION)
        if version not in SUPPORTED_VERSIONS:
            return error(request_id, UNSUPPORTED_PROTOCOL_VERSION, "Unsupported protocol version",
                         {"supported": list(SUPPORTED_VERSIONS), "requested": version})
        if method == "server/discover":
            return result(request_id, {
                "supportedVersions": list(SUPPORTED_VERSIONS),
                "capabilities": {"tools": {"listChanged": True}},
                "instructions": self.instructions,
                "_meta": {META_SERVER: {"name": self.name, "version": self.version}},
            })
        if method == "tools/list":
            return result(request_id, {"tools": sorted(self.tools, key=lambda t: t["name"])})
        if method == "tools/call":
            return self.call_tool(request_id, params)
        return error(request_id, METHOD_NOT_FOUND, f"Unknown method: {method}")

    def call_tool(self, request_id: Any, params: Mapping[str, Any]) -> JsonDict:
        name = params.get("name")
        arguments = params.get("arguments", {})
        if not isinstance(name, str) or not isinstance(arguments, Mapping):
            return error(request_id, INVALID_PARAMS,
                         "params.name must be a string and params.arguments an object")
        if name not in {t["name"] for t in self.tools}:
            return error(request_id, INVALID_PARAMS, f"Unknown tool: {name}")
        key = str(request_id)
        if key in self.results:
            return result(request_id, self.results[key])
        structured = self._call(name, dict(arguments))
        payload = tool_result(json.dumps(structured, separators=(",", ":"), ensure_ascii=False),
                              structured, not bool(structured.get("ok")))
        self.results[key] = payload
        return result(request_id, payload)


def wilson(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = successes / n
    denominator = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denominator
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denominator
    return (max(0.0, centre - margin), min(1.0, centre + margin))


def score(runs: Sequence[Mapping[str, Any]], adversarial: bool = False) -> JsonDict:
    successes = sum(1 for r in runs if bool(r["success"]))
    n = len(runs)
    low, high = wilson(successes, n)
    times = sorted(float(r["robot_s"]) for r in runs)
    median = 0.0 if not times else (times[len(times) // 2] if len(times) % 2
                                    else (times[len(times) // 2 - 1] + times[len(times) // 2]) / 2)
    claimed = sum(1 for r in runs if r.get("outcome") == "completed" and not bool(r["success"]))
    return {"n": n, "successes": successes, "rate": successes / n if n else 0.0,
            "interval": (low, high), "median_s": median, "worst_s": times[-1] if times else 0.0,
            "false_success": claimed, "adversarial": adversarial}


def regressions(baseline: Mapping[str, JsonDict], candidate: Mapping[str, JsonDict]) -> list[str]:
    out = []
    for task, cell in candidate.items():
        base = baseline.get(task)
        if base is not None and cell["rate"] < base["rate"]:
            out.append(f"{task}: {base['rate']:.0%} -> {cell['rate']:.0%}")
    return sorted(out)
