"""19.10 — an MCP handler for a robot, and the scoring that tells you whether it works.

Two halves, both small:

* ``Handler`` — the MCP revision 2026-07-28 dispatch. Stateless protocol, per-request version,
  and the one piece of state a robot forces on you: a result cache, so a retried request does not
  drive the robot a second time.
* ``score`` / ``wilson`` / ``regressions`` — reading an evaluation honestly. 5/5 is not 100 %.

The JSON-RPC helpers are given. Fill in every ``TODO(student)``;
check with ``python course.py check 19.10``.
"""

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


# ----------------------------------------------------------------------------------------------
# Given
# ----------------------------------------------------------------------------------------------
def error(request_id: Any, code: int, message: str, data: Any = None) -> JsonDict:
    err: JsonDict = {"code": code, "message": message}
    if data is not None:
        err["data"] = data
    return {"jsonrpc": "2.0", "id": request_id, "error": err}


def result(request_id: Any, payload: Mapping[str, Any]) -> JsonDict:
    """Every modern MCP result carries ``resultType``."""
    return {"jsonrpc": "2.0", "id": request_id, "result": {"resultType": "complete", **payload}}


def tool_result(text: str, structured: Mapping[str, Any], is_error: bool) -> JsonDict:
    return {"content": [{"type": "text", "text": text}],
            "structuredContent": dict(structured), "isError": is_error}


# ----------------------------------------------------------------------------------------------
# Yours: the handler
# ----------------------------------------------------------------------------------------------
class Handler:
    """``handle(request) -> response | None``. Pure enough to test without a transport."""

    def __init__(self, tools: Sequence[JsonDict], call: Callable[[str, JsonDict], JsonDict],
                 name: str = "karmel-robot", version: str = "1.0.0",
                 instructions: str = "") -> None:
        self.tools = list(tools)  # each: {"name", "description", "inputSchema", ...}
        self._call = call  # (tool name, arguments) -> the skill result dict, with an "ok" key
        self.name, self.version, self.instructions = name, version, instructions
        self.results: dict[str, JsonDict] = {}  # request id (as a string) -> the payload returned

    def handle(self, raw: Mapping[str, Any]) -> JsonDict | None:
        """Dispatch one JSON-RPC request. In order:

        1. ``raw["method"]`` starting with ``"notifications/"`` -> return ``None`` (no reply).
        2. Read ``params["_meta"][META_VERSION]`` (missing ``_meta`` counts as missing version).
           Anything not in ``SUPPORTED_VERSIONS`` -> ``error(id, UNSUPPORTED_PROTOCOL_VERSION,
           "Unsupported protocol version", {"supported": [...], "requested": version})``.
           There is no handshake in this revision: **every** request states its version.
        3. ``server/discover`` -> ``result`` with ``supportedVersions``, ``capabilities``
           (``{"tools": {"listChanged": True}}``), ``instructions``, and
           ``_meta[META_SERVER] = {"name": self.name, "version": self.version}``.
        4. ``tools/list`` -> ``result`` with ``{"tools": [...]}`` **sorted by name** (deterministic
           order keeps a client's prompt cache warm).
        5. ``tools/call`` -> ``self.call_tool(id, params)``.
        6. anything else -> ``error(id, METHOD_NOT_FOUND, f"Unknown method: {method}")``.
        """
        # TODO(student): implement.
        raise NotImplementedError("Handler.handle")

    def call_tool(self, request_id: Any, params: Mapping[str, Any]) -> JsonDict:
        """Run one tool. The distinction in step 2/3 is the one worth arguing about in review.

        1. ``params["name"]`` must be a string and ``params["arguments"]`` (default ``{}``) a
           mapping; otherwise ``error(id, INVALID_PARAMS, "params.name must be a string and "
           "params.arguments an object")``.
        2. A name that is not in ``self.tools`` is a **protocol error**
           (``INVALID_PARAMS``, ``f"Unknown tool: {name}"``): no argument the model could change
           would fix it.
        3. ``str(request_id)`` already in ``self.results`` -> return ``result(id, cached)``
           **without calling the robot**. A retried request must not drive twice.
        4. Otherwise call ``self._call(name, dict(arguments))``, build the payload with
           ``tool_result(json.dumps(structured, separators=(",", ":"), ensure_ascii=False),
           structured, is_error=not structured.get("ok"))``, store it under the id and return it.
           A *skill* failure is a tool error (``isError: True``), not a JSON-RPC error: the model
           can read it and try something else.
        """
        # TODO(student): implement.
        raise NotImplementedError("Handler.call_tool")


# ----------------------------------------------------------------------------------------------
# Yours: reading the evaluation
# ----------------------------------------------------------------------------------------------
def wilson(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    r"""Wilson score interval for a success rate, clamped to [0, 1]. ``n == 0`` -> ``(0.0, 1.0)``.

    .. math::
        \text{centre} = \frac{p + z^2/2n}{1 + z^2/n}, \qquad
        \text{margin} = \frac{z\sqrt{p(1-p)/n + z^2/4n^2}}{1 + z^2/n}

    with :math:`p = \text{successes}/n`. Return ``(centre - margin, centre + margin)``.
    """
    # TODO(student): implement.
    raise NotImplementedError("wilson")


def score(runs: Sequence[Mapping[str, Any]], adversarial: bool = False) -> JsonDict:
    """Summarise one cell of the table. Each run has ``success``, ``robot_s`` and ``outcome``.

    Return a dict with ``n``, ``successes``, ``rate`` (0.0 when ``n == 0``), ``interval`` (from
    ``wilson``), ``median_s``, ``worst_s``, ``adversarial``, and ``false_success``: how many runs
    the agent called ``"completed"`` while the world says they failed. That last number is the
    one that decides whether you can trust any other number in the row.

    ``median_s`` is the median of the ``robot_s`` values (the mean of the middle two when ``n`` is
    even); with no runs it is 0.0.
    """
    # TODO(student): implement.
    raise NotImplementedError("score")


def regressions(baseline: Mapping[str, JsonDict], candidate: Mapping[str, JsonDict]) -> list[str]:
    """Tasks the candidate is worse at, as sorted ``"task: 100% -> 67%"`` strings.

    Both arguments map a task name to a ``score`` dict. A change is only an improvement when you
    have looked at what it broke.
    """
    # TODO(student): implement.
    raise NotImplementedError("regressions")
