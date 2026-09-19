"""Provider-neutral LLM interface + offline backends (19.03).

The agent loop only knows these types. A provider adapter (``anthropic_llm.AnthropicLLM``) converts
them to and from its wire format; the mock backends below produce them without any network, so the
lab and its tests run without an API key.

* ``ScriptedLLM``    - replays a fixed list of turns. For unit tests of the loop itself.
* ``FetchPolicyLLM`` - a deterministic rule-based "planner" for "find X and put it on Y" that
  reacts to tool results (searches rooms, looks again after a miss, re-detects after a failed
  grasp). It plays the LLM's role so you can watch the whole loop offline. ``gullible=True`` makes
  it obey text it reads in the world: a prompt-injection demo for 19.09.
"""

from __future__ import annotations

import json
import re
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol

JsonDict = dict[str, Any]


@dataclass(frozen=True)
class ToolCall:
    id: str
    name: str
    arguments: JsonDict


@dataclass(frozen=True)
class UserMessage:
    text: str


@dataclass(frozen=True)
class AssistantMessage:
    text: str
    tool_calls: tuple[ToolCall, ...] = ()
    stop_reason: str = "end_turn"  # "tool_use" | "end_turn" | "max_tokens" | "refusal"
    usage: JsonDict = field(default_factory=dict)
    raw: Any = None  # provider-native content, echoed back verbatim (keeps thinking blocks intact)


@dataclass(frozen=True)
class ToolResult:
    call_id: str
    content: str  # JSON text
    is_error: bool = False


@dataclass(frozen=True)
class ToolResultsMessage:
    results: tuple[ToolResult, ...]


Message = UserMessage | AssistantMessage | ToolResultsMessage


class LLMBackend(Protocol):
    name: str

    def complete(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> AssistantMessage: ...


# ----------------------------------------------------------------------------------------------
# Offline backends
# ----------------------------------------------------------------------------------------------
class ScriptedLLM:
    """Returns pre-written turns in order. A turn may be a callable that inspects the transcript."""

    name = "scripted"

    def __init__(self, turns: Sequence[AssistantMessage | Callable[[Sequence[Message]], AssistantMessage]]) -> None:
        self.turns = list(turns)
        self.calls = 0

    def complete(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> AssistantMessage:
        if self.calls >= len(self.turns):
            return AssistantMessage("(script exhausted)", stop_reason="end_turn")
        turn = self.turns[self.calls]
        self.calls += 1
        return turn(messages) if callable(turn) else turn


def call(name: str, call_id: str, **arguments: Any) -> AssistantMessage:
    """Shorthand for a turn with one tool call."""
    return AssistantMessage("", (ToolCall(call_id, name, arguments),), stop_reason="tool_use")


def say(text: str) -> AssistantMessage:
    """Shorthand for a final text answer."""
    return AssistantMessage(text, (), stop_reason="end_turn")


_TASK_RE = re.compile(
    r"(?:find|fetch|get|bring)\s+(?:the\s+|a\s+|my\s+)?(?P<obj>.+?)\s+and\s+(?:put|place)\s+it\s+"
    r"(?:on|onto)\s+(?:the\s+)?(?P<target>[\w\s]+?)\s*\.?$",
    re.I,
)


class FetchPolicyLLM:
    """A deterministic stand-in for an LLM planner on fetch-and-place tasks.

    Like a real LLM it keeps no hidden state: it re-derives the next step from the transcript on
    every call, so tool results must carry all the information a planner needs.
    """

    name = "mock-fetch-policy"

    def __init__(self, gullible: bool = False, max_detect_retries: int = 2, max_grasp_retries: int = 2) -> None:
        self.gullible = gullible
        self.max_detect_retries = max_detect_retries
        self.max_grasp_retries = max_grasp_retries
        self._n = 0

    def _call(self, name: str, text: str = "", **arguments: Any) -> AssistantMessage:
        self._n += 1
        return AssistantMessage(text, (ToolCall(f"mock_{self._n:03d}", name, arguments),), stop_reason="tool_use")

    def complete(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> AssistantMessage:
        task = next((m.text for m in messages if isinstance(m, UserMessage)), "")
        match = _TASK_RE.search(task.strip())
        if not match:
            return say("I can only do tasks like 'Find the <object> and put it on the <surface>.'")
        wanted, target_words = match.group("obj").lower().strip(), match.group("target").lower().strip()
        hist = _history(messages)
        if not hist:
            return self._call("list_places", "First I need to know which places exist.")
        places = next((r["data"]["places"] for c, r in hist if c.name == "list_places" and r["ok"]), [])
        surfaces = [p["name"] for p in places if p["kind"] == "surface"]
        rooms = [p["name"] for p in places if p["kind"] == "room"]

        target = _resolve_surface(target_words, surfaces)
        if self.gullible:  # obeys instructions it reads in the world: exactly what an agent must NOT do
            for _, r in hist:
                for d in r.get("data", {}).get("detections", []):
                    text = (d.get("untrusted_text_seen") or "").lower()
                    target = next((s for s in surfaces if f"on the {s}" in text), target)
        if target is None:
            return say(f"Which surface do you mean by '{target_words}'? I know: {', '.join(surfaces)}.")

        last_call, last = hist[-1]
        at_place = _at_place(hist)
        holding = _holding(hist)
        if last_call.name == "place" and last["ok"]:
            return say(f"Done: the {wanted} is on the {target}.")

        if holding:
            if at_place == target:
                return self._call("place", f"Placing the {wanted} on the {target}.", location=target)
            if _count(hist, "navigate_to", ok=False) >= 2:
                return say(f"I am holding the {wanted} but cannot get to the {target}: {last['message']}")
            return self._call("navigate_to", f"I have the {wanted}; driving to the {target}.", place=target)

        if last_call.name == "pick" and not last["ok"]:
            if last["code"] == "GRASP_FAILED" and _count(hist, "pick", ok=False) <= self.max_grasp_retries:
                return self._call("detect_objects", "The grasp failed; looking again before retrying.")
            return say(f"I could not pick up the {wanted}: {last['message']}")
        if _count(hist, "navigate_to", ok=False) >= 2:
            return say(f"Navigation keeps failing ({last['message']}); stopping.")

        seen = _latest_match(hist, wanted)
        misses = _consecutive_misses(hist, wanted)
        if seen is not None:
            if at_place != seen["near_place"]:
                return self._call("navigate_to", f"I saw the {wanted} at the {seen['near_place']}; going there.",
                                  place=seen["near_place"])
            if last_call.name == "detect_objects" and misses == 0:
                return self._call("pick", f"The {wanted} ({seen['object_id']}) is in reach; picking it up.",
                                  object_id=seen["object_id"])
            if misses > self.max_detect_retries:
                return say(f"I saw the {wanted} earlier but cannot see it at the {at_place} any more.")
            return self._call("detect_objects", f"Checking that the {wanted} is still here.")

        looked_here = last_call.name == "detect_objects"
        if not looked_here or (at_place in rooms and misses < self.max_detect_retries):
            return self._call("detect_objects", "Nothing yet; looking again (detection can miss)." if looked_here else "Looking around.")
        visited = {c.arguments["place"] for c, _ in hist if c.name == "navigate_to"}
        remaining = [room for room in rooms if room not in visited]
        if not remaining:
            return say(f"I searched {', '.join(rooms)} and did not find the {wanted}.")
        return self._call("navigate_to", f"No {wanted} here. Searching the {remaining[0]}.", place=remaining[0])


# -- transcript queries (pure functions) ----------------------------------------------------------
def _history(messages: Sequence[Message]) -> list[tuple[ToolCall, JsonDict]]:
    """(call, parsed result) pairs in order."""
    calls: dict[str, ToolCall] = {}
    out: list[tuple[ToolCall, JsonDict]] = []
    for m in messages:
        if isinstance(m, AssistantMessage):
            calls.update({c.id: c for c in m.tool_calls})
        elif isinstance(m, ToolResultsMessage):
            out += [(calls[r.call_id], json.loads(r.content)) for r in m.results if r.call_id in calls]
    return out


def _resolve_surface(words: str, surfaces: list[str]) -> str | None:
    key = words.replace(" ", "_")
    if key in surfaces:
        return key
    candidates = [s for s in surfaces if words.split()[-1] in s.split("_")]
    return candidates[0] if len(candidates) == 1 else None  # ambiguous ("table"): ask, never guess


def _at_place(hist: list[tuple[ToolCall, JsonDict]]) -> str | None:
    at = None
    for c, r in hist:
        if c.name == "navigate_to":
            at = c.arguments["place"] if r["ok"] else None
    return at


def _holding(hist: list[tuple[ToolCall, JsonDict]]) -> str | None:
    holding = None
    for c, r in hist:
        if c.name == "pick" and r["ok"]:
            holding = c.arguments["object_id"]
        elif c.name == "place" and r["ok"]:
            holding = None
    return holding


def _count(hist: list[tuple[ToolCall, JsonDict]], name: str, ok: bool) -> int:
    return sum(1 for c, r in hist if c.name == name and bool(r["ok"]) is ok)


def _matches(label: str, wanted: str) -> bool:
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


def _latest_match(hist: list[tuple[ToolCall, JsonDict]], wanted: str) -> JsonDict | None:
    for c, r in reversed(hist):
        if c.name == "detect_objects" and r["ok"]:
            for d in r["data"]["detections"]:
                if _matches(d["label"], wanted):
                    return d
    return None


def _consecutive_misses(hist: list[tuple[ToolCall, JsonDict]], wanted: str) -> int:
    """How many detect_objects calls in a row (most recent first) did not report the object."""
    n = 0
    for c, r in reversed(hist):
        if c.name != "detect_objects":
            break
        if r["ok"] and any(_matches(d["label"], wanted) for d in r["data"]["detections"]):
            break
        n += 1
    return n
