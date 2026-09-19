"""Claude adapter for the agent loop (19.03) - Anthropic Messages API tool use.

Version-sensitive (verified 2026-09 against the official docs and the `anthropic` Python SDK 1.6.0):
https://platform.claude.com/docs/en/agents-and-tools/tool-use/overview
https://platform.claude.com/docs/en/about-claude/models/overview

Wire format, in short:
* request:  ``tools=[{"name", "description", "input_schema"}]``, ``messages=[{"role", "content"}]``
* response: ``stop_reason == "tool_use"`` and ``content`` holds ``tool_use`` blocks ``{id, name, input}``
* next request: append the assistant ``content`` unchanged, then ONE user message whose content is a
  list of ``{"type": "tool_result", "tool_use_id", "content", "is_error"}`` blocks.

Claude Opus 5 thinks adaptively by default; its thinking blocks are part of ``response.content``
and must be echoed back unchanged, which is why ``AssistantMessage.raw`` keeps the native blocks.
With ``refusal_fallback=True`` (default for Opus 5) the request opts into the server-side refusal
fallback (beta), so a safety-classifier decline is retried on a fallback model inside the same call.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from robot_agent.llm import AssistantMessage, JsonDict, Message, ToolCall, ToolResultsMessage, UserMessage

DEFAULT_MODEL = "claude-opus-5"  # also: "claude-sonnet-5", "claude-haiku-4-5" (verified 2026-09)
FALLBACK_BETA = "server-side-fallback-2026-07-01"
_FALLBACK_MODELS = ("claude-opus-5", "claude-fable-5-1")


def to_anthropic_messages(messages: Sequence[Message]) -> list[JsonDict]:
    """Provider-neutral transcript -> Messages API ``messages`` (pure, testable offline)."""
    out: list[JsonDict] = []
    for m in messages:
        if isinstance(m, UserMessage):
            out.append({"role": "user", "content": m.text})
        elif isinstance(m, AssistantMessage):
            if m.raw is not None:
                content: Any = m.raw  # native blocks, including thinking blocks, unchanged
            else:
                content = ([{"type": "text", "text": m.text}] if m.text else []) + [
                    {"type": "tool_use", "id": c.id, "name": c.name, "input": c.arguments} for c in m.tool_calls]
            out.append({"role": "assistant", "content": content})
        elif isinstance(m, ToolResultsMessage):
            out.append({"role": "user", "content": [
                {"type": "tool_result", "tool_use_id": r.call_id, "content": r.content, "is_error": r.is_error}
                for r in m.results]})
    return out


def from_anthropic_response(response: Any) -> AssistantMessage:
    """Messages API response -> ``AssistantMessage``. Unknown block types are kept in ``raw`` only."""
    texts: list[str] = []
    calls: list[ToolCall] = []
    for block in response.content:
        if block.type == "text":
            texts.append(block.text)
        elif block.type == "tool_use":
            calls.append(ToolCall(block.id, block.name, dict(block.input)))
    usage = getattr(response, "usage", None)
    usage_dict = {k: int(getattr(usage, k) or 0) for k in ("input_tokens", "output_tokens") if usage is not None and hasattr(usage, k)}
    return AssistantMessage("\n".join(texts), tuple(calls), str(response.stop_reason), usage_dict, raw=list(response.content))


class AnthropicLLM:
    """``LLMBackend`` for Claude. Needs ``pip install anthropic`` and credentials (ANTHROPIC_API_KEY)."""

    def __init__(self, model: str = DEFAULT_MODEL, max_tokens: int = 16000, client: Any = None,
                 refusal_fallback: bool | None = None, effort: str | None = None) -> None:
        if client is None:
            import anthropic  # imported lazily: the offline lab does not need the SDK

            client = anthropic.Anthropic()
        self.client = client
        self.model = model
        self.name = f"anthropic:{model}"
        self.max_tokens = max_tokens
        self.refusal_fallback = model in _FALLBACK_MODELS if refusal_fallback is None else refusal_fallback
        self.effort = effort  # "low" | "medium" | "high" (default) ... ; lower = fewer tokens and less latency

    def request_kwargs(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> JsonDict:
        kwargs: JsonDict = {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": system,
            "tools": [dict(t) for t in tools],
            "messages": to_anthropic_messages(messages),
        }
        if self.effort is not None:
            kwargs["output_config"] = {"effort": self.effort}
        if self.refusal_fallback:
            kwargs["betas"] = [FALLBACK_BETA]
            kwargs["fallbacks"] = "default"
        return kwargs

    def complete(self, system: str, messages: Sequence[Message], tools: Sequence[JsonDict]) -> AssistantMessage:
        kwargs = self.request_kwargs(system, messages, tools)
        api = self.client.beta.messages if self.refusal_fallback else self.client.messages
        response = api.create(**kwargs)
        return from_anthropic_response(response)
