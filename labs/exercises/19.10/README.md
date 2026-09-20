# 19.10 — An MCP handler for a robot, and honest scoring

Lesson: [19.10 Exposing ROS 2 to agents — MCP servers and evaluation](../../../19-llm-robot-agents/19.10-ros2-for-agents-mcp.md)

Two small things that carry the lesson: the dispatch of a stateless protocol in front of a very
stateful robot, and the arithmetic that stops you reporting "it works" from five runs.

> [!IMPORTANT]
> **Version-sensitive** (MCP revision 2026-07-28, verified 2026-09). That revision removed the
> `initialize` handshake: every request carries its version in `_meta` and the server accepts or
> rejects it per request. Check https://modelcontextprotocol.io/specification/2026-07-28/ before
> porting this to a real client.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `Handler.handle` | notifications, version check, `server/discover`, `tools/list`, `tools/call` |
| `Handler.call_tool` | argument validation, unknown tool, the result cache, tool vs protocol errors |
| `wilson` | a confidence interval for a success rate |
| `score` | one cell of the evaluation table, including `false_success` |
| `regressions` | what the change broke |

`error`, `result` and `tool_result` are given.

## Check

```bash
python course.py check 19.10              # your code
python course.py check 19.10 --solution   # the reference
```

## Hints

* The version is checked on **every** request, including the one after a successful
  `server/discover`. There is no session to remember it in.
* An unknown *tool* is a JSON-RPC error; a *skill failure* is `isError: true` inside a normal
  result. The rule: a protocol error means no argument the model could change would help, so do
  not make the model guess at one.
* The result cache is keyed by `str(request_id)`. Without it, a client that retries after a
  dropped connection drives the robot twice and both calls are "correct".
* `score` must count `false_success` — runs where the agent reported `completed` and the world
  disagrees. If that number is not zero, no other number in the row means anything.
* Do not compute the median with `sum(...)/len(...)`. A single 300-second run must not be able to
  make a row look bad, or a single 4-second failure make it look good.
