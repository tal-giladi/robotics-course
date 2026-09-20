"""run_mcp.py - the robot's skills as an MCP server, and the evaluation suite (19.10).

    py 19-llm-robot-agents/code/run_mcp.py                 # a client session, message by message
    py 19-llm-robot-agents/code/run_mcp.py --mode patrol   # the tool list shrinks with the mode
    py 19-llm-robot-agents/code/run_mcp.py --bad-version   # what a version mismatch looks like
    py 19-llm-robot-agents/code/run_mcp.py --stdio         # a real stdio server (Ctrl-C to stop)
    py 19-llm-robot-agents/code/run_mcp.py --eval          # the whole evaluation suite, 5 seeds
    py 19-llm-robot-agents/code/run_mcp.py --eval --seeds 3 --tasks fetch_bottle,injected

Version-sensitive: MCP revision 2026-07-28. Check the spec before copying the wire format.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.evaluation import RUNNERS, SUITE, regressions, run_suite, table  # noqa: E402
from robot_agent.mcp_server import (  # noqa: E402
    PROTOCOL_VERSION,
    InProcessClient,
    RobotMCPServer,
    serve_stdio,
)
from robot_agent.safety import Geofence, PhysicalBudget, SafetyLayer  # noqa: E402
from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402


def build_server(mode: str, seed: int) -> RobotMCPServer:
    home = HomeWorld(seed=seed)
    layer = SafetyLayer(RobotSkills(home), mode, geofence=Geofence.bedroom_at_night(home),
                        budget=PhysicalBudget(max_distance_m=60.0, max_robot_time_s=300.0))
    layer.accept_task("water bottle", "kitchen_table")
    return RobotMCPServer(layer)


def show(direction: str, message: dict) -> None:
    text = json.dumps(message, ensure_ascii=False)
    print(f"{direction} {text if len(text) < 300 else text[:297] + '...'}")


def session(mode: str, seed: int, bad_version: bool) -> int:
    server = build_server(mode, seed)
    client = InProcessClient(server)
    version = "2024-01-01" if bad_version else PROTOCOL_VERSION

    print("# 1. discovery — no handshake: the version is stated per request")
    show("-->", {"method": "server/discover", "_meta": {"protocolVersion": version}})
    discover = client.send("server/discover", version=version)
    show("<--", discover)
    if "error" in discover:
        print("\nThe client now picks a version from error.data.supported and retries.")
        return 1

    print("\n# 2. tools/list — what this mode exposes")
    tools = client.list_tools()
    for tool in tools:
        print(f"    {tool['name']:<16} {tool['description'][:96]}")

    print("\n# 3. tools/call — a read-only call")
    show("<--", client.call_tool("get_robot_state"))

    print("\n# 4. tools/call — one that moves the robot for 13 s")
    response = client.call_tool("navigate_to", {"place": "kitchen"})
    show("<--", {"isError": response["result"]["isError"],
                 "structuredContent": response["result"]["structuredContent"]})

    print("\n# 5. the same request id again — idempotency, not a second drive")
    before = server.skills.world.t
    replay = server.handle({"jsonrpc": "2.0", "id": "req-4", "method": "tools/call",
                            "params": {"name": "navigate_to", "arguments": {"place": "kitchen"},
                                       "_meta": {"io.modelcontextprotocol/protocolVersion": PROTOCOL_VERSION}}})
    print(f"    robot time before {before:.1f} s, after {server.skills.world.t:.1f} s, "
          f"isError={replay['result']['isError']}")

    print("\n# 6. a denied call — a tool error the model can read, not a crash")
    denied = client.call_tool("navigate_to", {"place": "bed"})
    show("<--", denied["result"]["structuredContent"])

    print("\n# 7. an unknown tool — a protocol error, because no argument fixes it")
    show("<--", client.call_tool("open_fridge", {}))
    return 0


def evaluate(seeds: int, task_names: str) -> int:
    tasks = [t for t in SUITE if not task_names or t.name in task_names.split(",")]
    print(f"tasks: {', '.join(t.name for t in tasks)}   seeds: {list(range(seeds))}   "
          f"runners: {len(RUNNERS)}\n")
    for t in tasks:
        print(f"  {t.name:<14} {t.expect}")
    print()
    cells = run_suite(tasks=tasks, seeds=tuple(range(seeds)))
    print(table(cells, tasks))
    print("\nper-cell detail (successes/runs [95 % Wilson interval] median, worst robot seconds)")
    for (runner, task), cell in cells.items():
        print(f"  {runner:<22} {task:<14} {cell.summary()}")
    regs = regressions(cells, "closed loop (19.07)", "guarded (19.09)")
    print("\nregressions from adding the safety layer: " + ("; ".join(regs) if regs else "none"))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="autonomous")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--bad-version", action="store_true")
    ap.add_argument("--stdio", action="store_true", help="serve MCP on stdin/stdout")
    ap.add_argument("--eval", action="store_true", help="run the evaluation suite")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--tasks", default="")
    args = ap.parse_args()

    if args.stdio:
        serve_stdio(build_server(args.mode, args.seed))
        return 0
    if args.eval:
        return evaluate(args.seeds, args.tasks)
    return session(args.mode, args.seed, args.bad_version)


if __name__ == "__main__":
    raise SystemExit(main())
