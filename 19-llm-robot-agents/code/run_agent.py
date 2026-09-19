"""run_agent.py - an LLM operates the simulated home robot through tools (19.03).

    py 19-llm-robot-agents/code/run_agent.py                                   # offline mock planner
    py 19-llm-robot-agents/code/run_agent.py --gullible                        # prompt injection demo
    py 19-llm-robot-agents/code/run_agent.py --llm anthropic --model claude-opus-5
    py 19-llm-robot-agents/code/run_agent.py --task "Find the keys and put it on the sofa." --seed 3

Every tool call is appended as one JSON line to --log (default: runs/agent_calls.jsonl).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]

from robot_agent.agent_loop import AgentLoop, Budget  # noqa: E402
from robot_agent.llm import FetchPolicyLLM  # noqa: E402
from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402


def print_event(kind: str, data: dict) -> None:
    if kind == "task":
        print(f"TASK  {data['task']}\nLLM   {data['llm']}   tools: {', '.join(data['tools'])}\n")
    elif kind == "assistant":
        if data["text"]:
            print(f"[{data['turn']:>2}] LLM   {data['text']}")
        for name, args in data["tool_calls"]:
            print(f"     CALL  {name}({json.dumps(args)})")
    elif kind == "tool":
        extra = ""
        if data["skill"] == "detect_objects" and data["ok"]:
            extra = "  " + ", ".join(f"{d.get('object_id')}:{d.get('label')}@{d.get('near_place')}({d.get('confidence')})"
                                     for d in data["data"].get("detections", []))
            if any(d.get("untrusted_text_seen") for d in data["data"].get("detections", [])):
                extra += "  [text seen in the world]"
        print(f"     -> {data['code']:<12} {data['message']}  (robot {data['robot_elapsed_s']} s, t={data['robot_t_s']} s){extra}")
    elif kind == "done":
        print(f"\nOUTCOME  {data['outcome']}\nANSWER   {data['text']}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="Find the water bottle and put it on the kitchen table.")
    ap.add_argument("--llm", choices=["mock", "anthropic"], default="mock")
    ap.add_argument("--model", default="claude-opus-5")
    ap.add_argument("--gullible", action="store_true", help="mock planner obeys text it reads in the world")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-turns", type=int, default=25)
    ap.add_argument("--max-tool-calls", type=int, default=40)
    ap.add_argument("--log", default=str(HERE / "runs" / "agent_calls.jsonl"))
    args = ap.parse_args()

    if args.llm == "anthropic":
        from robot_agent.anthropic_llm import AnthropicLLM

        llm = AnthropicLLM(model=args.model)
    else:
        llm = FetchPolicyLLM(gullible=args.gullible)
    home = HomeWorld(seed=args.seed)
    skills = RobotSkills(home)
    loop = AgentLoop(llm, skills, budget=Budget(max_llm_turns=args.max_turns, max_tool_calls=args.max_tool_calls),
                     log_path=args.log, on_event=print_event)
    run = loop.run(args.task)
    where = {o.id: o.on for o in home.objects.values() if o.graspable}
    print(f"WORLD    objects: {where}   holding: {home.holding}   robot time: {home.t:.1f} s   "
          f"LLM turns: {run.llm_turns}   tool calls: {len(run.tool_log)}   usage: {run.usage or '-'}")
    return 0 if run.outcome == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
