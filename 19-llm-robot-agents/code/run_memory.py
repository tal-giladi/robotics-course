"""run_memory.py - a robot that remembers where it saw things (19.08).

Three phases, in one run, all offline:

    A  patrol   drive through the rooms, look, and fill the object memory
    B  fetch    fetch the keys using the memory: no search
    C  moved    someone moves the keys; the stale belief is caught, revised, and the robot recovers

    py 19-llm-robot-agents/code/run_memory.py                 # all three phases
    py 19-llm-robot-agents/code/run_memory.py --no-memory     # the same fetch with an empty memory
    py 19-llm-robot-agents/code/run_memory.py --object "water bottle" --target kitchen_table
    py 19-llm-robot-agents/code/run_memory.py --ask "where did I leave the keys"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.memory import (  # noqa: E402
    ContextConfig,
    EpisodicMemory,
    MemoryContext,
    ObjectMemory,
    RememberingSkills,
    SemanticMap,
    plan_from_memory,
)
from robot_agent.planning import Goal  # noqa: E402
from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402
from robot_agent.verify import ClosedLoopExecutor, LoopConfig, SearchReplanner  # noqa: E402


def patrol(skills: RememberingSkills, rooms: list[str]) -> float:
    t0 = skills.world.t
    for room in rooms:
        skills.call("navigate_to", {"place": room})
        for _ in range(2):
            skills.call("detect_objects", {"min_confidence": 0.4})
    return skills.world.t - t0


def fetch(skills: RememberingSkills, goal: Goal, memory: ObjectMemory, semantic: SemanticMap,
          label: str) -> tuple[str, float, str]:
    plan, why = plan_from_memory(goal, memory, semantic, skills.world.t)
    t0 = skills.world.t
    cfg = LoopConfig(max_replans=4)  # a stale belief costs one wasted drive and up to four searches
    report = ClosedLoopExecutor(skills, goal, cfg, SearchReplanner(semantic.search_order(label))).run(plan)
    return report.outcome, skills.world.t - t0, why


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--object", default="keys")
    ap.add_argument("--target", default="kitchen_table")
    ap.add_argument("--no-memory", action="store_true", help="skip the patrol: fetch with an empty memory")
    ap.add_argument("--move-to", default="kitchen_counter", help="where phase C moves the object")
    ap.add_argument("--ask", default="", help="a question to answer from episodic memory")
    ap.add_argument("--seed", type=int, default=2)
    args = ap.parse_args()

    home = HomeWorld(seed=args.seed)
    memory, episodes = ObjectMemory(), EpisodicMemory()
    semantic = SemanticMap(home.places)
    skills = RememberingSkills(RobotSkills(home), memory, episodes)
    goal = Goal(args.object, args.target)
    episodes.add("task", f"the user asked for the {args.object} on the {args.target}", home.t)

    if not args.no_memory:
        t = patrol(skills, semantic.rooms)
        print(f"A PATROL   {t:.1f} s, {len(memory.beliefs)} objects remembered")
        for b in memory.snapshot(home.t):
            print(f"            {b['label']:<14} {b['place']:<16} belief {b['belief_now']:.2f}  "
                  f"seen {b['times_seen']}x, missed {b['times_missed']}x")
    else:
        print("A PATROL   skipped (--no-memory)")

    outcome, t, why = fetch(skills, goal, memory, semantic, args.object)
    print(f"\nB FETCH    {outcome} in {t:.1f} s\n            start chosen because: {why}")

    obj = next(o for o in home.objects.values() if o.label == args.object)
    obj.on = args.move_to
    surface = home.places[args.move_to].surface_xy
    obj.x, obj.y = surface
    print(f"\n--- someone moves the {args.object} to the {args.move_to} while the robot is elsewhere ---")

    outcome, t, why = fetch(skills, goal, memory, semantic, args.object)
    print(f"\nC MOVED    {outcome} in {t:.1f} s\n            start chosen because: {why}")
    for b in memory.snapshot(home.t):
        if b["label"] == args.object:
            print(f"            belief now: {b['place']} at {b['belief_now']:.2f} "
                  f"(seen {b['times_seen']}x, missed {b['times_missed']}x)")

    if args.ask:
        hits = episodes.recall(args.ask, home.t, 3)
        print(f"\nQ  {args.ask!r}")
        for ep, score in hits:
            print(f"   {score:5.2f}  {ep.line(home.t)}")

    print("\n--- the block the model would see ---")
    print(MemoryContext(semantic, memory, episodes, ContextConfig()).render(
        f"Find the {args.object} and put it on the {args.target}.", home.t))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
