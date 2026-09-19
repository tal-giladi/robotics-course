"""run_planner.py - plan, validate, then execute the plan as a behavior tree (19.06).

    py 19-llm-robot-agents/code/run_planner.py                       # a good plan: validated, then run
    py 19-llm-robot-agents/code/run_planner.py --plan literal_id     # rejected: invented object id
    py 19-llm-robot-agents/code/run_planner.py --plan hallucinated   # rejected: skills that do not exist
    py 19-llm-robot-agents/code/run_planner.py --plan bad_order      # rejected: right skills, wrong order
    py 19-llm-robot-agents/code/run_planner.py --plan sightseeing    # rejected: valid, but misses the goal
    py 19-llm-robot-agents/code/run_planner.py --plan bad_order --repair   # errors go back, model replans
    py 19-llm-robot-agents/code/run_planner.py --llm anthropic --model claude-opus-5

Nothing moves until the validator accepts a plan.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.planning import (  # noqa: E402
    Goal,
    MockPlanner,
    PlanExecConfig,
    plan_and_repair,
    plan_to_tree,
)
from robot_agent.sim_world import ArmParams, HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", default="good",
                    choices=["good", "literal_id", "hallucinated", "bad_order", "sightseeing"],
                    help="which plan the mock planner submits first")
    ap.add_argument("--repair", action="store_true", help="return the errors to the planner and let it replan")
    ap.add_argument("--llm", choices=["mock", "anthropic"], default="mock")
    ap.add_argument("--model", default="claude-opus-5")
    ap.add_argument("--object", default="water bottle")
    ap.add_argument("--target", default="kitchen_table")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--grasp-p", type=float, default=0.85)
    ap.add_argument("--attempts", type=int, default=3)
    args = ap.parse_args()

    home = HomeWorld(seed=args.seed, arm=ArmParams(grasp_success_p=args.grasp_p))
    skills = RobotSkills(home)
    task = f"Find the {args.object} and put it on the {args.target}."
    goal = Goal(args.object, args.target)

    if args.llm == "anthropic":
        from robot_agent.anthropic_llm import AnthropicLLM

        planner = AnthropicLLM(model=args.model)
    else:
        planner = MockPlanner(variant=args.plan, repair=args.repair, obj=args.object, target=args.target)

    print(f"TASK  {task}\nGOAL  a '{goal.label}' on '{goal.surface}'\n")
    plan, attempts = plan_and_repair(planner, skills, task, goal,
                                     max_attempts=args.attempts if args.repair or args.llm == "anthropic" else 1)
    for i, attempt in enumerate(attempts, start=1):
        print(f"--- attempt {i} ---  {attempt.text}")
        if attempt.raw:
            print("\n".join(f"  {n:>2}. {s.get('skill')} {s.get('args')}"
                            + (f"  -> ${s['bind']} = first '{s.get('label')}'" if s.get("bind") else "")
                            for n, s in enumerate(attempt.raw.get("steps", []), start=1)))
        print("  VALIDATOR: " + ("accepted" if attempt.plan else f"rejected, {len(attempt.errors)} error(s)"))
        for e in attempt.errors:
            print(f"    {e}")
        print()
    if plan is None:
        print(f"NO VALID PLAN after {len(attempts)} attempt(s). The robot has not moved: "
              f"t = {home.t:.1f} s, skill calls = {len(skills.log)}.")
        return 1

    import py_trees

    from robot_agent.bt import run_tree

    root, bindings = plan_to_tree(plan, skills, PlanExecConfig())
    run = run_tree(root, skills)
    for t, leaf, status in run.trace:
        print(f"t={t:6.2f}  {leaf:<22} {status}")
    print(py_trees.display.unicode_tree(root, show_status=True))
    print(f"OUTCOME {run.status.value}   ticks {run.ticks}   bindings: "
          f"{ {k: v['object_id'] for k, v in bindings.items()} }   "
          f"bottle on: {home.objects['bottle-1'].on}   holding: {home.holding}   robot time {home.t:.1f} s")
    return 0 if run.status.value == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
