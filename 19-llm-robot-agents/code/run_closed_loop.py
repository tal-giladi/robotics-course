"""run_closed_loop.py - plan, execute, verify every step, repair, replan (19.07).

    py 19-llm-robot-agents/code/run_closed_loop.py                     # verified execution
    py 19-llm-robot-agents/code/run_closed_loop.py --verify none       # open loop, for comparison
    py 19-llm-robot-agents/code/run_closed_loop.py --fault phantom_grasp
    py 19-llm-robot-agents/code/run_closed_loop.py --fault blind_gripper
    py 19-llm-robot-agents/code/run_closed_loop.py --fault blind_gripper --verify state
    py 19-llm-robot-agents/code/run_closed_loop.py --fault short_nav
    py 19-llm-robot-agents/code/run_closed_loop.py --object "red cup" --target sofa   # wrong world
    py 19-llm-robot-agents/code/run_closed_loop.py --object "red cup" --target sofa --no-replan

Everything runs offline: the plan comes from the mock planner of 19.06 and the replanner is the
deterministic ``SearchReplanner``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.planning import Goal, MockPlanner, plan_and_repair  # noqa: E402
from robot_agent.sim_world import ArmParams, HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402
from robot_agent.verify import (  # noqa: E402
    FAULTS,
    ClosedLoopExecutor,
    FaultySkills,
    LoopConfig,
    SearchReplanner,
    VerifyConfig,
)


def printer(kind: str, data: dict) -> None:
    if kind == "plan":
        print("PLAN  " + " · ".join(f"{i}.{s['skill']}" for i, s in enumerate(data["plan"]["steps"], 1)))
    elif kind == "step":
        r, v = data["record"], data["verification"]
        mark = {"VERIFIED": "ok ", "NOT_VERIFIED": "XX ", "UNCHECKABLE": "?? ", "SKIPPED": "-- "}[v.code]
        print(f"t={r.t_end:6.2f}  {r.index}.{r.skill:<15} {r.code:<12} {mark}{v.method:<10} "
              f"{v.message[:64]:<64} {r.repair}")
    elif kind == "replan":
        print(f"\nREPLAN #{data['attempt']} by {data['by']} because {data['because']}\n")
    elif kind == "goal":
        print(f"\nGOAL CHECK  {data['verification']}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--object", default="water bottle")
    ap.add_argument("--target", default="kitchen_table")
    ap.add_argument("--where", default="kitchen", help="room the planner will look in first")
    ap.add_argument("--verify", choices=["none", "state", "perception"], default="perception")
    ap.add_argument("--fault", choices=list(FAULTS), default="none")
    ap.add_argument("--no-replan", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--grasp-p", type=float, default=0.85)
    args = ap.parse_args()

    home = HomeWorld(seed=args.seed, arm=ArmParams(grasp_success_p=args.grasp_p))
    skills = RobotSkills(home)
    goal = Goal(args.object, args.target)
    task = f"Find the {args.object} and put it on the {args.target}."

    planner = MockPlanner(obj=args.object, where=args.where, target=args.target)
    plan, _ = plan_and_repair(planner, skills, task, goal, max_attempts=1)
    if plan is None:
        print("no valid plan")
        return 1

    executed = FaultySkills(skills, args.fault) if args.fault != "none" else skills
    cfg = LoopConfig(verify=VerifyConfig(level=args.verify))
    executor = ClosedLoopExecutor(executed, goal, cfg,
                                  None if args.no_replan else SearchReplanner(), on_event=printer)
    print(f"TASK  {task}\nFAULT {args.fault}   VERIFY {args.verify}\n")
    report = executor.run(plan)
    print("\n" + report.summary())
    print(f"WORLD  {args.object}: on={next((o.on for o in home.objects.values() if o.label == args.object), '?')}"
          f"   holding={home.holding}   lies told={getattr(executed, 'lies', 0)}")
    return 0 if report.outcome == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
