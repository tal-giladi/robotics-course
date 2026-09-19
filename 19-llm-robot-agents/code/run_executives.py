"""run_executives.py - the same fetch task run by a state machine (19.04) and a behavior tree (19.05).

    py 19-llm-robot-agents/code/run_executives.py fsm                   # happy path, prints the transition trace
    py 19-llm-robot-agents/code/run_executives.py fsm --grasp-p 0.3     # a clumsy gripper: watch the retries
    py 19-llm-robot-agents/code/run_executives.py fsm --mermaid         # print the machine as a Mermaid diagram
    py 19-llm-robot-agents/code/run_executives.py bt                    # the behavior tree, with its final status tree
    py 19-llm-robot-agents/code/run_executives.py bt --estop-at 12      # press the e-stop at t = 12 s (simulated)
    py 19-llm-robot-agents/code/run_executives.py fsm --estop-at 12
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.fsm import FetchConfig, build_fetch_fsm  # noqa: E402
from robot_agent.sim_world import ArmParams, HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("executive", choices=["fsm", "bt"])
    ap.add_argument("--object", default="water bottle")
    ap.add_argument("--target", default="kitchen_table")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--grasp-p", type=float, default=0.85, help="grasp success probability")
    ap.add_argument("--estop-at", type=float, default=None, help="press the e-stop at this simulated time (s)")
    ap.add_argument("--deadline", type=float, default=300.0)
    ap.add_argument("--mermaid", action="store_true")
    args = ap.parse_args()

    home = HomeWorld(seed=args.seed, arm=ArmParams(grasp_success_p=args.grasp_p))
    skills = RobotSkills(home)

    if args.executive == "fsm":
        fsm = build_fetch_fsm(skills, FetchConfig(args.object, args.target, deadline_s=args.deadline))
        if args.mermaid:
            print(fsm.to_mermaid())
            print(fsm.states["FIND"].to_mermaid())  # type: ignore[union-attr]
            return 0
        if args.estop_at is not None:  # between states only: a blocking skill must also watch the e-stop itself
            original = skills.call

            def call(name, a=None, call_id=None, cancel=None):  # type: ignore[no-untyped-def]
                handle_estop()
                return original(name, a, call_id, cancel)

            skills.call = call  # type: ignore[method-assign]
        bb = {"t_start": home.t}

        def handle_estop() -> None:
            if args.estop_at is not None and home.t >= args.estop_at:
                home.estop = True

        if args.estop_at is not None:
            home.base_read = home.base.read

            def read_and_check():  # type: ignore[no-untyped-def]
                handle_estop()
                return home.base_read()

            home.base.read = read_and_check  # type: ignore[method-assign]
        outcome = fsm.execute(bb)
        home.base.stop()
        for e in fsm.trace:
            print(f"t={e.t:6.2f}  {e.machine:<5} {e.state:<10} --{e.outcome}--> {e.next}")
        print(f"\nOUTCOME {outcome}   bottle on: {home.objects['bottle-1'].on}   holding: {home.holding}   robot time {home.t:.1f} s")
        return 0 if outcome == "succeeded" else 1

    import py_trees

    from robot_agent.bt import FetchTreeConfig, build_fetch_tree, run_tree

    root, _ = build_fetch_tree(skills, FetchTreeConfig(args.object, args.target, deadline_s=args.deadline))

    def on_tick(tick: int, _root: object) -> None:
        if args.estop_at is not None and home.t >= args.estop_at:
            home.estop = True

    run = run_tree(root, skills, on_tick=on_tick)
    for t, leaf, status in run.trace:
        print(f"t={t:6.2f}  {leaf:<22} {status}")
    print(py_trees.display.unicode_tree(root, show_status=True))
    print(f"OUTCOME {run.status.value}   ticks {run.ticks}   bottle on: {home.objects['bottle-1'].on}   "
          f"holding: {home.holding}   robot time {home.t:.1f} s")
    print("last skill calls:", [(c.skill, c.result.code) for c in skills.log[-3:]])
    return 0 if run.status.value == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
