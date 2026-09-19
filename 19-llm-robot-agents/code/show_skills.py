"""show_skills.py - the skill API as the LLM sees it, and the gateway as a caller feels it (19.02).

    py 19-llm-robot-agents/code/show_skills.py                 # catalogue + one full tool definition
    py 19-llm-robot-agents/code/show_skills.py --schema pick   # the full JSON schema of one skill
    py 19-llm-robot-agents/code/show_skills.py --calls         # a session of good and bad calls
    py 19-llm-robot-agents/code/show_skills.py --calls --confirm-manipulation

Nothing here needs an LLM, an API key or hardware: the world is the simulated apartment.
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

from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills, SkillSpec  # noqa: E402


def print_catalogue(skills: RobotSkills) -> None:
    defs = skills.tool_definitions()
    print(f"{len(defs)} tools, {len(json.dumps(defs))} characters of tool definitions "
          f"(~{len(json.dumps(defs)) // 4} tokens) sent with every request\n")
    print(f"{'skill':<16}{'effect':<14}{'idempotent':<12}{'timeout_s':>10}  failure codes")
    for name in sorted(skills.specs):
        s: SkillSpec = skills.specs[name]
        timeout = f"{s.default_timeout_s:.0f}/{s.max_timeout_s:.0f}" if s.default_timeout_s else "-"
        print(f"{s.name:<16}{s.effect:<14}{str(s.idempotent):<12}{timeout:>10}  {', '.join(s.failure_codes) or '-'}")


def print_calls(skills: RobotSkills) -> None:
    session: list[tuple[str, dict, str | None]] = [
        ("navigate_to", {"place": "kitchen"}, None),              # fine
        ("navigate_to", {"place": "garage"}, None),               # not in the enum
        ("navigate_to", {"place": "kitchen", "timeout_s": 900}, None),   # above the maximum
        ("navigate_to", {"place": "kitchen", "timeout_s": "30 s"}, None),  # a string, not a number
        ("navigate_to", {"place": "kitchen", "speed": 2.0}, None),  # invented parameter
        ("pick", {"object_id": "bottle-1"}, None),                # precondition: never detected
        ("detect_objects", {}, None),
        ("pick", {"object_id": "bottle-1"}, "call-7"),            # precondition: out of reach
        ("pick", {"object_id": "bottle-1"}, "call-7"),            # same call_id: served from the cache
        ("drive", {"left": 1.0, "right": 1.0}, None),             # not a skill at all
    ]
    print(f"{'#':>2}  {'call':<52}{'ok':<7}{'code':<22}message / hint")
    for i, (name, args, call_id) in enumerate(session, start=1):
        r = skills.call(name, args, call_id=call_id)
        call = f"{name}({json.dumps(args, separators=(',', ':'))})"
        cached = " [cached]" if skills.log[-1].cached else ""
        print(f"{i:>2}  {call:<52}{str(r.ok):<7}{r.code + cached:<22}{r.message[:70]}")
        if r.hint:
            print(f"{'':<63}hint: {r.hint}")
    print(f"\nrobot time used: {skills.world.t:.1f} s   calls logged: {len(skills.log)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--schema", metavar="SKILL", help="print the full tool definition of one skill")
    ap.add_argument("--calls", action="store_true", help="run a session of good and bad calls")
    ap.add_argument("--confirm-manipulation", action="store_true",
                    help="require a human 'yes' before any arm motion (auto-answers 'no' here)")
    args = ap.parse_args()

    home = HomeWorld(seed=0)
    confirm_effects = frozenset({"manipulation"}) if args.confirm_manipulation else frozenset()
    skills = RobotSkills(home, confirm=lambda spec, a: False, confirm_effects=confirm_effects)

    if args.schema:
        if args.schema not in skills.specs:
            print(f"unknown skill '{args.schema}'; known: {', '.join(sorted(skills.specs))}")
            return 2
        print(json.dumps(skills.specs[args.schema].tool_definition(), indent=2))
        return 0
    if args.calls:
        print_calls(skills)
        return 0
    print_catalogue(skills)
    print("\n--- one full tool definition ---")
    print(json.dumps(skills.specs["navigate_to"].tool_definition(), indent=2)[:1200])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
