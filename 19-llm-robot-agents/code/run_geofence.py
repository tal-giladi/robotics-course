"""run_geofence.py - what a geofence checks, and what it misses when it only checks the goal (19.09).

    py 19-llm-robot-agents/code/run_geofence.py

Two questions, answered with the same A* path the robot would actually drive:

1. Which named places are *inside* the night-time bedroom keep-out zone? (A goal-only check finds
   exactly these, and the course apartment is kind to a goal-only check: the "bedroom" door pose
   and the "bed" pose are the same point, so both are caught either way.)
2. Where does a goal-only check fail? A freshly mopped patch of the hallway is a keep-out zone with
   no named place in it at all. Every destination stays legal; the *route* is not.

Offline, deterministic, no model and no API key.
"""

from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parents[1] / "labs" / "python")]
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from robot_agent.safety import Geofence, Zone  # noqa: E402
from robot_agent.sim_world import HomeWorld  # noqa: E402

#: The floor by the kitchen doorway was just mopped. No place is inside it; the hallway is.
WET_FLOOR = Zone("wet_floor", 3.0, 0.9, 3.6, 1.9, "the floor was just mopped")


def main() -> int:
    home = HomeWorld(seed=0)
    bedroom = Geofence.bedroom_at_night(home)
    zone = bedroom.zones[0]
    print(f"zone '{zone.name}': x {zone.x_min}..{zone.x_max} m, y {zone.y_min}..{zone.y_max} m, "
          f"margin {bedroom.margin_m} m  ({zone.reason})\n")

    print(f"{'place':<16}{'x':>7}{'y':>7}   inside the bedroom zone?")
    for name, place in home.places.items():
        hit = bedroom.violated_by_point(place.x, place.y)
        print(f"{name:<16}{place.x:>7.2f}{place.y:>7.2f}   {'YES' if hit else 'no'}")

    print("\n--- a keep-out zone with no place in it: the mopped hallway ---")
    mopped = Geofence([WET_FLOOR])
    start = home.places["living_room"]
    goal = home.places["kitchen_counter"]
    path = home.plan_path((start.x, start.y), (goal.x, goal.y))
    print(f"route {start.name} -> {goal.name}: {len(path)} waypoints")
    print(f"  destination ({goal.x:.2f}, {goal.y:.2f}) inside the zone? "
          f"{'YES' if mopped.violated_by_point(goal.x, goal.y) else 'no'}   <- a goal-only check allows it")
    crossed = mopped.violated_by_path(path)
    print(f"  the path enters: {crossed.name if crossed else 'nothing'}   <- the path check refuses it")
    inside = [(x, y) for x, y in path if WET_FLOOR.contains(x, y, mopped.margin_m)]
    if inside:
        print(f"  {len(inside)} waypoints are in the zone, from ({inside[0][0]:.2f}, {inside[0][1]:.2f}) "
              f"to ({inside[-1][0]:.2f}, {inside[-1][1]:.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
