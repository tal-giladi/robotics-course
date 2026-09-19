"""12.07 — Read karmel's nav2_params.yaml the way Nav2 does, and check it against the robot.

    python 12-navigation/code/nav2_config.py            # the server tour + consistency checks
    python 12-navigation/code/nav2_config.py --file my_params.yaml

Nav2 has about 200 parameters spread over a dozen servers. This script answers the two questions
you actually have in front of the file: *which node does this block configure and which plugin does
it load*, and *do these numbers still match the robot in labs/config/karmel.yaml*.

The parameter structure is Nav2's: every block is ``<node name>: {ros__parameters: {...}}``, except
the two costmaps, which are owned by the planner and controller servers and so are nested twice
(``local_costmap: {local_costmap: {ros__parameters: ...}}``).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

import nav_common
from robotlab.config import KarmelConfig, load_config

DEFAULT_PARAMS = nav_common.ROOT / "labs" / "ros2_ws" / "src" / "karmel_bringup" / "config" / "nav2_params.yaml"

#: Which node each top-level block configures, and what it is for. Order = lifecycle startup order
#: of nav2_bringup's two lifecycle managers (localization first, then navigation).
SERVERS: list[tuple[str, str, str]] = [
    ("map_server", "localization", "serves the saved map on /map (latched)"),
    ("amcl", "localization", "particle filter: map -> odom (10.09)"),
    ("controller_server", "navigation", "/follow_path; owns local_costmap, progress and goal checkers (12.05)"),
    ("smoother_server", "navigation", "/smooth_path (only used by trees that call SmoothPath)"),
    ("planner_server", "navigation", "/compute_path_to_pose; owns global_costmap (12.02)"),
    ("route_server", "navigation", "route-graph navigation; karmel does not use it"),
    ("behavior_server", "navigation", "/spin /backup /drive_on_heading /wait /assisted_teleop (12.10)"),
    ("velocity_smoother", "navigation", "acceleration and speed limits on cmd_vel"),
    ("collision_monitor", "navigation", "last-line safety between cmd_vel_smoothed and cmd_vel (12.10)"),
    ("bt_navigator", "navigation", "/navigate_to_pose; ticks the behavior tree (12.06)"),
    ("waypoint_follower", "navigation", "/follow_waypoints (12.09)"),
    ("docking_server", "navigation", "charging dock approach; karmel does not use it"),
    ("map_saver", "-", "map_saver_cli defaults, not a running server"),
]


def load_params(path: Path | str = DEFAULT_PARAMS) -> dict[str, Any]:
    """Load a Nav2 parameter file as ``{node_name: {parameter: value}}``, costmaps flattened."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    out: dict[str, Any] = {}
    for name, block in raw.items():
        if "ros__parameters" in block:
            out[name] = block["ros__parameters"]
        elif name in block and "ros__parameters" in block[name]:   # local_costmap / global_costmap
            out[name] = block[name]["ros__parameters"]
        else:
            out[name] = block
    return out


def plugins_of(params: dict[str, Any]) -> list[tuple[str, str]]:
    """Every ``plugin:`` string in a parameter block, as ``(block name, plugin type)``."""
    found: list[tuple[str, str]] = []

    def walk(node: Any, name: str) -> None:
        if isinstance(node, dict):
            if "plugin" in node and isinstance(node["plugin"], str):
                found.append((name, node["plugin"]))
            for key, value in node.items():
                walk(value, key)

    walk(params, "")
    return found


def footprint_of(params: dict[str, Any], costmap: str = "global_costmap") -> list[tuple[float, float]]:
    """The costmap footprint polygon, parsed from Nav2's string-of-a-list form."""
    text = params[costmap]["footprint"]
    return [(float(x), float(y)) for x, y in yaml.safe_load(text)]


def footprint_radii(footprint: list[tuple[float, float]]) -> tuple[float, float]:
    """(inscribed, circumscribed) radius — the same computation as 12.04's costmap.py."""
    lo, hi = math.inf, 0.0
    n = len(footprint)
    for i in range(n):
        (ax, ay), (bx, by) = footprint[i], footprint[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, (-ax * dx - ay * dy) / length2))
        lo = min(lo, math.hypot(ax, ay), math.hypot(ax + t * dx, ay + t * dy))
        hi = max(hi, math.hypot(ax, ay))
    return lo, hi


@dataclass(frozen=True)
class Check:
    ok: bool
    name: str
    detail: str

    def __str__(self) -> str:
        return f"  [{'ok ' if self.ok else 'FAIL'}] {self.name}: {self.detail}"


def check_against_robot(params: dict[str, Any], cfg: KarmelConfig | None = None) -> list[Check]:
    """The cross-file invariants that karmel_bringup's own test_config.py pins in CI."""
    cfg = cfg or load_config()
    checks: list[Check] = []

    footprint = footprint_of(params)
    width = cfg.drive.wheel_separation_m + cfg.drive.wheel_width_m
    expected = {(round(sx * cfg.chassis.length_m / 2, 4), round(sy * width / 2, 4))
                for sx in (1, -1) for sy in (1, -1)}
    actual = {(round(x, 4), round(y, 4)) for x, y in footprint}
    checks.append(Check(actual == expected, "footprint = chassis length x wheel span",
                        f"{sorted(actual)} vs karmel.yaml {sorted(expected)}"))

    inscribed, circumscribed = footprint_radii(footprint)
    inflation = params["global_costmap"]["inflation_layer"]["inflation_radius"]
    checks.append(Check(inflation > inscribed, "inflation_radius > inscribed radius",
                        f"{inflation:.3f} m > {inscribed:.3f} m (circumscribed {circumscribed:.3f} m)"))

    smoother_max = params["velocity_smoother"]["max_velocity"][0]
    cruise = params["controller_server"]["FollowPath"]["desired_linear_vel"]
    checks.append(Check(cruise <= smoother_max <= cfg.drive.max_linear_speed_m_s,
                        "desired_linear_vel <= velocity_smoother max <= karmel.yaml limit",
                        f"{cruise} <= {smoother_max} <= {cfg.drive.max_linear_speed_m_s} m/s"))

    rpp_k = params["controller_server"]["FollowPath"]["inflation_cost_scaling_factor"]
    costmap_k = params["local_costmap"]["inflation_layer"]["cost_scaling_factor"]
    checks.append(Check(rpp_k == costmap_k, "RPP inflation_cost_scaling_factor == costmap cost_scaling_factor",
                        f"{rpp_k} vs {costmap_k} (RPP inverts the inflation formula to get a distance)"))

    lidar_max = params["amcl"]["laser_max_range"]
    checks.append(Check(abs(lidar_max - cfg.sensors.lidar.max_range_m) < 1e-6,
                        "amcl laser_max_range == karmel.yaml lidar range",
                        f"{lidar_max} vs {cfg.sensors.lidar.max_range_m} m"))

    stamped = [name for name in ("controller_server", "behavior_server", "velocity_smoother",
                                 "collision_monitor", "docking_server")
               if not params[name].get("enable_stamped_cmd_vel", False)]
    checks.append(Check(not stamped, "every cmd_vel producer publishes TwistStamped",
                        "all set" if not stamped else f"missing on {stamped} (Jazzy diff_drive_controller needs it)"))

    obstacle_range = params["local_costmap"]["obstacle_layer"]["scan"]["obstacle_max_range"]
    half_window = params["local_costmap"]["width"] / 2
    checks.append(Check(obstacle_range >= half_window, "obstacle_max_range covers the local costmap",
                        f"{obstacle_range} m >= {half_window} m half-window"))
    return checks


def tour(params: dict[str, Any]) -> None:
    """Print the server/plugin map."""
    plugin_map: dict[str, list[tuple[str, str]]] = {}
    for block, plugin in plugins_of(params):
        owner = next((s for s, _, _ in SERVERS if s in params and _contains(params[s], plugin)), None)
        plugin_map.setdefault(owner or "costmaps", []).append((block, plugin))

    print("Nav2 servers in karmel's nav2_params.yaml (lifecycle startup order)\n")
    for name, manager, what in SERVERS:
        mark = " " if name in params else "?"
        print(f"{mark} {name:<20} [{manager:^12}] {what}")
        for block, plugin in plugin_map.get(name, []):
            print(f"      {block:<22} {plugin}")
    print("\n  costmaps (parameter blocks owned by planner_server / controller_server)")
    for costmap in ("global_costmap", "local_costmap"):
        block = params[costmap]
        layers = ", ".join(block.get("plugins", []))
        filters = ", ".join(block.get("filters", [])) or "none"
        print(f"      {costmap:<16} {block['resolution']} m/cell, layers: {layers}; filters: {filters}")


def _contains(block: Any, plugin: str) -> bool:
    if isinstance(block, dict):
        return block.get("plugin") == plugin or any(_contains(v, plugin) for v in block.values())
    return False


def key_numbers(params: dict[str, Any]) -> dict[str, float]:
    """The handful of numbers you quote when explaining what the robot will do."""
    fp = params["controller_server"]["FollowPath"]
    return {
        "controller_frequency_hz": params["controller_server"]["controller_frequency"],
        "desired_linear_vel_m_s": fp["desired_linear_vel"],
        "lookahead_dist_m": fp["lookahead_dist"],
        "xy_goal_tolerance_m": params["controller_server"]["general_goal_checker"]["xy_goal_tolerance"],
        "yaw_goal_tolerance_rad": params["controller_server"]["general_goal_checker"]["yaw_goal_tolerance"],
        "local_costmap_width_m": params["local_costmap"]["width"],
        "local_costmap_update_hz": params["local_costmap"]["update_frequency"],
        "global_costmap_update_hz": params["global_costmap"]["update_frequency"],
        "inflation_radius_m": params["global_costmap"]["inflation_layer"]["inflation_radius"],
        "cost_scaling_factor": params["global_costmap"]["inflation_layer"]["cost_scaling_factor"],
        "max_velocity_m_s": params["velocity_smoother"]["max_velocity"][0],
        "max_accel_m_s2": params["velocity_smoother"]["max_accel"][0],
        "max_decel_m_s2": params["velocity_smoother"]["max_decel"][0],
        "amcl_max_particles": params["amcl"]["max_particles"],
        "amcl_update_min_d_m": params["amcl"]["update_min_d"],
        "bt_default_server_timeout_ms": params["bt_navigator"]["default_server_timeout"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Tour and check a Nav2 parameter file.")
    parser.add_argument("--file", type=Path, default=DEFAULT_PARAMS)
    args = parser.parse_args()

    params = load_params(args.file)
    tour(params)

    print("\nkey numbers")
    for name, value in key_numbers(params).items():
        print(f"      {name:<30} {value}")

    print("\nconsistency with labs/config/karmel.yaml")
    checks = check_against_robot(params)
    for check in checks:
        print(check)
    failed = [c for c in checks if not c.ok]
    print(f"\n{len(checks) - len(failed)} of {len(checks)} checks passed")


if __name__ == "__main__":
    main()
