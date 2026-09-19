"""12.08 — Nav2 tuning arithmetic for a small, slow robot: compute before you turn a knob.

    python 12-navigation/code/tuning.py                       # karmel's numbers, and what to change
    python 12-navigation/code/tuning.py --door 0.72 --scan-hz 5.5 --amcl-std 0.06

Every function here answers one tuning question with a formula and karmel's real parameters from
``labs/ros2_ws/src/karmel_bringup/config/nav2_params.yaml``. The ``recommend`` function turns
measurements you take **on the real robot** (scan rate, controller rate, AMCL jitter, TF age,
narrowest doorway) into a list of specific parameter changes with the reasoning attached.

Nothing here talks to ROS: it is the calculator you keep open in the other window while the robot
drives.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Any

import nav2_config

MAX_NON_OBSTACLE = 252
INSCRIBED_INFLATED_OBSTACLE = 253


# --- geometry -------------------------------------------------------------------------------------
def inflation_cost(distance_m: float, inscribed_radius_m: float, cost_scaling_factor: float) -> int:
    """Nav2's InflationLayer::computeCost, as derived in 12.04."""
    if distance_m == 0:
        return 254
    if distance_m <= inscribed_radius_m:
        return INSCRIBED_INFLATED_OBSTACLE
    return int(MAX_NON_OBSTACLE * math.exp(-cost_scaling_factor * (distance_m - inscribed_radius_m)))


@dataclass(frozen=True)
class Doorway:
    width_m: float
    inscribed_radius_m: float
    cost_scaling_factor: float

    @property
    def half_gap_m(self) -> float:
        """Distance from the centre line of the door to the nearest wall."""
        return self.width_m / 2.0

    @property
    def clearance_m(self) -> float:
        """Free space on each side when the robot drives down the middle."""
        return self.half_gap_m - self.inscribed_radius_m

    @property
    def geometrically_passable(self) -> bool:
        """A cell is lethal-inflated (253) inside the inscribed radius, and planners refuse those."""
        return self.clearance_m > 0.0

    @property
    def centre_cost(self) -> int:
        return inflation_cost(self.half_gap_m, self.inscribed_radius_m, self.cost_scaling_factor)

    def __str__(self) -> str:
        verdict = "passable" if self.geometrically_passable else "BLOCKED"
        return (f"door {self.width_m * 100:.0f} cm: half-gap {self.half_gap_m * 100:.1f} cm, "
                f"clearance {self.clearance_m * 100:+.1f} cm, cost at the centre {self.centre_cost} "
                f"-> {verdict}")


def corner_cut_m(lookahead_m: float, angle_rad: float = math.pi / 2) -> float:
    """How far inside a corner of ``angle_rad`` a pure-pursuit arc passes (12.05).

    For a 90 degree corner this is the familiar ``L (1 - 1/sqrt(2)) ~ 0.293 L``.
    """
    return lookahead_m * (1.0 - math.cos(angle_rad / 2.0))


# --- timing ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class StopBudget:
    """Where the distance between "the obstacle appears" and "the robot is still" goes."""

    speed_m_s: float
    decel_m_s2: float
    sensor_period_s: float
    costmap_period_s: float
    controller_period_s: float

    @property
    def latency_s(self) -> float:
        return self.sensor_period_s + self.costmap_period_s + self.controller_period_s

    @property
    def blind_m(self) -> float:
        return self.speed_m_s * self.latency_s

    @property
    def braking_m(self) -> float:
        return self.speed_m_s ** 2 / (2.0 * self.decel_m_s2)

    @property
    def total_m(self) -> float:
        return self.blind_m + self.braking_m

    def __str__(self) -> str:
        return (f"{self.speed_m_s:.2f} m/s: blind {self.blind_m * 100:5.1f} cm "
                f"({self.latency_s * 1000:.0f} ms of latency) + braking {self.braking_m * 100:4.1f} cm "
                f"= {self.total_m * 100:5.1f} cm")


def min_local_costmap_width_m(budget: StopBudget, margin_m: float = 0.3) -> float:
    """The rolling window must show the obstacle before it is inside the stopping distance."""
    return 2.0 * (budget.total_m + margin_m)


def replan_period_for_distance_s(speed_m_s: float, distance_m: float = 0.25) -> float:
    """``RateController hz`` follows from how far you are willing to drive on a stale path."""
    return distance_m / speed_m_s


# --- localization ---------------------------------------------------------------------------------
def goal_tolerance_m(amcl_std_m: float, overshoot_m: float, margin_m: float = 0.03) -> float:
    """A tolerance below localization noise + overshoot is never reliably satisfied (12.05-E6)."""
    return 2.0 * amcl_std_m + overshoot_m + margin_m


def amcl_update_distance_m(speed_m_s: float, scan_hz: float, scans_per_update: int = 2) -> float:
    """``update_min_d``: how far to drive between filter updates, given the scan rate."""
    return speed_m_s * scans_per_update / scan_hz


def transform_tolerance_s(publish_hz: float, safety: float = 3.0) -> float:
    """TF lookups fail if the newest transform is older than this. Cover a few missed updates."""
    return safety / publish_hz


# --- turning measurements into parameter changes ---------------------------------------------------
@dataclass
class Measurements:
    """What you measure on the real robot before touching a parameter (see the lesson)."""

    scan_hz: float = 10.0
    controller_hz: float = 20.0
    amcl_std_m: float = 0.02
    tf_age_s: float = 0.05
    narrowest_door_m: float = 0.80
    measured_max_speed_m_s: float = 0.25
    cpu_load: float = 0.5


@dataclass(frozen=True)
class Recommendation:
    parameter: str
    current: Any
    suggested: Any
    reason: str

    def __str__(self) -> str:
        return f"  {self.parameter:<52} {self.current!s:>8} -> {self.suggested!s:<8} {self.reason}"


def recommend(measured: Measurements, params: dict[str, Any] | None = None) -> list[Recommendation]:
    """Compare karmel's shipped parameters against what the real robot actually does."""
    params = params or nav2_config.load_params()
    out: list[Recommendation] = []

    follow = params["controller_server"]["FollowPath"]
    goal_checker = params["controller_server"]["general_goal_checker"]
    local = params["local_costmap"]
    inflation = local["inflation_layer"]
    smoother = params["velocity_smoother"]
    inscribed = nav2_config.footprint_radii(nav2_config.footprint_of(params))[0] + local["footprint_padding"]

    budget = StopBudget(speed_m_s=follow["desired_linear_vel"], decel_m_s2=abs(smoother["max_decel"][0]),
                        sensor_period_s=1.0 / measured.scan_hz,
                        costmap_period_s=1.0 / local["update_frequency"],
                        controller_period_s=1.0 / measured.controller_hz)

    needed_width = min_local_costmap_width_m(budget)
    if needed_width > local["width"]:
        out.append(Recommendation("local_costmap.width / .height", local["width"], round(needed_width + 0.5),
                                  f"stopping distance {budget.total_m * 100:.0f} cm + margin"))

    tolerance = goal_tolerance_m(measured.amcl_std_m, overshoot_m=budget.blind_m)
    if tolerance > 1.1 * goal_checker["xy_goal_tolerance"]:
        out.append(Recommendation("controller_server.general_goal_checker.xy_goal_tolerance",
                                  goal_checker["xy_goal_tolerance"], round(tolerance, 2),
                                  f"2 sigma AMCL ({measured.amcl_std_m * 100:.0f} cm) + overshoot"))

    door = Doorway(measured.narrowest_door_m, inscribed, inflation["cost_scaling_factor"])
    if not door.geometrically_passable:
        out.append(Recommendation("footprint / footprint_padding", f"{inscribed:.3f}",
                                  f"<{door.half_gap_m:.3f}",
                                  f"the {measured.narrowest_door_m * 100:.0f} cm door is narrower than the inflated robot"))
    elif door.centre_cost > 200:
        out.append(Recommendation("costmap inflation_layer.cost_scaling_factor",
                                  inflation["cost_scaling_factor"],
                                  round(-math.log(200 / MAX_NON_OBSTACLE) / door.clearance_m, 1),
                                  f"cost {door.centre_cost} in the middle of the narrowest door"))

    tf_tol = transform_tolerance_s(min(measured.scan_hz, 10.0))
    if tf_tol > follow["transform_tolerance"]:
        out.append(Recommendation("FollowPath.transform_tolerance", follow["transform_tolerance"],
                                  round(tf_tol, 2), f"3 periods of the slowest input ({measured.scan_hz:.1f} Hz)"))

    update_d = params["amcl"]["update_min_d"]
    if update_d > goal_checker["xy_goal_tolerance"]:
        out.append(Recommendation("amcl.update_min_d", update_d,
                                  round(amcl_update_distance_m(follow["desired_linear_vel"], measured.scan_hz), 2),
                                  "the robot drives further between pose fixes than the goal tolerance"))

    if measured.controller_hz < 0.8 * params["controller_server"]["controller_frequency"]:
        out.append(Recommendation("controller_server.controller_frequency",
                                  params["controller_server"]["controller_frequency"],
                                  round(measured.controller_hz), "the loop is already missing its rate"))
    if measured.cpu_load > 0.8:
        out.append(Recommendation("amcl.max_particles", params["amcl"]["max_particles"],
                                  params["amcl"]["max_particles"] // 2,
                                  f"CPU load {measured.cpu_load:.0%}: halve the cheapest thing first"))
        out.append(Recommendation("amcl.update_min_d", update_d, round(update_d * 2, 2),
                                  "fewer filter updates per metre buys back CPU"))

    if measured.measured_max_speed_m_s < 0.9 * follow["desired_linear_vel"]:
        out.append(Recommendation("FollowPath.desired_linear_vel", follow["desired_linear_vel"],
                                  round(measured.measured_max_speed_m_s, 2),
                                  "the robot cannot reach the commanded cruise speed"))

    cut = corner_cut_m(follow["lookahead_dist"])
    if cut > door.clearance_m:
        out.append(Recommendation("FollowPath.lookahead_dist (or use_rotate_to_heading)",
                                  follow["lookahead_dist"], round(door.clearance_m / 0.293, 2),
                                  f"a 90 deg corner cuts {cut * 100:.0f} cm, clearance is only "
                                  f"{door.clearance_m * 100:.0f} cm"))
    return out


# --- the sim/real difference table ------------------------------------------------------------------
SIM_VS_REAL: list[tuple[str, str, str]] = [
    ("Clock", "/clock from Gazebo; every node needs use_sim_time:=true", "system clock; use_sim_time must be false everywhere"),
    ("LiDAR", "perfect ranges, exact 10 Hz, no dropouts", "noise, reflections off glass, missing returns, 8-12 Hz jitter"),
    ("Wheel odometry", "exact integration of the commanded twist", "slip on rugs, wheel radius error, caster drag (09.05)"),
    ("Obstacles", "in the world file or not at all", "cables, socks and chair legs below the 0.165 m scan plane"),
    ("CPU", "as much as your laptop has", "a Raspberry Pi 5 also running SLAM/AMCL and the LiDAR driver"),
    ("Failure", "the simulation is reset", "the robot hits your wall; wheels off the ground first"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Nav2 tuning arithmetic for karmel.")
    parser.add_argument("--door", type=float, default=0.80, help="narrowest doorway, metres")
    parser.add_argument("--scan-hz", type=float, default=10.0)
    parser.add_argument("--controller-hz", type=float, default=20.0)
    parser.add_argument("--amcl-std", type=float, default=0.02, help="AMCL pose std while standing still")
    parser.add_argument("--cpu", type=float, default=0.5, help="CPU load, 0..1")
    parser.add_argument("--max-speed", type=float, default=0.25)
    args = parser.parse_args()

    params = nav2_config.load_params()
    inscribed = (nav2_config.footprint_radii(nav2_config.footprint_of(params))[0]
                 + params["local_costmap"]["footprint_padding"])
    k = params["local_costmap"]["inflation_layer"]["cost_scaling_factor"]
    follow = params["controller_server"]["FollowPath"]

    print(f"karmel: inflated inscribed radius {inscribed * 100:.1f} cm, cost_scaling_factor {k}, "
          f"inflation_radius {params['local_costmap']['inflation_layer']['inflation_radius']} m\n")

    print("doorways")
    for width in (0.60, 0.70, 0.80, 0.90):
        print("   ", Doorway(width, inscribed, k))

    print("\nstopping distance (sensor 10 Hz, local costmap 5 Hz, controller 20 Hz)")
    for speed in (0.15, 0.25, 0.30, 0.50):
        budget = StopBudget(speed, abs(params["velocity_smoother"]["max_decel"][0]), 0.1, 0.2, 0.05)
        print(f"    {budget}   -> local costmap >= {min_local_costmap_width_m(budget):.1f} m")

    print("\npure pursuit corner cut")
    for lookahead in (0.25, 0.40, 0.80):
        print(f"    lookahead {lookahead:.2f} m -> {corner_cut_m(lookahead) * 100:.1f} cm inside a 90 deg corner")

    print("\nreplanning: RateController hz for a 25 cm stale-path budget")
    for speed in (0.15, 0.25, 0.50):
        print(f"    {speed:.2f} m/s -> every {replan_period_for_distance_s(speed):.2f} s "
              f"= {1 / replan_period_for_distance_s(speed):.1f} Hz")

    print("\nsimulation vs the real robot")
    for what, sim, real in SIM_VS_REAL:
        print(f"    {what:<16} sim: {sim}")
        print(f"    {'':<16} real: {real}")

    measured = Measurements(scan_hz=args.scan_hz, controller_hz=args.controller_hz,
                            amcl_std_m=args.amcl_std, narrowest_door_m=args.door,
                            measured_max_speed_m_s=args.max_speed, cpu_load=args.cpu)
    print(f"\nrecommendations for your measurements {measured}")
    changes = recommend(measured, params)
    if not changes:
        print("  none: the shipped parameters already fit what you measured")
    for change in changes:
        print(change)
    print(f"\n  lookahead {follow['lookahead_dist']} m, cruise {follow['desired_linear_vel']} m/s, "
          f"goal tolerance {params['controller_server']['general_goal_checker']['xy_goal_tolerance']} m")


if __name__ == "__main__":
    main()
