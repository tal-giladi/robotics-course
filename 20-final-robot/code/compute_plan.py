"""Where does each job of the final robot run, and does the computer keep up?

A placement plan is only an opinion until you put numbers next to it. This script holds the final
robot's jobs (CPU, RAM, deadline, whether they need CUDA or bare-metal timing), the computers
available, and a feasibility check that fails when a plan does not fit.

    py 20-final-robot/code/compute_plan.py plan            # the course plan, with headroom per host
    py 20-final-robot/code/compute_plan.py rules           # the placement rules applied to each job
    py 20-final-robot/code/compute_plan.py whatif no-jetson    # detection moved onto the Pi 5
    py 20-final-robot/code/compute_plan.py whatif no-laptop    # everything on the robot

The CPU figures are REFERENCE values for a Raspberry Pi 5 (4x Cortex-A76 @ 2.4 GHz) running the
course stack; they are the right order of magnitude, not your robot's measurement. Lesson 20.03
shows how to measure yours with ``top -H``, ``ros2 topic hz`` and the diagnostics tree, and this
file is where you write the result down.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, replace

PI, PICO, JETSON, LAPTOP, CLOUD = "pi5", "pico", "jetson", "laptop", "cloud"


@dataclass(frozen=True)
class Host:
    name: str
    cores: float              # usable cores (one is reserved for the kernel and DDS on the Pi)
    ram_mb: float
    has_cuda: bool = False
    bare_metal: bool = False  # no OS: deterministic sub-millisecond timing
    on_robot: bool = True
    watts: float = 0.0


HOSTS: dict[str, Host] = {
    PICO: Host(PICO, 1.0, 0.5, bare_metal=True, watts=0.1),
    PI: Host(PI, 3.2, 8000, watts=9.0),            # 4 cores, 0.8 reserved for kernel + DDS + ssh
    JETSON: Host(JETSON, 5.0, 8000, has_cuda=True, watts=20.0),
    LAPTOP: Host(LAPTOP, 8.0, 32000, has_cuda=True, on_robot=False),
    CLOUD: Host(CLOUD, 1e6, 1e6, has_cuda=True, on_robot=False),
}


@dataclass(frozen=True)
class Job:
    name: str
    cores: float                 # fraction of one core, sustained
    ram_mb: float
    deadline_s: float            # how late this job may be before the robot misbehaves
    host: str
    needs_cuda: bool = False
    safety_critical: bool = False   # must keep working when everything else has crashed
    note: str = ""


def course_plan() -> list[Job]:
    """karmel, final configuration: base + LiDAR + camera + arm + Nav2 + agent."""
    return [
        Job("motor PWM + PIO encoders + wheel PID", 0.30, 0.05, 0.002, PICO,
            safety_critical=True, note="hard deadline, no OS"),
        Job("command watchdog + e-stop sensing", 0.05, 0.01, 0.300, PICO,
            safety_critical=True, note="must survive a Pi crash"),
        Job("karmel_hardware (serial) + ros2_control 50 Hz", 0.25, 120, 0.020, PI,
            safety_critical=True, note="owns the only writer to the base"),
        Job("sllidar_ros2 driver", 0.10, 60, 0.100, PI),
        Job("bno055 IMU driver", 0.05, 40, 0.020, PI),
        Job("v4l2_camera + JPEG", 0.35, 180, 0.067, PI, note="VGA @15 fps, on-chip encoder unused"),
        Job("robot_localization EKF 30 Hz", 0.20, 90, 0.033, PI),
        Job("AMCL (500 particles)", 0.30, 150, 0.100, PI),
        Job("Nav2 costmaps + controller + BT", 0.90, 400, 0.050, PI),
        Job("collision_monitor", 0.10, 50, 0.050, PI, safety_critical=True),
        Job("health_monitor + diagnostics", 0.05, 40, 1.000, PI),
        Job("rosbag2 recording (MCAP)", 0.20, 120, 1.000, PI, note="only during tests"),
        Job("arm skill server + MoveIt Servo", 0.45, 350, 0.020, PI),
        Job("object detection (YOLO-class, 15 fps)", 2.20, 1500, 0.067, JETSON, needs_cuda=True),
        Job("learned pick policy (ACT/SmolVLA, 30 Hz)", 1.80, 2500, 0.033, JETSON, needs_cuda=True),
        Job("RViz + Gazebo + training", 4.00, 6000, 1.000, LAPTOP),
        Job("LLM planner", 0.10, 100, 5.000, CLOUD, note="seconds; never in a control loop"),
    ]


# --------------------------------------------------------------------------------------------
#  The placement rules (the same table as the lesson, executable)
# --------------------------------------------------------------------------------------------
def required_host(job: Job) -> str:
    """The rule that decides where a job belongs, in priority order."""
    if job.deadline_s <= 0.005 or (job.safety_critical and job.deadline_s <= 0.3 and job.ram_mb < 1):
        return PICO
    if job.needs_cuda:
        return JETSON
    if job.safety_critical or job.deadline_s <= 0.2:
        return PI
    if job.deadline_s >= 2.0:
        return CLOUD
    if job.ram_mb > 4000 or job.cores > 3.0:
        return LAPTOP
    return PI


@dataclass
class HostLoad:
    host: Host
    cores: float = 0.0
    ram_mb: float = 0.0
    jobs: int = 0

    @property
    def core_headroom(self) -> float:
        return self.host.cores - self.cores

    @property
    def ok(self) -> bool:
        return self.core_headroom >= 0 and self.ram_mb <= self.host.ram_mb


def load_by_host(plan: list[Job]) -> dict[str, HostLoad]:
    loads = {name: HostLoad(host) for name, host in HOSTS.items()}
    for job in plan:
        load = loads[job.host]
        load.cores += job.cores
        load.ram_mb += job.ram_mb
        load.jobs += 1
    return loads


def problems(plan: list[Job]) -> list[str]:
    """Everything wrong with a placement plan."""
    out: list[str] = []
    for name, load in load_by_host(plan).items():
        if load.jobs and load.core_headroom < 0:
            out.append(f"{name}: {load.cores:.2f} cores of work on {load.host.cores:.1f} usable "
                       f"({-load.core_headroom:.2f} cores over) — deadlines will be missed under load")
        if load.jobs and load.ram_mb > load.host.ram_mb:
            out.append(f"{name}: {load.ram_mb:.0f} MB on a {load.host.ram_mb:.0f} MB host")
    for job in plan:
        host = HOSTS[job.host]
        if job.needs_cuda and not host.has_cuda:
            out.append(f"{job.name}: needs CUDA, {job.host} has none")
        if job.safety_critical and not host.on_robot:
            out.append(f"{job.name}: safety-critical but runs off the robot ({job.host})")
        if job.deadline_s <= 0.005 and not host.bare_metal:
            out.append(f"{job.name}: {job.deadline_s * 1000:.0f} ms deadline on {job.host}, "
                       "which runs a non-real-time kernel")
    return out


def whatif(name: str) -> tuple[str, list[Job]]:
    """Variants of the plan that a student really has to consider."""
    plan = course_plan()
    if name == "no-jetson":
        moved = [replace(j, host=PI, cores=j.cores * 4.0, needs_cuda=False)
                 if j.host == JETSON else j for j in plan]
        return ("No Jetson: detection and the learned policy run on the Pi 5's CPU "
                "(roughly 4x the CPU time and no GPU)"), moved
    if name == "no-laptop":
        return "No laptop: everything on the robot", [replace(j, host=PI) if j.host == LAPTOP else j
                                                      for j in plan]
    if name == "cloud-control":
        return "The LLM sends velocity commands directly", [
            replace(j, deadline_s=0.1, safety_critical=True) if j.host == CLOUD else j for j in plan]
    raise SystemExit(f"unknown scenario '{name}' (try: no-jetson, no-laptop, cloud-control)")


# --------------------------------------------------------------------------------------------
def print_plan(plan: list[Job]) -> None:
    print(f"{'job':<44} {'host':<8} {'cores':>6} {'RAM MB':>7} {'deadline':>9}")
    for job in sorted(plan, key=lambda j: (j.host, -j.cores)):
        print(f"{job.name:<44} {job.host:<8} {job.cores:>6.2f} {job.ram_mb:>7.0f} "
              f"{job.deadline_s * 1000:>7.0f} ms")
    print()
    print(f"{'host':<8} {'cores used':>11} {'of':>6} {'headroom':>9} {'RAM MB':>8} {'jobs':>5}")
    for name, load in load_by_host(plan).items():
        if not load.jobs:
            continue
        budget = "inf" if load.host.cores > 1e5 else f"{load.host.cores:.1f}"
        print(f"{name:<8} {load.cores:>11.2f} {budget:>6} {load.core_headroom:>9.2f} "
              f"{load.ram_mb:>8.0f} {load.jobs:>5}")


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["plan", "rules", "whatif"])
    ap.add_argument("scenario", nargs="?", default="no-jetson")
    args = ap.parse_args(argv)

    if args.command == "rules":
        print(f"{'job':<44} {'planned':<8} {'rule says':<8}")
        for job in course_plan():
            flag = "" if required_host(job) == job.host else "   <-- disagrees"
            print(f"{job.name:<44} {job.host:<8} {required_host(job):<8}{flag}")
        return 0

    plan = course_plan()
    if args.command == "whatif":
        title, plan = whatif(args.scenario)
        print(f"{title}\n")
    print_plan(plan)
    found = problems(plan)
    print()
    if not found:
        print("plan is feasible")
        return 0
    for p in found:
        print(f"PROBLEM  {p}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
