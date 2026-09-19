"""karmel's safety case: the hazards, the controls, and the arithmetic that says they are enough.

A safety case is an argument: *these* are the ways the robot can hurt someone or itself, *these*
are the controls, and *this* is the evidence each control works. Writing it as code means the
argument can be checked — and that a control nobody ever tested shows up as a failure, not as a
reassuring sentence in a document.

    py 20-final-robot/code/safety_case.py hazards          # the hazard table
    py 20-final-robot/code/safety_case.py check            # the rules; exit 1 if the case is weak
    py 20-final-robot/code/safety_case.py stops            # every stop mechanism, and how far it goes
    py 20-final-robot/code/safety_case.py stops --speed 0.5
    py 20-final-robot/code/safety_case.py checklist        # the pre-autonomy test procedure
    py 20-final-robot/code/safety_case.py whatif no-estop  # the case without a hardware e-stop

Read SAFETY.md first; this file is that document with the numbers filled in. It is a *teaching*
model of a hazard analysis, not a certified one: no hobby robot needs ISO 10218, and no
spreadsheet makes an untested robot safe.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

# ==============================================================================================
#  Controls
# ==============================================================================================
#: Layers, from the one that survives the most to the one that survives the least.
LAYERS = ("hardware", "firmware", "onboard-software", "offboard-software", "procedure")


@dataclass(frozen=True)
class Control:
    """One thing that stops a hazard becoming an injury.

    ``layer`` decides what the control survives. A ``hardware`` control still works when every
    line of software is wrong; a ``procedure`` control works only while a human is paying
    attention. ``tested_by`` is the evidence — a control with no test is a wish.
    """

    name: str
    layer: str
    tested_by: str
    reaction_s: float | None = None

    @property
    def independent_of_software(self) -> bool:
        return self.layer == "hardware"


@dataclass(frozen=True)
class Hazard:
    """One way the robot can cause harm, with how bad and how often, before and after controls."""

    id: str
    description: str
    cause: str
    severity: int          # 1 nuisance · 2 minor damage · 3 injury or expensive damage · 4 severe
    likelihood: int        # 1 rare · 2 occasional · 3 likely · 4 near-certain, without controls
    controls: tuple[Control, ...]
    residual_likelihood: int = 1

    @property
    def inherent_risk(self) -> int:
        return self.severity * self.likelihood

    @property
    def residual_risk(self) -> int:
        return self.severity * self.residual_likelihood


# --- the controls karmel actually has -----------------------------------------------------------
ESTOP = Control("latching e-stop cuts the actuator relay", "hardware",
                "20.04-E3: press it mid-drive, confirm motor current = 0 on the INA226", 0.012)
FUSE = Control("inline pack fuse", "hardware", "measure the fuse rating against the branch table (20.02)")
BMS = Control("battery BMS: over-current, over-discharge, cell balance", "hardware",
              "load-test to the cutoff with a lab load; confirm it opens")
STAIR_GATE = Control("physical barrier: stair gate or closed door during autonomy", "hardware",
                     "walk the route before the session; the barrier is in place and cannot be pushed open")
WATCHDOG = Control("Pico watchdog stops motors after 300 ms of silence", "firmware",
                   "unplug the USB cable mid-drive; wheels stop within 300 ms", 0.300)
TORQUE_LIMIT = Control("arm servo torque and speed limits", "firmware",
                       "14.11-E2: command a target through an obstruction, confirm the limit trips")
CMD_TIMEOUT = Control("diff_drive_controller cmd_vel_timeout 0.5 s", "onboard-software",
                      "kill the controller_server; confirm the base halts", 0.500)
COLLISION_MONITOR = Control("Nav2 collision_monitor stop zone", "onboard-software",
                            "12.10-E4: drive at a box, confirm the stop distance", 0.150)
SPEED_LIMIT = Control("velocity and acceleration limits in ros2_control", "onboard-software",
                      "command 2 m/s, confirm the wheels do 0.5 m/s")
KEEPOUT = Control("keepout costmap filter (stairs, balcony)", "onboard-software",
                  "12.10-E4: send a goal inside the keepout, confirm it is rejected")
SKILL_ALLOWLIST = Control("skill allowlist + argument validation", "onboard-software",
                          "19.02-E1: send out-of-range arguments, confirm rejection")
GEOFENCE = Control("geofence on place names the agent may use", "onboard-software",
                   "19.09: ask for a forbidden room, confirm refusal")
HEALTH_GATE = Control("no mission starts unless the robot state is READY", "onboard-software",
                      "20.03-E2: unplug the LiDAR, confirm missions are refused")
ARM_ENVELOPE = Control("arm workspace box + parked while the base drives", "onboard-software",
                       "14.11-E2: command a pose outside the box, confirm refusal")
STAND = Control("wheels off the ground for first tests", "procedure", "SAFETY.md rule 1: the stand exists")
SUPERVISION = Control("a human within reach of the e-stop during autonomy", "procedure",
                      "the test protocol names who holds it")
CHARGE_RULES = Control("charge only supervised, on a non-flammable surface, in a LiPo bag",
                       "procedure", "SAFETY.md rule 5")


def karmel_hazards() -> list[Hazard]:
    """The hazards of the integrated robot: base, arm, battery, autonomy and the agent."""
    return [
        Hazard("H1", "The base drives into a person's ankle or a pet",
               "navigation continues while the obstacle is inside the footprint", 2, 3,
               (COLLISION_MONITOR, SPEED_LIMIT, ESTOP, WATCHDOG, SUPERVISION)),
        Hazard("H2", "The base runs away at full speed with no software control",
               "the Pi crashes or the USB link dies mid-drive", 3, 2,
               (WATCHDOG, ESTOP, CMD_TIMEOUT)),
        Hazard("H3", "The robot drives down the stairs",
               "localization drifts, or the LiDAR cannot see a drop-off", 4, 2,
               (STAIR_GATE, KEEPOUT, ESTOP, SUPERVISION), residual_likelihood=1),
        Hazard("H4", "The arm strikes a face or hand",
               "a planned trajectory through occupied space, or a learned policy acting oddly", 3, 3,
               (TORQUE_LIMIT, ARM_ENVELOPE, ESTOP, SUPERVISION)),
        Hazard("H5", "The arm swings while the base is driving and tips the robot",
               "a skill commands the arm during navigation", 2, 2,
               (ARM_ENVELOPE, SPEED_LIMIT, HEALTH_GATE, ESTOP)),
        Hazard("H6", "Battery fire or venting",
               "short circuit, over-discharge, a damaged cell, unattended charging", 4, 1,
               (FUSE, BMS, CHARGE_RULES)),
        Hazard("H7", "Fingers pinched by wheels or gripper",
               "hands near the robot while it is powered", 2, 3,
               (ESTOP, TORQUE_LIMIT, STAND, SUPERVISION)),
        Hazard("H8", "The agent asks for something the robot must not do",
               "hallucination, an ambiguous request, or prompt injection through a label it reads", 3, 3,
               (SKILL_ALLOWLIST, GEOFENCE, SPEED_LIMIT, COLLISION_MONITOR, ESTOP)),
        Hazard("H9", "The robot starts a mission with a dead sensor and navigates blind",
               "the LiDAR or the EKF dies after bringup", 3, 2,
               (HEALTH_GATE, COLLISION_MONITOR, ESTOP)),
        Hazard("H10", "The robot is left running unattended and flattens the pack in a corner",
               "a mission that never terminates", 1, 3,
               (HEALTH_GATE, SUPERVISION)),
    ]


# ==============================================================================================
#  The rules the case must satisfy
# ==============================================================================================
def check_case(hazards: list[Hazard]) -> list[str]:
    """Every rule a hobby-scale safety case should pass. Empty list = the argument holds."""
    problems: list[str] = []
    for h in hazards:
        independent = [c for c in h.controls if c.independent_of_software]
        software_only = all(c.layer.endswith("software") for c in h.controls)
        if h.severity >= 3 and not independent:
            problems.append(f"{h.id}: severity {h.severity} with no control that survives a software "
                            f"failure — every control is {', '.join(sorted({c.layer for c in h.controls}))}")
        if h.severity >= 4 and len(independent) < 2:
            problems.append(f"{h.id}: severity 4 needs two independent hardware controls, has "
                            f"{len(independent)} ({', '.join(c.name for c in independent) or 'none'})")
        if software_only and h.severity >= 2:
            problems.append(f"{h.id}: all controls are software; one bad deploy removes all of them")
        for c in h.controls:
            if not c.tested_by:
                problems.append(f"{h.id}: control '{c.name}' has no test — it is a wish, not a control")
        if h.residual_risk > 4:
            problems.append(f"{h.id}: residual risk {h.residual_risk} (severity {h.severity} x "
                            f"likelihood {h.residual_likelihood}) is above the accepted 4")
        if not h.controls:
            problems.append(f"{h.id}: no controls at all")
    return problems


# ==============================================================================================
#  The stop chain: how far the robot travels before each mechanism has stopped it
# ==============================================================================================
@dataclass(frozen=True)
class StopMechanism:
    """One way the robot can be stopped, and what it costs in distance.

    ``detect_s`` is from "the world became dangerous" to "this mechanism knows"; ``act_s`` is from
    knowing to the motors changing; ``decel`` is what happens then. The e-stop is the fastest to
    act and the *slowest* to stop the robot, because removing power means coasting — a detail that
    surprises everyone the first time they measure it.
    """

    name: str
    detect_s: float
    act_s: float
    decel_m_s2: float
    layer: str
    note: str = ""

    @property
    def dead_time_s(self) -> float:
        return self.detect_s + self.act_s

    def distance_m(self, v: float) -> float:
        """$d = v\\,t_\\text{dead} + \\dfrac{v^2}{2a}$ — constant speed during the dead time, then braking."""
        return v * self.dead_time_s + v * v / (2.0 * self.decel_m_s2)


#: Active braking (the firmware shorts the motor terminals) vs coasting (power removed).
BRAKE_DECEL = 1.5    # m/s^2, measured on karmel with the wheels on a hard floor (estimate)
COAST_DECEL = 0.6    # m/s^2, rolling + gearbox friction only (estimate)


def stop_mechanisms() -> list[StopMechanism]:
    return [
        StopMechanism("hardware e-stop (button already pressed)", 0.002, 0.012, COAST_DECEL, "hardware",
                      "relay opens; the robot then COASTS — no software involved"),
        StopMechanism("collision_monitor stop zone", 0.100, 0.095, BRAKE_DECEL, "onboard-software",
                      "10 Hz scan + 20 Hz monitor + one 50 Hz controller tick"),
        StopMechanism("Pico watchdog (link lost)", 0.300, 0.005, BRAKE_DECEL, "firmware",
                      "the only stop that survives the Pi crashing"),
        StopMechanism("diff_drive_controller cmd_vel timeout", 0.500, 0.020, BRAKE_DECEL, "onboard-software"),
        StopMechanism("human sees it and hits the button", 0.700, 0.014, COAST_DECEL, "procedure",
                      "0.7 s is an alert adult's reaction time; add 1 s if they are talking"),
        StopMechanism("agent decides to stop (LLM in the loop)", 2.000, 0.200, BRAKE_DECEL, "offboard-software",
                      "here for comparison only — see 19.01"),
    ]


#: karmel's front range sensor sits 0.125 m forward of base_link and the body radius is 0.160 m,
#: so a "stop at 0.40 m" rule leaves this much real margin (lesson 19.01).
STOP_MARGIN_M = 0.365


# ==============================================================================================
#  Scenarios
# ==============================================================================================
def whatif(name: str) -> tuple[str, list[Hazard]]:
    hazards = karmel_hazards()
    if name == "no-estop":
        stripped = [Hazard(h.id, h.description, h.cause, h.severity, h.likelihood,
                           tuple(c for c in h.controls if c is not ESTOP), h.residual_likelihood)
                    for h in hazards]
        return "No hardware e-stop fitted: software is the only thing that can stop the robot", stripped
    if name == "untested":
        stripped = [Hazard(h.id, h.description, h.cause, h.severity, h.likelihood,
                           tuple(Control(c.name, c.layer, "", c.reaction_s) if c.layer == "hardware" else c
                                 for c in h.controls), h.residual_likelihood)
                    for h in hazards]
        return "The hardware controls are fitted but nobody has ever tested them", stripped
    raise SystemExit(f"unknown scenario '{name}' (try: no-estop, untested)")


# ==============================================================================================
#  Output
# ==============================================================================================
def print_hazards(hazards: list[Hazard]) -> None:
    print(f"{'id':4} {'sev':>3} {'lik':>3} {'risk':>4} {'res':>4}  hazard / controls")
    for h in hazards:
        print(f"{h.id:4} {h.severity:>3} {h.likelihood:>3} {h.inherent_risk:>4} {h.residual_risk:>4}  "
              f"{h.description}")
        for c in h.controls:
            print(f"{'':21}  - [{c.layer:17}] {c.name}")


def print_stops(speed: float) -> None:
    print(f"at {speed:.2f} m/s, with {STOP_MARGIN_M:.3f} m of margin in front of the robot\n")
    print(f"{'mechanism':46} {'dead s':>7} {'decel':>6} {'stop m':>7}  verdict")
    for m in sorted(stop_mechanisms(), key=lambda m: m.distance_m(speed)):
        d = m.distance_m(speed)
        verdict = "fits" if d <= STOP_MARGIN_M else f"OVER by {d - STOP_MARGIN_M:.2f} m"
        print(f"{m.name:46} {m.dead_time_s:>7.3f} {m.decel_m_s2:>6.1f} {d:>7.3f}  {verdict}")
    print("\nThe robot is only as safe as the SLOWEST mechanism you are relying on for a given hazard.")


def print_checklist(hazards: list[Hazard]) -> None:
    seen: dict[str, Control] = {}
    for h in hazards:
        for c in h.controls:
            seen.setdefault(c.name, c)
    print("Pre-autonomy checklist — run this before every session in which the robot moves on its own.\n")
    for i, c in enumerate(sorted(seen.values(), key=lambda c: (LAYERS.index(c.layer), c.name)), 1):
        print(f"{i:2}. [{c.layer:17}] {c.name}")
        print(f"      evidence: {c.tested_by or 'NONE — do not run autonomously until this is tested'}")


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["hazards", "check", "stops", "checklist", "whatif"])
    ap.add_argument("scenario", nargs="?", default="no-estop")
    ap.add_argument("--speed", type=float, default=0.3, help="speed for the stop-distance table (m/s)")
    args = ap.parse_args(argv)

    hazards = karmel_hazards()
    if args.command == "whatif":
        title, hazards = whatif(args.scenario)
        print(f"{title}\n")

    if args.command == "hazards":
        print_hazards(hazards)
        return 0
    if args.command == "stops":
        print_stops(args.speed)
        return 0
    if args.command == "checklist":
        print_checklist(hazards)
        return 0

    problems = check_case(hazards)
    print(f"{len(hazards)} hazards, {sum(len(h.controls) for h in hazards)} controls")
    if not problems:
        print("the safety case holds")
        return 0
    print()
    for p in problems:
        print(f"WEAK  {p}")
    print(f"\n{len(problems)} weakness(es) — the robot does not run autonomously until these are closed.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
