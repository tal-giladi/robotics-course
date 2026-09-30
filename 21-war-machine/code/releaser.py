"""Releasers I — sizing the static release mechanisms (lesson 21.03).

Three questions, answered with arithmetic before any part is bolted on:

* **flap** — how much edge force (and servo torque) the drop-bin flap needs to release the
  heaviest payload in the rack, and whether an MG996R has margin for it;
* **pin** — how much force a solenoid pin has *left over* after the spring and friction,
  at the pack voltages you will actually see (the V^2 problem from 21.01 Level 2);
* **pulse** — how long to energise a solenoid: long enough to drive the pin fully out,
  short enough that the coil doesn't become a heater.

    py 21-war-machine/code/releaser.py flap
    py 21-war-machine/code/releaser.py pin
    py 21-war-machine/code/releaser.py pulse
    py 21-war-machine/code/releaser.py check

Every number is either (measured), (datasheet) or (estimate).
"""

from __future__ import annotations

import argparse
import sys

G = 9.81  # m/s^2

# --- the drop bin (21.03) ----------------------------------------------------------------------
FLAP_SPAN_M = 0.080            # pivot to flap edge, (estimate: print dimension)
PAYLOAD_MAX_KG = 0.15          # heaviest drop-class payload in the BOM rack
SAFETY_FACTOR = 3.0            # release force must beat holding force by 3x

MG996R_TORQUE_N_M = 0.98       # 10 kg*cm, (datasheet)
SG90_TORQUE_N_M = 0.18         # ~1.8 kg*cm, (datasheet) — the cheap micro servo


def flap_edge_force_n(payload_kg: float, safety: float = SAFETY_FACTOR) -> float:
    """Force at the flap edge needed to release a payload reliably: 3x its weight."""
    return payload_kg * G * safety


def flap_torque_n_m(edge_force_n: float, span_m: float = FLAP_SPAN_M) -> float:
    """Torque the flap pivot must resist: force at the edge x span (worst case, payload at edge)."""
    return edge_force_n * span_m


# --- the solenoid pin release --------------------------------------------------------------------
PIN_SPRING_HOLD_N = 5.0        # torsion spring holding force on the pin, (estimate)
PIN_FRICTION_N = 3.0           # pin-in-sleeve friction, (estimate)
PIN_STROKE_M = 0.015           # pin withdrawal travel, (estimate: print dimension)
PIN_VELOCITY_M_S = 0.20        # mean pin velocity while powered, (estimate)
PIN_PULSE_MIN_MS = 100.0
PIN_PULSE_MAX_MS = 300.0
PIN_PULSE_MARGIN_MS = 50.0

SOL_RATED_N = {"42n": 42.0, "85n": 85.0}   # (datasheet, Hackstore 35 mm 12 V family)
RATED_V = 12.0


def solenoid_force_n(rated_n: float, v: float, rated_v: float = RATED_V) -> float:
    """The V^2 law from 21.01 Level 2: magnetic force scales with the square of the voltage."""
    return rated_n * (v / rated_v) ** 2


def pin_net_force_n(rated_n: float, v: float, rated_v: float = RATED_V) -> float:
    """What the pin actually has left after the spring and friction oppose it."""
    return solenoid_force_n(rated_n, v, rated_v) - PIN_SPRING_HOLD_N - PIN_FRICTION_N


def solenoid_pulse_ms(
    stroke_m: float = PIN_STROKE_M,
    velocity_m_s: float = PIN_VELOCITY_M_S,
    margin_ms: float = PIN_PULSE_MARGIN_MS,
    min_ms: float = PIN_PULSE_MIN_MS,
    max_ms: float = PIN_PULSE_MAX_MS,
) -> float:
    """Energise just long enough to drive the pin out: travel time + margin, clamped to
    100..300 ms. A held solenoid is a heater (42 N class: 3-5 W), so the pulse is the design."""
    travel_ms = stroke_m / velocity_m_s * 1000.0
    return min(max(travel_ms + margin_ms, min_ms), max_ms)


# --- CLI ------------------------------------------------------------------------------------------
def _print_flap() -> None:
    need_f = flap_edge_force_n(PAYLOAD_MAX_KG)
    need_t = flap_torque_n_m(need_f)
    print(f"drop-bin flap sizing (span {FLAP_SPAN_M * 1000:.0f} mm)")
    print(f"  heaviest payload          {PAYLOAD_MAX_KG:.2f} kg  ({PAYLOAD_MAX_KG * G:.2f} N weight)")
    print(f"  required edge force (x{SAFETY_FACTOR:.0f})      {need_f:.2f} N")
    print(f"  required pivot torque            {need_t:.3f} N*m")
    print(f"  MG996R available torque          {MG996R_TORQUE_N_M:.3f} N*m  -> margin {MG996R_TORQUE_N_M / need_t:.2f}x")
    print(f"  SG90  available torque           {SG90_TORQUE_N_M:.3f} N*m  -> margin {SG90_TORQUE_N_M / need_t:.2f}x"
          f"  {'OK' if SG90_TORQUE_N_M > need_t else 'TOO WEAK'}")
    print()
    print("verdict: the MG996R has ~2.8x margin; the SG90 cannot release the heaviest payload.")


def _print_pin() -> None:
    need = PIN_SPRING_HOLD_N + PIN_FRICTION_N
    print(f"solenoid pin sizing (spring {PIN_SPRING_HOLD_N:.0f} N + friction {PIN_FRICTION_N:.0f} N = {need:.0f} N required)")
    print(f"  {'solenoid':<8} {'@12.6 V':>8} {'@12.0 V':>8} {'@10.8 V':>8} {'@9.9 V':>8} {'@9.0 V':>8}")
    for tag, rated in SOL_RATED_N.items():
        row = [f"{pin_net_force_n(rated, v):8.1f} N" for v in (12.6, 12.0, 10.8, 9.9, 9.0)]
        print(f"  {tag:<8}" + "".join(row))
    print()
    print(f"  force at 9.9 V is {(9.9 / 12.0) ** 2 * 100:.0f}% of rated (V^2 law) - the empty-pack case is the design case.")
    print("  both solenoids clear the required 8 N at every pack voltage; the 85 N buys margin")
    print("  for a corroded or un-lubricated pin. Pulse width: see `pulse`.")


def _print_pulse() -> None:
    for tag, rated in SOL_RATED_N.items():
        pulse = solenoid_pulse_ms()
        # rough heating: 42 N class draws ~4 W at 12 V while energised (estimate)
        watts = 4.0 if tag == "42n" else 7.0
        joules = watts * pulse / 1000.0
        print(f"{tag:<6} pulse {pulse:5.0f} ms  (travel {PIN_STROKE_M * 1000:.0f} mm at {PIN_VELOCITY_M_S * 1000:.0f} mm/s + {PIN_PULSE_MARGIN_MS:.0f} ms margin)")
        print(f"       heat per pulse ~ {joules:.2f} J  — negligible at 1 pulse / 3 s duty")
    print()
    print(f"clamped to {PIN_PULSE_MIN_MS:.0f}..{PIN_PULSE_MAX_MS:.0f} ms: shorter can't finish the stroke,")
    print(f"longer just heats the coil. Add a flyback diode across the coil (21.03 wiring).")


def _check() -> list[str]:
    problems = []
    need_t = flap_torque_n_m(flap_edge_force_n(PAYLOAD_MAX_KG))
    if MG996R_TORQUE_N_M < 2 * need_t:
        problems.append(f"MG996R margin {MG996R_TORQUE_N_M / need_t:.2f}x < 2x on the flap")
    for v in (9.9, 9.0):
        for tag, rated in SOL_RATED_N.items():
            if pin_net_force_n(rated, v) <= 0:
                problems.append(f"{tag} pin has no net force at {v} V")
    pulse = solenoid_pulse_ms()
    if not (PIN_PULSE_MIN_MS <= pulse <= PIN_PULSE_MAX_MS):
        problems.append(f"pulse {pulse:.0f} ms outside the clamped range")
    return problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", choices=("flap", "pin", "pulse", "check"))
    args = ap.parse_args(argv)
    if args.command == "flap":
        _print_flap()
    elif args.command == "pin":
        _print_pin()
    elif args.command == "pulse":
        _print_pulse()
    elif args.command == "check":
        problems = _check()
        if problems:
            print(f"check: {len(problems)} problem(s)")
            for p in problems:
                print(f"  PROBLEM  {p}")
            return 1
        print("check: all rules pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
