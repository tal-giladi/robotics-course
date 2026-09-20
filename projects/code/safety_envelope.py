r"""How fast may the robot drive, given how late it finds out and how hard it can brake?

    python projects/code/safety_envelope.py --reaction 0.16 --decel 1.0 --clearance 0.35
    python projects/code/safety_envelope.py --speeds 0.10,0.20,0.30,0.50 --reaction 0.16

Every project from P01 onward runs into the same arithmetic. The robot notices something late
(network, loop period, sensor period, motor lag) and then needs distance to stop:

    stopping distance = speed * reaction_time  +  speed^2 / (2 * decel)
                        \_ blind distance _/     \_ braking distance _/

The first term is linear in speed and the second is quadratic, which is why "it was fine at
0.2 m/s" says nothing about 0.4 m/s: doubling the speed doubles the blind distance and
*quadruples* the braking distance.

Three questions, three functions:

* ``stopping_distance_m`` - I drive at *v*; how much clear space do I need?
* ``max_safe_speed_m_s``  - I have *d* of clear space; how fast may I drive?
* ``decel_from_coast_m_s2`` - measure the deceleration instead of guessing it: drive at a known
  speed, command stop, tape-measure how far it coasted.

Nothing here knows about hardware. Feed it numbers you measured
(``01-first-robot/code/link_latency.py`` gives you the latency part) rather than numbers you hoped
for; the defaults are karmel's, from ``labs/config/karmel.yaml``.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

# karmel defaults, from labs/config/karmel.yaml and the measurements in 01.15.
DEFAULT_DECEL_M_S2 = 1.0      # drive.max_linear_accel_m_s2 - braking is commanded, not coasting
DEFAULT_LOOP_HZ = 20.0        # the teleop / reflex loop rate of 01.13 and 01.15
DEFAULT_MOTOR_TAU_S = 0.08    # drive.motor_time_constant_s


@dataclass(frozen=True)
class Envelope:
    """One row of the speed table: what this speed costs you in centimetres."""

    speed_m_s: float
    reaction_s: float
    decel_m_s2: float
    reaction_distance_m: float
    braking_distance_m: float
    stopping_distance_m: float

    def fits_in(self, clearance_m: float) -> bool:
        """Does the robot stop inside the space it has?"""
        return self.stopping_distance_m <= clearance_m


def _check(speed_m_s: float, reaction_s: float, decel_m_s2: float) -> None:
    if speed_m_s < 0.0:
        raise ValueError("speed must be >= 0")
    if reaction_s < 0.0:
        raise ValueError("reaction time must be >= 0")
    if decel_m_s2 <= 0.0:
        raise ValueError("deceleration must be > 0 (a robot that cannot brake has no safe speed)")


def reaction_time_s(link_latency_s: float, loop_hz: float = DEFAULT_LOOP_HZ,
                    motor_tau_s: float = DEFAULT_MOTOR_TAU_S, extra_s: float = 0.0) -> float:
    """Add up the delays between "the world changed" and "the wheels changed".

    ``link_latency_s`` is the round trip you measured (use the **p99**, not the mean - the mean
    is the latency you never notice). One loop period is the worst case for a discrete loop that
    samples at ``loop_hz``. ``motor_tau_s`` is how long the motor takes to respond. ``extra_s``
    is anything else you know about: a blocking ultrasonic read, a sensor's own period, you.
    """
    if loop_hz <= 0.0:
        raise ValueError("loop rate must be > 0")
    if link_latency_s < 0.0 or motor_tau_s < 0.0 or extra_s < 0.0:
        raise ValueError("delays must be >= 0")
    return link_latency_s + 1.0 / loop_hz + motor_tau_s + extra_s


def braking_distance_m(speed_m_s: float, decel_m_s2: float) -> float:
    """``v^2 / (2a)`` - the distance used while actually slowing down."""
    _check(speed_m_s, 0.0, decel_m_s2)
    return speed_m_s * speed_m_s / (2.0 * decel_m_s2)


def stopping_distance_m(speed_m_s: float, reaction_s: float,
                        decel_m_s2: float = DEFAULT_DECEL_M_S2) -> float:
    """Blind distance plus braking distance: the clear space this speed requires."""
    _check(speed_m_s, reaction_s, decel_m_s2)
    return speed_m_s * reaction_s + braking_distance_m(speed_m_s, decel_m_s2)


def max_safe_speed_m_s(clearance_m: float, reaction_s: float,
                       decel_m_s2: float = DEFAULT_DECEL_M_S2) -> float:
    """Invert ``stopping_distance_m``: the fastest speed that still stops within ``clearance_m``.

    Solving ``v^2 / (2a) + r v - d = 0`` for the positive root gives
    ``v = -a r + sqrt((a r)^2 + 2 a d)``.
    """
    _check(0.0, reaction_s, decel_m_s2)
    if clearance_m < 0.0:
        raise ValueError("clearance must be >= 0")
    ar = decel_m_s2 * reaction_s
    return -ar + math.sqrt(ar * ar + 2.0 * decel_m_s2 * clearance_m)


def decel_from_coast_m_s2(speed_m_s: float, coast_distance_m: float) -> float:
    """Measure the deceleration: drive at ``speed_m_s``, command stop, measure how far it went.

    This is the honest way to get the number - a braked gearmotor on a tiled floor and the same
    motor on a rug are not the same robot. Measure it *after* the reaction time has been
    accounted for, i.e. from the moment the wheels visibly start slowing.
    """
    if speed_m_s <= 0.0:
        raise ValueError("speed must be > 0")
    if coast_distance_m <= 0.0:
        raise ValueError("coast distance must be > 0")
    return speed_m_s * speed_m_s / (2.0 * coast_distance_m)


def envelope(speed_m_s: float, reaction_s: float,
             decel_m_s2: float = DEFAULT_DECEL_M_S2) -> Envelope:
    """Everything about one speed, in one object."""
    reaction_distance = speed_m_s * reaction_s
    braking = braking_distance_m(speed_m_s, decel_m_s2)
    return Envelope(
        speed_m_s=speed_m_s,
        reaction_s=reaction_s,
        decel_m_s2=decel_m_s2,
        reaction_distance_m=reaction_distance,
        braking_distance_m=braking,
        stopping_distance_m=reaction_distance + braking,
    )


def markdown_table(envelopes: list[Envelope], clearance_m: float | None = None) -> str:
    """The table a project asks you to paste into your evidence write-up."""
    header = "| Speed | Blind | Braking | Stopping |"
    divider = "|---|---|---|---|"
    if clearance_m is not None:
        header += f" Fits in {clearance_m * 100:.0f} cm? |"
        divider += "---|"
    lines = [header, divider]
    for e in envelopes:
        row = (f"| {e.speed_m_s:.2f} m/s | {e.reaction_distance_m * 100:.1f} cm "
               f"| {e.braking_distance_m * 100:.1f} cm | {e.stopping_distance_m * 100:.1f} cm |")
        if clearance_m is not None:
            row += " yes |" if e.fits_in(clearance_m) else " **no** |"
        lines.append(row)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speeds", default="0.10,0.20,0.30,0.40,0.50",
                    help="comma-separated speeds in m/s")
    ap.add_argument("--reaction", type=float,
                    help="total reaction time in seconds; omit to build it from the parts below")
    ap.add_argument("--link-latency", type=float, default=0.0,
                    help="measured round-trip p99 in seconds (01-first-robot/code/link_latency.py)")
    ap.add_argument("--loop-hz", type=float, default=DEFAULT_LOOP_HZ, help="your control loop rate")
    ap.add_argument("--motor-tau", type=float, default=DEFAULT_MOTOR_TAU_S,
                    help="motor time constant in seconds")
    ap.add_argument("--extra", type=float, default=0.0,
                    help="any other delay in seconds (sensor period, blocking read, human)")
    ap.add_argument("--decel", type=float, default=DEFAULT_DECEL_M_S2,
                    help="braking deceleration in m/s^2 (measure it: --coast)")
    ap.add_argument("--coast", nargs=2, type=float, metavar=("SPEED", "DISTANCE"),
                    help="derive --decel from a measured stop test instead of guessing")
    ap.add_argument("--clearance", type=float,
                    help="metres of clear space you actually have; adds a verdict column")
    args = ap.parse_args(argv)

    decel = decel_from_coast_m_s2(*args.coast) if args.coast else args.decel
    reaction = args.reaction if args.reaction is not None else reaction_time_s(
        args.link_latency, args.loop_hz, args.motor_tau, args.extra)

    speeds = [float(s) for s in args.speeds.split(",") if s.strip()]
    if not speeds:
        raise ValueError("no speeds given")
    envelopes = [envelope(v, reaction, decel) for v in speeds]

    if args.coast:
        print(f"measured deceleration {decel:.2f} m/s^2 "
              f"(from {args.coast[0]:.2f} m/s over {args.coast[1]:.2f} m)")
    print(f"reaction time  {reaction * 1000:.0f} ms   deceleration {decel:.2f} m/s^2\n")
    print(markdown_table(envelopes, args.clearance))
    if args.clearance is not None:
        print(f"\nfastest speed that stops within {args.clearance * 100:.0f} cm: "
              f"{max_safe_speed_m_s(args.clearance, reaction, decel):.2f} m/s")
        return 0 if all(e.fits_in(args.clearance) for e in envelopes) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
