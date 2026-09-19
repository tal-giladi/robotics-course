"""01.08 — what a duty cycle actually does to the motor, from labs/config/karmel.yaml.

Run from the repository root (no hardware, no robot needed):

    python duty_map.py                         # the table, with karmel's configured deadband
    python duty_map.py --deadband 0.17         # after you measure YOUR deadband
    python duty_map.py --bus 9.9 --deadband 0.17

It prints, for each commanded duty: the duty after deadband compensation, the average voltage
the H-bridge puts across the motor, and the wheel speed you should expect if the motor is
roughly linear above the deadband. Compare the last column with what you measure in 01.09.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent
for candidate in (REPO / "labs" / "python", Path.cwd() / "labs" / "python"):
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))
        break

from robotlab.config import load_config  # noqa: E402

PWM_FREQ_HZ = 20_000          # labs/firmware/pico/config.py


def compensate_deadband(duty: float, deadband: float) -> float:
    """The mapping in labs/firmware/pico/motors.py: stretch 0..1 onto deadband..1.

    Zero stays zero, so "stopped" still means stopped.
    """
    if duty == 0:
        return 0.0
    magnitude = deadband + abs(duty) * (1.0 - deadband)
    return magnitude if duty > 0 else -magnitude


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--deadband", type=float, default=None,
                        help="fraction, e.g. 0.17 (default: drive.duty_deadband from karmel.yaml)")
    parser.add_argument("--bus", type=float, default=None,
                        help="battery volts (default: battery.nominal_v from karmel.yaml)")
    args = parser.parse_args()

    cfg = load_config()
    deadband = cfg.drive.duty_deadband if args.deadband is None else args.deadband
    bus = cfg.battery.nominal_v if args.bus is None else args.bus
    top_speed = cfg.drive.max_wheel_speed_rad_s
    radius = cfg.drive.wheel_radius_m
    ticks = cfg.drive.ticks_per_wheel_rev

    print(f"config          : {cfg.source}")
    print(f"bus voltage     : {bus:.1f} V")
    print(f"PWM             : {PWM_FREQ_HZ / 1000:.0f} kHz, period {1e6 / PWM_FREQ_HZ:.0f} us")
    print(f"deadband        : {deadband:.2f} ({deadband * 100:.0f} % duty before the wheel moves)")
    print(f"max wheel speed : {top_speed:.1f} rad/s loaded  ({top_speed * radius:.2f} m/s at the rim)")
    print()
    print("  commanded   compensated   on-time    avg motor V   wheel rad/s   m/s     ticks/s")
    print("  ---------   -----------   -------    -----------   -----------   -----   -------")
    for commanded in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0):
        applied = compensate_deadband(commanded, deadband)
        on_time_us = abs(applied) * 1e6 / PWM_FREQ_HZ
        volts = abs(applied) * bus
        # Linear model above the deadband: duty 1.0 -> max_wheel_speed_rad_s.
        speed = 0.0 if applied == 0 else max(0.0, (abs(applied) - deadband) / (1 - deadband)) * top_speed
        print(f"   {commanded:5.2f}       {applied:6.3f}      {on_time_us:5.1f} us     "
              f"{volts:5.2f} V       {speed:6.2f}      {speed * radius:5.2f}   "
              f"{speed / (2 * math.pi) * ticks:7.0f}")
    print()
    print("Read the table this way:")
    print(f"  * Without compensation, every command below {deadband:.2f} does nothing at all -")
    print("    the controller in module 08 would see zero response and wind up its integrator.")
    print("  * With compensation, the smallest non-zero command already produces motion,")
    print("    and the speed is roughly proportional to the command.")
    print("  * The avg motor V column is what a slow meter reads across the motor terminals;")
    print("    an oscilloscope shows the full bus voltage switching at 20 kHz (FE.07).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
