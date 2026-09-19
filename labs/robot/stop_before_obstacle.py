"""Drive forward and stop before hitting an obstacle, using the front range sensor.

Lesson 01.13 (ultrasonic and ToF sensors - reactive stop).

    python labs/robot/stop_before_obstacle.py --fake                   # wall 3 m ahead in the sim
    python labs/robot/stop_before_obstacle.py --speed 0.2 --stop-distance 0.30

How far before the obstacle must we start braking? Three things eat distance:
    1. sensor + serial latency: the reading is ~50-100 ms old when we see it  -> speed x latency
    2. the motors don't stop instantly (time constant ~80 ms)                 -> speed x ~0.1 s
    3. a margin for noisy readings
so the stop threshold grows with speed: stop_distance + speed * reaction_time.

If the sensor has no valid reading (nothing in range, or flag 4 = sensor error) we treat
"error" as "stop" - an unknown distance is not a free path.

SAFETY: start with a large --stop-distance and a slow --speed, aim at a soft obstacle (a
cardboard box), and keep a hand near the battery switch. Ctrl-C stops the robot.
"""

from __future__ import annotations

import argparse
import time

from robot_common import add_connection_args, body_to_wheels, connect
from robotlab.hal import FLAG_RANGE_ERROR


def braking_threshold_m(stop_distance_m: float, speed_m_s: float, reaction_time_s: float) -> float:
    return stop_distance_m + speed_m_s * reaction_time_s


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.2, help="forward speed in m/s (default 0.2)")
    parser.add_argument("--stop-distance", type=float, default=0.30, help="stop this far from the obstacle, m")
    parser.add_argument("--reaction-time", type=float, default=0.2, help="latency + motor stopping time, s")
    parser.add_argument("--max-distance", type=float, default=3.0, help="give up after driving this far, m")
    add_connection_args(parser)
    args = parser.parse_args(argv)

    with connect(args) as conn:
        base, params = conn.base, conn.params
        speed = min(args.speed, params.max_linear_speed_m_s)
        threshold = braking_threshold_m(args.stop_distance, speed, args.reaction_time)
        base.reset_encoders()
        state = base.wait_for_telemetry(timeout_s=1.0, newer_than=base.read().t)
        start_ticks = (state.left_ticks + state.right_ticks) / 2
        print(f"driving at {speed:.2f} m/s, braking below {threshold:.2f} m")
        reason = "max distance reached"
        while True:
            driven = ((state.left_ticks + state.right_ticks) / 2 - start_ticks) * params.meters_per_tick
            if state.flags & FLAG_RANGE_ERROR:
                reason = "range sensor error"
                break
            if state.range_m is not None and state.range_m < threshold:
                reason = f"obstacle at {state.range_m:.3f} m"
                break
            if driven >= args.max_distance:
                break
            left, right = body_to_wheels(speed, 0.0, params)
            base.set_wheel_velocity(left, right)  # SAFETY: the wheels turn here
            state = base.wait_for_telemetry(timeout_s=0.5, newer_than=state.t)
        base.stop()
        time.sleep(0.4)
        final = base.read()
        print(f"stopped: {reason}")
        shown = "no reading" if final.range_m is None else f"{final.range_m:.3f} m"
        print(f"distance to obstacle after stopping: {shown}")


if __name__ == "__main__":
    main()
