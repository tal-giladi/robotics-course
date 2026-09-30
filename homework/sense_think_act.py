"""00.01 — a robot is a loop, not a program: open loop vs sense–think–act.

A 1D robot drives toward a wall. Physics runs at 1 kHz. The "brain" runs at loop_hz.
Standard library only:  python sense_think_act.py
"""
from dataclasses import dataclass

PHYSICS_DT = 0.001          # s, the "real world" integrates at 1 kHz
WALL_X = 2.1                # m, wall position; robot starts at x = 0
CRUISE = 0.5                # m/s commanded speed
STOP_AT = 0.30              # m, desired gap to the wall
TAU = 0.08                  # s, motor time constant (labs/config/karmel.yaml)


@dataclass
class World:
    x: float = 0.0          # robot front position, m
    v: float = 0.0          # actual speed, m/s

    def step(self, v_cmd: float, motor_gain: float) -> None:
        # first-order motor: actual speed approaches gain * command with time constant TAU
        self.v += (motor_gain * v_cmd - self.v) * PHYSICS_DT / TAU
        self.x += self.v * PHYSICS_DT

    def range_sensor(self) -> float:
        return WALL_X - self.x


def open_loop(motor_gain: float) -> float:
    """'A program with motors attached': compute the drive time once, then just wait."""
    w = World()
    drive_time = (WALL_X - STOP_AT) / CRUISE          # 3.6 s — assumes the robot does exactly 0.5 m/s
    t = 0.0
    while t < drive_time + 1.0:                       # +1 s to let it coast to a stop
        w.step(CRUISE if t < drive_time else 0.0, motor_gain)
        t += PHYSICS_DT
    return w.range_sensor()


def closed_loop(motor_gain: float, loop_hz: float, latency_s: float = 0.0, first_tick_s: float = 0.0) -> float:
    """Sense–think–act at loop_hz, first iteration at first_tick_s. 'Think' sees a reading latency_s old."""
    w = World()
    history: list[tuple[float, float]] = []           # (time, range) — lets us model sensor latency
    period = 1.0 / loop_hz
    next_tick, v_cmd, t = first_tick_s, CRUISE, 0.0
    while t < 6.0:
        history.append((t, w.range_sensor()))
        if t >= next_tick:                            # --- one iteration of the loop ---
            stale = [r for (ts, r) in history if ts <= t - latency_s]
            sensed = stale[-1] if stale else history[0][1]          # SENSE
            v_cmd = 0.0 if sensed <= STOP_AT else CRUISE            # THINK
            next_tick += period                                     # ACT is applied below
        w.step(v_cmd, motor_gain)
        t += PHYSICS_DT
        if w.range_sensor() <= 0.0:
            return 0.0                                # crashed into the wall
    return w.range_sensor()


if __name__ == "__main__":
    rows = [
        ("open loop, fresh battery", open_loop(1.0)),
        ("open loop, tired battery + carpet", open_loop(0.85)),
        ("closed loop 50 Hz, fresh battery", closed_loop(1.0, 50)),
        ("closed loop 50 Hz, tired battery + carpet", closed_loop(0.85, 50)),
        ("closed loop 5 Hz", closed_loop(1.0, 5)),
        ("closed loop 0.5 Hz, first tick at 0.0 s", closed_loop(1.0, 0.5)),
        ("closed loop 0.5 Hz, first tick at 1.0 s", closed_loop(1.0, 0.5, first_tick_s=1.0)),
        ("closed loop 50 Hz, sensor 0.5 s old", closed_loop(1.0, 50, latency_s=0.5)),
        ("closed loop 50 Hz, sensor 1.5 s old", closed_loop(1.0, 50, latency_s=1.5)),
    ]
    print(f"Final gap to the wall (goal {STOP_AT:.2f} m, 0.000 = crashed)")
    for label, gap in rows:
        print(f"  {label:44s} {gap:6.3f} m")
