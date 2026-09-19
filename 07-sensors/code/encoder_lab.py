"""07.02 — Wheel encoders in depth: resolution, velocity estimation, missed edges, slip and skid.

    py 07-sensors/code/encoder_lab.py numbers          # resolution, edge rates, rollover
    py 07-sensors/code/encoder_lab.py velocity         # counting vs period timing, per speed
    py 07-sensors/code/encoder_lab.py irq              # why a slow interrupt handler loses edges
    py 07-sensors/code/encoder_lab.py slip             # encoders vs truth vs gyro in the simulator

All robot numbers come from labs/config/karmel.yaml.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT / "labs" / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World  # noqa: E402

TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class EncoderGeometry:
    ticks_per_rev: int
    wheel_radius_m: float
    wheel_separation_m: float

    @classmethod
    def from_config(cls) -> EncoderGeometry:
        d = load_config().drive
        return cls(d.ticks_per_wheel_rev, d.wheel_radius_m, d.wheel_separation_m)

    @property
    def meters_per_tick(self) -> float:
        return TWO_PI * self.wheel_radius_m / self.ticks_per_rev

    @property
    def heading_per_tick_rad(self) -> float:
        """Heading change when one wheel moves one tick more than the other."""
        return self.meters_per_tick / self.wheel_separation_m

    def edges_per_second(self, wheel_rad_s: float) -> float:
        return abs(wheel_rad_s) / TWO_PI * self.ticks_per_rev

    def rollover_distance_m(self, counter_bits: int) -> float:
        """Distance before a signed ``counter_bits`` counter wraps."""
        return (2 ** (counter_bits - 1)) * self.meters_per_tick


# --- velocity estimation ---------------------------------------------------------------------------
def counting_resolution_rad_s(ticks_per_rev: int, window_s: float) -> float:
    """'Frequency' method: count ticks in a fixed window. One tick more or less changes the estimate by this."""
    return TWO_PI / (ticks_per_rev * window_s)


def period_relative_error(wheel_rad_s: float, ticks_per_rev: int, timer_resolution_s: float) -> float:
    """'Period' method: time between two edges. A timing error of one clock step is this fraction of the speed."""
    edge_interval = TWO_PI / (ticks_per_rev * abs(wheel_rad_s))
    return timer_resolution_s / edge_interval


def crossover_speed_rad_s(ticks_per_rev: int, window_s: float, timer_resolution_s: float) -> float:
    """Speed where both methods have the same absolute error; below it, time the edges; above it, count them.

    counting: dw = 2*pi/(N*T);  period: dw = w * tau * w * N / (2*pi)  ->  w^2 = (2*pi)^2 / (N^2 * T * tau)
    """
    return TWO_PI / (ticks_per_rev * math.sqrt(window_s * timer_resolution_s))


# --- interrupt counting with a slow handler --------------------------------------------------------
QUAD_SEQUENCE = (0b00, 0b01, 0b11, 0b10)  # (B << 1) | A, forward order used by labs/firmware/pico/encoders.py
TRANSITION_TABLE = (0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0)  # same table as the firmware


def simulate_irq_counting(
    wheel_rad_s: float,
    seconds: float,
    ticks_per_rev: int,
    handler_us: float = 60.0,
    jitter_us: float = 40.0,
    phase_error_deg: float = 25.0,
    blocked_ms: float = 1.0,
    blocks_per_s: float = 2.0,
    seed: int = 0,
) -> tuple[int, int]:
    """Count forward edges the way a pin-interrupt handler does; return ``(true_ticks, counted_ticks)``.

    Model (illustrative numbers, not measured): each edge requests an interrupt. The handler starts
    when the CPU is free, takes ``handler_us`` plus exponential jitter, reads both pins and applies the
    transition table to the state it SEES then. Requests arriving while a handler runs merge into one
    pending request (that is how pin interrupt flags work). A few times per second the CPU is blocked
    for ``blocked_ms`` (other interrupts, USB, flash). If the pins moved two steps between two handler
    runs, the table says 0 - those ticks are lost; three steps look like one step BACKWARDS.
    Hall sensors are rarely exactly 90° apart: ``phase_error_deg`` squeezes every other edge gap.
    """
    rng = np.random.default_rng(seed)
    cycle_s = TWO_PI / (abs(wheel_rad_s) * ticks_per_rev / 4.0)  # one full A/B cycle = 4 ticks
    squeeze = phase_error_deg / 90.0
    gaps = np.array([1.0 - squeeze, 1.0 + squeeze, 1.0 - squeeze, 1.0 + squeeze]) * cycle_s / 4.0
    n_edges = int(seconds / cycle_s * 4)
    edge_times = np.cumsum(np.resize(gaps, n_edges))
    block_starts = np.sort(rng.uniform(0.0, seconds, rng.poisson(blocks_per_s * seconds)))
    block_s = blocked_ms / 1000.0

    def state_at(t: float) -> int:
        k = int(np.searchsorted(edge_times, t, side="right"))  # edges that happened by time t
        return QUAD_SEQUENCE[k % 4]

    def free_after(t: float) -> float:
        for b in block_starts:
            if b <= t < b + block_s:
                return b + block_s
        return t

    count, state, cpu_free, i = 0, QUAD_SEQUENCE[0], 0.0, 0
    while i < n_edges:
        start = free_after(max(edge_times[i], cpu_free))
        new_state = state_at(start)  # the handler reads the pins NOW, not at the edge
        count += TRANSITION_TABLE[(state << 2) | new_state]
        state = new_state
        cpu_free = start + (handler_us + rng.exponential(jitter_us)) * 1e-6
        # every edge up to the handler's START was seen by this read; later ones re-trigger the IRQ
        i = int(np.searchsorted(edge_times, start, side="right"))
    return n_edges, count


# --- slip and skid in the simulator ----------------------------------------------------------------
@dataclass(frozen=True)
class SlipResult:
    label: str
    true_distance_m: float
    encoder_distance_m: float
    true_heading_deg: float
    encoder_heading_deg: float
    gyro_heading_deg: float


def drive_and_compare(label: str, params: DiffDriveParams, left_rad_s: float, right_rad_s: float,
                      seconds: float, seed: int = 3) -> SlipResult:
    """Drive a constant wheel-speed command; compare encoder odometry and gyro heading with the truth."""
    cfg = load_config()
    sensors = replace(SensorParams.ideal(cfg), gyro_noise_std_rad_s=0.002)
    sim = DiffDriveSim(World(), params, sensors, pose=(0.0, 0.0, 0.0), seed=seed)
    geo = EncoderGeometry.from_config()  # what the robot BELIEVES (nominal radius and separation)
    dt = 0.01
    sim.set_velocity(left_rad_s, right_rad_s)
    true_distance = gyro_heading = true_heading = 0.0
    prev = sim.pose
    for _ in range(round(seconds / dt)):
        sim.step(dt)
        true_distance += math.hypot(sim.pose.x - prev.x, sim.pose.y - prev.y)
        true_heading += sim.yaw_rate * dt
        gyro_heading += sim.gyro_z() * dt
        prev = sim.pose
    left, right = sim.ticks
    d_left, d_right = left * geo.meters_per_tick, right * geo.meters_per_tick
    return SlipResult(
        label,
        true_distance,
        abs(d_left + d_right) / 2.0,
        math.degrees(true_heading),
        math.degrees((d_right - d_left) / geo.wheel_separation_m),
        math.degrees(gyro_heading),
    )


def slip_experiment() -> list[SlipResult]:
    cfg = load_config()
    ideal = DiffDriveParams.ideal(cfg)
    slippery = replace(ideal, slip_std=0.05)
    # Skid on turns: the tyres' contact patches scrub sideways, so the robot turns as if its wheels were
    # further apart than the ruler says. 1.06 = 6 % wider "effective" track.
    scrubbing = replace(ideal, wheel_separation_scale=1.06)
    spin = 2.0 * math.pi * cfg.drive.wheel_separation_m / 2.0 / cfg.drive.wheel_radius_m / 4.0  # rad/s: 1 turn in 4 s
    return [
        drive_and_compare("straight, ideal", ideal, 8.0, 8.0, 3.0),
        drive_and_compare("straight, 5 % slip noise", slippery, 8.0, 8.0, 3.0),
        drive_and_compare("spin ~1 turn, ideal", ideal, -spin, spin, 4.0),
        drive_and_compare("spin ~1 turn, scrub (6 % wider track)", scrubbing, -spin, spin, 4.0),
    ]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("part", choices=("numbers", "velocity", "irq", "slip"))
    args = ap.parse_args(argv)
    geo = EncoderGeometry.from_config()
    cfg = load_config()
    if args.part == "numbers":
        print(f"ticks per wheel revolution: {geo.ticks_per_rev}")
        print(f"distance per tick:          {geo.meters_per_tick * 1000:.4f} mm")
        print(f"heading per tick difference: {math.degrees(geo.heading_per_tick_rad):.4f} deg")
        for w in (1.0, 5.0, cfg.drive.max_wheel_speed_rad_s, 21.5):
            print(f"edges/s at {w:5.1f} rad/s (one wheel): {geo.edges_per_second(w):7.0f}"
                  f"   -> one edge every {1e6 / geo.edges_per_second(w):6.0f} us")
        for bits in (16, 32):
            print(f"signed {bits}-bit counter wraps after {geo.rollover_distance_m(bits):,.2f} m")
    elif args.part == "velocity":
        print("counting ticks in a window T (resolution = one tick):")
        for window in (0.005, 0.01, 0.02, 0.05, 0.1):
            res = counting_resolution_rad_s(geo.ticks_per_rev, window)
            print(f"  T = {window * 1000:5.0f} ms: +-{res:.3f} rad/s = +-{res * geo.wheel_radius_m * 1000:5.1f} mm/s at the rim,"
                  f" lag ~ {window * 500:.1f} ms")
        for clock_s, name in ((1e-6, "1 us hardware timer"), (100e-6, "100 us timestamp jitter (Python IRQ)")):
            print(f"timing the gap between edges, {name}:")
            for w in (0.2, 1.0, 5.0, 17.0):
                rel = period_relative_error(w, geo.ticks_per_rev, clock_s)
                gap_ms = TWO_PI / (geo.ticks_per_rev * w) * 1000
                print(f"  {w:5.1f} rad/s: edge gap {gap_ms:7.3f} ms, error {rel * 100:6.2f} % = {rel * w:.4f} rad/s")
            print(f"  crossover with a 10 ms counting window: {crossover_speed_rad_s(geo.ticks_per_rev, 0.01, clock_s):.2f} rad/s")
    elif args.part == "irq":
        print("simulated pin-interrupt counting, 2 s per speed (illustrative handler timing):")
        print("  wheel [rad/s]   edges/s   true   counted   lost")
        for w in (2.0, 5.0, 10.0, 17.0, 21.5):
            true, counted = simulate_irq_counting(w, 2.0, geo.ticks_per_rev)
            print(f"  {w:13.1f}  {geo.edges_per_second(w):8.0f}  {true:6d}  {counted:8d}  {100 * (true - counted) / true:5.2f} %")
    else:
        print("label                                   true dist  enc dist   true hdg   enc hdg   gyro hdg")
        for r in slip_experiment():
            print(f"{r.label:38s}  {r.true_distance_m:8.3f}  {r.encoder_distance_m:8.3f}  {r.true_heading_deg:9.1f}"
                  f"  {r.encoder_heading_deg:8.1f}  {r.gyro_heading_deg:9.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
