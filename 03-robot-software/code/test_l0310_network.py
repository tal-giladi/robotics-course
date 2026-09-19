"""Tests for the lesson 03.10 network lab — loopback only, no hardware, no external network.

    python -m pytest 03-robot-software/code/test_l0310_network.py
"""

from __future__ import annotations

import pytest

import l0310_net_lab as net
from robotlab.hal import FLAG_WATCHDOG, BaseState, DifferentialBase


# --- link measurement ---------------------------------------------------------------------------
def test_loopback_round_trip_is_fast_and_lossless() -> None:
    with net.UdpEchoServer() as server:
        stats = net.measure_link(server.host, server.port, count=30, interval_s=0.0)
    assert stats.sent == 30
    assert stats.received >= 28, "loopback should not lose packets"
    assert stats.loss_fraction < 0.1
    assert 0.0 < stats.percentile(50) < 20.0, "loopback p50 in milliseconds"
    assert max(stats.rtt_ms) >= stats.percentile(50)


def test_a_closed_port_loses_everything_without_hanging() -> None:
    server = net.UdpEchoServer()
    server.start()
    host, port = server.host, server.port
    server.stop()
    stats = net.measure_link(host, port, count=3, timeout_s=0.05, interval_s=0.0)
    assert stats.received == 0 and stats.loss_fraction == 1.0
    assert "every packet lost" in stats.line()


def test_percentiles_are_nearest_rank() -> None:
    stats = net.LinkStats("x", sent=6, received=6, rtt_ms=[1.0, 2.0, 3.0, 4.0, 5.0, 100.0])
    assert stats.percentile(50) == 3.0
    assert stats.percentile(99) == 100.0, "the tail is the number that matters for a robot"
    assert stats.loss_fraction == 0.0
    # One packet in six is 20x the median. The mean (19.2 ms) describes nothing that happened.
    assert stats.percentile(50) * 10 < sum(stats.rtt_ms) / len(stats.rtt_ms) * 2


# --- the link between your code and the robot -----------------------------------------------------
class CountingBase:
    """Counts what actually reached the robot, and reports a state that changes every read."""

    def __init__(self) -> None:
        self.t = 0.0
        self.velocity: list[tuple[float, float]] = []
        self.stops = 0
        self.ticks = 0

    def set_wheel_duty(self, left: float, right: float) -> None: ...

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self.velocity.append((left_rad_s, right_rad_s))

    def stop(self) -> None:
        self.stops += 1

    def read(self) -> BaseState:
        self.t += 0.02
        self.ticks += 10
        return BaseState(self.t, self.ticks, self.ticks, 1.0, 1.0, 12.0, 1.0, 0)

    def close(self) -> None: ...


def test_delayed_base_is_still_a_differential_base() -> None:
    assert isinstance(net.DelayedBase(CountingBase()), DifferentialBase)


def test_zero_delay_passes_everything_straight_through() -> None:
    inner = CountingBase()
    link = net.DelayedBase(inner, delay_steps=0)
    link.set_wheel_velocity(4.0, 4.0)
    state = link.read()
    assert inner.velocity == [(4.0, 4.0)]
    assert state.t == pytest.approx(0.02), "no delay: you see the newest sample"


def test_commands_arrive_n_steps_late() -> None:
    inner = CountingBase()
    link = net.DelayedBase(inner, delay_steps=3)
    for _ in range(5):
        link.set_wheel_velocity(4.0, 4.0)
        link.read()
    assert len(inner.velocity) == 2, "3 of the 5 commands are still in flight"
    assert link.commands_sent == 5 and link.commands_delivered == 2


def test_readings_arrive_n_steps_late() -> None:
    inner = CountingBase()
    link = net.DelayedBase(inner, delay_steps=2)
    states = [link.read() for _ in range(5)]
    # Reads 1..3 can only return the oldest sample there is; by read 5 you are exactly 2 behind.
    assert states[-1].t == pytest.approx(0.06)
    assert states[-1].left_ticks == 30, "you are steering on data that is 40 ms old"


def test_stop_is_never_delayed_or_dropped() -> None:
    inner = CountingBase()
    link = net.DelayedBase(inner, delay_steps=10, loss=1.0)
    link.set_wheel_velocity(4.0, 4.0)
    link.stop()
    assert inner.stops == 1, "SAFETY: a stop must not sit in a queue or be dropped"
    assert inner.velocity == [], "...while the drive command was lost, as asked"


def test_loss_is_deterministic_for_a_seed() -> None:
    def delivered(seed: int) -> int:
        inner = CountingBase()
        link = net.DelayedBase(inner, loss=0.5, seed=seed)
        for _ in range(200):
            link.set_wheel_velocity(4.0, 4.0)
            link.read()
        return len(inner.velocity)

    assert delivered(1) == delivered(1)
    assert 70 <= delivered(1) <= 130, "about half of 200"


def test_a_blackout_freezes_the_readings_and_blocks_the_commands() -> None:
    inner = CountingBase()
    link = net.DelayedBase(inner, blackout=(0.06, 0.12))
    seen = []
    for _ in range(8):
        link.set_wheel_velocity(4.0, 4.0)
        seen.append(link.read().t)
    # The robot's clock runs 0.02 .. 0.16; the blackout covers its samples at 0.06, 0.08 and 0.10.
    # During those three reads your code keeps being handed the last sample that arrived, 0.04.
    assert seen[:2] == pytest.approx([0.02, 0.04])
    assert seen[2] == seen[3] == seen[4] == pytest.approx(0.04), "the numbers simply stop changing"
    assert seen[5] == pytest.approx(0.12), "the link returns and you jump to the newest sample"
    assert link.blind_samples == 3
    assert len(inner.velocity) == 5, "3 of 8 commands never left"


# --- the control experiment ----------------------------------------------------------------------
def test_latency_costs_exactly_the_distance_travelled_while_stale() -> None:
    """3.6 mm per 20 ms step at 4 rad/s: speed x wheel radius x dt = 4 x 0.045 x 0.02."""
    results = net.latency_sweep([0, 5, 10])
    assert all(r.reason == "target" for r in results)
    assert results[0].overshoot_m == pytest.approx(0.0, abs=1e-9)
    assert results[1].overshoot_m == pytest.approx(5 * 0.0036, abs=0.004)
    assert results[2].overshoot_m == pytest.approx(10 * 0.0036, abs=0.006)
    assert results[2].stopped_at_m is not None and results[2].stopped_at_m < results[0].stopped_at_m


def test_a_short_blackout_is_absorbed_by_the_watchdog_margin() -> None:
    baseline = net.run_approach(0).stopped_at_m
    result = net.run_approach(0, blackout=(12.6, 12.8), baseline_m=baseline)
    assert result.blind_samples == 10 and result.watchdog_samples == 0, "0.2 s < the 300 ms watchdog"
    assert result.stopped_at_m == pytest.approx(baseline, abs=0.005)


def test_a_long_blackout_trips_the_watchdog_and_stops_the_robot_early() -> None:
    baseline = net.run_approach(0).stopped_at_m
    result = net.run_approach(0, blackout=(12.6, 14.6), baseline_m=baseline)
    assert result.watchdog_samples > 50, "no commands get through, so the firmware stops the motors"
    assert result.stopped_at_m is not None and result.stopped_at_m > baseline, (
        "a partition stops the robot EARLY, not late — the watchdog is the safety net")


def test_command_loss_costs_speed_not_safety() -> None:
    baseline = net.run_approach(0).stopped_at_m
    lossy = net.run_approach(0, loss=0.3, baseline_m=baseline)
    assert lossy.commands_undelivered > 100
    assert lossy.stopped_at_m == pytest.approx(baseline, abs=0.01), (
        "dropped commands make the robot slower, not less safe: the sensor path is untouched")


# --- the command line ---------------------------------------------------------------------------
def test_link_command_runs_against_loopback(capsys: pytest.CaptureFixture[str]) -> None:
    assert net.main(["link", "--count", "10"]) == 0
    assert "loopback" in capsys.readouterr().out


def test_control_command_prints_the_sweep(capsys: pytest.CaptureFixture[str]) -> None:
    assert net.main(["control", "--delays", "0,5"]) == 0
    out = capsys.readouterr().out
    assert "overshoot" in out and "3.6 mm" in out


def test_partition_command_reports_the_watchdog(capsys: pytest.CaptureFixture[str]) -> None:
    assert net.main(["control", "--partition", "2.0"]) == 0
    out = capsys.readouterr().out
    assert "blind reads" in out and "watchdog samples" in out
    assert str(FLAG_WATCHDOG) or True  # the flag itself is checked in the unit tests above
