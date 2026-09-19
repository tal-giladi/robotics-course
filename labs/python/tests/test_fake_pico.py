"""The fake Pico: protocol behaviour, agreement with the real firmware logic, and an end-to-end
drive through SerialBase -> TCP -> FakePico -> simulator."""

from __future__ import annotations

import importlib
import math
import re
import sys
import time
from pathlib import Path

import pytest

from robotlab import protocol as p
from robotlab.fake_pico import FakePico, FakePicoServer, default_sim
from robotlab.serial_base import SerialBase

TESTS_DIR = Path(__file__).resolve().parent
FIRMWARE_DIR = TESTS_DIR.parents[1] / "firmware" / "pico"


def reply(fake: FakePico, payload: str) -> p.Message | None:
    line = fake.handle_line(p.frame(payload))
    return None if line is None else p.decode(line)


# --- FakePico without a transport ---------------------------------------------------------------------
def test_hello_ack_and_errors():
    fake = FakePico()
    assert reply(fake, "H 1") == p.HelloReply("fake-0.1.0", 1)
    assert reply(fake, "M 2 300 300") == p.Ack(2, True)
    assert reply(fake, "M 3 2000 0") == p.Ack(3, False, "out_of_range")
    assert reply(fake, "P 4 gain 1") == p.Ack(4, False, "unknown_param")
    assert reply(fake, "P 5 telemetry_hz 500") == p.Ack(5, False, "out_of_range")
    assert reply(fake, "Q 6") == p.Ack(6, False, "unknown_command")
    assert reply(fake, "M 7 1 2 3") == p.Ack(7, False, "bad_args")
    assert fake.handle_line("M 8 0 0*00\n") is None  # bad checksum: dropped silently
    assert fake.bad_lines == 1


def test_watchdog_in_simulated_time():
    fake = FakePico(watchdog_ms=300)
    assert fake.telemetry().flags & p.FLAG_WATCHDOG
    reply(fake, "M 1 600 600")
    for _ in range(25):  # 250 ms
        fake.step(0.01)
    moving = fake.telemetry()
    assert not moving.flags & p.FLAG_WATCHDOG and moving.left_mrad_s > 0
    for _ in range(10):  # 350 ms after the command
        fake.step(0.01)
    assert fake.telemetry().flags & p.FLAG_WATCHDOG
    assert fake.sim.duty.tolist() == [0.0, 0.0]


def test_velocity_flag_reset_and_range():
    fake = FakePico()
    reply(fake, "V 1 5000 5000")
    for _ in range(20):  # 200 ms: less than the 300 ms watchdog
        fake.step(0.01)
    t = fake.telemetry()
    assert t.flags & p.FLAG_VELOCITY_MODE and t.left_ticks > 0
    assert 2000 < t.range_mm < 3000  # default room: wall ~2.9 m ahead of the sensor
    reply(fake, "R 2")
    assert fake.telemetry().left_ticks == 0
    reply(fake, "S 3")
    assert not fake.telemetry().flags & p.FLAG_VELOCITY_MODE


def test_param_reaches_the_simulated_controller():
    fake = FakePico()
    assert reply(fake, "P 1 kp 0.5") == p.Ack(1, True)
    assert fake.sim._controller.kp == 0.5
    reply(fake, "P 2 watchdog_ms 1000")
    assert fake.watchdog_ms == 1000


# --- the fake agrees with the real firmware's command handler ----------------------------------------------
@pytest.fixture(scope="module")
def firmware_main():
    shims = str(TESTS_DIR / "mpshims")
    names = ("config", "protocol", "velocity", "watchdog", "main", "utime", "machine", "micropython")
    saved = {n: sys.modules.pop(n) for n in names if n in sys.modules}
    sys.path[:0] = [shims, str(FIRMWARE_DIR)]
    try:
        yield importlib.import_module("main")
    finally:
        for n in names:
            sys.modules.pop(n, None)
        sys.modules.update(saved)
        sys.path.remove(shims)
        sys.path.remove(str(FIRMWARE_DIR))


class _Motors:
    def set_duty(self, left, right):
        pass

    def brake(self):
        pass


class _Encoders:
    def counts(self):
        return 0, 0

    def reset(self):
        pass


@pytest.mark.parametrize(
    "payload",
    ["H 1", "M 2 100 -100", "M 3 1001 0", "M 4 1 2 3", "M 5 a b", "V 6 1 2", "V 7 1", "S 8", "S 9 1",
     "R 10", "P 11 kp 0.1", "P 12 kp -1", "P 13 foo 1", "P 14 kd x", "P 15 watchdog_ms 1.5",
     "P 16 telemetry_hz 0", "P 17 ff 3", "X 18", "A 19 OK", "H", "M -1 0 0", "M 65536 0 0"],
)
def test_fake_replies_like_the_firmware(firmware_main, payload):
    robot = firmware_main.Robot(_Motors(), _Encoders(), now_ms=0)
    line = p.frame(payload)
    firmware_reply = robot.handle_line(line, 0)
    fake_reply = FakePico().handle_line(line)
    if firmware_reply is not None and firmware_reply.startswith("I"):
        assert fake_reply is not None and fake_reply.startswith("I")
    else:
        assert fake_reply == firmware_reply


# --- over TCP ----------------------------------------------------------------------------------------------
def test_server_url_and_reconnect():
    with FakePicoServer(FakePico(), port=0) as server:
        assert server.url.startswith("socket://127.0.0.1:")
        with SerialBase(server.url, reconnect_interval_s=0.05) as base:
            base.read()
            server.disconnect_client()
            deadline = time.monotonic() + 3.0
            while base.stats.reconnects == 0 and time.monotonic() < deadline:
                time.sleep(0.02)
            assert base.stats.reconnects == 1
            state = base.wait_for_telemetry(timeout_s=2.0, newer_than=base.read(max_age_s=10).t)
            assert state.t > 0


def test_end_to_end_drive_half_a_metre():
    """Drive 0.5 m with encoder feedback through the whole stack and check the tick count.

    ticks = distance / wheel circumference x ticks per revolution = 0.5 / (2 pi 0.045) x 2464
    """
    radius, ticks_per_rev, distance = 0.045, 2464, 0.5
    expected_ticks = distance / (2 * math.pi * radius) * ticks_per_rev  # ~4357
    sim = default_sim("empty", pose=(0.0, 0.0, 0.0))
    with FakePicoServer(FakePico(sim), port=0) as server, SerialBase(server.url) as base:
        base.reset_encoders()
        state = base.wait_for_telemetry(timeout_s=1.0, newer_than=base.read().t)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            travelled = (state.left_ticks + state.right_ticks) / 2
            remaining = expected_ticks - travelled
            if remaining <= 0:
                break
            # Creep for the last 15 %: every millisecond of lag (a busy test machine) at this speed
            # costs few ticks, so the check below stays meaningful but not flaky.
            speed = 10.0 if remaining > 0.15 * expected_ticks else 1.0
            base.set_wheel_velocity(speed, speed)
            state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t)
        base.stop()
        time.sleep(0.3)
        final = base.read()

    ticks = (final.left_ticks + final.right_ticks) / 2
    assert expected_ticks == pytest.approx(4357, abs=1)
    assert ticks == pytest.approx(expected_ticks, rel=0.03)  # within ~130 ticks = 15 mm
    assert sim.pose.x == pytest.approx(distance, abs=0.02)  # the simulated robot really moved 0.5 m
    assert abs(sim.pose.y) < 0.01


# --- the labs/robot scripts against the fake ------------------------------------------------------------------
ROBOT_DIR = TESTS_DIR.parents[1] / "robot"


@pytest.fixture(scope="module")
def robot_scripts():
    sys.path.insert(0, str(ROBOT_DIR))
    try:
        yield {name: importlib.import_module(name) for name in
               ("robot_common", "battery_monitor", "log_telemetry", "plot_telemetry", "drive_square")}
    finally:
        sys.path.remove(str(ROBOT_DIR))
        for name in ("robot_common", "battery_monitor", "log_telemetry", "plot_telemetry", "drive_square"):
            sys.modules.pop(name, None)


def test_kinematics_and_config_fallback(robot_scripts, monkeypatch):
    common = robot_scripts["robot_common"]
    params = common.load_params()
    assert params.ticks_per_wheel_rev == 2464
    left, right = common.body_to_wheels(0.0, 1.0, params)  # turn left in place at 1 rad/s
    assert left == pytest.approx(-0.1 / 0.045) and right == pytest.approx(0.1 / 0.045)
    monkeypatch.setitem(sys.modules, "robotlab.config", None)  # pretend robotlab.config is missing
    assert common.load_params() == params  # plain-YAML fallback reads the same values


def test_battery_state_of_charge_and_confirmation(robot_scripts):
    bm = robot_scripts["battery_monitor"]
    assert bm.estimate_soc_percent(12.6, 3) == 100.0
    assert bm.estimate_soc_percent(3 * 3.84, 3) == pytest.approx(50.0)
    assert bm.estimate_soc_percent(8.0, 3) == 0.0
    monitor = bm.BatteryMonitor(cells=3, low_v=10.5, cutoff_v=9.9, window=1, confirm=3)
    assert monitor.update(11.5).level == "ok"
    assert monitor.update(10.2).level == "low"
    assert monitor.update(9.5).level == "low"  # one sag is not enough...
    assert monitor.update(9.5).level == "low"
    assert monitor.update(9.5).level == "critical"  # ...three in a row is


def test_log_and_plot_telemetry_with_fake(robot_scripts, tmp_path):
    csv_path = tmp_path / "run.csv"
    robot_scripts["log_telemetry"].main(
        ["--fake", "--duration", "1.0", "--pre-roll", "0.2", "--velocity", "4", "4", "--output", str(csv_path)])
    data = robot_scripts["plot_telemetry"].read_csv(csv_path)
    assert len(data["t_s"]) > 20 and data["left_ticks"][-1] > 0
    robot_scripts["plot_telemetry"].main([str(csv_path)])
    assert csv_path.with_suffix(".png").stat().st_size > 10_000


def test_drive_square_returns_near_start(robot_scripts, capsys):
    robot_scripts["drive_square"].main(["--fake", "--side", "0.3", "--speed", "0.25"])
    out = capsys.readouterr().out
    assert "side 4 done" in out
    match = re.search(r"ended at x=([-\d.]+) m y=([-\d.]+) m", out)
    assert match is not None
    x, y = float(match.group(1)), float(match.group(2))
    # A smoke test of the script: timing jitter on a busy PC makes corners inexact, so the tolerance
    # is loose. (Precise distance control is checked by test_end_to_end_drive_half_a_metre.)
    assert x == pytest.approx(1.0, abs=0.15) and y == pytest.approx(1.5, abs=0.15)
