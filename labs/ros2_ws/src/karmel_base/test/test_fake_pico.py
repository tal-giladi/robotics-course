"""The fake Pico: model behaviour, and a real round trip through a pseudo-terminal with pyserial."""
import math
import os
import subprocess
import sys
import time

import pytest

from karmel_base import protocol as p
from karmel_base import wire
from karmel_base.fake_pico import FakePicoModel


def replies(model, msg):
    return [p.decode(line) for line in model.handle_line(p.encode_bytes(msg))]


def test_hello_reports_protocol_v1():
    out = replies(FakePicoModel(), p.Hello(1))
    assert p.HelloReply('fake-1.0', 1) in out
    assert out == [p.HelloReply('fake-1.0', 1)]          # like the firmware: I only, no A


def test_velocity_command_moves_wheels_and_counts_ticks():
    model = FakePicoModel(ticks_per_rev=2464)
    replies(model, p.VelocityCommand(2, 2000, 2000))
    for _ in range(50):                       # 1 s at 50 Hz, re-sending to feed the watchdog
        replies(model, p.VelocityCommand(3, 2000, 2000))
        model.step(0.02)
    t = model.telemetry()
    assert wire.has_flag(t, p.FLAG_VELOCITY_MODE) and not wire.has_flag(t, p.FLAG_WATCHDOG)
    assert t.left_mrad_s == pytest.approx(2000, abs=30)
    # ~ 2 rad/s for ~1 s minus the motor lag -> ~ (2 - 2*0.08) rad -> ticks
    assert t.left_ticks == pytest.approx((2.0 - 0.16) * 2464 / (2 * math.pi), rel=0.05)


def test_watchdog_stops_motors_without_commands():
    model = FakePicoModel()
    replies(model, p.VelocityCommand(1, 5000, 5000))
    for _ in range(50):
        model.step(0.02)                      # no more commands for 1 s > 300 ms
    t = model.telemetry()
    assert wire.has_flag(t, p.FLAG_WATCHDOG)
    assert abs(t.left_mrad_s) < 100


def test_reset_encoders_and_bad_param():
    model = FakePicoModel()
    replies(model, p.VelocityCommand(1, 3000, 3000))
    for _ in range(10):
        model.step(0.02)
    replies(model, p.ResetEncoders(2))
    assert model.telemetry().left_ticks == 0
    out = replies(model, p.SetParam(3, 'telemetry_hz', 1000))
    assert out == [p.Ack(3, False, 'out_of_range')]


def test_corrupted_command_is_ignored():
    model = FakePicoModel()
    line = bytearray(p.encode_bytes(p.VelocityCommand(1, 3000, 3000)))
    line[2] ^= 1
    assert model.handle_line(bytes(line)) == []


@pytest.mark.skipif(sys.platform == 'win32', reason='pty transport needs Linux/macOS')
def test_round_trip_over_pty(tmp_path):
    serial = pytest.importorskip('serial')
    link = str(tmp_path / 'ttyFAKE')
    proc = subprocess.Popen([sys.executable, '-m', 'karmel_base.fake_pico', '--link', link],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for _ in range(100):
            if os.path.exists(link):
                break
            time.sleep(0.05)
        port = serial.serial_for_url(link, baudrate=115200, timeout=0.1)
        buf = wire.LineBuffer()
        port.write(p.encode_bytes(p.Hello(1)))
        got = []
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            port.write(p.encode_bytes(p.VelocityCommand(2, 1000, -1000)))
            for line in buf.feed(port.read(256)):
                got.append(p.decode(line))
            if any(isinstance(m, p.Telemetry) and m.left_ticks > 50 for m in got):
                break
        port.close()
        assert p.HelloReply('fake-1.0', 1) in got
        telemetry = [m for m in got if isinstance(m, p.Telemetry)]
        assert telemetry and telemetry[-1].left_ticks > 50 and telemetry[-1].right_ticks < -50
    finally:
        proc.terminate()
        proc.wait(timeout=5)


@pytest.mark.parametrize('payload, reason', [
    ('Q 5', 'unknown_command'),
    ('P 5 gain 1', 'unknown_param'),
    ('M 5 2000 0', 'out_of_range'),
    ('V 5 1.5 2', 'bad_args'),
    ('P 5 telemetry_hz 1000', 'out_of_range'),
])
def test_error_replies_match_firmware_codes(payload, reason):
    out = [p.decode(line) for line in FakePicoModel().handle_line(p.frame(payload).encode())]
    assert out == [p.Ack(5, False, reason)]
