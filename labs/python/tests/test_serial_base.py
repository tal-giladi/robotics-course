"""SerialBase against an in-memory serial port: acks, retries, keep-alive, dead-man, reconnect.

The fake port answers like the firmware (via :class:`robotlab.fake_pico.FakePico`) but lets
tests drop acknowledgements or "unplug the cable" at will.
"""

from __future__ import annotations

import threading
import time

import pytest

from robotlab import protocol
from robotlab.fake_pico import FakePico
from robotlab.hal import BaseState, DifferentialBase
from robotlab.serial_base import (
    AckTimeoutError,
    CommandRejectedError,
    SerialBase,
    TelemetryTimeoutError,
    duty_to_permille,
    mm_to_m,
    mrad_s_to_rad_s,
    mv_to_v,
    rad_s_to_mrad_s,
    telemetry_to_state,
)


class MemoryPort:
    """A pyserial-like port connected to a FakePico. Telemetry every 20 ms of wall time."""

    def __init__(self, pico: FakePico, owner: PortFactory) -> None:
        self.pico = pico
        self.owner = owner
        self.rx = bytearray()
        self.lock = threading.Lock()
        self.last_telemetry = 0.0
        self.written: list[str] = []
        self.closed = False

    def _check(self) -> None:
        if self.closed or self.owner.unplugged:
            raise OSError("device disconnected")

    def write(self, data: bytes) -> int:
        self._check()
        for line in data.decode("ascii").splitlines():
            self.written.append(line)
            reply = self.pico.handle_line(line)
            if reply is None:
                continue
            if reply.startswith("A") and self.owner.drop_acks > 0:
                self.owner.drop_acks -= 1
                continue
            with self.lock:
                self.rx += reply.encode("ascii")
        return len(data)

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def read(self, n: int = 1) -> bytes:
        self._check()
        now = time.monotonic()
        if self.owner.telemetry and now - self.last_telemetry >= 0.02:
            self.last_telemetry = now
            self.pico.step(0.02)
            with self.lock:
                self.rx += self.pico.telemetry_line().encode("ascii")
        with self.lock:
            if not self.rx:
                data = b""
            else:
                data = bytes(self.rx[:n])
                del self.rx[:n]
        return data

    def close(self) -> None:
        self.closed = True


class PortFactory:
    def __init__(self) -> None:
        self.pico = FakePico()
        self.ports: list[MemoryPort] = []
        self.drop_acks = 0
        self.unplugged = False
        self.telemetry = True

    def __call__(self, port: str, baud: int, timeout: float) -> MemoryPort:
        if self.unplugged:
            raise OSError("no such device")
        memory_port = MemoryPort(self.pico, self)
        self.ports.append(memory_port)
        return memory_port

    def written(self, kind: str) -> list[str]:
        return [line for port in self.ports for line in port.written if line.startswith(kind)]


@pytest.fixture
def factory() -> PortFactory:
    return PortFactory()


def make_base(factory: PortFactory, **kwargs) -> SerialBase:
    options = dict(ack_timeout_s=0.1, connect_timeout_s=2.0, reconnect_interval_s=0.05, max_wheel_speed_rad_s=17.0)
    options.update(kwargs)
    return SerialBase("memory", serial_factory=factory, **options)


# --- unit conversions ---------------------------------------------------------------------------------
def test_unit_conversions():
    assert rad_s_to_mrad_s(5.0) == 5000
    assert rad_s_to_mrad_s(-0.0014) == -1
    assert mrad_s_to_rad_s(-2500) == -2.5
    assert duty_to_permille(0.4) == 400
    assert duty_to_permille(-3.0) == -1000
    assert mv_to_v(11850) == pytest.approx(11.85) and mv_to_v(-1) is None
    assert mm_to_m(523) == pytest.approx(0.523) and mm_to_m(-1) is None
    with pytest.raises(ValueError):
        duty_to_permille(float("nan"))


def test_telemetry_to_state():
    state = telemetry_to_state(protocol.Telemetry(1500, 10, -20, 6283, -100, 11100, 350, 9))
    assert state == BaseState(1.5, 10, -20, 6.283, -0.1, 11.1, 0.35, 9)


# --- behaviour ----------------------------------------------------------------------------------------
def test_connects_and_implements_the_hal(factory):
    with make_base(factory) as base:
        assert isinstance(base, DifferentialBase)
        assert base.firmware == protocol.HelloReply("fake-0.1.0", 1)
        state = base.read()
        assert state.flags & protocol.FLAG_WATCHDOG  # nothing commanded yet
        assert state.battery_v is not None and state.battery_v > 9.0


def test_commands_are_sent_in_wire_units_and_clamped(factory):
    with make_base(factory) as base:
        base.set_wheel_duty(0.25, -2.0)
        base.set_wheel_velocity(5.0, -30.0)
        base.set_param("kp", 0.05)
        base.reset_encoders()
        m = protocol.decode(factory.written("M")[0] + "\n")
        v = protocol.decode(factory.written("V")[0] + "\n")
        assert (m.left, m.right) == (250, -1000)
        assert (v.left_mrad_s, v.right_mrad_s) == (5000, -17000)
        assert factory.written("P")[0].startswith("P ") and " kp 0.05*" in factory.written("P")[0]
        seqs = [protocol.decode(line + "\n").seq for line in factory.written("")]
        assert len(set(seqs)) == len(seqs)  # every command has its own sequence number


def test_stop_retries_when_an_ack_is_lost(factory):
    with make_base(factory) as base:
        factory.drop_acks = 1
        base.stop()  # first S unacknowledged -> resent
        assert len(factory.written("S")) == 2
        assert base.stats.ack_timeouts == 1


def test_stop_gives_up_after_retries(factory):
    base = make_base(factory, retries=1)
    try:
        factory.drop_acks = 10
        with pytest.raises(AckTimeoutError):
            base.stop()
        assert len(factory.written("S")) == 2
    finally:
        factory.drop_acks = 0
        base.close()


def test_rejected_parameter_raises(factory):
    with make_base(factory) as base:
        with pytest.raises(CommandRejectedError, match="out_of_range"):
            base.set_param("watchdog_ms", 1)


def test_keepalive_repeats_the_drive_command_so_the_watchdog_does_not_trip(factory):
    with make_base(factory, keepalive_s=0.05) as base:
        base.set_wheel_duty(0.3, 0.3)
        time.sleep(0.6)  # the script "sleeps"; the firmware watchdog is 300 ms
        assert len(factory.written("M")) >= 6
        assert not base.read().flags & protocol.FLAG_WATCHDOG
        base.stop()
        count = len(factory.written("M"))
        time.sleep(0.2)
        assert len(factory.written("M")) == count  # stop() ends the keep-alive


def test_command_timeout_is_a_dead_man_switch(factory):
    with make_base(factory, keepalive_s=0.05, command_timeout_s=0.2) as base:
        base.set_wheel_velocity(3.0, 3.0)
        time.sleep(0.9)  # the application stopped calling set_wheel_velocity
        sent = len(factory.written("V"))
        assert 2 <= sent <= 6
        assert base.read().flags & protocol.FLAG_WATCHDOG  # firmware stopped the robot


def test_stale_telemetry_raises(factory):
    with make_base(factory, max_telemetry_age_s=0.15) as base:
        base.read()
        factory.telemetry = False
        time.sleep(0.3)
        with pytest.raises(TelemetryTimeoutError):
            base.read()
        factory.telemetry = True
        base.wait_for_telemetry(timeout_s=1.0, newer_than=base.read(max_age_s=10).t)


def test_reconnects_after_unplug_and_forgets_the_drive_command(factory):
    with make_base(factory, keepalive_s=0.05) as base:
        base.set_wheel_duty(0.5, 0.5)
        factory.unplugged = True
        deadline = time.monotonic() + 2.0
        while base.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not base.connected
        time.sleep(0.2)
        factory.unplugged = False
        deadline = time.monotonic() + 2.0
        while not base.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        assert base.connected and base.stats.reconnects == 1
        m_count = len(factory.written("M"))
        base.wait_for_telemetry(timeout_s=1.0, newer_than=base.read(max_age_s=10).t)
        time.sleep(0.3)
        assert len(factory.written("M")) == m_count  # SAFETY: no automatic restart of the motors


def test_sequence_numbers_wrap_at_16_bits(factory):
    with make_base(factory) as base:
        base._seq = protocol.SEQ_MAX - 1
        base.set_wheel_duty(0.0, 0.0)
        base.set_wheel_duty(0.0, 0.0)
        seqs = [protocol.decode(line + "\n").seq for line in factory.written("M")[-2:]]
        assert seqs == [protocol.SEQ_MAX, 0]
