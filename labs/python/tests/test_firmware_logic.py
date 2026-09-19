"""Firmware logic tests: the MicroPython modules in labs/firmware/pico, run under CPython.

The hardware modules (``machine``, ``rp2``, ``micropython``, ``utime``) are replaced by the small
shims in ``tests/mpshims``. The ``rp2`` shim includes a PIO interpreter, so the quadrature
decoder *program* is tested instruction by instruction, not just the Python around it.

What these tests can't tell you: real timing, electrical behaviour, whether the sensors answer.
That needs the robot (lessons 01.08-01.14).
"""

from __future__ import annotations

import importlib
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from robotlab import protocol as host_protocol
from robotlab.config import load_config

TESTS_DIR = Path(__file__).resolve().parent
SHIMS_DIR = TESTS_DIR / "mpshims"
FIRMWARE_DIR = TESTS_DIR.parents[1] / "firmware" / "pico"
FIRMWARE_MODULES = ("config", "protocol", "motors", "encoders", "velocity", "watchdog", "sensors", "vl53l1x", "main")
SHIM_MODULES = ("machine", "rp2", "micropython", "utime")


@pytest.fixture(scope="module")
def fw():
    """Import every firmware module (plus shims) in isolation; clean sys.modules/sys.path after."""
    saved = {name: sys.modules.pop(name) for name in FIRMWARE_MODULES + SHIM_MODULES if name in sys.modules}
    sys.path[:0] = [str(SHIMS_DIR), str(FIRMWARE_DIR)]
    try:
        modules = {name: importlib.import_module(name) for name in SHIM_MODULES + FIRMWARE_MODULES}
        yield SimpleNamespace(**modules)
    finally:
        for name in FIRMWARE_MODULES + SHIM_MODULES:
            sys.modules.pop(name, None)
        sys.modules.update(saved)
        for path in (str(SHIMS_DIR), str(FIRMWARE_DIR)):
            sys.path.remove(path)


# --- config.py mirrors karmel.yaml ------------------------------------------------------------------
def test_firmware_config_matches_karmel_yaml(fw):
    cfg = load_config()
    c = fw.config
    for key, value in cfg.pins.items():
        assert getattr(c, key.upper()) == value, f"config.py {key.upper()} != karmel.yaml pins.{key}"
    d = cfg.drive
    assert c.WHEEL_RADIUS_M == d.wheel_radius_m
    assert c.WHEEL_SEPARATION_M == d.wheel_separation_m
    assert c.TICKS_PER_WHEEL_REV == d.ticks_per_wheel_rev == 2464
    assert c.MAX_WHEEL_SPEED_RAD_S == d.max_wheel_speed_rad_s
    assert c.DUTY_DEADBAND == d.duty_deadband
    assert c.WATCHDOG_MS == cfg.serial.watchdog_ms
    assert c.TELEMETRY_HZ == cfg.serial.telemetry_hz
    assert c.BATTERY_LOW_WARNING_V == cfg.battery.low_warning_v
    assert c.BATTERY_CUTOFF_V == cfg.battery.cutoff_v
    assert c.BATTERY_CELLS == cfg.battery.cells_series


def test_firmware_protocol_constants_match_host(fw):
    p = fw.protocol
    assert p.PROTOCOL_VERSION == host_protocol.PROTOCOL_VERSION
    assert p.PARAM_KEYS == host_protocol.PARAM_KEYS
    assert p.SEQ_MAX == host_protocol.SEQ_MAX
    assert (p.FLAG_WATCHDOG, p.FLAG_LOW_BATTERY, p.FLAG_RANGE_ERROR, p.FLAG_VELOCITY_MODE) == (1, 2, 4, 8)


# --- velocity.py: speed estimation --------------------------------------------------------------------
def test_one_wheel_revolution_per_second_is_two_pi(fw):
    est = fw.velocity.WheelSpeedEstimator(ticks_per_rev=2464)
    est.update(1000, 0.01)  # the first sample only primes the estimator
    assert est.rad_s == 0.0
    assert est.update(1000 + 2464, 1.0) == pytest.approx(2 * math.pi)  # 2464 ticks in 1 s
    assert est.update(1000 + 2464 + 246, 0.1) == pytest.approx(246 * 2 * math.pi / 2464 / 0.1)


def test_speed_estimate_quantization_and_filter(fw):
    raw = fw.velocity.WheelSpeedEstimator(2464, alpha=1.0)
    raw.update(0, 0.01)
    assert raw.update(1, 0.01) == pytest.approx(0.255, abs=1e-3)  # one tick in 10 ms

    smooth = fw.velocity.WheelSpeedEstimator(2464, alpha=0.5)
    smooth.update(0, 0.01)
    assert smooth.update(10, 0.01) == pytest.approx(0.5 * 10 * 2 * math.pi / 2464 / 0.01)
    smooth.reset(ticks=0)
    assert smooth.rad_s == 0.0 and smooth.update(0, 0.01) == 0.0


def test_negative_ticks_give_negative_speed(fw):
    est = fw.velocity.WheelSpeedEstimator(2464)
    est.update(0, 0.01)
    assert est.update(-50, 0.01) < 0


# --- velocity.py: PID --------------------------------------------------------------------------------
def test_feedforward_inverts_the_motor_model(fw):
    ff = fw.velocity.feedforward_duty
    assert ff(0.0, 1.0, 17.0, 0.12) == 0.0
    assert ff(17.0, 1.0, 17.0, 0.12) == pytest.approx(1.0)
    assert ff(8.5, 1.0, 17.0, 0.12) == pytest.approx(0.12 + 0.5 * 0.88)
    assert ff(-8.5, 1.0, 17.0, 0.12) == pytest.approx(-(0.12 + 0.5 * 0.88))
    assert ff(100.0, 1.0, 17.0, 0.12) == pytest.approx(1.0)  # beyond max speed: saturates
    assert ff(8.5, 0.5, 17.0, 0.12) == pytest.approx(0.5 * (0.12 + 0.5 * 0.88))


def test_proportional_and_integral_terms(fw):
    pid = fw.velocity.PID(kp=0.1, ki=0.5, kd=0.0)
    assert pid.update(setpoint=2.0, measured=1.0, dt_s=0.01) == pytest.approx(0.1 * 1.0 + 0.5 * 1.0 * 0.01)
    assert pid.update(setpoint=2.0, measured=1.0, dt_s=0.01) == pytest.approx(0.1 + 2 * 0.005)


def test_derivative_acts_on_measurement_so_setpoint_steps_do_not_kick(fw):
    pid = fw.velocity.PID(kp=0.0, ki=0.0, kd=0.1)
    pid.update(0.0, 0.0, 0.01)
    assert pid.update(10.0, 0.0, 0.01) == 0.0  # setpoint jumped, measurement didn't: no kick
    assert pid.update(10.0, 1.0, 0.01) == -1.0  # -kd * dmeas/dt = -10, clamped to the output range


def test_anti_windup_limits_integral_and_recovers_fast(fw):
    dt = 0.01
    pid = fw.velocity.PID(kp=0.05, ki=2.0, kd=0.0)
    # Wheel is blocked: measured stays 0 while we ask for 10 rad/s for 5 seconds.
    for _ in range(500):
        out = pid.update(10.0, 0.0, dt)
    assert out >= 0.9  # at (or just below) saturation
    assert pid.integral <= 0.5  # integration stopped once P + I reached the limit
    # A naive integrator would have accumulated ki * 10 * 5 s = 100 of "debt".
    # Released: measured speed overshoots to 12 -> the output must drop below 1 immediately.
    assert pid.update(10.0, 12.0, dt) < 1.0

    naive_integral = sum(2.0 * 10.0 * dt for _ in range(500))
    assert naive_integral == pytest.approx(100.0)


def test_integral_still_unwinds_while_saturated(fw):
    pid = fw.velocity.PID(kp=0.0, ki=1.0, kd=0.0, ff=1.0, max_speed_rad_s=17.0, deadband=0.12)
    pid.integral = 0.5
    # Output saturates high (ff ~ 1 + integral), but the error is negative: integration allowed.
    before = pid.integral
    pid.update(17.0, 18.0, 0.01)
    assert pid.integral < before


def test_anti_windup_integrates_up_to_the_limit(fw):
    """A step bigger than the headroom fills the headroom, it isn't refused (lessons 08.05, 08.07)."""
    pid = fw.velocity.PID(kp=0.1, ki=2.0, kd=0.0)
    # error 8.7: P = 0.87, one integration step = 2 * 8.7 * 0.02 = 0.348 > headroom 0.13
    assert pid.update(25.0, 16.3, 0.02) == pytest.approx(1.0)
    assert pid.integral == pytest.approx(0.13), "integrate until P + I sits exactly at +1"
    # mirror image below the lower limit
    neg = fw.velocity.PID(kp=0.1, ki=2.0, kd=0.0)
    assert neg.update(-25.0, -16.3, 0.02) == pytest.approx(-1.0)
    assert neg.integral == pytest.approx(-0.13)
    # P alone already beyond the limit: no headroom, and the integral must not move against the error
    held = fw.velocity.PID(kp=0.1, ki=2.0, kd=0.0)
    held.integral = 0.2
    held.update(25.0, 0.0, 0.02)
    assert held.integral == pytest.approx(0.2)


def test_unreachable_setpoint_uses_the_full_duty(fw):
    """Pure PID (ff = 0) asked for more than the motor can do must end at duty 1.0 and top speed.
    Refusing whole integration steps stalled this loop at 15.7 rad/s (P alone = 0.93 duty)."""
    dt, tau, max_speed, deadband = 0.01, 0.08, 17.0, 0.12
    pid = fw.velocity.PID(0.1, 2.0, 0.0, 0.0, max_speed, deadband)
    speed = 0.0
    for _ in range(300):
        duty = pid.update(25.0, speed, dt)
        target = math.copysign(max(abs(duty) - deadband, 0) / (1 - deadband) * max_speed, duty)
        speed += (target - speed) * (1 - math.exp(-dt / tau))
    assert duty == pytest.approx(1.0)
    assert speed == pytest.approx(max_speed, abs=0.05)
    # ...and it still recovers at once when the setpoint becomes reachable
    assert pid.update(6.0, speed, dt) < 0.0


def test_pid_holds_speed_on_first_order_motor_model(fw):
    """Closed loop against the same motor model the simulator uses: settles within 2 %."""
    dt, tau, max_speed, deadband = 0.01, 0.08, 17.0, 0.12
    pid = fw.velocity.PID(0.02, 0.3, 0.0, 1.0, max_speed, deadband)
    speed = 0.0
    motor_gain = 0.9  # the real motor is 10 % weaker than the model: the integrator fixes it
    for _ in range(300):
        duty = pid.update(8.0, speed, dt)
        target = math.copysign(max(abs(duty) - deadband, 0) / (1 - deadband) * max_speed, duty) * motor_gain
        speed += (target - speed) * (1 - math.exp(-dt / tau))
    assert speed == pytest.approx(8.0, rel=0.02)


# --- watchdog.py --------------------------------------------------------------------------------------
def test_command_watchdog_trips_once_after_timeout(fw):
    wd = fw.watchdog.CommandWatchdog(timeout_ms=300, now_ms=1000)
    assert wd.tripped  # stopped at boot until the first command
    assert wd.check(5000) is False
    wd.feed(1000)
    assert not wd.tripped
    assert wd.check(1299) is False
    assert wd.check(1300) is True
    assert wd.tripped
    assert wd.check(1400) is False  # already tripped: report once
    wd.feed(1400)
    assert wd.check(1600) is False and not wd.tripped


def test_command_watchdog_handles_tick_wraparound(fw):
    period = fw.utime.TICKS_PERIOD
    wd = fw.watchdog.CommandWatchdog(timeout_ms=300, now_ms=0)
    wd.feed(period - 100)  # 100 ms before ticks_ms() wraps to 0
    assert wd.check(150) is False  # 250 ms later
    assert wd.elapsed_ms(150) == 250
    assert wd.check(200) is True  # 300 ms later


def test_hardware_watchdog_is_optional(fw):
    off = fw.watchdog.HardwareWatchdog(0)
    assert not off.enabled
    off.feed()  # no-op
    on = fw.watchdog.HardwareWatchdog(2000)
    assert on.enabled
    on.feed()
    assert on._wdt.timeout == 2000 and on._wdt.feeds == 1


# --- motors.py ----------------------------------------------------------------------------------------
def test_slow_decay_drive_brake_pwm(fw):
    m = fw.motors.Motor(2, 3)
    m.set_duty(0.4)
    assert m._in1.duty_u16() == 65535
    assert m._in2.duty_u16() == 65535 - int(0.4 * 65535)  # IN2 low 40 % of the time = drive
    m.set_duty(-0.4)
    assert m._in1.duty_u16() == 65535 - int(0.4 * 65535)
    assert m._in2.duty_u16() == 65535
    m.set_duty(0.0)
    assert (m._in1.duty_u16(), m._in2.duty_u16()) == (65535, 65535)  # brake
    m.set_duty(5.0)
    assert m.duty == 1.0 and m._in2.duty_u16() == 0


def test_fast_decay_invert_brake_coast(fw):
    m = fw.motors.Motor(6, 7, invert=True, decay="fast")
    assert m._in1.freq() == 20_000
    m.set_duty(0.5)  # inverted: electrically reverse
    assert (m._in1.duty_u16(), m._in2.duty_u16()) == (0, int(0.5 * 65535))
    m.brake()
    assert (m._in1.duty_u16(), m._in2.duty_u16()) == (65535, 65535)
    m.coast()
    assert (m._in1.duty_u16(), m._in2.duty_u16()) == (0, 0)
    with pytest.raises(ValueError):
        fw.motors.Motor(6, 7, decay="medium")


def test_deadband_compensation(fw):
    comp = fw.motors.compensate_deadband
    assert comp(0.0, 0.12) == 0.0
    assert comp(0.1, 0.12) == pytest.approx(0.208)
    assert comp(-0.1, 0.12) == pytest.approx(-0.208)
    assert comp(1.0, 0.12) == pytest.approx(1.0)


# --- encoders.py ----------------------------------------------------------------------------------------
FORWARD_AB = [(0, 0), (1, 0), (1, 1), (0, 1)]  # A leads B


def _set_ab(fw, pin_a, a, b):
    fw.machine.Pin.levels[pin_a] = a
    fw.machine.Pin.levels[pin_a + 1] = b


def test_pio_program_fits_at_address_zero_and_matches_table(fw):
    program = fw.encoders.quadrature_program
    assert len(program.instructions) == 32  # padded: can only load at address 0
    labels = program.labels
    step_of = {labels["increment"]: +1, labels["decrement"]: -1, labels["update"]: 0}
    for index in range(16):
        op, cond, target = program.instructions[index]
        assert op == "jmp" and cond is None
        assert step_of[target] == fw.encoders.TRANSITION_TABLE[index], f"jump table entry {index:04b}"


def test_pio_encoder_counts_both_directions(fw):
    enc = fw.encoders.QuadratureEncoderPIO(4, 10)
    _set_ab(fw, 10, 0, 0)
    assert enc.count() == 0
    for k in range(1, 2464 + 1):  # one wheel revolution forward
        _set_ab(fw, 10, *FORWARD_AB[k % 4])
        enc.count()
    assert enc.count() == 2464
    for k in range(2464 + 100, 0, -1):  # 100 ticks further back than we started
        _set_ab(fw, 10, *FORWARD_AB[(k - 1) % 4])
        last = enc.count()
    assert last == -100  # the Y register went below zero: to_signed32 handles it
    enc.reset()
    assert enc.count() == 0


def test_pio_encoder_invert_and_ignores_double_steps(fw):
    enc = fw.encoders.QuadratureEncoderPIO(5, 12, invert=True)
    for k in range(1, 9):
        _set_ab(fw, 12, *FORWARD_AB[k % 4])
        enc.count()
    assert enc.count() == -8
    _set_ab(fw, 12, 1, 1)  # from (0,0) straight to (1,1): impossible jump, ignored
    assert enc.count() == -8


def test_irq_encoder_uses_the_same_table(fw):
    enc = fw.encoders.QuadratureEncoderIRQ(20, 21)
    pin_a = enc._pin_a
    for k in range(1, 41):
        a, b = FORWARD_AB[k % 4]
        fw.machine.Pin.levels[20], fw.machine.Pin.levels[21] = a, b
        pin_a.irq_handler(pin_a)
    assert enc.count() == 40
    enc.reset()
    assert enc.count() == 0


def test_to_signed32(fw):
    assert fw.encoders.to_signed32(0xFFFFFFFF) == -1
    assert fw.encoders.to_signed32(5) == 5
    assert fw.encoders.to_signed32(0x80000000) == -(2**31)


# --- sensors.py and vl53l1x.py --------------------------------------------------------------------------
def test_ultrasonic_echo_time(fw):
    assert fw.sensors.echo_time_to_mm(5831) == 1000  # 1 m: sound needs ~5.83 ms there and back
    sonar = fw.sensors.US100(14, 15)
    fw.machine.PULSE_RESULT_US = -2
    assert sonar.read_mm() is None  # timeout: no echo
    fw.machine.PULSE_RESULT_US = 2915
    assert sonar.read_mm() == 499
    fw.machine.PULSE_RESULT_US = 60  # ~1 cm: closer than the sensor can measure
    assert sonar.read_mm() is None
    fw.machine.PULSE_RESULT_US = -2


def test_battery_divider_math(fw):
    v_pin = 12.6 * 22 / 122
    assert fw.sensors.divider_voltage(v_pin / 3.3 * 65535) == pytest.approx(12.6)
    battery = fw.sensors.DividerBattery()
    battery._adc.value_u16 = int(v_pin / 3.3 * 65535)
    assert battery.read_mv() == pytest.approx(12600, abs=2)


class FakeRegisters:
    """An I2C device with 16-bit registers in a dict (big-endian, like INA219/VL53L1X)."""

    def __init__(self, width=2, values=None):
        self.width = width
        self.values = dict(values or {})
        self.writes = []

    def readfrom_mem(self, register, n):
        out = bytearray()
        while len(out) < n:
            value = self.values.get(register, 0)
            out += value.to_bytes(self.width, "big") if n >= self.width else bytes([value & 0xFF])
            register += self.width
        return bytes(out[:n])

    def writeto_mem(self, register, data):
        self.writes.append((register, bytes(data)))


def test_ina219_voltage_and_current(fw):
    i2c = fw.machine.I2C(0)
    # bus voltage 11.84 V -> raw = (11840 mV / 4 mV) << 3 ; shunt 0.123 V -> 12300 x 10 uV
    i2c.devices[0x40] = FakeRegisters(values={0x02: (11840 // 4) << 3, 0x01: 12300})
    ina = fw.sensors.INA219(i2c)
    assert ina.read_mv() == 11840
    assert ina.current_a() == pytest.approx(1.23)
    i2c.devices[0x40].values[0x01] = 0x10000 - 500  # -5 mV: current flowing back (charging)
    assert ina.current_a() == pytest.approx(-0.05)
    with pytest.raises(OSError):
        fw.sensors.INA219(fw.machine.I2C(0))  # no device on the bus


class FakeVL53L1X:
    def __init__(self):
        self.ready = 1
        self.result = bytearray(17)
        self.writes = []

    def readfrom_mem(self, register, n):
        if register == 0x010F:
            return bytes([0xEA, 0xCC])
        if register == 0x0022:
            return bytes([0x00, 0x05])
        if register == 0x0030:
            return bytes([0x01])  # interrupt active high
        if register == 0x0031:
            return bytes([self.ready])
        if register == 0x0089:
            return bytes(self.result[:n])
        return bytes(n)

    def writeto_mem(self, register, data):
        self.writes.append((register, bytes(data)))


def test_vl53l1x_init_and_read(fw):
    i2c = fw.machine.I2C(0)
    dev = FakeVL53L1X()
    i2c.devices[0x29] = dev
    tof = fw.vl53l1x.VL53L1X(i2c)
    config_write = [w for w in dev.writes if w[0] == 0x2D]
    assert len(config_write) == 1 and len(config_write[0][1]) == 0x87 - 0x2D + 1 == 91
    assert config_write[0][1][-1] == 0x40  # starts ranging
    assert (0x001E, bytes([0x00, 20])) in dev.writes  # offset x 4
    dev.result[0], dev.result[13], dev.result[14] = 9, 0x03, 0x2C  # valid, 812 mm
    assert tof.read_mm() == 812
    assert dev.writes[-1] == (0x0086, b"\x01")  # interrupt cleared for the next measurement
    dev.result[0] = 4  # signal fail
    assert tof.read_mm() is None and tof.last_status == 4
    dev.ready = 0
    assert tof.read_mm(timeout_ms=0) is None


# --- main.py: the Robot command handler and control step ---------------------------------------------------
class FakeMotors:
    def __init__(self):
        self.duty = (0.0, 0.0)
        self.braked = 0

    def set_duty(self, left, right):
        self.duty = (left, right)

    def brake(self):
        self.duty = (0.0, 0.0)
        self.braked += 1


class FakeEncoders:
    def __init__(self):
        self.left = 0
        self.right = 0

    def counts(self):
        return self.left, self.right

    def reset(self):
        self.left = self.right = 0


@pytest.fixture
def robot(fw):
    motors, encoders = FakeMotors(), FakeEncoders()
    r = fw.main.Robot(motors, encoders, now_ms=0)
    return SimpleNamespace(robot=r, motors=motors, encoders=encoders, p=fw.protocol)


def _line(payload):
    return host_protocol.frame(payload)


def test_hello_and_acks(robot):
    r = robot.robot
    reply = host_protocol.decode(r.handle_line(_line("H 1"), 0))
    assert reply == host_protocol.HelloReply("0.1.0", 1)
    assert host_protocol.decode(r.handle_line(_line("M 2 500 -250"), 10)) == host_protocol.Ack(2, True)
    assert robot.motors.duty == (0.5, -0.25)


def test_bad_lines_are_dropped_and_bad_args_rejected(robot):
    r = robot.robot
    assert r.handle_line("M 2 500 -250*00\n", 0) is None  # wrong checksum: silent
    assert r.handle_line("hello\n", 0) is None
    assert r.bad_lines == 2
    assert robot.motors.duty == (0.0, 0.0)
    assert host_protocol.decode(r.handle_line(_line("M 3 1500 0"), 0)) == host_protocol.Ack(3, False, "out_of_range")
    assert host_protocol.decode(r.handle_line(_line("P 4 gain 1"), 0)) == host_protocol.Ack(4, False, "unknown_param")
    assert host_protocol.decode(r.handle_line(_line("P 5 kp 50"), 0)) == host_protocol.Ack(5, False, "out_of_range")
    assert host_protocol.decode(r.handle_line(_line("X 6"), 0)) == host_protocol.Ack(6, False, "unknown_command")


def test_watchdog_stops_motors_and_sets_flag(robot):
    r, motors = robot.robot, robot.motors
    r.handle_line(_line("M 1 400 400"), 1000)
    r.control_step(1000)
    r.control_step(1290)
    assert motors.duty == (0.4, 0.4)
    assert not r.flags() & 1
    r.control_step(1300)  # 300 ms without a drive command
    assert motors.duty == (0.0, 0.0) and motors.braked == 1
    assert r.flags() & 1
    r.handle_line(_line("M 2 400 400"), 1310)
    assert not r.flags() & 1 and motors.duty == (0.4, 0.4)


def test_velocity_mode_runs_pid_and_zero_setpoint_brakes(robot):
    r, motors, enc = robot.robot, robot.motors, robot.encoders
    r.handle_line(_line("V 1 5000 -5000"), 0)
    assert r.flags() & 8
    r.control_step(0)
    r.control_step(10)
    left, right = motors.duty
    assert left > 0.3 and right < -0.3  # feedforward alone is ~0.38 for 5 rad/s
    r.handle_line(_line("V 2 50000 0"), 15)  # 50 rad/s is clamped to 17
    assert r.setpoints == [17.0, 0.0]
    enc.left += 100
    r.control_step(20)
    assert motors.duty[1] == 0.0
    assert host_protocol.decode(r.handle_line(_line("S 3"), 25)) == host_protocol.Ack(3, True)
    assert not r.flags() & 8 and motors.duty == (0.0, 0.0)


def test_params_and_reset(robot):
    r, enc = robot.robot, robot.encoders
    for payload in ("P 1 kp 0.05", "P 2 ki 0.4", "P 3 kd 0.001", "P 4 ff 0.8", "P 5 watchdog_ms 500", "P 6 telemetry_hz 20"):
        assert host_protocol.decode(r.handle_line(_line(payload), 0)).ok
    assert r.pids[0].kp == 0.05 and r.pids[1].ki == 0.4 and r.pids[0].ff == 0.8
    assert r.watchdog.timeout_ms == 500 and r.telemetry_hz == 20
    assert host_protocol.decode(r.handle_line(_line("P 7 watchdog_ms 10"), 0)).reason == "out_of_range"
    enc.left, enc.right = 1234, -99
    r.control_step(0)
    assert r.ticks == [1234, -99]
    r.handle_line(_line("R 8"), 5)
    assert r.ticks == [0, 0] and enc.counts() == (0, 0)


def test_telemetry_line_decodes_on_the_host(robot, fw):
    r, enc = robot.robot, robot.encoders
    r.control_step(0)
    enc.left, enc.right = 25, -25
    r.control_step(10)
    r.battery_mv, r.range_mm = 10400, None
    t = host_protocol.decode(r.telemetry_line(10))
    assert isinstance(t, host_protocol.Telemetry)
    assert (t.left_ticks, t.right_ticks, t.battery_mv, t.range_mm) == (25, -25, 10400, -1)
    assert t.left_mrad_s == -t.right_mrad_s > 0
    assert t.flags == 1 | 2  # watchdog (no command yet) + low battery (10.4 V < 10.5 V)


def test_uptime_survives_tick_wraparound(fw):
    period = fw.utime.TICKS_PERIOD
    r = fw.main.Robot(FakeMotors(), FakeEncoders(), now_ms=period - 500)
    assert r.uptime_ms(period - 400) == 100
    assert r.uptime_ms(600) == 1100  # ticks wrapped, uptime keeps counting


class FailingSensor:
    def read_mm(self):
        raise OSError(5, "EIO")

    def read_mv(self):
        raise OSError(5, "EIO")


def test_sensor_failures_set_flags_not_exceptions(fw):
    r = fw.main.Robot(FakeMotors(), FakeEncoders(), 0, range_sensor=FailingSensor(), battery=FailingSensor())
    r.update_sensors()
    assert r.flags() & 4
    assert r.range_mm is None and r.battery_mv is None
