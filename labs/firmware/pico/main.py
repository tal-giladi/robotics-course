# main.py - karmel robot firmware for the Raspberry Pi Pico 2 (MicroPython).
#
# Lessons: 01.10 (protocol + watchdog), 03.04 (loops, timing, asyncio), 08.12 (the velocity
# loop on the microcontroller). MicroPython runs main.py automatically at power-up.
#
# What it does
#   * reads protocol v1 commands, one line at a time, from the USB serial port (sys.stdin);
#   * runs the control loop at config.CONTROL_HZ: read encoders, estimate wheel speeds, run the
#     velocity PID when in velocity mode, stop the motors if the command watchdog trips;
#   * sends telemetry lines at telemetry_hz (50 Hz by default) to sys.stdout;
#   * reads the range sensor and the battery at config.SENSOR_HZ;
#   * blinks the status LED: slow = idle / watchdog stopped, fast = driving.
#
# Structure
#   class Robot  - all the decisions, no timing and no I/O of its own. Every method takes
#                  "now" as an argument, so tests (labs/python/tests/test_firmware_logic.py)
#                  can drive it with a fake clock on a PC.
#   async tasks  - the timing: each task wakes up at its own rate and calls into Robot.
#   run()        - builds the hardware objects and starts the tasks.
#
# SAFETY: this program makes the wheels turn when the host sends M or V. Test with the wheels
# off the ground first. It stops them if commands stop arriving for watchdog_ms (300 ms).

import sys

import config
import protocol
from velocity import PID, WheelSpeedEstimator
from watchdog import CommandWatchdog, HardwareWatchdog

try:                                    # MicroPython
    from time import ticks_add, ticks_diff, ticks_ms
except ImportError:                     # CPython (tests)
    from utime import ticks_add, ticks_diff, ticks_ms

MODE_STOPPED = 0
MODE_DUTY = 1                           # open loop: last command was M
MODE_VELOCITY = 2                       # closed loop: last command was V


class Robot:
    """The firmware's brain. Hardware objects are passed in, so tests can pass fakes.

    motors   - has set_duty(left, right) and brake()         (motors.DriveMotors)
    encoders - has counts() -> (left, right) and reset()      (encoders.WheelEncoders)
    range_sensor, battery - have read_mm() / read_mv() returning an int or None; may be None
    """

    def __init__(self, motors, encoders, now_ms, range_sensor=None, battery=None,
                 range_sensor_failed=False):
        self.motors = motors
        self.encoders = encoders
        self.range_sensor = range_sensor
        self.battery = battery

        self.mode = MODE_STOPPED
        self.setpoints = [0.0, 0.0]         # rad/s, used in velocity mode
        self.estimators = [WheelSpeedEstimator(config.TICKS_PER_WHEEL_REV, config.VELOCITY_FILTER_ALPHA)
                           for _ in range(2)]
        self.pids = [PID(config.VELOCITY_KP, config.VELOCITY_KI, config.VELOCITY_KD, config.VELOCITY_FF,
                         config.MAX_WHEEL_SPEED_RAD_S, config.DUTY_DEADBAND) for _ in range(2)]
        self.watchdog = CommandWatchdog(config.WATCHDOG_MS, now_ms)
        self.telemetry_hz = config.TELEMETRY_HZ

        self.ticks = [0, 0]
        self.range_mm = None
        self.range_error = range_sensor_failed
        self.battery_mv = None
        self.bad_lines = 0                  # lines dropped for bad framing/checksum

        self._last_control_ms = None
        self._clock_last_ms = now_ms
        self._uptime_ms = 0

    # --- time ---------------------------------------------------------------------------------
    def uptime_ms(self, now_ms):
        """Milliseconds since boot that never wrap around (ticks_ms() does, after ~12 days)."""
        self._uptime_ms += ticks_diff(now_ms, self._clock_last_ms)
        self._clock_last_ms = now_ms
        return self._uptime_ms

    # --- commands -----------------------------------------------------------------------------
    def handle_line(self, line, now_ms):
        """Process one received line. Returns the reply line (str) or None."""
        try:
            payload = protocol.unframe(line)
        except protocol.ProtocolError:
            # Corrupted or not a protocol line: drop it silently (its seq can't be trusted).
            self.bad_lines += 1
            return None
        try:
            kind, seq, args = protocol.parse_command(payload)
        except protocol.ProtocolError as error:
            if error.seq is None:
                self.bad_lines += 1
                return None
            return protocol.format_error(error.seq, error.reason)
        return self.execute(kind, seq, args, now_ms)

    def execute(self, kind, seq, args, now_ms):
        if kind == "H":
            return protocol.format_hello_reply(config.FIRMWARE_VERSION)

        if kind == "M":
            # SAFETY: open-loop duty - the wheels start turning right here.
            self.watchdog.feed(now_ms)
            self.mode = MODE_DUTY
            self.motors.set_duty(args[0] / protocol.DUTY_SCALE, args[1] / protocol.DUTY_SCALE)
            return protocol.format_ack(seq)

        if kind == "V":
            # SAFETY: velocity setpoints - the control loop starts the wheels on its next step.
            self.watchdog.feed(now_ms)
            if self.mode != MODE_VELOCITY:
                for pid in self.pids:
                    pid.reset()             # don't carry an old integral into a new run
            self.mode = MODE_VELOCITY
            limit = config.MAX_WHEEL_SPEED_RAD_S
            for i in range(2):
                rad_s = args[i] / 1000.0
                self.setpoints[i] = max(-limit, min(limit, rad_s))  # clamp to what motors can do
            return protocol.format_ack(seq)

        if kind == "S":
            self.stop()
            return protocol.format_ack(seq)

        if kind == "R":
            self.encoders.reset()
            self.ticks = [0, 0]
            for estimator in self.estimators:
                estimator.reset(ticks=0)
            return protocol.format_ack(seq)

        if kind == "P":
            return self.set_param(seq, args[0], args[1])

        return protocol.format_error(seq, protocol.ERR_UNKNOWN_COMMAND)

    def set_param(self, seq, key, value):
        # Allowed ranges: wide enough to experiment, narrow enough that a typo can't make the
        # robot unsafe (e.g. a 1 hour watchdog) or flood the serial link.
        if key in ("kp", "ki", "kd"):
            if not 0.0 <= value <= 10.0:
                return protocol.format_error(seq, protocol.ERR_OUT_OF_RANGE)
            for pid in self.pids:
                setattr(pid, key, value)
        elif key == "ff":
            if not 0.0 <= value <= 2.0:
                return protocol.format_error(seq, protocol.ERR_OUT_OF_RANGE)
            for pid in self.pids:
                pid.ff = value
        elif key == "watchdog_ms":
            if not 50 <= value <= 5000:
                return protocol.format_error(seq, protocol.ERR_OUT_OF_RANGE)
            self.watchdog.timeout_ms = int(value)
        elif key == "telemetry_hz":
            if not 1 <= value <= 200:
                return protocol.format_error(seq, protocol.ERR_OUT_OF_RANGE)
            self.telemetry_hz = int(value)
        else:
            return protocol.format_error(seq, protocol.ERR_UNKNOWN_PARAM)
        return protocol.format_ack(seq)

    def stop(self):
        """Brake both motors and leave any drive mode."""
        self.mode = MODE_STOPPED
        self.setpoints = [0.0, 0.0]
        for pid in self.pids:
            pid.reset()
        self.motors.brake()

    # --- control loop -------------------------------------------------------------------------
    def control_step(self, now_ms):
        """Call at config.CONTROL_HZ: encoders -> speed estimate -> watchdog -> PID -> motors."""
        dt_s = 0.0
        if self._last_control_ms is not None:
            dt_s = ticks_diff(now_ms, self._last_control_ms) / 1000.0
        self._last_control_ms = now_ms

        left, right = self.encoders.counts()
        self.ticks = [left, right]
        for i in range(2):
            self.estimators[i].update(self.ticks[i], dt_s)

        if self.watchdog.check(now_ms):
            self.stop()                     # host went silent: stop NOW
            return

        if self.mode == MODE_VELOCITY and dt_s > 0:
            duties = [0.0, 0.0]
            for i in range(2):
                if self.setpoints[i] == 0.0:
                    # Setpoint zero means "stand still": no PID output that could creep or hum.
                    self.pids[i].reset()
                else:
                    duties[i] = self.pids[i].update(self.setpoints[i], self.estimators[i].rad_s, dt_s)
            # SAFETY: the PID output goes to the motors here.
            self.motors.set_duty(duties[0], duties[1])

    # --- sensors ------------------------------------------------------------------------------
    def update_sensors(self):
        """Call at config.SENSOR_HZ. Sensor failures set flags; they never crash the firmware."""
        if self.range_sensor is not None:
            try:
                self.range_mm = self.range_sensor.read_mm()
                self.range_error = False
            except OSError:                 # I2C error, sensor unplugged...
                self.range_mm = None
                self.range_error = True
        if self.battery is not None:
            try:
                self.battery_mv = self.battery.read_mv()
            except OSError:
                self.battery_mv = None

    # --- telemetry ----------------------------------------------------------------------------
    def flags(self):
        value = 0
        if self.watchdog.tripped:
            value |= protocol.FLAG_WATCHDOG
        if self.battery_mv is not None and self.battery_mv < config.BATTERY_LOW_WARNING_V * 1000:
            value |= protocol.FLAG_LOW_BATTERY
        if self.range_error:
            value |= protocol.FLAG_RANGE_ERROR
        if self.mode == MODE_VELOCITY:
            value |= protocol.FLAG_VELOCITY_MODE
        return value

    def telemetry_line(self, now_ms):
        return protocol.format_telemetry(
            self.uptime_ms(now_ms),
            self.ticks[0], self.ticks[1],
            int(round(self.estimators[0].rad_s * 1000)), int(round(self.estimators[1].rad_s * 1000)),
            -1 if self.battery_mv is None else self.battery_mv,
            -1 if self.range_mm is None else self.range_mm,
            self.flags())

    @property
    def driving(self):
        return self.mode != MODE_STOPPED and not self.watchdog.tripped


# ================================================================================================
# Everything below runs only on the Pico.
# ================================================================================================
class ToFRangeReader:
    """Adapts the VL53L1X (which measures on its own schedule) to read_mm(): returns the most
    recent measurement, re-reading only when the sensor says a new one is ready."""

    def __init__(self, tof):
        self.tof = tof
        self.last_mm = None

    def read_mm(self):
        if self.tof.data_ready():
            self.last_mm = self.tof.read_mm()
        return self.last_mm


def build_robot():
    """Create the hardware objects from config.py. Missing sensors are reported, not fatal."""
    from machine import I2C, Pin

    from encoders import WheelEncoders
    from motors import DriveMotors

    motors = DriveMotors()
    motors.brake()
    encoders = WheelEncoders()

    i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)

    range_sensor = None
    range_failed = False
    try:
        if config.RANGE_SENSOR == "vl53l1x":
            from vl53l1x import VL53L1X
            range_sensor = ToFRangeReader(VL53L1X(i2c))
        elif config.RANGE_SENSOR == "us100":
            from sensors import US100
            range_sensor = US100(config.ULTRASONIC_TRIG, config.ULTRASONIC_ECHO)
    except OSError:
        range_failed = True                 # telemetry flag 4 tells the host

    battery = None
    try:
        if config.BATTERY_SENSOR == "ina219":
            from sensors import INA219
            battery = INA219(i2c)
        elif config.BATTERY_SENSOR == "adc":
            from sensors import DividerBattery
            battery = DividerBattery()
    except OSError:
        battery = None                      # battery_mv stays -1

    return Robot(motors, encoders, ticks_ms(), range_sensor, battery, range_failed)


async def command_task(robot):
    """Read characters from USB serial without blocking the other tasks."""
    import asyncio
    import select

    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    chars = []
    while True:
        # poll(0) = "is there a character?" without waiting. Read everything available.
        while poller.poll(0):
            ch = sys.stdin.read(1)
            if ch == "\n":
                reply = robot.handle_line("".join(chars), ticks_ms())
                chars = []
                if reply:
                    sys.stdout.write(reply)
            elif ch != "\r":
                chars.append(ch)
                if len(chars) > config.MAX_LINE_LENGTH:
                    chars = []              # garbage without newlines: start over
                    robot.bad_lines += 1
        await asyncio.sleep_ms(2)


async def periodic(period_ms_fn, step):
    """Call step(now) every period_ms_fn() milliseconds, on a fixed schedule.

    We compute the NEXT deadline from the previous deadline, not from "now", so small delays
    don't accumulate into drift. If we fall far behind, we resynchronise instead of running
    many steps in a burst.
    """
    import asyncio

    deadline = ticks_ms()
    while True:
        step(ticks_ms())
        deadline = ticks_add(deadline, period_ms_fn())
        delay = ticks_diff(deadline, ticks_ms())
        if delay < 0:
            deadline = ticks_ms()
            delay = 0
        await asyncio.sleep_ms(delay)


async def led_task(robot):
    import asyncio
    from machine import Pin

    led = Pin(config.STATUS_LED, Pin.OUT)
    while True:
        led.toggle()
        await asyncio.sleep_ms(100 if robot.driving else 500)


async def main_async(robot, hardware_watchdog):
    import asyncio

    control_period_ms = 1000 // config.CONTROL_HZ

    def control(now):
        robot.control_step(now)
        hardware_watchdog.feed()            # proves the control loop is still running

    def telemetry(now):
        sys.stdout.write(robot.telemetry_line(now))

    asyncio.create_task(command_task(robot))
    asyncio.create_task(periodic(lambda: control_period_ms, control))
    asyncio.create_task(periodic(lambda: 1000 // robot.telemetry_hz, telemetry))
    asyncio.create_task(periodic(lambda: 1000 // config.SENSOR_HZ, lambda now: robot.update_sensors()))
    await led_task(robot)


def run():
    import asyncio

    robot = build_robot()
    hardware_watchdog = HardwareWatchdog(config.HARDWARE_WATCHDOG_MS)
    try:
        asyncio.run(main_async(robot, hardware_watchdog))
    finally:
        # Ctrl-C from mpremote, or an unexpected error: never leave the motors running.
        robot.motors.brake()


if __name__ == "__main__":
    run()
