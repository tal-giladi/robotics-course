# sensors.py - distance and battery sensors: US-100 ultrasonic, INA219, ADC voltage divider.
#
# Lessons: 01.13 (ultrasonic + ToF, stop before obstacles), 01.14 (battery voltage).
# The VL53L1X time-of-flight driver lives in vl53l1x.py.
#
# Every sensor class has a small read method that returns SI-ish integers or floats, or
# None when there is no valid reading. main.py turns None into -1 on the wire.

from machine import ADC, Pin, time_pulse_us

try:                                    # MicroPython
    from time import sleep_us
except ImportError:                     # CPython (tests)
    from utime import sleep_us

import config


# --- US-100 ultrasonic, pulse mode --------------------------------------------------------------
def echo_time_to_mm(pulse_us, speed_of_sound_m_s=config.SPEED_OF_SOUND_M_S):
    """Convert an echo pulse length to a distance.

    The pulse lasts as long as the sound took to reach the obstacle AND come back, so the
    distance is half of (time x speed of sound):

        mm = pulse_us * 1e-6 s * 343 m/s / 2 * 1000 mm/m  ~=  pulse_us * 0.1715
    """
    return int(pulse_us * speed_of_sound_m_s / 2000.0)


class US100:
    """US-100 in pulse mode (remove the jumper on the back; with the jumper it speaks UART).

    Powered at 3.3 V its echo output is 3.3 V, safe for the Pico. (An HC-SR04 needs 5 V and
    its 5 V echo needs a resistor divider - see lesson 01.13.)

        sonar = US100(trig_pin=14, echo_pin=15)
        sonar.read_mm()       # e.g. 523, or None if nothing echoed back in time
    """

    MIN_MM = 20                         # closer than ~2 cm the echo overlaps the ping

    def __init__(self, trig_pin, echo_pin, timeout_us=config.US100_TIMEOUT_US,
                 max_mm=config.RANGE_MAX_MM):
        self._trig = Pin(trig_pin, Pin.OUT, value=0)
        self._echo = Pin(echo_pin, Pin.IN)
        self.timeout_us = timeout_us
        self.max_mm = max_mm

    def read_mm(self):
        # A 10 microsecond HIGH pulse on TRIG starts one measurement.
        self._trig.value(0)
        sleep_us(2)
        self._trig.value(1)
        sleep_us(10)
        self._trig.value(0)
        # Wait for ECHO to go high, then measure how long it stays high.
        # time_pulse_us returns -2 if the pulse never started, -1 if it never ended.
        # NOTE: this BLOCKS for up to timeout_us (30 ms). In main.py that is why the sensor
        # task runs only 10 times per second - lesson 03.04 discusses the cost.
        pulse_us = time_pulse_us(self._echo, 1, self.timeout_us)
        if pulse_us < 0:
            return None
        mm = echo_time_to_mm(pulse_us)
        if mm < self.MIN_MM or mm > self.max_mm:
            return None
        return mm


# --- INA219 current / voltage monitor on I2C ------------------------------------------------------
class INA219:
    """Minimal INA219 driver: bus voltage and current, using the chip's power-on defaults.

    The INA219 measures two things:
      * the BUS voltage (battery +, relative to ground), 0..32 V in 4 mV steps;
      * the tiny SHUNT voltage across a 0.1 ohm resistor the battery current flows through,
        in 10 microvolt steps. Ohm's law gives the current: I = V_shunt / R_shunt.

    The power-on configuration (32 V range, +/-320 mV shunt range, continuous conversion) is
    what we want, so this driver never writes to the chip - it only reads two registers.
    With a 0.1 ohm shunt, +/-320 mV means currents up to +/-3.2 A.
    """

    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02

    def __init__(self, i2c, address=config.INA219_ADDRESS, shunt_ohms=config.INA219_SHUNT_OHMS):
        self.i2c = i2c
        self.address = address
        self.shunt_ohms = shunt_ohms
        if address not in i2c.scan():
            raise OSError("INA219 not found at I2C address 0x%02X" % address)

    def _read_register(self, register):
        data = self.i2c.readfrom_mem(self.address, register, 2)   # big-endian 16-bit
        return (data[0] << 8) | data[1]

    def bus_voltage_v(self):
        raw = self._read_register(self.REG_BUS_VOLTAGE)
        # Bits 15..3 hold the voltage in 4 mV steps; bits 2..0 are status flags.
        return (raw >> 3) * 0.004

    def shunt_voltage_v(self):
        raw = self._read_register(self.REG_SHUNT_VOLTAGE)
        if raw & 0x8000:                 # two's complement: negative when current flows back
            raw -= 0x10000
        return raw * 0.00001             # 10 microvolt steps

    def current_a(self):
        return self.shunt_voltage_v() / self.shunt_ohms

    def read_mv(self):
        return int(self.bus_voltage_v() * 1000)


# --- battery voltage through a resistor divider into an ADC pin ------------------------------------
def divider_voltage(adc_u16, r_top=config.BATTERY_DIVIDER_TOP_OHMS,
                    r_bottom=config.BATTERY_DIVIDER_BOTTOM_OHMS, v_ref=config.ADC_REFERENCE_V):
    """Battery voltage from a 16-bit ADC reading of the divider's middle point.

    The ADC can only measure 0..3.3 V, a full 3S pack is 12.6 V. The divider scales it down:

        V_pin = V_batt * r_bottom / (r_top + r_bottom)     100k/22k: 12.6 V -> 2.27 V

    So V_batt = V_pin * (r_top + r_bottom) / r_bottom.
    """
    v_pin = adc_u16 * v_ref / 65535
    return v_pin * (r_top + r_bottom) / r_bottom


class DividerBattery:
    """Cheapest battery monitor: two resistors and an ADC pin. Accuracy ~ +/-2-3 %.

    SAFETY: the divider must be wired exactly as in lesson 01.14 BEFORE connecting the pack -
    the full battery voltage on a GPIO pin destroys the Pico.
    """

    def __init__(self, adc_pin=config.BATTERY_ADC, samples=8):
        self._adc = ADC(Pin(adc_pin))
        self.samples = samples

    def read_mv(self):
        total = 0
        for _ in range(self.samples):   # averaging a few readings reduces ADC noise
            total += self._adc.read_u16()
        return int(divider_voltage(total / self.samples) * 1000)
