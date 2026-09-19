# vl53l1x.py - minimal driver for the ST VL53L1X time-of-flight distance sensor (I2C).
#
# Lesson 01.13. Used by main.py and examples/05_tof.py.
#
# ATTRIBUTION AND LICENSE
# -----------------------
# Adapted from "vl53l1x_pico" by Lee Halls, https://github.com/drakxtwo/vl53l1x_pico
# (MIT License, text below), which was itself taken from the OpenMV project's MicroPython
# driver https://github.com/openmv/openmv/blob/master/scripts/libraries/vl53l1x.py
# (OpenMV: most of the repository, including this script, is MIT licensed).
# The 91-byte default configuration below originates in ST's VL53L1X Ultra Lite Driver
# (STSW-IMG009, offered under the BSD 3-Clause license).
# Changes for the course: comments, range-status check, data-ready check, interrupt clear,
# timeout, time.sleep_ms instead of machine.lightsleep.
#
#   MIT License
#
#   Copyright (c) 2021 Lee Halls
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.
#
# How the sensor works
# --------------------
# It fires an invisible infrared laser pulse and times the reflection with an array of
# single-photon detectors (SPADs): distance = time of flight x speed of light / 2. ST does not
# publish the register map; instead it ships a C "Ultra Lite Driver" whose initialisation
# writes a block of magic values starting at register 0x2D. We write the same block, and the
# last value in it (0x40 into register 0x87) starts continuous ranging - about 10 readings
# per second with the default 100 ms timing budget. Registers have 16-bit addresses.

try:                                    # MicroPython
    from time import sleep_ms, ticks_ms, ticks_diff
except ImportError:                     # CPython (tests)
    from utime import sleep_ms, ticks_ms, ticks_diff

DEFAULT_ADDRESS = 0x29
MODEL_ID = 0xEACC

REG_SOFT_RESET = 0x0000
REG_OUTER_OFFSET_MM = 0x0022           # MM_CONFIG__OUTER_OFFSET_MM
REG_PART_TO_PART_OFFSET_MM = 0x001E    # ALGO__PART_TO_PART_RANGE_OFFSET_MM
REG_GPIO_HV_MUX_CTRL = 0x0030          # interrupt polarity
REG_GPIO_TIO_HV_STATUS = 0x0031        # "new measurement ready" bit
REG_INTERRUPT_CLEAR = 0x0086
REG_RESULT_RANGE_STATUS = 0x0089       # start of the 17-byte result block
REG_MODEL_ID = 0x010F

# Range status codes (lowest 5 bits of RESULT__RANGE_STATUS) that mean "this distance is good".
STATUS_RANGE_VALID = 9
STATUS_RANGE_VALID_MIN_CLIPPED = 8     # valid, but the target is closer than the minimum range

# Default configuration for registers 0x2D..0x87 (see ATTRIBUTION above).
DEFAULT_CONFIGURATION = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,  # 0x2D-0x34
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,  # 0x35-0x3C
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,  # 0x3D-0x44
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,  # 0x45-0x4C
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,  # 0x4D-0x54
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,  # 0x55-0x5C
    0x00, 0x01, 0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01,  # 0x5D-0x64
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,  # 0x65-0x6C
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,  # 0x6D-0x74
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,  # 0x75-0x7C
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,  # 0x7D-0x84
    0x01, 0x01, 0x40,                                # 0x85-0x87: clear interrupt, start ranging
])


class VL53L1X:
    """
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
        tof = VL53L1X(i2c)
        tof.read_mm()      # e.g. 812, or None if there is no valid target
    """

    def __init__(self, i2c, address=DEFAULT_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.last_status = None
        self._soft_reset()
        if self._read_u16(REG_MODEL_ID) != MODEL_ID:
            raise OSError("VL53L1X not found at 0x%02X - check wiring and power" % address)
        self.i2c.writeto_mem(self.address, 0x2D, DEFAULT_CONFIGURATION, addrsize=16)
        # The ST driver applies this offset correction once ranging starts.
        self._write_u16(REG_PART_TO_PART_OFFSET_MM, self._read_u16(REG_OUTER_OFFSET_MM) * 4)
        sleep_ms(200)                   # let the first measurements complete

    # --- register access (16-bit register addresses, big-endian values) ----------------------
    def _write_u8(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes([value]), addrsize=16)

    def _write_u16(self, register, value):
        self.i2c.writeto_mem(self.address, register,
                             bytes([(value >> 8) & 0xFF, value & 0xFF]), addrsize=16)

    def _read_u8(self, register):
        return self.i2c.readfrom_mem(self.address, register, 1, addrsize=16)[0]

    def _read_u16(self, register):
        data = self.i2c.readfrom_mem(self.address, register, 2, addrsize=16)
        return (data[0] << 8) | data[1]

    def _soft_reset(self):
        self._write_u8(REG_SOFT_RESET, 0x00)
        sleep_ms(100)
        self._write_u8(REG_SOFT_RESET, 0x01)
        sleep_ms(2)

    # --- measurements ------------------------------------------------------------------------
    def data_ready(self):
        """True when a new measurement is waiting."""
        active_high = not (self._read_u8(REG_GPIO_HV_MUX_CTRL) & 0x10)
        ready_bit = self._read_u8(REG_GPIO_TIO_HV_STATUS) & 0x01
        return ready_bit == (1 if active_high else 0)

    def read_mm(self, timeout_ms=0):
        """The latest distance in millimetres, or None.

        timeout_ms=0: don't wait - return None if no new measurement is ready yet.
        Otherwise wait up to timeout_ms for one.
        """
        start = ticks_ms()
        while not self.data_ready():
            if ticks_diff(ticks_ms(), start) >= timeout_ms:
                return None
            sleep_ms(5)
        data = self.i2c.readfrom_mem(self.address, REG_RESULT_RANGE_STATUS, 17, addrsize=16)
        status = data[0] & 0x1F
        distance_mm = (data[13] << 8) | data[14]    # final crosstalk-corrected range
        self.last_status = status
        self._write_u8(REG_INTERRUPT_CLEAR, 0x01)   # allow the next measurement to be flagged
        if status not in (STATUS_RANGE_VALID, STATUS_RANGE_VALID_MIN_CLIPPED):
            return None                 # no target, too weak a signal, or out of range
        return distance_mm
