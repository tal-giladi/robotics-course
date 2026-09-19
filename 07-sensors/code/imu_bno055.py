"""07.05 — A small, readable BNO055 reader for the Raspberry Pi (Linux I²C via smbus2).

    sudo apt install python3-smbus2 i2c-tools       # Ubuntu 24.04 on the Pi 5
    i2cdetect -y 1                                   # the BNO055 answers at 0x28 (0x29 if ADR is high)
    python3 07-sensors/code/imu_bno055.py            # prints gyro / accel / mag / fused heading

Why our own 150 lines instead of a library: every register this lesson talks about is visible here.
The register addresses and scale factors match Bosch's datasheet (BST-BNO055-DS000) and Adafruit's
CircuitPython driver (https://github.com/adafruit/Adafruit_CircuitPython_BNO055).

The bus object only needs the three smbus2 methods used below, so tests use a fake bus.
Units follow the chip's power-on UNIT_SEL: m/s², degrees per second, µT, degrees; we convert to SI.
"""

from __future__ import annotations

import math
import struct
import time
from collections.abc import Callable
from typing import Protocol

ADDRESS = 0x28
CHIP_ID = 0xA0

REG_CHIP_ID = 0x00
REG_ACC_DATA = 0x08  # x, y, z: int16 little-endian, 1 m/s² = 100 LSB
REG_MAG_DATA = 0x0E  # 1 µT = 16 LSB
REG_GYR_DATA = 0x14  # 1 dps = 16 LSB
REG_EUL_DATA = 0x1A  # heading, roll, pitch: 1 degree = 16 LSB   (fusion modes only)
REG_QUA_DATA = 0x20  # w, x, y, z: 1 = 2^14 LSB                   (fusion modes only)
REG_LIA_DATA = 0x28  # linear acceleration = accel - gravity      (fusion modes only)
REG_GRV_DATA = 0x2E  # gravity vector                             (fusion modes only)
REG_TEMP = 0x34  # int8, °C
REG_CALIB_STAT = 0x35  # 2 bits each: system, gyro, accel, mag (3 = calibrated)
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F

MODES = {
    "config": 0x00,
    "acconly": 0x01,
    "magonly": 0x02,
    "gyronly": 0x03,
    "amg": 0x07,  # all three raw sensors, no fusion: what lesson 07.05 analyses
    "imu": 0x08,  # accel + gyro fusion: relative heading, no magnetometer
    "compass": 0x09,
    "m4g": 0x0A,
    "ndof_fmc_off": 0x0B,
    "ndof": 0x0C,  # 9-axis fusion: absolute heading from the magnetometer
}
FUSION_MODES = {"imu", "compass", "m4g", "ndof_fmc_off", "ndof"}

ACCEL_LSB_PER_M_S2 = 100.0
MAG_LSB_PER_UT = 16.0
GYRO_LSB_PER_DPS = 16.0
EULER_LSB_PER_DEG = 16.0
QUAT_LSB = float(1 << 14)


class SMBusLike(Protocol):
    def read_byte_data(self, i2c_addr: int, register: int) -> int: ...
    def write_byte_data(self, i2c_addr: int, register: int, value: int) -> None: ...
    def read_i2c_block_data(self, i2c_addr: int, register: int, length: int) -> list[int]: ...


class BNO055:
    """Bosch BNO055 over I²C.

    >>> from smbus2 import SMBus
    >>> with SMBus(1) as bus:
    ...     imu = BNO055(bus)
    ...     imu.begin("amg")
    ...     imu.gyro_rad_s()
    """

    def __init__(self, bus: SMBusLike, address: int = ADDRESS, sleep: Callable[[float], None] = time.sleep) -> None:
        self.bus = bus
        self.address = address
        self.sleep = sleep
        self.mode = "config"

    # --- setup ---------------------------------------------------------------------------------------
    def begin(self, mode: str = "amg") -> None:
        """Check the chip, reset it, and enter ``mode``. Takes about a second."""
        if self._read_u8(REG_CHIP_ID) != CHIP_ID:
            self.sleep(1.0)  # the chip needs ~650-850 ms after power-on before it answers
            if self._read_u8(REG_CHIP_ID) != CHIP_ID:
                raise OSError(f"no BNO055 at 0x{self.address:02X}: check wiring, `i2cdetect -y 1`")
        self.set_mode("config")
        self._write_u8(REG_SYS_TRIGGER, 0x20)  # reset
        self.sleep(0.7)
        for _ in range(20):  # wait until it answers again
            try:
                if self._read_u8(REG_CHIP_ID) == CHIP_ID:
                    break
            except OSError:
                pass
            self.sleep(0.05)
        else:
            raise OSError("BNO055 did not come back after reset")
        self._write_u8(REG_PWR_MODE, 0x00)  # normal power
        self._write_u8(REG_SYS_TRIGGER, 0x00)  # internal oscillator
        self.sleep(0.01)
        self.set_mode(mode)

    def set_mode(self, mode: str) -> None:
        """Switch operating mode. Any change goes through CONFIG mode (datasheet: 19 ms in, 7 ms out)."""
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}; choose from {sorted(MODES)}")
        self._write_u8(REG_OPR_MODE, MODES["config"])
        self.sleep(0.025)
        if mode != "config":
            self._write_u8(REG_OPR_MODE, MODES[mode])
            self.sleep(0.02)
        self.mode = mode

    # --- measurements (SI units) ---------------------------------------------------------------------
    def gyro_rad_s(self) -> tuple[float, float, float]:
        return self._vector(REG_GYR_DATA, 3, math.radians(1.0) / GYRO_LSB_PER_DPS)

    def accel_m_s2(self) -> tuple[float, float, float]:
        """Specific force: at rest, flat, z reads about +9.81 (the table pushing up), not 0."""
        return self._vector(REG_ACC_DATA, 3, 1.0 / ACCEL_LSB_PER_M_S2)

    def mag_ut(self) -> tuple[float, float, float]:
        return self._vector(REG_MAG_DATA, 3, 1.0 / MAG_LSB_PER_UT)

    def euler_deg(self) -> tuple[float, float, float]:
        """(heading, roll, pitch) in degrees from the fusion.

        Heading is 0..360 in the chip's compass-style convention, NOT ROS yaw (counter-clockwise
        positive, REP-103): turn the robot left by hand and watch which way it moves before trusting it.
        """
        return self._vector(REG_EUL_DATA, 3, 1.0 / EULER_LSB_PER_DEG)

    def quaternion_wxyz(self) -> tuple[float, float, float, float]:
        return self._vector(REG_QUA_DATA, 4, 1.0 / QUAT_LSB)

    def linear_accel_m_s2(self) -> tuple[float, float, float]:
        return self._vector(REG_LIA_DATA, 3, 1.0 / ACCEL_LSB_PER_M_S2)

    def gravity_m_s2(self) -> tuple[float, float, float]:
        return self._vector(REG_GRV_DATA, 3, 1.0 / ACCEL_LSB_PER_M_S2)

    def temperature_c(self) -> int:
        value = self._read_u8(REG_TEMP)
        return value - 256 if value > 127 else value

    def calibration_status(self) -> tuple[int, int, int, int]:
        """(system, gyro, accel, mag), each 0 (not calibrated) .. 3 (fully calibrated)."""
        v = self._read_u8(REG_CALIB_STAT)
        return (v >> 6) & 3, (v >> 4) & 3, (v >> 2) & 3, v & 3

    # --- register access -----------------------------------------------------------------------------
    def _read_u8(self, register: int) -> int:
        return self.bus.read_byte_data(self.address, register)

    def _write_u8(self, register: int, value: int) -> None:
        self.bus.write_byte_data(self.address, register, value)

    def _vector(self, register: int, count: int, scale: float) -> tuple[float, ...]:
        raw = bytes(self.bus.read_i2c_block_data(self.address, register, 2 * count))
        return tuple(v * scale for v in struct.unpack(f"<{count}h", raw))


def main() -> int:  # pragma: no cover - needs the hardware
    from smbus2 import SMBus

    with SMBus(1) as bus:
        imu = BNO055(bus)
        imu.begin("ndof")
        print("Move the robot gently. Ctrl-C to stop. Calibration: sys/gyro/accel/mag 0..3")
        try:
            while True:
                g, a, m = imu.gyro_rad_s(), imu.accel_m_s2(), imu.mag_ut()
                heading, roll, pitch = imu.euler_deg()
                print(f"gyro {g[0]:+6.3f} {g[1]:+6.3f} {g[2]:+6.3f} rad/s | accel {a[0]:+6.2f} {a[1]:+6.2f} {a[2]:+6.2f}"
                      f" m/s2 | mag {m[0]:+6.1f} {m[1]:+6.1f} {m[2]:+6.1f} uT | heading {heading:6.1f} deg"
                      f" | calib {imu.calibration_status()}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
