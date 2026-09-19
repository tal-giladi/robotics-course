"""07.05 — Record an IMU to CSV at a fixed rate: BNO055 or ICM-20948 on the Pi's I²C, or synthetic data.

On the Raspberry Pi 5 (Ubuntu 24.04), IMU wired to header pins 1 (3V3), 3 (SDA), 5 (SCL), 6 (GND):

    sudo apt install python3-smbus2 python3-numpy i2c-tools
    i2cdetect -y 1                                        # BNO055: 28   ICM-20948: 68 or 69
    python3 07-sensors/code/imu_log.py --imu bno055 --mode amg --seconds 120 --out still.csv

ICM-20948 instead (SparkFun's pure-Python driver, which uses smbus2; install it in a venv):

    python3 -m venv --system-site-packages ~/imu-venv && ~/imu-venv/bin/pip install sparkfun-qwiic-icm20948
    ~/imu-venv/bin/python 07-sensors/code/imu_log.py --imu icm20948 --seconds 120 --out still.csv

No hardware:

    py 07-sensors/code/imu_log.py --imu synthetic --seconds 120 --out still.csv

Columns: t_s, gx, gy, gz [rad/s], ax, ay, az [m/s²], mx, my, mz [µT] - in the SENSOR's axes, as printed
on the board. Mapping them to the robot's base_link axes is lesson 07.06.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Protocol

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from imu_analysis import COLUMNS, G, synthetic_imu, write_log  # noqa: E402

Sample = tuple[float, float, float, float, float, float, float, float, float]


class ImuReader(Protocol):
    def read(self) -> Sample: ...  # gx gy gz [rad/s], ax ay az [m/s²], mx my mz [µT]
    def close(self) -> None: ...


class Bno055Reader:
    def __init__(self, bus_number: int = 1, mode: str = "amg") -> None:
        from smbus2 import SMBus

        from imu_bno055 import BNO055

        self._bus = SMBus(bus_number)
        self.imu = BNO055(self._bus)
        self.imu.begin(mode)

    def read(self) -> Sample:
        return (*self.imu.gyro_rad_s(), *self.imu.accel_m_s2(), *self.imu.mag_ut())

    def close(self) -> None:
        self._bus.close()


class Icm20948Reader:
    """SparkFun driver defaults after begin(): ±2 g, ±250 °/s, digital low-pass filters off."""

    ACCEL_LSB_PER_G = 16384.0  # ±2 g range
    GYRO_LSB_PER_DPS = 131.0  # ±250 °/s range
    MAG_UT_PER_LSB = 0.15  # AK09916 magnetometer inside the ICM-20948

    def __init__(self) -> None:
        import qwiic_icm20948

        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            raise OSError("ICM-20948 not found at 0x68/0x69 - check wiring and `i2cdetect -y 1`")
        self.imu.begin()

    def read(self) -> Sample:
        i = self.imu
        i.getAgmt()
        g = math.radians(1.0) / self.GYRO_LSB_PER_DPS
        a = G / self.ACCEL_LSB_PER_G
        m = self.MAG_UT_PER_LSB
        return (i.gxRaw * g, i.gyRaw * g, i.gzRaw * g, i.axRaw * a, i.ayRaw * a, i.azRaw * a,
                i.mxRaw * m, i.myRaw * m, i.mzRaw * m)

    def close(self) -> None:
        pass


def record(reader: ImuReader, seconds: float, rate_hz: float) -> dict[str, np.ndarray]:
    """Sample on a fixed schedule (sleep until the next tick, never accumulate the delay)."""
    period = 1.0 / rate_hz
    rows: list[tuple[float, ...]] = []
    start = time.monotonic()
    next_tick = start
    late = 0
    while True:
        now = time.monotonic()
        if now - start >= seconds:
            break
        if now < next_tick:
            time.sleep(next_tick - now)
        elif now - next_tick > period:
            late += 1
        rows.append((time.monotonic() - start, *reader.read()))
        next_tick += period
    if late:
        print(f"warning: {late} samples were more than one period late - lower --rate", file=sys.stderr)
    arr = np.array(rows)
    return {c: arr[:, i] for i, c in enumerate(COLUMNS)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--imu", choices=("bno055", "icm20948", "synthetic"), required=True)
    ap.add_argument("--mode", default="amg", help="BNO055 operating mode: amg (raw) or ndof/imu (fusion)")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=100.0, help="Hz")
    ap.add_argument("--up-axis", default="+z", help="synthetic only: which sensor axis points up (+x ... -z)")
    ap.add_argument("--yaw-rate", type=float, default=0.0, help="synthetic only: constant turn rate, rad/s")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    if args.imu == "synthetic":
        data = synthetic_imu(args.seconds, args.rate, args.yaw_rate, up_axis=args.up_axis, seed=args.seed)
    else:
        reader: ImuReader = Bno055Reader(mode=args.mode) if args.imu == "bno055" else Icm20948Reader()
        print(f"recording {args.seconds:.0f} s at {args.rate:.0f} Hz - don't touch the robot", file=sys.stderr)
        try:
            data = record(reader, args.seconds, args.rate)
        finally:
            reader.close()
    write_log(args.out, data)
    print(f"wrote {len(data['t_s'])} samples to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
