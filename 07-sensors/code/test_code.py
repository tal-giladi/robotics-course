"""Tests for the module-07 (07.01-07.05) scripts, no hardware:  py -m pytest 07-sensors/code"""

from __future__ import annotations

import importlib
import math
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import encoder_lab as el
import imu_analysis as ia
import imu_bno055
import imu_log
import range_bench_sim as rbs
import range_physics as rp
import sensor_stats as ss

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


# --- sensor_stats ---------------------------------------------------------------------------------
def test_characterize_hand_computed():
    # true 1.0 m; readings 1.01, 1.02, 1.03, 1.02 plus one outlier (3.0) and one dropout
    c = ss.characterize([1.01, 1.02, 1.03, 1.02, 3.0, None], 1.0)
    assert (c.n_total, c.n_valid, c.n_outliers) == (6, 5, 1)
    assert c.mean_m == pytest.approx(1.02)
    assert c.bias_m == pytest.approx(0.02)
    assert c.std_m == pytest.approx(math.sqrt((0.01**2 + 0 + 0.01**2 + 0) / 3))
    assert c.dropout_rate == pytest.approx(1 / 6)
    assert c.outlier_rate == pytest.approx(1 / 5)
    assert c.raw_std_m > 0.8  # one outlier inflates sigma ~100x


def test_mad_outliers_handles_quantized_data():
    x = [0.500] * 60 + [0.501] * 30 + [0.499] * 10  # MAD = 0: falls back to the mean deviation
    assert not ss.mad_outliers(x).any()
    assert ss.mad_outliers(x + [0.9])[-1]


def test_fit_calibration_recovers_line():
    t = np.array([0.1, 0.5, 1.0, 2.0])
    line = ss.fit_calibration(t, 1.03 * t + 0.012)
    assert (line.gain, line.offset_m) == pytest.approx((1.03, 0.012))
    assert line.correct(1.03 * 1.5 + 0.012) == pytest.approx(1.5)
    with pytest.raises(ValueError):
        ss.fit_calibration([1.0, 1.0], [1.0, 1.1])


def test_logger_lines_and_csv_roundtrip(tmp_path):
    lines = ["Local directory . is mounted at /remote", "us100,512", "vl53l1x,-1", "# done", "vl53l1x,498"]
    rows = ss.parse_logger_lines(lines, 0.5)
    assert rows == [(0.5, "us100", 0.512), (0.5, "vl53l1x", None), (0.5, "vl53l1x", 0.498)]
    path = tmp_path / "bench.csv"
    ss.write_samples(path, rows, append=True)
    ss.write_samples(path, rows, append=True)  # header only once
    groups = ss.read_samples(path)
    assert len(groups[("vl53l1x", 0.5)]) == 4 and np.isnan(groups[("vl53l1x", 0.5)]).sum() == 2
    assert "us100" in ss.report(groups)


def test_stats_cli_and_plot(tmp_path, capsys):
    csv_path = tmp_path / "sim.csv"
    rbs.main(["--sensor", "both", "--distances", "0.25", "1.0", "--n", "80", "--out", str(csv_path)])
    png = tmp_path / "sim.png"
    assert ss.main([str(csv_path), "--plot", str(png)]) == 0
    assert png.stat().st_size > 10_000
    assert "calibration" in capsys.readouterr().out


# --- range physics and the simulated bench --------------------------------------------------------
def test_speed_of_sound_israeli_seasons():
    assert rp.speed_of_sound(20.0) == pytest.approx(343.2, abs=0.1)
    assert rp.ultrasonic_reading_m(1.0, 35.0) == pytest.approx(0.9747, abs=5e-4)  # summer: 25 mm short
    assert rp.ultrasonic_reading_m(1.0, 15.0) == pytest.approx(1.0080, abs=5e-4)  # winter: 8 mm long


def test_geometry_helpers():
    assert rp.beam_footprint_radius_m(1.0, 15.0) == pytest.approx(0.1317, abs=1e-3)
    assert rp.light_round_trip_s(0.001) == pytest.approx(6.67e-12, rel=1e-3)
    assert rp.shot_noise_sigma_scale(1.0, ambient_fraction=3.0) == pytest.approx(1.0)
    assert rp.shot_noise_sigma_scale(2.0) == pytest.approx(2.0)
    assert math.isinf(rp.specular_miss_distance_m(1.0, 50.0))
    assert "Speed of sound" in rp.tables()


def test_bench_ultrasonic_temperature_scale_and_tof_offset():
    summer = ss.characterize([r[2] for r in rbs.simulate_static_test(rbs.RangeSensorModel.ultrasonic(35.0), (2.0,), 300)], 2.0)
    assert summer.bias_m == pytest.approx(2.0 * (343.0 / rp.speed_of_sound(35.0) - 1.0), abs=0.002)
    tof = ss.characterize([r[2] for r in rbs.simulate_static_test(rbs.RangeSensorModel.tof(), (1.0,), 300)], 1.0)
    assert tof.bias_m == pytest.approx(0.008, abs=0.002)


def test_bench_angled_wall_silences_ultrasonic_not_tof():
    us = [r[2] for r in rbs.simulate_static_test(rbs.RangeSensorModel.ultrasonic(), (1.0,), 200, wall_angle_deg=30.0)]
    tof = [r[2] for r in rbs.simulate_static_test(rbs.RangeSensorModel.tof(), (1.0,), 200, wall_angle_deg=30.0)]
    assert sum(v is None for v in us) > 150
    assert sum(v is None for v in tof) < 10


# --- encoders --------------------------------------------------------------------------------------
def test_encoder_numbers():
    geo = el.EncoderGeometry(2464, 0.045, 0.2)
    assert geo.meters_per_tick * 1000 == pytest.approx(0.11475, abs=1e-5)
    assert geo.edges_per_second(17.0) == pytest.approx(6667, abs=1)
    assert geo.rollover_distance_m(16) == pytest.approx(3.76, abs=0.01)
    assert el.counting_resolution_rad_s(2464, 0.01) == pytest.approx(0.255, abs=1e-3)
    w = el.crossover_speed_rad_s(2464, 0.01, 1e-4)
    counting = el.counting_resolution_rad_s(2464, 0.01)
    assert el.period_relative_error(w, 2464, 1e-4) * w == pytest.approx(counting)


def test_irq_counting_loses_edges_only_when_fast():
    slow_true, slow_counted = el.simulate_irq_counting(2.0, 1.0, 2464, blocks_per_s=0.0)
    fast_true, fast_counted = el.simulate_irq_counting(21.5, 1.0, 2464, handler_us=150.0, blocks_per_s=0.0)
    assert slow_counted == slow_true
    assert fast_counted < 0.97 * fast_true


def test_slip_experiment_gyro_sees_scrub_encoders_dont():
    results = {r.label: r for r in el.slip_experiment()}
    scrub = results["spin ~1 turn, scrub (6 % wider track)"]
    assert abs(scrub.gyro_heading_deg - scrub.true_heading_deg) < 2.0
    assert scrub.encoder_heading_deg - scrub.true_heading_deg > 15.0


def test_encoder_lab_cli(capsys):
    for part in ("numbers", "velocity", "irq"):
        assert el.main([part]) == 0
    assert "2464" in capsys.readouterr().out


# --- IMU ---------------------------------------------------------------------------------------------
class FakeBno055Bus:
    def __init__(self):
        self.reg = {0x00: 0xA0, 0x34: 0xE7, 0x35: 0b11_10_01_00}
        self.blocks = {
            0x14: struct.pack("<3h", 16, -32, 1600),  # 1, -2, 100 dps
            0x08: struct.pack("<3h", 0, 0, 981),
            0x0E: struct.pack("<3h", 480, -160, -512),
            0x1A: struct.pack("<3h", 1440, 0, -16),
            0x20: struct.pack("<4h", 16384, 0, 0, 0),
        }
        self.writes = []

    def read_byte_data(self, addr, register):
        assert addr == 0x28
        return self.reg.get(register, 0)

    def write_byte_data(self, addr, register, value):
        self.writes.append((register, value))

    def read_i2c_block_data(self, addr, register, length):
        return list(self.blocks[register][:length])


def test_bno055_reader_scales_and_modes():
    bus = FakeBno055Bus()
    imu = imu_bno055.BNO055(bus, sleep=lambda s: None)
    imu.begin("amg")
    assert (0x3F, 0x20) in bus.writes  # reset
    assert bus.writes[-1] == (0x3D, 0x07)  # AMG mode last
    assert imu.gyro_rad_s() == pytest.approx((math.radians(1), math.radians(-2), math.radians(100)))
    assert imu.accel_m_s2() == pytest.approx((0.0, 0.0, 9.81))
    assert imu.mag_ut() == pytest.approx((30.0, -10.0, -32.0))
    assert imu.euler_deg() == pytest.approx((90.0, 0.0, -1.0))
    assert imu.quaternion_wxyz() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert imu.temperature_c() == -25
    assert imu.calibration_status() == (3, 2, 1, 0)
    with pytest.raises(ValueError):
        imu.set_mode("turbo")
    bus.reg[0x00] = 0x00
    with pytest.raises(OSError):
        imu_bno055.BNO055(bus, sleep=lambda s: None).begin()


def test_gyro_bias_and_drift():
    data = ia.synthetic_imu(seconds=60, seed=1)
    bias, sem = ia.estimate_bias(data["gz"][:1000])
    assert bias == pytest.approx(0.009, abs=0.001)
    heading = ia.integrate_rate(data["t_s"], data["gz"])
    assert math.degrees(heading[-1]) == pytest.approx(math.degrees(0.009 * 60), abs=4.0)
    assert abs(math.degrees(ia.integrate_rate(data["t_s"], data["gz"], bias)[-1])) < 3.0
    assert "bias-corrected" in ia.drift_report(data)


def test_integrate_rate_trapezoid():
    t = np.array([0.0, 1.0, 2.0])
    assert ia.integrate_rate(t, np.array([0.0, 1.0, 1.0])) == pytest.approx([0.0, 0.5, 1.5])


def test_six_position_and_tilt():
    model = ia.ImuErrorModel()
    means = {}
    for axis in ("+x", "-x", "+y", "-y", "+z", "-z"):
        d = ia.synthetic_imu(seconds=5, up_axis=axis, seed=2)
        means[axis] = (np.mean(d["ax"]), np.mean(d["ay"]), np.mean(d["az"]))
    bias, scale = ia.six_position_calibration(means)
    assert bias == pytest.approx(model.accel_bias_m_s2, abs=0.01)
    assert scale == pytest.approx(model.accel_scale, abs=0.002)
    roll, pitch = ia.tilt_from_accel(0.0, 9.81 * math.sin(0.1), 9.81 * math.cos(0.1))
    assert (roll, pitch) == pytest.approx((0.1, 0.0))


def test_hard_soft_iron_and_heading():
    model = ia.ImuErrorModel()
    d = ia.synthetic_imu(seconds=20, yaw_rate_rad_s=2 * math.pi / 10, seed=3)
    offset, scale = ia.hard_soft_iron(d["mx"], d["my"])
    assert offset == pytest.approx(model.mag_hard_iron_ut[:2], abs=1.5)
    assert heading_ok(d, offset, scale)


def heading_ok(d, offset, scale) -> bool:
    """Heading from the corrected field must follow the true yaw (+5° declination offset) within 8°."""
    mx = (d["mx"] - offset[0]) * scale[0]
    my = (d["my"] - offset[1]) * scale[1]
    heading = ia.heading_from_mag(mx, my)
    yaw = d["t_s"] * 2 * math.pi / 10  # the synthetic robot turns once every 10 s from true north
    declination = math.atan2(2.7, 31.0)  # field 5° east of north: true north reads +5° from magnetic north
    err = np.angle(np.exp(1j * (heading - yaw - declination)))
    return bool(np.max(np.abs(np.degrees(err))) < 8.0)


def test_robot_demo_gyro_beats_encoders():
    demo = ia.robot_demo()
    t = demo["t_s"]
    bias, _ = ia.estimate_bias(demo["gz"][t < 5.0])
    fixed = ia.integrate_rate(t, demo["gz"], bias)
    assert abs(math.degrees(fixed[-1] - demo["truth"][-1])) < 2.0
    assert abs(math.degrees(demo["encoders"][-1] - demo["truth"][-1])) > 5.0


def test_imu_log_synthetic_and_record(tmp_path):
    out = tmp_path / "still.csv"
    assert imu_log.main(["--imu", "synthetic", "--seconds", "3", "--out", str(out)]) == 0
    data = ia.read_log(out)
    assert len(data["t_s"]) == 300 and np.nanmean(data["az"]) == pytest.approx(10.0, abs=0.3)

    class Fake:
        def read(self):
            return (0.0,) * 9

        def close(self):
            pass

    rec = imu_log.record(Fake(), 0.2, 50.0)
    assert 8 <= len(rec["t_s"]) <= 11


# --- MicroPython files, under the firmware test shims ------------------------------------------------
@pytest.fixture()
def mp():
    shims = ROOT / "labs" / "python" / "tests" / "mpshims"
    firmware = ROOT / "labs" / "firmware" / "pico"
    names = ("machine", "rp2", "micropython", "utime", "config", "vl53l1x", "encoders", "motors", "sensors",
             "tof_modes", "encoder_irq_vs_pio", "range_logger")
    saved = {n: sys.modules.pop(n) for n in names if n in sys.modules}
    sys.path[:0] = [str(shims), str(firmware), str(HERE / "pico")]
    try:
        yield SimpleNamespace(**{n: importlib.import_module(n) for n in ("machine", "tof_modes", "encoder_irq_vs_pio", "range_logger")})
    finally:
        for n in names:
            sys.modules.pop(n, None)
        sys.modules.update(saved)
        for p in (str(shims), str(firmware), str(HERE / "pico")):
            sys.path.remove(p)


class FakeTof:
    def __init__(self):
        self.writes = []
        self.result = bytearray(17)
        self.ready = 1

    def readfrom_mem(self, register, n):
        values = {0x010F: b"\xea\xcc", 0x0022: b"\x00\x05", 0x0030: b"\x01", 0x0031: bytes([self.ready]),
                  0x00DE: b"\x01\x00"}
        if register == 0x0089:
            return bytes(self.result[:n])
        return values.get(register, bytes(n))[:n]

    def writeto_mem(self, register, data):
        self.writes.append((register, bytes(data)))


def test_tof_modes_writes_uld_values(mp):
    i2c = mp.machine.I2C(0)
    dev = FakeTof()
    i2c.devices[0x29] = dev
    tof = mp.tof_modes.VL53L1XModes(i2c)
    dev.writes.clear()
    tof.configure("short", 20, (4, 4))
    w = dict(dev.writes)
    assert w[0x004B] == b"\x14" and w[0x0060] == b"\x07" and w[0x0078] == b"\x07\x05"
    assert w[0x005E] == b"\x00\x51" and w[0x0061] == b"\x00\x6e"
    assert w[0x0080] == bytes([0x33]) and w[0x007F] == bytes([199])
    assert w[0x006C] == int(0x100 * 20 * 1.075).to_bytes(4, "big")
    assert dev.writes[-1][0] in (0x0086, 0x0087)
    with pytest.raises(ValueError):
        tof.set_timing_budget_ms(15 if tof.mode == "long" else 40)
    dev.result[0], dev.result[7], dev.result[8] = 9, 0x00, 0x10  # valid; ambient 16*8
    dev.result[13], dev.result[14], dev.result[15], dev.result[16] = 0x01, 0xF4, 0x01, 0x00
    assert tof.read_full(timeout_ms=10) == (500, 9, 256 * 8, 16 * 8)
    dev.result[0] = 2  # signal fail
    assert tof.read_full(timeout_ms=10)[0] is None
    dev.ready = 0
    assert tof.read_full(timeout_ms=0) == (None, None, 0, 0)
    assert mp.tof_modes.summarize([(500, 9, 10, 2), (502, 9, 10, 2), (None, 2, 0, 0)])[:3] == pytest.approx(
        (2 / 3, 501.0, math.sqrt(2)))


def test_encoder_miss_percent(mp):
    assert mp.encoder_irq_vs_pio.lost_percent(1000, 950) == pytest.approx(5.0)
    assert mp.encoder_irq_vs_pio.lost_percent(0, 0) == 0.0
    assert mp.range_logger.as_mm(None) == -1
