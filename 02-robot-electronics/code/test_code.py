"""Tests for the module-02 scripts (no hardware): py -m pytest 02-robot-electronics/code

They pin the numbers the lessons quote, so a change to a script that changes a lesson's
numbers fails here first.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import board_drc
import brownout_sim
import bus_budget
import datasheet_check
import fault_tree
import harness
import i2c_bus
import level_shift
import motor_driver_lab as mdl
import noise_lab
import power_budget

ROOT = Path(__file__).resolve().parents[2]


# --- 02.01 --------------------------------------------------------------------------------------
def test_datasheet_links():
    r = {x.link.split(" -> ")[0] + " -> " + x.link.split(" -> ")[1][:12]: x for x in datasheet_check.main([])["links"]}
    pico_drv = datasheet_check.check_link(datasheet_check.PICO_GPIO_OUT, datasheet_check.DRV8874_IN)
    assert (pico_drv.nm_high, pico_drv.nm_low, pico_drv.ok) == (1.12, 0.3, True)
    raw = datasheet_check.check_link(datasheet_check.HCSR04_ECHO_5V, datasheet_check.PICO_GPIO_FT_IN)
    assert raw.stress_unpowered and not raw.stress_powered and not raw.ok
    adc = datasheet_check.check_link(datasheet_check.HCSR04_ECHO_5V, datasheet_check.PICO_ADC_PIN_IN)
    assert adc.stress_powered
    assert len(r) == 7
    assert datasheet_check.pwm_timing()["lost_fraction"] == pytest.approx(0.013)


# --- 02.02 --------------------------------------------------------------------------------------
def test_power_budget_numbers():
    r = power_budget.main([])
    assert r["rail_5v"] == pytest.approx((0.92, 3.81))
    assert r["rail_3v3"][1] < 0.3
    assert power_budget.DRV8874_ITRIP_A == pytest.approx(2.945, abs=0.01)
    assert r["battery_worst_12.6"] == pytest.approx(7.57, abs=0.02)
    assert r["r_path_karmel"] == pytest.approx(0.216, abs=0.001)
    assert r["r_path_breadboard"] == pytest.approx(0.426, abs=0.001)
    assert r["runtime_typ_h"] == pytest.approx(3.3, abs=0.1)
    assert r["runtime_worst_h"] == pytest.approx(0.8, abs=0.1)
    assert power_budget.regulator_input_a(19.05, 9.9) > power_budget.regulator_input_a(19.05, 12.6)


def test_brownout_scenarios():
    r = brownout_sim.main([])
    names = list(r)
    assert not r[names[0]]["brownout"] and r[names[0]]["min_pi_v"] == pytest.approx(5.0, abs=0.01)
    assert r[names[2]]["brownout"] and r[names[2]]["min_pi_v"] < 4.4
    assert r[names[3]]["min_pi_v"] > 4.63                      # ramp fixes it
    assert r[names[4]]["brownout"]                              # a bulk capacitor alone does not
    assert not r[names[5]]["brownout"] and not r[names[6]]["brownout"]
    assert r[names[0]]["peak_motor_a"] == pytest.approx(2.95, abs=0.01)


# --- 02.03 --------------------------------------------------------------------------------------
def test_pwm_ripple_and_decay_modes():
    assert mdl.ripple_at_stall(0.5, 1_000) > 1.0
    assert mdl.ripple_at_stall(0.5, 20_000) < 0.1
    brake = mdl.pwm_steady_state(0.5, "brake", 20_000)
    ideal = (0.5 * 12 - mdl.I0 * (mdl.R_M + mdl.R_DS_HS_LS)) / mdl.K_M
    assert brake.speed_rad_s == pytest.approx(ideal, rel=0.02)   # drive/brake: speed linear in duty
    coast = mdl.pwm_steady_state(0.5, "coast", 20_000)
    assert coast.speed_rad_s < 0.2 * ideal                       # fast decay at 20 kHz: huge deadband


def test_losses_limit_and_sense_designs():
    p = mdl.driver_losses(2.1, 12.0, 20_000)
    assert p["conduction_w"] == pytest.approx(0.882)
    assert p["switching_w"] == pytest.approx(0.0756)
    assert mdl.junction_temp(2.1, 12, 20_000) == pytest.approx(71, abs=1)
    assert mdl.i_trip(3.3) == pytest.approx(2.945, abs=0.01)
    assert mdl.i_trip(5.0) == pytest.approx(4.462, abs=0.01)
    a, b, c, d = mdl.SENSE_DESIGNS
    assert a.pin_v_at(6) <= 3.3 and a.clamp_current_ma(12) == pytest.approx(0.84, abs=0.01)
    assert b.pin_v_at(6) == pytest.approx(5.0)
    assert c.trip_a() == pytest.approx(4.56, abs=0.01) and c.pin_v_at(c.trip_a()) == pytest.approx(2.96, abs=0.01)
    assert d.trip_a() == pytest.approx(8.17, abs=0.01)           # the "normal" divider disables the limit
    assert mdl.bus_pump_voltage(12.6, 0.4744, 470e-6, 0.5) == pytest.approx(34.2, abs=0.2)


# --- 02.04 --------------------------------------------------------------------------------------
def test_i2c_numbers():
    r = i2c_bus.main([])
    assert r["c_bus_pf"] == pytest.approx(49)
    assert r["ranges"]["fast 400 kHz"][0] == pytest.approx(966.7, abs=1)
    assert r["tr_existing_s"] == pytest.approx(207.6e-9, rel=0.01)
    assert i2c_bus.register_read_s(100_000, 2) == pytest.approx(0.00048)
    assert i2c_bus.us100_uart_distance_mm(0x02, 0x0B) == 523
    assert i2c_bus.us100_uart_temperature_c(0x45) == 24


def _import_pico(name: str, monkeypatch):
    import machine
    for cls in ("UART", "SPI"):
        if not hasattr(machine, cls):
            monkeypatch.setattr(machine, cls, object, raising=False)
    sys.modules.pop(name, None)
    return __import__(name)


class FakeI2C:
    def __init__(self, replies):
        self.replies = replies

    def readfrom_mem(self, address, register, n, addrsize=8):
        value = self.replies.pop(0)
        if isinstance(value, Exception):
            raise value
        return value


def test_i2c_doctor_logic(monkeypatch):
    doc = _import_pico("i2c_doctor", monkeypatch)
    assert doc.classify_idle(1, 1) == "ok"
    assert doc.classify_idle(0, 1).startswith("SDA stuck low")
    assert doc.classify_idle(0, 0).startswith("both low")
    fake = FakeI2C([b"\x39\x9f", OSError(5), b"\x00\x00", b"\x39\x9f"])
    good, errors, wrong, _ = doc.stress(fake, 0x40, 0x00, 2, 8, b"\x39\x9f", reads=4)
    assert (good, errors, wrong) == (2, 1, 1)


def test_us100_uart_and_spi_helpers(monkeypatch):
    us = _import_pico("us100_uart", monkeypatch)
    assert us.parse_distance(b"\x02\x0b") == 523
    assert us.parse_distance(b"\x00\x05") is None and us.parse_distance(b"\x02") is None
    assert us.parse_temperature(b"\x40") == 19
    spi = _import_pico("spi_stress", monkeypatch)
    block = spi.lcg_bytes(1, 64)
    assert len(block) == 64 and block == spi.lcg_bytes(1, 64)
    assert spi.bit_errors(b"\x0f\x00", b"\x00\x01") == 5


def test_noise_probe_sample(monkeypatch):
    sys.path.insert(0, str(ROOT / "labs" / "firmware" / "pico"))
    try:
        probe = _import_pico("noise_probe", monkeypatch)
    finally:
        sys.path.pop(0)

    class Tof:
        def read_mm(self, timeout_ms=0):
            return 512

    class Ina:
        def bus_voltage_v(self):
            raise OSError(5)

        def current_a(self):
            return 0.2

    class Adc:
        def read_u16(self):
            return 655

    lines: list[str] = []
    probe.sample("off", None, Tof(), Ina(), Adc(), lines.append)
    assert lines == ["off,tof_mm,512", "off,bus_v,err", "off,adc_gnd_mv,33.0"]


# --- 02.05 / 02.06 ------------------------------------------------------------------------------
def test_level_shift():
    r = level_shift.main([])
    lo, hi, ok = r[(1_000, 2_000)]
    assert lo == pytest.approx(3.06, abs=0.01) and hi == pytest.approx(3.61, abs=0.01) and ok
    lo, hi, ok = r[(2_200, 3_300)]
    assert hi < 3.3 and ok
    assert r[(1_000, 1_000)][2] is False
    assert r["faults"]["12 V onto an ADC pin (GP26) via 1 k: internal diode to 3V3"] == pytest.approx(8.2)


def test_noise_numbers_and_analysis():
    r = noise_lab.main([])
    assert r[("bounce", 100, 2.95)] == pytest.approx(1.967, abs=0.01)
    assert r[("motor_cap_w", 100e-9)] == pytest.approx(0.288)
    assert noise_lab.encoder_quadrature_spacing_s() == pytest.approx(119e-6, abs=1e-6)
    log = "\n".join(["off,tof_mm,500"] * 10 + ["off,tof_mm,501"] * 10 + ["reverse,tof_mm,500"] * 18
                    + ["reverse,tof_mm,900", "reverse,tof_mm,err"])
    res = noise_lab.analyse(log)
    assert res[("off", "tof_mm")].outliers == 0
    assert res[("reverse", "tof_mm")].outliers == 1
    assert res[("reverse", "tof_mm_errors")].n == 1


# --- 02.07 - 02.10 ------------------------------------------------------------------------------
def test_harness():
    assert harness.ohm_per_m(18) == pytest.approx(0.02094, rel=0.01)
    r = harness.main([])
    assert r["battery -> fuse -> switch -> power board"][0] == 20
    assert r["5 V regulator -> Pi 5 (USB-C pigtail)"][0] == 22
    assert harness.pick_connector(2.0).name == "JST XH 2.5 mm"


def test_board_drc_clean_and_catches_mistakes():
    assert board_drc.main([]) == []
    nets = board_drc.karmel_board()
    nets[3].nodes[0] = "PICO.GP8"                                   # L_IN1 on the wrong pin
    nets.append(board_drc.Net("OOPS", 12.6, 0, ["PICO.GP14"]))      # 12 V to a GPIO, 1 node, short with TRIG
    nets[0].nodes.remove("C_BULK.+")
    nets[1].nodes.remove("TP.2")
    problems = board_drc.drc(nets, yaml_pins=board_drc.pico_pins_from_yaml())
    text = "\n".join(problems)
    for expected in ("uses PICO.GP8", "only one connection", "exceeds PICO.GP14", "short circuit",
                     "missing C_BULK", "ground net without a test point"):
        assert expected in text, expected


def test_fault_tree_finds_every_injected_fault():
    r = fault_tree.main([])
    for fault, (_, phrase) in fault_tree.FAULTS.items():
        assert phrase in r[fault][0]


def test_bus_budget():
    r = bus_budget.main([])
    assert r[("arm", 1_000_000, 0.0, 0.002)]["max_hz"] == pytest.approx(205, abs=2)
    assert bus_budget.can_classic_frame_bits(8) == 135
    assert r[("CAN FD 1/5 Mbit/s, 8 B frames", 500)] < 0.7
    assert r[("rs485", 15, 1_000_000)] is False


def test_sag_logger_resistance_estimate(monkeypatch):
    sys.path.insert(0, str(ROOT / "labs" / "firmware" / "pico"))
    try:
        sag = _import_pico("sag_logger", monkeypatch)
    finally:
        sys.path.pop(0)
    samples = [(t, 11.90, 0.45) for t in range(0, 100, 2)]
    samples += [(100 + t, 11.90 - 0.216 * 2.2 * (1 if t < 40 else 0.3), 0.45 + 2.2 * (1 if t < 40 else 0.3))
                for t in range(0, 600, 2)]
    r = sag.estimate_resistance(samples)
    assert r["r_ohm"] == pytest.approx(0.216, abs=0.002)
    assert sag.estimate_resistance([(t, 12.0, 0.4) for t in range(0, 300, 2)]) is None
