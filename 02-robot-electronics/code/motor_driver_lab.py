"""02.03 — the DRV8874 driving a Yahboom 520 motor: PWM ripple, decay modes, losses, heat, current
limit, IPROPI scaling for the Pico ADC and regenerative bus pumping.

    python 02-robot-electronics/code/motor_driver_lab.py              all sections
    python 02-robot-electronics/code/motor_driver_lab.py --section pwm

Datasheet values: TI DRV8874 (SLVSF66A) and Pololu carrier 4035. Motor constants as in FE.10.
L_M (winding inductance) is an ESTIMATE; exercise 02.03-E2 measures it.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

# --- motor (Yahboom 520, 1:56; FE.10 derivation) ------------------------------------------------
R_M = 3.0
K_M = 0.00973
I0 = 0.10
L_M = 2.0e-3
# --- DRV8874 (typical, 25 C) --------------------------------------------------------------------
R_DS_HS_LS = 0.200            # ohm, high side + low side (100 + 100 mOhm typ)
T_RISE = 150e-9
T_FALL = 150e-9
T_DEAD = 100e-9
V_SD = 0.9                    # body diode forward voltage at 1 A
A_IPROPI = 450e-6             # A of IPROPI current per A of motor current
R_IPROPI_CARRIER = 2490.0     # Pololu carrier's CS resistor
R_THETA_JA_JEDEC = 36.0       # C/W on TI's JEDEC test board; a 0.6" x 0.7" carrier is worse
TSD_MIN_C = 160.0


# =============================================================================================
# 1. PWM: current ripple and speed vs duty for drive/brake (slow decay) and drive/coast (fast decay)
# =============================================================================================
@dataclass(frozen=True)
class PwmResult:
    freq_hz: float
    duty: float
    mode: str
    speed_rad_s: float        # motor shaft, free-running
    i_mean: float
    i_ripple_pp: float


def _period_average(v_bus: float, duty: float, mode: str, freq: float, w: float,
                    steps_per_period: int = 80, max_periods: int = 400) -> tuple[float, float, float]:
    """Periodic steady-state current for a FIXED speed w. Returns (mean, min, max) over one period."""
    period = 1.0 / freq
    dt = period / steps_per_period
    i = 0.0
    last_start = None
    for _ in range(max_periods):
        start = i
        acc = 0.0
        lo, hi = i, i
        for n in range(steps_per_period):
            on = n < duty * steps_per_period
            e = K_M * w
            if on:
                v = v_bus
            elif mode == "brake":
                v = 0.0                                  # both low sides on: terminals shorted
            else:                                        # coast: current returns through body diodes
                v = -(v_bus + 2 * V_SD) if i > 0 else e  # until it reaches zero, then terminals float
            if not on and mode == "coast" and i <= 0:
                i = 0.0
                di = 0.0
            else:
                di = (v - (R_M + R_DS_HS_LS) * i - e) / L_M
            i += di * dt
            if not on and mode == "coast" and i < 0:
                i = 0.0
            acc += i
            lo, hi = min(lo, i), max(hi, i)
        if last_start is not None and abs(start - last_start) < 1e-5:
            return acc / steps_per_period, lo, hi
        last_start = start
    return acc / steps_per_period, lo, hi


def pwm_steady_state(duty: float, mode: str, freq: float, v_bus: float = 12.0) -> PwmResult:
    """Free-running wheel: find the speed where the mean motor current equals the friction current I0."""
    lo_w, hi_w = 0.0, v_bus / K_M
    for _ in range(22):
        w = 0.5 * (lo_w + hi_w)
        mean, _, _ = _period_average(v_bus, duty, mode, freq, w)
        if mean > I0:
            lo_w = w
        else:
            hi_w = w
    mean, i_lo, i_hi = _period_average(v_bus, duty, mode, freq, w)
    return PwmResult(freq, duty, mode, w, mean, i_hi - i_lo)


def ripple_at_stall(duty: float, freq: float, v_bus: float = 12.0) -> float:
    """Worst-case ripple (wheel held): peak-to-peak current in drive/brake at the given frequency."""
    _, lo, hi = _period_average(v_bus, duty, "brake", freq, 0.0)
    return hi - lo


# =============================================================================================
# 2. Losses and junction temperature
# =============================================================================================
def driver_losses(i_motor: float, v_bus: float, freq: float, r_ds: float = R_DS_HS_LS) -> dict[str, float]:
    conduction = i_motor ** 2 * r_ds
    switching = 0.5 * v_bus * i_motor * (T_RISE + T_FALL) * freq
    dead_time = 2 * V_SD * i_motor * T_DEAD * freq
    return {"conduction_w": conduction, "switching_w": switching, "dead_time_w": dead_time,
            "total_w": conduction + switching + dead_time}


def junction_temp(i_motor: float, v_bus: float, freq: float, t_ambient: float = 30.0,
                  r_theta: float = R_THETA_JA_JEDEC, rds_tempco: float = 0.004) -> float:
    """R_DS(on) rises ~0.4 %/C (estimate), which raises the heat: iterate to the fixed point."""
    tj = t_ambient
    for _ in range(50):
        r_ds = R_DS_HS_LS * (1 + rds_tempco * (tj - 25.0))
        p = driver_losses(i_motor, v_bus, freq, r_ds)["total_w"]
        new_tj = t_ambient + p * r_theta
        if new_tj > 400:
            return math.inf                             # no fixed point: thermal runaway
        if abs(new_tj - tj) < 0.01:
            return new_tj
        tj = new_tj
    return tj


# =============================================================================================
# 3. Current limit and IPROPI -> Pico ADC
# =============================================================================================
def i_trip(v_ref: float, r_ipropi: float = R_IPROPI_CARRIER) -> float:
    """TI: I_TRIP x A_IPROPI = V_VREF / R_IPROPI."""
    return v_ref / (r_ipropi * A_IPROPI)


def parallel(a: float, b: float) -> float:
    return a * b / (a + b)


@dataclass(frozen=True)
class SenseDesign:
    name: str
    v_ref: float                  # V on VREF (= SLEEP on the Pololu carrier)
    r_top: float | None = None    # divider from CS to the ADC pin (None = no divider)
    r_bottom: float | None = None
    r_series: float = 10_000.0    # series resistor into the ADC pin / clamp
    c_filter: float = 10e-9

    def r_cs_effective(self) -> float:
        if self.r_top is None or self.r_bottom is None:
            return R_IPROPI_CARRIER
        return parallel(R_IPROPI_CARRIER, self.r_top + self.r_bottom)

    def divider_ratio(self) -> float:
        if self.r_top is None or self.r_bottom is None:
            return 1.0
        return self.r_bottom / (self.r_top + self.r_bottom)

    def volts_per_amp_at_pin(self) -> float:
        return A_IPROPI * self.r_cs_effective() * self.divider_ratio()

    def trip_a(self) -> float:
        return i_trip(self.v_ref, self.r_cs_effective())

    def pin_v_at(self, amps: float) -> float:
        # IPROPI is clamped near VREF by the chip (exact clamp level not published): cap at VREF.
        v_cs = min(A_IPROPI * self.r_cs_effective() * amps, self.v_ref)
        return v_cs * self.divider_ratio()

    def filter_cutoff_hz(self) -> float:
        r_source = self.r_series + (parallel(self.r_top, self.r_bottom) if self.r_top and self.r_bottom else 0.0)
        if r_source <= 0 or self.c_filter <= 0:
            return math.inf                              # no filter at all
        return 1.0 / (2 * math.pi * r_source * self.c_filter)

    def clamp_current_ma(self, fault_v: float) -> float:
        """A wiring mistake puts fault_v on the CS net: current forced into the 3V3 Schottky clamp."""
        v_before_series = fault_v * self.divider_ratio()
        r_source = self.r_series + (parallel(self.r_top, self.r_bottom) if self.r_top and self.r_bottom else 0.0)
        if r_source <= 0:
            return math.inf
        return max(0.0, v_before_series - (3.3 + 0.3)) / r_source * 1000


SENSE_DESIGNS = [
    SenseDesign("A: SLEEP=3.3 V (karmel), 10k series + BAT54S clamp + 10 nF", v_ref=3.3),
    SenseDesign("B: naive, SLEEP=5 V straight into ADC", v_ref=5.0, r_series=0.0, c_filter=0.0),
    SenseDesign("C: SLEEP=5 V, 47k/68k divider + 1k series + clamp + 100 nF", v_ref=5.0,
                r_top=47_000, r_bottom=68_000, r_series=1_000, c_filter=100e-9),
    SenseDesign("D: SLEEP=5 V, 1k/2k 'normal' divider (loads CS!)", v_ref=5.0,
                r_top=1_000, r_bottom=2_000, r_series=1_000),
]


# =============================================================================================
# 4. Braking energy and bus pumping
# =============================================================================================
def bus_pump_voltage(v0: float, energy_j: float, c: float, fraction_returned: float) -> float:
    return math.sqrt(v0 ** 2 + 2 * fraction_returned * energy_j / c)


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--section", choices=["pwm", "losses", "sense", "brake", "all"], default="all")
    args = ap.parse_args(argv)
    out: dict = {}

    if args.section in ("pwm", "all"):
        print("1) Free-running speed and current ripple at 12 V")
        print(f"   {'duty':>5} {'freq':>7} {'mode':>6} {'speed':>9} {'ideal d*V/k':>11} {'I mean':>7} {'ripple pp':>9}")
        rows = []
        for freq in (1_000, 20_000):
            for mode in ("brake", "coast"):
                for duty in (0.1, 0.2, 0.3, 0.5, 0.8):
                    r = pwm_steady_state(duty, mode, freq)
                    rows.append(r)
                    ideal = (duty * 12.0 - I0 * (R_M + R_DS_HS_LS)) / K_M
                    print(f"   {duty:5.1f} {freq:7d} {mode:>6} {r.speed_rad_s:7.0f}/s {ideal:9.0f}/s {r.i_mean:7.3f} {r.i_ripple_pp:9.3f}")
        out["pwm"] = rows
        for freq in (1_000, 20_000):
            pp = ripple_at_stall(0.5, freq)
            out[f"stall_ripple_{freq}"] = pp
            print(f"   wheel held, 50 % duty, {freq / 1000:4.0f} kHz: ripple {pp:.3f} A peak-to-peak "
                  f"(tau_e = L/R = {1000 * L_M / (R_M + R_DS_HS_LS):.2f} ms)")

    if args.section in ("losses", "all"):
        print("\n2) DRV8874 losses at 12 V (typical values) and junction temperature at 30 C ambient")
        print(f"   {'I':>5} {'f':>7} {'cond W':>7} {'sw W':>6} {'dead W':>6} {'total':>6} {'Tj JEDEC':>9} {'Tj 70C/W':>9}   (TSD = shuts down, >= 160 C)")
        losses = {}
        for amps in (0.5, 2.1, 2.95):
            for freq in (1_000, 20_000, 100_000):
                p = driver_losses(amps, 12.0, freq)
                tj36 = junction_temp(amps, 12.0, freq)
                tj70 = junction_temp(amps, 12.0, freq, r_theta=70.0)
                losses[(amps, freq)] = (p, tj36, tj70)
                t36 = f"{tj36:8.0f}C" if tj36 < TSD_MIN_C else "     TSD"
                t70 = f"{tj70:8.0f}C" if tj70 < TSD_MIN_C else "     TSD"
                print(f"   {amps:5.2f} {freq:7d} {p['conduction_w']:7.3f} {p['switching_w']:6.3f} {p['dead_time_w']:6.3f} "
                      f"{p['total_w']:6.3f} {t36} {t70}")
        out["losses"] = losses

    if args.section in ("sense", "all"):
        print(f"\n3) Current limit: I_TRIP = V_REF / (R_IPROPI * A_IPROPI); SLEEP 3.3 V -> {i_trip(3.3):.2f} A, "
              f"SLEEP 5.0 V -> {i_trip(5.0):.2f} A (+/-5.5 % mirror error above 2 A)")
        print(f"   {'design':62} {'V/A pin':>7} {'I_trip':>7} {'pin@trip':>8} {'pin@6A':>7} {'fc Hz':>7} {'ADC safe':>8} {'limit kept':>10}")
        sense = {}
        for d in SENSE_DESIGNS:
            v_trip = d.pin_v_at(d.trip_a())
            v_6a = d.pin_v_at(6.0)
            ok = v_6a <= 3.3
            limit_kept = abs(d.trip_a() / i_trip(d.v_ref) - 1) < 0.05
            sense[d.name] = {"v_per_a": d.volts_per_amp_at_pin(), "trip_a": d.trip_a(), "pin_at_trip": v_trip,
                             "pin_at_6a": v_6a, "fc_hz": d.filter_cutoff_hz(), "ok": ok, "limit_kept": limit_kept,
                             "clamp_ma_at_12v": d.clamp_current_ma(12.0)}
            print(f"   {d.name:62} {d.volts_per_amp_at_pin():7.3f} {d.trip_a():6.2f}A {v_trip:7.2f}V {v_6a:6.2f}V "
                  f"{d.filter_cutoff_hz():7.0f} {'yes' if ok else 'NO':>8} {'yes' if limit_kept else 'NO':>10}")
        counts_per_amp = SENSE_DESIGNS[0].volts_per_amp_at_pin() / 3.3 * 4096
        print(f"   design A: if a wiring slip puts 12 V on CS, the clamp carries {SENSE_DESIGNS[0].clamp_current_ma(12.0):.2f} mA")
        print(f"   design A resolution: {counts_per_amp:.0f} ADC counts per amp (12-bit), {1000 / counts_per_amp:.2f} mA per count")
        out["sense"] = sense

    if args.section in ("brake", "all"):
        mass, v = 1.6, 0.77
        w_motor = v / 0.045 * 56
        emf = K_M * w_motor
        e_kin = 0.5 * mass * v ** 2
        brake_i = emf / (R_M + R_DS_HS_LS)
        plug_i = (12.0 + emf) / (R_M + R_DS_HS_LS)
        print(f"\n4) Braking from {v} m/s: motor at {w_motor:.0f} rad/s, back-EMF {emf:.1f} V, kinetic energy {e_kin:.2f} J")
        print(f"   brake (short) initial current {brake_i:.2f} A per motor; reverse at full speed would need {plug_i:.2f} A "
              f"(the driver limits it to {i_trip(3.3):.2f} A)")
        for frac in (0.25, 0.5):
            for c in (470e-6, 2 * 470e-6):
                vp = bus_pump_voltage(12.6, e_kin, c, frac)
                print(f"   battery disconnected, {int(frac * 100)} % of the energy returned into {c * 1e6:.0f} uF: bus rises to {vp:.1f} V "
                      f"({'exceeds' if vp > 40 else 'below'} DRV8874 40 V abs max)")
        out["brake"] = {"emf": emf, "e_kin": e_kin, "brake_i": brake_i, "plug_i": plug_i,
                        "pump_50pct_470uF": bus_pump_voltage(12.6, e_kin, 470e-6, 0.5)}
    return out


if __name__ == "__main__":
    main()
