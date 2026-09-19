"""02.01 — turn datasheet tables into checks: logic-level margins and absolute-maximum violations.

Every number below was copied from a manufacturer datasheet (the source is next to it). The
script checks each signal link on karmel the way you should check it by hand:

    noise margin high  NM_H = V_OH,min(driver) - V_IH,min(receiver)   must be > 0
    noise margin low   NM_L = V_IL,max(receiver) - V_OL,max(driver)   must be > 0
    stress             V_high(driver) <= absolute maximum(receiver), powered AND unpowered

Run:  python 02-robot-electronics/code/datasheet_check.py
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class Output:
    """What a driving pin guarantees."""
    name: str
    v_oh_min: float          # guaranteed high level at the rated load
    v_ol_max: float          # guaranteed low level at the rated load
    v_high_max: float        # the highest voltage this output can ever put on the wire
    source: str


@dataclass(frozen=True)
class Input:
    """What a receiving pin needs and survives."""
    name: str
    v_ih_min: float
    v_il_max: float
    abs_max_powered: float
    abs_max_unpowered: float  # when the receiving chip has no supply (USB unplugged, battery on)
    source: str


# --- Outputs ------------------------------------------------------------------------------------
PICO_GPIO_OUT = Output("Pico 2 GPIO (IOVDD 3.3 V, 4 mA)", 2.62, 0.50, 3.3,
                       "RP2350 datasheet, table 'Digital IO characteristics'")
I2C_OPEN_DRAIN_3V3 = Output("I2C line, pull-ups to 3.3 V (INA219 sinks 3 mA)", 3.3, 0.40, 3.3,
                            "INA219 datasheet: VOL 0.4 V max at 3 mA; high level = pull-up voltage")
US100_ECHO_3V3 = Output("US-100 echo, VCC = 3.3 V", 3.0, 0.3, 3.3,
                        "no official datasheet; output follows VCC (Adafruit 4019). 3.0/0.3 V assumed")
HCSR04_ECHO_5V = Output("HC-SR04 echo, VCC = 5 V", 4.5, 0.4, 5.0,
                        "no official datasheet; 5 V CMOS output assumed")
HCSR04_ECHO_DIVIDED = Output("HC-SR04 echo through 1 k / 2 k divider", 4.5 * 2 / 3, 0.4 * 2 / 3,
                             5.25 * 2 / 3, "divider ratio 2/3; 5.25 V = USB 5 V at +5 %")

# --- Inputs -------------------------------------------------------------------------------------
PICO_GPIO_FT_IN = Input("Pico 2 digital GPIO (FT)", 2.0, 0.8, 5.5, 3.63,
                        "RP2350 datasheet: VIH/VIL at IOVDD 3.3 V; VPIN_FT 5.5 V (IOVDD 3.3 V), 3.63 V (IOVDD 0 V)")
PICO_ADC_PIN_IN = Input("Pico 2 GP26-GP29 (ADC-capable, not FT)", 2.0, 0.8, 3.8, 3.8,
                        "RP2350 datasheet: standard IO abs max IOVDD + 0.5 V; ADC input must not exceed IOVDD")
DRV8874_IN = Input("DRV8874 EN/IN1, PH/IN2, nSLEEP (VM >= 5 V)", 1.5, 0.8, 5.75, 5.75,
                   "TI DRV8874 SLVSF66A: VIH 1.5 V, VIL 0.8 V; logic pins abs max 5.75 V")
INA219_SDA_3V3 = Input("INA219 SDA/SCL (VS = 3.3 V)", 0.7 * 3.3, 0.3 * 3.3, 3.6, 3.6,
                       "TI INA219 SBOS448G: VIH 0.7 VS, VIL 0.3 VS; SCL abs max VS + 0.3 V")


@dataclass(frozen=True)
class LinkResult:
    link: str
    nm_high: float
    nm_low: float
    stress_powered: bool
    stress_unpowered: bool

    @property
    def ok(self) -> bool:
        return self.nm_high > 0 and self.nm_low > 0 and not self.stress_powered and not self.stress_unpowered


def check_link(out: Output, inp: Input) -> LinkResult:
    return LinkResult(
        link=f"{out.name} -> {inp.name}",
        nm_high=round(out.v_oh_min - inp.v_ih_min, 3),
        nm_low=round(inp.v_il_max - out.v_ol_max, 3),
        stress_powered=out.v_high_max > inp.abs_max_powered,
        stress_unpowered=out.v_high_max > inp.abs_max_unpowered,
    )


KARMEL_LINKS: list[tuple[Output, Input]] = [
    (PICO_GPIO_OUT, DRV8874_IN),             # PWM to the motor drivers
    (I2C_OPEN_DRAIN_3V3, PICO_GPIO_FT_IN),   # INA219 / VL53L1X answering the Pico
    (PICO_GPIO_OUT, INA219_SDA_3V3),         # the Pico talking to the INA219
    (US100_ECHO_3V3, PICO_GPIO_FT_IN),       # US-100 at 3.3 V
    (HCSR04_ECHO_5V, PICO_GPIO_FT_IN),       # an HC-SR04 wired straight in (don't)
    (HCSR04_ECHO_DIVIDED, PICO_GPIO_FT_IN),  # the same through the divider
    (HCSR04_ECHO_5V, PICO_ADC_PIN_IN),       # ... or straight into GP26 (much worse)
]


def pwm_timing(freq_hz: float = 20_000, t_pd_s: float = 400e-9, t_rise_s: float = 150e-9,
               t_dead_s: float = 100e-9) -> dict[str, float]:
    """How much of a PWM period the DRV8874's own switching times eat (typical values)."""
    period = 1.0 / freq_hz
    lost = t_pd_s + t_rise_s + t_dead_s
    return {"period_us": period * 1e6, "lost_us": lost * 1e6, "lost_fraction": lost / period,
            "min_useful_duty": 2 * lost / period}


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pwm-hz", type=float, default=20_000)
    args = ap.parse_args(argv)

    results = [check_link(o, i) for o, i in KARMEL_LINKS]
    print(f"{'link':88} {'NM_H':>6} {'NM_L':>6}  stress(pwr/unpwr)  verdict")
    for r in results:
        verdict = "OK" if r.ok else "PROBLEM"
        print(f"{r.link:88} {r.nm_high:6.2f} {r.nm_low:6.2f}  {str(r.stress_powered):5}/{str(r.stress_unpowered):5}      {verdict}")

    t = pwm_timing(args.pwm_hz)
    print(f"\nDRV8874 at {args.pwm_hz / 1000:.0f} kHz: period {t['period_us']:.1f} us, "
          f"switching delays ~{t['lost_us']:.2f} us = {100 * t['lost_fraction']:.2f} % of the period; "
          f"duties below ~{100 * t['min_useful_duty']:.1f} % are mostly edges")
    return {"links": results, "pwm": t}


if __name__ == "__main__":
    main()
