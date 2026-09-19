"""08.02 — Measure a motor: duty sweep (deadband, gain), step response (time constant), first-order model.

    python 08-control/code/motor_step_response.py --plot step_response.png --csv step.csv
    python 08-control/code/motor_step_response.py --port /dev/ttyACM0       # real robot, WHEELS IN THE AIR

Part 1  sweep: hold each duty 0.00, 0.02, ... 1.00 for 0.6 s, record the settled wheel speed.
        Fit  speed = slope * (duty - deadband)  above the deadband.
Part 2  step: duty 0 -> STEP_DUTY at t = 0.2 s, log every telemetry sample for 1 s.
        Fit  w(t) = w_final * (1 - exp(-(t - t_step) / tau))  two ways:
          a) the 63 % rule      b) least squares (scipy.optimize.curve_fit)
        and a third way that bypasses the speed estimate's own lag: fit the TICKS (wheel angle).
Part 3  validate: run the fitted model on a different duty sequence and compare with the robot.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import curve_fit

from control_lab import Log, Target, add_target_args, let_wheels_stop, plt_headless, run_loop
from robotlab.config import load_config

STEP_DUTY = 0.6
T_STEP = 0.2


@dataclass(frozen=True)
class MotorModel:
    """First-order motor with a deadband:  tau * dw/dt = -w + slope * dz(u),  dz(u) = sign(u) * max(|u| - deadband, 0)."""

    slope_rad_s_per_duty: float
    deadband: float
    tau_s: float

    def steady_speed(self, duty: float) -> float:
        return math.copysign(max(abs(duty) - self.deadband, 0.0), duty) * self.slope_rad_s_per_duty

    def simulate(self, t: np.ndarray, duty: np.ndarray, w0: float = 0.0) -> np.ndarray:
        """Exact discretization per sample, duty held constant between samples (zero-order hold)."""
        w = np.empty_like(t)
        w[0] = w0
        for k in range(1, len(t)):
            a = 1.0 - math.exp(-(t[k] - t[k - 1]) / self.tau_s)
            w[k] = w[k - 1] + a * (self.steady_speed(duty[k - 1]) - w[k - 1])
        return w


def sweep(base, duties: np.ndarray, hold_s: float = 0.6) -> tuple[np.ndarray, np.ndarray]:
    """Settled (left, right) speed for each duty."""
    left, right = [], []
    for d in duties:
        log = run_loop(base, lambda t, dt, s, e, d=d: (d, d), hold_s)
        late = log["t"] >= hold_s - 0.2
        left.append(log["left_rad_s"][late].mean())
        right.append(log["right_rad_s"][late].mean())
    let_wheels_stop(base)
    return np.array(left), np.array(right)


def fit_static(duties: np.ndarray, speeds: np.ndarray, moving: float = 0.3) -> tuple[float, float]:
    """(deadband, slope): a straight line through the points where the wheel clearly moves."""
    mask = np.abs(speeds) > moving
    slope, intercept = np.polyfit(duties[mask], speeds[mask], 1)
    return -intercept / slope, slope  # the line crosses zero speed at duty = deadband


def fit_tau_63(t: np.ndarray, w: np.ndarray, t_step: float) -> tuple[float, float]:
    """(w_final, tau) with the 63 % rule: tau = time to reach 63.2 % of the final change."""
    w_final = w[t >= t[-1] - 0.2].mean()
    after = (t >= t_step) & (w >= 0.632 * w_final)
    t63 = t[np.argmax(after)]
    # interpolate between the two samples around the crossing for sub-sample resolution
    k = int(np.argmax(after))
    if k > 0 and w[k] != w[k - 1]:
        t63 = t[k - 1] + (0.632 * w_final - w[k - 1]) / (w[k] - w[k - 1]) * (t[k] - t[k - 1])
    return float(w_final), float(t63 - t_step)


def step_model(t, w_final, tau, t0):
    return np.where(t < t0, 0.0, w_final * (1.0 - np.exp(-(t - t0) / tau)))


def angle_model(t, w_final, tau, t0):
    """Integral of step_model: wheel angle after the step."""
    s = np.maximum(t - t0, 0.0)
    return w_final * (s - tau * (1.0 - np.exp(-s / tau)))


def main(argv: list[str] | None = None) -> dict[str, float]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--csv", help="save the step-response log as CSV")
    ap.add_argument("--quick", action="store_true", help="coarser sweep (for tests)")
    args = ap.parse_args(argv)
    cfg = load_config()
    rad_per_tick = 2 * math.pi / cfg.drive.ticks_per_wheel_rev

    duties = np.round(np.arange(0.0, 1.0001, 0.1 if args.quick else 0.02), 3)
    with Target(args) as base:
        sweep_left, sweep_right = sweep(base, duties)
        step_log: Log = run_loop(base, lambda t, dt, s, e: (STEP_DUTY if t >= T_STEP - 1e-9 else 0.0,) * 2, 1.2)
        let_wheels_stop(base)
        # validation: a staircase the fit never saw
        def stairs(t, dt, s, e):
            d = 0.35 if t < 0.6 else 0.9 if t < 1.2 else 0.5 if t < 1.8 else 0.25
            return d, d
        val_log = run_loop(base, stairs, 2.4)
        let_wheels_stop(base)
        voltage = float(np.nanmean(step_log["battery_v"]))

    results: dict[str, float] = {"battery_v": voltage}
    print(f"battery during the test: {voltage:.2f} V")
    print("\nPart 1 - duty sweep (settled speed, rad/s)")
    for name, speeds in (("left", sweep_left), ("right", sweep_right)):
        deadband, slope = fit_static(duties, speeds)
        results[f"{name}_deadband"], results[f"{name}_slope"] = deadband, slope
        print(f"  {name:5s}: deadband {deadband:.3f} duty, slope {slope:.2f} rad/s per duty, "
              f"speed at duty 1.0 = {speeds[-1]:.2f} rad/s")
    print(f"  karmel.yaml says: deadband {cfg.drive.duty_deadband}, "
          f"max {cfg.drive.max_wheel_speed_rad_s} rad/s -> slope {cfg.drive.max_wheel_speed_rad_s / (1 - cfg.drive.duty_deadband):.2f}")

    t = step_log["t"]
    w = step_log["left_rad_s"]
    w_final, tau63 = fit_tau_63(t, w, T_STEP)
    (w_fit, tau_fit, t0_fit), _ = curve_fit(step_model, t, w, p0=(w_final, 0.1, T_STEP))
    angle = (step_log["left_ticks"] - step_log["left_ticks"][0]) * rad_per_tick
    (wa_fit, taua_fit, t0a_fit), _ = curve_fit(angle_model, t, angle, p0=(w_final, 0.1, T_STEP))
    results.update(w_final=w_final, tau_63=tau63, tau_lsq=tau_fit, delay_lsq=t0_fit - T_STEP, tau_ticks=taua_fit,
                   delay_ticks=t0a_fit - T_STEP)
    print(f"\nPart 2 - step 0 -> {STEP_DUTY} duty (left wheel)")
    print(f"  final speed {w_final:.2f} rad/s")
    print(f"  a) 63% rule on the speed estimate : tau = {tau63 * 1000:.0f} ms")
    print(f"  b) least squares on the estimate  : tau = {tau_fit * 1000:.0f} ms, apparent delay {(t0_fit - T_STEP) * 1000:+.0f} ms")
    print(f"  c) least squares on the ticks     : tau = {taua_fit * 1000:.0f} ms, apparent delay {(t0a_fit - T_STEP) * 1000:+.0f} ms")
    print(f"  karmel.yaml motor_time_constant_s : {cfg.drive.motor_time_constant_s * 1000:.0f} ms")

    model = MotorModel(float(results["left_slope"]), float(results["left_deadband"]), float(taua_fit))
    tv, wv = val_log["t"], val_log["left_rad_s"]
    predicted = model.simulate(tv, val_log["left_cmd"])
    rms = float(np.sqrt(np.mean((predicted - wv) ** 2)))
    results["validation_rms"] = rms
    print(f"\nPart 3 - model (slope {model.slope_rad_s_per_duty:.2f}, deadband {model.deadband:.3f}, "
          f"tau {model.tau_s * 1000:.0f} ms) on a new staircase: RMS error {rms:.2f} rad/s "
          f"({rms / np.max(np.abs(wv)) * 100:.1f}% of the top speed)")

    if args.csv:
        step_log.save_csv(args.csv)
        print(f"saved {args.csv}")
    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
        ax = axes[0]
        ax.plot(duties, sweep_left, "o-", ms=3, label="left")
        ax.plot(duties, sweep_right, "s-", ms=3, label="right")
        d = np.linspace(0, 1, 50)
        ax.plot(d, [MotorModel(results["left_slope"], results["left_deadband"], 0.1).steady_speed(x) for x in d],
                "k:", label="fit (left)")
        ax.axvspan(0, results["left_deadband"], color="grey", alpha=0.2, label="deadband")
        ax.set(xlabel="duty", ylabel="settled speed [rad/s]", title="1. duty sweep")
        ax.legend(fontsize=8)
        ax = axes[1]
        ax.plot(t, w, ".", label="measured (telemetry)")
        ax.plot(t, step_model(t, w_fit, tau_fit, t0_fit), "-", label=f"fit: tau={tau_fit * 1000:.0f} ms")
        ax.axhline(0.632 * w_final, color="grey", ls=":", lw=1)
        ax.axvline(T_STEP + tau63, color="grey", ls=":", lw=1)
        ax.annotate("63%", (T_STEP + tau63, 0.632 * w_final), textcoords="offset points", xytext=(5, -12))
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title=f"2. step 0 -> {STEP_DUTY}")
        ax.legend(fontsize=8)
        ax = axes[2]
        ax.plot(tv, wv, ".", label="measured")
        ax.plot(tv, predicted, "-", label=f"model (RMS {rms:.2f} rad/s)")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="3. validation on a new input")
        ax.legend(fontsize=8)
        for a in axes:
            a.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
