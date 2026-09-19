"""08.03 — Wheel speed from encoder ticks: sample period, moving average, low-pass (EMA), lag, jitter.

    python 08-control/code/wheel_speed_estimation.py --plot speed_estimation.png
    python 08-control/code/wheel_speed_estimation.py --port /dev/ttyACM0   # real robot, WHEELS IN THE AIR

Part 1  One wheel runs 1.5 s at constant duty, then a 1 Hz speed wave, in the simulator; its encoder
        is read every 1 ms. Differentiate the ticks with sample periods of 1, 2, 5, 10, 20 and 50 ms and
        compare each estimate with the TRUE wheel speed: noise (constant part) vs. lag (wave).
Part 2  Filter the 10 ms estimate (what the Pico computes) with moving averages and first-order
        low-pass filters (EMA). Measure noise, lag and total error for each.
Part 3  The Pi's view: 50 Hz telemetry arrives with a few ms of jitter and the odd lost line.
        Divide by the right dt: the robot's timestamps, not a constant and not the Pi's clock.
On the real robot (--port) there is no ground truth: it prints the firmware's estimate next to a raw
20 ms difference of the telemetry ticks (using the robot's timestamps) and a low-passed version.
"""

from __future__ import annotations

import argparse
import math
from collections import deque

import numpy as np

from control_lab import Target, add_target_args, plt_headless, run_loop
from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

TWO_PI = 2.0 * math.pi


class DifferenceEstimator:
    """speed = (ticks - previous ticks) * 2*pi / ticks_per_rev / dt — the raw estimate."""

    def __init__(self, ticks_per_rev: int) -> None:
        self.rad_per_tick = TWO_PI / ticks_per_rev
        self._last: int | None = None

    def update(self, ticks: int, dt: float) -> float:
        if self._last is None:
            self._last = ticks
            return 0.0
        speed = (ticks - self._last) * self.rad_per_tick / dt
        self._last = ticks
        return speed


class MovingAverage:
    """Mean of the last ``n`` inputs. Lag = (n - 1) / 2 samples."""

    def __init__(self, n: int) -> None:
        self.window: deque[float] = deque(maxlen=n)

    def update(self, x: float) -> float:
        self.window.append(x)
        return sum(self.window) / len(self.window)


class LowPass:
    """First-order low-pass / exponential moving average, exactly like WheelSpeedEstimator in
    labs/firmware/pico/velocity.py:  y += alpha * (x - y).  Time constant tau = dt * (1 - alpha) / alpha."""

    def __init__(self, alpha: float) -> None:
        self.alpha = alpha
        self.y: float | None = None

    def update(self, x: float) -> float:
        self.y = x if self.y is None else self.y + self.alpha * (x - self.y)
        return self.y


def duty_profile(t: float) -> float:
    """1.5 s at constant duty (to measure noise), then a 1 Hz wave (to measure lag)."""
    return 0.5 if t < 1.5 else 0.5 + 0.3 * math.sin(TWO_PI * 1.0 * (t - 1.5))


def record_1khz(seconds: float = 3.5, seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t, ticks, true speed) of the left wheel sampled every 1 ms, driven by duty_profile."""
    cfg = load_config()
    sim = DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=seed)
    n = round(seconds / 0.001)
    t, ticks, truth = np.zeros(n), np.zeros(n, dtype=np.int64), np.zeros(n)
    for k in range(n):
        duty = duty_profile(k * 0.001)
        sim.set_duty(duty, duty)
        sim.step(0.001)
        t[k], ticks[k], truth[k] = sim.t, sim.ticks[0], sim.wheel_rad_s[0]
    return t, ticks, truth


def evaluate(ts: np.ndarray, estimate: np.ndarray, t: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    """noise: std at constant speed (1.0-1.5 s).  lag: the delay that best aligns the estimate with the
    truth during the wave (1.7-3.5 s).  rms: total error vs the truth *now* during the wave."""
    still = (ts >= 1.0) & (ts < 1.5)
    wave = ts >= 1.7
    noise = float(np.std(estimate[still]))
    lags = np.arange(0.0, 0.2001, 0.001)
    errors = [np.mean((estimate[wave] - np.interp(ts[wave] - lag, t, truth)) ** 2) for lag in lags]
    lag = float(lags[int(np.argmin(errors))])
    rms = float(np.sqrt(np.mean((estimate[wave] - np.interp(ts[wave], t, truth)) ** 2)))
    return {"noise": noise, "lag_ms": lag * 1000.0, "rms": rms}


def main(argv: list[str] | None = None) -> dict[str, dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    args = ap.parse_args(argv)
    tpr = load_config().drive.ticks_per_wheel_rev
    results: dict[str, dict[str, float]] = {}

    if args.port or args.fake:  # the real robot: 50 Hz telemetry ticks, no ground truth
        with Target(args) as base:
            log = run_loop(base, lambda t, dt, s, e: (duty_profile(t),) * 2, 3.5)
        est, lp = DifferenceEstimator(tpr), LowPass(0.3)
        raw = [est.update(int(k), dt) for k, dt in zip(log["left_ticks"], np.diff(log["t"], prepend=log["t"][0] - 0.02))]
        smooth = [lp.update(x) for x in raw]
        print("t [s]   firmware estimate   raw 20 ms difference   EMA(0.3)")
        for i in range(0, len(raw), 10):
            print(f"{log['t'][i]:5.2f}   {log['left_rad_s'][i]:8.2f}            {raw[i]:8.2f}              {smooth[i]:8.2f}")
        return results

    t, ticks, truth = record_1khz(seed=args.seed)

    print("Part 1 - raw tick-difference estimate vs sample period (left wheel)")
    print(f"  {'period':>7s} {'1 tick =':>9s} {'noise':>7s} {'lag':>6s} {'total rms':>10s}")
    part1 = {}
    for period_ms in (1, 2, 5, 10, 20, 50):
        ts, tk = t[period_ms - 1 :: period_ms], ticks[period_ms - 1 :: period_ms]
        est = DifferenceEstimator(tpr)
        e = np.array([est.update(int(k), period_ms / 1000.0) for k in tk])
        m = evaluate(ts, e, t, truth)
        part1[period_ms] = (ts, e)
        results[f"raw {period_ms} ms"] = m
        print(f"  {period_ms:5d}ms {TWO_PI / tpr / (period_ms / 1000):7.3f}   {m['noise']:6.3f} {m['lag_ms']:4.0f}ms "
              f"{m['rms']:9.3f}   rad/s")

    print("\nPart 2 - filtering the 10 ms raw estimate (what the Pico computes)")
    t10, raw10 = part1[10]
    print(f"  {'filter':16s} {'noise':>7s} {'lag':>6s} {'total rms':>10s}")
    filters = {"none": None, "MA 2": MovingAverage(2), "MA 5": MovingAverage(5), "MA 10": MovingAverage(10),
               "EMA alpha 0.5": LowPass(0.5), "EMA alpha 0.2": LowPass(0.2), "EMA alpha 0.05": LowPass(0.05)}
    filtered = {}
    for name, f in filters.items():
        y = raw10 if f is None else np.array([f.update(float(x)) for x in raw10])
        filtered[name] = y
        m = evaluate(t10, y, t, truth)
        results[name] = m
        print(f"  {name:16s} {m['noise']:7.3f} {m['lag_ms']:4.0f}ms {m['rms']:10.3f}   rad/s")

    print("\nPart 3 - speed on the Pi from 50 Hz telemetry: 3 ms arrival jitter, 3% of lines lost")
    rng = np.random.default_rng(args.seed)
    robot_t = np.arange(0.019, 3.499, 0.02)  # the Pico samples ticks every 20 ms on ITS clock
    kept = rng.random(len(robot_t)) > 0.03  # a few lines lost on the wire
    robot_t = robot_t[kept]
    arrival_t = robot_t + 0.002 + rng.normal(0.0, 0.003, len(robot_t))  # when the Pi's thread sees them
    k_ticks = ticks[np.round(robot_t * 1000).astype(int)]
    for name, times in (("dt = nominal 20 ms", None), ("dt from the Pi's clock", arrival_t),
                        ("dt from the robot's stamps", robot_t)):
        est = DifferenceEstimator(tpr)
        e = np.array([est.update(int(k), 0.02 if times is None or i == 0 else times[i] - times[i - 1])
                      for i, k in enumerate(k_ticks)])
        # compare with the true average speed over each interval (removes the unavoidable T/2 lag)
        true_avg = np.diff(np.interp(robot_t, t, np.cumsum(truth) * 0.001)) / np.diff(robot_t)
        err = float(np.sqrt(np.mean((e[1:] - true_avg) ** 2)))
        worst = float(np.max(np.abs(e[1:] - true_avg)))
        results[name] = {"rms": err, "worst": worst}
        print(f"  {name:28s} rms error {err:5.2f} rad/s, worst {worst:5.2f} rad/s")

    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        window = (t >= 2.1) & (t <= 2.9)
        ax = axes[0]
        ax.plot(t[window], truth[window], "k-", lw=2.5, label="true speed")
        for period_ms in (2, 10, 50):
            ts, e = part1[period_ms]
            w = (ts >= 2.1) & (ts <= 2.9)
            m = results[f"raw {period_ms} ms"]
            ax.step(ts[w], e[w], where="post", lw=1,
                    label=f"raw, {period_ms} ms: noise {m['noise']:.2f}, lag {m['lag_ms']:.0f} ms")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="1. differentiating ticks")
        ax.legend(fontsize=8, loc="upper left")
        ax = axes[1]
        w10 = (t10 >= 2.1) & (t10 <= 2.9)
        ax.plot(t[window], truth[window], "k-", lw=2.5, label="true speed")
        for name in ("none", "MA 5", "EMA alpha 0.5", "EMA alpha 0.05"):
            m = results[name]
            ax.plot(t10[w10], filtered[name][w10], lw=1.2,
                    label=f"{name}: noise {m['noise']:.2f}, lag {m['lag_ms']:.0f} ms")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="2. filtering the 10 ms estimate")
        ax.legend(fontsize=8, loc="upper left")
        for a in axes:
            a.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
