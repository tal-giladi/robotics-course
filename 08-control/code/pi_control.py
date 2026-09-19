"""08.05 — Integral control: the steady-state error disappears, then integral windup bites.

    python 08-control/code/pi_control.py                 # uses YOUR labs/exercises/08.05/student.py
    python 08-control/code/pi_control.py --solution --plot pi_control.png
    python 08-control/code/pi_control.py --port /dev/ttyACM0      # real robot, WHEELS IN THE AIR

Part 1  Step 0 -> 8 rad/s: P alone (ki = 0) and PI with ki = 0.5, 2 and 8.
Part 2  Windup: ask for 25 rad/s for 2 s (more than the motor can do: the duty saturates at 1.0),
        then 6 rad/s. With and without anti-windup. Watch the integral.
"""

from __future__ import annotations

import argparse

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, load_exercise, oscillation_std, overshoot_percent, plt_headless, run_loop,
    settling_time, steady_value,
)

KP = 0.1


def run(base, controller, profile, seconds: float):
    controller.reset()

    def step(t, dt, state, extra):
        setpoint = profile(t)
        duty = controller.update(setpoint, state.left_rad_s, dt)
        extra.update(setpoint=setpoint, integral=controller.integral)
        return duty, 0.0

    log = run_loop(base, step, seconds)
    let_wheels_stop(base)
    if is_sim(base):
        base.sim.reset((0.0, 0.0, 0.0))
    return log


def step_profile(t: float) -> float:
    return 8.0 if t >= 0.2 else 0.0


def windup_profile(t: float) -> float:
    return 0.0 if t < 0.2 else 25.0 if t < 2.2 else 6.0


def main(argv: list[str] | None = None) -> dict[str, dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--ki", type=float, nargs="+", default=[0.0, 0.5, 2.0, 8.0])
    ap.add_argument("--solution", action="store_true", help="use the reference solution")
    args = ap.parse_args(argv)
    ex = load_exercise("08.05", solution=args.solution)
    results: dict[str, dict[str, float]] = {}
    step_logs, windup_logs = {}, {}

    with Target(args) as base:
        print("Part 1 - step 0 -> 8 rad/s, kp = 0.1")
        print(f"  {'ki':>5s} {'settled':>8s} {'overshoot':>9s} {'settling (+-0.2)':>16s} {'wobble':>7s}")
        for ki in args.ki:
            log = run(base, ex.PIController(KP, ki), step_profile, 3.0)
            t, w = log["t"], log["left_rad_s"]
            r = {"settled": steady_value(t, w, 1.0), "overshoot": overshoot_percent(w, 0.0, 8.0),
                 "settling": settling_time(t, w, 8.0, 0.2, 0.2), "wobble": oscillation_std(t, w, 1.0)}
            results[f"ki={ki}"] = r
            step_logs[ki] = log
            settling = "never" if r["wobble"] > 0.5 or r["settling"] == float("inf") else f"{r['settling']:.2f} s"
            print(f"  {ki:5.1f} {r['settled']:8.2f} {r['overshoot']:8.0f}% {settling:>15s} {r['wobble']:7.2f}")

        print("\nPart 2 - 25 rad/s (unreachable) for 2 s, then 6 rad/s; kp = 0.1, ki = 2")
        print(f"  {'anti-windup':12s} {'integral at 2.2 s':>17s} {'speed at 2.1 s':>14s} {'recovery (+-0.5)':>16s} {'speed 0.3 s later':>18s}")
        for aw in (False, True):
            log = run(base, ex.PIController(KP, 2.0, anti_windup=aw), windup_profile, 5.0)
            t, w, integral = log["t"], log["left_rad_s"], log["integral"]
            k = int((t < 2.2).sum()) - 1
            r = {"integral": float(integral[k]), "speed_saturated": float(w[int((t < 2.1).sum())]),
                 "recovery": settling_time(t, w, 6.0, 0.5, 2.2),
                 "speed_after": float(w[int((t < 2.5).sum())])}
            results[f"anti_windup={aw}"] = r
            windup_logs[aw] = log
            print(f"  {str(aw):12s} {r['integral']:17.2f} {r['speed_saturated']:14.2f} {r['recovery']:14.2f} s {r['speed_after']:15.2f} rad/s")

    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        ax = axes[0]
        for ki, log in step_logs.items():
            ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=("P only" if ki == 0 else f"PI, ki = {ki}"))
        ax.plot(log["t"], log["setpoint"], "k--", lw=1, label="setpoint")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="1. adding integral (kp = 0.1)", xlim=(0, 2))
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[1]
        for aw, style in ((False, "-"), (True, "-")):
            log = windup_logs[aw]
            ax.plot(log["t"], log["left_rad_s"], style, lw=1.4, label=f"speed, anti-windup {'on' if aw else 'off'}")
        ax.plot(log["t"], log["setpoint"], "k--", lw=1, label="setpoint")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="2. windup: saturated for 2 s", ylim=(-1, 26))
        twin = ax.twinx()
        for aw, style in ((False, ":"), (True, ":")):
            twin.plot(windup_logs[aw]["t"], windup_logs[aw]["integral"], style, lw=1.5,
                      label=f"integral, anti-windup {'on' if aw else 'off'}")
        twin.set_ylabel("integral term [duty]")
        lines = ax.get_legend_handles_labels()
        lines2 = twin.get_legend_handles_labels()
        ax.legend(lines[0] + lines2[0], lines[1] + lines2[1], fontsize=8, loc="upper right")
        for a in axes:
            a.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
