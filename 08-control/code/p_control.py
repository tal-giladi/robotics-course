"""08.04 — Proportional control: hold a wheel speed, watch the steady-state error, then the oscillation.

    python 08-control/code/p_control.py                 # uses YOUR labs/exercises/08.04/student.py
    python 08-control/code/p_control.py --solution      # uses the reference solution
    python 08-control/code/p_control.py --plot p_control.png
    python 08-control/code/p_control.py --port /dev/ttyACM0 --kp 0.05 0.1 0.2   # real robot, WHEELS IN THE AIR

For each gain: setpoint 0 -> 8 rad/s at t = 0.2 s on the left wheel, 3 s, loop at 50 Hz on the Pi.
Run once as is, and once with one extra loop period (20 ms) of delay between computing a command
and the motor receiving it.
"""

from __future__ import annotations

import argparse

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, load_exercise, oscillation_std, plt_headless, run_loop,
    steady_value,
)
from robotlab.config import load_config

SETPOINT = 8.0
T_STEP = 0.2
SECONDS = 3.0


def step_run(base, ex, kp: float, delay_steps: int):
    controller = ex.PController(kp)

    def step(t, dt, state, extra):
        setpoint = SETPOINT if t >= T_STEP - 1e-9 else 0.0
        extra["setpoint"] = setpoint
        return controller.update(setpoint, state.left_rad_s, dt), 0.0

    log = run_loop(base, step, SECONDS, delay_steps=delay_steps)
    let_wheels_stop(base)
    if is_sim(base):
        base.sim.reset((0.0, 0.0, 0.0))  # fresh battery for every run: comparable numbers
    return log


def main(argv: list[str] | None = None) -> dict[tuple[float, int], dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--kp", type=float, nargs="+", default=[0.02, 0.05, 0.1, 0.2, 0.4, 0.8])
    ap.add_argument("--delays", type=int, nargs="+", default=[0, 1], help="extra delay in loop periods")
    ap.add_argument("--solution", action="store_true", help="use the reference solution")
    args = ap.parse_args(argv)
    ex = load_exercise("08.04", solution=args.solution)
    d = load_config().drive
    battery_nominal = load_config().battery.nominal_v

    results: dict[tuple[float, int], dict[str, float]] = {}
    logs = {}
    with Target(args) as base:
        for delay in args.delays:
            print(f"\nextra delay: {delay * 20} ms")
            print(f"  {'kp':>5s} {'loop gain':>9s} {'settled':>8s} {'predicted':>9s} {'error':>6s} {'peak':>6s} {'wobble':>7s}")
            for kp in args.kp:
                log = step_run(base, ex, kp, delay)
                t, w = log["t"], log["left_rad_s"]
                volts = float(log["battery_v"][t > T_STEP].mean())
                slope = d.max_wheel_speed_rad_s / (1 - d.duty_deadband) * volts / battery_nominal
                settled = steady_value(t, w, 1.0)
                r = {
                    "settled": settled,
                    "predicted": ex.steady_state_speed(SETPOINT, kp, slope, d.duty_deadband),
                    "error": SETPOINT - settled,
                    "peak": float(w.max()),
                    "wobble": oscillation_std(t, w, 1.0),
                    "loop_gain": kp * slope,
                }
                results[(kp, delay)] = r
                logs[(kp, delay)] = log
                verdict = "OSCILLATES" if r["wobble"] > 0.5 else ""
                print(f"  {kp:5.2f} {r['loop_gain']:9.2f} {settled:8.2f} {r['predicted']:9.2f} {r['error']:6.2f} "
                      f"{r['peak']:6.2f} {r['wobble']:7.2f}  {verdict}")
    print("\nwobble = std of the speed over the last second (rad/s). predicted = steady_state_speed() "
          "(meaningless once it oscillates)")

    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, len(args.delays), figsize=(6 * len(args.delays), 4), sharey=True, squeeze=False)
        for ax, delay in zip(axes[0], args.delays):
            for kp in args.kp:
                log = logs[(kp, delay)]
                ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=f"kp = {kp}")
            ax.plot(log["t"], log["setpoint"], "k--", lw=1, label="setpoint")
            ax.set(xlabel="time [s]", title=f"P control, extra delay {delay * 20} ms", xlim=(0, 1.6))
            ax.grid(True)
        axes[0][0].set_ylabel("left wheel [rad/s]")
        axes[0][0].legend(fontsize=8, loc="lower right")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
