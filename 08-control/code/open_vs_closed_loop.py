"""08.01 — Open loop vs closed loop: why equal duty doesn't drive straight.

    python 08-control/code/open_vs_closed_loop.py --plot open_vs_closed.png
    python 08-control/code/open_vs_closed_loop.py --port /dev/ttyACM0      # real robot, wheels in the air

Three 3-second runs, each commanding "both wheels the same":

1. open loop     : duty 0.5 on both motors (set_wheel_duty) — no measurement at all
2. sync feedback : a 3-line loop on the Pi that watches the TICK DIFFERENCE and trims the right duty
3. firmware loop : set_wheel_velocity() — the Pico's own speed controller holds each wheel's speed

Then the open-loop run again with a tired battery, to show the second weakness of open loop.
In the simulator it prints where the robot ended up; on the robot it prints the tick counts.
"""

from __future__ import annotations

import argparse
import math

from control_lab import Target, add_target_args, is_sim, let_wheels_stop, plt_headless, run_loop

DUTY = 0.5
SECONDS = 3.0
SYNC_GAIN = 0.002  # duty per tick of difference


def open_loop(t, dt, state, extra):
    return DUTY, DUTY


def make_sync_loop():
    """The smallest useful feedback loop: setpoint = 'no tick difference'."""
    start: dict[str, int] = {}

    def step(t, dt, state, extra):
        if not start:
            start.update(left=state.left_ticks, right=state.right_ticks)
        left_turned = state.left_ticks - start["left"]
        right_turned = state.right_ticks - start["right"]
        error = left_turned - right_turned  # setpoint 0 minus measurement (right - left)
        correction = SYNC_GAIN * error
        extra["tick_error"] = error
        return DUTY, DUTY + correction  # right behind -> more right duty

    return step


def main(argv: list[str] | None = None) -> dict[str, dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    args = ap.parse_args(argv)

    results: dict[str, dict[str, float]] = {}
    paths: dict[str, list[tuple[float, float]]] = {}
    with Target(args) as base:
        # the same average wheel speed the open-loop run reaches, for the firmware-loop run
        runs = [
            ("open loop, duty 0.5", open_loop, "duty"),
            ("sync feedback on ticks", make_sync_loop(), "duty"),
        ]
        for name, step, command in runs + [("firmware speed loop", None, "velocity")]:
            if step is None:  # command the average speed the open-loop run actually reached
                ol = results["open loop, duty 0.5"]
                target = (ol["left_rad_s"] + ol["right_rad_s"]) / 2.0
                step = lambda t, dt, s, e, w=target: (w, w)  # noqa: E731
            if is_sim(base):
                base.sim.reset((0.0, 0.0, 0.0))
            track: list[tuple[float, float]] = []

            def logged(t, dt, state, extra, step=step, track=track):
                if is_sim(base):
                    track.append((base.sim.pose.x, base.sim.pose.y))
                return step(t, dt, state, extra)

            log = run_loop(base, logged, SECONDS, command=command)
            let_wheels_stop(base)
            t = log["t"]
            late = t > SECONDS - 1.0
            r = {
                "left_ticks": log["left_ticks"][-1] - log["left_ticks"][0],
                "right_ticks": log["right_ticks"][-1] - log["right_ticks"][0],
                "left_rad_s": float(log["left_rad_s"][late].mean()),
                "right_rad_s": float(log["right_rad_s"][late].mean()),
            }
            if is_sim(base):
                p = base.sim.pose
                r.update(x=p.x, y=p.y, heading_deg=math.degrees(p.theta))
                paths[name] = track
            results[name] = r

        # open loop again, with a tired battery: same command, different robot
        if is_sim(base):
            from dataclasses import replace

            sim = base.sim
            sim.params = replace(sim.params, battery_initial_soc=0.1)
            sim.reset((0.0, 0.0, 0.0))
            log = run_loop(base, open_loop, SECONDS)
            let_wheels_stop(base)
            late = log["t"] > SECONDS - 1.0
            results["open loop, battery at 10%"] = {
                "left_ticks": log["left_ticks"][-1] - log["left_ticks"][0],
                "right_ticks": log["right_ticks"][-1] - log["right_ticks"][0],
                "left_rad_s": float(log["left_rad_s"][late].mean()),
                "right_rad_s": float(log["right_rad_s"][late].mean()),
                "x": sim.pose.x, "y": sim.pose.y, "heading_deg": math.degrees(sim.pose.theta),
            }

    print(f"{'run':28s} {'L ticks':>8s} {'R ticks':>8s} {'R/L':>6s} {'L rad/s':>8s} {'R rad/s':>8s}", end="")
    print(f" {'x [m]':>6s} {'y [m]':>6s} {'heading':>8s}" if paths else "")
    for name, r in results.items():
        ratio = r["right_ticks"] / r["left_ticks"] if r["left_ticks"] else math.nan
        line = (f"{name:28s} {r['left_ticks']:8.0f} {r['right_ticks']:8.0f} {ratio:6.3f} "
                f"{r['left_rad_s']:8.2f} {r['right_rad_s']:8.2f}")
        if "x" in r:
            line += f" {r['x']:6.2f} {r['y']:+6.3f} {r['heading_deg']:+7.1f} deg"
        print(line)

    if args.plot and paths:
        plt = plt_headless()
        fig, ax = plt.subplots(figsize=(7, 3.6))
        for name, track in paths.items():
            xs, ys = zip(*track)
            ax.plot(xs, ys, label=name)
        ax.axhline(0.0, color="k", lw=0.8, ls=":")
        ax.set_xlabel("x [m]  (the robot starts at 0,0 facing +x)")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True)
        ax.legend(loc="lower left", fontsize=8)
        ax.set_title("Same command to both wheels, three ways (realistic simulator, 3 s)")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
