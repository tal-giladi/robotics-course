"""09.05 — Record UMBmark (and straight/spin) runs, then calibrate from the log.

Record (simulator, realistic preset: the hidden truth is DiffDriveParams.realistic()):
    python 09-odometry/code/umbmark_run.py record --out my_runs.json
    python 09-odometry/code/umbmark_run.py record --out my_runs.json --extra        # + 3 straight, 3 spin runs
    python 09-odometry/code/umbmark_run.py record --out after.json --radius-left 0.0448 --radius-right 0.0454 --separation 0.2078

Record on karmel (on the Pi; you measure the final pose of each run and type it in):
    python 09-odometry/code/umbmark_run.py record --port /dev/ttyACM0 --side 1.5 --out karmel_runs.json --extra

Analyze (uses YOUR labs/exercises/09.05/student.py; --solution for the reference):
    python 09-odometry/code/umbmark_run.py analyze my_runs.json --solution

The log format is the one of labs/exercises/09.05/logged_runs.json.

SAFETY (real robot): a 1.5 m square needs about 2 m x 2 m of clear floor. Nobody (pets,
children, feet) inside the square while it drives. Keep the power switch within reach.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from course_code import WheelOdometry, drive_segments, load_exercise, open_base
from robotlab.config import load_config


def plans(side: float, runs: int, extra: bool) -> list[tuple[str, list[tuple[str, float]]]]:
    out = []
    for kind, sign in (("cw", -1.0), ("ccw", 1.0)):
        out += [(kind, [("line", side), ("turn", sign * math.pi / 2)] * 4)] * runs
    if extra:
        out += [("straight", [("line", side + 0.5)])] * 3
        out += [("spin", [("turn", sign * 4 * math.pi)]) for sign in (1.0, -1.0, 1.0)]
    return out


def ask_pose(name: str) -> tuple[float, float, float]:
    print(f"[{name}] measure the robot's final pose relative to the start mark (x forward, y left).")
    x = float(input("  x [m]: "))
    y = float(input("  y [m]: "))
    heading_deg = float(input("  heading [deg, + = turned left]: "))
    return x, y, math.radians(heading_deg)


def record(args: argparse.Namespace) -> None:
    cfg = load_config()
    d = cfg.drive
    radius_left = args.radius_left or d.wheel_radius_m
    radius_right = args.radius_right or d.wheel_radius_m
    separation = args.separation or d.wheel_separation_m
    runs = []
    counts: dict[str, int] = {}
    base = open_base(args.port, realistic=True, seed=args.seed)
    sim = getattr(base, "sim", None)
    try:
        for i, (kind, segments) in enumerate(plans(args.side, args.runs, args.extra)):
            counts[kind] = counts.get(kind, 0) + 1
            name = f"{kind}-{counts[kind]}"
            if sim is not None:
                sim.reset((0.0, 0.0, 0.0))  # "put the robot back on the start mark"
                sim.rng = __import__("numpy").random.default_rng(args.seed + i)
            else:
                input(f"[{name}] place the robot on the start mark, facing +x, then press Enter")
                base.reset_encoders()
            ticks: list[list[int]] = []
            odom = WheelOdometry(radius_left, radius_right, separation, d.ticks_per_wheel_rev)
            drive_segments(base, odom, segments, on_state=lambda s: ticks.append([s.left_ticks, s.right_ticks]))
            base.stop()
            if sim is not None:
                p = sim.pose
                measured = (p.x, p.y, p.theta)
                turns = round((odom.heading - p.theta) / (2 * math.pi))
                heading_change = p.theta + 2 * math.pi * turns
            else:
                measured = ask_pose(name)
                turns = round((odom.heading - measured[2]) / (2 * math.pi))
                heading_change = measured[2] + 2 * math.pi * turns
            start = ticks[0]
            run = {
                "name": name,
                "kind": kind,
                "ticks": [[left - start[0], right - start[1]] for left, right in ticks],
                "measured_pose": [round(v, 5) for v in measured],
            }
            if kind == "straight":
                run["measured_distance_m"] = round(math.hypot(measured[0], measured[1]), 5)
            if kind == "spin":
                run["measured_heading_change_rad"] = round(heading_change, 5)
            runs.append(run)
            ox, oy, oth = odom.pose
            print(f"{name:10s} odometry ({ox:+.3f}, {oy:+.3f}, {math.degrees(oth):+6.1f} deg)  "
                  f"measured ({measured[0]:+.3f}, {measured[1]:+.3f}, {math.degrees(measured[2]):+6.1f} deg)")
    finally:
        base.stop()
        base.close()
    data = {
        "description": "recorded by 09-odometry/code/umbmark_run.py" + (" in the simulator" if sim else " on the robot"),
        "ticks_per_rev": d.ticks_per_wheel_rev,
        "nominal": {"wheel_radius_m": d.wheel_radius_m, "wheel_separation_m": d.wheel_separation_m},
        "driven_with": {"radius_left_m": radius_left, "radius_right_m": radius_right, "separation_m": separation},
        "square_side_m": args.side,
        "sample_rate_hz": round(1.0 / base.dt) if sim is not None else cfg.serial.telemetry_hz,
        "runs": runs,
    }
    Path(args.out).write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    print(f"wrote {args.out}")


def analyze(args: argparse.Namespace) -> None:
    ex = load_exercise("09.05", solution=args.solution)
    data = json.loads(Path(args.log).read_text(encoding="utf-8"))
    tpr = data["ticks_per_rev"]
    driven = data.get("driven_with")
    if driven:
        params = (driven["radius_left_m"], driven["radius_right_m"], driven["separation_m"])
    else:
        params = (data["nominal"]["wheel_radius_m"], data["nominal"]["wheel_radius_m"], data["nominal"]["wheel_separation_m"])

    def umbmark(p):
        errors: dict[str, list[tuple[float, float]]] = {"cw": [], "ccw": []}
        for run in data["runs"]:
            if run["kind"] in errors:
                x, y, _ = ex.replay_odometry(run["ticks"], *p, tpr)
                errors[run["kind"]].append((run["measured_pose"][0] - x, run["measured_pose"][1] - y))
        return ex.umbmark_summary(errors["cw"], errors["ccw"])

    def show(label, p):
        u = umbmark(p)
        print(f"{label:22s} r_L={p[0] * 1000:.2f} mm r_R={p[1] * 1000:.2f} mm b={p[2] * 1000:.1f} mm | "
              f"cg_cw=({u.cg_cw[0]:+.3f}, {u.cg_cw[1]:+.3f}) cg_ccw=({u.cg_ccw[0]:+.3f}, {u.cg_ccw[1]:+.3f}) "
              f"E_max,syst={u.e_max_syst * 100:.1f} cm")
        return u

    before = show("as driven", params)
    c_left, c_right, e_b = ex.borenstein_correction(before.cg_cw[0], before.cg_ccw[0], data["square_side_m"], params[2])
    print(f"Borenstein: E_d = c_R/c_L = {c_right / c_left:.4f}, E_b = {e_b:.4f}")
    show("Borenstein (replayed)", (params[0] * c_left, params[1] * c_right, params[2] * e_b))

    kinds = {run["kind"] for run in data["runs"]}
    if {"straight", "spin"} <= kinds:
        straight = [r for r in data["runs"] if r["kind"] == "straight"]
        angles = [(r["ticks"][-1][0] + r["ticks"][-1][1]) / 2 * 2 * math.pi / tpr for r in straight]
        radius = ex.fit_wheel_radius(angles, [r["measured_distance_m"] for r in straight])
        spins = [r for r in data["runs"] if r["kind"] == "spin"]
        left = [r["ticks"][-1][0] * 2 * math.pi / tpr * radius for r in spins]
        right = [r["ticks"][-1][1] * 2 * math.pi / tpr * radius for r in spins]
        separation = ex.fit_wheel_separation(left, right, [r["measured_heading_change_rad"] for r in spins])
        show("tape + spin fits", (radius, radius, separation))
    runs = [(r["ticks"], tuple(r["measured_pose"])) for r in data["runs"]]
    found = ex.calibrate_least_squares(runs, tpr, params)
    show("least squares (all)", found)
    if not {"straight", "spin"} <= kinds:
        print("note: without straight runs the overall scale is poorly determined (record with --extra)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--out", required=True)
    rec.add_argument("--port", help="real robot serial port; omit for the simulator")
    rec.add_argument("--side", type=float, default=2.0, help="square side (m); use 1.5 at home")
    rec.add_argument("--runs", type=int, default=5, help="runs per direction")
    rec.add_argument("--extra", action="store_true", help="also 3 straight runs and 3 spins")
    rec.add_argument("--seed", type=int, default=0)
    rec.add_argument("--radius-left", type=float)
    rec.add_argument("--radius-right", type=float)
    rec.add_argument("--separation", type=float)
    rec.set_defaults(func=record)
    ana = sub.add_parser("analyze")
    ana.add_argument("log")
    ana.add_argument("--solution", action="store_true")
    ana.set_defaults(func=analyze)
    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
