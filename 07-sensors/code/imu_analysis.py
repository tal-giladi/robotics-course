"""07.05 — IMU analysis: gyro bias and heading drift, accelerometer 6-position and magnetometer calibration.

    py 07-sensors/code/imu_log.py --imu synthetic --seconds 120 --out still.csv   # or --imu bno055 on the Pi
    py 07-sensors/code/imu_analysis.py drift still.csv --plot drift.png
    py 07-sensors/code/imu_analysis.py sixpos px.csv nx.csv py.csv ny.csv pz.csv nz.csv
    py 07-sensors/code/imu_analysis.py mag turning.csv --plot mag.png
    py 07-sensors/code/imu_analysis.py robot --plot robot.png      # course simulator: gyro vs encoders vs truth

CSV columns (imu_log.py): t_s, gx, gy, gz [rad/s], ax, ay, az [m/s²], mx, my, mz [µT].
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT / "labs" / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "labs" / "python"))

G = 9.80665  # standard gravity, m/s²
COLUMNS = ("t_s", "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz")
COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7")
Array = NDArray[np.float64]


# --- recording format ------------------------------------------------------------------------------
def read_log(path: str | Path) -> dict[str, Array]:
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {c: np.array([float(r[c]) if r.get(c) not in (None, "") else math.nan for r in rows]) for c in COLUMNS}


def write_log(path: str | Path, data: dict[str, Array]) -> None:
    n = len(data["t_s"])
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i in range(n):
            w.writerow([f"{data[c][i]:.6f}" for c in COLUMNS])


# --- synthetic IMU (no hardware) --------------------------------------------------------------------
@dataclass(frozen=True)
class ImuErrorModel:
    """Illustrative MEMS errors, roughly hobby-grade before calibration."""

    gyro_bias_rad_s: tuple[float, float, float] = (0.004, -0.006, 0.009)  # 0.23, -0.34, 0.52 °/s
    gyro_noise_std_rad_s: float = 0.003  # per sample at 100 Hz
    gyro_bias_walk_rad_s_per_sqrt_s: float = 0.00005  # the bias itself wanders slowly
    accel_bias_m_s2: tuple[float, float, float] = (0.12, -0.08, 0.20)
    accel_scale: tuple[float, float, float] = (1.01, 0.995, 1.02)
    accel_noise_std_m_s2: float = 0.02
    # Earth's field in a frame whose x points to true north: ~44 µT total in Israel (Survey of Israel: 42.7-44.6 µT),
    # declination ~5° east, pointing steeply down (the inclination here is an approximate ~45°)
    mag_field_ut: tuple[float, float, float] = (31.0, -2.7, -31.0)
    mag_hard_iron_ut: tuple[float, float, float] = (18.0, -9.0, 5.0)
    mag_soft_iron: tuple[tuple[float, float, float], ...] = ((1.08, 0.03, 0.0), (0.03, 0.93, 0.0), (0.0, 0.0, 1.0))
    mag_noise_std_ut: float = 0.4


def rot_z(yaw: float) -> Array:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def synthetic_imu(seconds: float = 120.0, rate_hz: float = 100.0, yaw_rate_rad_s: float = 0.0,
                  model: ImuErrorModel = ImuErrorModel(), up_axis: str = "+z", seed: int = 7) -> dict[str, Array]:
    """A robot IMU at rest (or turning at a constant yaw rate) on a flat floor.

    ``up_axis`` puts that sensor axis pointing up (for the 6-position test: "+x", "-x", ... "-z").
    """
    rng = np.random.default_rng(seed)
    n = int(seconds * rate_hz)
    dt = 1.0 / rate_hz
    t = np.arange(n) * dt
    walk = np.cumsum(rng.normal(0.0, model.gyro_bias_walk_rad_s_per_sqrt_s * math.sqrt(dt), (n, 3)), axis=0)
    gyro = np.array(model.gyro_bias_rad_s) + walk + rng.normal(0.0, model.gyro_noise_std_rad_s, (n, 3))
    # at rest the accelerometer measures the reaction to gravity: +g along the axis pointing up
    axis = "xyz".index(up_axis[1])
    sign = 1.0 if up_axis[0] == "+" else -1.0
    specific_force = np.zeros(3)
    specific_force[axis] = sign * G
    if up_axis == "+z":
        gyro[:, 2] += yaw_rate_rad_s
    accel = (np.array(model.accel_scale) * specific_force + np.array(model.accel_bias_m_s2)
             + rng.normal(0.0, model.accel_noise_std_m_s2, (n, 3)))
    yaw = yaw_rate_rad_s * t
    soft = np.array(model.mag_soft_iron)
    world_field = np.array(model.mag_field_ut)
    mag = np.array([soft @ (rot_z(-y) @ world_field) for y in yaw]) + np.array(model.mag_hard_iron_ut)
    mag += rng.normal(0.0, model.mag_noise_std_ut, (n, 3))
    data = {"t_s": t}
    for i, c in enumerate("xyz"):
        data["g" + c], data["a" + c], data["m" + c] = gyro[:, i], accel[:, i], mag[:, i]
    return data


# --- gyroscope ---------------------------------------------------------------------------------------
def estimate_bias(samples: Array) -> tuple[float, float]:
    """(mean, standard error of the mean) of a stationary gyro axis."""
    x = np.asarray(samples, dtype=float)
    return float(np.mean(x)), float(np.std(x, ddof=1) / math.sqrt(x.size))


def integrate_rate(t_s: Array, rate: Array, bias: float = 0.0) -> Array:
    """Angle (rad, not wrapped) from angular rate samples with the trapezoidal rule, starting at 0."""
    corrected = np.asarray(rate, dtype=float) - bias
    increments = 0.5 * (corrected[1:] + corrected[:-1]) * np.diff(t_s)
    return np.concatenate([[0.0], np.cumsum(increments)])


def drift_report(data: dict[str, Array], calibration_s: float = 10.0) -> str:
    """Estimate bias from the first ``calibration_s`` seconds, integrate the rest with and without it."""
    t, gz = data["t_s"], data["gz"]
    cal = t < t[0] + calibration_s
    lines = ["axis  mean [deg/s]  sigma [deg/s]  (whole log, robot must be still)"]
    for c in "xyz":
        g = data["g" + c]
        lines.append(f"  {c}   {math.degrees(np.mean(g)):+9.4f}   {math.degrees(np.std(g, ddof=1)):9.4f}")
    bias, sem = estimate_bias(gz[cal])
    rest_t, rest = t[~cal], gz[~cal]
    raw = np.degrees(integrate_rate(rest_t, rest))
    fixed = np.degrees(integrate_rate(rest_t, rest, bias))
    lines.append(f"\nz bias from the first {calibration_s:.0f} s: {math.degrees(bias):+.4f} deg/s "
                 f"(+- {math.degrees(sem):.4f} standard error)")
    lines.append("  after [s]   heading raw [deg]   heading bias-corrected [deg]")
    for after in (10, 60, 300, 600):
        k = np.searchsorted(rest_t - rest_t[0], after)
        if k < rest_t.size:
            lines.append(f"  {after:8d}   {raw[k]:+17.2f}   {fixed[k]:+28.2f}")
    return "\n".join(lines)


# --- accelerometer -----------------------------------------------------------------------------------
def tilt_from_accel(ax: float, ay: float, az: float) -> tuple[float, float]:
    """(roll, pitch) in radians from gravity, valid only when the robot isn't accelerating."""
    return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


def six_position_calibration(means: dict[str, tuple[float, float, float]], g: float = G) -> tuple[Array, Array]:
    """Per-axis (bias, scale) from the mean reading with each axis pointing up (+x..) and down (-x..).

    Up reads  s*g + b, down reads -s*g + b  ->  b = (up + down)/2,  s = (up - down)/(2g).
    Corrected reading = (raw - b) / s.
    """
    bias, scale = np.zeros(3), np.zeros(3)
    for i, c in enumerate("xyz"):
        up, down = means["+" + c][i], means["-" + c][i]
        bias[i] = (up + down) / 2.0
        scale[i] = (up - down) / (2.0 * g)
    return bias, scale


# --- magnetometer ------------------------------------------------------------------------------------
def hard_soft_iron(mx: Array, my: Array, mz: Array | None = None) -> tuple[Array, Array]:
    """Min/max calibration: offset = centre of the data (hard iron), per-axis scale to a sphere (soft iron).

    Needs data from turning the sensor through all headings (flat: 2 axes; tumbling: 3 axes).
    Corrected = (raw - offset) * scale. Axis-aligned only; a full ellipsoid fit also removes cross-coupling.
    """
    axes = [np.asarray(mx, float), np.asarray(my, float)] + ([np.asarray(mz, float)] if mz is not None else [])
    lo = np.array([a.min() for a in axes])
    hi = np.array([a.max() for a in axes])
    offset = (hi + lo) / 2.0
    radius = (hi - lo) / 2.0
    scale = radius.mean() / radius
    return offset, scale


def heading_from_mag(mx: Array | float, my: Array | float) -> Array | float:
    """Heading (rad) of a level sensor's x axis, counter-clockwise from magnetic north seen from above.

    The horizontal field points north; seen from the body frame it appears rotated by -yaw.
    Only valid when level: tilt mixes the (large, downward in Israel) vertical field into x and y.
    """
    return np.arctan2(-np.asarray(my), np.asarray(mx))


# --- the simulator: gyro vs wheel encoders vs truth ------------------------------------------------
def robot_demo(still_s: float = 5.0, seed: int = 4) -> dict[str, Array]:
    """karmel (realistic preset) stands still, then does four quarter-turn spins with pauses."""
    from robotlab.config import load_config
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

    cfg = load_config()
    params = DiffDriveParams.realistic(cfg)
    sensors = replace(SensorParams.realistic(cfg), gyro_bias_rad_s=0.01)
    base = SimBase(DiffDriveSim(World(), params, sensors, seed=seed), dt=0.01)
    t, gz, truth, enc = [], [], [], []
    b = cfg.drive.wheel_separation_m
    m_per_tick = cfg.drive.meters_per_tick
    plan = [(still_s, 0.0)] + [(0.9, 4.0), (1.0, 0.0)] * 4 + [(20.0, 0.0)]  # 4 rad/s for 0.9 s ~ 90°
    true_heading = 0.0
    for seconds, wheel in plan:
        for _ in range(round(seconds / base.dt)):
            base.set_wheel_velocity(-wheel, wheel)
            state = base.read()
            true_heading += base.sim.yaw_rate * base.dt
            t.append(state.t)
            gz.append(base.sim.gyro_z())
            truth.append(true_heading)
            enc.append((state.right_ticks - state.left_ticks) * m_per_tick / b)
    return {"t_s": np.array(t), "gz": np.array(gz), "truth": np.array(truth), "encoders": np.array(enc)}


def plot_lines(path: str, t: Array, series: list[tuple[str, Array]], ylabel: str, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, (label, y) in enumerate(series):
        if label == "truth":  # reference: dark, dashed, drawn on top so it stays visible
            ax.plot(t, y, color="#0b0b0b", linewidth=1.5, linestyle="--", label=label, zorder=5)
        else:
            ax.plot(t, y, color=COLORS[i % len(COLORS)], linewidth=2, label=label)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, color="#e4e3df", linewidth=0.8)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("drift", help="gyro bias and heading drift from a still log")
    p.add_argument("csv")
    p.add_argument("--calibration-s", type=float, default=10.0)
    p.add_argument("--plot")
    p = sub.add_parser("sixpos", help="accelerometer bias/scale from six still logs")
    p.add_argument("files", nargs=6, metavar="CSV", help="+x -x +y -y +z -z (that axis pointing UP)")
    p = sub.add_parser("mag", help="hard/soft iron from a log while turning through all headings")
    p.add_argument("csv")
    p.add_argument("--plot")
    p = sub.add_parser("robot", help="simulated karmel: gyro heading vs encoders vs truth")
    p.add_argument("--plot")
    args = ap.parse_args(argv)

    if args.cmd == "drift":
        data = read_log(args.csv)
        print(drift_report(data, args.calibration_s))
        if args.plot:
            t = data["t_s"]
            cal = t < t[0] + args.calibration_s
            bias, _ = estimate_bias(data["gz"][cal])
            plot_lines(args.plot, t, [("integrated raw gz", np.degrees(integrate_rate(t, data["gz"]))),
                                      ("bias-corrected", np.degrees(integrate_rate(t, data["gz"], bias)))],
                       "heading [deg]", "A still IMU: integrated gyro heading")
    elif args.cmd == "sixpos":
        names = ("+x", "-x", "+y", "-y", "+z", "-z")
        means = {}
        for name, path in zip(names, args.files):
            d = read_log(path)
            means[name] = (float(np.mean(d["ax"])), float(np.mean(d["ay"])), float(np.mean(d["az"])))
        bias, scale = six_position_calibration(means)
        for i, c in enumerate("xyz"):
            print(f"{c}: bias {bias[i]:+.3f} m/s2   scale {scale[i]:.4f}")
    elif args.cmd == "mag":
        d = read_log(args.csv)
        offset, scale = hard_soft_iron(d["mx"], d["my"], d["mz"])
        print("hard-iron offset [uT]:", np.round(offset, 2), "  soft-iron scale:", np.round(scale, 3))
        if args.plot:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(5.5, 5.5))
            ax.plot(d["mx"], d["my"], ".", color=COLORS[1], markersize=3, label="raw")
            ax.plot((d["mx"] - offset[0]) * scale[0], (d["my"] - offset[1]) * scale[1], ".", color=COLORS[0],
                    markersize=3, label="corrected")
            ax.set_aspect("equal")
            ax.axhline(0, color="#52514e", linewidth=0.8)
            ax.axvline(0, color="#52514e", linewidth=0.8)
            ax.set_xlabel("mx [uT]")
            ax.set_ylabel("my [uT]")
            ax.legend(frameon=False)
            fig.tight_layout()
            fig.savefig(args.plot, dpi=120)
            plt.close(fig)
    else:
        demo = robot_demo()
        t = demo["t_s"]
        still = t < 5.0
        bias, _ = estimate_bias(demo["gz"][still])
        raw = integrate_rate(t, demo["gz"])
        fixed = integrate_rate(t, demo["gz"], bias)
        print(f"gyro bias estimated in 5 s still: {math.degrees(bias):+.3f} deg/s")
        print("  time [s]   truth   encoders   gyro raw   gyro corrected   [deg]")
        for k in np.searchsorted(t, [5.0, 12.0, 20.0, t[-1]]):
            k = min(k, t.size - 1)
            print(f"  {t[k]:7.1f}  {math.degrees(demo['truth'][k]):7.1f}  {math.degrees(demo['encoders'][k]):8.1f}"
                  f"  {math.degrees(raw[k]):9.1f}  {math.degrees(fixed[k]):15.1f}")
        if args.plot:
            plot_lines(args.plot, t, [("truth", np.degrees(demo["truth"])),
                                      ("wheel encoders", np.degrees(demo["encoders"])),
                                      ("gyro, raw", np.degrees(raw)),
                                      ("gyro, bias-corrected", np.degrees(fixed))],
                       "heading [deg]", "karmel in the simulator: four quarter turns")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
