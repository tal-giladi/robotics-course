"""16.05 — learn karmel's motor model from logged data: physics fit vs a small neural network,
then use the model as feedforward for the wheel-speed loop of 08.09.

    py motor_sysid.py log      # drive the simulated robot with random duty steps at 4 battery levels -> data/motor/*.csv
    py motor_sysid.py fit      # fit physics model + MLP, compare in-range vs extrapolation   -> data/motor/models.json
    py motor_sysid.py ff       # closed-loop test: PI alone vs PI + learned feedforward, full and low battery

Real robot: log the same columns from telemetry (t, duty_l, duty_r, battery_v, left_rad_s, right_rad_s)
with the wheels in the air, save as CSV in data/motor/, and run `fit` unchanged.

The physics model (first-order motor with deadband and battery-voltage scaling):

    tau * dw/dt = k * dz(duty) * V / V_nom - w,     dz(d) = sign(d) * max(|d| - deadband, 0)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(ROOT.parent / "labs" / "python") not in sys.path:      # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT.parent / "labs" / "python"))

from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402

DATA = HERE / "data" / "motor"
DT = 0.02                                        # 50 Hz telemetry
V_NOM = 11.1                                     # labs/config/karmel.yaml battery.nominal_v
HOLD_S = 0.8                                     # each random duty is held this long
TRAIN_SOCS = (1.0, 0.75, 0.5)                    # battery state of charge for training logs
TEST_SOC = 0.12                                  # nearly empty battery: a voltage the model never saw
MAX_TRAIN_DUTY = 0.8                             # training logs never exceed |duty| 0.8


# ------------------------------------------------------------------------------ 1. logging
def make_base(soc: float, seed: int) -> SimBase:
    cfg = load_config()
    params = replace(DiffDriveParams.realistic(cfg), battery_initial_soc=soc, slip_std=0.0)
    return SimBase(DiffDriveSim(World(), params, SensorParams.ideal(cfg), seed=seed), dt=DT, watchdog_s=None)


def log_session(soc: float, seconds: float, max_duty: float, seed: int) -> list[dict]:
    """Random duty steps on both wheels, like 08.02's sweep but shuffled. Wheels in the air (no world)."""
    rng = np.random.default_rng(seed)
    base, rows = make_base(soc, seed), []
    steps = int(HOLD_S / DT)
    for _ in range(int(seconds / HOLD_S)):
        duty = rng.uniform(-max_duty, max_duty, 2)
        duty[rng.random(2) < 0.15] = 0.0                      # sometimes stop
        near_db = rng.random(2) < 0.15                        # sometimes probe the deadband region
        duty[near_db] = rng.uniform(-0.25, 0.25, int(near_db.sum()))
        for _ in range(steps):
            base.set_wheel_duty(float(duty[0]), float(duty[1]))
            s = base.read()
            rows.append({"t": round(s.t, 3), "duty_l": float(duty[0]), "duty_r": float(duty[1]),
                         "battery_v": round(s.battery_v, 3), "left_rad_s": round(s.left_rad_s, 3),
                         "right_rad_s": round(s.right_rad_s, 3)})
    return rows


def cmd_log() -> None:
    DATA.mkdir(parents=True, exist_ok=True)
    plan = [(soc, MAX_TRAIN_DUTY, f"train_soc{int(soc * 100)}") for soc in TRAIN_SOCS]
    plan += [(TEST_SOC, MAX_TRAIN_DUTY, f"test_soc{int(TEST_SOC * 100)}"), (0.75, 1.0, "test_fullduty_soc75")]
    for i, (soc, max_duty, name) in enumerate(plan):
        rows = log_session(soc, 120.0, max_duty, seed=100 + i)
        with (DATA / f"{name}.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        v = [r["battery_v"] for r in rows]
        print(f"{name:20s} {len(rows)} samples, battery {min(v):.2f}-{max(v):.2f} V, |duty| <= {max_duty}")


def read_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


# ------------------------------------------------------------------------------ 2. models
@dataclass
class PhysicsMotor:
    k: float            # rad/s per unit of (duty above deadband) at V_NOM
    deadband: float
    tau: float          # s

    def steady(self, duty: np.ndarray, volts: np.ndarray) -> np.ndarray:
        dz = np.sign(duty) * np.maximum(np.abs(duty) - self.deadband, 0.0)
        return self.k * dz * volts / V_NOM

    def simulate(self, duty: np.ndarray, volts: np.ndarray, w0: float = 0.0) -> np.ndarray:
        a = 1.0 - math.exp(-DT / self.tau)
        target, w = self.steady(duty, volts), np.empty(len(duty))
        w_prev = w0
        for n in range(len(duty)):
            w[n] = w_prev = w_prev + a * (target[n] - w_prev)
        return w

    def inverse(self, w_target: float, volts: float) -> float:
        """Feedforward duty that should produce w_target at this battery voltage."""
        if abs(w_target) < 1e-6:
            return 0.0
        return float(np.clip(np.sign(w_target) * (self.deadband + abs(w_target) * V_NOM / (self.k * volts)), -1, 1))


def steady_points(log: dict[str, np.ndarray], wheel: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Average the last 0.3 s of every 0.8 s hold: (duty, volts, settled speed). tau=0.08 s, so 0.5 s = 6 tau."""
    steps, tail = int(HOLD_S / DT), int(0.3 / DT)
    duty, speed = log["duty_" + wheel[0]], log[wheel + "_rad_s"]
    d, v, w = [], [], []
    for start in range(0, len(duty) - steps + 1, steps):
        sl = slice(start + steps - tail, start + steps)
        d.append(duty[start]); v.append(log["battery_v"][sl].mean()); w.append(speed[sl].mean())
    return np.array(d), np.array(v), np.array(w)


def fit_physics(logs: list[dict[str, np.ndarray]], wheel: str) -> PhysicsMotor:
    """Step 1, linear regression: for each candidate deadband, k has a closed-form least-squares
    solution k = sum(x*w)/sum(x*x) with x = dz(duty)*V/V_nom. Keep the deadband with the lowest error.
    Step 2, tau: 1-D search that makes the simulated response match the logged one."""
    d, v, w = (np.concatenate(z) for z in zip(*(steady_points(L, wheel) for L in logs)))
    best = None
    for db in np.arange(0.0, 0.301, 0.005):
        x = np.sign(d) * np.maximum(np.abs(d) - db, 0.0) * v / V_NOM
        k = float(x @ w / (x @ x))
        sse = float(np.sum((k * x - w) ** 2))
        if best is None or sse < best[0]:
            best = (sse, k, float(db))
    _, k, db = best

    def dyn_error(tau: float) -> float:
        m = PhysicsMotor(k, db, tau)
        return sum(float(np.mean((m.simulate(L["duty_" + wheel[0]], L["battery_v"]) - L[wheel + "_rad_s"]) ** 2)) for L in logs)

    tau = minimize_scalar(dyn_error, bounds=(0.01, 0.5), method="bounded").x
    return PhysicsMotor(k, db, float(tau))


class MLP:
    """2 inputs -> 32 -> 32 -> 1, tanh. Trained full-batch with Adam on normalised inputs (PyTorch)."""

    def __init__(self, x: np.ndarray, y: np.ndarray, seed: int = 0, epochs: int = 3000) -> None:
        import torch
        torch.manual_seed(seed)
        torch.set_num_threads(1)                      # 360 points: threads only add overhead
        self.torch = torch
        self.mu, self.sd = x.mean(0), x.std(0) + 1e-9
        self.ymu, self.ysd = y.mean(), y.std() + 1e-9
        self.net = torch.nn.Sequential(torch.nn.Linear(x.shape[1], 32), torch.nn.Tanh(),
                                       torch.nn.Linear(32, 32), torch.nn.Tanh(), torch.nn.Linear(32, 1))
        X = torch.tensor((x - self.mu) / self.sd, dtype=torch.float32)
        Y = torch.tensor((y - self.ymu) / self.ysd, dtype=torch.float32)[:, None]
        opt = torch.optim.Adam(self.net.parameters(), lr=1e-2)
        for _ in range(epochs):
            opt.zero_grad()
            loss = torch.mean((self.net(X) - Y) ** 2)
            loss.backward()
            opt.step()

    def __call__(self, x: np.ndarray) -> np.ndarray:
        with self.torch.no_grad():
            X = self.torch.tensor((np.atleast_2d(x) - self.mu) / self.sd, dtype=self.torch.float32)
            return self.net(X).numpy()[:, 0] * self.ysd + self.ymu


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))


def fit_all(wheel: str = "left") -> tuple[PhysicsMotor, MLP, MLP]:
    train = [read_csv(DATA / f"train_soc{int(s * 100)}.csv") for s in TRAIN_SOCS]
    tests = {"held-out time, same battery": None,
             f"low battery (soc {TEST_SOC:.2f})": read_csv(DATA / f"test_soc{int(TEST_SOC * 100)}.csv"),
             "|duty| up to 1.0 (trained <= 0.8)": read_csv(DATA / "test_fullduty_soc75.csv")}
    # hold out the last 20 % of every training log as the in-distribution test
    cut = [int(0.8 * len(L["t"])) // int(HOLD_S / DT) * int(HOLD_S / DT) for L in train]
    fit_logs = [{k: v[:c] for k, v in L.items()} for L, c in zip(train, cut)]
    tests["held-out time, same battery"] = {k: np.concatenate([v[c:] for L, c in zip(train, cut) for kk, v in L.items() if kk == k])
                                            for k in train[0]}

    phys = fit_physics(fit_logs, wheel)
    d, v, w = (np.concatenate(z) for z in zip(*(steady_points(L, wheel) for L in fit_logs)))
    nn_fwd = MLP(np.column_stack([d, v]), w)                      # (duty, V) -> speed
    moving = np.abs(w) > 0.5                                       # inverse is only defined outside the deadband
    nn_inv = MLP(np.column_stack([w[moving], v[moving]]), d[moving])   # (speed, V) -> duty

    print(f"physics fit ({wheel} wheel): k = {phys.k:.2f} rad/s per duty, deadband = {phys.deadband:.3f}, "
          f"tau = {phys.tau * 1000:.0f} ms   ({len(d)} steady-state points)")
    print(f"{'test set':36s} {'physics RMSE':>13s} {'MLP RMSE':>9s}   (steady-state wheel speed, rad/s)")
    for name, L in tests.items():
        td, tv, tw = steady_points(L, wheel)
        print(f"{name:36s} {rmse(phys.steady(td, tv), tw):13.3f} {rmse(nn_fwd(np.column_stack([td, tv])), tw):9.3f}")
    return phys, nn_fwd, nn_inv


def cmd_fit() -> None:
    phys, nn_fwd, _ = fit_all("left")
    probe = [(0.5, 12.4), (0.5, 9.3), (1.0, 11.5), (0.1, 11.5)]
    print("probe (duty, V) -> physics / MLP rad/s: " + "  ".join(
        f"({d}, {v}) {phys.steady(np.array([d]), np.array([v]))[0]:.1f}/{nn_fwd(np.array([[d, v]]))[0]:.1f}" for d, v in probe))
    DATA.mkdir(parents=True, exist_ok=True)
    (DATA / "models.json").write_text(json.dumps({"left": asdict(phys), "right": asdict(fit_physics(
        [read_csv(DATA / f"train_soc{int(s * 100)}.csv") for s in TRAIN_SOCS], "right"))}, indent=2))
    print(f"saved {DATA / 'models.json'}")


# ------------------------------------------------------------------------------ 3. feedforward
TARGET = 0.5 / 0.045                             # 0.5 m/s at wheel radius 0.045 m = 11.11 rad/s (08.09)


def run_speed_loop(soc: float, ff, kp: float, ki: float, seconds: float = 1.5, seed: int = 7) -> dict[str, float]:
    """Left wheel speed loop at 50 Hz on the Pi side (as in 08.04-08.09): duty = ff(w*, V) + kp*e + ki*integral(e).
    Step from rest to 11.11 rad/s at t = 0. Metrics: steady-state error (mean over the last 0.5 s),
    settling time into a +-5 % band, and integrated absolute error (IAE)."""
    base = make_base(soc, seed)
    integral, log = 0.0, []
    for n in range(int(seconds / DT)):
        s = base.read()
        e = TARGET - s.left_rad_s
        u = ff(TARGET, s.battery_v) + kp * e + integral
        if -1.0 < u < 1.0:                                  # anti-windup: integrate only when not saturated
            integral += ki * e * DT
        base.set_wheel_duty(float(np.clip(u, -1.0, 1.0)), 0.0)
        log.append((n * DT, s.left_rad_s))
    t, w = np.array(log).T
    outside = np.abs(w - TARGET) > 0.05 * TARGET
    settle = float(t[np.nonzero(outside)[0][-1]] + DT) if outside.any() else 0.0
    return {"ss_err": float(np.mean(TARGET - w[t >= seconds - 0.5])), "settle": settle,
            "iae": float(np.sum(np.abs(TARGET - w)) * DT)}


def cmd_ff() -> None:
    cfg = load_config().drive
    phys, _, nn_inv = fit_all("left")
    k_yaml = cfg.max_wheel_speed_rad_s / (1 - cfg.duty_deadband)          # 08.09's constant feedforward
    feedforwards = {
        "none": lambda w, v: 0.0,
        "karmel.yaml constants (ignores V)": lambda w, v: float(np.sign(w) * (cfg.duty_deadband + abs(w) / k_yaml)),
        "fitted physics model (uses V)": phys.inverse,
        "MLP inverse model (uses V)": lambda w, v: float(np.clip(nn_inv(np.array([[w, v]]))[0], -1.0, 1.0)),
    }
    for kp, ki, label in ((0.03, 0.0, "P only, kp=0.03"), (0.03, 0.4, "PI, kp=0.03 ki=0.4"), (0.03, 0.1, "PI, kp=0.03 ki=0.1")):
        print(f"\n{label}: step to {TARGET:.2f} rad/s (0.5 m/s)")
        print(f"  {'feedforward':36s} {'battery':>9s} {'ss error':>9s} {'settle 5%':>10s} {'IAE':>6s}")
        for name, ff in feedforwards.items():
            for soc in (1.0, TEST_SOC):
                r = run_speed_loop(soc, ff, kp, ki)
                v = make_base(soc, 0).read().battery_v
                settle = f"{r['settle']:.2f} s" if r["settle"] < 1.49 else "never"
                print(f"  {name:36s} {v:7.2f} V {r['ss_err']:+9.2f} {settle:>10s} {r['iae']:6.2f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["log", "fit", "ff"])
    args = ap.parse_args()
    {"log": cmd_log, "fit": cmd_fit, "ff": cmd_ff}[args.cmd]()


if __name__ == "__main__":
    main()
