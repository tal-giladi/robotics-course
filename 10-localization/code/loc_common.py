"""Shared helpers for the module-10 scripts (10.01-10.10).

* ``load_exercise("10.04")`` imports YOUR ``labs/exercises/10.04/student.py`` (``solution=True`` for
  the reference), so the experiment scripts run your own filter.
* ``load_provided("10.03", "corridor_sim")`` imports a provided helper from an exercise folder.
* ``drive_living_room`` + ``odometry_cloud``: the Monte Carlo "belief cloud" used in 10.01 and 10.02.
* ``drive_tour``: a longer drive through three rooms of the apartment (10.06-10.10) that logs encoder
  ticks, gyro, landmark observations and LiDAR scans, plus the ground truth to grade estimators.
"""

from __future__ import annotations

import importlib.util
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

import numpy as np
from numpy.typing import NDArray

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
EXERCISES = ROOT / "labs" / "exercises"

if str(ROOT / "labs" / "python") not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import KarmelConfig, load_config  # noqa: E402
from robotlab.geometry import SE2, wrap_angle  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World, arc_update  # noqa: E402

CHI2_2DOF_95 = 5.991  # 95% of a 2D Gaussian lies inside Mahalanobis distance^2 < 5.991


def _import(path: Path, name: str) -> ModuleType:
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_exercise(lesson_id: str, solution: bool = False) -> ModuleType:
    """Import ``student.py`` (or ``solution.py``) of ``labs/exercises/<lesson_id>``."""
    kind = "solution" if solution else "student"
    return _import(EXERCISES / lesson_id / f"{kind}.py", f"module10_{re.sub(r'\W', '_', lesson_id)}_{kind}")


def load_provided(lesson_id: str, module: str) -> ModuleType:
    """Import a provided helper such as ``labs/exercises/10.03/corridor_sim.py``."""
    return _import(EXERCISES / lesson_id / f"{module}.py", f"course_exercise_{re.sub(r'\W', '_', lesson_id)}_{module}")


def out_dir(path: str | Path) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


# --- ellipses (10.02) ---------------------------------------------------------------------------
def chi2_2dof_threshold(probability: float) -> float:
    """k such that P(d^2 <= k) = probability for a 2D Gaussian: k = -2 ln(1 - p) (exact in 2D)."""
    return -2.0 * math.log(1.0 - probability)


def ellipse_axes(cov: NDArray[np.floating], k: float) -> tuple[float, float, float]:
    """(major semi-axis, minor semi-axis, major-axis angle in degrees) of the ellipse d^2 = k."""
    eigvals, eigvecs = np.linalg.eigh(np.asarray(cov, dtype=float)[:2, :2])  # ascending
    eigvals = np.maximum(eigvals, 0.0)
    major = eigvecs[:, 1]
    angle = math.degrees(math.atan2(major[1], major[0]))
    angle = (angle + 90.0) % 180.0 - 90.0  # an axis has no direction: report it in (-90, 90]
    return math.sqrt(k * eigvals[1]), math.sqrt(k * eigvals[0]), angle


def fraction_inside(points: NDArray[np.floating], mean: NDArray[np.floating], cov: NDArray[np.floating], k: float) -> float:
    """Fraction of (N, 2) points with Mahalanobis distance^2 <= k."""
    d = np.asarray(points, dtype=float) - np.asarray(mean, dtype=float)
    m2 = np.einsum("ij,ij->i", d @ np.linalg.inv(cov), d)
    return float(np.mean(m2 <= k))


# --- the living-room drive and the odometry cloud (10.01, 10.02) --------------------------------
START = (1.0, 1.3, 0.0)
# (seconds, left rad/s, right rad/s): along the sofa, turn left, up, turn left, back.
ROUTE = [(7.0, 6.0, 6.0), (1.16, -3.0, 3.0), (4.4, 6.0, 6.0), (1.16, -3.0, 3.0), (6.0, 6.0, 6.0)]


@dataclass(frozen=True)
class Drive:
    t: NDArray[np.floating]      # (T,)
    truth: NDArray[np.floating]  # (T, 3) true poses
    ticks: NDArray[np.int64]     # (T, 2) encoder counts
    landmarks: list[tuple[int, list]]  # (step index, [LandmarkObservation]) every ``observe_every`` steps
    world: World


def drive_living_room(
    seed: int = 7,
    dt: float = 0.05,
    kidnap_at_s: float | None = None,
    kidnap_to: tuple[float, float, float] = (2.0, 1.3, math.pi / 2),
    observe_every: int = 10,
    config: KarmelConfig | None = None,
) -> Drive:
    """Drive ROUTE with the realistic robot in the apartment; log truth, encoder ticks and landmarks.

    ``kidnap_at_s``: at that time, pick the robot up and put it down at ``kidnap_to`` (the encoders
    notice nothing — the kidnapped robot problem).
    """
    cfg = config or load_config()
    world = World.apartment()
    sim = DiffDriveSim(world, DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), pose=START, seed=seed, config=cfg)
    t, truth, ticks, landmarks = [0.0], [sim.pose.as_tuple()], [sim.ticks], []
    kidnapped = False
    for seconds, left, right in ROUTE:
        for _ in range(round(seconds / dt)):
            if kidnap_at_s is not None and not kidnapped and sim.t >= kidnap_at_s - 1e-9:
                sim.pose = SE2.from_tuple(kidnap_to)
                kidnapped = True
            sim.set_velocity(left, right)
            sim.step(dt)
            t.append(sim.t)
            truth.append(sim.pose.as_tuple())
            ticks.append(sim.ticks)
            if (len(t) - 1) % observe_every == 0:
                landmarks.append((len(t) - 1, sim.observe_landmarks()))
    return Drive(np.array(t), np.array(truth), np.array(ticks, dtype=np.int64), landmarks, world)


def dead_reckon(ticks: NDArray[np.int64], start: tuple[float, float, float], config: KarmelConfig | None = None) -> NDArray[np.floating]:
    """Plain wheel odometry (nominal karmel.yaml numbers) from a tick log: (T, 3)."""
    cfg = config or load_config()
    pose = SE2.from_tuple(start)
    out = [pose.as_tuple()]
    for d in np.diff(ticks, axis=0) * cfg.drive.meters_per_tick:
        pose = arc_update(pose, float(d[0]), float(d[1]), cfg.drive.wheel_separation_m)
        out.append(pose.as_tuple())
    return np.array(out)


def odometry_cloud(
    ticks: NDArray[np.int64],
    start: tuple[float, float, float],
    n: int = 500,
    seed: int = 0,
    radius_std: float = 0.01,
    separation_std: float = 0.04,
    slip_std: float = 0.02,
    config: KarmelConfig | None = None,
) -> NDArray[np.floating]:
    """Monte Carlo belief: ``n`` hypothetical robots integrate the SAME encoder log, each with its
    own plausible calibration error (radius, wheelbase) and its own random slip. Returns (T, n, 3).

    Nobody knows which of them is the real robot — the spread of the cloud IS the uncertainty.
    """
    cfg = config or load_config()
    rng = np.random.default_rng(seed)
    scale_l = 1.0 + rng.normal(0.0, radius_std, n)
    scale_r = 1.0 + rng.normal(0.0, radius_std, n)
    b = cfg.drive.wheel_separation_m * (1.0 + rng.normal(0.0, separation_std, n))
    x = np.full(n, start[0])
    y = np.full(n, start[1])
    th = np.full(n, start[2])
    out = [np.column_stack([x, y, th])]
    for d in np.diff(ticks, axis=0) * cfg.drive.meters_per_tick:
        dl = d[0] * scale_l * (1.0 + rng.normal(0.0, slip_std, n))
        dr = d[1] * scale_r * (1.0 + rng.normal(0.0, slip_std, n))
        ds, dth = (dl + dr) / 2.0, (dr - dl) / b
        mid = th + dth / 2.0  # midpoint heading: accurate enough for 50 ms steps
        x, y, th = x + ds * np.cos(mid), y + ds * np.sin(mid), wrap_angle(th + dth)
        out.append(np.column_stack([x, y, th]))
    return np.array(out)


def expected_landmark_obs(pose: tuple[float, float, float], landmark_xy: NDArray[np.floating]) -> tuple[float, float]:
    """(range, bearing) a robot at ``pose`` should measure to a landmark at ``landmark_xy``."""
    dx, dy = landmark_xy[0] - pose[0], landmark_xy[1] - pose[1]
    return math.hypot(dx, dy), float(wrap_angle(math.atan2(dy, dx) - pose[2]))


def landmark_residuals(drive: Drive, estimate: NDArray[np.floating]) -> list[tuple[float, float, float]]:
    """For every landmark observation: (time, range residual m, bearing residual rad), where the
    residual = measured - expected-from-``estimate``. Small residuals: the estimate explains what the
    robot sees. Large ones: the estimate is wrong (or the robot was kidnapped)."""
    out = []
    for k, observations in drive.landmarks:
        for obs in observations:
            i = int(np.flatnonzero(drive.world.landmark_ids == obs.id)[0])
            r, b = expected_landmark_obs(tuple(estimate[k]), drive.world.landmarks[i])
            out.append((float(drive.t[k]), obs.range_m - r, float(wrap_angle(obs.bearing_rad - b))))
    return out


# --- the apartment tour (10.06-10.10) ------------------------------------------------------------
def tour_sim() -> ModuleType:
    """The provided ``labs/exercises/10.06/tour_sim.py`` (``drive_tour``, ``TourLog``, ``TOUR``)."""
    return load_provided("10.06", "tour_sim")


def drive_tour(*args, **kwargs):
    """``tour_sim.drive_tour``: drive living room -> kitchen -> study and log every sensor + truth."""
    return tour_sim().drive_tour(*args, **kwargs)


def wheel_travels(ticks: NDArray[np.int64], config: KarmelConfig | None = None) -> NDArray[np.floating]:
    """(T-1, 2) left/right wheel travel per step in meters, from a tick log (nominal radius)."""
    return tour_sim().wheel_travels(ticks, config)
