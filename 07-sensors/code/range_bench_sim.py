"""07.01 / 07.03 / 07.04 — A simulated sensor bench: a wall at known distances, many readings per distance.

    py 07-sensors/code/range_bench_sim.py --sensor tof --out tof_sim.csv
    py 07-sensors/code/range_bench_sim.py --sensor both --temp-c 35 --out summer.csv
    py 07-sensors/code/range_bench_sim.py --sensor both --wall-angle-deg 30 --out angled.csv
    py 07-sensors/code/sensor_stats.py summer.csv --plot summer.png

Geometry comes from the course simulator (robotlab): the robot faces a wall and ``front_range()``
casts its cone of rays, so an angled wall really is closer at the edge of the cone. robotlab only
adds Gaussian noise, so this script layers the imperfections a static test is designed to find on
top of the geometric distance: constant offset, scale error (the speed of sound!), distance-dependent
noise, resolution, outliers, dropouts, and the ultrasonic mirror effect on angled walls.

The numbers in the two presets are ILLUSTRATIVE, chosen to behave like the real parts qualitatively.
Your own sensors will differ; that is exactly why you characterize them.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
for _p in (HERE, ROOT / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from range_physics import FIRMWARE_SPEED_OF_SOUND_M_S, speed_of_sound  # noqa: E402
from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveSim, SensorParams, World  # noqa: E402
from sensor_stats import write_samples  # noqa: E402

DEFAULT_DISTANCES = (0.10, 0.25, 0.50, 1.00, 2.00, 3.00)


@dataclass(frozen=True)
class RangeSensorModel:
    """Imperfections applied to the geometric distance (meters, probabilities per reading)."""

    name: str
    fov_rad: float  # full cone angle
    min_m: float
    max_m: float
    noise_std_m: float  # distance-dependent sigma at 1 m
    noise_distance_exponent: float  # that part grows as (d / 1 m) ** exponent (ToF: photons fall as 1/d^2)
    noise_floor_m: float  # distance-independent sigma (timing jitter, electronics); the two add in quadrature
    offset_m: float  # constant bias
    scale: float  # reading = scale * distance: the ultrasonic speed-of-sound error lives here
    resolution_m: float  # readings are rounded to this step
    outlier_prob: float
    dropout_prob: float
    far_dropout_start_m: float  # dropouts rise linearly from here to max_m (weak return signal)
    specular_limit_deg: float | None  # ultrasonic: past this wall angle the echo mostly misses

    @classmethod
    def tof(cls) -> RangeSensorModel:
        """VL53L1X-like: 27° cone, sigma grows with distance, small uncalibrated offset."""
        return cls("vl53l1x", math.radians(27.0), 0.04, 4.0, 0.003, 1.0, 0.0015, 0.008, 1.0, 0.001,
                   0.003, 0.005, 2.5, None)

    @classmethod
    def ultrasonic(cls, air_temp_c: float = 20.0) -> RangeSensorModel:
        """US-100-like in pulse mode with firmware that assumes 343 m/s, in air at ``air_temp_c``."""
        scale = FIRMWARE_SPEED_OF_SOUND_M_S / speed_of_sound(air_temp_c)
        return cls("us100", math.radians(15.0), 0.02, 4.5, 0.002, 0.0, 0.002, 0.0, scale, 0.001,
                   0.02, 0.01, 3.5, 15.0)


def wall_world(distance_m: float, wall_angle_deg: float, sensor_x_m: float) -> World:
    """A long wall whose closest point on the sensor axis is ``distance_m`` in front of the sensor.

    The robot sits at the origin facing +x; the wall is rotated by ``wall_angle_deg`` about the point
    where the sensor axis hits it (0° = square to the beam).
    """
    hit_x = sensor_x_m + distance_m
    a = math.radians(wall_angle_deg)
    half = 5.0
    dx, dy = -math.sin(a) * half, math.cos(a) * half  # direction along the wall
    return World.from_segments([[hit_x - dx, -dy, hit_x + dx, dy]])


def simulate_static_test(
    model: RangeSensorModel,
    distances_m: tuple[float, ...] = DEFAULT_DISTANCES,
    n: int = 200,
    wall_angle_deg: float = 0.0,
    seed: int = 1,
) -> list[tuple[float, str, float | None]]:
    """``n`` readings per distance: ``(true_m, sensor, measured_m or None)`` rows."""
    rng = np.random.default_rng(seed)
    cfg = load_config()
    sensor_params = replace(SensorParams.ideal(cfg), range_fov_rad=model.fov_rad, range_rays=9,
                            range_max_m=model.max_m * 2, range_noise_std_m=0.0)
    rows: list[tuple[float, str, float | None]] = []
    for d in distances_m:
        sim = DiffDriveSim(wall_world(d, wall_angle_deg, sensor_params.range_x_m), sensor_params=sensor_params,
                           pose=(0.0, 0.0, 0.0), seed=seed)
        geometric = sim.front_range()  # nearest ray of the cone: already shorter than d on an angled wall
        for _ in range(n):
            rows.append((d, model.name, read_once(model, geometric, wall_angle_deg, rng)))
    return rows


def read_once(model: RangeSensorModel, geometric_m: float | None, wall_angle_deg: float,
              rng: np.random.Generator) -> float | None:
    """One reading of a target whose nearest cone hit is ``geometric_m`` meters away."""
    if geometric_m is None:
        return None
    d = geometric_m
    if model.specular_limit_deg is not None and abs(wall_angle_deg) > model.specular_limit_deg:
        # The ping bounces away like light off a mirror. Usually silence; sometimes a ghost echo that
        # went wall -> something else -> back, i.e. much longer than the true distance.
        if rng.random() < 0.85:
            return None
        d = d * rng.uniform(1.8, 2.5)
    far = max(0.0, (d - model.far_dropout_start_m) / max(model.max_m - model.far_dropout_start_m, 1e-9))
    if rng.random() < model.dropout_prob + 0.5 * min(far, 1.0):
        return None
    if rng.random() < model.outlier_prob:
        reading = rng.uniform(model.min_m, model.max_m)  # multipath, crosstalk, a passing foot
    else:
        sigma = math.hypot(model.noise_floor_m, model.noise_std_m * d**model.noise_distance_exponent)
        reading = model.scale * d + model.offset_m + rng.normal(0.0, sigma)
    reading = round(reading / model.resolution_m) * model.resolution_m
    if reading < model.min_m or reading > model.max_m:
        return None
    return float(reading)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sensor", choices=("tof", "us100", "both"), default="both")
    ap.add_argument("--distances", type=float, nargs="+", default=list(DEFAULT_DISTANCES))
    ap.add_argument("--n", type=int, default=200, help="readings per distance")
    ap.add_argument("--temp-c", type=float, default=20.0, help="air temperature for the ultrasonic model")
    ap.add_argument("--wall-angle-deg", type=float, default=0.0, help="0 = wall square to the beam")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    models = []
    if args.sensor in ("tof", "both"):
        models.append(RangeSensorModel.tof())
    if args.sensor in ("us100", "both"):
        models.append(RangeSensorModel.ultrasonic(args.temp_c))
    rows = []
    for i, model in enumerate(models):
        rows += simulate_static_test(model, tuple(args.distances), args.n, args.wall_angle_deg, args.seed + i)
    n = write_samples(args.out, rows)
    print(f"wrote {n} simulated readings to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
