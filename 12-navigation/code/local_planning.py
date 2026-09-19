"""12.05 — Local planning: pure pursuit, regulated pure pursuit, a dynamic window (DWB-style) planner and
a small MPPI, all driving the course simulator (reference implementations).

    python 12-navigation/code/local_planning.py             # all four controllers, PNGs + a GIF in nav_out/
    python 12-navigation/code/local_planning.py --no-gif    # skip the animation (faster)

Scenario: the global path (A* on the saved map, 12.02/12.04) goes from the living room to the kitchen.
Someone left a laundry basket on it. The basket is NOT in the map; only the LiDAR sees it.
Localization is the simulator's ground truth here; on the robot it is AMCL + odometry (module 10).
"""

from __future__ import annotations

import argparse
import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

import nav_common
from robotlab.geometry import SE2
from robotlab.sim import LaserScan

Pose = tuple[float, float, float]


# --- shared geometry --------------------------------------------------------------------------------
@dataclass(frozen=True)
class Limits:
    """Velocity and acceleration limits (karmel's velocity_smoother values in nav2_params.yaml)."""

    v_max: float = 0.25
    v_min: float = 0.0  # no reversing
    w_max: float = 1.2
    a_v: float = 0.6
    a_w: float = 2.0


def twist_to_wheels(v: float, w: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]:
    """cmd_vel (v, omega) -> (left, right) wheel speeds in rad/s (lesson 09.02)."""
    return (v - w * wheel_separation / 2.0) / wheel_radius, (v + w * wheel_separation / 2.0) / wheel_radius


def to_robot_frame(pose: Pose, points: NDArray[np.floating]) -> NDArray[np.floating]:
    """World points ``(N, 2)`` -> base_link coordinates (x forward, y left)."""
    return SE2(*pose).inverse().apply(np.asarray(points, dtype=float).reshape(-1, 2))


def path_distances(path: NDArray[np.floating]) -> NDArray[np.floating]:
    """Cumulative arc length at every path point."""
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])


def cross_track_error(path: NDArray[np.floating], xy: NDArray[np.floating]) -> float:
    """Distance from ``xy`` to the nearest point of the path polyline."""
    a, b = path[:-1], path[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", xy - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + t[:, None] * ab - xy, axis=1)))


# --- pure pursuit and its regulated variant ----------------------------------------------------------------
def lookahead_point(path: NDArray[np.floating], xy: NDArray[np.floating], lookahead: float, start: int) -> tuple[NDArray[np.floating], int]:
    """First point on the path, from segment ``start`` on, at distance ``lookahead`` from ``xy``
    (interpolated where the circle crosses a segment). Returns the last point if the path ends first."""
    for i in range(start, len(path) - 1):
        p, q = path[i], path[i + 1]
        if np.linalg.norm(q - xy) >= lookahead:
            # solve |p + t (q - p) - xy| = lookahead for the largest t in [0, 1]
            d, f = q - p, p - xy
            a, b, c = d @ d, 2 * f @ d, f @ f - lookahead**2
            disc = max(b * b - 4 * a * c, 0.0)
            t = (-b + math.sqrt(disc)) / (2 * a) if a > 0 else 1.0
            return p + min(max(t, 0.0), 1.0) * d, i
    return path[-1].copy(), len(path) - 2


def pure_pursuit_curvature(x_r: float, y_r: float) -> float:
    """Curvature of the circular arc through base_link, tangent to x, that ends at (x_r, y_r)."""
    d2 = x_r * x_r + y_r * y_r
    return 0.0 if d2 < 1e-9 else 2.0 * y_r / d2


@dataclass
class PurePursuit:
    """Pure pursuit with the goal handling every real controller needs; ``regulated=True`` adds
    Regulated Pure Pursuit's curvature speed limit and velocity-scaled lookahead (parameter names as in Nav2)."""

    path: NDArray[np.floating]
    lookahead_dist: float = 0.3
    desired_linear_vel: float = 0.25
    xy_goal_tolerance: float = 0.05
    rotate_to_heading_min_angle: float = 0.785
    rotate_to_heading_angular_vel: float = 1.0
    approach_velocity_scaling_dist: float = 0.4
    min_approach_linear_velocity: float = 0.05
    max_angular_vel: float = 1.2
    regulated: bool = False
    regulated_linear_scaling_min_radius: float = 0.5
    regulated_linear_scaling_min_speed: float = 0.08
    lookahead_time: float = 1.5
    min_lookahead_dist: float = 0.25
    max_lookahead_dist: float = 0.8
    done: bool = field(default=False, init=False)
    _index: int = field(default=0, init=False)
    _last_v: float = field(default=0.0, init=False)
    carrot: NDArray[np.floating] | None = field(default=None, init=False)

    def compute(self, pose: Pose) -> tuple[float, float]:
        xy = np.array(pose[:2])
        if self.done or np.linalg.norm(self.path[-1] - xy) <= self.xy_goal_tolerance:
            self.done = True
            return 0.0, 0.0
        # progress: nearest path point within a window ahead of the last one (never jump backwards)
        window = self.path[self._index : self._index + 80]
        self._index += int(np.argmin(np.linalg.norm(window - xy, axis=1)))
        lookahead = self.lookahead_dist
        if self.regulated:  # use_velocity_scaled_lookahead_dist
            lookahead = min(max(abs(self._last_v) * self.lookahead_time, self.min_lookahead_dist), self.max_lookahead_dist)
        carrot, _ = lookahead_point(self.path, xy, lookahead, min(self._index, len(self.path) - 2))
        self.carrot = carrot
        x_r, y_r = to_robot_frame(pose, carrot)[0]
        angle = math.atan2(y_r, x_r)
        if abs(angle) > self.rotate_to_heading_min_angle:  # use_rotate_to_heading
            self._last_v = 0.0
            return 0.0, math.copysign(self.rotate_to_heading_angular_vel, angle)
        kappa = pure_pursuit_curvature(x_r, y_r)
        v = self.desired_linear_vel
        if self.regulated and kappa != 0.0:  # curvature constraint (regulation_functions.hpp)
            radius = abs(1.0 / kappa)
            if radius < self.regulated_linear_scaling_min_radius:
                v *= 1.0 - abs(radius - self.regulated_linear_scaling_min_radius) / self.regulated_linear_scaling_min_radius
                v = max(v, self.regulated_linear_scaling_min_speed)
        remaining = float(path_distances(self.path)[-1] - path_distances(self.path)[self._index]) + float(np.linalg.norm(self.path[self._index] - xy))
        if remaining < self.approach_velocity_scaling_dist:  # approach velocity scaling
            v = min(v, max(v * remaining / self.approach_velocity_scaling_dist, self.min_approach_linear_velocity))
        w = v * kappa
        if abs(w) > self.max_angular_vel:  # keep the curvature, slow down instead
            v *= self.max_angular_vel / abs(w)
            w = math.copysign(self.max_angular_vel, w)
        self._last_v = v
        return v, w


# --- dynamic window approach (the idea behind DWB) ----------------------------------------------------------
def rollout(v: NDArray[np.floating], w: NDArray[np.floating], sim_time: float, dt: float) -> NDArray[np.floating]:
    """Constant (v, w) arcs from the robot origin: ``(K, steps, 3)`` poses (x, y, theta) in base_link."""
    steps = int(round(sim_time / dt))
    v, w = np.asarray(v, dtype=float)[:, None], np.asarray(w, dtype=float)[:, None]
    t = np.arange(1, steps + 1)[None, :] * dt
    theta = w * t
    small = np.abs(w) < 1e-6
    safe_w = np.where(small, 1.0, w)
    x = np.where(small, v * t, v / safe_w * np.sin(theta))
    y = np.where(small, 0.0, v / safe_w * (1.0 - np.cos(theta)))
    return np.stack([x, y, np.broadcast_to(theta, x.shape)], axis=-1)


def min_clearance(points: NDArray[np.floating], obstacles: NDArray[np.floating]) -> NDArray[np.floating]:
    """For ``(K, T, 2)`` trajectory points, the smallest distance of each trajectory to any obstacle point."""
    if len(obstacles) == 0:
        return np.full(points.shape[0], np.inf)
    d, _ = cKDTree(obstacles).query(points.reshape(-1, 2))  # nearest obstacle point for every trajectory point
    return d.reshape(points.shape[:2]).min(axis=1)


@dataclass
class DWAConfig:
    sim_time: float = 1.7  # DWB StandardTrajectoryGenerator default
    dt: float = 0.1
    vx_samples: int = 8
    vtheta_samples: int = 21
    path_dist_scale: float = 2.0  # like DWB's PathDist critic
    goal_dist_scale: float = 3.0  # like GoalDist (here: distance to a carrot on the path)
    obstacle_scale: float = 0.1  # like BaseObstacle: grows as clearance shrinks
    speed_scale: float = 1.0  # prefer fast forward motion
    safety_margin: float = 0.03


def dynamic_window(v: float, w: float, limits: Limits, control_dt: float) -> tuple[float, float, float, float]:
    """Velocities reachable within one control period: the 'dynamic window'."""
    return (max(limits.v_min, v - limits.a_v * control_dt), min(limits.v_max, v + limits.a_v * control_dt),
            max(-limits.w_max, w - limits.a_w * control_dt), min(limits.w_max, w + limits.a_w * control_dt))


def dwa_choose(
    v: float, w: float, path_r: NDArray[np.floating], carrot_r: NDArray[np.floating], obstacles_r: NDArray[np.floating],
    robot_radius: float, limits: Limits, cfg: DWAConfig, control_dt: float,
) -> tuple[float, float, dict[str, NDArray[np.floating]]]:
    """Score every (v, w) in the dynamic window; return the cheapest one. Everything is in base_link."""
    v_lo, v_hi, w_lo, w_hi = dynamic_window(v, w, limits, control_dt)
    vv, ww = np.meshgrid(np.linspace(v_lo, v_hi, cfg.vx_samples), np.linspace(w_lo, w_hi, cfg.vtheta_samples))
    vv, ww = vv.ravel(), ww.ravel()
    traj = rollout(vv, ww, cfg.sim_time, cfg.dt)
    ends = traj[:, -1, :2]
    clearance = min_clearance(traj[:, :, :2], obstacles_r) - robot_radius
    path_cost = np.min(np.linalg.norm(ends[:, None, :] - path_r[None, :, :], axis=-1), axis=1)
    goal_cost = np.linalg.norm(ends - carrot_r, axis=1)
    obstacle_cost = 1.0 / np.maximum(clearance, 1e-3)
    speed_cost = (limits.v_max - vv) / limits.v_max
    total = (cfg.path_dist_scale * path_cost + cfg.goal_dist_scale * goal_cost
             + cfg.obstacle_scale * obstacle_cost + cfg.speed_scale * speed_cost)
    total[clearance < cfg.safety_margin] = np.inf  # would collide: not admissible
    best = int(np.argmin(total))
    if not np.isfinite(total[best]):
        return 0.0, 0.0, {"traj": traj, "cost": total}  # nothing safe: stop (Nav2 would start recovery)
    return float(vv[best]), float(ww[best]), {"traj": traj, "cost": total}


# --- MPPI (model predictive path integral), small and readable -------------------------------------------------
@dataclass
class MPPIConfig:
    time_steps: int = 20
    model_dt: float = 0.1
    batch_size: int = 400
    vx_std: float = 0.1
    wz_std: float = 0.6
    temperature: float = 0.3
    path_weight: float = 4.0
    goal_weight: float = 6.0
    obstacle_weight: float = 0.05
    collision_cost: float = 1000.0
    safety_margin: float = 0.03


@dataclass
class MPPI:
    """Sample noisy control sequences around the previous best, roll them out, weight them by
    exp(-cost / temperature), average. No gradients, any cost function works."""

    limits: Limits
    robot_radius: float
    cfg: MPPIConfig = field(default_factory=MPPIConfig)
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))
    u: NDArray[np.floating] = field(init=False)

    def __post_init__(self) -> None:
        self.u = np.zeros((self.cfg.time_steps, 2))

    def compute(self, path_r: NDArray[np.floating], carrot_r: NDArray[np.floating], obstacles_r: NDArray[np.floating]) -> tuple[float, float, NDArray[np.floating]]:
        c, lim = self.cfg, self.limits
        eps = self.rng.normal(size=(c.batch_size, c.time_steps, 2)) * np.array([c.vx_std, c.wz_std])
        controls = self.u[None] + eps
        controls[..., 0] = np.clip(controls[..., 0], lim.v_min, lim.v_max)
        controls[..., 1] = np.clip(controls[..., 1], -lim.w_max, lim.w_max)
        # roll out the unicycle model for every sample
        x = np.zeros((c.batch_size, 3))
        states = np.empty((c.batch_size, c.time_steps, 3))
        for t in range(c.time_steps):
            x = x + np.column_stack([controls[:, t, 0] * np.cos(x[:, 2]), controls[:, t, 0] * np.sin(x[:, 2]), controls[:, t, 1]]) * c.model_dt
            states[:, t] = x
        pts = states[:, :, :2]
        path_cost = cKDTree(path_r).query(pts.reshape(-1, 2))[0].reshape(pts.shape[:2]).mean(axis=1)
        goal_cost = np.linalg.norm(pts[:, -1] - carrot_r, axis=1)
        clearance = min_clearance(pts, obstacles_r) - self.robot_radius
        obstacle_cost = c.obstacle_weight / np.maximum(clearance, 1e-3) + c.collision_cost * (clearance < c.safety_margin)
        cost = c.path_weight * path_cost + c.goal_weight * goal_cost + obstacle_cost
        weights = np.exp(-(cost - cost.min()) / c.temperature)
        weights /= weights.sum()
        self.u = np.einsum("k,ktj->tj", weights, controls)  # softmax-weighted average of the samples
        v, w = float(self.u[0, 0]), float(self.u[0, 1])
        self.u = np.vstack([self.u[1:], self.u[-1:]])  # warm start: shift one step
        return v, w, states


# --- running a controller in the simulator ---------------------------------------------------------------
@dataclass
class RunLog:
    name: str
    poses: list[Pose] = field(default_factory=list)
    commands: list[tuple[float, float]] = field(default_factory=list)
    scans: list[LaserScan | None] = field(default_factory=list)
    collided: bool = False
    reached: bool = False
    t: float = 0.0


def run_in_sim(
    name: str,
    world,
    path: NDArray[np.floating],
    policy: Callable[[Pose, LaserScan | None, float, float], tuple[float, float, bool]],
    *,
    timeout_s: float = 60.0,
    control_every: int = 5,
    realistic: bool = False,
    seed: int = 0,
    use_scan: bool = True,
) -> RunLog:
    """Drive SimBase at 50 Hz; call ``policy(pose, scan, v, w)`` every ``control_every`` reads (10 Hz with the
    default), with a fresh LiDAR scan; send wheel speeds every read (the firmware watchdog wants that)."""
    from robotlab.config import load_config
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase

    cfg = load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    heading = math.atan2(path[5][1] - path[0][1], path[5][0] - path[0][0])
    base = SimBase(DiffDriveSim(world, params, sensors, pose=(path[0][0], path[0][1], heading), seed=seed), dt=0.02)
    log = RunLog(name)
    v = w = 0.0
    wheels = (0.0, 0.0)
    stuck = 0
    for k in range(int(timeout_s / base.dt)):
        pose = tuple(base.true_pose)
        if k % control_every == 0:
            scan = base.scan() if use_scan else None
            v, w, done = policy(pose, scan, v, w)  # type: ignore[arg-type]
            wheels = twist_to_wheels(v, w, cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m)
            log.scans.append(scan)
            if done:
                log.reached = True
        else:
            log.scans.append(None)
        base.set_wheel_velocity(*wheels)
        base.read()
        log.collided |= base.sim.collided
        stuck = stuck + 1 if base.sim.collided else 0
        if stuck * base.dt >= 3.0:  # pushing against an obstacle for 3 s: give up (Nav2 would try recoveries)
            break
        log.poses.append(tuple(base.true_pose))  # type: ignore[arg-type]
        log.commands.append((v, w))
        if log.reached and max(abs(s) for s in base.sim.wheel_rad_s) < 0.05:
            break
    log.t = base.sim.t
    return log


def scan_points_in_base(scan: LaserScan, max_range: float = 1.5, keep: int = 120) -> NDArray[np.floating]:
    """Nearby LiDAR returns in base_link (karmel's LiDAR sits on the base_link z axis), thinned."""
    pts = scan.points()
    pts = pts[np.linalg.norm(pts, axis=1) <= max_range]
    if len(pts) > keep:
        pts = pts[np.linspace(0, len(pts) - 1, keep).astype(int)]
    return pts


def global_path(resolution: float = nav_common.GRID_RESOLUTION) -> NDArray[np.floating]:
    """The living-room -> kitchen path: cost-aware A* on the apartment map, shortcut, densified to 5 cm."""
    import costmap as cm
    import grid_planning as gp
    from robotlab.config import load_config
    from robotlab.sim import World

    cfg = load_config()
    grid = World.apartment().to_occupancy_grid(resolution)
    inscribed = cfg.chassis.footprint_radius_m
    costs = cm.inflate(cm.static_layer(grid), grid.resolution, inscribed, 3.0, 0.8)
    blocked, extra = cm.cost_to_planner_penalty(costs)
    res = gp.astar(blocked, grid.world_to_cell(*nav_common.START_XY), grid.world_to_cell(*nav_common.KITCHEN_XY), cell_cost=extra)
    smooth = gp.shortcut(costs >= 120, res.path)  # only shortcut through low-cost space
    return gp.densify(gp.cells_to_world(grid, smooth), 0.05)


def point_along(path: NDArray[np.floating], s: float) -> NDArray[np.floating]:
    return path[int(np.searchsorted(path_distances(path), s))]


def main() -> None:
    import matplotlib.pyplot as plt

    from robotlab.config import load_config
    from robotlab.sim import World, viz

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()

    cfg = load_config()
    radius = cfg.chassis.footprint_radius_m
    limits = Limits()
    path = global_path()
    apartment = World.apartment()
    basket = point_along(path, 1.3)
    world = World.from_segments(apartment.segments, np.vstack([apartment.circles, [[basket[0], basket[1], 0.15]]]))
    print(f"global path {path_distances(path)[-1]:.2f} m, {len(path)} points; laundry basket (r 0.15 m) at "
          f"({basket[0]:.2f}, {basket[1]:.2f}), on the path")

    def pp_policy(ctrl: PurePursuit):
        return lambda pose, scan, v, w: (*ctrl.compute(pose), ctrl.done)

    def local_goal(pose: Pose, s_ahead: float = 1.0):
        xy = np.array(pose[:2])
        i = int(np.argmin(np.linalg.norm(path - xy, axis=1)))
        j = min(int(np.searchsorted(path_distances(path), path_distances(path)[i] + s_ahead)), len(path) - 1)
        window = path[i : min(i + 40, len(path))]
        return to_robot_frame(pose, window), to_robot_frame(pose, path[j])[0]

    def reached(pose: Pose) -> bool:
        return bool(np.linalg.norm(path[-1] - np.array(pose[:2])) <= 0.08)

    def dwa_policy(pose, scan, v, w):
        if reached(pose):
            return 0.0, 0.0, True
        path_r, carrot_r = local_goal(pose)
        nv, nw, _ = dwa_choose(v, w, path_r, carrot_r, scan_points_in_base(scan), radius, limits, DWAConfig(), 0.1)
        return nv, nw, False

    mppi = MPPI(limits, radius)

    def mppi_policy(pose, scan, v, w):
        if reached(pose):
            return 0.0, 0.0, True
        path_r, carrot_r = local_goal(pose, 1.2)
        nv, nw, _ = mppi.compute(path_r, carrot_r, scan_points_in_base(scan))
        return nv, nw, False

    runs = []
    # pure pursuit on the clean apartment first: path tracking quality
    for label, regulated in (("pure pursuit", False), ("regulated pure pursuit", True)):
        ctrl = PurePursuit(path, regulated=regulated)
        log = run_in_sim(label + " (no basket)", apartment, path, pp_policy(ctrl), control_every=1, use_scan=False)
        cte = [cross_track_error(path, np.array(p[:2])) for p in log.poses]
        final = float(np.linalg.norm(np.array(log.poses[-1][:2]) - path[-1]))
        print(f"{log.name:34s} reached={log.reached} t={log.t:5.1f} s  max CTE {max(cte) * 100:4.1f} cm  "
              f"final error {final * 100:4.1f} cm  collided={log.collided}")
        runs.append(log)
    # now with the basket
    for label, policy, every in (("pure pursuit (basket)", pp_policy(PurePursuit(path)), 1),
                                 ("DWA (basket)", dwa_policy, 5),
                                 ("MPPI (basket)", mppi_policy, 5)):
        log = run_in_sim(label, world, path, policy, control_every=every, timeout_s=45.0, use_scan=every > 1)
        xy = np.array([p[:2] for p in log.poses])
        clearance = world.distance_to_obstacles(xy).min() - radius
        cte = max(cross_track_error(path, p) for p in xy)
        print(f"{log.name:34s} reached={log.reached} t={log.t:5.1f} s  min clearance {clearance * 100:5.1f} cm  "
              f"max deviation from path {cte * 100:4.1f} cm  collided={log.collided}")
        runs.append(log)

    fig, axes = plt.subplots(1, 2, figsize=(14, 4.2))
    for ax, subset, title, w_ in ((axes[0], runs[:2], "tracking the global path (no basket)", apartment),
                                  (axes[1], runs[2:], "unmapped laundry basket on the path", world)):
        viz.draw_world(ax, w_, show_landmarks=False)
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.plot(path[:, 0], path[:, 1], "k--", lw=1, label="global path")
        for log in subset:
            xy = np.array([p[:2] for p in log.poses])
            ax.plot(xy[:, 0], xy[:, 1], lw=2, label=f"{log.name}{'  COLLIDED' if log.collided else ''}")
        ax.set_xlim(0.5, 5.6)
        ax.set_ylim(0.5, 3.0)
        ax.legend(loc="lower right", fontsize=8)
    nav_common.save(fig, "12.05_local_planners")

    # one DWA decision, drawn: the sampled arcs coloured by cost
    pose = None
    for k, (p, s) in enumerate(zip(runs[3].poses, runs[3].scans)):
        if s is not None and k > 0 and np.linalg.norm(np.array(p[:2]) - basket) < 0.65:  # basket 0.5 m ahead
            pose, scan = runs[3].poses[k - 1], runs[3].scans[k]
            v0, w0 = runs[3].commands[k - 1]
            break
    if pose is not None:
        path_r, carrot_r = local_goal(pose)
        obstacles = scan_points_in_base(scan)
        v, w, dbg = dwa_choose(v0, w0, path_r, carrot_r, obstacles, radius, limits, DWAConfig(), 0.1)
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.set_aspect("equal")
        ax.set_title(f"DWA in base_link: light gray = all velocities, colored = dynamic window\n"
                     f"({len(dbg['cost'])} arcs, gray ones collide); chosen v={v:.2f} m/s, w={w:.2f} rad/s")
        every = rollout(*[a.ravel() for a in np.meshgrid(np.linspace(0, limits.v_max, 6), np.linspace(-limits.w_max, limits.w_max, 25))],
                        DWAConfig().sim_time, DWAConfig().dt)
        for tr in every:
            ax.plot(tr[:, 0], tr[:, 1], color="0.9", lw=0.8)
        cost = dbg["cost"]
        finite = np.isfinite(cost)
        norm = plt.Normalize(np.min(cost[finite]), np.percentile(cost[finite], 90))
        for k in range(len(cost)):
            tr = dbg["traj"][k]
            color = "0.8" if not finite[k] else plt.cm.viridis(norm(cost[k]))
            ax.plot(tr[:, 0], tr[:, 1], color=color, lw=1)
        best = int(np.argmin(cost))
        ax.plot(dbg["traj"][best][:, 0], dbg["traj"][best][:, 1], color="tab:red", lw=3, label="chosen")
        ax.scatter(obstacles[:, 0], obstacles[:, 1], s=6, color="k", label="LiDAR points")
        ax.plot(path_r[:, 0], path_r[:, 1], "k--", label="global path")
        ax.plot(*carrot_r, "c*", ms=14, label="carrot")
        ax.add_patch(plt.Circle((0, 0), radius, fill=False, color="tab:blue"))
        ax.set_xlim(-0.4, 1.6)
        ax.set_ylim(-0.9, 0.9)
        ax.legend(loc="lower left", fontsize=8)
        nav_common.save(fig, "12.05_dwa_window")

    if not args.no_gif:
        log = runs[4]
        anim = viz.animate(world, log.poses[::25], scans=log.scans[::25], robot_radius=radius, interval_ms=100)
        nav_common.OUT_DIR.mkdir(parents=True, exist_ok=True)
        out = nav_common.OUT_DIR / "12.05_mppi_basket.gif"
        anim.save(out, writer="pillow")
        plt.close("all")
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
