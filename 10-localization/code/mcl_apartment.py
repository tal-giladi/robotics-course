"""10.08 — Monte Carlo localization of karmel in the simulated apartment.

    python 10-localization/code/mcl_apartment.py              # YOUR labs/exercises/10.08/student.py
    python 10-localization/code/mcl_apartment.py --solution   # the reference
    python 10-localization/code/mcl_apartment.py --solution --quick

1. Tracking from a known start: MCL vs dead reckoning, and how many particles you actually need.
2. Global localization: 3000 particles over the whole apartment, with no idea where the robot is.
3. Symmetry: the same filter in a plain rectangular room, where the belief stays bimodal forever.
4. Resampling: low-variance vs multinomial, and what resampling every step does to diversity.
5. Kidnapped at t = 20 s: plain MCL never recovers; 5 percent random injection does.
6. The two sensor models: likelihood field vs ray casting, accuracy and cost.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World, viz  # noqa: E402

K_WHEEL = 0.02   # m^0.5: motion-sampling noise. Particle filters like MORE noise than an EKF.
BEAMS = 40       # beams per scan after subsampling (AMCL's max_beams default is 60)


def mcl_sim():
    """The provided ``labs/exercises/10.08/mcl_sim.py`` helpers."""
    return lc.load_provided("10.08", "mcl_sim")


@dataclass(frozen=True)
class RoomLog:
    """The same fields ``run()`` needs from a ``TourLog``, for a world other than the apartment.

    (``tour_sim.drive_tour`` always builds ``World.apartment()``; part 3 needs a different room.)
    """

    t: np.ndarray
    truth: np.ndarray
    ticks: np.ndarray
    scans: list
    dt: float


def drive_room(world, waypoints, start=(1.0, 1.0, 0.0), seed=2, dt=0.02, speed=0.3,
               scan_every=25, hold: int = 0) -> RoomLog:
    """Drive ``world`` through ``waypoints`` with the realistic robot and log truth, ticks, scans."""
    cfg = lc.load_config()
    sim = DiffDriveSim(world, DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg),
                       pose=start, seed=seed, config=cfg)
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    t, truth, ticks, scans = [0.0], [sim.pose.as_tuple()], [sim.ticks], []
    i = k = 0
    for _ in range(hold):  # stand still: the robot gets scans but learns nothing new
        sim.set_velocity(0.0, 0.0)
        sim.step(dt)
        k += 1
        t.append(sim.t)
        truth.append(sim.pose.as_tuple())
        ticks.append(sim.ticks)
        if k % scan_every == 0:
            scans.append((k, sim.lidar_scan()))
    while i < len(waypoints) and k < 10_000:
        x, y, th = sim.pose.as_tuple()
        gx, gy = waypoints[i]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.12:
            i += 1
            continue
        err = float(lc.wrap_angle(math.atan2(gy - y, gx - x) - th))
        v = 0.0 if abs(err) > 0.6 else speed * min(1.0, dist / 0.3 + 0.3)
        w = max(-1.5, min(1.5, 2.5 * err))
        sim.set_velocity((v - w * b / 2.0) / r, (v + w * b / 2.0) / r)
        sim.step(dt)
        k += 1
        t.append(sim.t)
        truth.append(sim.pose.as_tuple())
        ticks.append(sim.ticks)
        if k % scan_every == 0:
            scans.append((k, sim.lidar_scan()))
    return RoomLog(np.array(t), np.array(truth), np.array(ticks, dtype=np.int64), scans, dt)


def run(ex, log, field, particles, k=K_WHEEL, beams=BEAMS, seed=0, inject=0.0, world=None,
        ess_ratio=0.5, weight_fn=None) -> dict:
    """Drive a whole tour log through the particle filter. Returns estimates, errors and health."""
    ms = mcl_sim()
    rng = np.random.default_rng(seed)
    b = lc.load_config().drive.wheel_separation_m
    travels = ms.wheel_travels(log.ticks)
    pf = ex.ParticleFilter(particles)
    scans = dict(log.scans)
    est, err, ess, sigma = [], [], [], []
    t0 = time.perf_counter()
    for i in range(1, len(log.t)):
        pf.predict(travels[i - 1], b, k, rng)
        scan = scans.get(i)
        if scan is not None:
            ranges, angles = ms.subsample_scan(scan, beams)
            if weight_fn is None:
                pf.update(ranges, angles, field)
            else:  # the beam model, injected for part 6
                w = pf.weights * weight_fn(pf.particles, ranges, angles)
                pf.weights = w / w.sum() if np.isfinite(w.sum()) and w.sum() > 0 else np.full(pf.n, 1.0 / pf.n)
            if inject:
                pf.inject_random(world, inject, rng)
            ess.append(ex.effective_sample_size(pf.weights))
            pf.resample(rng, ess_ratio)
        x = pf.estimate()
        est.append(x)
        err.append(math.hypot(x[0] - log.truth[i, 0], x[1] - log.truth[i, 1]))
        sigma.append(math.sqrt(max(pf.covariance()[0, 0], 0.0)))
    est, err = np.array(est), np.array(err)
    head = np.abs(lc.wrap_angle(est[:, 2] - log.truth[1:, 2]))
    return {
        "est": est, "err": err, "t": log.t[1:], "ess": np.array(ess), "sigma": np.array(sigma),
        "rmse": float(np.sqrt(np.mean(err**2))), "max": float(err.max()), "final": float(err[-1]),
        "heading_rmse_deg": math.degrees(float(np.sqrt(np.mean(head**2)))),
        "settled": float(err[-20:].mean()), "resampled": pf.resampled,
        "seconds": time.perf_counter() - t0, "pf": pf,
    }


def converged_at(res, threshold=0.20, stay=0.40):
    """First time the error drops below ``threshold`` and never rises above ``stay`` again."""
    good = np.flatnonzero(res["err"] < threshold)
    for i in good:
        if np.all(res["err"][i:] < stay):
            return float(res["t"][i])
    return None


def plot_cloud(world, log, res, particles_snapshots, path) -> None:
    fig, axes = plt.subplots(1, len(particles_snapshots), figsize=(4.2 * len(particles_snapshots), 4.0))
    for ax, (label, cloud, step) in zip(np.atleast_1d(axes), particles_snapshots):
        viz.draw_world(ax, world)
        ax.plot(cloud[:, 0], cloud[:, 1], ".", ms=1.0, color="tab:blue", alpha=0.35)
        viz.draw_robot(ax, log.truth[step])
        ax.set_title(label, fontsize=9)
        ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_runs(runs, path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    for label, res, color in runs:
        axes[0].plot(res["t"], res["err"] * 100, color=color, label=label)
        axes[1].plot(res["t"], res["sigma"] * 100, color=color, label=label)
    axes[0].set_ylabel("position error (cm)")
    axes[0].set_yscale("log")
    axes[1].set_ylabel("cloud sigma_x (cm)")
    axes[1].set_xlabel("time (s)")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--solution", action="store_true", help="use labs/exercises/10.08/solution.py")
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--quick", action="store_true", help="skip the particle-count sweep")
    parser.add_argument("--out", default="localization_out")
    args = parser.parse_args(argv)
    ex = lc.load_exercise("10.08", solution=args.solution)
    ms = mcl_sim()
    out = lc.out_dir(args.out)
    results: dict = {}

    world = World.apartment()
    field = ms.build_likelihood_field(world, resolution=0.05, max_dist=2.0)
    log = ms.drive_tour(seed=args.seed, observe_every=1000, scan_every=25)  # scans at 2 Hz
    dr = lc.dead_reckon(log.ticks, tuple(log.truth[0]))
    dr_err = np.hypot(dr[1:, 0] - log.truth[1:, 0], dr[1:, 1] - log.truth[1:, 1])

    # 1. tracking -----------------------------------------------------------------------------------
    print(f"1) Tracking a known start, {log.t[-1]:.0f} s tour, {len(log.scans)} scans, {BEAMS} beams each")
    print(f"   dead reckoning        : RMSE {np.sqrt(np.mean(dr_err**2)) * 100:5.1f} cm, final {dr_err[-1] * 100:5.1f} cm")
    track = {}
    counts = (100,) if args.quick else (50, 100, 300, 500, 1000, 3000)
    for n in counts:
        p0 = ex.initialize_gaussian(log.truth[0], [0.05, 0.05, 0.05], n, np.random.default_rng(1))
        res = run(ex, log, field, p0, seed=1)
        track[n] = res
        print(f"   MCL, {n:5d} particles : RMSE {res['rmse'] * 100:5.2f} cm, max {res['max'] * 100:5.1f} cm, "
              f"heading {res['heading_rmse_deg']:4.1f} deg, {res['resampled']:3d} resamplings, {res['seconds']:.1f} s")
    results["tracking"] = {n: {k: v for k, v in r.items() if np.isscalar(v)} for n, r in track.items()}
    results["dead_reckoning_rmse"] = float(np.sqrt(np.mean(dr_err**2)))

    # 2. global localization ------------------------------------------------------------------------
    print("\n2) Global localization: no initial pose at all")
    glob = {}
    for n in ((1000,) if args.quick else (500, 1000, 3000)):
        p0 = ex.initialize_uniform(world, n, np.random.default_rng(3))
        res = run(ex, log, field, p0, seed=3)
        glob[n] = res
        c = converged_at(res)
        print(f"   {n:5d} particles: converged at {('%.1f s' % c) if c else 'never':>8}, "
              f"settled error {res['settled'] * 100:5.2f} cm, {res['seconds']:.1f} s")
    results["global"] = {n: {"converged_s": converged_at(r), "settled": r["settled"]} for n, r in glob.items()}
    big = glob[3000] if 3000 in glob else list(glob.values())[-1]
    small = track[min(track)]
    plot_runs([(f"tracking, {min(track)} particles", small, "tab:blue"),
               ("global, 3000 particles", big, "tab:red")], out / "mcl_runs.png")

    # 3. symmetry -----------------------------------------------------------------------------------
    print("\n3) The same filter in a featureless rectangular room (4 x 3 m)")
    room = World.rectangle_room(4.0, 3.0)
    room_field = ms.build_likelihood_field(room, resolution=0.05, max_dist=2.0)
    loop = [(3.0, 1.0), (3.0, 2.0), (1.0, 2.0), (1.0, 1.0)]
    room_log = drive_room(room, loop if args.quick else loop * 2, seed=2)
    p0 = ex.initialize_uniform(room, 1200 if args.quick else 3000, np.random.default_rng(5))
    room_res = run(ex, room_log, room_field, p0, seed=5)
    pf = room_res["pf"]
    mirrored = np.hypot(pf.particles[:, 0] - (4.0 - room_log.truth[-1, 0]),
                        pf.particles[:, 1] - (3.0 - room_log.truth[-1, 1])) < 0.5
    near_truth = np.hypot(pf.particles[:, 0] - room_log.truth[-1, 0],
                          pf.particles[:, 1] - room_log.truth[-1, 1]) < 0.5
    print(f"   final error {room_res['err'][-1] * 100:.0f} cm, cloud sigma_x {room_res['sigma'][-1] * 100:.1f} cm")
    print(f"   particles near the truth: {near_truth.mean() * 100:.0f}%, near the 180-degree mirror pose: "
          f"{mirrored.mean() * 100:.0f}%")
    print("   A rectangle looks identical rotated by 180 degrees, and driving a symmetric loop does not")
    if mirrored.mean() > near_truth.mean():
        print("   break the tie. The cloud collapsed onto the WRONG one of the two poses and is now")
        print("   confident about it: the covariance is small and the estimate is 2 m out.")
    else:
        print("   break the tie. This time the cloud happened to collapse onto the RIGHT one of the two")
        print("   poses - which is luck, not localization: rerun with another seed and it flips.")
    print("   No filter can fix this - only a feature that breaks the symmetry, or an initial pose.")
    results["symmetry"] = {"final": room_res["err"][-1], "near_truth": float(near_truth.mean()),
                           "mirrored": float(mirrored.mean())}
    plot_cloud(room, room_log, room_res, [("rectangular room: bimodal belief", pf.particles, len(room_log.t) - 1)],
               out / "mcl_symmetry.png")

    # 4. resampling ---------------------------------------------------------------------------------
    print("\n4) Resampling")
    rng = np.random.default_rng(0)
    n = 200
    w = rng.random(n)
    w /= w.sum()
    lv = np.array([np.bincount(ex.low_variance_resample(w, float(rng.uniform(0, 1))), minlength=n) for _ in range(200)])
    mn = np.array([rng.multinomial(n, w) for _ in range(200)])
    lv_var = float(np.mean(np.var(lv - n * w, axis=0)))
    mn_var = float(np.mean(np.var(mn - n * w, axis=0)))
    print(f"   copy-count variance over 200 draws: low-variance {lv_var:.3f}, multinomial {mn_var:.3f} "
          f"({mn_var / lv_var:.1f}x)")
    print(f"   worst |copies - N*w|: low-variance {np.abs(lv - n * w).max():.2f} (never more than 1), "
          f"multinomial {np.abs(mn - n * w).max():.2f}")
    # Particle depletion: a robot that is NOT moving gets no new information, but every resampling
    # still throws particles away. k = 0 removes the motion noise that normally re-scatters them.
    still = drive_room(world, [], start=(1.0, 1.3, 0.0), seed=1, hold=1000)  # 20 s of standing still
    depletion = {}
    for label, ratio in (("every scan", 1e9), ("when ESS < N/2", 0.5)):
        p0 = ex.initialize_gaussian(log.truth[0], [0.10, 0.10, 0.10], 300, np.random.default_rng(1))
        res = run(ex, still, field, p0, k=0.0, seed=1, ess_ratio=ratio)
        unique = len(np.unique(res["pf"].particles[:, 0]))
        depletion[label] = (res["resampled"], unique)
        print(f"   standing still, 300 particles, resample {label:<15}: {res['resampled']:3d} resamplings, "
              f"{unique:3d} distinct particles left")
    results["resampling"] = {"lv_var": lv_var, "mn_var": mn_var, "depletion": depletion}

    # 5. kidnapping ---------------------------------------------------------------------------------
    print("\n5) Kidnapped at t = 20 s and put down in the bedroom")
    kid = ms.drive_tour(seed=args.seed, observe_every=1000, scan_every=25, kidnap_at_s=20.0,
                        kidnap_to=(5.0, 4.2, 0.0))
    p0 = ex.initialize_gaussian(kid.truth[0], [0.05, 0.05, 0.05], 1500, np.random.default_rng(7))
    plain = run(ex, kid, field, p0.copy(), seed=7)
    inject = run(ex, kid, field, p0.copy(), seed=7, inject=0.05, world=world)
    late = kid.t[1:] > 40.0
    print(f"   plain MCL           : error before 20 s {plain['err'][kid.t[1:] < 19][-1] * 100:5.1f} cm, "
          f"after 40 s {plain['err'][late].mean() * 100:6.1f} cm")
    print(f"   + 5% random particles: error before 20 s {inject['err'][kid.t[1:] < 19][-1] * 100:5.1f} cm, "
          f"after 40 s {inject['err'][late].mean() * 100:6.1f} cm")
    results["kidnap"] = {"plain_late": float(plain["err"][late].mean()), "inject_late": float(inject["err"][late].mean()),
                         "plain_before": float(plain["err"][kid.t[1:] < 19][-1]),
                         "inject_before": float(inject["err"][kid.t[1:] < 19][-1])}
    plot_runs([("plain MCL", plain, "tab:orange"), ("5% random injection", inject, "tab:green")],
              out / "mcl_kidnap.png")

    # 6. the two sensor models ----------------------------------------------------------------------
    n_models = 150 if args.quick else 500
    print(f"\n6) Sensor models, {n_models} particles")
    p0 = ex.initialize_gaussian(log.truth[0], [0.05, 0.05, 0.05], n_models, np.random.default_rng(1))
    beam = run(ex, log, field, p0.copy(), seed=1,
               weight_fn=lambda p, r, a: ex.beam_weights(p, r, a, world))
    lf = track.get(n_models) or run(ex, log, field, p0.copy(), seed=1)
    print(f"   likelihood field: RMSE {lf['rmse'] * 100:5.2f} cm in {lf['seconds']:5.1f} s")
    print(f"   beam (raycast)  : RMSE {beam['rmse'] * 100:5.2f} cm in {beam['seconds']:5.1f} s "
          f"({beam['seconds'] / lf['seconds']:.0f}x slower)")
    results["models"] = {"lf_rmse": lf["rmse"], "lf_s": lf["seconds"], "beam_rmse": beam["rmse"], "beam_s": beam["seconds"]}

    print(f"\nwrote {out / 'mcl_runs.png'}, {out / 'mcl_symmetry.png'} and {out / 'mcl_kidnap.png'}")
    return results


if __name__ == "__main__":
    main()
