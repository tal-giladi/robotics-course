"""10.10 — Localizing karmel from printed AprilTags at known map positions.

    python 10-localization/code/tag_localization.py --solution
    python 10-localization/code/tag_localization.py --solution --quick --repeats 1

1. The tag map: ten tag36h11 markers on the apartment's walls, with the pose of each one measured
   once and written down. A tag map is a map — and it is only as good as your tape measure.
2. What a tag detection actually costs you: render a real image of a tag with the course camera
   (13.05-13.07), detect it, run solvePnP, and MEASURE the range and bearing error against distance
   and viewing angle. That measurement IS the EKF's R matrix.
3. Visibility: field of view, minimum apparent size, grazing angle and occlusion decide when a tag
   is usable. Over the apartment tour, how often does karmel see one?
4. The EKF of 10.06, fed tag detections instead of the simulator's idealised landmarks.
5. The failure that matters: one tag entered 40 cm wrong in the map, and what the NIS gate does
   about it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402

CV_CODE = lc.ROOT / "13-computer-vision" / "code"
if str(CV_CODE) not in sys.path:
    sys.path.insert(0, str(CV_CODE))

from camera_model import inv_T, karmel_T_base_optical, karmel_camera, make_T, rot_z  # noqa: E402
from markers import R_BASE_TAG_FACING, detect_and_estimate  # noqa: E402
from synthetic import marker_scene  # noqa: E402

TAG_SIZE_M = 0.10        # the black square of a tag printed on A4
TAG_HEIGHT_M = 0.25      # tags are taped this high on the wall; the camera sits at 0.145 m
TAG_DICT_SIZE = 10       # World.apartment() has ten landmarks -> ten tag ids, 0..9
MAX_RANGE_M = 4.0        # beyond this a 10 cm tag is a handful of pixels
MAX_INCIDENCE_RAD = math.radians(65.0)   # a tag seen edge-on cannot be decoded reliably
HFOV_RAD = 1.20          # karmel.yaml webcam, horizontal field of view
# The rendered images have pixel noise but perfect optics: no calibration error, no rolling
# shutter, no motion blur, no printer scaling. Those dominate on a real robot, so never hand the
# measured sigmas straight to a filter - floor them at values you can defend outdoors of a lab.
SIGMA_REL_FLOOR = 0.02            # 2 % of the range
SIGMA_BEARING_FLOOR = math.radians(0.5)

# Which way each of World.apartment()'s ten landmarks faces (outward normal of the wall it is on).
TAG_FACING_RAD = [
    0.0,              # (0.0, 1.8)  west wall of the living room, faces +x
    -math.pi / 2,     # (1.0, 3.2)  living room / study divider, faces -y
    math.pi,          # (3.5, 0.6)  east wall of the living room, faces -x
    math.pi / 2,      # (2.0, 0.0)  south wall, faces +y
    0.0,              # (0.0, 4.0)  west wall of the study, faces +x
    -math.pi / 2,     # (2.8, 5.0)  north wall of the study, faces -y
    math.pi,          # (6.0, 3.5)  east wall of the bedroom, faces -x
    -math.pi / 2,     # (4.0, 5.0)  north wall of the bedroom, faces -y
    math.pi,          # (6.0, 2.4)  east wall of the kitchen, faces -x
    math.pi / 2,      # (4.5, 0.0)  south wall of the kitchen, faces +y
]


# --- the tag map ---------------------------------------------------------------------------------
def tag_map(world) -> dict[int, tuple[float, float, float]]:
    """``{tag id: (x, y, facing yaw)}`` in the map frame, from the world's landmark positions."""
    return {int(i): (float(p[0]), float(p[1]), TAG_FACING_RAD[int(i)])
            for i, p in zip(world.landmark_ids, world.landmarks)}


def T_map_tag(x: float, y: float, facing: float) -> np.ndarray:
    """4x4 pose of a wall-mounted tag. ``facing`` is the direction its front normal points.

    ``markers.R_BASE_TAG_FACING`` is the orientation of a tag whose normal points along **-x**
    (i.e. squarely at a robot sitting at the origin looking down +x), so a tag whose normal should
    point along ``facing`` is that rotated by ``facing - pi``. Getting this by pi wrong is the
    classic fiducial bug: every tag decodes, and every pose is mirrored.
    """
    return make_T(rot_z(facing - math.pi) @ R_BASE_TAG_FACING, (x, y, TAG_HEIGHT_M))


def visible(pose, tag, world, max_range=MAX_RANGE_M) -> tuple[bool, float, float, float]:
    """``(usable, range, bearing, incidence)`` of one tag from a robot pose, all in SI units.

    Four independent reasons a tag is not usable, and every one of them bites in practice:
    it is too far for a 10 cm square to be decoded, it is outside the camera's field of view, it is
    seen too obliquely, or a wall is in the way.
    """
    x, y, th = pose
    tx, ty, facing = tag
    dx, dy = tx - x, ty - y
    r = math.hypot(dx, dy)
    bearing = float(lc.wrap_angle(math.atan2(dy, dx) - th))
    incidence = abs(float(lc.wrap_angle(math.atan2(-dy, -dx) - facing)))  # tag normal vs the robot
    if r > max_range or abs(bearing) > HFOV_RAD / 2 or incidence > MAX_INCIDENCE_RAD:
        return False, r, bearing, incidence
    hit = float(world.raycast((x, y), [math.atan2(dy, dx)], max_range + 1.0)[0])
    return hit >= r - 0.05, r, bearing, incidence


# --- what a detection really costs ---------------------------------------------------------------
def detect_once(range_m: float, bearing_rad: float, incidence_rad: float, rng, cam,
                tag_size=TAG_SIZE_M, noise_sigma=3.0) -> dict | None:
    """Render one image of a tag at a known relative pose, detect it, and report the error.

    Returns ``None`` when the detector does not find the tag — which is itself a measurement.
    """
    T_base_optical = karmel_T_base_optical()
    # Put the tag at (range, bearing) in base_link, at TAG_HEIGHT_M, turned by the incidence angle.
    tx = range_m * math.cos(bearing_rad)
    ty = range_m * math.sin(bearing_rad)
    facing = math.atan2(-ty, -tx) + incidence_rad     # normal direction, tilted by the incidence
    T_base_tag = T_map_tag(tx, ty, facing)
    image = marker_scene(cam, inv_T(T_base_optical) @ T_base_tag, rng, marker_id=0,
                         marker_len_m=tag_size, noise_sigma=noise_sigma)
    found = detect_and_estimate(image, cam.K, None, tag_size)
    if not found:
        return None
    _, T_optical_tag, errs, ratio, _ = found[0]
    t = (T_base_optical @ T_optical_tag)[:3, 3]
    r_hat, b_hat = math.hypot(t[0], t[1]), math.atan2(t[1], t[0])
    return {"range": range_m, "bearing": bearing_rad, "incidence": incidence_rad,
            "range_err": r_hat - range_m, "bearing_err": float(lc.wrap_angle(b_hat - bearing_rad)),
            "reproj": float(errs[0]), "ambiguity": ratio}


def characterise(cam, rng, ranges, incidences, repeats: int) -> list[dict]:
    """Detect a tag at every (range, incidence) combination ``repeats`` times."""
    rows = []
    for r in ranges:
        for inc in incidences:
            for _ in range(repeats):
                row = detect_once(r, 0.0, inc, rng, cam)
                rows.append(row if row else {"range": r, "bearing": 0.0, "incidence": inc,
                                             "range_err": math.nan, "bearing_err": math.nan,
                                             "reproj": math.nan, "ambiguity": math.nan})
    return rows


def noise_model(rows: list[dict]) -> tuple[float, float, float]:
    """Fit sigma_r = a * range (proportional error) and a constant sigma_bearing.

    Returns ``(a, sigma_bearing_rad, detection_rate)``.
    """
    ok = [r for r in rows if math.isfinite(r["range_err"])]
    if not ok:
        return math.nan, math.nan, 0.0
    d = np.array([r["range"] for r in ok])
    er = np.array([r["range_err"] for r in ok])
    eb = np.array([r["bearing_err"] for r in ok])
    a = float(np.sqrt(np.mean((er / d) ** 2)))           # RMS of the relative range error
    return a, float(np.sqrt(np.mean(eb**2))), len(ok) / len(rows)


# --- the EKF over the tour with tag measurements --------------------------------------------------
def run_tour(ex, log, tags, world, sigma_rel: float, sigma_bearing: float, seed: int = 0,
             every: int = 10, map_error: dict | None = None, gate: float = 5.991,
             k_wheel: float = 0.01) -> dict:
    """10.06's EKF, corrected by tag detections instead of ``observe_landmarks()``.

    ``map_error``: ``{tag id: (dx, dy)}`` — a tag whose position in YOUR map is wrong.
    """
    rng = np.random.default_rng(seed)
    b = lc.load_config().drive.wheel_separation_m
    travels = lc.wheel_travels(log.ticks)
    ekf = ex.EKF(log.truth[0], np.diag([0.02**2, 0.02**2, 0.02**2]))
    est, nis, seen, rejected, per_step = [ekf.x.copy()], [], 0, 0, []
    for i in range(1, len(log.t)):
        u = travels[i - 1]
        ekf.predict(u, b, ex.wheel_noise(u, k_wheel))
        n_here = 0
        check = i % every == 0
        if check:
            for tag_id, tag in tags.items():
                ok, r, bearing, _ = visible(log.truth[i], tag, world)
                if not ok:
                    continue
                n_here += 1
                seen += 1
                sr = max(sigma_rel * r, 1e-4)
                z = [r + rng.normal(0.0, sr), bearing + rng.normal(0.0, sigma_bearing)]
                lx, ly, _ = tag
                if map_error and tag_id in map_error:
                    lx, ly = lx + map_error[tag_id][0], ly + map_error[tag_id][1]
                R = np.diag([sr**2, sigma_bearing**2])
                nu, S, _ = ekf.innovation(z, [lx, ly], R)
                d2 = float(nu @ np.linalg.solve(S, nu))
                if d2 >= gate:
                    rejected += 1
                    continue
                nis.append(ekf.update(z, [lx, ly], R))
        if check:
            per_step.append(n_here)
        est.append(ekf.x.copy())
    est = np.array(est)
    err = np.hypot(est[:, 0] - log.truth[:, 0], est[:, 1] - log.truth[:, 1])
    head = np.abs(lc.wrap_angle(est[:, 2] - log.truth[:, 2]))
    return {
        "est": est, "err": err, "rmse": float(np.sqrt(np.mean(err**2))), "max": float(err.max()),
        "final": float(err[-1]), "heading_rmse_deg": math.degrees(float(np.sqrt(np.mean(head**2)))),
        "nis": float(np.mean(nis)) if nis else math.nan, "updates": len(nis),
        "seen": seen, "rejected": rejected, "per_step": np.array(per_step),
    }


def plot_visibility(world, log, tags, res, path) -> None:
    from robotlab.sim import viz

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    viz.draw_world(axes[0], world)
    for tag_id, (x, y, facing) in tags.items():
        axes[0].plot(x, y, "s", color="tab:red", ms=5)
        axes[0].arrow(x, y, 0.25 * math.cos(facing), 0.25 * math.sin(facing),
                      head_width=0.08, color="tab:red", length_includes_head=True)
        axes[0].annotate(str(tag_id), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=7)
    viz.draw_trajectory(axes[0], log.truth, color="k", lw=1.0, label="truth")
    viz.draw_trajectory(axes[0], res["est"], color="tab:blue", lw=1.0, label="EKF with tags")
    axes[0].set_title("tag map and the tour", fontsize=9)
    axes[0].legend(fontsize=7)
    axes[1].step(np.linspace(0, log.t[-1], len(res["per_step"])), res["per_step"], where="post", color="tab:purple")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("tags in view")
    axes[1].set_title("how often karmel can see a tag", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--solution", action="store_true", help="use labs/exercises/10.06/solution.py")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--repeats", type=int, default=2, help="detections per (range, angle) cell")
    parser.add_argument("--quick", action="store_true", help="fewer renders")
    parser.add_argument("--out", default="localization_out")
    args = parser.parse_args(argv)
    ex = lc.load_exercise("10.06", solution=args.solution)   # the EKF from 10.06
    out = lc.out_dir(args.out)
    rng = np.random.default_rng(args.seed)
    cam = karmel_camera()
    from robotlab.sim import World

    world = World.apartment()
    tags = tag_map(world)
    results: dict = {}

    # 1. one detection, by the numbers --------------------------------------------------------------
    print("1) One tag, one image, one pose")
    row = detect_once(1.50, 0.0, 0.0, rng, cam)
    print(f"   tag {TAG_SIZE_M * 100:.0f} cm at 1.50 m, straight ahead, square on:")
    print(f"   range error {row['range_err'] * 1000:+.1f} mm, bearing error {math.degrees(row['bearing_err']):+.3f} deg, "
          f"reprojection {row['reproj']:.3f} px, ambiguity ratio {row['ambiguity']:.2f}")
    results["single"] = {k: v for k, v in row.items()}

    # 2. the noise model ----------------------------------------------------------------------------
    ranges = (0.5, 1.0, 2.0, 3.0) if args.quick else (0.5, 1.0, 1.5, 2.0, 3.0, 4.0)
    incidences = (0.0, math.radians(45.0)) if args.quick else (0.0, math.radians(30.0), math.radians(60.0))
    t0 = time.perf_counter()
    rows = characterise(cam, rng, ranges, incidences, args.repeats)
    print(f"\n2) Measured detection error ({len(rows)} rendered images, {time.perf_counter() - t0:.0f} s)")
    print("   range   incidence   detected   range err (mm)   bearing err (deg)   reproj (px)")
    table = {}
    for r in ranges:
        for inc in incidences:
            cell = [x for x in rows if x["range"] == r and x["incidence"] == inc]
            ok = [x for x in cell if math.isfinite(x["range_err"])]
            if not ok:
                print(f"   {r:4.1f} m   {math.degrees(inc):5.0f} deg      0/{len(cell)}              -                   -            -")
                table[(r, round(math.degrees(inc)))] = None
                continue
            er = np.array([x["range_err"] for x in ok]) * 1000
            eb = np.degrees([x["bearing_err"] for x in ok])
            rp = np.mean([x["reproj"] for x in ok])
            print(f"   {r:4.1f} m   {math.degrees(inc):5.0f} deg   {len(ok):3d}/{len(cell):<3d}       "
                  f"{er.mean():+7.1f}         {eb.mean():+9.3f}        {rp:6.3f}")
            table[(r, round(math.degrees(inc)))] = {"n": len(ok), "range_mm": float(er.mean()),
                                                    "bearing_deg": float(eb.mean()), "reproj": float(rp)}
    a_raw, sb_raw, rate = noise_model(rows)
    a, sigma_b = max(a_raw, SIGMA_REL_FLOOR), max(sb_raw, SIGMA_BEARING_FLOOR)
    print(f"   measured: sigma_range = {a_raw * 100:.2f} % of the range, "
          f"sigma_bearing = {math.degrees(sb_raw):.3f} deg, detection rate {rate * 100:.0f}%")
    print(f"   used    : {a * 100:.2f} % and {math.degrees(sigma_b):.2f} deg after flooring for the")
    print("             calibration, rolling-shutter and motion-blur errors this render has none of")
    print(f"   -> at 2 m, R = diag({(a * 2) ** 2:.2e} m^2, {sigma_b ** 2:.2e} rad^2) "
          f"(sigma {a * 2 * 100:.1f} cm, {math.degrees(sigma_b):.2f} deg)")

    print("\n2b) Tag size sets your range (detections out of 3, straight on)")
    for size in (0.10, 0.20):
        line = []
        for r in ((1.0, 2.0) if args.quick else (1.0, 2.0, 3.0, 4.0)):
            hits = [detect_once(r, 0.0, 0.0, rng, cam, tag_size=size) for _ in range(1 if args.quick else 3)]
            ok = [h for h in hits if h]
            px = size / r * cam.fx
            line.append(f"{r:.0f} m: {len(ok)}/{1 if args.quick else 3} ({px:4.0f} px)")
        print(f"   {size * 100:3.0f} cm tag   " + "   ".join(line))
    results["table"] = {f"{k[0]}m_{k[1]}deg": v for k, v in table.items()}
    results["fit"] = {"sigma_rel_measured": a_raw, "sigma_bearing_measured": sb_raw,
                      "sigma_rel": a, "sigma_bearing": sigma_b, "detection_rate": rate}

    # 3-4. visibility and the EKF over the tour ------------------------------------------------------
    log = lc.drive_tour(seed=args.seed, observe_every=10)
    dr = lc.dead_reckon(log.ticks, tuple(log.truth[0]))
    dr_err = np.hypot(dr[:, 0] - log.truth[:, 0], dr[:, 1] - log.truth[:, 1])
    good = run_tour(ex, log, tags, world, a, sigma_b, seed=args.seed)
    frac = float(np.mean(good["per_step"] > 0))
    print(f"\n3) Visibility over the {log.t[-1]:.0f} s tour (checks at 5 Hz)")
    print(f"   a tag was in view at {frac * 100:.0f}% of the checks; {good['seen']} detections, "
          f"at most {int(good['per_step'].max())} tags at once")
    print("\n4) EKF with tag measurements")
    print(f"   dead reckoning     : RMSE {np.sqrt(np.mean(dr_err**2)) * 100:5.1f} cm, final {dr_err[-1] * 100:5.1f} cm")
    print(f"   EKF with tags      : RMSE {good['rmse'] * 100:5.2f} cm, max {good['max'] * 100:5.2f} cm, "
          f"heading {good['heading_rmse_deg']:.2f} deg, NIS {good['nis']:.2f} ({good['updates']} updates)")
    results["visibility"] = {"fraction": frac, "detections": good["seen"],
                             "max_at_once": int(good["per_step"].max())}
    results["ekf"] = {k: v for k, v in good.items() if np.isscalar(v)}
    results["dead_reckoning"] = {"rmse": float(np.sqrt(np.mean(dr_err**2))), "final": float(dr_err[-1])}
    plot_visibility(world, log, tags, good, out / "tag_map.png")

    # 5. a tag in the wrong place --------------------------------------------------------------------
    print("\n5) One tag written into the map 40 cm from where it really is (tag 2, on the east wall"
          "\n   of the living room - the tag karmel sees most often)")
    for gate, name in ((1e9, "no gate"), (5.991, "chi-square 95% gate")):
        bad = run_tour(ex, log, tags, world, a, sigma_b, seed=args.seed,
                       map_error={2: (0.40, 0.0)}, gate=gate)
        print(f"   {name:<22}: RMSE {bad['rmse'] * 100:5.2f} cm, max {bad['max'] * 100:5.2f} cm, "
              f"NIS {bad['nis']:6.2f}, {bad['rejected']:3d} of {bad['seen']} rejected")
        results[f"maperror_{name.replace(' ', '_')}"] = {k: v for k, v in bad.items() if np.isscalar(v)}

    print(f"\nwrote {out / 'tag_map.png'}")
    return results


if __name__ == "__main__":
    main()
