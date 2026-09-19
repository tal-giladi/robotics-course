"""Lesson 15.03 — hand-eye calibration: solving AX = XB, and checking the answer.

Everything runs offline on synthetic data, so you can see how the error depends on the number of
poses, the noise and — the part that surprises everyone — the *variety of rotations*.

    py hand_eye.py                    the tables printed in the lesson
    py hand_eye.py --only degenerate

Frames (the notation of 05-frames-and-transforms and 14-robotic-arm/code/arm_kinematics.py):

    T_a_b   pose of frame b expressed in frame a; T_a_c = T_a_b @ T_b_c

Eye-in-hand:  camera bolted to the gripper, target fixed on the table.
    unknown X = T_gripper_cam      (what you want)
    known     T_base_gripper       (forward kinematics)
    measured  T_cam_target         (solvePnP on the board, 13.06 / 13.07)

Eye-to-hand: camera bolted to the table, target on the gripper.
    unknown X = T_base_cam

The Park & Martin closed-form solver is implemented here in numpy (20 lines) so the lesson does
not depend on ``cv2.calibrateHandEye``, whose Python binding is missing from OpenCV 5.0. When the
binding *is* present (OpenCV 4.x, including Ubuntu 24.04's apt 4.6), ``--only opencv`` cross-checks
this implementation against all five OpenCV methods.
"""

from __future__ import annotations

import argparse
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# ============================================================================== SE(3) helpers
def rot_xyz(roll: float, pitch: float, yaw: float) -> Array:
    """R = Rz(yaw) Ry(pitch) Rx(roll) — the URDF/ROS convention."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def make_T(R: ArrayLike, t: ArrayLike) -> Array:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rotvec_of(R: ArrayLike) -> Array:
    """log of a rotation matrix: the axis-angle 3-vector (Rodrigues, without OpenCV)."""
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:                      # near 180 deg: use the symmetric part
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        axis = A[:, k] / axis[k] if axis[k] > 1e-9 else axis
        axis = axis / np.linalg.norm(axis)
        return axis * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def rot_of(rotvec: ArrayLike) -> Array:
    """exp of an axis-angle 3-vector (Rodrigues' formula)."""
    r = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def rotation_angle_deg(R: ArrayLike) -> float:
    """The angle of the rotation, degrees — the natural 'how wrong is this orientation' number."""
    return math.degrees(float(np.linalg.norm(rotvec_of(R))))


def pose_error(T_est: ArrayLike, T_true: ArrayLike) -> tuple[float, float]:
    """(translation error in mm, rotation error in degrees) between two poses."""
    d = inv_T(np.asarray(T_true, float)) @ np.asarray(T_est, float)
    return float(np.linalg.norm(d[:3, 3]) * 1000.0), rotation_angle_deg(d[:3, :3])


# ============================================================================== AX = XB
def motion_pairs(T_base_grippers: list[Array], T_cam_targets: list[Array],
                 *, eye_in_hand: bool = True) -> tuple[list[Array], list[Array]]:
    """Turn absolute poses into the relative motions of AX = XB.

    Eye-in-hand (target fixed): ``g_i X c_i`` is the same pose for every i, so

        A_ij = g_j^-1 g_i ,  B_ij = c_j c_i^-1 ,  A X = X B  with  X = T_gripper_cam

    Eye-to-hand (target on the gripper): ``g_i^-1 X c_i`` is constant, giving

        A_ij = g_j g_i^-1 ,  B_ij = c_j c_i^-1 ,  X = T_base_cam

    All C(n,2) pairs are used: 20 poses give 190 equations, and pairs with a large relative
    rotation are exactly the informative ones.
    """
    n = len(T_base_grippers)
    if n != len(T_cam_targets) or n < 3:
        raise ValueError("need at least 3 matching pose pairs (use >= 10 in practice)")
    A, B = [], []
    for i in range(n):
        for j in range(i + 1, n):
            gi, gj = T_base_grippers[i], T_base_grippers[j]
            A.append(inv_T(gj) @ gi if eye_in_hand else gj @ inv_T(gi))
            B.append(T_cam_targets[j] @ inv_T(T_cam_targets[i]))
    return A, B


def solve_ax_xb_park(A: list[Array], B: list[Array]) -> Array:
    """Park & Martin (1994) closed form for A X = X B. Returns the 4x4 X.

    Rotation: with alpha_i = log(R_Ai) and beta_i = log(R_Bi), the least-squares rotation is

        M = sum_i beta_i alpha_i^T ,   R_X = (M^T M)^(-1/2) M^T

    Translation: R_Ai t_X + t_Ai = R_X t_Bi + t_X, i.e. (R_Ai - I) t_X = R_X t_Bi - t_Ai,
    stacked over all pairs and solved by least squares.
    """
    M = np.zeros((3, 3))
    for Ai, Bi in zip(A, B, strict=True):
        alpha = rotvec_of(Ai[:3, :3])
        beta = rotvec_of(Bi[:3, :3])
        M += np.outer(beta, alpha)
    w, V = np.linalg.eigh(M.T @ M)
    w = np.maximum(w, 1e-18)
    inv_sqrt = V @ np.diag(1.0 / np.sqrt(w)) @ V.T
    R_X = inv_sqrt @ M.T
    u, _, vt = np.linalg.svd(R_X)                    # re-orthonormalise against numerical drift
    R_X = u @ vt
    if np.linalg.det(R_X) < 0:
        R_X = u @ np.diag([1.0, 1.0, -1.0]) @ vt

    C = np.zeros((3 * len(A), 3))
    d = np.zeros(3 * len(A))
    for k, (Ai, Bi) in enumerate(zip(A, B, strict=True)):
        C[3 * k:3 * k + 3, :] = Ai[:3, :3] - np.eye(3)
        d[3 * k:3 * k + 3] = R_X @ Bi[:3, 3] - Ai[:3, 3]
    t_X, *_ = np.linalg.lstsq(C, d, rcond=None)
    return make_T(R_X, t_X)


def refine_ax_xb(A: list[Array], B: list[Array], X0: Array) -> Array:
    """Nonlinear refinement of X: minimise the SE(3) residual of A X - X B over all pairs.

    The closed form minimises rotation and translation separately; this fixes the coupling. It
    typically buys a few per cent on clean data and more when the rotations are poorly spread.
    """
    from scipy.optimize import least_squares  # imported here so the solver works without scipy

    def residual(p: Array) -> Array:
        X = make_T(rot_of(p[:3]), p[3:])
        out = []
        for Ai, Bi in zip(A, B, strict=True):
            E = inv_T(Ai @ X) @ (X @ Bi)
            out.extend(rotvec_of(E[:3, :3]).tolist())
            out.extend(E[:3, 3].tolist())
        return np.asarray(out)

    p0 = np.concatenate([rotvec_of(X0[:3, :3]), X0[:3, 3]])
    sol = least_squares(residual, p0, method="lm", xtol=1e-12, ftol=1e-12)
    return make_T(rot_of(sol.x[:3]), sol.x[3:])


def calibrate_eye_in_hand(T_base_grippers: list[Array], T_cam_targets: list[Array],
                          *, refine: bool = True) -> Array:
    """X = T_gripper_cam — the camera's pose in the gripper frame."""
    A, B = motion_pairs(T_base_grippers, T_cam_targets, eye_in_hand=True)
    X = solve_ax_xb_park(A, B)
    return refine_ax_xb(A, B, X) if refine else X


def calibrate_eye_to_hand(T_base_grippers: list[Array], T_cam_targets: list[Array],
                          *, refine: bool = True) -> Array:
    """X = T_base_cam — the fixed camera's pose in the robot base frame."""
    A, B = motion_pairs(T_base_grippers, T_cam_targets, eye_in_hand=False)
    X = solve_ax_xb_park(A, B)
    return refine_ax_xb(A, B, X) if refine else X


# ============================================================================== validation
def target_in_base(T_base_grippers: list[Array], T_cam_targets: list[Array], X: Array) -> list[Array]:
    """Where each observation says the (fixed) target is, in the base frame. They must agree."""
    return [T_bg @ X @ T_ct for T_bg, T_ct in zip(T_base_grippers, T_cam_targets, strict=True)]


def consistency_residual(T_base_grippers: list[Array], T_cam_targets: list[Array],
                         X: Array) -> tuple[float, float]:
    """The check you can run **without ground truth**, on the real robot.

    The target does not move, so ``T_base_gripper_i @ X @ T_cam_target_i`` must be the same pose for
    every i. Returns (translation spread in mm, rotation spread in degrees) around their mean.
    """
    poses = target_in_base(T_base_grippers, T_cam_targets, X)
    centre = np.mean([T[:3, 3] for T in poses], axis=0)
    t_rms = float(np.sqrt(np.mean([np.sum((T[:3, 3] - centre) ** 2) for T in poses])) * 1000.0)
    R0 = poses[0][:3, :3]
    r_rms = float(np.sqrt(np.mean([rotation_angle_deg(R0.T @ T[:3, :3]) ** 2 for T in poses])))
    return t_rms, r_rms


def rotation_spread_deg(T_base_grippers: list[Array]) -> float:
    """Mean relative rotation angle over all pose pairs — the 'is my dataset varied enough' number."""
    n = len(T_base_grippers)
    angles = [rotation_angle_deg((inv_T(T_base_grippers[j]) @ T_base_grippers[i])[:3, :3])
              for i in range(n) for j in range(i + 1, n)]
    return float(np.mean(angles))


# ============================================================================== synthetic data
def default_X() -> Array:
    """Camera 35 mm above the tool point, 30 mm behind it, tilted 20 deg down, rolled -90 deg."""
    return make_T(rot_xyz(0.0, math.radians(20.0), 0.0) @ rot_xyz(math.radians(-90), 0, 0),
                  (0.0, -0.030, 0.035))


def random_gripper_poses(n: int, *, rng: np.random.Generator,
                         centre: ArrayLike = (0.22, 0.0, 0.18), spread_m: float = 0.07,
                         rot_spread_deg: float = 35.0, single_axis: bool = False) -> list[Array]:
    """``n`` plausible SO-101 gripper poses looking down at a board on the table.

    ``single_axis=True`` rotates only about z — the classic bad dataset that leaves AX = XB
    under-determined (the translation along the common rotation axis is unobservable).
    """
    centre = np.asarray(centre, dtype=float)
    s = math.radians(rot_spread_deg)
    poses = []
    for _ in range(n):
        t = centre + rng.uniform(-spread_m, spread_m, size=3)
        if single_axis:
            rpy = (math.pi, 0.0, rng.uniform(-s, s))
        else:
            rpy = (math.pi + rng.uniform(-s, s), rng.uniform(-s, s), rng.uniform(-math.pi, math.pi))
        poses.append(make_T(rot_xyz(*rpy), t))
    return poses


def perturb(T: Array, sigma_mm: float, sigma_deg: float, rng: np.random.Generator) -> Array:
    """Zero-mean Gaussian noise on a pose: a small rotation and a small translation."""
    if sigma_mm <= 0.0 and sigma_deg <= 0.0:
        return T.copy()
    out = T.copy()
    out[:3, :3] = T[:3, :3] @ rot_of(rng.normal(0.0, math.radians(sigma_deg), size=3))
    out[:3, 3] = T[:3, 3] + rng.normal(0.0, sigma_mm / 1000.0, size=3)
    return out


def synth_eye_in_hand(n: int, *, rng: np.random.Generator, X_true: Array | None = None,
                      T_base_target: Array | None = None, pnp_noise_deg: float = 0.0,
                      pnp_noise_mm: float = 0.0, fk_noise_deg: float = 0.0,
                      fk_noise_mm: float = 0.0, single_axis: bool = False,
                      ) -> tuple[list[Array], list[Array], Array, Array]:
    """(T_base_gripper list, T_cam_target list, X_true, T_base_target) for an eye-in-hand rig.

    The two noise pairs are the two real error sources: the PnP pose of the board (``pnp_noise_*``)
    and the arm's own kinematics and encoders (``fk_noise_*``).
    """
    X_true = default_X() if X_true is None else X_true
    if T_base_target is None:
        T_base_target = make_T(rot_xyz(0.0, 0.0, math.radians(12.0)), (0.25, 0.02, 0.0))
    T_base_grippers, T_cam_targets = [], []
    for T_bg in random_gripper_poses(n, rng=rng, single_axis=single_axis):
        T_ct = inv_T(T_bg @ X_true) @ T_base_target          # exact observation
        T_cam_targets.append(perturb(T_ct, pnp_noise_mm, pnp_noise_deg, rng))
        T_base_grippers.append(perturb(T_bg, fk_noise_mm, fk_noise_deg, rng))
    return T_base_grippers, T_cam_targets, X_true, T_base_target


def synth_eye_to_hand(n: int, *, rng: np.random.Generator, pnp_noise_deg: float = 0.0,
                      pnp_noise_mm: float = 0.0, fk_noise_deg: float = 0.0, fk_noise_mm: float = 0.0,
                      ) -> tuple[list[Array], list[Array], Array]:
    """(T_base_gripper list, T_cam_target list, T_base_cam) for a camera on a tripod."""
    T_gripper_target = make_T(rot_xyz(math.radians(10), 0.0, math.radians(-25)), (0.0, 0.0, 0.055))
    T_base_cam = make_T(rot_xyz(math.radians(-150), 0.0, math.radians(25)), (0.45, -0.30, 0.40))
    T_base_grippers, T_cam_targets = [], []
    for T_bg in random_gripper_poses(n, rng=rng):
        T_ct = inv_T(T_base_cam) @ T_bg @ T_gripper_target
        T_cam_targets.append(perturb(T_ct, pnp_noise_mm, pnp_noise_deg, rng))
        T_base_grippers.append(perturb(T_bg, fk_noise_mm, fk_noise_deg, rng))
    return T_base_grippers, T_cam_targets, T_base_cam


NOISE = dict(pnp_noise_deg=0.5, pnp_noise_mm=1.0, fk_noise_deg=0.3, fk_noise_mm=0.5)


# ============================================================================== demos
def demo_noise() -> None:
    print("--- eye-in-hand, 15 poses: where does the error come from? (mean of 20 seeds) ---")
    print(f"{'noise':<34}{'t err mm':>10}{'R err deg':>11}{'residual mm':>13}")
    cases = [
        ("perfect data", dict(pnp_noise_deg=0.0, pnp_noise_mm=0.0, fk_noise_deg=0.0, fk_noise_mm=0.0)),
        ("PnP only (0.5 deg, 1 mm)", dict(pnp_noise_deg=0.5, pnp_noise_mm=1.0, fk_noise_deg=0.0, fk_noise_mm=0.0)),
        ("FK only (0.3 deg, 0.5 mm)", dict(pnp_noise_deg=0.0, pnp_noise_mm=0.0, fk_noise_deg=0.3, fk_noise_mm=0.5)),
        ("both (the realistic case)", NOISE),
        ("sloppy arm (1.5 deg, 3 mm FK)", dict(pnp_noise_deg=0.5, pnp_noise_mm=1.0, fk_noise_deg=1.5, fk_noise_mm=3.0)),
    ]
    for label, noise in cases:
        dts, drs, rts = [], [], []
        for seed in range(20):
            rng = np.random.default_rng(100 + seed)
            A, B, X_true, _ = synth_eye_in_hand(15, rng=rng, **noise)
            X = calibrate_eye_in_hand(A, B)
            dt, dr = pose_error(X, X_true)
            dts.append(dt)
            drs.append(dr)
            rts.append(consistency_residual(A, B, X)[0])
        print(f"{label:<34}{np.mean(dts):>10.2f}{np.mean(drs):>11.3f}{np.mean(rts):>13.2f}")
    print()


def demo_n_poses() -> None:
    print("--- how many poses do you need? (realistic noise, mean of 20 seeds) ---")
    print(f"{'n poses':>8}{'pairs':>7}{'t err mm':>11}{'R err deg':>11}{'residual mm':>13}")
    for n in (3, 5, 8, 12, 20, 40):
        dts, drs, rts = [], [], []
        for seed in range(20):
            rng = np.random.default_rng(1000 + seed)
            A, B, X_true, _ = synth_eye_in_hand(n, rng=rng, **NOISE)
            X = calibrate_eye_in_hand(A, B)
            dt, dr = pose_error(X, X_true)
            dts.append(dt)
            drs.append(dr)
            rts.append(consistency_residual(A, B, X)[0])
        print(f"{n:>8}{n * (n - 1) // 2:>7}{np.mean(dts):>11.2f}{np.mean(drs):>11.3f}{np.mean(rts):>13.2f}")
    print()


def demo_degenerate() -> None:
    print("--- the dataset that silently fails: every pose rotated about the same axis ---")
    print(f"{'dataset':<30}{'mean pair rot':>14}{'t err mm':>10}{'R err deg':>11}{'residual mm':>13}")
    for label, single in (("varied rotations", False), ("z-axis rotations only", True)):
        dts, drs, rts, spreads = [], [], [], []
        for seed in range(20):
            rng = np.random.default_rng(50 + seed)
            A, B, X_true, _ = synth_eye_in_hand(20, rng=rng, single_axis=single, **NOISE)
            X = calibrate_eye_in_hand(A, B)
            dt, dr = pose_error(X, X_true)
            dts.append(dt)
            drs.append(dr)
            rts.append(consistency_residual(A, B, X)[0])
            spreads.append(rotation_spread_deg(A))
        print(f"{label:<30}{np.mean(spreads):>13.1f} {np.mean(dts):>10.2f}"
              f"{np.mean(drs):>11.3f}{np.mean(rts):>13.2f}")
    print("  the residual stays small in the bad case: a *consistent* answer is not a *correct* one.")
    print()


def demo_eye_to_hand() -> None:
    print("--- eye-to-hand: camera on a tripod, board bolted to the gripper (20 poses) ---")
    dts, drs = [], []
    for seed in range(20):
        rng = np.random.default_rng(300 + seed)
        A, B, T_base_cam = synth_eye_to_hand(20, rng=rng, **NOISE)
        X = calibrate_eye_to_hand(A, B)
        dt, dr = pose_error(X, T_base_cam)
        dts.append(dt)
        drs.append(dr)
    print(f"  T_base_cam error: {np.mean(dts):.2f} mm, {np.mean(drs):.3f} deg (mean of 20 seeds)")
    print()


def demo_opencv() -> None:
    """Cross-check against cv2.calibrateHandEye when the binding exists (OpenCV 4.x)."""
    try:
        import cv2
    except ImportError:
        print("OpenCV is not installed; skipping the cross-check.\n")
        return
    if not hasattr(cv2, "calibrateHandEye"):
        print(f"OpenCV {cv2.__version__} has no cv2.calibrateHandEye binding "
              f"(removed from the Python API in 5.0); the numpy solver above is the course path.\n")
        return
    methods = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
               "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
               "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}
    rng = np.random.default_rng(7)
    G, C, X_true, _ = synth_eye_in_hand(15, rng=rng, **NOISE)
    print(f"--- cross-check against OpenCV {cv2.__version__} (15 poses, realistic noise) ---")
    for name, flag in methods.items():
        R, t = cv2.calibrateHandEye([T[:3, :3] for T in G], [T[:3, 3] for T in G],
                                    [T[:3, :3] for T in C], [T[:3, 3] for T in C], method=flag)
        dt, dr = pose_error(make_T(R, t), X_true)
        print(f"  cv2 {name:<12}{dt:7.2f} mm{dr:8.3f} deg")
    dt, dr = pose_error(calibrate_eye_in_hand(G, C), X_true)
    print(f"  this module  {dt:7.2f} mm{dr:8.3f} deg")
    print()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["noise", "n", "degenerate", "eye-to-hand", "opencv"])
    args = ap.parse_args(argv)
    if args.only in (None, "noise"):
        demo_noise()
    if args.only in (None, "n"):
        demo_n_poses()
    if args.only in (None, "degenerate"):
        demo_degenerate()
    if args.only in (None, "eye-to-hand"):
        demo_eye_to_hand()
    if args.only in (None, "opencv"):
        demo_opencv()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
