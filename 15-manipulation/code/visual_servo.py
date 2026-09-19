"""Lesson 15.06 — visual servoing: driving the camera error to zero instead of aiming once.

Everything runs offline. The "camera" is a pinhole projection of four known points; the "arm" is
the SO-101 chain from 14-robotic-arm/code/arm_kinematics.py, so the joint-space part uses the real
Jacobian and the real joint limits.

    py visual_servo.py                    every table printed in the lesson
    py visual_servo.py --only retreat     just the camera-retreat experiment

Two control laws, both of the form  v_c = -lambda * L^+ e:

    PBVS  the error lives in SE(3):   e = (t_cam_target - t*, theta u)     "where is the object"
    IBVS  the error lives in pixels:  e = s(t) - s*                        "what does it look like"

Frames follow REP-104: the optical frame is x right, y down, z forward, and a velocity screw is
written v = (vx, vy, vz, wx, wy, wz) in the *camera* frame.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray

# The arm chain of module 14 — the same file 14.05/14.06 use, not a copy.
_ARM_CODE = Path(__file__).resolve().parents[2] / "14-robotic-arm" / "code"
if str(_ARM_CODE) not in sys.path:
    sys.path.insert(0, str(_ARM_CODE))
import arm_kinematics as ak  # noqa: E402

Array = NDArray[np.float64]


# ============================================================================== SE(3) helpers
def skew(v: ArrayLike) -> Array:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def rotvec_of(R: ArrayLike) -> Array:
    """theta * u of a rotation matrix (the 'orientation error' PBVS uses directly)."""
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        axis = A[:, k] / axis[k]
        return axis / float(np.linalg.norm(axis)) * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def rot_of(rotvec: ArrayLike) -> Array:
    """Rodrigues: exp of an axis-angle 3-vector."""
    r = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = skew(k)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def integrate_pose(T: Array, v: ArrayLike, dt: float) -> Array:
    """Move a frame by a body-frame twist v = (vx, vy, vz, wx, wy, wz) for dt seconds.

    Exact (matrix-exponential) integration, so a large step does not spuriously shrink the
    rotation: T <- T @ exp(v_hat * dt).
    """
    v = np.asarray(v, dtype=float).reshape(6)
    R = rot_of(v[3:] * dt)
    out = np.asarray(T, dtype=float).copy()
    out[:3, 3] = out[:3, 3] + out[:3, :3] @ (v[:3] * dt)
    out[:3, :3] = out[:3, :3] @ R
    return out


def camera_twist_to_tool_twist(v_cam: ArrayLike, T_base_cam: ArrayLike,
                               T_base_tool: ArrayLike) -> Array:
    """Re-express a camera twist as the tool twist the arm Jacobian of 14.06 can serve.

    Two changes happen at once, and forgetting the second is the classic bug:

    1. **Rotate** both halves into base coordinates (the Jacobian's coordinates):
       omega_base = R_base_cam omega_cam, v_base = R_base_cam v_cam.
    2. **Move the reference point** from the camera's optical centre to the tool point:
       v_tool = v_cam + omega x (p_tool - p_cam). A camera 35 mm off the tool that is asked to
       rotate at 1 rad/s must have the tool translating at 35 mm/s, or it is not the same motion.
    """
    v = np.asarray(v_cam, dtype=float).reshape(6)
    Tbc = np.asarray(T_base_cam, dtype=float)
    Tbt = np.asarray(T_base_tool, dtype=float)
    R = Tbc[:3, :3]
    omega = R @ v[3:]
    lin = R @ v[:3] + np.cross(omega, Tbt[:3, 3] - Tbc[:3, 3])
    return np.concatenate([lin, omega])


# ============================================================================== the camera
@dataclass(frozen=True)
class PinholeCamera:
    """Intrinsics of the wrist camera: the 640x480 USB webcam, calibrated in 13.06."""

    fx: float = 525.0
    fy: float = 525.0
    cx: float = 319.5
    cy: float = 239.5
    width: int = 640
    height: int = 480

    def project(self, points_cam: ArrayLike) -> Array:
        """(N, 3) points in the optical frame -> (N, 2) pixels. Z must be > 0."""
        p = np.asarray(points_cam, dtype=float).reshape(-1, 3)
        if np.any(p[:, 2] <= 1e-6):
            raise ValueError("a feature is at or behind the image plane (Z <= 0)")
        return np.column_stack((self.fx * p[:, 0] / p[:, 2] + self.cx,
                                self.fy * p[:, 1] / p[:, 2] + self.cy))

    def normalize(self, pixels: ArrayLike) -> Array:
        """(N, 2) pixels -> (N, 2) normalised image coordinates x = (u - cx)/fx, y = (v - cy)/fy.

        IBVS is written in normalised coordinates, never in pixels: that is what makes the
        interaction matrix independent of the lens.
        """
        uv = np.asarray(pixels, dtype=float).reshape(-1, 2)
        return np.column_stack(((uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy))

    def in_view(self, pixels: ArrayLike, margin_px: float = 0.0) -> bool:
        uv = np.asarray(pixels, dtype=float).reshape(-1, 2)
        return bool(np.all((uv[:, 0] >= margin_px) & (uv[:, 0] <= self.width - 1 - margin_px)
                           & (uv[:, 1] >= margin_px) & (uv[:, 1] <= self.height - 1 - margin_px)))


# ============================================================================== IBVS
def interaction_matrix_point(x: float, y: float, Z: float) -> Array:
    """The 2x6 image Jacobian of ONE normalised point feature (x, y) at depth Z.

        [ -1/Z   0    x/Z    x y     -(1 + x^2)   y  ]
        [  0   -1/Z   y/Z   1 + y^2    -x y      -x  ]

    Row 1 is how x moves, row 2 how y moves, per unit of camera twist (vx, vy, vz, wx, wy, wz).
    Note the asymmetry that explains everything IBVS does badly: the translation columns are
    divided by Z, the rotation columns are not.
    """
    if Z <= 1e-9:
        raise ValueError("depth must be positive")
    return np.array([
        [-1.0 / Z, 0.0, x / Z, x * y, -(1.0 + x * x), y],
        [0.0, -1.0 / Z, y / Z, 1.0 + y * y, -x * y, -x],
    ])


def interaction_matrix(features: ArrayLike, depths: ArrayLike) -> Array:
    """Stack ``interaction_matrix_point`` for N normalised features -> (2N, 6)."""
    f = np.asarray(features, dtype=float).reshape(-1, 2)
    Z = np.asarray(depths, dtype=float).reshape(-1)
    if len(Z) != len(f):
        raise ValueError("need one depth per feature")
    return np.vstack([interaction_matrix_point(float(x), float(y), float(z))
                      for (x, y), z in zip(f, Z, strict=True)])


def damped_pinv(L: ArrayLike, damping: float = 0.0) -> Array:
    """L^+ = L^T (L L^T + lambda^2 I)^-1 for a wide L, or (L^T L + lambda^2 I)^-1 L^T for a tall one.

    Same damped least squares as 14.06, applied to the image Jacobian instead of the arm's.
    """
    L = np.asarray(L, dtype=float)
    m, n = L.shape
    if m <= n:
        return L.T @ np.linalg.solve(L @ L.T + damping ** 2 * np.eye(m), np.eye(m))
    return np.linalg.solve(L.T @ L + damping ** 2 * np.eye(n), L.T)


def ibvs_velocity(features: Array, features_star: Array, depths: ArrayLike, *,
                  gain: float = 0.8, damping: float = 0.01) -> Array:
    """Camera twist that reduces the image error: v_c = -gain * L^+ (s - s*).

    ``depths`` is the depth *estimate* — IBVS's one unavoidable piece of 3D knowledge. Using the
    goal depth Z* for every iteration is the standard cheap approximation and it still converges;
    ``--only depth`` measures what it costs.
    """
    e = (np.asarray(features, dtype=float) - np.asarray(features_star, dtype=float)).reshape(-1)
    L = interaction_matrix(features, depths)
    return -gain * (damped_pinv(L, damping) @ e)


# ============================================================================== a 4-DOF task
def moment_features(features: ArrayLike) -> Array:
    """Four numbers that describe how a blob *looks*: (xg, yg, a, alpha).

    xg, yg  centroid in normalised image coordinates      -> where the object is in the image
    a       sqrt of the mean squared radius               -> apparent size, i.e. 1 / distance
    alpha   orientation of the point set, rad             -> how it is turned in the image

    This is the classical image-moment feature set, and it is the right one for a 5-DOF arm:
    four constraints, so the arm has one joint left over instead of one task it cannot serve.
    """
    f = np.asarray(features, dtype=float).reshape(-1, 2)
    g = f.mean(axis=0)
    c = f - g
    mu20 = float(np.mean(c[:, 0] ** 2))
    mu02 = float(np.mean(c[:, 1] ** 2))
    mu11 = float(np.mean(c[:, 0] * c[:, 1]))
    a = math.sqrt(max(mu20 + mu02, 1e-18))
    alpha = 0.5 * math.atan2(2.0 * mu11, mu20 - mu02)
    return np.array([g[0], g[1], a, alpha])


def moment_interaction(features: ArrayLike, depths: ArrayLike, eps: float = 1e-6) -> Array:
    """The 4x6 interaction matrix of ``moment_features``, by the chain rule.

        L_moment = (d moments / d points) @ L_points

    The left factor is a 4x2N numerical Jacobian. There is a closed form in Chaumette's papers,
    but this is ten lines, exact to 1e-8, and it works for any feature you can write as a
    function of the points — which is the transferable idea.
    """
    f = np.asarray(features, dtype=float).reshape(-1, 2)
    flat = f.reshape(-1)
    A = np.zeros((4, flat.size))
    base = moment_features(f)
    for k in range(flat.size):
        bumped = flat.copy()
        bumped[k] += eps
        d = moment_features(bumped.reshape(-1, 2)) - base
        d[3] = wrap_pi_2(float(d[3]))
        A[:, k] = d / eps
    return A @ interaction_matrix(f, depths)


def wrap_pi_2(a: float) -> float:
    """Wrap an angle to (-pi/2, pi/2]: the point set's orientation is an axis, not a direction."""
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def moment_error_px(features: Array, features_star: Array, cam: PinholeCamera) -> float:
    """The moment error in one honest unit: pixels of feature motion.

    The size term is already a normalised radius, and an orientation error ``alpha`` at radius
    ``a`` moves a feature by ``a * alpha``, so every component converts with the focal length.
    """
    m, m_star = moment_features(features), moment_features(features_star)
    e = m - m_star
    return max(abs(e[0]) * cam.fx, abs(e[1]) * cam.fy, abs(e[2]) * cam.fx,
               abs(wrap_pi_2(float(e[3]))) * m[2] * cam.fx)


def moment_velocity(features: Array, features_star: Array, depths: ArrayLike, *,
                    gain: float = 0.8, damping: float = 0.01) -> Array:
    """v_c = -gain * L^+ (m - m*) on the four moment features."""
    m = moment_features(features)
    m_star = moment_features(features_star)
    e = m - m_star
    e[3] = wrap_pi_2(float(e[3]))
    L = moment_interaction(features, depths)
    return -gain * (damped_pinv(L, damping) @ e)


# ============================================================================== PBVS
def pbvs_velocity(T_cam_obj: ArrayLike, T_cam_obj_star: ArrayLike, *, gain: float = 0.8) -> Array:
    """Camera twist from a *pose* error — the 'position-based' law.

    With T_err = (T_cam_obj*)^-1 T_cam_obj (the object's pose seen from where the camera should
    end up), the camera must move by exactly that pose, so

        v = +gain * (t_err, theta u_err)      expressed in the camera frame

    This is a straight line in Cartesian space, and it makes no promise at all about the image.
    """
    T_err = inv_T(T_cam_obj_star) @ np.asarray(T_cam_obj, dtype=float)
    return gain * np.concatenate([T_err[:3, 3], rotvec_of(T_err[:3, :3])])


# ============================================================================== the target
def block_target(long_m: float = 0.060, short_m: float = 0.030) -> Array:
    """The four top corners of the 60 x 30 mm wooden block of 15.04, in the object's own frame.

    A **rectangle**, not a square, and that is not cosmetic: a square's second moments are
    isotropic, so its image orientation is undefined and the ``alpha`` feature below is garbage.
    Choose features your object can actually supply — the first rule of IBVS in practice.
    """
    a, b = long_m / 2.0, short_m / 2.0
    return np.array([[-a, -b, 0.0], [a, -b, 0.0], [a, b, 0.0], [-a, b, 0.0]])


def points_in_camera(T_cam_obj: ArrayLike, points_obj: ArrayLike) -> Array:
    T = np.asarray(T_cam_obj, dtype=float)
    return np.asarray(points_obj, dtype=float) @ T[:3, :3].T + T[:3, 3]


@dataclass
class ServoResult:
    """What one servoing run did. ``reason`` is the interesting field, not ``iterations``."""

    converged: bool
    iterations: int
    reason: str
    error_px: float
    pose_err_mm: float
    pose_err_deg: float
    path_length_m: float
    max_depth_m: float
    min_depth_m: float
    left_view: bool = False
    trace: list[Array] = field(default_factory=list)

    def line(self, label: str) -> str:
        return (f"{label:<26}{self.iterations:>6}{self.error_px:>11.2f}{self.pose_err_mm:>11.2f}"
                f"{self.pose_err_deg:>10.2f}{self.path_length_m * 1000:>11.0f}"
                f"{self.max_depth_m * 1000:>11.0f}  {self.reason}")


def servo_free_camera(T_cam_obj0: Array, T_cam_obj_star: Array, *, law: str = "ibvs",
                      cam: PinholeCamera | None = None, points_obj: Array | None = None,
                      gain: float = 0.8, dt: float = 0.05, max_iterations: int = 600,
                      depth_mode: str = "goal", depth_scale: float = 1.0,
                      pixel_noise_px: float = 0.0, latency_steps: int = 0,
                      tol_px: float = 0.5, rng: np.random.Generator | None = None) -> ServoResult:
    """Servo a free-flying camera (6 DOF, no arm) from one pose to another.

    ``law``: "ibvs" or "pbvs". ``depth_mode``: "true" (cheating), "goal" (use Z*, the usual
    choice), or "constant" (one number for every feature). ``depth_scale`` multiplies the depth
    estimate, so 0.5 means "I think everything is twice as close as it is".
    ``latency_steps`` delays the measurement by that many control periods — the real reason a
    servo loop that works in simulation oscillates on a robot.
    """
    cam = cam or PinholeCamera()
    points_obj = block_target() if points_obj is None else points_obj
    rng = np.random.default_rng(0) if rng is None else rng

    s_star = cam.normalize(cam.project(points_in_camera(T_cam_obj_star, points_obj)))
    Z_star = points_in_camera(T_cam_obj_star, points_obj)[:, 2]

    T = np.asarray(T_cam_obj0, dtype=float).copy()      # object pose in the camera frame
    trace = [T.copy()]
    history: list[Array] = []
    path = 0.0
    depths_seen: list[float] = []
    left_view = False

    for it in range(1, max_iterations + 1):
        p_cam = points_in_camera(T, points_obj)
        depths_seen.extend(p_cam[:, 2].tolist())
        try:
            uv = cam.project(p_cam)
        except ValueError:
            return _finish(False, it, "feature behind the camera", cam, T, T_cam_obj_star,
                           points_obj, path, depths_seen, True, trace)
        if not cam.in_view(uv):
            left_view = True
        # The convergence test uses the TRUE image error, not the delayed/noisy measurement:
        # otherwise a laggy loop appears to converge early because it is looking at old data.
        true_err_px = float(np.linalg.norm(uv - cam.project(
            points_in_camera(T_cam_obj_star, points_obj)), axis=1).max())
        if true_err_px < tol_px:
            return _finish(True, it, "converged", cam, T, T_cam_obj_star, points_obj,
                           path, depths_seen, left_view, trace)

        if pixel_noise_px > 0.0:
            uv = uv + rng.normal(0.0, pixel_noise_px, uv.shape)
        history.append(uv)
        s = cam.normalize(history[max(0, len(history) - 1 - latency_steps)])   # delayed

        if law == "ibvs":
            if depth_mode == "true":
                Z = points_in_camera(T, points_obj)[:, 2]
            elif depth_mode == "goal":
                Z = Z_star
            else:
                Z = np.full(len(points_obj), float(np.mean(Z_star)))
            v = ibvs_velocity(s, s_star, np.asarray(Z) * depth_scale, gain=gain)
        elif law == "pbvs":
            # PBVS needs the object pose: recovered from the (possibly noisy, possibly delayed)
            # image by PnP on the robot; here the geometry is exact, so use the pose directly.
            v = pbvs_velocity(T, T_cam_obj_star, gain=gain)
        else:
            raise ValueError(f"unknown law {law!r}")

        # The camera moves by +v; the object's pose in the camera frame therefore moves by -v.
        T_new = integrate_pose(inv_T(T), v, dt)
        T = inv_T(T_new)
        path += float(np.linalg.norm(v[:3]) * dt)
        trace.append(T.copy())

    return _finish(False, max_iterations, "did not converge", cam, T, T_cam_obj_star,
                   points_obj, path, depths_seen, left_view, trace)


def _finish(ok: bool, it: int, reason: str, cam: PinholeCamera, T: Array, T_star: Array,
            points_obj: Array, path: float, depths: list[float], left: bool,
            trace: list[Array], err_override: float | None = None) -> ServoResult:
    try:
        uv = cam.project(points_in_camera(T, points_obj))
        uv_star = cam.project(points_in_camera(T_star, points_obj))
        err_px = float(np.linalg.norm(uv - uv_star, axis=1).max())
    except ValueError:
        err_px = float("nan")
    if err_override is not None and not math.isnan(err_px):
        err_px = err_override
    T_err = inv_T(T_star) @ T
    return ServoResult(ok, it, reason, err_px,
                       float(np.linalg.norm(T_err[:3, 3]) * 1000.0),
                       math.degrees(float(np.linalg.norm(rotvec_of(T_err[:3, :3])))),
                       path, max(depths) if depths else 0.0, min(depths) if depths else 0.0,
                       left, trace)


# ============================================================================== on the arm
def default_T_gripper_cam() -> Array:
    """Wrist camera 35 mm above the tool point and 30 mm behind it, tilted 20 deg down.

    The same X that 15.03 calibrates — here it is known exactly, which is the point of
    15.06-E4: a *wrong* X barely matters for IBVS and matters a lot for open-loop reaching.
    """
    R = ak.rpy_to_matrix(0.0, math.radians(20.0), 0.0) @ ak.rpy_to_matrix(math.radians(-90), 0, 0)
    return ak.se3(R, (0.0, -0.030, 0.035))


def servo_arm(q0: ArrayLike, T_base_obj: ArrayLike, T_cam_obj_star: ArrayLike, *,
              chain: ak.SerialChain | None = None, T_gripper_cam: Array | None = None,
              cam: PinholeCamera | None = None, points_obj: Array | None = None,
              gain: float = 0.8, dt: float = 0.05, max_iterations: int = 600,
              damping: float = 0.02, joint_speed_limit: float = math.radians(60.0),
              law: str = "ibvs", tol_px: float = 1.0,
              X_error_mm: float = 0.0) -> tuple[ServoResult, Array, list[Array]]:
    """IBVS on the SO-101: image error -> camera twist -> tool twist -> joint velocities.

    The chain of three Jacobians the lesson is about:

        v_cam  = -gain L^+ e                     (interaction matrix, this file)
        v_tool = Ad(T_tool_cam) v_cam            (a fixed 6x6, from the hand-eye X of 15.03)
        q_dot  = J^T (J J^T + lambda^2 I)^-1 v   (the arm Jacobian of 14.06, damped)

    The SO-101 has **five** joints, so J is 6x5: one camera DOF is not available and the damped
    solve silently trades it away. ``X_error_mm`` perturbs the hand-eye transform used by the
    controller (not the one used by the simulated camera) to show how little IBVS cares.
    """
    chain = chain or ak.load_so101()
    X = default_T_gripper_cam() if T_gripper_cam is None else np.asarray(T_gripper_cam, float)
    cam = cam or PinholeCamera()
    points_obj = block_target() if points_obj is None else points_obj
    X_ctrl = X.copy()
    X_ctrl[0, 3] += X_error_mm / 1000.0                      # the controller's belief is wrong

    s_star = cam.normalize(cam.project(points_in_camera(T_cam_obj_star, points_obj)))
    Z_star = points_in_camera(T_cam_obj_star, points_obj)[:, 2]

    q = chain.clip(np.asarray(q0, dtype=float))
    qs = [q.copy()]
    path = 0.0
    depths: list[float] = []
    left_view = False
    prev_tool = chain.fk(q)[:3, 3]

    for it in range(1, max_iterations + 1):
        T_base_cam = chain.fk(q) @ X
        T_cam_obj = inv_T(T_base_cam) @ np.asarray(T_base_obj, dtype=float)
        p_cam = points_in_camera(T_cam_obj, points_obj)
        depths.extend(p_cam[:, 2].tolist())
        try:
            uv = cam.project(p_cam)
        except ValueError:
            return (_finish(False, it, "feature behind the camera", cam, T_cam_obj,
                            T_cam_obj_star, points_obj, path, depths, True, []), q, qs)
        if not cam.in_view(uv):
            left_view = True
        s = cam.normalize(uv)
        if law == "moment":
            err_px = moment_error_px(s, s_star, cam)
        else:
            err_px = float(np.linalg.norm((s - s_star) * np.array([cam.fx, cam.fy]), axis=1).max())
        if err_px < tol_px:
            return (_finish(True, it, "converged", cam, T_cam_obj, T_cam_obj_star, points_obj,
                            path, depths, left_view, [],
                            err_px if law == "moment" else None), q, qs)

        if law == "ibvs":
            v_cam = ibvs_velocity(s, s_star, Z_star, gain=gain)
        elif law == "moment":
            v_cam = moment_velocity(s, s_star, Z_star, gain=gain)
        else:
            v_cam = pbvs_velocity(T_cam_obj, T_cam_obj_star, gain=gain)

        T_base_tool = chain.fk(q)
        v_tool = camera_twist_to_tool_twist(v_cam, T_base_tool @ X_ctrl, T_base_tool)
        J = chain.geometric_jacobian(q)                       # 6 x 5 (14.06)
        dq = ak.dls_solve(J, v_tool, damping)
        biggest = float(np.max(np.abs(dq)))
        if biggest > joint_speed_limit:                       # scale the whole vector (14.06)
            dq *= joint_speed_limit / biggest
        q = chain.clip(q + dq * dt)
        tool = chain.fk(q)[:3, 3]
        path += float(np.linalg.norm(tool - prev_tool))
        prev_tool = tool
        qs.append(q.copy())

    T_base_cam = chain.fk(q) @ X
    T_cam_obj = inv_T(T_base_cam) @ np.asarray(T_base_obj, dtype=float)
    s_final = cam.normalize(cam.project(points_in_camera(T_cam_obj, points_obj)))
    return (_finish(False, max_iterations, "did not converge", cam, T_cam_obj, T_cam_obj_star,
                    points_obj, path, depths, left_view, [],
                    moment_error_px(s_final, s_star, cam) if law == "moment" else None), q, qs)


# ============================================================================== demos
HEADER = (f"{'case':<26}{'iters':>6}{'err px':>11}{'pos mm':>11}{'rot deg':>10}"
          f"{'path mm':>11}{'max Z mm':>11}  reason")


def _start_pose(dx: float = 0.05, dy: float = 0.03, dz: float = 0.10,
                roll_deg: float = 0.0, yaw_deg: float = 20.0) -> Array:
    """Object pose in the camera frame at the start of a servo run."""
    R = rot_of([math.radians(roll_deg), 0.0, 0.0]) @ rot_of([0.0, 0.0, math.radians(yaw_deg)])
    return ak.se3(R, (dx, dy, 0.25 + dz))


def goal_pose() -> Array:
    """Where the object should sit in the camera frame when the servo is done: straight ahead,
    250 mm away, square to the lens. On the robot this is the pre-grasp view of 15.07."""
    return ak.se3(np.eye(3), (0.0, 0.0, 0.25))


def demo_compare() -> None:
    print("--- free-flying camera, 4-point 60 x 30 mm block target, goal = 250 mm straight ahead ---")
    print(HEADER)
    cases = [
        ("small error", _start_pose(0.04, 0.02, 0.05, 0.0, 15.0)),
        ("large translation", _start_pose(0.15, -0.12, 0.20, 0.0, 10.0)),
        ("large rotation 80 deg", _start_pose(0.02, 0.01, 0.03, 0.0, 80.0)),
        ("tilted 35 deg", _start_pose(0.03, 0.02, 0.06, 35.0, 25.0)),
    ]
    for law in ("ibvs", "pbvs"):
        for label, T0 in cases:
            r = servo_free_camera(T0, goal_pose(), law=law)
            print(r.line(f"{law.upper():<5} {label}"))
        print()


def demo_retreat() -> None:
    print("--- the camera-retreat failure: a pure rotation about the optical axis ---")
    print("    (the object is already centred and at the right distance; only its roll is wrong)")
    print(HEADER)
    for roll in (30.0, 90.0, 150.0, 179.0):
        T0 = ak.se3(rot_of([0.0, 0.0, math.radians(roll)]), (0.0, 0.0, 0.25))
        for law in ("ibvs", "pbvs"):
            r = servo_free_camera(T0, goal_pose(), law=law, max_iterations=1200)
            print(r.line(f"{law.upper():<5} roll {roll:.0f} deg"))
    print("  IBVS moves the four image points along straight lines, and for a large roll the only")
    print("  way to do that is to shrink the target first: the camera retreats along z. At 179 deg")
    print("  the straight image paths all pass through the centre and it retreats to infinity.")
    print("  PBVS rotates in place: the pose error is a pure rotation, so the twist is too.\n")


def demo_depth() -> None:
    print("--- IBVS and the depth it does not know (goal Z* = 250 mm, start 350 mm) ---")
    print(f"{'depth estimate':<26}{'iters':>7}{'converged':>11}{'path mm':>10}")
    T0 = _start_pose(0.10, -0.08, 0.10, 0.0, 25.0)
    for label, kw in (("true Z every step", dict(depth_mode="true")),
                      ("goal Z* (the usual)", dict(depth_mode="goal")),
                      ("one constant Z", dict(depth_mode="constant")),
                      ("Z* scaled x0.5", dict(depth_mode="goal", depth_scale=0.5)),
                      ("Z* scaled x2.0", dict(depth_mode="goal", depth_scale=2.0)),
                      ("Z* scaled x5.0", dict(depth_mode="goal", depth_scale=5.0))):
        r = servo_free_camera(T0, goal_pose(), law="ibvs", **kw)
        print(f"{label:<26}{r.iterations:>7}{str(r.converged):>11}{r.path_length_m * 1000:>10.0f}")
    print("  A wrong depth scales the commanded velocity, it does not change its sign: IBVS still")
    print("  converges, just faster or slower. That is the whole reason to prefer it when your")
    print("  depth is bad - which, after 15.04, you know it is.\n")


def demo_gain_and_latency() -> None:
    print("--- gain and latency: the two numbers that decide whether the loop is stable ---")
    print(f"{'gain':>6}{'latency':>9}{'iters':>7}{'settle s':>10}{'path mm':>9}"
          f"{'max Z mm':>10}  note")
    T0 = _start_pose(0.08, -0.05, 0.08, 0.0, 20.0)
    straight_line_mm = float(np.linalg.norm(T0[:3, 3] - goal_pose()[:3, 3])) * 1000.0
    print(f"  (the straight-line distance from start to goal is {straight_line_mm:.0f} mm)")
    for gain in (0.2, 0.8, 2.0, 4.0, 6.0):
        for lat in (0, 2, 6):
            r = servo_free_camera(T0, goal_pose(), law="ibvs", gain=gain, latency_steps=lat,
                                  max_iterations=800)
            note = ("" if r.converged else
                    "DIVERGED" if math.isnan(r.error_px) else "did not settle in 40 s")
            print(f"{gain:>6.1f}{f'{lat * 50} ms':>9}{r.iterations:>7}{r.iterations * 0.05:>10.1f}"
                  f"{r.path_length_m * 1000:>9.0f}{r.max_depth_m * 1000:>10.0f}  {note}")
    print("  dt = 50 ms (a 20 Hz vision loop). Latency is measured in control periods: 6 steps is")
    print("  300 ms, which is what an unoptimised detector + USB camera actually costs (13.15).\n")


def demo_noise() -> None:
    print("--- pixel noise: where does the servo SETTLE? (gain 0.8, 20 runs, 400 steps each) ---")
    print("    convergence is disabled on purpose: the loop keeps running and we measure the jitter")
    print(f"{'noise px':>10}{'feature mm at 250 mm':>22}{'settled pos err mm':>21}{'settled rot deg':>18}")
    T0 = _start_pose(0.06, -0.04, 0.06, 0.0, 20.0)
    T_star = goal_pose()
    for noise in (0.0, 0.3, 1.0, 3.0):
        pos, rot = [], []
        for seed in range(20):
            r = servo_free_camera(T0, T_star, law="ibvs", pixel_noise_px=noise, tol_px=0.0,
                                  max_iterations=400, rng=np.random.default_rng(seed))
            for T in r.trace[-150:]:                       # the settled part of the run
                E = inv_T(T_star) @ T
                pos.append(float(np.linalg.norm(E[:3, 3]) * 1000.0))
                rot.append(math.degrees(float(np.linalg.norm(rotvec_of(E[:3, :3])))))
        feature_mm = noise / 525.0 * 0.250 * 1000.0
        print(f"{noise:>10.1f}{feature_mm:>22.2f}{float(np.sqrt(np.mean(np.square(pos)))):>21.2f}"
              f"{float(np.sqrt(np.mean(np.square(rot)))):>18.3f}")
    print("  The servo does not average the noise away, it chases it: a 1 px standard deviation")
    print("  at 250 mm with f = 525 px is 0.48 mm of apparent feature motion, and the settled")
    print("  pose error tracks it. Lower the gain and it jitters less and lags more.\n")


def demo_arm() -> None:
    chain = ak.load_so101()
    print("--- IBVS on the SO-101 (5 joints, 6 camera DOF: one is not available) ---")
    print(HEADER)
    q0 = np.radians([10.0, -35.0, 55.0, 40.0, 0.0])
    T_base_cam0 = chain.fk(q0) @ default_T_gripper_cam()
    for label, T0 in (("centred, 60 mm away", _start_pose(0.02, 0.01, 0.06, 0.0, 15.0)),
                      ("offset 80 mm", _start_pose(0.08, -0.05, 0.08, 0.0, 25.0)),
                      ("rolled 45 deg", _start_pose(0.02, 0.01, 0.05, 45.0, 10.0))):
        T_base_obj = T_base_cam0 @ T0
        for law in ("ibvs", "pbvs", "moment"):
            r, q, _ = servo_arm(q0, T_base_obj, goal_pose(), law=law, chain=chain,
                                max_iterations=700)
            print(r.line(f"{law.upper():<6} {label}"))
        print()
    print("  IBVS and PBVS both ask for a full 6-DOF camera pose and both stall: the SO-101 has")
    print("  five joints, so one camera DOF simply is not for sale, and the damped solve settles")
    print("  at the least-squares compromise. MOMENT asks for four numbers (centroid, apparent")
    print("  size, in-image angle) and converges, because that task fits the arm.\n")
    print("  the hand-eye X the controller uses is deliberately wrong (moment law, 700 steps):")
    print(f"{'X error mm':>12}{'iters':>7}{'converged':>11}{'final err px':>14}{'path mm':>10}")
    T_base_obj = T_base_cam0 @ _start_pose(0.06, -0.04, 0.07, 0.0, 20.0)
    for err_mm in (0.0, 5.0, 20.0, 50.0):
        r, q, _ = servo_arm(q0, T_base_obj, goal_pose(), law="moment", chain=chain,
                            X_error_mm=err_mm, max_iterations=700)
        print(f"{err_mm:>12.0f}{r.iterations:>7}{str(r.converged):>11}{r.error_px:>14.2f}"
              f"{r.path_length_m * 1000:>10.0f}")
    print("  The image error still goes to zero: a wrong X bends the path, it does not move the")
    print("  goal, because the goal is defined in the image. Compare with 15.03, where the same")
    print("  20 mm of hand-eye error went straight into the grasp and stayed there.\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["compare", "retreat", "depth", "gain", "noise", "arm"])
    args = ap.parse_args(argv)
    if args.only in (None, "compare"):
        demo_compare()
    if args.only in (None, "retreat"):
        demo_retreat()
    if args.only in (None, "depth"):
        demo_depth()
    if args.only in (None, "gain"):
        demo_gain_and_latency()
    if args.only in (None, "noise"):
        demo_noise()
    if args.only in (None, "arm"):
        demo_arm()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
