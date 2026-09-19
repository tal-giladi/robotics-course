"""arm_kinematics — forward kinematics, Jacobians and inverse kinematics in plain numpy (module 14).

Two families of arms, one notation:

* **Planar arms** (``planar_*``, ``two_link_ik``): n revolute joints in a vertical or horizontal
  plane, link lengths ``lengths``. Used to learn FK/IK/Jacobians with numbers you can check by hand.
* **Serial chains from a URDF** (``Joint``, ``SerialChain``, ``load_so101``): the real SO-101,
  built from ``data/so101_kinematics.yaml`` (copied verbatim from the official URDF).

Notation (the same as 05-frames-and-transforms/code/frames_math.py):

    T_a_b      = the pose of frame b expressed in frame a (4x4), maps points from b to a
    T_a_c      = T_a_b @ T_b_c
    FK(q)      = T_base_tool(q) = T_base_1(q1) @ T_1_2(q2) @ ... @ T_n_tool

Units: meters, radians. Angles of planar arms are measured counter-clockwise from the x axis;
each joint angle is relative to the previous link.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]
HERE = Path(__file__).resolve().parent
SO101_YAML = HERE / "data" / "so101_kinematics.yaml"


# ============================================================================================
# Small SE(3) helpers — identical in meaning to frames_math (test_arm_kinematics checks that)
# ============================================================================================
def rot_x(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Array:
    """URDF / ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def axis_angle_to_matrix(axis: ArrayLike, angle: float) -> Array:
    """Rodrigues' formula: rotation by ``angle`` about the unit vector ``axis``."""
    k = np.asarray(axis, dtype=float)
    n = np.linalg.norm(k)
    if n < 1e-12:
        raise ValueError("rotation axis must be non-zero")
    kx, ky, kz = k / n
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def se3(R: ArrayLike | None = None, t: ArrayLike = (0.0, 0.0, 0.0)) -> Array:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def se3_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> Array:
    """Same meaning as a URDF ``<origin xyz="..." rpy="..."/>``."""
    return se3(rpy_to_matrix(*rpy), xyz)


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


# ============================================================================================
# Planar arms
# ============================================================================================
def planar_joint_points(lengths: Sequence[float], q: Sequence[float]) -> Array:
    """(n+1, 2) array: base, every joint, and the tip, for a planar arm.

    Link i points along the cumulative angle q1 + ... + qi.
    """
    if len(lengths) != len(q):
        raise ValueError(f"{len(lengths)} links but {len(q)} joint angles")
    pts = np.zeros((len(lengths) + 1, 2))
    phi = 0.0
    for i, (l, qi) in enumerate(zip(lengths, q)):
        phi += qi
        pts[i + 1] = pts[i] + l * np.array([math.cos(phi), math.sin(phi)])
    return pts


def planar_fk(lengths: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
    """Tip (x, y, phi) of a planar arm; phi = sum(q) is the last link's direction, wrapped."""
    x, y = planar_joint_points(lengths, q)[-1]
    return float(x), float(y), wrap_angle(float(sum(q)))


def planar_fk_homogeneous(lengths: Sequence[float], q: Sequence[float]) -> Array:
    """The same FK as a product of 3x3 transforms: prod_i  Rot(q_i) @ Trans(l_i, 0)."""
    T = np.eye(3)
    for l, qi in zip(lengths, q):
        c, s = math.cos(qi), math.sin(qi)
        T = T @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.array(
            [[1.0, 0.0, l], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    return T


def planar_jacobian(lengths: Sequence[float], q: Sequence[float], with_orientation: bool = False) -> Array:
    """Analytic Jacobian of a planar arm.

    Rows: dx, dy (and dphi if ``with_orientation``). Column j: the effect of joint j, which moves
    every point beyond it on a circle: d(tip)/dq_j = z x (tip - joint_j) = (-(y - y_j), x - x_j).
    """
    pts = planar_joint_points(lengths, q)
    tip = pts[-1]
    rows = 3 if with_orientation else 2
    J = np.zeros((rows, len(q)))
    for j in range(len(q)):
        r = tip - pts[j]
        J[0, j] = -r[1]
        J[1, j] = r[0]
        if with_orientation:
            J[2, j] = 1.0
    return J


@dataclass(frozen=True)
class TwoLinkSolution:
    q1: float
    q2: float
    elbow: str  # "up" or "down"


def two_link_ik(l1: float, l2: float, x: float, y: float, tol: float = 1e-9) -> list[TwoLinkSolution]:
    """All analytic IK solutions of a planar 2-link arm reaching (x, y).

    Returns [] if the target is unreachable (farther than l1 + l2 or closer than |l1 - l2|),
    one solution at the boundary (arm fully stretched or folded), two otherwise.

    Elbow "up" means the elbow is on the left of the line base -> target (for a target on +x in a
    vertical x-z plane: above it). That is the q2 < 0 branch; "down" is q2 > 0.
    """
    r2 = x * x + y * y
    c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if c2 > 1.0 + tol or c2 < -1.0 - tol:
        return []
    c2 = max(-1.0, min(1.0, c2))
    s2_abs = math.sqrt(max(0.0, 1.0 - c2 * c2))
    solutions: list[TwoLinkSolution] = []
    for s2, elbow in ((-s2_abs, "up"), (s2_abs, "down")):
        q2 = math.atan2(s2, c2)
        q1 = math.atan2(y, x) - math.atan2(l2 * s2, l1 + l2 * c2)
        sol = TwoLinkSolution(wrap_angle(q1), q2, elbow)
        if solutions and abs(s2_abs) < 1e-12:
            break  # boundary: both branches coincide
        solutions.append(sol)
    return solutions


# ============================================================================================
# Jacobian tools shared by planar and 3D arms
# ============================================================================================
def numeric_jacobian(f: Callable[[Array], Array], q: ArrayLike, eps: float = 1e-6) -> Array:
    """Central-difference Jacobian of f: R^n -> R^m at q (column j = df/dq_j)."""
    q = np.asarray(q, dtype=float)
    f0 = np.atleast_1d(np.asarray(f(q), dtype=float))
    J = np.zeros((f0.size, q.size))
    for j in range(q.size):
        dq = np.zeros_like(q)
        dq[j] = eps
        J[:, j] = (np.atleast_1d(f(q + dq)) - np.atleast_1d(f(q - dq))) / (2.0 * eps)
    return J


def manipulability(J: ArrayLike) -> float:
    """Yoshikawa's measure sqrt(det(J J^T)); 0 at a singularity. For a square J it is |det J|."""
    J = np.asarray(J, dtype=float)
    return float(math.sqrt(max(0.0, np.linalg.det(J @ J.T))))


def dls_solve(J: ArrayLike, e: ArrayLike, damping: float) -> Array:
    """Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e.

    damping = 0 is the pseudo-inverse solution (explodes near singularities);
    damping > 0 trades a little accuracy for bounded joint motion.
    """
    J = np.asarray(J, dtype=float)
    e = np.asarray(e, dtype=float)
    m = J.shape[0]
    return J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(m), e)


def velocity_ellipse(J: ArrayLike) -> tuple[Array, Array]:
    """Axes of the manipulability ellipse {J qd : |qd| <= 1} for a 2xn position Jacobian.

    Returns (semi_axis_lengths, directions as columns) from the SVD J = U S V^T.
    """
    U, S, _ = np.linalg.svd(np.asarray(J, dtype=float))
    return S, U


# ============================================================================================
# Serial chains (URDF semantics)
# ============================================================================================
@dataclass(frozen=True)
class Joint:
    name: str
    type: str                       # "revolute" or "fixed"
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    lower: float = -math.pi
    upper: float = math.pi

    @property
    def origin(self) -> Array:
        """T_parent_child at q = 0."""
        return se3_from_xyz_rpy(self.xyz, self.rpy)

    def transform(self, q: float = 0.0) -> Array:
        """T_parent_child(q) = origin @ Rot(axis, q)."""
        if self.type == "fixed":
            return self.origin
        if self.type != "revolute":
            raise ValueError(f"joint type {self.type!r} not supported")
        return self.origin @ se3(axis_angle_to_matrix(self.axis, q))


@dataclass(frozen=True)
class SerialChain:
    """base -> tool chain of revolute and fixed joints, listed parent to child."""

    joints: tuple[Joint, ...]
    name: str = "chain"
    _actuated: tuple[int, ...] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        for a, b in zip(self.joints, self.joints[1:]):
            if a.child != b.parent:
                raise ValueError(f"broken chain: {a.name} ends at {a.child}, {b.name} starts at {b.parent}")
        object.__setattr__(self, "_actuated", tuple(i for i, j in enumerate(self.joints) if j.type == "revolute"))

    @property
    def base(self) -> str:
        return self.joints[0].parent

    @property
    def tool(self) -> str:
        return self.joints[-1].child

    @property
    def n_dof(self) -> int:
        return len(self._actuated)

    @property
    def joint_names(self) -> list[str]:
        return [self.joints[i].name for i in self._actuated]

    @property
    def lower(self) -> Array:
        return np.array([self.joints[i].lower for i in self._actuated])

    @property
    def upper(self) -> Array:
        return np.array([self.joints[i].upper for i in self._actuated])

    def within_limits(self, q: ArrayLike, tol: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    def clip(self, q: ArrayLike) -> Array:
        return np.clip(np.asarray(q, dtype=float), self.lower, self.upper)

    def _check(self, q: ArrayLike) -> Array:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.n_dof:
            raise ValueError(f"{self.name} has {self.n_dof} joints, got {q.size} values")
        return q

    def frames(self, q: ArrayLike) -> list[tuple[str, Array]]:
        """[(child link name, T_base_child)] for every joint, in chain order.

        For a revolute joint the returned frame is the child frame, whose z axis (after the
        joint's own axis) is the joint axis, and whose origin lies on the axis.
        """
        q = self._check(q)
        T = np.eye(4)
        out = []
        k = 0
        for j in self.joints:
            if j.type == "revolute":
                T = T @ j.transform(q[k])
                k += 1
            else:
                T = T @ j.transform()
            out.append((j.child, T.copy()))
        return out

    def fk(self, q: ArrayLike) -> Array:
        """T_base_tool(q)."""
        return self.frames(q)[-1][1]

    def geometric_jacobian(self, q: ArrayLike) -> Array:
        """6 x n Jacobian [v; w] of the tool point, expressed in the base frame.

        For revolute joint i with unit axis z_i (in base) through point p_i:
            linear column  = z_i x (p_tool - p_i)
            angular column = z_i
        """
        q = self._check(q)
        frames = self.frames(q)
        p_tool = frames[-1][1][:3, 3]
        J = np.zeros((6, self.n_dof))
        col = 0
        for (_, T), joint in zip(frames, self.joints):
            if joint.type != "revolute":
                continue
            z = T[:3, :3] @ (np.asarray(joint.axis, dtype=float) / np.linalg.norm(joint.axis))
            p = T[:3, 3]
            J[:3, col] = np.cross(z, p_tool - p)
            J[3:, col] = z
            col += 1
        return J


def load_chain_yaml(path: Path | str = SO101_YAML) -> SerialChain:
    """Build the base->tool chain from a kinematics YAML file (see data/so101_kinematics.yaml)."""
    import yaml  # pyyaml, in labs/requirements.txt

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    joints = tuple(
        Joint(
            name=j["name"], type=j["type"], parent=j["parent"], child=j["child"],
            xyz=tuple(float(v) for v in j["xyz"]), rpy=tuple(float(v) for v in j["rpy"]),
            axis=tuple(float(v) for v in j["axis"]) if j["type"] != "fixed" else (0.0, 0.0, 1.0),
            lower=float(j["lower"]), upper=float(j["upper"]),
        )
        for j in data["joints"]
    )
    chain = SerialChain(joints, name=data.get("model", "chain"))
    if chain.base != data["base_frame"] or chain.tool != data["tool_frame"]:
        raise ValueError("base_frame/tool_frame in the YAML do not match the joint list")
    return chain


def load_so101() -> SerialChain:
    """The SO-101 follower arm: 5 revolute joints, base_link -> gripper_frame_link."""
    return load_chain_yaml(SO101_YAML)


def so101_link_lengths(chain: SerialChain | None = None) -> dict[str, float]:
    """Distances between consecutive joint axes at q = 0 (for intuition and planar models)."""
    chain = chain or load_so101()
    frames = dict(chain.frames(np.zeros(chain.n_dof)))
    p = {name: T[:3, 3] for name, T in frames.items()}
    return {
        "shoulder_lift_to_elbow": float(np.linalg.norm(p["lower_arm_link"] - p["upper_arm_link"])),
        "elbow_to_wrist_flex": float(np.linalg.norm(p["wrist_link"] - p["lower_arm_link"])),
        "wrist_flex_to_tool": float(np.linalg.norm(p["gripper_frame_link"] - p["wrist_link"])),
    }


# ============================================================================================
# SO-101 task space and numerical IK
# ============================================================================================
def approach_pitch(T_base_tool: ArrayLike) -> float:
    """Angle of the tool's approach axis (its z axis) below the horizontal, in rad.

    0 = pointing horizontally, +pi/2 = pointing straight down at the table.
    """
    a = np.asarray(T_base_tool, dtype=float)[:3, 2]
    return math.atan2(-a[2], math.hypot(a[0], a[1]))


def so101_task(chain: SerialChain, q: ArrayLike, with_pitch: bool = True) -> Array:
    """Task vector of the SO-101: tool position (x, y, z) [m], optionally approach pitch [rad]."""
    T = chain.fk(q)
    if with_pitch:
        return np.array([T[0, 3], T[1, 3], T[2, 3], approach_pitch(T)])
    return T[:3, 3].copy()


@dataclass(frozen=True)
class IKResult:
    q: Array
    converged: bool
    iterations: int
    position_error_m: float
    pitch_error_rad: float
    reason: str


def ik_dls(
    chain: SerialChain,
    target_xyz: ArrayLike,
    q0: ArrayLike,
    target_pitch: float | None = None,
    pitch_weight: float = 0.1,
    damping: float = 0.01,
    max_iterations: int = 200,
    position_tol_m: float = 1e-4,
    pitch_tol_rad: float = math.radians(0.5),
    max_step_rad: float = math.radians(10.0),
) -> IKResult:
    """Numerical IK by damped least squares with Levenberg–Marquardt step acceptance.

    Minimizes |p(q) - p*|^2 (+ (w * (pitch(q) - pitch*))^2) subject to the joint limits
    (enforced by clipping every step). ``pitch_weight`` converts radians into "meters that
    matter": 0.1 means 1 rad of pitch error costs as much as 10 cm of position error.

    Each iteration:  dq = J^T (J J^T + lambda^2 I)^-1 e,  clipped to max_step_rad.
    If the step reduces the error it is accepted and lambda halves; otherwise it is rejected
    and lambda grows 4x. Always look at ``converged`` before sending ``q`` to a real arm.
    """
    target = np.asarray(target_xyz, dtype=float)
    use_pitch = target_pitch is not None
    weights = np.array([1.0, 1.0, 1.0] + ([pitch_weight] if use_pitch else []))

    def residual(q: Array) -> Array:
        task = so101_task(chain, q, with_pitch=use_pitch)
        e = np.empty_like(task)
        e[:3] = target - task[:3]
        if use_pitch:
            e[3] = wrap_angle(float(target_pitch) - task[3])
        return e * weights

    def task_fn(q: Array) -> Array:
        return so101_task(chain, q, with_pitch=use_pitch) * weights

    q = chain.clip(q0)
    e = residual(q)
    cost = float(e @ e)
    lam = damping
    reason = "max_iterations"
    it = 0
    for it in range(1, max_iterations + 1):
        pos_err = float(np.linalg.norm(e[:3]))
        pitch_err = abs(e[3] / pitch_weight) if use_pitch else 0.0
        if pos_err < position_tol_m and pitch_err < pitch_tol_rad:
            reason = "converged"
            it -= 1
            break
        J = numeric_jacobian(task_fn, q)
        dq = dls_solve(J, e, lam)
        biggest = float(np.max(np.abs(dq)))
        if biggest > max_step_rad:
            dq *= max_step_rad / biggest
        q_new = chain.clip(q + dq)
        e_new = residual(q_new)
        cost_new = float(e_new @ e_new)
        if cost_new < cost:
            q, e, cost = q_new, e_new, cost_new
            lam = max(lam * 0.5, 1e-6)
        else:
            lam *= 4.0
            if lam > 1e3:
                reason = "stalled (target unreachable or blocked by joint limits)"
                break
    pos_err = float(np.linalg.norm(e[:3]))
    pitch_err = abs(float(e[3]) / pitch_weight) if use_pitch else 0.0
    converged = pos_err < position_tol_m and pitch_err < pitch_tol_rad
    return IKResult(q=q, converged=converged, iterations=it, position_error_m=pos_err,
                    pitch_error_rad=pitch_err, reason="converged" if converged else reason)


def reach_limits_so101(chain: SerialChain | None = None, samples: int = 20000, seed: int = 0) -> Array:
    """Tool positions for random joint vectors inside the limits: a point cloud of the workspace."""
    chain = chain or load_so101()
    rng = np.random.default_rng(seed)
    qs = rng.uniform(chain.lower, chain.upper, size=(samples, chain.n_dof))
    return np.array([chain.fk(q)[:3, 3] for q in qs])


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    arm = load_so101()
    print("SO-101 joints:", arm.joint_names)
    print("link lengths [m]:", {k: round(v, 4) for k, v in so101_link_lengths(arm).items()})
    T0 = arm.fk(np.zeros(arm.n_dof))
    print("T_base_tool at q = 0:\n", T0)
    print("approach pitch at q = 0: %.2f deg" % math.degrees(approach_pitch(T0)))
    q = np.radians([20, -30, 40, 60, 0])
    T = arm.fk(q)
    print("tool at q = (20, -30, 40, 60, 0) deg:", T[:3, 3], "pitch %.1f deg" % math.degrees(approach_pitch(T)))
    res = ik_dls(arm, T[:3, 3], np.zeros(5), target_pitch=approach_pitch(T))
    print("IK back from q = 0:", np.degrees(res.q).round(2), res.reason, "iterations", res.iterations,
          "pos err %.2e m" % res.position_error_m)
