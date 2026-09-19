"""A tiny analytic RGB-D renderer for modules 15.04-15.05 — boxes, cylinders and spheres on a table.

Why not reuse a simulator? Because the point of 15.04 is that a depth camera sees **one side of
one surface**, and nothing teaches that faster than a renderer you can read in one sitting. Every
primitive is a closed-form ray intersection; there is no lighting, no texture and no mesh.

World frame: the table top is the plane z = 0, x forward, y left, z up (REP-103). Poses are the
4x4 matrices of 05-frames-and-transforms.

    py tabletop_scene.py         renders the default scene and prints what the camera sees
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# ============================================================================== camera
@dataclass(frozen=True)
class Camera:
    """Pinhole intrinsics, the optical frame of REP-104: x right, y down, z forward."""

    fx: float = 525.0
    fy: float = 525.0
    cx: float = 319.5
    cy: float = 239.5
    width: int = 640
    height: int = 480

    @property
    def K(self) -> Array:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def rays(self) -> Array:
        """(H, W, 3) unit-z direction vectors in the optical frame."""
        u, v = np.meshgrid(np.arange(self.width, dtype=float), np.arange(self.height, dtype=float))
        return np.stack([(u - self.cx) / self.fx, (v - self.cy) / self.fy, np.ones_like(u)], axis=-1)


def look_at(eye: ArrayLike, target: ArrayLike, up: ArrayLike = (0.0, 0.0, 1.0)) -> Array:
    """T_world_optical for a camera at ``eye`` looking at ``target`` (optical: x right, y down, z fwd)."""
    eye = np.asarray(eye, dtype=float)
    z = np.asarray(target, dtype=float) - eye
    z = z / np.linalg.norm(z)
    up = np.asarray(up, dtype=float)
    if abs(float(z @ up)) > 0.999:          # looking straight along 'up': pick another reference
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(z, up)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack((x, y, z))
    T[:3, 3] = eye
    return T


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def transform_points(T: ArrayLike, pts: ArrayLike) -> Array:
    """Apply a 4x4 to an (N, 3) array of points."""
    T = np.asarray(T, dtype=float)
    pts = np.asarray(pts, dtype=float)
    return pts @ T[:3, :3].T + T[:3, 3]


def rot_z(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ============================================================================== primitives
@dataclass(frozen=True)
class Box:
    """Axis-aligned box of size (sx, sy, sz) standing on the table, then yawed about its centre."""

    name: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0

    def intersect(self, o: Array, d: Array) -> Array:
        """Ray-box distance (slab method) in world coordinates; inf where the ray misses."""
        R = rot_z(self.yaw)
        c = np.asarray(self.centre, dtype=float)
        ol = (o - c) @ R                     # into the box frame
        dl = d @ R
        h = np.asarray(self.size, dtype=float) / 2.0
        dl = np.where(np.abs(dl) < 1e-12, 1e-12, dl)     # a ray parallel to a slab never enters it
        t1 = (-h - ol) / dl
        t2 = (h - ol) / dl
        tmin = np.max(np.minimum(t1, t2), axis=-1)
        tmax = np.min(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= np.maximum(tmin, 0.0))
        return np.where(hit, np.where(tmin > 0, tmin, tmax), np.inf)

    @property
    def top_z(self) -> float:
        return self.centre[2] + self.size[2] / 2.0


@dataclass(frozen=True)
class Cylinder:
    """Upright cylinder (a can, a mug body) of radius r and height h, base on the table."""

    name: str
    centre_xy: tuple[float, float]
    radius: float
    height: float

    def intersect(self, o: Array, d: Array) -> Array:
        cx, cy = self.centre_xy
        ox, oy = o[..., 0] - cx, o[..., 1] - cy
        dx, dy = d[..., 0], d[..., 1]
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - self.radius ** 2
        disc = b * b - 4 * a * c
        out = np.full(o.shape[:-1] if o.ndim > 1 else d.shape[:-1], np.inf)
        ok = (disc > 0) & (a > 1e-12)
        sq = np.sqrt(np.where(ok, disc, 0.0))
        for t in ((-b - sq) / np.where(a > 1e-12, 2 * a, 1.0), (-b + sq) / np.where(a > 1e-12, 2 * a, 1.0)):
            z = o[..., 2] + t * d[..., 2]
            good = ok & (t > 0) & (z >= 0.0) & (z <= self.height) & (t < out)
            out = np.where(good, t, out)
        # the flat top disc
        with np.errstate(divide="ignore", invalid="ignore"):
            t_top = (self.height - o[..., 2]) / d[..., 2]
        px, py = o[..., 0] + t_top * dx - cx, o[..., 1] + t_top * dy - cy
        good = (t_top > 0) & (px * px + py * py <= self.radius ** 2) & (t_top < out)
        return np.where(good, t_top, out)

    @property
    def top_z(self) -> float:
        return self.height


@dataclass(frozen=True)
class Sphere:
    name: str
    centre: tuple[float, float, float]
    radius: float

    def intersect(self, o: Array, d: Array) -> Array:
        oc = o - np.asarray(self.centre, dtype=float)
        b = 2.0 * np.sum(oc * d, axis=-1)
        c = np.sum(oc * oc, axis=-1) - self.radius ** 2
        a = np.sum(d * d, axis=-1)
        disc = b * b - 4 * a * c
        sq = np.sqrt(np.maximum(disc, 0.0))
        t = (-b - sq) / (2 * a)
        return np.where((disc > 0) & (t > 0), t, np.inf)

    @property
    def top_z(self) -> float:
        return self.centre[2] + self.radius


Primitive = Box | Cylinder | Sphere


@dataclass
class Scene:
    """A table of half-size ``table_half`` at z = 0 plus a list of objects standing on it."""

    objects: list[Primitive] = field(default_factory=list)
    table_half: tuple[float, float] = (0.35, 0.35)

    def labelled_depth(self, cam: Camera, T_world_optical: Array) -> tuple[Array, Array]:
        """(depth_m, label) images. depth = Z along the optical axis, 0 where nothing is hit.

        ``label`` is -1 for nothing, 0 for the table and 1..n for ``objects[i - 1]`` — ground truth
        the pipeline is never allowed to look at, only to be scored against.
        """
        rays = cam.rays()
        R, eye = T_world_optical[:3, :3], T_world_optical[:3, 3]
        d = rays @ R.T                                   # world directions, optical-z component = 1
        o = np.broadcast_to(eye, d.shape)
        best = np.full(d.shape[:2], np.inf)
        label = np.full(d.shape[:2], -1, dtype=np.int32)

        with np.errstate(divide="ignore", invalid="ignore"):
            t_table = -o[..., 2] / d[..., 2]
        p = o + t_table[..., None] * d
        on_table = ((t_table > 0) & (np.abs(p[..., 0]) <= self.table_half[0])
                    & (np.abs(p[..., 1]) <= self.table_half[1]))
        best = np.where(on_table, t_table, best)
        label = np.where(on_table, 0, label)

        for i, obj in enumerate(self.objects, start=1):
            t = obj.intersect(o, d)
            closer = t < best
            best = np.where(closer, t, best)
            label = np.where(closer, i, label)

        depth = np.where(np.isfinite(best), best, 0.0)   # t is already Z: |optical-z of d| = 1
        return depth.astype(np.float32), label

    def depth(self, cam: Camera, T_world_optical: Array) -> Array:
        return self.labelled_depth(cam, T_world_optical)[0]


def noisy_depth(depth_m: Array, rng: np.random.Generator, k: float = 0.002,
                dropout: float = 0.01, quant_m: float = 0.001) -> Array:
    """Depth-camera noise: sigma_Z = k Z^2 (stereo and structured light both look like this),
    random dropouts, and millimetre quantisation as 16UC1 streams have."""
    z = depth_m.astype(np.float64)
    out = z + rng.normal(0.0, 1.0, z.shape) * k * z * z
    if quant_m > 0:
        out = np.round(out / quant_m) * quant_m
    out[rng.random(z.shape) < dropout] = 0.0
    out[z <= 0] = 0.0
    return out.astype(np.float32)


def depth_to_points(depth_m: Array, cam: Camera, stride: int = 1) -> tuple[Array, Array]:
    """((N, 3) points in the optical frame, (N, 2) their (v, u) pixels). 0 depth is dropped."""
    d = depth_m[::stride, ::stride]
    v, u = np.nonzero(d > 0)
    z = d[v, u].astype(np.float64)
    uu, vv = u * stride, v * stride
    x = (uu - cam.cx) / cam.fx * z
    y = (vv - cam.cy) / cam.fy * z
    return np.column_stack((x, y, z)), np.column_stack((vv, uu))


# ============================================================================== the demo scene
def demo_scene() -> Scene:
    """Four objects on the table in front of the arm, sized for the SO-101's 45 mm jaw."""
    return Scene(objects=[
        Box("wooden block", (0.24, 0.06, 0.020), (0.060, 0.030, 0.040), yaw=math.radians(25)),
        Cylinder("drinks can", (0.30, -0.10), radius=0.033, height=0.115),
        Box("phone", (0.14, -0.14, 0.005), (0.150, 0.072, 0.010), yaw=math.radians(-10)),
        Sphere("golf ball", (0.33, 0.15, 0.0215), radius=0.0215),
        Box("eraser", (0.19, 0.16, 0.012), (0.045, 0.024, 0.024), yaw=math.radians(-40)),
    ])


def cluttered_scene(gap_m: float = 0.012) -> Scene:
    """Two identical blocks ``gap_m`` apart — the scene that decides your clustering voxel size."""
    return Scene(objects=[
        Box("block A", (0.24, +(0.015 + gap_m / 2), 0.020), (0.060, 0.030, 0.040)),
        Box("block B", (0.24, -(0.015 + gap_m / 2), 0.020), (0.060, 0.030, 0.040)),
    ])


def camera_above(height: float = 0.35, tilt_deg: float = 60.0,
                 target: tuple[float, float, float] = (0.25, 0.0, 0.03)) -> Array:
    """A wrist camera at ``tilt_deg`` above the horizon looking at ``target``. 90 deg = straight down."""
    a = math.radians(tilt_deg)
    back = height / math.tan(a) if a < math.pi / 2 - 1e-6 else 0.0
    eye = (target[0] - back, target[1], target[2] + height)
    return look_at(eye, target)


if __name__ == "__main__":
    cam, scene = Camera(), demo_scene()
    for tilt in (45.0, 60.0, 90.0):
        depth, label = scene.labelled_depth(cam, camera_above(tilt_deg=tilt))
        counts = [int((label == i).sum()) for i in range(len(scene.objects) + 1)]
        print(f"tilt {tilt:4.0f} deg: depth {depth[depth > 0].min():.3f}-{depth.max():.3f} m, "
              f"table {counts[0]:6d} px, objects {counts[1:]}")
