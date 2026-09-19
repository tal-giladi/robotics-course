"""Synthetic images with known ground truth, so every lesson in 13.01-13.08 runs without a camera.

Every renderer ray-casts through a pinhole camera (camera_model.py), so the images obey exactly
the geometry the lessons teach: a ball 1.0 m away really is f*R/Z pixels in radius, a
checkerboard really is bent by the distortion coefficients you pass, a tag really sits at the
pose you give it.

  ball_scene()          orange ball on a tiled floor, lighting gradient, noise, distractors
  checkerboard_image()  calibration board seen through K + distortion at a given pose
  random_board_poses()  a varied set of board poses for calibration
  marker_scene()        ArUco/AprilTag on a wall at a given pose
  tabletop()            depth image + grayscale image of a table with a box and a ball
  stereo_pair()         rectified left/right images of the same tabletop

All randomness comes from an explicit numpy Generator.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import cv2
import numpy as np

from camera_model import (PinholeCamera, inv_T, karmel_camera, karmel_T_base_optical,
                          make_T, rot_x, rot_y, rot_z)

BALL_RADIUS_M = 0.035                 # a 7 cm orange foam ball
BALL_BGR = (20, 110, 235)             # orange, in OpenCV's BGR order
SUPERSAMPLE = 2


# ----------------------------------------------------------------------------- utilities
def distort_normalized(x: np.ndarray, y: np.ndarray, dist) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV / ROS 'plumb_bob' model: dist = (k1, k2, p1, p2[, k3])."""
    k1, k2, p1, p2 = dist[:4]
    k3 = dist[4] if len(dist) > 4 else 0.0
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return xd, yd


def undistort_normalized(xd: np.ndarray, yd: np.ndarray, dist,
                         iterations: int = 12) -> tuple[np.ndarray, np.ndarray]:
    """Invert distort_normalized by fixed-point iteration (what cv2.undistortPoints does inside)."""
    k1, k2, p1, p2 = dist[:4]
    k3 = dist[4] if len(dist) > 4 else 0.0
    x, y = xd.copy(), yd.copy()
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return x, y


def _pixel_grid(width: int, height: int) -> np.ndarray:
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    return np.column_stack((u.ravel(), v.ravel()))


@lru_cache(maxsize=16)
def _normalized_rays(fx: float, fy: float, cx: float, cy: float, width: int, height: int,
                     dist: tuple[float, ...] | None, s: int) -> np.ndarray:
    """(H*s, W*s, 3) ray directions (z = 1) in camera_optical for a supersampled pixel grid.

    With distortion, each output pixel is *undistorted* to find which ideal ray it sees; that is
    how a real lens maps the world onto the sensor.
    """
    hi_w, hi_h = width * s, height * s
    if dist is None or not any(dist):
        grid = _pixel_grid(hi_w, hi_h)
        lo = (grid + 0.5) / s - 0.5                           # hi-res pixel center in low-res pixels
        x = ((lo[:, 0] - cx) / fx).reshape(hi_h, hi_w)
        y = ((lo[:, 1] - cy) / fy).reshape(hi_h, hi_w)
    else:
        # undistort at native resolution (slow part), then upsample the smooth ray field;
        # cv2.resize uses the same pixel-center convention as above
        grid = _pixel_grid(width, height)
        x, y = undistort_normalized((grid[:, 0] - cx) / fx, (grid[:, 1] - cy) / fy, dist)
        x = cv2.resize(x.reshape(height, width), (hi_w, hi_h), interpolation=cv2.INTER_LINEAR)
        y = cv2.resize(y.reshape(height, width), (hi_w, hi_h), interpolation=cv2.INTER_LINEAR)
    rays = np.stack((x, y, np.ones_like(x)), axis=-1)
    rays.setflags(write=False)
    return rays


def _downsample(img_hi: np.ndarray, width: int, height: int) -> np.ndarray:
    return cv2.resize(img_hi, (width, height), interpolation=cv2.INTER_AREA)


def _add_noise(img: np.ndarray, sigma: float, rng: np.random.Generator) -> np.ndarray:
    noisy = img.astype(np.float32) + rng.normal(0.0, sigma, img.shape).astype(np.float32)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def _ray_plane(rays: np.ndarray, T_cam_plane: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Intersect camera rays with the z=0 plane of a frame. Returns (a, b, t) plane coords and ray
    parameter t (= depth Z because rays have z = 1). t <= 0 means no hit."""
    R, p = T_cam_plane[:3, :3], T_cam_plane[:3, 3]
    d_plane = rays @ R                                       # R^T d for every ray
    o_plane = -R.T @ p                                        # camera origin in plane frame
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -o_plane[2] / d_plane[..., 2]
    t = np.where(np.isfinite(t), t, -1.0)
    a = o_plane[0] + t * d_plane[..., 0]
    b = o_plane[1] + t * d_plane[..., 1]
    return a, b, t


# ----------------------------------------------------------------------------- 13.01-13.03 ball
@dataclass(frozen=True)
class BallTruth:
    u: float              # ball center in pixels (projection of the sphere center)
    v: float
    radius_px: float      # apparent radius, f * R / Z
    distance_m: float     # camera center -> ball center
    ball_base: tuple[float, float, float]   # ball center in base_footprint


def ball_scene(rng: np.random.Generator, ball_xy=(1.0, 0.1), light=(0.55, 1.15),
               noise_sigma: float = 6.0, distractors: bool = True,
               cam: PinholeCamera | None = None, T_base_optical: np.ndarray | None = None,
               ball_bgr=BALL_BGR, supersample: int = 1) -> tuple[np.ndarray, BallTruth]:
    """640x480 BGR image of an orange ball on a tiled floor seen by karmel's camera.

    ball_xy: ball position on the floor in base_footprint (x forward, y left), meters.
    light:   (left, right) brightness multipliers -> a horizontal lighting gradient (a window).
    distractors: a brown cardboard box (similar hue, low saturation) and a small orange cap.
    """
    cam = cam or karmel_camera()
    T_bo = karmel_T_base_optical() if T_base_optical is None else T_base_optical
    s = supersample
    rays = _normalized_rays(cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, None, s)
    H, W = rays.shape[:2]
    R, c = T_bo[:3, :3], T_bo[:3, 3]
    d = (rays @ R.T).astype(np.float32)                       # ray directions in base frame
    img = np.zeros((H, W, 3), dtype=np.float32)
    depth = np.full((H, W), np.inf, dtype=np.float32)

    # wall 3 m ahead (light grey) as the far background
    img[:] = (185, 190, 195)
    # floor z = 0 with 30 cm tiles and dark grout
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = -c[2] / d[..., 2]
        fx_ = c[0] + t_floor * d[..., 0]
        fy_ = c[1] + t_floor * d[..., 1]
        floor = (t_floor > 0) & (fx_ < 3.0)
        tile = 0.30
        gu = np.abs(((fx_ + 5 * tile) % tile) - tile / 2) > tile / 2 - 0.006
        gv = np.abs(((fy_ + 5 * tile) % tile) - tile / 2) > tile / 2 - 0.006
    floor_col = np.where((gu | gv)[..., None], (95, 100, 105), (160, 172, 180)).astype(np.float32)
    img[floor] = floor_col[floor]
    depth[floor] = t_floor[floor]

    def paint_box(x0, x1, y0, y1, h, bgr):
        # axis-aligned box on the floor, slab method
        o = c
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / d
            ax0, ax1 = (x0 - o[0]) * inv[..., 0], (x1 - o[0]) * inv[..., 0]
            ay0, ay1 = (y0 - o[1]) * inv[..., 1], (y1 - o[1]) * inv[..., 1]
            az0, az1 = (0 - o[2]) * inv[..., 2], (h - o[2]) * inv[..., 2]
            tmin = np.maximum(np.maximum(np.minimum(ax0, ax1), np.minimum(ay0, ay1)), np.minimum(az0, az1))
            tmax = np.minimum(np.minimum(np.maximum(ax0, ax1), np.maximum(ay0, ay1)), np.maximum(az0, az1))
        hit = (tmax >= tmin) & (tmin > 0) & (tmin < depth)
        # face shading: top brighter than sides
        with np.errstate(invalid="ignore"):
            pz = o[2] + tmin * d[..., 2]
        shade = np.where(np.abs(pz - h) < 1e-3, 1.0, 0.8)[..., None]
        img[hit] = (np.array(bgr, dtype=np.float32) * shade)[hit]
        depth[hit] = tmin[hit]

    if distractors:
        paint_box(1.6, 1.9, -0.75, -0.40, 0.25, (70, 105, 150))   # cardboard box: brown
        paint_box(0.70, 0.73, 0.28, 0.31, 0.012, ball_bgr)       # bottle cap: tiny, same orange

    # the ball (sphere) with Lambertian shading
    center = np.array([ball_xy[0], ball_xy[1], BALL_RADIUS_M])
    oc = c - center
    bq = np.einsum("hwk,k->hw", d, oc)
    dd = np.einsum("hwk,hwk->hw", d, d)
    disc = bq * bq - dd * (oc @ oc - BALL_RADIUS_M ** 2)
    with np.errstate(invalid="ignore"):
        t_ball = (-bq - np.sqrt(disc)) / dd
    hit = (disc > 0) & (t_ball > 0) & (t_ball < depth)
    p = c + t_ball[..., None] * d
    n = (p - center) / BALL_RADIUS_M
    light_dir = np.array([-0.3, 0.4, 0.87])
    light_dir /= np.linalg.norm(light_dir)
    lam = np.clip(n @ light_dir, 0, 1)
    ball_col = np.array(ball_bgr, dtype=np.float32)[None, None, :] * (0.45 + 0.6 * lam[..., None])
    img[hit] = ball_col[hit]
    depth[hit] = t_ball[hit]

    # horizontal lighting gradient (brighter toward the right = window) and slight vignetting
    xs = np.linspace(light[0], light[1], W, dtype=np.float32)[None, :, None]
    yy, xx = np.mgrid[0:H, 0:W]
    r2 = ((xx - W / 2) ** 2 + (yy - H / 2) ** 2) / ((W / 2) ** 2 + (H / 2) ** 2)
    img *= xs * (1.0 - 0.25 * r2[..., None]).astype(np.float32)
    img = np.clip(img, 0, 255).astype(np.uint8)
    out = _add_noise(_downsample(img, cam.width, cam.height), noise_sigma, rng)

    center_opt = inv_T(T_bo)[:3, :3] @ center + inv_T(T_bo)[:3, 3]
    (uv,), _ = cam.project(center_opt[None, :])
    dist_m = float(np.linalg.norm(center_opt))
    truth = BallTruth(float(uv[0]), float(uv[1]), float(cam.fx * BALL_RADIUS_M / center_opt[2]),
                      dist_m, (float(center[0]), float(center[1]), float(center[2])))
    return out, truth


# ----------------------------------------------------------------------------- 13.06 calibration
def checkerboard_texture(inner_cols: int = 9, inner_rows: int = 6, px_per_square: int = 60,
                         margin_squares: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Grayscale board image with a white margin, and the 2x3 affine that maps board coordinates
    in *squares* (x along columns, y along rows, origin at the first inner corner) to texture
    pixels. Divide its first two columns by the square size to map meters instead."""
    sq_c, sq_r = inner_cols + 1, inner_rows + 1
    m = int(round(margin_squares * px_per_square))
    tex = np.full((sq_r * px_per_square + 2 * m, sq_c * px_per_square + 2 * m), 255, np.uint8)
    for r in range(sq_r):
        for col in range(sq_c):
            if (r + col) % 2 == 0:
                y0, x0 = m + r * px_per_square, m + col * px_per_square
                tex[y0:y0 + px_per_square, x0:x0 + px_per_square] = 0
    # first inner corner sits one square in from the board's top-left black square
    A = np.array([[px_per_square, 0, m + px_per_square - 0.5],
                  [0, px_per_square, m + px_per_square - 0.5]], dtype=np.float64)
    return tex, A


def checkerboard_object_points(inner_cols: int = 9, inner_rows: int = 6,
                               square_m: float = 0.025) -> np.ndarray:
    """(N,3) inner corner coordinates in the board frame, in OpenCV's row-major order."""
    xs, ys = np.meshgrid(np.arange(inner_cols), np.arange(inner_rows))
    return np.column_stack((xs.ravel(), ys.ravel(), np.zeros(xs.size))).astype(np.float32) * square_m


def render_plane(cam: PinholeCamera, dist, T_cam_plane: np.ndarray, texture: np.ndarray,
                 A_plane_to_tex: np.ndarray, background: np.ndarray | float,
                 noise_sigma: float, rng: np.random.Generator) -> np.ndarray:
    """Render a textured plane (texture mapped by the affine A: (x, y) meters -> (col, row))."""
    s = SUPERSAMPLE
    key = None if dist is None else tuple(float(k) for k in np.ravel(dist))
    rays = _normalized_rays(cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, key, s)
    a, b, t = _ray_plane(rays, T_cam_plane)
    map_c = (A_plane_to_tex[0, 0] * a + A_plane_to_tex[0, 1] * b + A_plane_to_tex[0, 2]).astype(np.float32)
    map_r = (A_plane_to_tex[1, 0] * a + A_plane_to_tex[1, 1] * b + A_plane_to_tex[1, 2]).astype(np.float32)
    map_c[t <= 0] = -1e6
    tex_f = texture.astype(np.float32)
    sampled = cv2.remap(tex_f, map_c, map_r, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
    mask = cv2.remap(np.ones(texture.shape[:2], np.float32), map_c, map_r, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    if texture.ndim == 3:
        mask = mask[..., None]
    if np.isscalar(background):
        bg = np.full_like(sampled, float(background))
    else:
        bg = cv2.resize(np.asarray(background, np.float32), (sampled.shape[1], sampled.shape[0]),
                        interpolation=cv2.INTER_LINEAR)
    img = mask * np.maximum(sampled, 0) + (1 - mask) * bg
    img = _downsample(img, cam.width, cam.height)
    return _add_noise(np.clip(img, 0, 255), noise_sigma, rng)


def random_board_poses(cam: PinholeCamera, rng: np.random.Generator, n: int,
                       inner_cols: int = 9, inner_rows: int = 6, square_m: float = 0.025,
                       margin_px: float = 8.0) -> list[np.ndarray]:
    """n varied T_cam_board poses: tilted up to ~40 deg, 0.3-0.6 m away, spread over the image
    (including the corners, where distortion is strongest), every corner inside the frame."""
    obj = checkerboard_object_points(inner_cols, inner_rows, square_m).astype(np.float64)
    center = obj.mean(axis=0)
    ext = np.array([[0, 0, 0], [inner_cols + 1, 0, 0], [0, inner_rows + 1, 0],
                    [inner_cols + 1, inner_rows + 1, 0]], float) * square_m - square_m
    poses: list[np.ndarray] = []
    while len(poses) < n:
        u = rng.uniform(0.1, 0.9) * cam.width
        v = rng.uniform(0.1, 0.9) * cam.height
        z = rng.uniform(0.30, 0.60)
        R = rot_z(rng.uniform(-0.3, 0.3)) @ rot_x(rng.uniform(-0.7, 0.7)) @ rot_y(rng.uniform(-0.7, 0.7))
        target = np.array([(u - cam.cx) / cam.fx * z, (v - cam.cy) / cam.fy * z, z])
        t = target - R @ center
        T = make_T(R, t)
        pts = ext @ R.T + t
        if np.any(pts[:, 2] < 0.1):
            continue
        uv, _ = cam.project(pts)
        if (uv[:, 0].min() < margin_px or uv[:, 1].min() < margin_px
                or uv[:, 0].max() > cam.width - margin_px or uv[:, 1].max() > cam.height - margin_px):
            continue
        poses.append(T)
    return poses


def checkerboard_image(cam: PinholeCamera, dist, T_cam_board: np.ndarray, rng: np.random.Generator,
                       inner_cols: int = 9, inner_rows: int = 6, square_m: float = 0.025,
                       noise_sigma: float = 3.0) -> np.ndarray:
    tex, A = checkerboard_texture(inner_cols, inner_rows)
    A = A.copy()
    A[:, :2] /= square_m                                      # squares -> meters
    return render_plane(cam, dist, T_cam_board, tex, A, background=90.0,
                        noise_sigma=noise_sigma, rng=rng)


# ----------------------------------------------------------------------------- 13.07 markers
def marker_scene(cam: PinholeCamera, T_cam_marker: np.ndarray, rng: np.random.Generator,
                 marker_id: int = 0, marker_len_m: float = 0.10,
                 dict_id: int = cv2.aruco.DICT_APRILTAG_36h11, dist=None,
                 noise_sigma: float = 3.0, px_per_cell: int = 30) -> np.ndarray:
    """Grayscale image of a printed tag (black square side = marker_len_m) on a white sheet.

    Marker frame as OpenCV's IPPE_SQUARE: origin at the center, x right, y up, z toward viewer.
    """
    from aruco_compat import generate_marker, get_dictionary
    dictionary = get_dictionary(dict_id)
    cells = dictionary.markerSize + 2
    side = cells * px_per_cell
    marker = generate_marker(dictionary, marker_id, side, border_bits=1)
    quiet = 2 * px_per_cell                                   # white margin = 2 cells
    sheet = np.full((side + 2 * quiet, side + 2 * quiet), 245, np.uint8)
    sheet[quiet:quiet + side, quiet:quiet + side] = np.where(marker > 127, 245, 15).astype(np.uint8)
    ppm = side / marker_len_m
    c0 = quiet + side / 2 - 0.5
    A = np.array([[ppm, 0, c0], [0, -ppm, c0]])             # y up in the marker -> rows down
    yy, xx = np.mgrid[0:cam.height, 0:cam.width]
    wall = (120 + 30 * np.sin(xx / 37.0) * np.cos(yy / 53.0) + 0.05 * xx).astype(np.float32)
    return render_plane(cam, dist, T_cam_marker, sheet, A, background=wall,
                        noise_sigma=noise_sigma, rng=rng)


# ----------------------------------------------------------------------------- 13.08 tabletop
@dataclass(frozen=True)
class TabletopWorld:
    """World frame: table surface is z = 0, x/y on the table. Floor at z = -0.75."""

    box_min: tuple[float, float, float] = (0.00, -0.12, 0.0)
    box_max: tuple[float, float, float] = (0.08, -0.06, 0.10)       # 8 x 6 x 10 cm box
    ball_center: tuple[float, float, float] = (0.05, 0.10, BALL_RADIUS_M)
    table_half: tuple[float, float] = (0.40, 0.30)                   # 80 x 60 cm table top
    floor_z: float = -0.75


def tabletop_camera_pose(eye=(-0.45, 0.0, 0.40), target=(0.05, 0.0, 0.0)) -> np.ndarray:
    """T_world_optical for a camera looking down at the table (as a head or arm camera would)."""
    from camera_model import look_at
    return look_at(eye, target)


@lru_cache(maxsize=2)
def _texture(seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    tex = rng.uniform(0, 255, (256, 256)).astype(np.float32)
    tex = cv2.GaussianBlur(np.tile(tex, (3, 3)), (0, 0), 0.8)[256:512, 256:512]   # wrap-around blur
    tex = cv2.normalize(tex, None, 30, 225, cv2.NORM_MINMAX)
    tex.setflags(write=False)
    return tex


def _sample_texture(a: np.ndarray, b: np.ndarray, cell_m: float = 0.004) -> np.ndarray:
    tex = _texture()
    n = tex.shape[0]
    mc = np.mod(a / cell_m, n).astype(np.float32)
    mr = np.mod(b / cell_m, n).astype(np.float32)
    return cv2.remap(tex, mc, mr, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def tabletop(cam: PinholeCamera, T_world_optical: np.ndarray, world: TabletopWorld = TabletopWorld(),
             supersample: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Noise-free (depth_m float32, gray uint8) of the tabletop. depth = Z along the optical axis,
    0 where nothing is hit. Textured surfaces so stereo matching has something to match."""
    s = supersample
    rays = _normalized_rays(cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, None, s)
    R, c = T_world_optical[:3, :3], T_world_optical[:3, 3]
    d = rays @ R.T                                          # world directions, |z_cam| = 1
    H, W = d.shape[:2]
    t_best = np.full((H, W), np.inf)
    gray = np.zeros((H, W), np.float32)
    light = np.array([0.3, -0.2, 0.93])
    light /= np.linalg.norm(light)

    def shade(hit, t, normal, albedo, ta, tb):
        tex = _sample_texture(ta, tb)
        lam = 0.35 + 0.65 * max(float(normal @ light), 0.0)
        gray[hit] = (albedo * lam * tex)[hit]
        t_best[hit] = t[hit]

    with np.errstate(divide="ignore", invalid="ignore"):
        # table top and floor
        for z, half, albedo in ((0.0, world.table_half, 1.0), (world.floor_z, None, 0.6)):
            t = (z - c[2]) / d[..., 2]
            px, py = c[0] + t * d[..., 0], c[1] + t * d[..., 1]
            hit = (t > 0) & (t < t_best)
            if half is not None:
                hit &= (np.abs(px) <= half[0]) & (np.abs(py) <= half[1])
            shade(hit, t, np.array([0, 0, 1.0]), albedo, px, py)
        # box (slab method, then pick the face)
        bmin, bmax = np.array(world.box_min), np.array(world.box_max)
        t1 = (bmin[None, None, :] - c) / d
        t2 = (bmax[None, None, :] - c) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmin > 0) & (tmin < t_best)
        p = c + tmin[..., None] * d
        for axis in range(3):
            for bound, sign in ((bmin, -1.0), (bmax, 1.0)):
                face = hit & (np.abs(p[..., axis] - bound[axis]) < 1e-6 + 1e-9 * tmin)
                nrm = np.zeros(3)
                nrm[axis] = sign
                o = [k for k in range(3) if k != axis]
                shade(face, tmin, nrm, 0.9, p[..., o[0]], p[..., o[1]])
        # ball
        center = np.array(world.ball_center)
        oc = c - center
        bq = d @ oc
        dd = np.einsum("hwk,hwk->hw", d, d)
        disc = bq * bq - dd * (oc @ oc - BALL_RADIUS_M ** 2)
        tb_ = (-bq - np.sqrt(disc)) / dd
        hit = (disc > 0) & (tb_ > 0) & (tb_ < t_best)
        p = c + tb_[..., None] * d
        nrm = (p - center) / BALL_RADIUS_M
        lam = 0.35 + 0.65 * np.clip(nrm @ light, 0, 1)
        tex = _sample_texture(p[..., 0] + p[..., 2], p[..., 1] + p[..., 2])
        gray[hit] = (lam * tex * 1.1)[hit]
        t_best[hit] = tb_[hit]

    depth = np.where(np.isfinite(t_best), t_best, 0.0).astype(np.float32)   # rays have z = 1 -> t = Z
    if s > 1:
        gray = _downsample(gray, cam.width, cam.height)
        depth = depth[s // 2::s, s // 2::s]
    return depth, np.clip(gray, 0, 255).astype(np.uint8)


def noisy_depth(depth_m: np.ndarray, rng: np.random.Generator, k: float = 0.002,
                dropout: float = 0.01) -> np.ndarray:
    """Depth-camera-like noise: sigma_Z = k * Z^2 (stereo/structured light) plus random dropouts (0)."""
    z = depth_m.astype(np.float64)
    noisy = z + rng.normal(0.0, 1.0, z.shape) * k * z * z
    noisy[rng.random(z.shape) < dropout] = 0.0
    noisy[z <= 0] = 0.0
    return noisy.astype(np.float32)


def stereo_pair(cam: PinholeCamera, T_world_left: np.ndarray, baseline_m: float,
                rng: np.random.Generator, world: TabletopWorld = TabletopWorld(),
                noise_sigma: float = 2.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rectified stereo: the right camera is the left one moved baseline_m along its own x axis.
    Returns (left_gray, right_gray, true_depth_left)."""
    T_left_right = make_T(np.eye(3), (baseline_m, 0.0, 0.0))
    _, left = tabletop(cam, T_world_left, world, supersample=2)
    _, right = tabletop(cam, T_world_left @ T_left_right, world, supersample=2)
    depth_l, _ = tabletop(cam, T_world_left, world, supersample=1)
    return _add_noise(left, noise_sigma, rng), _add_noise(right, noise_sigma, rng), depth_l
