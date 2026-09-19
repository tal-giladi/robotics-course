# 13.16 — A box is a ray: finding the missing depth, and how wrong it is

Lesson: [13.16 From detection to a 3D position in the map frame](../../../13-computer-vision/13.16-detection-to-map-position.md)

A detector gives you a box. A box is a direction. This exercise is the geometry that turns it into
a place the robot can drive to — plus the uncertainty that decides whether two detections are one
object or two.

Everything is in `camera_optical_frame` (REP-103): **z forward (depth), x right, y down**.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `scale_intrinsics(intr, w, h)` | K for the same camera after the image is resized |
| `back_project(intr, u, v, z)` | pixel + depth → 3D point |
| `depth_from_depth_image(depth_m, box, ...)` | median of the valid depths in the central part of the box, or `None` |
| `depth_from_known_size(box, intr, H)` | `Z = fy·H / h_px` |
| `depth_from_ground_plane(box, intr, h, pitch)` | ray through the box's **bottom** edge ∩ the floor |
| `depth_from_range_sensor(r, box, intr)` | the forward component of a slant range along the box's bearing |
| `position_covariance(box, intr, Z, σ_Z, σ_px)` | 3×3 covariance by first-order propagation |
| `plausible_size(class, box, intr, Z)` | the 16.01 gate: is the implied real height believable? |
| `mahalanobis2(p1, C1, p2, C2)` | squared Mahalanobis distance, the association gate |
| `fuse(p1, C1, p2, C2, min_sigma)` | Kalman update of a static object, with a covariance floor |

`Intrinsics`, `Box`, `OBJECT_HEIGHT_M`, `GATE_95` and `GATE_99` are given.

## Check

```bash
python course.py check 13.16              # your code
python course.py check 13.16 --solution   # the reference, to see what passing looks like
```

The tests use karmel's real camera from `labs/config/karmel.yaml` (640×480, hfov 1.20 rad → fx = 467.7;
320×240 → fx = 233.9) with the camera 0.145 m above the floor.

## Hints

- **The consistency test is the one to aim at.** On a perfect box around an object of known height
  standing on the floor, all four depth methods must return the *same* number to 1e-6. If one is
  off, that method has a sign error, a swapped `fx`/`fy`, or is using the wrong edge of the box.
- `depth_from_ground_plane` uses the box's **bottom** edge (`box.y2`), not its centre. For a level
  camera it must reduce exactly to `Z = h·fy / (v - cy)`, and it must return `None` when
  `v <= cy` (at or above the horizon).
- A downward pitch tilts the floor normal from `(0, 1, 0)` into `(0, cos θ, sin θ)` — the sign
  matters, and the pitch test pins it: 1° makes a 2 m object read 1.61 m.
- `depth_from_depth_image`: the depth array is `(H, W)` — index it `[y1:y2, x1:x2]`, clip the
  window to the image, drop zeros *and* NaNs, and use the **median**, never the mean.
- `position_covariance`: write the 3×3 Jacobian of `(X, Y, Z)` with respect to `(u, v, Z)` first.
  The `∂X/∂Z = (u - cx)/fx` term is the one that makes an off-centre object inherit the depth error
  sideways — a test checks exactly that.
- `fuse`: floor the covariance **diagonal** after the update (`np.fill_diagonal` with
  `np.maximum`). Without the floor, 50 fusions claim millimetre certainty and the object then
  rejects its own next detection — the last two tests are that story.
