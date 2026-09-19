"""ArUco / AprilTag helpers that work on both OpenCV APIs.

OpenCV 4.7 moved ArUco from opencv_contrib into the main `objdetect` module and changed the API:

  old (<= 4.6, e.g. Ubuntu 24.04 apt python3-opencv 4.6.0)   new (>= 4.7, pip opencv-python 4.x / 5.x)
  cv2.aruco.DetectorParameters_create()                       cv2.aruco.DetectorParameters()
  cv2.aruco.detectMarkers(img, dict, parameters=p)            cv2.aruco.ArucoDetector(dict, p).detectMarkers(img)
  cv2.aruco.drawMarker(dict, id, side, borderBits=1)          cv2.aruco.generateImageMarker(dict, id, side, borderBits=1)
  cv2.aruco.estimatePoseSingleMarkers(...)                    removed from objdetect -> use cv2.solvePnP (below)

cv2.solvePnP with SOLVEPNP_IPPE_SQUARE exists in both, so pose estimation is written once.
"""
from __future__ import annotations

import cv2
import numpy as np

HAS_NEW_ARUCO_API = hasattr(cv2.aruco, "ArucoDetector")


def get_dictionary(dict_id: int = cv2.aruco.DICT_APRILTAG_36h11):
    return cv2.aruco.getPredefinedDictionary(dict_id)


def detector_parameters(subpixel: bool = True):
    params = cv2.aruco.DetectorParameters() if HAS_NEW_ARUCO_API else cv2.aruco.DetectorParameters_create()
    if subpixel:                    # default is CORNER_REFINE_NONE: whole-pixel corners, worse poses
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def generate_marker(dictionary, marker_id: int, side_px: int, border_bits: int = 1) -> np.ndarray:
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, borderBits=border_bits)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px, borderBits=border_bits)


def detect_markers(gray: np.ndarray, dictionary, params=None):
    """Returns (corners, ids, rejected) exactly like OpenCV: corners is a tuple of (1,4,2) arrays
    ordered top-left, top-right, bottom-right, bottom-left (as printed); ids is (N,1) or None."""
    params = params if params is not None else detector_parameters()
    if HAS_NEW_ARUCO_API:
        return cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def marker_object_points(marker_len_m: float) -> np.ndarray:
    """3D corners in the marker frame, in the order detectMarkers returns them.

    Marker frame: origin at the marker center, x to the right, y up, z out of the paper toward
    the viewer. This is the order SOLVEPNP_IPPE_SQUARE requires.
    """
    h = marker_len_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def estimate_marker_pose(corners_px: np.ndarray, marker_len_m: float, K: np.ndarray,
                         dist: np.ndarray | None,
                         max_err_px: float = 2.0) -> tuple[np.ndarray, np.ndarray, float]:
    """Pose of one square marker. Returns (T_optical_marker 4x4, [err_best, err_second], ratio).

    Uses solvePnPGeneric + IPPE_SQUARE, which returns BOTH candidate poses of a planar square.
    ratio = err_second / err_best; a ratio close to 1 means the pose is ambiguous (flip risk).
    IPPE can return garbage in a degenerate case (a perfectly axis-aligned, noise-free square),
    so the result is checked by reprojection and SQPNP is used as a fallback.
    """
    obj = marker_object_points(marker_len_m)
    img = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    errs = np.asarray(errs, dtype=np.float64).reshape(-1)
    order = np.argsort(errs)
    best = int(order[0])
    rvec, tvec = rvecs[best], tvecs[best]
    if not np.isfinite(errs[best]) or errs[best] > max_err_px:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        errs = np.array([float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1)))), np.inf])
        order = np.array([0, 1])
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec).reshape(3)
    sorted_errs = errs[order]
    ratio = float(sorted_errs[1] / max(sorted_errs[0], 1e-12)) if len(sorted_errs) > 1 else float("inf")
    return T, sorted_errs, ratio
