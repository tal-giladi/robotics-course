"""05.05 — 3D rotations in practice: matrices, ROS roll-pitch-yaw, quaternions, scipy.

    python 05-frames-and-transforms/code/rotations_demo.py

ROS convention (URDF rpy, tf2 setRPY, static_transform_publisher --roll --pitch --yaw):
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)   ==  fixed axes x, y, z  ==  intrinsic Z, Y', X''
scipy:  Rotation.from_euler("xyz", [roll, pitch, yaw])   (lowercase = fixed/extrinsic axes)
Quaternions: ROS and scipy both use (x, y, z, w).
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import frames_math as fm

deg, rad = math.degrees, math.radians


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)

    # 1. positive rotations follow the right-hand rule (REP-103: x forward, y left, z up)
    print("pitch +15 deg moves the x axis to", fm.rot_y(rad(15)) @ [1, 0, 0], "(nose DOWN)")
    print("roll  +10 deg moves the y axis to", fm.rot_x(rad(10)) @ [0, 1, 0], "(left side UP)")

    # 2. order matters
    print("Rz(90) @ Ry(90): child x ->", fm.rot_z(rad(90)) @ fm.rot_y(rad(90)) @ [1, 0, 0])
    print("Ry(90) @ Rz(90): child x ->", fm.rot_y(rad(90)) @ fm.rot_z(rad(90)) @ [1, 0, 0])

    # 3. ROS rpy == scipy "xyz" == scipy "ZYX" with reversed angles; "XYZ" is a different rotation
    roll, pitch, yaw = rad(30), rad(45), rad(90)
    R_ros = fm.rpy_to_matrix(roll, pitch, yaw)
    for seq, angles in (("xyz", [30, 45, 90]), ("ZYX", [90, 45, 30]), ("XYZ", [30, 45, 90])):
        R = Rotation.from_euler(seq, angles, degrees=True).as_matrix()
        rpy = [round(deg(a), 2) for a in fm.matrix_to_rpy(R)]
        print(f"scipy from_euler({seq!r}, {angles}) == ROS rpy(30, 45, 90)? {np.allclose(R, R_ros)}"
              f"  -> as ROS rpy {rpy}")
    print("forgot degrees=True: from_euler('xyz', [0, 0, 90]) is yaw",
          Rotation.from_euler("xyz", [0, 0, 90]).as_euler("xyz", degrees=True)[2].round(2), "deg")

    # 4. quaternions
    q_yaw90 = Rotation.from_euler("xyz", [0, 0, 90], degrees=True).as_quat()
    print("yaw 90 deg quaternion (x, y, z, w):", q_yaw90, " scalar_first:",
          Rotation.from_euler("xyz", [0, 0, 90], degrees=True).as_quat(scalar_first=True))
    print("camera_link -> camera_optical_frame, rpy(-90, 0, -90) deg:",
          Rotation.from_euler("xyz", [-90, 0, -90], degrees=True).as_quat())
    print("yaw of (0, 0, 0.3827, 0.9239):", round(deg(fm.yaw_from_quat([0, 0, 0.3827, 0.9239])), 2), "deg")
    print("q and -q are the same rotation:",
          np.allclose(fm.quat_to_matrix(q_yaw90), fm.quat_to_matrix(-q_yaw90)))
    q30, q60 = fm.quat_from_yaw(rad(30)), fm.quat_from_yaw(rad(60))
    print("q(yaw 30) * q(yaw 60) =", fm.quat_multiply(q30, q60), "= yaw",
          round(deg(fm.yaw_from_quat(fm.quat_multiply(q30, q60))), 2))
    print("yaw 90 then roll 90 (moving axes) =", fm.quat_multiply(fm.quat_from_yaw(rad(90)), fm.quat_from_rpy(rad(90), 0, 0)),
          "= 120 deg about (1,1,1)/sqrt3:", Rotation.from_rotvec(rad(120) * np.ones(3) / math.sqrt(3)).as_quat())

    # 5. the w-order bug: ROS (x, y, z, w) read as (w, x, y, z)
    wrong = Rotation.from_quat(q_yaw90, scalar_first=True)
    print("yaw 90 read in the wrong order -> rpy", wrong.as_euler("xyz", degrees=True).round(2),
          "forward axis ->", wrong.apply([1, 0, 0]).round(4))

    # 6. gimbal lock: at pitch 90 deg only (yaw - roll) matters
    a = fm.rpy_to_matrix(rad(10), rad(90), rad(20))
    b = fm.rpy_to_matrix(rad(0), rad(90), rad(10))
    print("rpy(10, 90, 20) == rpy(0, 90, 10)?", np.allclose(a, b),
          " matrix_to_rpy returns", [round(deg(v), 2) for v in fm.matrix_to_rpy(a)])

    # 7. interpolation and averaging: slerp, not component averages
    q_a = Rotation.from_euler("z", 170, degrees=True)
    q_b = Rotation.from_euler("z", -170, degrees=True)
    mid = Slerp([0.0, 1.0], Rotation.concatenate([q_a, q_b]))([0.5])
    naive = (q_a.as_quat() + q_b.as_quat()) / 2
    print("slerp halfway 170 -> -170:", mid.as_euler("xyz", degrees=True)[0].round(2),
          " naive component average ->", Rotation.from_quat(naive).as_euler("xyz", degrees=True).round(2))


if __name__ == "__main__":
    main()
