#!/usr/bin/env python3
"""Pinhole projection helpers: pixel -> ray -> map plane (ROS-free, unit-tested).

Camera convention: ROS optical frame, +x right, +y down, +z forward.
World/map convention: arena frame, +x east, +y north, +z up.

`pixel_to_plane` intersects the camera ray with a horizontal plane (the cube-centre
height), which is the simplest robust way to get a world pose from a bbox centre when
the objects sit on a flat floor/tier.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CameraInfo:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 640
    height: int = 480

    @classmethod
    def from_k(cls, k: list[float], width: int = 640, height: int = 480) -> "CameraInfo":
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
                   width=width, height=height)


def rotation_matrix(quat: tuple[float, float, float, float]):
    """Quaternion (x, y, z, w) -> 3x3 body-to-world rotation matrix."""
    x, y, z, w = quat
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _mat_vec(r, v):
    return (
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    )


def _transpose(r):
    return tuple(tuple(r[j][i] for j in range(3)) for i in range(3))


def camera_ray(info: CameraInfo, u: float, v: float) -> tuple[float, float, float]:
    return ((u - info.cx) / info.fx, (v - info.cy) / info.fy, 1.0)


def pixel_to_plane(
    info: CameraInfo,
    u: float,
    v: float,
    camera_xyz: tuple[float, float, float],
    camera_quat_xyzw: tuple[float, float, float, float],
    plane_z: float,
) -> Optional[tuple[float, float]]:
    """World (x, y) where the pixel ray crosses z = plane_z, or None."""
    r = rotation_matrix(camera_quat_xyzw)
    direction = _mat_vec(r, camera_ray(info, u, v))
    if abs(direction[2]) < 1e-9:
        return None
    t = (plane_z - camera_xyz[2]) / direction[2]
    if t <= 0.0:
        return None
    return (camera_xyz[0] + t * direction[0], camera_xyz[1] + t * direction[1])


def world_to_pixel(
    info: CameraInfo,
    point_xyz: tuple[float, float, float],
    camera_xyz: tuple[float, float, float],
    camera_quat_xyzw: tuple[float, float, float, float],
) -> Optional[tuple[float, float]]:
    """Inverse projection (used for tests and diagnostics)."""
    r = rotation_matrix(camera_quat_xyzw)
    rel = (point_xyz[0] - camera_xyz[0],
           point_xyz[1] - camera_xyz[1],
           point_xyz[2] - camera_xyz[2])
    cam = _mat_vec(_transpose(r), rel)
    if cam[2] <= 1e-6:
        return None
    return (info.cx + info.fx * cam[0] / cam[2],
            info.cy + info.fy * cam[1] / cam[2])


def inside_arena(x: float, y: float, arena: dict) -> bool:
    ox, oy = arena.get("origin", [0.0, 0.0])
    sx, sy = arena.get("size", [3.0, 3.0])
    return ox <= x <= ox + sx and oy <= y <= oy + sy


def plane_for_class(cls: str, default: float, by_class: dict[str, float]) -> float:
    return float(by_class.get(cls, default))


def yaw_from_quaternion(quat: tuple[float, float, float, float]) -> float:
    x, y, z, w = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------------------
# Production DLT (transform_3d.dat), used on the real robot
# ---------------------------------------------------------------------------

# Camera intrinsics from the robot's D435 color stream (match_logic.py).
DLT_FX = 605.6510620117188
DLT_FY = 605.8469848632812
DLT_CX = 315.03875732421875
DLT_CY = 249.8477020263672

# URDF mount: base_link -> Base = (0.044, 0, 0.138), no rotation.
ARM_MOUNT_X = 0.044
ARM_MOUNT_Y = 0.0


def load_dlt_matrix(path: str):
    """Load the 4x3 NumPy DLT matrix (transform_3d.dat)."""
    import numpy as np

    return np.load(path)


def _matrix_columns(m):
    return ([m[i][0] for i in range(4)],
            [m[i][1] for i in range(4)],
            [m[i][2] for i in range(4)])


def pixel_to_arm_at_z(m, u: float, v: float, z: float,
                      fx: float = DLT_FX, fy: float = DLT_FY,
                      cx: float = DLT_CX, cy: float = DLT_CY):
    """Closed-form inverse of the 4x3 DLT at height z (mm) -> arm (x, y) in mm.

    Copied from production match_logic.pixel_to_xy_at_z; m is the 4x3 matrix.
    """
    up = (u - cx) / fx
    vp = (v - cy) / fy
    r0, r1, r2 = _matrix_columns(m)
    c0 = r0[2] * z + r0[3]
    c1 = r1[2] * z + r1[3]
    c2 = r2[2] * z + r2[3]
    a11 = r0[0] - up * r2[0]
    a12 = r0[1] - up * r2[1]
    a21 = r1[0] - vp * r2[0]
    a22 = r1[1] - vp * r2[1]
    b1 = up * c2 - c0
    b2 = vp * c2 - c1
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-12:
        return None
    return (float((b1 * a22 - a12 * b2) / det),
            float((a11 * b2 - b1 * a21) / det))


def project_arm(m, x: float, y: float, z: float,
                fx: float = DLT_FX, fy: float = DLT_FY,
                cx: float = DLT_CX, cy: float = DLT_CY):
    """Forward DLT: arm (x, y, z) in mm -> pixel (u, v)."""
    try:
        cam = [x * m[0][j] + y * m[1][j] + z * m[2][j] + m[3][j]
               for j in range(3)]
        if cam[2] <= 1e-9:
            return None
        return (fx * cam[0] / cam[2] + cx, fy * cam[1] / cam[2] + cy)
    except Exception:
        return None


def arm_to_base(x_mm: float, y_mm: float,
                mount_x: float = ARM_MOUNT_X, mount_y: float = ARM_MOUNT_Y):
    """Arm-frame millimetres -> base_footprint metres (mount has no rotation)."""
    return (mount_x + x_mm / 1000.0, mount_y + y_mm / 1000.0)
