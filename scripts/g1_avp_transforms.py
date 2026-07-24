"""Shared AVP / OpenXR -> Unitree G1 robot-frame transforms (work-master aligned)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

INV_YUP2ZUP = np.linalg.inv(
    np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
)

T_ROBOT_OPENXR = np.array(
    [
        [0.0, 0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_OPENXR_ROBOT = np.array(
    [
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def avp_to_robot(transform: np.ndarray) -> np.ndarray:
    """Map raw Vision Pro (Y-up) 4x4 pose into robot Z-up frame."""
    openxr = INV_YUP2ZUP @ np.asarray(transform, dtype=np.float64)
    return T_ROBOT_OPENXR @ openxr @ T_OPENXR_ROBOT


def openxr_to_robot(transform: np.ndarray) -> np.ndarray:
    """Map TeleVuer / native streamer OpenXR pose into robot Z-up frame."""
    openxr = np.asarray(transform, dtype=np.float64)
    return T_ROBOT_OPENXR @ openxr @ T_OPENXR_ROBOT


def yaw_from_rot_matrix(rot: np.ndarray) -> float:
    """Matrix-column yaw used by Pico-style hand yaw compensation."""
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    return float(np.arctan2(r[1, 0], r[0, 0]))


def yaw_from_rot(rot: np.ndarray) -> float:
    """Horizontal heading for locomotion / facing (singularity at look-down).

    Uses the Z-axis twist component so gimbal lock sits under the feet, not when
    the user looks up. Do not use this for hand IK — use yaw_from_rot_matrix().
    """
    x, y, z, w = R.from_matrix(np.asarray(rot, dtype=np.float64)[:3, :3]).as_quat()
    norm = float(np.hypot(z, w))
    if norm < 1e-8:
        return yaw_from_rot_matrix(rot)
    return float(2.0 * np.arctan2(z, w))


def rotation_z(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
