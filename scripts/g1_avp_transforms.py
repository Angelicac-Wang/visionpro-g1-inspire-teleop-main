"""Shared AVP / OpenXR -> Unitree G1 robot-frame transforms (work-master aligned)."""

from __future__ import annotations

import numpy as np

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


def yaw_from_rot(rot: np.ndarray) -> float:
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    return float(np.arctan2(r[1, 0], r[0, 0]))


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
