"""IMU feedback helpers for head locomotion closed loop."""

from __future__ import annotations

import numpy as np

from g1_teleop.bridge.rotation_utils import OfficialCalibration
from g1_teleop.bridge.vr_targets import official_calibration_head_pose
from g1_teleop.locomotion.head import horizontal_yaw_from_quat_wxyz


def loco_sync_head_pose(
    captured: OfficialCalibration,
    hold_kind: str,
    official_calibration: OfficialCalibration | None,
) -> np.ndarray:
    """Head reference for walk/squat/facing zeros.

    CALIB_SYNC only realigns wrists — keep the CALIB_FULL head height zero.
    """
    if hold_kind == "sync" and official_calibration is not None:
        return official_calibration_head_pose(official_calibration)
    return captured.head_pose


def loco_robot_base_quat(feedback) -> np.ndarray | None:
    """Return pelvis/base IMU quaternion (w,x,y,z) from deploy feedback, if present."""
    if feedback is None:
        return None
    if feedback.base_quat is not None:
        return np.asarray(feedback.base_quat, dtype=np.float64).reshape(4)
    if feedback.body_torso_quat is not None:
        return np.asarray(feedback.body_torso_quat, dtype=np.float64).reshape(4)
    return None


def loco_robot_base_yaw(feedback) -> float | None:
    quat = loco_robot_base_quat(feedback)
    if quat is None:
        return None
    return horizontal_yaw_from_quat_wxyz(quat)
