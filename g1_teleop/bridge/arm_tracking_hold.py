"""Hold last valid arm VR targets when wrist tracking is briefly lost."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ArmTrackingHoldState:
    left_valid: bool = False
    right_valid: bool = False
    left_position: np.ndarray | None = None
    left_orientation: np.ndarray | None = None
    right_position: np.ndarray | None = None
    right_orientation: np.ndarray | None = None
    left_wrist_joints: np.ndarray | None = None
    right_wrist_joints: np.ndarray | None = None
    debug: dict = field(default_factory=dict)

    def reset(self) -> None:
        self.left_valid = False
        self.right_valid = False
        self.left_position = None
        self.left_orientation = None
        self.right_position = None
        self.right_orientation = None
        self.left_wrist_joints = None
        self.right_wrist_joints = None
        self.debug = {}


def _side_slices(side: str) -> tuple[slice, slice]:
    if side == "left":
        return slice(0, 3), slice(0, 4)
    return slice(3, 6), slice(4, 8)


def apply_arm_tracking_hold(
    vr_position: np.ndarray,
    vr_orientation: np.ndarray,
    left_wrist_joints: np.ndarray | None,
    right_wrist_joints: np.ndarray | None,
    *,
    left_tracked: bool,
    right_tracked: bool,
    left_active: bool,
    right_active: bool,
    state: ArmTrackingHoldState,
    enabled: bool = True,
    last_sent_position: np.ndarray | None = None,
    last_sent_orientation: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None]:
    """Replace untracked active arms with the last valid command for that arm."""
    pos = np.asarray(vr_position, dtype=np.float64).reshape(9).copy()
    orn = np.asarray(vr_orientation, dtype=np.float64).reshape(12).copy()
    left_joints = None if left_wrist_joints is None else np.asarray(left_wrist_joints, dtype=np.float64).reshape(3).copy()
    right_joints = None if right_wrist_joints is None else np.asarray(right_wrist_joints, dtype=np.float64).reshape(3).copy()

    debug: dict = {"left_hold": False, "right_hold": False}

    if not enabled:
        state.debug = debug
        return pos, orn, left_joints, right_joints

    for side, tracked, active in (
        ("left", left_tracked, left_active),
        ("right", right_tracked, right_active),
    ):
        pos_sl, orn_sl = _side_slices(side)
        if not active:
            continue
        if tracked:
            if side == "left":
                state.left_valid = True
                state.left_position = pos[pos_sl].copy()
                state.left_orientation = orn[orn_sl].copy()
                if left_joints is not None:
                    state.left_wrist_joints = left_joints.copy()
            else:
                state.right_valid = True
                state.right_position = pos[pos_sl].copy()
                state.right_orientation = orn[orn_sl].copy()
                if right_joints is not None:
                    state.right_wrist_joints = right_joints.copy()
        elif side == "left":
            if (
                not state.left_valid
                and last_sent_position is not None
                and last_sent_orientation is not None
            ):
                state.left_valid = True
                state.left_position = np.asarray(last_sent_position, dtype=np.float64).reshape(9)[pos_sl].copy()
                state.left_orientation = np.asarray(last_sent_orientation, dtype=np.float64).reshape(12)[orn_sl].copy()
            if state.left_valid and state.left_position is not None and state.left_orientation is not None:
                pos[pos_sl] = state.left_position
                orn[orn_sl] = state.left_orientation
                if left_joints is not None and state.left_wrist_joints is not None:
                    left_joints = state.left_wrist_joints.copy()
                debug["left_hold"] = True
        elif side == "right":
            if (
                not state.right_valid
                and last_sent_position is not None
                and last_sent_orientation is not None
            ):
                state.right_valid = True
                state.right_position = np.asarray(last_sent_position, dtype=np.float64).reshape(9)[pos_sl].copy()
                state.right_orientation = np.asarray(last_sent_orientation, dtype=np.float64).reshape(12)[orn_sl].copy()
            if state.right_valid and state.right_position is not None and state.right_orientation is not None:
                pos[pos_sl] = state.right_position
                orn[orn_sl] = state.right_orientation
                if right_joints is not None and state.right_wrist_joints is not None:
                    right_joints = state.right_wrist_joints.copy()
                debug["right_hold"] = True

    state.debug = debug
    return pos, orn, left_joints, right_joints
