"""Build SONIC vr_position / vr_orientation targets from AVP tracking."""

from __future__ import annotations

import os

import numpy as np

from g1_teleop.bridge.constants import (
    INV_YUP2ZUP,
    T_OPENXR_ROBOT,
    T_ROBOT_OPENXR,
    T_TO_UNITREE_HUMANOID_LEFT_ARM,
    T_TO_UNITREE_HUMANOID_RIGHT_ARM,
)
from g1_teleop.bridge.rotation_utils import (
    OfficialCalibration,
    quat_wxyz_to_rotmat,
    rotmat_to_quat_wxyz,
    scale_rotation,
)
from g1_teleop.calibration.session import finalize_pose_buffer
from g1_teleop.transforms.frames import rotation_z, yaw_from_rot, yaw_from_rot_matrix


def avp_to_robot(transform: np.ndarray) -> np.ndarray:
    openxr = INV_YUP2ZUP @ np.asarray(transform, dtype=np.float64)
    return T_ROBOT_OPENXR @ openxr @ T_OPENXR_ROBOT


def clamp_vec(vec: np.ndarray, limits: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(vec, -limits), limits)


def hand_is_active(args, side: str) -> bool:
    return getattr(args, "active_hands", "both") in ("both", side)


def build_vr_targets(tracking, initial_head_robot_pos: np.ndarray, args) -> tuple[np.ndarray, np.ndarray] | None:
    if tracking is None or tracking.head is None:
        return None

    head_robot = avp_to_robot(tracking.head)
    head_local = (head_robot[:3, 3] - initial_head_robot_pos) * args.head_position_scale
    head_local += np.array([args.head_to_waist_x, args.head_to_waist_y, args.head_to_waist_z], dtype=np.float64)

    positions = []
    orientations = []

    for hand, arm_transform, fallback_y in (
        (getattr(tracking, "left", None) if hand_is_active(args, "left") else None, T_TO_UNITREE_HUMANOID_LEFT_ARM, args.left_fallback_y),
        (getattr(tracking, "right", None) if hand_is_active(args, "right") else None, T_TO_UNITREE_HUMANOID_RIGHT_ARM, args.right_fallback_y),
    ):
        if hand is not None and getattr(hand, "wrist", None) is not None:
            wrist_robot = avp_to_robot(hand.wrist)
            wrist_pose = wrist_robot @ arm_transform
            rel = (wrist_robot[:3, 3] - head_robot[:3, 3]) * args.hand_position_scale
            pos = head_local + rel
            orn = rotmat_to_quat_wxyz(wrist_pose[:3, :3])
        else:
            pos = head_local + np.array([0.25, fallback_y, -0.15], dtype=np.float64)
            orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        positions.append(clamp_vec(pos, np.array([args.max_x, args.max_y, args.max_z], dtype=np.float64)))
        orientations.append(orn)

    positions.append(clamp_vec(head_local, np.array([args.max_x, args.max_y, args.max_z], dtype=np.float64)))
    orientations.append(rotmat_to_quat_wxyz(head_robot[:3, :3]))

    return np.concatenate(positions), np.concatenate(orientations)


def pose_or_none(tracking, side: str, arm_transform: np.ndarray, args=None) -> np.ndarray | None:
    if args is not None and not hand_is_active(args, side):
        return None
    hand = getattr(tracking, side, None) if tracking is not None else None
    if hand is None or getattr(hand, "wrist", None) is None:
        return None
    return avp_to_robot(hand.wrist) @ arm_transform


def head_yaw_compensated_relative(head_pose: np.ndarray, wrist_pose: np.ndarray) -> np.ndarray:
    """Official Pico-style wrist target: wrist relative to head with headset yaw removed."""
    inverse_head_yaw = rotation_z(-yaw_from_rot_matrix(head_pose[:3, :3]))
    return inverse_head_yaw @ (wrist_pose[:3, 3] - head_pose[:3, 3])


def head_yaw_compensated_rotation(head_pose: np.ndarray, wrist_pose: np.ndarray) -> np.ndarray:
    inverse_head_yaw = rotation_z(-yaw_from_rot_matrix(head_pose[:3, :3]))
    return inverse_head_yaw @ wrist_pose[:3, :3]


def official_calibration_head_pose(calibration: OfficialCalibration) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = np.asarray(calibration.head_rotation, dtype=np.float64)[:3, :3]
    pose[:3, 3] = np.asarray(calibration.head_pose[:3, 3], dtype=np.float64)
    return pose


def make_official_calibration(
    head_pose: np.ndarray,
    left_pose: np.ndarray | None,
    right_pose: np.ndarray | None,
) -> OfficialCalibration:
    return OfficialCalibration(
        head_pose=head_pose.copy(),
        head_rotation=head_pose[:3, :3].copy(),
        left_rel=None if left_pose is None else head_yaw_compensated_relative(head_pose, left_pose),
        right_rel=None if right_pose is None else head_yaw_compensated_relative(head_pose, right_pose),
        left_orientation=None if left_pose is None else rotmat_to_quat_wxyz(left_pose[:3, :3]),
        right_orientation=None if right_pose is None else rotmat_to_quat_wxyz(right_pose[:3, :3]),
        left_rotation=None if left_pose is None else head_yaw_compensated_rotation(head_pose, left_pose),
        right_rotation=None if right_pose is None else head_yaw_compensated_rotation(head_pose, right_pose),
    )


def tracking_arm_poses(tracking, args=None) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    if tracking is None or tracking.head is None:
        return None, None, None
    head_pose = avp_to_robot(tracking.head)
    left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM, args)
    right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM, args)
    return head_pose, left_pose, right_pose


def capture_official_calibration(tracking, args=None) -> OfficialCalibration | None:
    head_pose, left_pose, right_pose = tracking_arm_poses(tracking, args)
    if head_pose is None:
        return None
    return make_official_calibration(head_pose, left_pose, right_pose)


def finalize_calib_buffer(
    buffer,
    args,
    *,
    head_only: bool = False,
) -> tuple[OfficialCalibration | None, object]:
    require_left = hand_is_active(args, "left")
    require_right = hand_is_active(args, "right")

    def build_fn(head_pose, left_pose, right_pose):
        return make_official_calibration(head_pose, left_pose, right_pose)

    return finalize_pose_buffer(
        buffer,
        min_frames=args.calib_min_frames,
        max_head_std_m=args.calib_max_head_std,
        max_wrist_std_m=args.calib_max_wrist_std,
        require_left=require_left,
        require_right=require_right,
        head_only=head_only,
        build_calibration_fn=build_fn,
    )


def print_staged_calib_help() -> None:
    print(
        "\nStaged calibration (PICO VR_3PT aligned):\n"
        "  F  CALIB_FULL — YOU hold forearms-forward L-shape ~2s.\n"
        "                 Records your AVP pose + head/squat/walk zeros.\n"
        "                 Robot mapping reference = configured L init (not sent yet if policy off).\n"
        "  ]  ENGAGE      — Start balance policy; robot holds configured L init-pose.\n"
        "  S  CALIB_SYNC  — Match YOUR arms to that robot L-shape on screen, hold ~2s.\n"
        "                 Updates wrist zero + mapping base (init-pose uses commanded L,\n"
        "                 not FK lag). Head/squat zero stays from F.\n"
        "  T  TELEOP      — Live AVP hands + head walk/squat (wrist zero re-snapped at T).\n"
        "  H  HEAD zero   — Recalibrate facing/squat height only\n"
        "  P  PAUSE       — Freeze upper-body mapping\n"
        "  c  alias for F"
    )


def configured_robot_init_pose(args) -> np.ndarray:
    if args.robot_init_pose in ("debug-ready", "forearms-forward"):
        left = np.array([0.38, 0.124, 0.095], dtype=np.float64)
        right = np.array([0.38, -0.124, 0.095], dtype=np.float64)
        head = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)
    elif args.robot_init_pose == "arms-forward":
        left = np.array([0.32, 0.18, 0.26], dtype=np.float64)
        right = np.array([0.32, -0.18, 0.26], dtype=np.float64)
        head = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)
    elif args.robot_init_pose == "low-ready":
        left = np.array([0.24, 0.22, 0.28], dtype=np.float64)
        right = np.array([0.24, -0.22, 0.28], dtype=np.float64)
        head = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)
    else:
        left = np.array([args.neutral_x, args.neutral_left_y, args.neutral_hand_z], dtype=np.float64)
        right = np.array([args.neutral_x, args.neutral_right_y, args.neutral_hand_z], dtype=np.float64)
        head = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)

    return np.concatenate([left, right, head])


def default_official_base_positions(args) -> np.ndarray:
    return configured_robot_init_pose(args)


def default_official_base_orientations() -> np.ndarray:
    return np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64), 3)


def official_delta_axis_sign(args, side: str | None = None) -> np.ndarray:
    signs = []
    for axis in ("x", "y", "z"):
        global_sign = float(getattr(args, f"official_delta_sign_{axis}"))
        if side in ("left", "right"):
            override = getattr(args, f"{side}_official_delta_sign_{axis}", None)
            signs.append(global_sign if override is None else float(override))
        else:
            signs.append(global_sign)
    return np.array(signs, dtype=np.float64)


def hand_delta_remap_matrix(side: str | None, args) -> np.ndarray:
    """Per-arm linear basis for AVP rel deltas before axis signs / reach scales."""
    preset = "identity"
    if side in ("left", "right"):
        preset = getattr(args, f"{side}_hand_delta_remap", "identity")

    presets = {
        "identity": np.eye(3, dtype=np.float64),
        # Left AVP wrist rel couples forward/back (x) with reach-up (z) and lateral (y).
        # Decouple robot-X from vertical/lateral motion so upper-left does not pull the arm back.
        "unitree-left-arm": np.array(
            [
                [1.0, 0.2, -0.55],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
    }
    if preset not in presets:
        raise ValueError(f"Unknown hand-delta remap preset: {preset!r}")
    return presets[preset]


def hand_forward_backward_scales(args, side: str | None = None) -> tuple[float, float]:
    forward = float(args.hand_forward_scale)
    backward = float(args.hand_backward_scale)
    if side in ("left", "right"):
        forward_override = getattr(args, f"{side}_hand_forward_scale", None)
        backward_override = getattr(args, f"{side}_hand_backward_scale", None)
        if forward_override is not None:
            forward = float(forward_override)
        if backward_override is not None:
            backward = float(backward_override)
    return forward, backward


def apply_hand_workspace_shape(pos: np.ndarray, args) -> np.ndarray:
    shaped = np.asarray(pos, dtype=np.float64).copy()
    shaped[0] = np.clip(shaped[0], args.min_hand_x, args.max_hand_x)
    return shaped


def left_hand_z_delta_scale(raw_delta: np.ndarray, args) -> float:
    """Damp spurious Z on forward extension; keep full Z when reaching up."""
    damp = float(getattr(args, "left_hand_delta_z_scale", 1.0))
    up = float(getattr(args, "left_hand_delta_z_up_scale", 1.0))
    raw_dz = float(raw_delta[2])
    raw_dx = float(raw_delta[0])
    if raw_dz <= 0.0:
        return 1.0
    # AVP left wrist often picks up positive dz while reaching forward (negative dx).
    if raw_dx < -0.02 and abs(raw_dx) >= raw_dz:
        return damp
    return up


def official_hand_delta(rel: np.ndarray, calib_rel: np.ndarray, args, side: str | None = None) -> np.ndarray:
    raw_delta = np.asarray(rel - calib_rel, dtype=np.float64)
    mapped_delta = hand_delta_remap_matrix(side, args) @ raw_delta
    signed_delta = official_delta_axis_sign(args, side) * mapped_delta
    forward_scale, backward_scale = hand_forward_backward_scales(args, side)
    signed_delta[0] *= forward_scale if signed_delta[0] >= 0.0 else backward_scale
    delta = args.body_scale * signed_delta
    if side == "left":
        delta *= float(getattr(args, "left_hand_delta_scale", 1.0))
        delta[2] *= left_hand_z_delta_scale(raw_delta, args)
    elif side == "right":
        delta *= float(getattr(args, "right_hand_delta_scale", 1.0))
    return delta


def wrist_orientation_mode_for(side: str, args) -> str:
    override = getattr(args, f"{side}_wrist_orientation_mode", None)
    if override:
        return override
    return args.wrist_orientation_mode


def wrist_axis_remap_matrix(args, side: str | None = None) -> np.ndarray:
    remap = args.wrist_axis_remap
    if side == "left" and getattr(args, "left_wrist_axis_remap", None):
        remap = args.left_wrist_axis_remap
    elif side == "right" and getattr(args, "right_wrist_axis_remap", None):
        remap = args.right_wrist_axis_remap

    presets = {
        "identity": np.eye(3, dtype=np.float64),
        # AVP palm up/down currently lands on a side-flip axis. This preset rotates
        # the wrist delta basis so source local Z becomes target local X.
        "avp-palm": np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
        # Left-arm mirror: flip the medial/lateral wrist axis only (Y in remapped basis).
        "avp-palm-left": np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
        "x-to-y": np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "x-to-z": np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
        "y-to-x": np.array(
            [
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        "z-to-x": np.array(
            [
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        ),
    }
    return presets[remap]


def remap_wrist_rotation_delta(rotation_delta: np.ndarray, args, side: str | None = None) -> np.ndarray:
    basis = wrist_axis_remap_matrix(args, side)
    return basis @ rotation_delta @ basis.T


def rotmat_to_rotvec(rot: np.ndarray) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    cos_angle = np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))
    if angle < 1e-8:
        return np.zeros(3, dtype=np.float64)

    axis = np.array(
        [
            r[2, 1] - r[1, 2],
            r[0, 2] - r[2, 0],
            r[1, 0] - r[0, 1],
        ],
        dtype=np.float64,
    )
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-8:
        return np.zeros(3, dtype=np.float64)
    axis = axis / axis_norm
    return axis * angle


def rotvec_to_rotmat(rotvec: np.ndarray) -> np.ndarray:
    vec = np.asarray(rotvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vec))
    if angle < 1e-8:
        return np.eye(3, dtype=np.float64)
    axis = vec / angle
    k = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def wrist_rotation_signs(side: str, args) -> np.ndarray:
    if side == "left":
        return np.array(
            [args.left_wrist_rot_sign_x, args.left_wrist_rot_sign_y, args.left_wrist_rot_sign_z],
            dtype=np.float64,
        )
    return np.array(
        [args.right_wrist_rot_sign_x, args.right_wrist_rot_sign_y, args.right_wrist_rot_sign_z],
        dtype=np.float64,
    )


def apply_side_wrist_rotation_signs(rotation_delta: np.ndarray, side: str, args) -> np.ndarray:
    signs = wrist_rotation_signs(side, args)
    if np.allclose(signs, 1.0):
        return rotation_delta
    return rotvec_to_rotmat(signs * rotmat_to_rotvec(rotation_delta))


def calibrated_wrist_orientation(
    head_pose: np.ndarray,
    wrist_pose: np.ndarray,
    calibration_rotation: np.ndarray | None,
    base_orientation: np.ndarray,
    side: str,
    args,
) -> np.ndarray:
    if calibration_rotation is None:
        return np.asarray(base_orientation, dtype=np.float64).reshape(4)

    current_rotation = head_yaw_compensated_rotation(head_pose, wrist_pose)
    rotation_delta = calibration_rotation.T @ current_rotation
    rotation_delta = remap_wrist_rotation_delta(rotation_delta, args, side)
    rotation_delta = apply_side_wrist_rotation_signs(rotation_delta, side, args)
    rotation_delta = scale_rotation(rotation_delta, args.wrist_rotation_scale)
    target_rotation = quat_wxyz_to_rotmat(base_orientation) @ rotation_delta
    return rotmat_to_quat_wxyz(target_rotation)


def calibrated_head_orientation(
    head_pose: np.ndarray,
    calibration: OfficialCalibration,
    base_orientation: np.ndarray,
    args,
) -> np.ndarray:
    if args.head_yaw_only:
        yaw_delta = yaw_from_rot_matrix(head_pose[:3, :3]) - yaw_from_rot_matrix(calibration.head_rotation)
        rotation_delta = rotation_z(yaw_delta)
    else:
        rotation_delta = calibration.head_rotation.T @ head_pose[:3, :3]
    target_rotation = quat_wxyz_to_rotmat(base_orientation) @ rotation_delta
    return rotmat_to_quat_wxyz(target_rotation)


def rotmat_to_xyz_euler(rot: np.ndarray) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    pitch = float(np.arcsin(np.clip(r[0, 2], -1.0, 1.0)))
    roll = float(np.arctan2(-r[1, 2], r[2, 2]))
    yaw = float(np.arctan2(-r[0, 1], r[0, 0]))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def calibrated_wrist_joints(
    head_pose: np.ndarray,
    wrist_pose: np.ndarray,
    calibration_rotation: np.ndarray | None,
    side: str,
    args,
) -> np.ndarray | None:
    if calibration_rotation is None:
        return None

    current_rotation = head_yaw_compensated_rotation(head_pose, wrist_pose)
    rotation_delta = calibration_rotation.T @ current_rotation
    rotation_delta = remap_wrist_rotation_delta(rotation_delta, args, side)
    rotation_delta = apply_side_wrist_rotation_signs(rotation_delta, side, args)
    rotation_delta = scale_rotation(rotation_delta, args.wrist_rotation_scale)
    joints = rotmat_to_xyz_euler(rotation_delta)
    signs = np.array(
        [args.wrist_joint_sign_roll, args.wrist_joint_sign_pitch, args.wrist_joint_sign_yaw],
        dtype=np.float64,
    )
    limits = np.array(
        [args.max_wrist_roll, args.max_wrist_pitch, args.max_wrist_yaw],
        dtype=np.float64,
    )
    return clamp_vec(signs * joints, limits)


def build_calibrated_vr_targets(
    tracking,
    neutral_left_pose: np.ndarray | None,
    neutral_right_pose: np.ndarray | None,
    neutral_head_pose: np.ndarray,
    args,
) -> tuple[np.ndarray, np.ndarray] | None:
    if tracking is None or tracking.head is None:
        return None

    head_pose = avp_to_robot(tracking.head)
    left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM, args)
    right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM, args)

    left_base = np.array([args.neutral_x, args.neutral_left_y, args.neutral_hand_z], dtype=np.float64)
    right_base = np.array([args.neutral_x, args.neutral_right_y, args.neutral_hand_z], dtype=np.float64)
    head_base = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)
    limits = np.array([args.max_x, args.max_y, args.max_z], dtype=np.float64)

    positions = []
    orientations = []

    for pose, neutral_pose, base in (
        (left_pose, neutral_left_pose, left_base),
        (right_pose, neutral_right_pose, right_base),
    ):
        if pose is not None and neutral_pose is not None:
            delta = (pose[:3, 3] - neutral_pose[:3, 3]) * args.hand_position_scale
            pos = base + delta
            if args.wrist_orientation_mode == "live":
                orn = rotmat_to_quat_wxyz(pose[:3, :3])
            elif args.wrist_orientation_mode == "neutral":
                orn = rotmat_to_quat_wxyz(neutral_pose[:3, :3])
            else:
                orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            pos = base
            orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        positions.append(clamp_vec(pos, limits))
        orientations.append(orn)

    head_delta = (head_pose[:3, 3] - neutral_head_pose[:3, 3]) * args.head_position_scale
    positions.append(clamp_vec(head_base + head_delta, limits))
    orientations.append(rotmat_to_quat_wxyz(head_pose[:3, :3]))

    return np.concatenate(positions), np.concatenate(orientations)


def build_official_calib_vr_targets(
    tracking,
    calibration: OfficialCalibration,
    base_positions: np.ndarray,
    base_orientations: np.ndarray,
    args,
    force_base: bool = False,
) -> tuple[np.ndarray, np.ndarray, dict] | None:
    if tracking is None or tracking.head is None:
        return None

    head_pose = avp_to_robot(tracking.head)
    left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM, args)
    right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM, args)

    base_positions = np.asarray(base_positions, dtype=np.float64).reshape(9)
    base_orientations = np.asarray(base_orientations, dtype=np.float64).reshape(12)
    left_base = base_positions[0:3]
    right_base = base_positions[3:6]
    head_base = base_positions[6:9]
    left_base_orn = base_orientations[0:4]
    right_base_orn = base_orientations[4:8]
    head_base_orn = base_orientations[8:12]
    limits = np.array([args.max_x, args.max_y, args.max_z], dtype=np.float64)

    positions = []
    orientations = []
    debug = {"left_rel": None, "right_rel": None, "left_delta": None, "right_delta": None}
    left_wrist_joints = (
        np.zeros(3, dtype=np.float64)
        if wrist_orientation_mode_for("left", args) == "wrist-joints"
        else None
    )
    right_wrist_joints = (
        np.zeros(3, dtype=np.float64)
        if wrist_orientation_mode_for("right", args) == "wrist-joints"
        else None
    )

    for side, pose, calib_rel, calib_orientation, calib_rotation, base in (
        ("left", left_pose, calibration.left_rel, calibration.left_orientation, calibration.left_rotation, left_base),
        ("right", right_pose, calibration.right_rel, calibration.right_orientation, calibration.right_rotation, right_base),
    ):
        base_orn = left_base_orn if side == "left" else right_base_orn
        wrist_mode = wrist_orientation_mode_for(side, args)
        if force_base:
            pos = base
            orn = base_orn
        elif pose is not None and calib_rel is not None:
            rel = head_yaw_compensated_relative(head_pose, pose)
            delta = official_hand_delta(rel, calib_rel, args, side=side)
            pos = apply_hand_workspace_shape(base + delta, args)
            debug[f"{side}_rel"] = rel.copy()
            debug[f"{side}_delta"] = delta.copy()
            if wrist_mode == "live":
                orn = rotmat_to_quat_wxyz(pose[:3, :3])
            elif wrist_mode == "calibrated":
                orn = calibrated_wrist_orientation(head_pose, pose, calib_rotation, base_orn, side, args)
            elif wrist_mode == "wrist-joints":
                orn = base_orn
                wrist_joints = calibrated_wrist_joints(head_pose, pose, calib_rotation, side, args)
                if side == "left":
                    left_wrist_joints = wrist_joints
                else:
                    right_wrist_joints = wrist_joints
            elif wrist_mode == "neutral":
                orn = base_orn
            else:
                orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            pos = base
            orn = base_orn
        positions.append(clamp_vec(pos, limits))
        orientations.append(orn)

    if force_base or (args.lock_head_translation and not args.head_vertical_follow):
        head_delta = np.zeros(3, dtype=np.float64)
    elif args.lock_head_translation and args.head_vertical_follow:
        dz = (head_pose[2, 3] - calibration.head_pose[2, 3]) * args.head_vertical_scale
        head_delta = np.array([0.0, 0.0, dz], dtype=np.float64)
    else:
        head_delta = (head_pose[:3, 3] - calibration.head_pose[:3, 3]) * args.head_position_scale
    positions.append(clamp_vec(head_base + head_delta, limits))
    orientations.append(head_base_orn if force_base else calibrated_head_orientation(head_pose, calibration, head_base_orn, args))

    return np.concatenate(positions), np.concatenate(orientations), debug, left_wrist_joints, right_wrist_joints


def build_head_relative_vr_targets(
    tracking,
    neutral_left_pose: np.ndarray | None,
    neutral_right_pose: np.ndarray | None,
    args,
) -> tuple[np.ndarray, np.ndarray, dict] | None:
    if tracking is None or tracking.head is None:
        return None

    head_pose = avp_to_robot(tracking.head)
    left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM, args)
    right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM, args)

    head_base = np.array([args.neutral_head_x, args.neutral_head_y, args.neutral_head_z], dtype=np.float64)
    user_offset = np.array(
        [args.head_relative_x_offset, args.head_relative_y_offset, args.head_relative_z_offset],
        dtype=np.float64,
    )
    limits = np.array([args.max_x, args.max_y, args.max_z], dtype=np.float64)

    positions = []
    orientations = []
    debug = {"left_rel": None, "right_rel": None}

    for side, pose, neutral_pose, fallback_y in (
        ("left", left_pose, neutral_left_pose, args.left_fallback_y),
        ("right", right_pose, neutral_right_pose, args.right_fallback_y),
    ):
        if pose is not None:
            rel_to_head = pose[:3, 3] - head_pose[:3, 3]
            debug[f"{side}_rel"] = rel_to_head.copy()
            hand_anchor = head_base if args.hybrid_add_head_base_to_hands else np.zeros(3, dtype=np.float64)
            pos = hand_anchor + args.body_scale * rel_to_head + user_offset
            if args.wrist_orientation_mode == "live":
                orn = rotmat_to_quat_wxyz(pose[:3, :3])
            elif args.wrist_orientation_mode == "neutral" and neutral_pose is not None:
                orn = rotmat_to_quat_wxyz(neutral_pose[:3, :3])
            else:
                orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            pos = head_base + np.array([0.25, fallback_y, -0.55], dtype=np.float64) * args.body_scale + user_offset
            orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        positions.append(clamp_vec(pos, limits))
        orientations.append(orn)

    positions.append(clamp_vec(head_base, limits))
    orientations.append(rotmat_to_quat_wxyz(head_pose[:3, :3]))

    return np.concatenate(positions), np.concatenate(orientations), debug


