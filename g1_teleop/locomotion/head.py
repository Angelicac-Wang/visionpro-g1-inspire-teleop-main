"""Head-driven locomotion for G1 teleop (work-master aligned).

Isaac sim: velocity commands on rt/run_command/cmd.
SONIC planner: movement/facing command vectors for legged walking policy.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

from g1_teleop.transforms.frames import avp_to_robot, openxr_to_robot, rotation_z, yaw_from_rot


def wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass
class HeadLocomotionConfig:
    velocity_gain: float = 1.0
    yaw_rate_gain: float = 0.9
    forward_scale: float = 1.0
    lateral_scale: float = 1.25
    lateral_left_scale: float = 1.0
    lateral_right_scale: float = 1.4
    sign_x: float = 1.0
    sign_y: float = 1.0
    max_speed: float = 0.45
    max_yaw_rate: float = 0.35
    velocity_deadzone: float = 0.07
    lateral_velocity_deadzone: float = 0.035
    lateral_displacement_gain: float = 0.0
    max_lateral_displacement: float = 0.12
    lateral_axis_ratio: float = 0.38
    lateral_strafe_min: float = 0.010
    lateral_coupling_suppress: float = 0.055
    yaw_rate_deadzone: float = 0.15
    smooth_alpha: float = 0.12
    facing_smooth_alpha: float = 0.2
    output_deadzone: float = 0.04
    lateral_output_deadzone: float = 0.025
    idle_decay: float = 0.85
    imu_yaw_enabled: bool = True
    imu_yaw_gain: float = 1.0
    imu_yaw_deadzone: float = 0.05
    imu_yaw_max_correction: float = 0.45


@dataclass
class HeadHeightSquatConfig:
    """Map AVP head vertical drop to SONIC planner squat/kneel height."""

    walk_height_threshold: float = 0.72
    squat_height_min: float = 0.50
    kneel_height_min: float = 0.35
    head_drop_start: float = 0.06
    head_drop_to_squat: float = 0.24
    head_drop_to_kneel: float = 0.42
    min_height: float = 0.35
    max_height: float = 0.88
    smooth_alpha: float = 0.18
    squat_mode: int = 4
    kneel_mode: int = 6


@dataclass
class HeadLocomotionState:
    prev_head_pos: np.ndarray | None = None
    prev_head_yaw: float | None = None
    smooth_vx: float = 0.0
    smooth_vy: float = 0.0
    smooth_vyaw: float = 0.0
    smooth_pelvis_height: float = -1.0
    facing_angle: float = 0.0
    calib_yaw: float = 0.0
    calib_rot: np.ndarray | None = None
    robot_base_yaw_at_calib: float | None = None
    calibrated: bool = False
    debug: dict = field(default_factory=dict)


@dataclass
class SonicPlannerCommand:
    mode: int
    movement: np.ndarray
    facing: np.ndarray
    speed: float
    height: float = -1.0


def head_pose_from_tracking(tracking) -> np.ndarray | None:
    if tracking is None or getattr(tracking, "head", None) is None:
        return None
    return avp_to_robot(np.asarray(tracking.head, dtype=np.float64))


def head_pose_from_openxr(head_pose_openxr: np.ndarray) -> np.ndarray:
    """Convert TeleVuer OpenXR 4x4 head pose to robot frame."""
    return openxr_to_robot(np.asarray(head_pose_openxr, dtype=np.float64))


def head_delta_yaw_compensated(head_pose: np.ndarray, calib_pos: np.ndarray) -> np.ndarray:
    """Horizontal head displacement in a yaw-stabilized body frame."""
    delta_world = head_pose[:3, 3] - calib_pos
    head_yaw = yaw_from_rot(head_pose[:3, :3])
    return rotation_z(-head_yaw) @ delta_world


def head_delta_calib_frame(
    head_pose: np.ndarray,
    calib_pos: np.ndarray,
    calib_rot: np.ndarray,
) -> np.ndarray:
    """Head displacement in the calibrated head frame (x=lateral, z=forward/back)."""
    delta_world = head_pose[:3, 3] - calib_pos
    rot = np.asarray(calib_rot, dtype=np.float64)[:3, :3]
    return rot.T @ delta_world


def horizontal_delta_calib_yaw(
    head_pose: np.ndarray,
    calib_pos: np.ndarray,
    calib_yaw: float,
) -> np.ndarray:
    """Horizontal head displacement in the F-calibration yaw frame (x=fwd, y=lat)."""
    delta_world = head_pose[:3, 3] - calib_pos
    delta_world[2] = 0.0
    return rotation_z(-calib_yaw) @ delta_world


def horizontal_velocity_calib_yaw(
    vel_world: np.ndarray,
    calib_yaw: float,
) -> np.ndarray:
    """Horizontal head velocity in the F-calibration yaw frame (x=fwd, y=lat)."""
    vel = np.asarray(vel_world, dtype=np.float64).copy()
    vel[2] = 0.0
    return rotation_z(-calib_yaw) @ vel


def horizontal_basis_from_calib_rot(calib_rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Unit forward/lateral axes on the ground plane from the F-calibration rotation."""
    rot = np.asarray(calib_rot, dtype=np.float64)[:3, :3]
    forward = rot[:, 0].copy()
    forward[2] = 0.0
    fn = float(np.linalg.norm(forward))
    if fn < 1e-6:
        forward = (-rot[:, 2]).copy()
        forward[2] = 0.0
        fn = float(np.linalg.norm(forward))
    forward /= max(fn, 1e-6)
    lateral = np.cross(np.array([0.0, 0.0, 1.0]), forward)
    ln = float(np.linalg.norm(lateral))
    if ln < 1e-6:
        lateral = rot[:, 1].copy()
        lateral[2] = 0.0
        lateral /= max(float(np.linalg.norm(lateral)), 1e-6)
    else:
        lateral /= ln
    return forward, lateral


def horizontal_heading_from_calib_rot(calib_rot: np.ndarray) -> float:
    """Ground-plane heading of the calibrated forward axis."""
    forward, _ = horizontal_basis_from_calib_rot(calib_rot)
    return float(np.arctan2(forward[1], forward[0]))


def horizontal_velocity_calib_rot(
    vel_world: np.ndarray,
    calib_rot: np.ndarray,
) -> np.ndarray:
    """Horizontal velocity in the F-calibration walk frame (x=fwd, y=lat)."""
    forward, lateral = horizontal_basis_from_calib_rot(calib_rot)
    vel = np.asarray(vel_world, dtype=np.float64).copy()
    vel[2] = 0.0
    return np.array([np.dot(vel, forward), np.dot(vel, lateral), 0.0])


def horizontal_delta_calib_rot(
    head_pose: np.ndarray,
    calib_pos: np.ndarray,
    calib_rot: np.ndarray,
) -> np.ndarray:
    """Horizontal head displacement in the F-calibration walk frame."""
    delta_world = head_pose[:3, 3] - calib_pos
    delta_world[2] = 0.0
    forward, lateral = horizontal_basis_from_calib_rot(calib_rot)
    return np.array(
        [np.dot(delta_world, forward), np.dot(delta_world, lateral), 0.0],
        dtype=np.float64,
    )


def walk_vector_to_world(calib_yaw: float, vx: float, vy: float) -> tuple[float, float]:
    """Map walk-frame (vx forward, vy lateral) to world XY using F-calib yaw."""
    c = np.cos(calib_yaw)
    s = np.sin(calib_yaw)
    return c * vx - s * vy, s * vx + c * vy


def walk_vector_to_world_rot(calib_rot: np.ndarray, vx: float, vy: float) -> tuple[float, float]:
    """Map walk-frame (vx forward, vy lateral) to world XY using F-calib rotation."""
    forward, lateral = horizontal_basis_from_calib_rot(calib_rot)
    world = forward * vx + lateral * vy
    return float(world[0]), float(world[1])


def _calib_rot_or_yaw(state: HeadLocomotionState) -> np.ndarray:
    if state.calib_rot is not None:
        return state.calib_rot
    return rotation_z(state.calib_yaw)


def horizontal_yaw_from_quat_wxyz(quat: np.ndarray) -> float:
    """Ground-plane heading from a base/torso IMU quaternion (w, x, y, z)."""
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    w, x, y, z = q
    rot = R.from_quat([x, y, z, w]).as_matrix()
    return horizontal_heading_from_calib_rot(rot)


def _deadband_angle(angle: float, deadzone: float) -> float:
    if abs(angle) <= deadzone:
        return 0.0
    return angle - deadzone if angle > 0.0 else angle + deadzone


def apply_imu_yaw_closed_loop(
    facing_angle: float,
    state: HeadLocomotionState,
    cfg: HeadLocomotionConfig,
    base_quat: np.ndarray | None,
) -> tuple[float, dict]:
    """Close the loop between commanded facing and measured base IMU yaw.

    Returns corrected facing angle (rad, calib-relative) and debug fields.
    """
    debug: dict = {
        "imu_active": False,
        "robot_yaw_rel": None,
        "yaw_err": None,
        "facing_corr": 0.0,
    }
    if (
        not cfg.imu_yaw_enabled
        or base_quat is None
        or state.robot_base_yaw_at_calib is None
    ):
        return facing_angle, debug

    robot_yaw = horizontal_yaw_from_quat_wxyz(base_quat)
    robot_yaw_rel = wrap_to_pi(robot_yaw - state.robot_base_yaw_at_calib)
    yaw_err = wrap_to_pi(facing_angle - robot_yaw_rel)
    yaw_err_db = _deadband_angle(yaw_err, cfg.imu_yaw_deadzone)
    facing_corr = float(
        np.clip(cfg.imu_yaw_gain * yaw_err_db, -cfg.imu_yaw_max_correction, cfg.imu_yaw_max_correction)
    )
    corrected = wrap_to_pi(facing_angle + facing_corr)
    debug.update(
        {
            "imu_active": True,
            "robot_yaw_rel": round(robot_yaw_rel, 3),
            "yaw_err": round(yaw_err, 3),
            "facing_corr": round(facing_corr, 3),
            "facing_out": round(corrected, 3),
        }
    )
    return corrected, debug


def _loco_axes_from_yaw_velocity(
    vel_local: np.ndarray,
    cfg: HeadLocomotionConfig,
) -> tuple[float, float]:
    """Map yaw-stabilized horizontal velocity to (forward vx, lateral vy)."""
    vx = cfg.velocity_gain * cfg.forward_scale * cfg.sign_x * float(vel_local[0])
    vy = cfg.velocity_gain * cfg.lateral_scale * cfg.sign_y * float(vel_local[1])
    return vx, vy


def compute_head_locomotion_velocity(
    head_pose: np.ndarray,
    state: HeadLocomotionState,
    cfg: HeadLocomotionConfig,
    dt: float,
    *,
    calib_pos: np.ndarray,
) -> tuple[float, float, float]:
    """Return (vx, vy, vyaw) in the F-calibration yaw walk frame (velocity only)."""
    pos = head_pose[:3, 3]
    yaw = yaw_from_rot(head_pose[:3, :3])

    if state.prev_head_pos is None:
        state.prev_head_pos = pos.copy()
        state.prev_head_yaw = yaw
        state.calibrated = True
        state.debug = {
            "delta_body": [0.0, 0.0, 0.0],
            "raw_vel": [0.0, 0.0],
            "raw_vyaw": 0.0,
        }
        return 0.0, 0.0, 0.0

    vel_world = (pos - state.prev_head_pos) / max(dt, 1e-6)
    vel_world[2] = 0.0
    yaw_rate = wrap_to_pi(yaw - state.prev_head_yaw) / max(dt, 1e-6)
    state.prev_head_pos = pos.copy()
    state.prev_head_yaw = yaw

    calib_rot = _calib_rot_or_yaw(state)
    vel_local = horizontal_velocity_calib_rot(vel_world, calib_rot)
    vx, vy = _loco_axes_from_yaw_velocity(vel_local, cfg)
    delta_body = horizontal_delta_calib_rot(head_pose, calib_pos, calib_rot)
    vel_world_h = vel_world.copy()
    vel_world_h[2] = 0.0
    delta_body[2] = 0.0
    vy_disp = 0.0
    vx_disp = 0.0
    strafe_intent = False
    forward_intent = False

    if cfg.lateral_displacement_gain > 0.0:
        lat_disp = float(np.clip(delta_body[1], -cfg.max_lateral_displacement, cfg.max_lateral_displacement))
        fwd_disp = float(np.clip(delta_body[0], -cfg.max_lateral_displacement, cfg.max_lateral_displacement))
        strafe_intent = abs(lat_disp) >= cfg.lateral_strafe_min
        forward_intent = abs(fwd_disp) >= cfg.lateral_strafe_min
        if strafe_intent:
            vy_disp = cfg.lateral_displacement_gain * cfg.sign_y * lat_disp
            vy += vy_disp
        if forward_intent:
            vx_disp = cfg.lateral_displacement_gain * cfg.sign_x * fwd_disp
            vx += vx_disp

        if (
            not strafe_intent
            and abs(vx) >= cfg.velocity_deadzone
            and abs(vy) < cfg.lateral_coupling_suppress
        ):
            vy = 0.0

    if vy > 0.0:
        vy *= cfg.lateral_left_scale
    elif vy < 0.0:
        vy *= cfg.lateral_right_scale

    vyaw = cfg.yaw_rate_gain * yaw_rate

    move_speed = float(np.hypot(vx, vy))
    if move_speed < cfg.velocity_deadzone:
        vx = 0.0
        vy = 0.0
    lat_dz = cfg.lateral_velocity_deadzone
    if abs(vy) < lat_dz and vx == 0.0:
        vy = 0.0
    if abs(vyaw) < cfg.yaw_rate_deadzone:
        vyaw = 0.0

    alpha = float(np.clip(cfg.smooth_alpha, 0.05, 1.0))
    state.smooth_vx += alpha * (vx - state.smooth_vx)
    state.smooth_vy += alpha * (vy - state.smooth_vy)
    state.smooth_vyaw += alpha * (vyaw - state.smooth_vyaw)

    if vx == 0.0 and vy == 0.0:
        decay = float(np.clip(cfg.idle_decay, 0.0, 0.99))
        state.smooth_vx *= decay
        state.smooth_vy *= decay
    if vyaw == 0.0:
        state.smooth_vyaw *= float(np.clip(cfg.idle_decay, 0.0, 0.99))

    out_vx = float(np.clip(state.smooth_vx, -cfg.max_speed, cfg.max_speed))
    out_vy = float(np.clip(state.smooth_vy, -cfg.max_speed, cfg.max_speed))
    out_vyaw = float(np.clip(state.smooth_vyaw, -cfg.max_yaw_rate, cfg.max_yaw_rate))

    out_dz = float(getattr(cfg, "output_deadzone", cfg.velocity_deadzone))
    if float(np.hypot(out_vx, out_vy)) < out_dz:
        out_vx = 0.0
        out_vy = 0.0
        state.smooth_vx = 0.0
        state.smooth_vy = 0.0
    if abs(out_vyaw) < out_dz:
        out_vyaw = 0.0
        state.smooth_vyaw = 0.0

    state.debug = {
        "delta_body": np.round(delta_body, 3).tolist(),
        "vel_local": np.round(vel_local, 3).tolist(),
        "vel_world_h": np.round(vel_world_h, 3).tolist(),
        "raw_vel": np.round([vx, vy], 3).tolist(),
        "vy_disp": round(vy_disp, 3),
        "vx_disp": round(vx_disp, 3),
        "raw_vyaw": round(float(yaw_rate), 3),
        "cmd": np.round([out_vx, out_vy, out_vyaw], 3).tolist(),
        "strafe_intent": strafe_intent,
        "forward_intent": forward_intent,
        "head_yaw": round(float(yaw), 3),
        "calib_yaw": round(float(state.calib_yaw), 3),
        "sign": [cfg.sign_x, cfg.sign_y],
    }
    return out_vx, out_vy, out_vyaw


def compute_sonic_planner_command(
    head_pose: np.ndarray,
    state: HeadLocomotionState,
    cfg: HeadLocomotionConfig,
    dt: float,
    *,
    calib_pos: np.ndarray,
    locomotion_mode: int = 1,
    robot_base_quat: np.ndarray | None = None,
) -> SonicPlannerCommand:
    """Map head motion to SONIC planner movement/facing vectors.

    SONIC expects movement (travel direction, can be backward) separately from
    facing (body/head heading). Do NOT set facing from arctan2(vy, vx) or the
    robot turns around instead of walking backward.
    """
    calib_rot = _calib_rot_or_yaw(state)
    head_rot = head_pose[:3, :3]
    target_facing = wrap_to_pi(
        horizontal_heading_from_calib_rot(head_rot)
        - horizontal_heading_from_calib_rot(calib_rot)
    )
    fac_alpha = float(np.clip(cfg.facing_smooth_alpha, 0.05, 1.0))
    state.facing_angle = wrap_to_pi(
        state.facing_angle + fac_alpha * wrap_to_pi(target_facing - state.facing_angle)
    )
    facing_angle, imu_debug = apply_imu_yaw_closed_loop(
        state.facing_angle,
        state,
        cfg,
        robot_base_quat,
    )
    facing = _facing_from_angle(facing_angle)

    idle = SonicPlannerCommand(
        mode=0,
        movement=np.zeros(3, dtype=np.float64),
        facing=facing,
        speed=-1.0,
    )

    vx, vy, vyaw = compute_head_locomotion_velocity(
        head_pose,
        state,
        cfg,
        dt,
        calib_pos=calib_pos,
    )
    state.debug["imu"] = imu_debug

    local_speed = float(np.hypot(vx, vy))
    lat_dz = float(getattr(cfg, "lateral_velocity_deadzone", cfg.velocity_deadzone))

    forward_active = abs(vx) >= cfg.velocity_deadzone
    lateral_active = abs(vy) >= lat_dz
    if (forward_active or lateral_active) and local_speed > 1e-6:
        vel_wh = np.asarray(state.debug.get("vel_world_h") or [0.0, 0.0, 0.0], dtype=np.float64)
        wh_norm = float(np.linalg.norm(vel_wh[:2]))
        if wh_norm > 1e-6:
            movement_xy = vel_wh[:2] / wh_norm
        else:
            wx, wy = walk_vector_to_world_rot(calib_rot, vx, vy)
            movement_xy = np.array([wx, wy], dtype=np.float64)
            movement_xy /= max(float(np.linalg.norm(movement_xy)), 1e-6)
        movement = np.array([movement_xy[0], movement_xy[1], 0.0], dtype=np.float64)
        return SonicPlannerCommand(
            mode=locomotion_mode,
            movement=movement,
            facing=facing,
            speed=min(local_speed, cfg.max_speed),
        )

    return idle


def compute_head_pelvis_height(
    head_pose: np.ndarray,
    calib_pos: np.ndarray,
    state: HeadLocomotionState,
    cfg: HeadHeightSquatConfig,
) -> tuple[float, dict]:
    """Return smoothed planner pelvis height. -1 means standing default height."""
    delta_z = float(head_pose[2, 3] - calib_pos[2])
    drop = max(0.0, -delta_z - cfg.head_drop_start)

    if drop <= 0.0:
        raw_height = -1.0
    elif drop < cfg.head_drop_to_squat:
        t = drop / max(cfg.head_drop_to_squat, 1e-6)
        raw_height = cfg.walk_height_threshold - t * (cfg.walk_height_threshold - cfg.squat_height_min)
    elif drop < cfg.head_drop_to_kneel:
        t = (drop - cfg.head_drop_to_squat) / max(cfg.head_drop_to_kneel - cfg.head_drop_to_squat, 1e-6)
        raw_height = cfg.squat_height_min - t * (cfg.squat_height_min - cfg.kneel_height_min)
    else:
        raw_height = cfg.kneel_height_min

    alpha = float(np.clip(cfg.smooth_alpha, 0.05, 1.0))
    if raw_height < 0.0:
        if state.smooth_pelvis_height >= 0.0:
            state.smooth_pelvis_height += alpha * (cfg.walk_height_threshold - state.smooth_pelvis_height)
            if state.smooth_pelvis_height >= cfg.walk_height_threshold - 0.01:
                state.smooth_pelvis_height = -1.0
    else:
        raw_height = float(np.clip(raw_height, cfg.min_height, cfg.max_height))
        if state.smooth_pelvis_height < 0.0:
            state.smooth_pelvis_height = raw_height
        else:
            state.smooth_pelvis_height += alpha * (raw_height - state.smooth_pelvis_height)

    debug = {
        "head_delta_z": round(delta_z, 3),
        "head_drop": round(drop, 3),
        "pelvis_height": round(state.smooth_pelvis_height, 3),
    }
    return state.smooth_pelvis_height, debug


def apply_height_to_planner_command(
    cmd: SonicPlannerCommand,
    pelvis_height: float,
    cfg: HeadHeightSquatConfig,
) -> SonicPlannerCommand:
    """Merge pelvis height into a locomotion planner command (ROS2 teleop rules)."""
    if pelvis_height < 0.0 or pelvis_height >= cfg.walk_height_threshold:
        return SonicPlannerCommand(
            mode=cmd.mode,
            movement=cmd.movement.copy(),
            facing=cmd.facing.copy(),
            speed=cmd.speed,
            height=-1.0,
        )

    height = float(np.clip(pelvis_height, 0.1, cfg.max_height))
    if height >= cfg.squat_height_min:
        mode = cfg.squat_mode
    else:
        mode = cfg.kneel_mode

    return SonicPlannerCommand(
        mode=mode,
        movement=np.zeros(3, dtype=np.float64),
        facing=cmd.facing.copy(),
        speed=-1.0,
        height=height,
    )


def update_facing_from_head(
    head_pose: np.ndarray,
    state: HeadLocomotionState,
    cfg: HeadLocomotionConfig,
    *,
    robot_base_quat: np.ndarray | None = None,
) -> np.ndarray:
    calib_rot = _calib_rot_or_yaw(state)
    target_facing = wrap_to_pi(
        horizontal_heading_from_calib_rot(head_pose[:3, :3])
        - horizontal_heading_from_calib_rot(calib_rot)
    )
    fac_alpha = float(np.clip(cfg.facing_smooth_alpha, 0.05, 1.0))
    state.facing_angle = wrap_to_pi(
        state.facing_angle + fac_alpha * wrap_to_pi(target_facing - state.facing_angle)
    )
    facing_angle, imu_debug = apply_imu_yaw_closed_loop(
        state.facing_angle,
        state,
        cfg,
        robot_base_quat,
    )
    state.debug["imu"] = imu_debug
    return _facing_from_angle(facing_angle)


def _facing_from_angle(angle: float) -> np.ndarray:
    return np.array([np.cos(angle), np.sin(angle), 0.0], dtype=np.float64)


def reset_head_locomotion_state(
    state: HeadLocomotionState,
    *,
    calib_yaw: float | None = None,
    calib_rot: np.ndarray | None = None,
) -> None:
    state.prev_head_pos = None
    state.prev_head_yaw = None
    state.smooth_vx = 0.0
    state.smooth_vy = 0.0
    state.smooth_vyaw = 0.0
    state.smooth_pelvis_height = -1.0
    state.calibrated = False
    state.debug = {}
    state.robot_base_yaw_at_calib = None
    if calib_rot is not None:
        state.calib_rot = np.asarray(calib_rot, dtype=np.float64)[:3, :3].copy()
        state.calib_yaw = horizontal_heading_from_calib_rot(state.calib_rot)
        state.facing_angle = 0.0
    elif calib_yaw is not None:
        state.calib_yaw = float(calib_yaw)
        state.calib_rot = rotation_z(state.calib_yaw)
        state.facing_angle = 0.0
