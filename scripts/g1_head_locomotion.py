"""Head-driven locomotion for G1 teleop (work-master aligned).

Isaac sim: velocity commands on rt/run_command/cmd.
SONIC planner: movement/facing command vectors for legged walking policy.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

from g1_avp_transforms import avp_to_robot, openxr_to_robot, rotation_z, yaw_from_rot


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
    lateral_displacement_gain: float = 2.2
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


@dataclass
class HeadLocomotionState:
    prev_head_pos: np.ndarray | None = None
    prev_head_yaw: float | None = None
    smooth_vx: float = 0.0
    smooth_vy: float = 0.0
    smooth_vyaw: float = 0.0
    facing_angle: float = 0.0
    calib_yaw: float = 0.0
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


def compute_head_locomotion_velocity(
    head_pose: np.ndarray,
    state: HeadLocomotionState,
    cfg: HeadLocomotionConfig,
    dt: float,
    *,
    calib_pos: np.ndarray,
    calib_rot: np.ndarray | None = None,
) -> tuple[float, float, float]:
    """Return (vx, vy, vyaw) in robot-base frame from head motion derivative."""
    pos = head_pose[:3, 3]
    yaw = yaw_from_rot(head_pose[:3, :3])

    if state.prev_head_pos is None:
        state.prev_head_pos = pos.copy()
        state.prev_head_yaw = yaw
        state.calibrated = True
        state.debug = {
            "delta_body": [0.0, 0.0, 0.0],
            "raw_vel": [0.0, 0.0],
            "vy_disp": 0.0,
            "raw_vyaw": 0.0,
        }
        return 0.0, 0.0, 0.0

    vel_world = (pos - state.prev_head_pos) / max(dt, 1e-6)
    yaw_rate = wrap_to_pi(yaw - state.prev_head_yaw) / max(dt, 1e-6)
    state.prev_head_pos = pos.copy()
    state.prev_head_yaw = yaw

    vel_local = rotation_z(-yaw) @ vel_world
    vx = cfg.velocity_gain * cfg.forward_scale * cfg.sign_x * float(vel_local[0])
    vy = cfg.velocity_gain * cfg.lateral_scale * cfg.sign_y * float(vel_local[1])
    vy_disp = 0.0
    lat_disp = 0.0
    strafe_intent = False

    if calib_rot is not None:
        delta_body = head_delta_calib_frame(head_pose, calib_pos, calib_rot)
        lat_disp = float(np.clip(delta_body[0], -cfg.max_lateral_displacement, cfg.max_lateral_displacement))
        fwd_disp = abs(float(delta_body[2]))
        strafe_intent = abs(lat_disp) >= max(cfg.lateral_strafe_min, fwd_disp * cfg.lateral_axis_ratio)
        if strafe_intent:
            vy_disp = cfg.lateral_displacement_gain * cfg.sign_y * lat_disp
            vy += vy_disp
    else:
        delta_body = head_delta_yaw_compensated(head_pose, calib_pos)

    if vy > 0.0:
        vy *= cfg.lateral_left_scale
    elif vy < 0.0:
        vy *= cfg.lateral_right_scale

    # Drop weak lateral coupling during forward/back, but keep strafe when head shifts sideways.
    if (
        not strafe_intent
        and abs(vx) >= cfg.velocity_deadzone
        and abs(vy) < cfg.lateral_coupling_suppress
    ):
        vy = 0.0

    vyaw = cfg.yaw_rate_gain * yaw_rate

    if abs(vx) < cfg.velocity_deadzone:
        vx = 0.0
    lat_dz = cfg.lateral_velocity_deadzone
    if abs(vy) < lat_dz:
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
    lat_out_dz = float(getattr(cfg, "lateral_output_deadzone", out_dz))
    if abs(out_vx) < out_dz:
        out_vx = 0.0
        state.smooth_vx = 0.0
    if abs(out_vy) < lat_out_dz:
        out_vy = 0.0
        state.smooth_vy = 0.0
    if abs(out_vyaw) < out_dz:
        out_vyaw = 0.0
        state.smooth_vyaw = 0.0

    state.debug = {
        "delta_body": np.round(delta_body, 3).tolist(),
        "raw_vel": np.round([vx, vy], 3).tolist(),
        "vy_disp": round(vy_disp, 3),
        "raw_vyaw": round(float(yaw_rate), 3),
        "cmd": np.round([out_vx, out_vy, out_vyaw], 3).tolist(),
        "lat_disp": round(lat_disp, 3),
        "strafe_intent": strafe_intent,
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
    calib_rot: np.ndarray | None = None,
    locomotion_mode: int = 1,
) -> SonicPlannerCommand:
    """Map head motion to SONIC planner movement/facing vectors.

    SONIC expects movement (travel direction, can be backward) separately from
    facing (body/head heading). Do NOT set facing from arctan2(vy, vx) or the
    robot turns around instead of walking backward.
    """
    head_yaw = yaw_from_rot(head_pose[:3, :3])
    target_facing = wrap_to_pi(head_yaw - state.calib_yaw)
    fac_alpha = float(np.clip(cfg.facing_smooth_alpha, 0.05, 1.0))
    state.facing_angle = wrap_to_pi(
        state.facing_angle + fac_alpha * wrap_to_pi(target_facing - state.facing_angle)
    )
    facing = _facing_from_angle(state.facing_angle)

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
        calib_rot=calib_rot,
    )

    cos_y = np.cos(head_yaw)
    sin_y = np.sin(head_yaw)
    wx = cos_y * vx - sin_y * vy
    wy = sin_y * vx + cos_y * vy
    local_speed = float(np.hypot(vx, vy))
    lat_dz = float(getattr(cfg, "lateral_velocity_deadzone", cfg.velocity_deadzone))

    forward_active = abs(vx) >= cfg.velocity_deadzone
    lateral_active = abs(vy) >= lat_dz
    if (forward_active or lateral_active) and local_speed > 1e-6:
        movement = np.array([wx / local_speed, wy / local_speed, 0.0], dtype=np.float64)
        return SonicPlannerCommand(
            mode=locomotion_mode,
            movement=movement,
            facing=facing,
            speed=min(local_speed, cfg.max_speed),
        )

    return idle


def _facing_from_angle(angle: float) -> np.ndarray:
    return np.array([np.cos(angle), np.sin(angle), 0.0], dtype=np.float64)


def reset_head_locomotion_state(
    state: HeadLocomotionState,
    *,
    calib_yaw: float | None = None,
) -> None:
    state.prev_head_pos = None
    state.prev_head_yaw = None
    state.smooth_vx = 0.0
    state.smooth_vy = 0.0
    state.smooth_vyaw = 0.0
    state.calibrated = False
    state.debug = {}
    if calib_yaw is not None:
        state.calib_yaw = float(calib_yaw)
        state.facing_angle = 0.0
