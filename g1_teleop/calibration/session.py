"""AVP ↔ SONIC staged calibration (PICO VR_3PT aligned).

Phases:
  F  CALIB_FULL  — operator forearms-forward vs robot init pose
  ]  ENGAGE      — start policy, stand hold
  S  CALIB_SYNC  — wrists vs current robot (ZMQ feedback / optional FK)
  T  TELEOP      — live control (unified head/walk/squat zero)
  H  HEAD zero   — recalibrate facing + squat height only
  P  PAUSE       — freeze live mapping until realigned
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field

import msgpack
import numpy as np
import zmq
from scipy.spatial.transform import Rotation as R

from g1_teleop.transforms.frames import yaw_from_rot
from g1_teleop.locomotion.head import horizontal_heading_from_calib_rot


class CalibPhase(enum.Enum):
    WAIT_TRACKING = "wait_tracking"
    READY = "ready"
    FULL_HOLD = "full_hold"
    ENGAGED = "engaged"
    SYNC_HOLD = "sync_hold"
    HEAD_HOLD = "head_hold"
    TELEOP = "teleop"
    PAUSED = "paused"


@dataclass
class CalibQuality:
    frames: int = 0
    head_pos_std_m: float = 0.0
    left_pos_std_m: float = 0.0
    right_pos_std_m: float = 0.0
    passed: bool = False
    message: str = ""

    def summary(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return (
            f"{status} frames={self.frames} "
            f"head_std={self.head_pos_std_m * 1000:.1f}mm "
            f"L_std={self.left_pos_std_m * 1000:.1f}mm "
            f"R_std={self.right_pos_std_m * 1000:.1f}mm"
        )


@dataclass
class FeedbackSnapshot:
    body_q: np.ndarray | None = None
    vr_position: np.ndarray | None = None
    vr_orientation: np.ndarray | None = None
    base_quat: np.ndarray | None = None
    body_torso_quat: np.ndarray | None = None
    delta_heading: float | None = None
    monotonic_time: float = 0.0


@dataclass
class CalibSession:
    phase: CalibPhase = CalibPhase.WAIT_TRACKING
    full_done: bool = False
    sync_done: bool = False
    hold_deadline: float = 0.0
    hold_kind: str = ""
    paused: bool = False
    buffer: PoseFrameBuffer = field(default_factory=lambda: PoseFrameBuffer())
    last_quality: CalibQuality | None = None


@dataclass
class PoseFrameBuffer:
    head_positions: list[np.ndarray] = field(default_factory=list)
    head_rotations: list[np.ndarray] = field(default_factory=list)
    left_positions: list[np.ndarray] = field(default_factory=list)
    left_rotations: list[np.ndarray] = field(default_factory=list)
    right_positions: list[np.ndarray] = field(default_factory=list)
    right_rotations: list[np.ndarray] = field(default_factory=list)

    def clear(self) -> None:
        self.head_positions.clear()
        self.head_rotations.clear()
        self.left_positions.clear()
        self.left_rotations.clear()
        self.right_positions.clear()
        self.right_rotations.clear()

    def add(
        self,
        head_pose: np.ndarray,
        left_pose: np.ndarray | None,
        right_pose: np.ndarray | None,
        *,
        head_only: bool = False,
    ) -> None:
        self.head_positions.append(head_pose[:3, 3].copy())
        self.head_rotations.append(head_pose[:3, :3].copy())
        if head_only:
            return
        if left_pose is not None:
            self.left_positions.append(left_pose[:3, 3].copy())
            self.left_rotations.append(left_pose[:3, :3].copy())
        if right_pose is not None:
            self.right_positions.append(right_pose[:3, 3].copy())
            self.right_rotations.append(right_pose[:3, :3].copy())

    def _mean_pos(self, positions: list[np.ndarray]) -> np.ndarray:
        return np.mean(np.stack(positions, axis=0), axis=0)

    def _mean_rot(self, rotations: list[np.ndarray]) -> np.ndarray:
        if not rotations:
            return np.eye(3, dtype=np.float64)
        return R.from_matrix(np.stack(rotations, axis=0)).mean().as_matrix()

    def mean_head_pose(self) -> np.ndarray:
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = self._mean_rot(self.head_rotations)
        pose[:3, 3] = self._mean_pos(self.head_positions)
        return pose

    def mean_left_pose(self) -> np.ndarray | None:
        if not self.left_positions:
            return None
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = self._mean_rot(self.left_rotations)
        pose[:3, 3] = self._mean_pos(self.left_positions)
        return pose

    def mean_right_pose(self) -> np.ndarray | None:
        if not self.right_positions:
            return None
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = self._mean_rot(self.right_rotations)
        pose[:3, 3] = self._mean_pos(self.right_positions)
        return pose

    def quality(
        self,
        *,
        min_frames: int,
        max_head_std_m: float,
        max_wrist_std_m: float,
        require_left: bool,
        require_right: bool,
    ) -> CalibQuality:
        frames = len(self.head_positions)
        if frames < min_frames:
            return CalibQuality(
                frames=frames,
                passed=False,
                message=f"need >= {min_frames} frames, got {frames}",
            )

        head_std = float(np.linalg.norm(np.std(np.stack(self.head_positions, axis=0), axis=0)))
        left_std = 0.0
        right_std = 0.0
        if self.left_positions:
            left_std = float(np.linalg.norm(np.std(np.stack(self.left_positions, axis=0), axis=0)))
        elif require_left:
            return CalibQuality(frames=frames, passed=False, message="missing left wrist samples")
        if self.right_positions:
            right_std = float(np.linalg.norm(np.std(np.stack(self.right_positions, axis=0), axis=0)))
        elif require_right:
            return CalibQuality(frames=frames, passed=False, message="missing right wrist samples")

        passed = head_std <= max_head_std_m
        if self.left_positions:
            passed = passed and left_std <= max_wrist_std_m
        if self.right_positions:
            passed = passed and right_std <= max_wrist_std_m

        msg = "ok" if passed else "motion too large during hold"
        return CalibQuality(
            frames=frames,
            head_pos_std_m=head_std,
            left_pos_std_m=left_std,
            right_pos_std_m=right_std,
            passed=passed,
            message=msg,
        )


class ZmqFeedbackClient:
    """Subscribe to SONIC deploy ZMQ PUB (default g1_debug @ :5557)."""

    def __init__(self, host: str, port: int, topic: str = "g1_debug"):
        self.topic_bytes = topic.encode("utf-8")
        self._last: FeedbackSnapshot | None = None
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.RCVHWM, 3)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._socket.connect(f"tcp://{host}:{port}")

    def close(self) -> None:
        self._socket.close(0)
        self._context.term()

    @property
    def has_data(self) -> bool:
        return self._last is not None

    def latest(self) -> FeedbackSnapshot | None:
        """Return the most recent feedback snapshot (polls the socket first)."""
        self.poll()
        return self._last

    def poll(self) -> FeedbackSnapshot | None:
        while True:
            try:
                message = self._socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                return self._last
            snap = self._parse_message(message)
            if snap is not None:
                self._last = snap

    def _parse_message(self, message: bytes) -> FeedbackSnapshot | None:
        if not message.startswith(self.topic_bytes):
            return None
        payload = message[len(self.topic_bytes) :]
        try:
            data = msgpack.unpackb(payload, raw=False)
        except Exception:
            return None
        if not isinstance(data, dict):
            return None

        body_q = None
        for key in ("body_q_measured", "body_q"):
            if key in data:
                body_q = np.asarray(data[key], dtype=np.float64).reshape(-1)
                break

        vr_pos = None
        if "vr_3point_position" in data:
            vr_pos = np.asarray(data["vr_3point_position"], dtype=np.float64).reshape(9)
        vr_orn = None
        if "vr_3point_orientation" in data:
            vr_orn = np.asarray(data["vr_3point_orientation"], dtype=np.float64).reshape(12)

        base_quat = None
        for key in ("base_quat_measured", "base_quat"):
            if key in data:
                base_quat = np.asarray(data[key], dtype=np.float64).reshape(4)
                break

        body_torso_quat = None
        if "body_torso_quat" in data:
            body_torso_quat = np.asarray(data["body_torso_quat"], dtype=np.float64).reshape(4)

        delta_heading = None
        if "delta_heading" in data:
            delta_heading = float(data["delta_heading"])

        return FeedbackSnapshot(
            body_q=body_q,
            vr_position=vr_pos,
            vr_orientation=vr_orn,
            base_quat=base_quat,
            body_torso_quat=body_torso_quat,
            delta_heading=delta_heading,
            monotonic_time=time.monotonic(),
        )


class FkRobotReference:
    """Optional G1 FK wrist/neck reference from measured body_q."""

    def __init__(self):
        self._robot_model = None
        self._get_poses = None
        self._load_error: str | None = None
        try:
            groot_root = None
            import os

            for candidate in (
                os.environ.get("GR00T_WBC_ROOT"),
                "/mnt/newssd/GR00T-WholeBodyControl",
            ):
                if candidate and os.path.isdir(candidate):
                    groot_root = candidate
                    break
            if groot_root:
                import sys

                if groot_root not in sys.path:
                    sys.path.insert(0, groot_root)
            from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
            from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import get_g1_key_frame_poses

            self._robot_model = instantiate_g1_robot_model()
            self._get_poses = get_g1_key_frame_poses
        except Exception as exc:
            self._load_error = str(exc)

    @property
    def available(self) -> bool:
        return self._robot_model is not None and self._get_poses is not None

    def vr_targets_from_body_q(self, body_q: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        if not self.available:
            return None
        q = np.asarray(body_q, dtype=np.float64).reshape(-1)
        if q.size < 29:
            return None
        q = q[:29]
        try:
            cfg = self._robot_model.get_configuration_from_actuated_joints(body_actuated_joint_values=q)
            poses = self._get_poses(self._robot_model, q=cfg)
        except Exception:
            return None

        # Order: left wrist, right wrist, head (torso + offset approximates neck)
        positions = np.zeros(9, dtype=np.float64)
        orientations = np.zeros(12, dtype=np.float64)
        for idx, key in enumerate(("left_wrist", "right_wrist", "torso")):
            positions[idx * 3 : (idx + 1) * 3] = poses[key]["position"]
            orientations[idx * 4 : (idx + 1) * 4] = poses[key]["orientation_wxyz"]
        return positions, orientations


def robot_base_from_feedback(
    feedback: FeedbackSnapshot | None,
    fk_ref: FkRobotReference | None,
    *,
    fallback_positions: np.ndarray,
    fallback_orientations: np.ndarray,
    prefer_fk: bool = True,
    last_sent_positions: np.ndarray | None = None,
    last_sent_orientations: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, str]:
    """Return (positions9, orientations12, source_label)."""
    if (
        prefer_fk
        and fk_ref is not None
        and feedback is not None
        and feedback.body_q is not None
    ):
        fk_targets = fk_ref.vr_targets_from_body_q(feedback.body_q)
        if fk_targets is not None:
            return fk_targets[0].copy(), fk_targets[1].copy(), "fk(body_q)"

    if (
        last_sent_positions is not None
        and last_sent_orientations is not None
    ):
        return (
            np.asarray(last_sent_positions, dtype=np.float64).reshape(9).copy(),
            np.asarray(last_sent_orientations, dtype=np.float64).reshape(12).copy(),
            "last_sent_command",
        )

    if feedback is not None and feedback.vr_position is not None and feedback.vr_orientation is not None:
        return (
            feedback.vr_position.copy(),
            feedback.vr_orientation.copy(),
            "vr_3point_feedback",
        )

    return fallback_positions.copy(), fallback_orientations.copy(), "fallback_init"


def build_official_calibration_from_poses(
    head_pose: np.ndarray,
    left_pose: np.ndarray | None,
    right_pose: np.ndarray | None,
    *,
    head_yaw_compensated_relative,
    head_yaw_compensated_rotation,
    rotmat_to_quat_wxyz,
) -> object:
    """Build OfficialCalibration-like object (duck typed for avp_to_sonic_zmq)."""

    @dataclass
    class OfficialCalibration:
        head_pose: np.ndarray
        head_rotation: np.ndarray
        left_rel: np.ndarray | None
        right_rel: np.ndarray | None
        left_orientation: np.ndarray | None
        right_orientation: np.ndarray | None
        left_rotation: np.ndarray | None
        right_rotation: np.ndarray | None

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


def finalize_pose_buffer(
    buffer: PoseFrameBuffer,
    *,
    min_frames: int,
    max_head_std_m: float,
    max_wrist_std_m: float,
    require_left: bool,
    require_right: bool,
    head_only: bool = False,
    build_calibration_fn,
) -> tuple[object | None, CalibQuality]:
    quality = buffer.quality(
        min_frames=min_frames,
        max_head_std_m=max_head_std_m,
        max_wrist_std_m=max_wrist_std_m,
        require_left=require_left and not head_only,
        require_right=require_right and not head_only,
    )
    if not quality.passed:
        return None, quality

    head_pose = buffer.mean_head_pose()
    left_pose = None if head_only else buffer.mean_left_pose()
    right_pose = None if head_only else buffer.mean_right_pose()
    calib = build_calibration_fn(head_pose, left_pose, right_pose)
    return calib, quality


def merge_calibration(
    base,
    update,
    *,
    preserve_head: bool = False,
    preserve_wrists: bool = False,
):
    if preserve_head:
        return type(base)(
            head_pose=base.head_pose.copy(),
            head_rotation=base.head_rotation.copy(),
            left_rel=base.left_rel if preserve_wrists else update.left_rel,
            right_rel=base.right_rel if preserve_wrists else update.right_rel,
            left_orientation=base.left_orientation if preserve_wrists else update.left_orientation,
            right_orientation=base.right_orientation if preserve_wrists else update.right_orientation,
            left_rotation=base.left_rotation if preserve_wrists else update.left_rotation,
            right_rotation=base.right_rotation if preserve_wrists else update.right_rotation,
        )
    if preserve_wrists:
        return type(base)(
            head_pose=update.head_pose.copy(),
            head_rotation=update.head_rotation.copy(),
            left_rel=base.left_rel.copy() if base.left_rel is not None else None,
            right_rel=base.right_rel.copy() if base.right_rel is not None else None,
            left_orientation=base.left_orientation,
            right_orientation=base.right_orientation,
            left_rotation=base.left_rotation,
            right_rotation=base.right_rotation,
        )
    return update


def sync_loco_zeros(
    head_pose: np.ndarray,
    head_loco_state,
    reset_head_locomotion_state,
    *,
    robot_base_yaw: float | None = None,
) -> tuple[np.ndarray, np.ndarray, float]:
    calib_pos = head_pose[:3, 3].copy()
    calib_rot = head_pose[:3, :3].copy()
    calib_yaw = horizontal_heading_from_calib_rot(calib_rot)
    reset_head_locomotion_state(head_loco_state, calib_rot=calib_rot)
    if robot_base_yaw is not None:
        head_loco_state.robot_base_yaw_at_calib = float(robot_base_yaw)
    return calib_pos, calib_rot, calib_yaw


def arm_hold(session: CalibSession, kind: str, hold_sec: float) -> None:
    session.buffer.clear()
    session.hold_kind = kind
    session.hold_deadline = time.time() + max(hold_sec, 0.0)
    if kind == "full":
        session.phase = CalibPhase.FULL_HOLD
    elif kind == "sync":
        session.phase = CalibPhase.SYNC_HOLD
    elif kind == "head":
        session.phase = CalibPhase.HEAD_HOLD


def collect_hold_sample(
    session: CalibSession,
    head_pose: np.ndarray,
    left_pose: np.ndarray | None,
    right_pose: np.ndarray | None,
) -> None:
    if session.phase == CalibPhase.HEAD_HOLD:
        session.buffer.add(head_pose, left_pose, right_pose, head_only=True)
    else:
        session.buffer.add(head_pose, left_pose, right_pose)
