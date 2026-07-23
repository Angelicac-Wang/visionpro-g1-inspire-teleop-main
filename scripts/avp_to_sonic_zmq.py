#!/usr/bin/env python3
"""Bridge Apple Vision Pro tracking into SONIC's ZMQ manager input.

This script reads AVP tracking and publishes SONIC ZMQ packed messages. For
MuJoCo testing it can publish sim-only Inspire hand commands over ZMQ; for
real-robot tests it can also publish right-hand commands over Unitree DDS to
the Headless_driver_r.py Inspire hand bridge.
"""

import argparse
import json
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass

import numpy as np
import zmq


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from g1_head_locomotion import (
    HeadLocomotionConfig,
    HeadLocomotionState,
    compute_sonic_planner_command,
    head_pose_from_tracking,
    reset_head_locomotion_state,
)
UNITREE_SIM_ROOT = os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")
INSPIRE_HAND_SDK_ROOT = os.environ.get(
    "INSPIRE_HAND_SDK_ROOT",
    os.path.join(UNITREE_SIM_ROOT, "inspire_hand_ws", "inspire_hand_sdk"),
)
VISIONPRO_TELEOP_ROOT = os.environ.get(
    "VISIONPRO_TELEOP_ROOT",
    os.path.join(UNITREE_SIM_ROOT, "inspire_hand_ws", "VisionProTeleop"),
)
if VISIONPRO_TELEOP_ROOT not in sys.path:
    sys.path.insert(0, VISIONPRO_TELEOP_ROOT)

from avp_stream import VisionProStreamer
from avp_inspire_hand_mapping import InspireHandMapper, format_debug as format_hand_debug


HEADER_SIZE = 1280

YUP2ZUP = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
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
T_TO_UNITREE_HUMANOID_LEFT_ARM = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_TO_UNITREE_HUMANOID_RIGHT_ARM = np.array(
    [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
INV_YUP2ZUP = np.linalg.inv(YUP2ZUP)


class PackedPublisher:
    def __init__(self, host: str, port: int, connect_delay: float = 0.5):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.endpoint = f"tcp://{host}:{port}"
        self.socket.bind(self.endpoint)
        time.sleep(connect_delay)

    def close(self):
        self.socket.close(0)
        self.context.term()

    def _send(self, topic: str, fields: list[dict], payload: bytes):
        header = {
            "v": 1,
            "endian": "le",
            "count": 1,
            "fields": fields,
        }
        header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
        if len(header_json) > HEADER_SIZE:
            raise ValueError(f"ZMQ header too large: {len(header_json)} > {HEADER_SIZE}")
        message = topic.encode("utf-8") + header_json + b"\0" * (HEADER_SIZE - len(header_json)) + payload
        self.socket.send(message)

    def send_command(self, start: bool, stop: bool, planner: bool):
        fields = [
            {"name": "start", "dtype": "u8", "shape": [1]},
            {"name": "stop", "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ]
        payload = struct.pack("BBB", int(start), int(stop), int(planner))
        self._send("command", fields, payload)

    def send_planner(
        self,
        mode: int,
        movement: np.ndarray,
        facing: np.ndarray,
        speed: float,
        height: float,
        vr_position: np.ndarray,
        vr_orientation: np.ndarray,
        left_wrist_joints: np.ndarray | None = None,
        right_wrist_joints: np.ndarray | None = None,
    ):
        fields = [
            {"name": "mode", "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing", "dtype": "f32", "shape": [3]},
            {"name": "speed", "dtype": "f32", "shape": [1]},
            {"name": "height", "dtype": "f32", "shape": [1]},
            {"name": "vr_position", "dtype": "f32", "shape": [9]},
            {"name": "vr_orientation", "dtype": "f32", "shape": [12]},
        ]
        payload_parts = [
            struct.pack("<i", int(mode)),
            np.asarray(movement, dtype=np.float32).reshape(3).tobytes(),
            np.asarray(facing, dtype=np.float32).reshape(3).tobytes(),
            struct.pack("<f", float(speed)),
            struct.pack("<f", float(height)),
            np.asarray(vr_position, dtype=np.float32).reshape(9).tobytes(),
            np.asarray(vr_orientation, dtype=np.float32).reshape(12).tobytes(),
        ]
        if left_wrist_joints is not None:
            fields.append({"name": "left_wrist_joints", "dtype": "f32", "shape": [3]})
            payload_parts.append(np.asarray(left_wrist_joints, dtype=np.float32).reshape(3).tobytes())
        if right_wrist_joints is not None:
            fields.append({"name": "right_wrist_joints", "dtype": "f32", "shape": [3]})
            payload_parts.append(np.asarray(right_wrist_joints, dtype=np.float32).reshape(3).tobytes())
        fields.append({"name": "timestamp_monotonic", "dtype": "f64", "shape": [1]})
        payload_parts.append(struct.pack("<d", time.monotonic()))
        payload = b"".join(payload_parts)
        self._send("planner", fields, payload)

    def send_inspire_hand(self, left: np.ndarray, right: np.ndarray, topic: str = "inspire_hand"):
        fields = [
            {"name": "left", "dtype": "f32", "shape": [6]},
            {"name": "right", "dtype": "f32", "shape": [6]},
            {"name": "timestamp_monotonic", "dtype": "f64", "shape": [1]},
        ]
        payload = b"".join(
            [
                np.asarray(left, dtype=np.float32).reshape(6).tobytes(),
                np.asarray(right, dtype=np.float32).reshape(6).tobytes(),
                struct.pack("<d", time.monotonic()),
            ]
        )
        self._send(topic, fields, payload)


class DdsInspireHandPublisher:
    def __init__(self, topic_side: str, dds_network: str | None):
        unitree_sdk2_root = os.environ.get(
            "UNITREE_SDK2_ROOT",
            os.path.join(UNITREE_SIM_ROOT, "unitree_sdk2_python"),
        )
        for path in (unitree_sdk2_root, INSPIRE_HAND_SDK_ROOT):
            if path not in sys.path:
                sys.path.insert(0, path)

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from inspire_sdkpy import inspire_dds, inspire_hand_defaut

        if dds_network:
            ChannelFactoryInitialize(0, dds_network)
        else:
            ChannelFactoryInitialize(0)

        self._hand_default = inspire_hand_defaut
        self._publisher = ChannelPublisher(
            f"rt/inspire_hand/ctrl/{topic_side}",
            inspire_dds.inspire_hand_ctrl,
        )
        self._publisher.Init()

    def send(self, values: np.ndarray):
        cmd = self._hand_default.get_inspire_hand_ctrl()
        cmd.angle_set = [int(v) for v in np.asarray(values, dtype=np.int16).tolist()]
        cmd.mode = 0b0001
        return self._publisher.Write(cmd)


class RawKeyboard:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_keys(self) -> list[str]:
        keys = []
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return keys
            keys.append(sys.stdin.read(1))


@dataclass
class BridgeState:
    mode: int = 0
    movement: np.ndarray = None
    facing_angle: float = 0.0
    speed: float = -1.0
    height: float = -1.0

    def __post_init__(self):
        if self.movement is None:
            self.movement = np.zeros(3, dtype=np.float64)

    @property
    def facing(self) -> np.ndarray:
        return np.array([np.cos(self.facing_angle), np.sin(self.facing_angle), 0.0], dtype=np.float64)

    def set_idle(self):
        self.mode = 0
        self.movement[:] = 0.0
        self.speed = -1.0
        self.height = -1.0

    def set_mode(self, mode: int):
        self.mode = int(mode)
        self.speed = -1.0
        self.height = -1.0


class PositionSmoother:
    def __init__(self):
        self.position = None
        self.last_time = None

    def reset(self):
        self.position = None
        self.last_time = None

    def update(self, target: np.ndarray, now: float, tau: float, max_speed: float) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64).reshape(9)
        if self.position is None or self.last_time is None:
            self.position = target.copy()
            self.last_time = now
            return self.position.copy()

        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        alpha = 1.0 if tau <= 1e-6 else 1.0 - np.exp(-dt / tau)
        proposed = self.position + alpha * (target - self.position)

        if max_speed > 0.0:
            for i in range(3):
                start = i * 3
                delta = proposed[start : start + 3] - self.position[start : start + 3]
                norm = float(np.linalg.norm(delta))
                max_delta = max_speed * dt
                if norm > max_delta > 0.0:
                    proposed[start : start + 3] = self.position[start : start + 3] + delta * (max_delta / norm)

        self.position = proposed
        return self.position.copy()


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


def rotmat_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    trace = np.trace(r)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def scale_rotation(rot: np.ndarray, scale: float) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64)[:3, :3]
    if abs(scale - 1.0) < 1e-6:
        return r

    cos_angle = np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))
    if angle < 1e-8:
        return np.eye(3, dtype=np.float64)

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
        return np.eye(3, dtype=np.float64)
    axis = axis / axis_norm

    scaled_angle = angle * scale
    kx = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + np.sin(scaled_angle) * kx + (1.0 - np.cos(scaled_angle)) * (kx @ kx)


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


def head_yaw_compensated_relative(head_pose: np.ndarray, wrist_pose: np.ndarray) -> np.ndarray:
    """Official Pico-style wrist target: wrist relative to head with headset yaw removed."""
    inverse_head_yaw = rotation_z(-yaw_from_rot(head_pose[:3, :3]))
    return inverse_head_yaw @ (wrist_pose[:3, 3] - head_pose[:3, 3])


def head_yaw_compensated_rotation(head_pose: np.ndarray, wrist_pose: np.ndarray) -> np.ndarray:
    inverse_head_yaw = rotation_z(-yaw_from_rot(head_pose[:3, :3]))
    return inverse_head_yaw @ wrist_pose[:3, :3]


def capture_official_calibration(tracking, args=None) -> OfficialCalibration | None:
    if tracking is None or tracking.head is None:
        return None

    head_pose = avp_to_robot(tracking.head)
    left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM, args)
    right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM, args)

    return OfficialCalibration(
        head_pose=head_pose,
        head_rotation=head_pose[:3, :3].copy(),
        left_rel=None if left_pose is None else head_yaw_compensated_relative(head_pose, left_pose),
        right_rel=None if right_pose is None else head_yaw_compensated_relative(head_pose, right_pose),
        left_orientation=None if left_pose is None else rotmat_to_quat_wxyz(left_pose[:3, :3]),
        right_orientation=None if right_pose is None else rotmat_to_quat_wxyz(right_pose[:3, :3]),
        left_rotation=None if left_pose is None else head_yaw_compensated_rotation(head_pose, left_pose),
        right_rotation=None if right_pose is None else head_yaw_compensated_rotation(head_pose, right_pose),
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


def official_delta_axis_sign(args) -> np.ndarray:
    return np.array(
        [args.official_delta_sign_x, args.official_delta_sign_y, args.official_delta_sign_z],
        dtype=np.float64,
    )


def apply_hand_workspace_shape(pos: np.ndarray, args) -> np.ndarray:
    shaped = np.asarray(pos, dtype=np.float64).copy()
    shaped[0] = np.clip(shaped[0], args.min_hand_x, args.max_hand_x)
    return shaped


def official_hand_delta(rel: np.ndarray, calib_rel: np.ndarray, args) -> np.ndarray:
    signed_delta = official_delta_axis_sign(args) * (rel - calib_rel)
    signed_delta[0] *= args.hand_forward_scale if signed_delta[0] >= 0.0 else args.hand_backward_scale
    return args.body_scale * signed_delta


def wrist_axis_remap_matrix(args) -> np.ndarray:
    remap = args.wrist_axis_remap
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


def remap_wrist_rotation_delta(rotation_delta: np.ndarray, args) -> np.ndarray:
    basis = wrist_axis_remap_matrix(args)
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
    rotation_delta = remap_wrist_rotation_delta(rotation_delta, args)
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
        yaw_delta = yaw_from_rot(head_pose[:3, :3]) - yaw_from_rot(calibration.head_rotation)
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
    rotation_delta = remap_wrist_rotation_delta(rotation_delta, args)
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
    left_wrist_joints = np.zeros(3, dtype=np.float64) if args.wrist_orientation_mode == "wrist-joints" else None
    right_wrist_joints = np.zeros(3, dtype=np.float64) if args.wrist_orientation_mode == "wrist-joints" else None

    for side, pose, calib_rel, calib_orientation, calib_rotation, base in (
        ("left", left_pose, calibration.left_rel, calibration.left_orientation, calibration.left_rotation, left_base),
        ("right", right_pose, calibration.right_rel, calibration.right_orientation, calibration.right_rotation, right_base),
    ):
        base_orn = left_base_orn if side == "left" else right_base_orn
        if force_base:
            pos = base
            orn = base_orn
        elif pose is not None and calib_rel is not None:
            rel = head_yaw_compensated_relative(head_pose, pose)
            delta = official_hand_delta(rel, calib_rel, args)
            pos = apply_hand_workspace_shape(base + delta, args)
            debug[f"{side}_rel"] = rel.copy()
            debug[f"{side}_delta"] = delta.copy()
            if args.wrist_orientation_mode == "live":
                orn = rotmat_to_quat_wxyz(pose[:3, :3])
            elif args.wrist_orientation_mode == "calibrated":
                orn = calibrated_wrist_orientation(head_pose, pose, calib_rotation, base_orn, side, args)
            elif args.wrist_orientation_mode == "wrist-joints":
                orn = base_orn
                wrist_joints = calibrated_wrist_joints(head_pose, pose, calib_rotation, side, args)
                if side == "left":
                    left_wrist_joints = wrist_joints
                else:
                    right_wrist_joints = wrist_joints
            elif args.wrist_orientation_mode == "neutral":
                orn = base_orn
            else:
                orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            pos = base
            orn = base_orn
        positions.append(clamp_vec(pos, limits))
        orientations.append(orn)

    if force_base or args.lock_head_translation:
        head_delta = np.zeros(3, dtype=np.float64)
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


def handle_key(state: BridgeState, key: str, *, head_locomotion: bool = False) -> bool:
    facing = state.facing
    if key == "0":
        state.set_idle()
    elif key == "1":
        state.set_mode(1)
    elif key == "2":
        state.set_mode(2)
    elif key == "3":
        state.set_mode(3)
    elif key in (" ", "r", "R"):
        state.movement[:] = 0.0
    elif head_locomotion:
        pass
    elif key in ("j", "J", "q", "Q"):
        state.facing_angle += np.pi / 12.0
    elif key in ("l", "L", "e", "E"):
        state.facing_angle -= np.pi / 12.0
    elif key in ("w", "W"):
        state.movement[:] = facing
        if state.mode == 0:
            state.set_mode(1)
    elif key in ("s", "S"):
        state.movement[:] = -facing
        if state.mode == 0:
            state.set_mode(1)
    elif key in ("a", "A"):
        state.facing_angle += 0.1
        state.movement[:] = state.facing
        if state.mode == 0:
            state.set_mode(1)
    elif key in ("d", "D"):
        state.facing_angle -= 0.1
        state.movement[:] = state.facing
        if state.mode == 0:
            state.set_mode(1)
    elif key == ",":
        state.movement[:] = np.array([-np.sin(state.facing_angle), np.cos(state.facing_angle), 0.0])
        if state.mode == 0:
            state.set_mode(1)
    elif key == ".":
        state.movement[:] = np.array([np.sin(state.facing_angle), -np.cos(state.facing_angle), 0.0])
        if state.mode == 0:
            state.set_mode(1)
    elif key in ("o", "O", "\x03"):
        return False
    elif key in ("t", "T"):
        return True
    return True


def default_hand_calibration_file() -> str:
    repo_path = os.path.join(SCRIPT_DIR, "visionpro_right_hand_calibration.json")
    sdk_example_path = os.path.join(INSPIRE_HAND_SDK_ROOT, "example", "visionpro_right_hand_calibration.json")
    if os.path.exists(repo_path):
        return repo_path
    if os.path.exists(sdk_example_path):
        return sdk_example_path
    return repo_path


def parse_args():
    parser = argparse.ArgumentParser(description="Send AVP tracking to SONIC ZMQManager.")
    parser.add_argument("--avp-endpoint", default="192.168.2.45", help="Vision Pro IP or room code.")
    parser.add_argument("--host", default="*", help="ZMQ PUB bind host. Use '*' when SONIC uses --zmq-host localhost.")
    parser.add_argument("--port", type=int, default=5556, help="ZMQ PUB port.")
    parser.add_argument("--publish-rate", type=float, default=50.0)
    start_group = parser.add_mutually_exclusive_group()
    start_group.add_argument(
        "--no-auto-start",
        dest="no_auto_start",
        action="store_true",
        default=True,
        help="Do not send SONIC start command automatically.",
    )
    start_group.add_argument(
        "--auto-start",
        dest="no_auto_start",
        action="store_false",
        help="Send SONIC start commands automatically after AVP tracking is ready.",
    )
    parser.add_argument(
        "--active-hands",
        choices=("both", "left", "right"),
        default="both",
        help="Only track the selected hand(s); inactive hands are held at the configured robot init target.",
    )
    parser.add_argument("--head-to-waist-x", type=float, default=0.15)
    parser.add_argument("--head-to-waist-y", type=float, default=0.0)
    parser.add_argument("--head-to-waist-z", type=float, default=0.45)
    parser.add_argument("--head-position-scale", type=float, default=0.5)
    parser.add_argument("--hand-position-scale", type=float, default=0.8)
    parser.add_argument(
        "--mapping-mode",
        choices=("relative-zero", "head-relative", "hybrid", "official-calib"),
        default="official-calib",
        help=(
            "relative-zero maps startup hand pose to neutral targets; head-relative uses "
            "live wrist-head offsets; hybrid uses wrist-head offsets with smoothing; "
            "official-calib uses Pico-style headset-yaw-compensated calibration deltas."
        ),
    )
    parser.add_argument("--body-scale", type=float, default=1.0, help="Scale applied to wrist-head relative positions.")
    parser.add_argument("--head-relative-x-offset", type=float, default=0.0)
    parser.add_argument("--head-relative-y-offset", type=float, default=0.0)
    parser.add_argument("--head-relative-z-offset", type=float, default=0.0)
    parser.add_argument(
        "--hybrid-add-head-base-to-hands",
        action="store_true",
        help="Old hybrid behavior: add robot head base height to hand targets.",
    )
    parser.add_argument("--hybrid-smoothing-tau", type=float, default=0.04, help="Low-pass time constant for hybrid vr_position.")
    parser.add_argument("--hybrid-max-speed", type=float, default=0.45, help="Max target speed in m/s for each VR point in hybrid mode.")
    parser.add_argument(
        "--official-calib-base",
        choices=("current-target", "init-pose", "neutral"),
        default="init-pose",
        help=(
            "Robot-side base used when pressing c in official-calib mode. "
            "current-target avoids a jump by calibrating against the current published target; "
            "init-pose returns to the configured robot init pose. neutral is kept as an alias."
        ),
    )
    parser.add_argument(
        "--calib-hold-sec",
        type=float,
        default=1.5,
        help="Seconds to hold the fixed robot init target after pressing c in official-calib mode.",
    )
    head_translation_group = parser.add_mutually_exclusive_group()
    head_translation_group.add_argument(
        "--lock-head-translation",
        dest="lock_head_translation",
        action="store_true",
        default=True,
        help="Keep the robot head target xyz fixed after calibration; headset walking will not move the upper-body target.",
    )
    head_translation_group.add_argument(
        "--allow-head-translation",
        dest="lock_head_translation",
        action="store_false",
        help="Allow headset translation to move the robot head target.",
    )
    head_rotation_group = parser.add_mutually_exclusive_group()
    head_rotation_group.add_argument(
        "--head-yaw-only",
        dest="head_yaw_only",
        action="store_true",
        default=True,
        help="Track only headset yaw for the robot head target; ignore headset pitch/roll.",
    )
    head_rotation_group.add_argument(
        "--full-head-rotation",
        dest="head_yaw_only",
        action="store_false",
        help="Track headset yaw, pitch, and roll for the robot head target.",
    )
    parser.add_argument(
        "--official-delta-sign-x",
        type=float,
        default=-1.0,
        help="Axis sign for official-calib hand forward/back delta.",
    )
    parser.add_argument(
        "--official-delta-sign-y",
        type=float,
        default=-1.0,
        help="Axis sign for official-calib hand left/right delta.",
    )
    parser.add_argument(
        "--official-delta-sign-z",
        type=float,
        default=1.0,
        help="Axis sign for official-calib hand up/down delta.",
    )
    parser.add_argument(
        "--hand-forward-scale",
        type=float,
        default=1.55,
        help="Extra scale for positive robot-X hand motion in official-calib mode.",
    )
    parser.add_argument(
        "--hand-backward-scale",
        type=float,
        default=0.35,
        help="Extra scale for negative robot-X hand motion in official-calib mode.",
    )
    parser.add_argument(
        "--min-hand-x",
        type=float,
        default=0.20,
        help="Lower robot-X limit for hand targets after official-calib mapping.",
    )
    parser.add_argument(
        "--max-hand-x",
        type=float,
        default=0.88,
        help="Upper robot-X limit for hand targets after official-calib mapping.",
    )
    parser.add_argument(
        "--robot-init-pose",
        choices=("forearms-forward", "debug-ready", "arms-forward", "low-ready", "custom"),
        default="forearms-forward",
        help=(
            "Fixed robot-side pose used by official-calib startup and by c when "
            "--official-calib-base init-pose is selected."
        ),
    )
    parser.add_argument(
        "--wrist-orientation-mode",
        choices=("neutral", "identity", "live", "calibrated", "wrist-joints"),
        default="calibrated",
        help=(
            "How to fill wrist quaternions in vr_orientation. 'calibrated' tracks "
            "wrist rotation through the policy; 'wrist-joints' keeps policy orientation neutral "
            "and sends direct wrist roll/pitch/yaw overrides."
        ),
    )
    parser.add_argument(
        "--wrist-rotation-scale",
        type=float,
        default=0.65,
        help="Scale applied to calibrated wrist rotation. Use 0.5 if wrist tracking feels too aggressive.",
    )
    parser.add_argument(
        "--wrist-axis-remap",
        choices=("identity", "avp-palm", "x-to-y", "x-to-z", "y-to-x", "z-to-x"),
        default="avp-palm",
        help="Extra local-basis remap for calibrated wrist rotation.",
    )
    parser.add_argument("--left-wrist-rot-sign-x", type=float, default=1.0)
    parser.add_argument("--left-wrist-rot-sign-y", type=float, default=1.0)
    parser.add_argument("--left-wrist-rot-sign-z", type=float, default=-1.0)
    parser.add_argument("--right-wrist-rot-sign-x", type=float, default=-1.0)
    parser.add_argument("--right-wrist-rot-sign-y", type=float, default=1.0)
    parser.add_argument("--right-wrist-rot-sign-z", type=float, default=-1.0)
    parser.add_argument("--wrist-joint-sign-roll", type=float, default=1.0)
    parser.add_argument("--wrist-joint-sign-pitch", type=float, default=1.0)
    parser.add_argument("--wrist-joint-sign-yaw", type=float, default=1.0)
    parser.add_argument("--max-wrist-roll", type=float, default=1.5)
    parser.add_argument("--max-wrist-pitch", type=float, default=1.2)
    parser.add_argument("--max-wrist-yaw", type=float, default=1.2)
    parser.add_argument(
        "--legacy-head-relative",
        action="store_true",
        help="Use the old head-relative mapping instead of startup neutral calibration.",
    )
    parser.add_argument("--neutral-x", type=float, default=0.22, help="Neutral wrist target x in SONIC local frame.")
    parser.add_argument("--neutral-left-y", type=float, default=0.24, help="Neutral left wrist y.")
    parser.add_argument("--neutral-right-y", type=float, default=-0.24, help="Neutral right wrist y.")
    parser.add_argument("--neutral-hand-z", type=float, default=0.18, help="Neutral wrist height. Lower this if arms start high.")
    parser.add_argument("--neutral-head-x", type=float, default=0.0)
    parser.add_argument("--neutral-head-y", type=float, default=0.0)
    parser.add_argument("--neutral-head-z", type=float, default=0.75)
    parser.add_argument("--max-x", type=float, default=1.0)
    parser.add_argument("--max-y", type=float, default=1.0)
    parser.add_argument("--max-z", type=float, default=1.0)
    parser.add_argument("--left-fallback-y", type=float, default=0.25)
    parser.add_argument("--right-fallback-y", type=float, default=-0.25)
    parser.add_argument("--print-debug", action="store_true")
    parser.add_argument(
        "--enable-inspire-hand-sim",
        action="store_true",
        help="Also publish AVP finger tracking as sim-only Inspire hand ZMQ commands.",
    )
    parser.add_argument(
        "--enable-inspire-hand-dds",
        action="store_true",
        help="Publish AVP right-hand finger tracking to the physical Inspire hand DDS topic.",
    )
    parser.add_argument("--inspire-hand-topic", default="inspire_hand")
    parser.add_argument(
        "--hand-topic-side",
        choices=("l", "r"),
        default="l",
        help="DDS side for the physical right hand. Use the same side as Headless_driver_r --lr.",
    )
    parser.add_argument(
        "--hand-dds-network",
        default=None,
        help="Optional DDS NIC for physical Inspire hand commands, e.g. enp3s0.",
    )
    parser.add_argument("--hand-tracking-timeout", type=float, default=1.0)
    parser.add_argument(
        "--hand-calibration-file",
        default=default_hand_calibration_file(),
    )
    parser.add_argument("--open-angle", type=int, default=1000)
    parser.add_argument("--close-angle", type=int, default=0)
    parser.add_argument("--thumb-bend-open-angle", type=int, default=800)
    parser.add_argument("--thumb-bend-close-angle", type=int, default=200)
    parser.add_argument("--thumb-rotation-open-angle", type=int, default=200)
    parser.add_argument("--thumb-rotation-close-angle", type=int, default=800)
    parser.add_argument("--finger-range-scale", type=float, default=1.15)
    parser.add_argument("--thumb-bend-range-scale", type=float, default=1.35)
    parser.add_argument("--thumb-rotation-range-scale", type=float, default=1.25)
    parser.add_argument("--finger-mcp-weight", type=float, default=0.45)
    parser.add_argument("--finger-pip-weight", type=float, default=0.40)
    parser.add_argument("--finger-dip-weight", type=float, default=0.15)
    parser.add_argument("--thumb-mcp-weight", type=float, default=0.65)
    parser.add_argument("--thumb-ip-weight", type=float, default=0.35)
    parser.add_argument("--finger-smoothing", type=float, default=0.45)
    parser.add_argument("--thumb-smoothing", type=float, default=0.35)
    parser.add_argument("--finger-deadband", type=float, default=0.015)
    parser.add_argument("--thumb-deadband", type=float, default=0.02)
    parser.add_argument("--thumb-rotation-metric", choices=("angle", "distance"), default="angle")
    parser.add_argument("--thumb-open-distance", type=float, default=0.12)
    parser.add_argument("--thumb-close-distance", type=float, default=0.035)
    parser.add_argument("--flip-thumb-rotation", action="store_true")
    parser.add_argument(
        "--head-locomotion",
        action="store_true",
        help="Drive SONIC walking from head motion (work-master velocity derivative).",
    )
    parser.add_argument("--loco-velocity-gain", type=float, default=1.0)
    parser.add_argument("--loco-yaw-gain", type=float, default=0.9)
    parser.add_argument("--loco-forward-scale", type=float, default=1.0)
    parser.add_argument("--loco-lateral-scale", type=float, default=1.25)
    parser.add_argument("--loco-lateral-left-scale", type=float, default=1.0)
    parser.add_argument("--loco-lateral-right-scale", type=float, default=1.4)
    parser.add_argument("--loco-lateral-displacement-gain", type=float, default=2.5)
    parser.add_argument("--loco-lateral-deadzone", type=float, default=0.035)
    parser.add_argument("--loco-sign-x", type=float, default=1.0)
    parser.add_argument("--loco-sign-y", type=float, default=1.0)
    parser.add_argument("--loco-max-speed", type=float, default=0.45)
    parser.add_argument("--loco-max-yaw-rate", type=float, default=0.35)
    parser.add_argument("--loco-velocity-deadzone", type=float, default=0.07)
    parser.add_argument("--loco-yaw-deadzone", type=float, default=0.15)
    parser.add_argument("--loco-smooth", type=float, default=0.12)
    parser.add_argument("--loco-facing-smooth", type=float, default=0.22)
    parser.add_argument("--loco-output-deadzone", type=float, default=0.04)
    parser.add_argument("--loco-idle-decay", type=float, default=0.85)
    parser.add_argument("--loco-mode", type=int, default=1, help="SONIC planner mode when walking.")
    thumb_rotation_command_group = parser.add_mutually_exclusive_group()
    thumb_rotation_command_group.add_argument(
        "--invert-thumb-rotation-command",
        dest="invert_thumb_rotation_command",
        action="store_true",
        default=True,
        help="Reverse the physical Inspire thumb-root command direction after AVP calibration.",
    )
    thumb_rotation_command_group.add_argument(
        "--no-invert-thumb-rotation-command",
        dest="invert_thumb_rotation_command",
        action="store_false",
        help="Use the non-inverted physical Inspire thumb-root command direction.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    period = 1.0 / max(args.publish_rate, 1e-6)
    streamer = VisionProStreamer(ip=args.avp_endpoint, record=False, benchmark_quiet=True)
    publisher = PackedPublisher(args.host, args.port)
    state = BridgeState()
    smoother = PositionSmoother()
    enable_inspire_hand = args.enable_inspire_hand_sim or args.enable_inspire_hand_dds
    left_hand_mapper = InspireHandMapper.from_args(args) if enable_inspire_hand else None
    right_hand_mapper = InspireHandMapper.from_args(args) if enable_inspire_hand else None
    dds_hand_publisher = (
        DdsInspireHandPublisher(args.hand_topic_side, args.hand_dds_network)
        if args.enable_inspire_hand_dds
        else None
    )
    hand_open_command = np.array(
        [
            args.open_angle,
            args.open_angle,
            args.open_angle,
            args.open_angle,
            args.thumb_bend_open_angle,
            args.thumb_rotation_open_angle,
        ],
        dtype=np.float64,
    )
    hand_close_command = np.array(
        [
            args.close_angle,
            args.close_angle,
            args.close_angle,
            args.close_angle,
            args.thumb_bend_close_angle,
            args.thumb_rotation_close_angle,
        ],
        dtype=np.float64,
    )
    hand_command_min = np.minimum(hand_open_command, hand_close_command)
    hand_command_max = np.maximum(hand_open_command, hand_close_command)
    left_hand_command = hand_open_command.copy()
    right_hand_command = hand_open_command.copy()
    last_hand_tracking_time = 0.0

    print(f"AVP -> SONIC bridge bound to {publisher.endpoint}")
    print(f"Mapping mode: {args.mapping_mode}")
    print(f"Active hands: {args.active_hands}")
    if args.head_locomotion:
        print(
            "Head locomotion ON: move head to walk/turn (work-master velocity model). "
            f"max_speed={args.loco_max_speed} smooth={args.loco_smooth}"
        )
    if args.enable_inspire_hand_sim:
        print(f"Publishing sim Inspire hand commands on topic '{args.inspire_hand_topic}'.")
    if dds_hand_publisher is not None:
        print(f"Publishing physical Inspire right-hand commands to rt/inspire_hand/ctrl/{args.hand_topic_side}.")
    if args.mapping_mode == "official-calib":
        print(
            "Official calibration mode: startup holds the configured robot init pose; "
            "press c, hold your hands for the calibration delay, then tracking starts."
        )
    print("Waiting for AVP head tracking...")

    initial_head_robot_pos = None
    neutral_head_pose = None
    neutral_left_pose = None
    neutral_right_pose = None
    official_calibration = None
    official_base_positions = default_official_base_positions(args)
    official_base_orientations = default_official_base_orientations()
    calib_hold_until = 0.0
    pending_calibration_deadline = 0.0
    pending_base_positions = None
    pending_base_orientations = None
    last_sent_vr_position = None
    last_sent_vr_orientation = None
    head_loco_state = HeadLocomotionState()
    head_loco_cfg = HeadLocomotionConfig(
        velocity_gain=args.loco_velocity_gain,
        yaw_rate_gain=args.loco_yaw_gain,
        forward_scale=args.loco_forward_scale,
        lateral_scale=args.loco_lateral_scale,
        lateral_left_scale=args.loco_lateral_left_scale,
        lateral_right_scale=args.loco_lateral_right_scale,
        lateral_displacement_gain=args.loco_lateral_displacement_gain,
        lateral_velocity_deadzone=args.loco_lateral_deadzone,
        sign_x=args.loco_sign_x,
        sign_y=args.loco_sign_y,
        max_speed=args.loco_max_speed,
        max_yaw_rate=args.loco_max_yaw_rate,
        velocity_deadzone=args.loco_velocity_deadzone,
        yaw_rate_deadzone=args.loco_yaw_deadzone,
        smooth_alpha=args.loco_smooth,
        facing_smooth_alpha=args.loco_facing_smooth,
        output_deadzone=args.loco_output_deadzone,
        idle_decay=args.loco_idle_decay,
    )
    head_loco_calib_pos = None
    head_loco_calib_rot = None
    last_head_loco_time = None
    stand_hold = True
    policy_started = False
    try:
        with RawKeyboard() as keyboard:
            last_debug = 0.0
            last_start = 0.0
            while True:
                now = time.time()
                tracking = streamer.get_latest()

                if tracking is not None and tracking.head is not None and initial_head_robot_pos is None:
                    neutral_head_pose = avp_to_robot(tracking.head)
                    initial_head_robot_pos = neutral_head_pose[:3, 3].copy()
                    neutral_left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM)
                    neutral_right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM)
                    if args.mapping_mode != "official-calib":
                        official_calibration = capture_official_calibration(tracking, args)
                    official_base_positions = default_official_base_positions(args)
                    official_base_orientations = default_official_base_orientations()
                    print("AVP tracking locked.")
                    print(
                        "Robot base target: left=",
                        np.round(official_base_positions[0:3], 3).tolist(),
                        "right=",
                        np.round(official_base_positions[3:6], 3).tolist(),
                        "head=",
                        np.round(official_base_positions[6:9], 3).tolist(),
                    )
                    if args.mapping_mode == "official-calib":
                        print("Holding robot init pose. Press C to calibrate and start AVP tracking.")
                    if not args.no_auto_start:
                        publisher.send_command(start=True, stop=False, planner=True)
                        last_start = now
                        policy_started = True
                        print("Sent SONIC start command in planner mode (stand-hold until T).")

                if initial_head_robot_pos is None:
                    time.sleep(0.02)
                    continue

                for key in keyboard.read_keys():
                    if key == "]":
                        publisher.send_command(start=True, stop=False, planner=True)
                        last_start = now
                        policy_started = True
                        stand_hold = True
                        state.set_idle()
                        state.facing_angle = 0.0
                        print(
                            "\nSent SONIC start command (stand-hold). "
                            "Keep head/hands still until robot balances, then press T for teleop."
                        )
                        continue
                    if key in ("t", "T"):
                        if not policy_started:
                            publisher.send_command(start=True, stop=False, planner=True)
                            last_start = now
                            policy_started = True
                        stand_hold = False
                        head_loco_calib_pos = None
                        head_loco_calib_rot = None
                        calib_yaw = None
                        if tracking is not None and tracking.head is not None:
                            hp = head_pose_from_tracking(tracking)
                            if hp is not None:
                                calib_yaw = yaw_from_rot(hp[:3, :3])
                        reset_head_locomotion_state(head_loco_state, calib_yaw=calib_yaw)
                        msg = "Teleop enabled: AVP hands drive upper body."
                        if args.head_locomotion:
                            msg += " Forward/back = lean head; strafe = shift head left/right; turn head to change direction."
                        else:
                            msg += " Use WASD in this terminal to walk."
                        print(f"\n{msg}")
                        continue
                    if key in ("c", "C"):
                        if args.mapping_mode == "official-calib":
                            if args.official_calib_base == "current-target" and last_sent_vr_position is not None:
                                pending_base_positions = np.asarray(last_sent_vr_position, dtype=np.float64).reshape(9).copy()
                                if last_sent_vr_orientation is not None:
                                    pending_base_orientations = np.asarray(last_sent_vr_orientation, dtype=np.float64).reshape(12).copy()
                                else:
                                    pending_base_orientations = official_base_orientations.copy()
                            else:
                                pending_base_positions = default_official_base_positions(args)
                                pending_base_orientations = default_official_base_orientations()

                            official_base_positions = pending_base_positions.copy()
                            official_base_orientations = pending_base_orientations.copy()
                            pending_calibration_deadline = time.time() + max(args.calib_hold_sec, 0.0)
                            calib_hold_until = pending_calibration_deadline
                            print(
                                f"\nCalibration armed: hold your hands at the desired zero pose for {max(args.calib_hold_sec, 0.0):.1f}s.",
                                "Robot target held at left=",
                                np.round(official_base_positions[0:3], 3).tolist(),
                                "right=",
                                np.round(official_base_positions[3:6], 3).tolist(),
                            )
                        else:
                            captured = capture_official_calibration(tracking, args)
                            if captured is None:
                                print("\nCalibration skipped: AVP head tracking is not ready.")
                            else:
                                official_calibration = captured
                                print("\nOfficial-style AVP calibration captured.")
                        continue
                    if not handle_key(state, key, head_locomotion=args.head_locomotion):
                        publisher.send_command(start=False, stop=True, planner=True)
                        print("\nSent SONIC stop command.")
                        return

                head_loco_allowed = (
                    not stand_hold
                    and (args.mapping_mode != "official-calib" or official_calibration is not None)
                )
                if (
                    args.head_locomotion
                    and head_loco_allowed
                    and tracking is not None
                    and tracking.head is not None
                ):
                    head_pose = head_pose_from_tracking(tracking)
                    if head_pose is not None:
                        if head_loco_calib_pos is None:
                            head_loco_calib_pos = head_pose[:3, 3].copy()
                            head_loco_calib_rot = head_pose[:3, :3].copy()
                            head_loco_state.calib_yaw = yaw_from_rot(head_pose[:3, :3])
                            head_loco_state.facing_angle = 0.0
                            print(
                                "\nHead locomotion calibrated at "
                                f"{np.round(head_loco_calib_pos, 3).tolist()} "
                                f"(forward = current head yaw)"
                            )
                        loop_dt = max(now - last_head_loco_time, period) if last_head_loco_time else period
                        last_head_loco_time = now
                        planner_cmd = compute_sonic_planner_command(
                            head_pose,
                            head_loco_state,
                            head_loco_cfg,
                            loop_dt,
                            calib_pos=head_loco_calib_pos,
                            calib_rot=head_loco_calib_rot,
                            locomotion_mode=args.loco_mode,
                        )
                        state.mode = planner_cmd.mode
                        state.movement = planner_cmd.movement.copy()
                        state.speed = planner_cmd.speed
                        state.facing_angle = float(
                            np.arctan2(planner_cmd.facing[1], planner_cmd.facing[0])
                        )

                if stand_hold:
                    state.set_idle()

                if not args.no_auto_start and now - last_start > 2.0:
                    publisher.send_command(start=True, stop=False, planner=True)
                    last_start = now
                    policy_started = True

                if pending_calibration_deadline > 0.0 and now >= pending_calibration_deadline:
                    captured = capture_official_calibration(tracking, args)
                    if captured is None:
                        print("\nCalibration delayed: AVP head tracking is not ready.")
                        pending_calibration_deadline = now + 0.25
                        calib_hold_until = pending_calibration_deadline
                    else:
                        official_calibration = captured
                        head_loco_calib_pos = None
                        head_loco_calib_rot = None
                        calib_yaw = None
                        if tracking is not None and tracking.head is not None:
                            hp = head_pose_from_tracking(tracking)
                            if hp is not None:
                                calib_yaw = yaw_from_rot(hp[:3, :3])
                        reset_head_locomotion_state(head_loco_state, calib_yaw=calib_yaw)
                        if pending_base_positions is not None:
                            official_base_positions = pending_base_positions.copy()
                        if pending_base_orientations is not None:
                            official_base_orientations = pending_base_orientations.copy()
                        pending_calibration_deadline = 0.0
                        pending_base_positions = None
                        pending_base_orientations = None
                        calib_hold_until = 0.0
                        print(
                            "\nCalibration captured at delayed hand pose:",
                            "left_base=",
                            np.round(official_base_positions[0:3], 3).tolist(),
                            "right_base=",
                            np.round(official_base_positions[3:6], 3).tolist(),
                        )

                hand_debug_lines = []
                if enable_inspire_hand:
                    saw_hand_tracking = False
                    for side, mapper in (
                        ("left", left_hand_mapper),
                        ("right", right_hand_mapper),
                    ):
                        if not hand_is_active(args, side):
                            continue
                        hand = getattr(tracking, side, None) if tracking is not None else None
                        if hand is None:
                            continue
                        try:
                            command, hand_map_debug = mapper.build_command(hand)
                        except AttributeError as exc:
                            if args.print_debug and now - last_debug > 1.0:
                                hand_debug_lines.append(f"{side}_hand missing joint: {exc}")
                            continue

                        command = np.clip(command, hand_command_min, hand_command_max)
                        if side == "left":
                            left_hand_command = command
                        else:
                            right_hand_command = command
                        saw_hand_tracking = True
                        if args.print_debug and now - last_debug > 1.0:
                            hand_debug_lines.append(f"{side}_{format_hand_debug(command, hand_map_debug)}")

                    if saw_hand_tracking:
                        last_hand_tracking_time = now
                        if args.enable_inspire_hand_sim:
                            publisher.send_inspire_hand(
                                left_hand_command,
                                right_hand_command,
                                topic=args.inspire_hand_topic,
                            )
                        if dds_hand_publisher is not None:
                            dds_hand_publisher.send(right_hand_command)
                    elif (
                        last_hand_tracking_time
                        and now - last_hand_tracking_time >= args.hand_tracking_timeout
                    ):
                        left_hand_command = hand_open_command.copy()
                        right_hand_command = hand_open_command.copy()
                        left_hand_mapper.reset()
                        right_hand_mapper.reset()
                        last_hand_tracking_time = now
                        if args.enable_inspire_hand_sim:
                            publisher.send_inspire_hand(
                                left_hand_command,
                                right_hand_command,
                                topic=args.inspire_hand_topic,
                            )
                        if dds_hand_publisher is not None:
                            dds_hand_publisher.send(right_hand_command)
                        if args.print_debug and now - last_debug > 1.0:
                            hand_debug_lines.append("hand tracking lost -> sim safe open")

                target_debug = {}
                left_wrist_joints = None
                right_wrist_joints = None
                if stand_hold:
                    targets = (
                        official_base_positions.copy(),
                        official_base_orientations.copy(),
                    )
                elif args.legacy_head_relative:
                    targets = build_vr_targets(tracking, initial_head_robot_pos, args)
                elif args.mapping_mode == "relative-zero":
                    targets = build_calibrated_vr_targets(
                        tracking,
                        neutral_left_pose,
                        neutral_right_pose,
                        neutral_head_pose,
                        args,
                    )
                elif args.mapping_mode == "official-calib":
                    if official_calibration is None:
                        targets = (official_base_positions.copy(), official_base_orientations.copy())
                    else:
                        result = build_official_calib_vr_targets(
                            tracking,
                            official_calibration,
                            official_base_positions,
                            official_base_orientations,
                            args,
                            force_base=now < calib_hold_until,
                        )
                        if result is None:
                            targets = None
                        else:
                            vr_position, vr_orientation, target_debug, left_wrist_joints, right_wrist_joints = result
                            targets = (vr_position, vr_orientation)
                else:
                    result = build_head_relative_vr_targets(
                        tracking,
                        neutral_left_pose,
                        neutral_right_pose,
                        args,
                    )
                    if result is None:
                        targets = None
                    else:
                        vr_position, vr_orientation, target_debug = result
                        targets = (vr_position, vr_orientation)
                if targets is not None and policy_started:
                    vr_position, vr_orientation = targets
                    if args.mapping_mode in ("hybrid", "official-calib") and not args.legacy_head_relative:
                        vr_position = smoother.update(
                            vr_position,
                            now,
                            tau=args.hybrid_smoothing_tau,
                            max_speed=args.hybrid_max_speed,
                        )
                    last_sent_vr_position = vr_position.copy()
                    last_sent_vr_orientation = vr_orientation.copy()
                    publisher.send_planner(
                        mode=state.mode,
                        movement=state.movement,
                        facing=np.array([1.0, 0.0, 0.0], dtype=np.float64)
                        if stand_hold
                        else state.facing,
                        speed=state.speed,
                        height=state.height,
                        vr_position=vr_position,
                        vr_orientation=vr_orientation,
                        left_wrist_joints=left_wrist_joints,
                        right_wrist_joints=right_wrist_joints,
                    )

                    if args.print_debug and now - last_debug > 1.0:
                        print(
                            "mode=",
                            state.mode,
                            "move=",
                            np.round(state.movement, 3).tolist(),
                            "face=",
                            np.round(state.facing, 3).tolist(),
                            "vr_pos=",
                            np.round(vr_position, 3).tolist(),
                        )
                        if target_debug:
                            print(
                                "rel_to_head left=",
                                None
                                if target_debug.get("left_rel") is None
                                else np.round(target_debug.get("left_rel"), 3).tolist(),
                                "right=",
                                None
                                if target_debug.get("right_rel") is None
                                else np.round(target_debug.get("right_rel"), 3).tolist(),
                            )
                            if "left_delta" in target_debug:
                                print(
                                    "calib_delta left=",
                                    None
                                    if target_debug.get("left_delta") is None
                                    else np.round(target_debug.get("left_delta"), 3).tolist(),
                                    "right=",
                                    None
                                    if target_debug.get("right_delta") is None
                                    else np.round(target_debug.get("right_delta"), 3).tolist(),
                                )
                            if left_wrist_joints is not None or right_wrist_joints is not None:
                                print(
                                    "wrist_joints left=",
                                    None if left_wrist_joints is None else np.round(left_wrist_joints, 3).tolist(),
                                    "right=",
                                    None if right_wrist_joints is None else np.round(right_wrist_joints, 3).tolist(),
                                )
                        if args.head_locomotion and head_loco_state.debug:
                            print(
                                "head_loco delta_body=",
                                head_loco_state.debug.get("delta_body"),
                                "cmd=",
                                head_loco_state.debug.get("cmd"),
                                "strafe=",
                                head_loco_state.debug.get("strafe_intent"),
                            )
                        for line in hand_debug_lines:
                            print(line)
                        last_debug = now

                time.sleep(period)
    finally:
        if dds_hand_publisher is not None:
            try:
                dds_hand_publisher.send(hand_open_command)
            except Exception as exc:
                print(f"Failed to safe-open Inspire hand cleanly: {exc}")
        publisher.close()


if __name__ == "__main__":
    main()
