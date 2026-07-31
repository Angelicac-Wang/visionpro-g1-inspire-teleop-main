"""TeleVuer-compatible shim backed by VisionProTeleop (Tracking Streamer app).

Safari WebXR (8012) often does not stream head/hand pose to the PC. The native
Tracking Streamer app sends full tracking over gRPC (same LAN) or WebRTC
(cross-network room code), which is what xr_teleoperate needs for arm IK and
HEAD_LOCO.
"""

from __future__ import annotations

import os
import re
import sys
import time

import numpy as np

VISIONPRO_TELEOP_ROOT = os.environ.get(
    "VISIONPRO_TELEOP_ROOT",
    "/mnt/newssd/unitree_sim_isaaclab/inspire_hand_ws/VisionProTeleop",
)
if VISIONPRO_TELEOP_ROOT not in sys.path:
    sys.path.insert(0, VISIONPRO_TELEOP_ROOT)

from avp_stream import VisionProStreamer  # noqa: E402

YUP2ZUP = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
INV_YUP2ZUP = np.linalg.inv(YUP2ZUP)

_ROOM_CODE_RE = re.compile(r"^[A-Za-z0-9]{2,8}-[A-Za-z0-9]{2,8}$")


def _is_room_code(value: str) -> bool:
    return bool(_ROOM_CODE_RE.match(value.strip()))


def _avp_mat_to_openxr(mat4: np.ndarray) -> np.ndarray:
    return INV_YUP2ZUP @ np.asarray(mat4, dtype=np.float64)


def _avp_point_to_openxr(point3) -> np.ndarray:
    hom = np.array([point3[0], point3[1], point3[2], 1.0], dtype=np.float64)
    return (INV_YUP2ZUP @ hom)[:3]


class AvpNativeTeleVuer:
    """Minimal TeleVuer surface used by TeleVuerWrapper and teleop_hand_and_arm."""

    def __init__(
        self,
        endpoint: str,
        img_shape: tuple[int, int] = (480, 640),
        display_mode: str = "immersive",
        display_fps: float = 30.0,
        stereo_video: bool = False,
        ht_backend: str = "grpc",
        webrtc_port: int = 9999,
        video_size: str | None = None,
    ):
        self.display_mode = display_mode
        self.img_shape = img_shape
        self.display_fps = display_fps
        self.zmq = True
        self.webrtc = False
        self.binocular = stereo_video

        endpoint = endpoint.strip()
        cross_network = _is_room_code(endpoint)
        if cross_network and ht_backend == "grpc":
            ht_backend = "webrtc"

        mode_label = "cross-network (room code)" if cross_network else "same LAN (gRPC)"
        print(
            f"[avp_native] Connecting to Tracking Streamer: {endpoint} [{mode_label}]",
            flush=True,
        )
        if cross_network:
            print(
                "[avp_native] On Vision Pro: open Tracking Streamer -> Cross-Network ON "
                f"-> enter the SAME room code: {endpoint}",
                flush=True,
            )
        else:
            print(
                "[avp_native] On Vision Pro: open Tracking Streamer and tap Start "
                "(same Wi‑Fi as this PC).",
                flush=True,
            )

        self.streamer = VisionProStreamer(
            ip=endpoint,
            record=False,
            ht_backend=ht_backend,
            benchmark_quiet=True,
        )

        if video_size is None:
            video_size = os.environ.get("AVP_VIDEO_SIZE", "1280x720")
        vw, vh = map(int, video_size.split("x"))
        self._video_w = vw
        self._video_h = vh
        self._stereo_video = stereo_video

        self.streamer.configure_video(
            device=None,
            size=video_size,
            fps=int(max(display_fps, 1)),
            stereo=stereo_video,
        )
        self.streamer.start_webrtc(port=webrtc_port, blocking=False)
        print(
            f"[avp_native] Sim camera will stream to VP via WebRTC ({video_size}, port {webrtc_port})",
            flush=True,
        )

        self._head = np.eye(4)
        self._left_arm = np.eye(4)
        self._right_arm = np.eye(4)
        self._left_hand_pos = np.zeros((25, 3))
        self._right_hand_pos = np.zeros((25, 3))
        self._left_hand_rot = np.tile(np.eye(3), (25, 1, 1))
        self._right_hand_rot = np.tile(np.eye(3), (25, 1, 1))
        self._left_pinch = False
        self._left_pinch_value = 0.05
        self._right_pinch = False
        self._right_pinch_value = 0.05
        self._left_squeeze = False
        self._left_squeeze_value = 0.0
        self._right_squeeze = False
        self._right_squeeze_value = 0.0
        self._have_tracking = False
        self._frames = 0
        self._last_diag = 0.0

    @classmethod
    def create(cls, endpoint: str, **kwargs) -> "AvpNativeTeleVuer":
        return cls(endpoint=endpoint, **kwargs)

    def _poll(self) -> None:
        latest = self.streamer.get_latest()
        if latest is None or latest.head is None:
            return

        head_oxr = _avp_mat_to_openxr(latest.head)
        self._head = head_oxr

        for side, arm_attr, pos_attr, rot_attr, pinch_attr, pinch_val_attr in (
            ("left", "_left_arm", "_left_hand_pos", "_left_hand_rot", "_left_pinch", "_left_pinch_value"),
            ("right", "_right_arm", "_right_hand_pos", "_right_hand_rot", "_right_pinch", "_right_pinch_value"),
        ):
            hand = getattr(latest, side, None)
            if hand is None or hand.shape[0] < 25:
                continue
            wrist = hand.wrist if hasattr(hand, "wrist") else hand[0]
            setattr(self, arm_attr, _avp_mat_to_openxr(wrist))

            positions = np.zeros((25, 3), dtype=np.float64)
            rotations = np.zeros((25, 3, 3), dtype=np.float64)
            for i in range(min(25, hand.shape[0])):
                joint_oxr = _avp_mat_to_openxr(hand[i])
                positions[i] = joint_oxr[:3, 3]
                rotations[i] = joint_oxr[:3, :3]
            setattr(self, pos_attr, positions)
            setattr(self, rot_attr, rotations)

            pinch_dist = float(getattr(hand, "pinch_distance", 0.05))
            setattr(self, pinch_val_attr, pinch_dist)
            setattr(self, pinch_attr, pinch_dist < 0.03)

        self._frames += 1
        if not self._have_tracking:
            det = np.linalg.det(self._head[:3, :3])
            print(
                f"[avp_native] Tracking live (head det={det:.3f}, "
                f"L0={self._left_hand_pos[0].round(3).tolist()})",
                flush=True,
            )
            self._have_tracking = True

        now = time.time()
        if now - self._last_diag > 5.0 and self._frames % 300 == 0:
            det = np.linalg.det(self._head[:3, :3])
            print(
                f"[avp_native] tracking OK frame={self._frames} head_det={det:.3f}",
                flush=True,
            )
            self._last_diag = now

    @property
    def head_pose(self) -> np.ndarray:
        self._poll()
        return self._head.copy()

    @property
    def left_arm_pose(self) -> np.ndarray:
        self._poll()
        return self._left_arm.copy()

    @property
    def right_arm_pose(self) -> np.ndarray:
        self._poll()
        return self._right_arm.copy()

    @property
    def left_hand_positions(self) -> np.ndarray:
        self._poll()
        return self._left_hand_pos.copy()

    @property
    def right_hand_positions(self) -> np.ndarray:
        self._poll()
        return self._right_hand_pos.copy()

    @property
    def left_hand_orientations(self) -> np.ndarray:
        self._poll()
        return self._left_hand_rot.copy()

    @property
    def right_hand_orientations(self) -> np.ndarray:
        self._poll()
        return self._right_hand_rot.copy()

    @property
    def left_hand_pinch(self) -> bool:
        self._poll()
        return self._left_pinch

    @property
    def left_hand_pinchValue(self) -> float:
        self._poll()
        return self._left_pinch_value

    @property
    def right_hand_pinch(self) -> bool:
        self._poll()
        return self._right_pinch

    @property
    def right_hand_pinchValue(self) -> float:
        self._poll()
        return self._right_pinch_value

    @property
    def left_hand_squeeze(self) -> bool:
        return self._left_squeeze

    @property
    def left_hand_squeezeValue(self) -> float:
        return self._left_squeeze_value

    @property
    def right_hand_squeeze(self) -> bool:
        return self._right_squeeze

    @property
    def right_hand_squeezeValue(self) -> float:
        return self._right_squeeze_value

    def render_to_xr(self, image) -> None:
        if image is None:
            return
        import cv2

        if hasattr(image, "bgr"):
            image = image.bgr
        frame = np.asarray(image)
        if self._stereo_video and frame.shape[1] * 2 == self._video_w:
            frame = np.hstack([frame, frame])
        elif frame.shape[1] != self._video_w or frame.shape[0] != self._video_h:
            frame = cv2.resize(frame, (self._video_w, self._video_h))
        self.streamer.update_frame(frame)

    def close(self) -> None:
        try:
            self.streamer.cleanup()
        except Exception:
            pass
