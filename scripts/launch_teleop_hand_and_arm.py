#!/usr/bin/env python3
"""Launch teleop_hand_and_arm.py with user-writable paths and AVP-friendly fixes."""

from __future__ import annotations

import builtins
import logging
import os
import runpy
import sys
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
XR_TELEOP_ROOT = os.environ.get(
    "XR_TELEOP_ROOT",
    "/mnt/newssd/unitree_sim_isaaclab/xr_teleoperate",
)
TELEOP_DIR = os.path.join(XR_TELEOP_ROOT, "teleop")
USER_CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".config", "xr_teleoperate")
CLIENT_CONFIG = os.path.join(USER_CONFIG_DIR, "cam_config_client.yaml")
XR_FORCE_ZMQ = os.environ.get("XR_FORCE_ZMQ", "0") == "1"
XR_CLIENT_NATIVE = os.environ.get("XR_CLIENT", "").lower() in ("native", "avp", "visionpro")
AVP_ENDPOINT = (os.environ.get("AVP_ENDPOINT") or os.environ.get("AVP_IP") or "").strip()
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Head-tracking -> whole-body walking. When enabled, the launcher reads the AVP
# head pose every teleop frame, maps head lean/turn to a velocity command, and
# publishes it to the sim's whole-body walking policy on rt/run_command/cmd.
# Only use this together with a "Wholebody" sim task (see run_sim_wholebody.sh).
XR_HEAD_LOCO = os.environ.get("XR_HEAD_LOCO", "0") == "1"
LOG = logging.getLogger("xr_teleop_launcher")

os.makedirs(USER_CONFIG_DIR, exist_ok=True)

sys.path.insert(0, TELEOP_DIR)
sys.path.insert(0, os.path.join(TELEOP_DIR, "teleimager", "src"))

_IMAGE_CLIENT_PATCHED = False
_TELEVUER_PATCHED = False
_TV_WRAPPER_PATCHED = False
_FRAME_LOG = {"count": 0, "null": 0}
_HEAD_LOCO = None


# Vision Pro immersive eye-cameras only render the stereo image planes
# (layers=1 left eye, layers=2 right eye). A monocular (layer 0) image plane shows
# on a normal PC monitor but NOT inside the Vision Pro headset. So we present the
# single sim camera as a duplicated side-by-side "stereo" frame (480 x 1280).
SIM_MONO_W = 640
SIM_H = 480
STEREO_W = SIM_MONO_W * 2  # 1280


def _normalize_head_camera(cfg: dict) -> dict:
    head = dict(cfg.get("head_camera", {}))
    if XR_CLIENT_NATIVE:
        # Native Tracking Streamer receives sim video via VisionProStreamer WebRTC,
        # not TeleVuer 8012 / teleimager WebRTC. Pull frames from sim ZMQ locally.
        head["enable_webrtc"] = False
        head["enable_zmq"] = True
        head["image_shape"] = [SIM_H, SIM_MONO_W]
        head["binocular"] = False
    elif XR_FORCE_ZMQ:
        head["enable_webrtc"] = False
        head["enable_zmq"] = True
        # binocular so TeleVuer uses the layers=1/2 path that Vision Pro renders.
        head["image_shape"] = [SIM_H, STEREO_W]
        head["binocular"] = True
    else:
        head["image_shape"] = [SIM_H, SIM_MONO_W]
        head["binocular"] = False
    cfg["head_camera"] = head
    return cfg


def _patch_image_client(module) -> None:
    global _IMAGE_CLIENT_PATCHED
    if _IMAGE_CLIENT_PATCHED:
        return
    _IMAGE_CLIENT_PATCHED = True

    orig_requester_init = module.ZMQ_Requester.__init__

    def requester_init(self, host: str, port: int):
        orig_requester_init(self, host, port)
        self._config_client_path = CLIENT_CONFIG

    module.ZMQ_Requester.__init__ = requester_init

    orig_get_cam_config = module.ImageClient.get_cam_config

    def get_cam_config(self):
        cfg = orig_get_cam_config(self)
        return _normalize_head_camera(cfg)

    module.ImageClient.get_cam_config = get_cam_config


def _patch_televuer(module) -> None:
    global _TELEVUER_PATCHED
    if _TELEVUER_PATCHED or not hasattr(module, "TeleVuer"):
        return
    _TELEVUER_PATCHED = True

    import cv2

    orig_render_to_xr = module.TeleVuer.render_to_xr

    def render_to_xr(self, image):
        if hasattr(image, "bgr"):
            image = image.bgr
        if image is None:
            _FRAME_LOG["null"] += 1
            if _FRAME_LOG["null"] in (1, 30, 100):
                print(
                    f"[xr_launcher] WARNING: no head camera frame yet ({_FRAME_LOG['null']} empty reads). "
                    "Is Terminal 1 sim running and Isaac window clicked?",
                    flush=True,
                )
            return
        expected_h, expected_w = self.img_shape[0], self.img_shape[1]
        # Duplicate a mono sim frame into a side-by-side stereo frame when the
        # display buffer is wider than the source (Vision Pro stereo path).
        if self.binocular and image.shape[1] * 2 == expected_w and image.shape[0] == expected_h:
            import numpy as np

            image = np.hstack([image, image])
        elif image.shape[0] != expected_h or image.shape[1] != expected_w:
            image = cv2.resize(image, (expected_w, expected_h))
        _FRAME_LOG["count"] += 1
        if _FRAME_LOG["count"] in (1, 30, 100):
            print(
                f"[xr_launcher] Pushing camera frame #{_FRAME_LOG['count']} to VR, shape={image.shape}",
                flush=True,
            )
        orig_render_to_xr(self, image)

    module.TeleVuer.render_to_xr = render_to_xr

    # Vision Pro immersive eyes render layer 1 (left) and layer 2 (right). A mono
    # WebRTCVideoPlane sits on layer 0 -> visible on a PC monitor but black in the
    # headset. Duplicate it onto both eye layers so both eyes show the sim video.
    import asyncio as _asyncio
    from vuer.schemas import WebRTCVideoPlane, Hands, MotionControllers

    async def mono_webrtc_two_layers(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(stream=True, key="hands", hideLeft=True, hideRight=True),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(stream=True, key="motionControllers", left=True, right=True),
                to="bgChildren",
            )
        print("[xr_launcher] webrtc immersive: video plane on eye layers 1 and 2", flush=True)
        while True:
            session.upsert(
                [
                    WebRTCVideoPlane(
                        src=self.webrtc_url, iceServer=None, iceServers=[],
                        key="video-left", aspect=self.aspect_ratio, height=7, layers=1,
                    ),
                    WebRTCVideoPlane(
                        src=self.webrtc_url, iceServer=None, iceServers=[],
                        key="video-right", aspect=self.aspect_ratio, height=7, layers=2,
                    ),
                ],
                to="bgChildren",
            )
            await _asyncio.sleep(1.0 / self.display_fps)

    module.TeleVuer.main_image_monocular_webrtc = mono_webrtc_two_layers


class _HeadLoco:
    """Maps AVP head pose to whole-body walking velocity commands for Isaac sim.

    Isaac Wholebody uses Unitree policy.onnx (NOT SONIC). We send
    [vx, vy, vyaw, height] on rt/run_command/cmd.

    Default mode (velocity): work-master style head motion derivative in robot
    frame — move head to walk, stop when head stops, head turn drives vyaw.

    Legacy modes: tilt (rotation lean), displacement (P-control on offset).
    """

    TOPIC = "rt/run_command/cmd"
    SHM_NAME = "isaac_run_command_cmd"

    def __init__(self):
        import numpy as np

        sys.path.insert(0, SCRIPT_DIR)
        from g1_head_locomotion import (
            HeadLocomotionConfig,
            HeadLocomotionState,
            compute_head_locomotion_velocity,
            head_pose_from_openxr,
            horizontal_heading_from_calib_rot,
            reset_head_locomotion_state,
            walk_vector_to_world_rot,
        )

        self._np = np
        self._horizontal_heading_from_calib_rot = horizontal_heading_from_calib_rot
        self._compute_head_locomotion_velocity = compute_head_locomotion_velocity
        self._head_pose_from_openxr = head_pose_from_openxr
        self._reset_head_locomotion_state = reset_head_locomotion_state
        self._walk_vector_to_world_rot = walk_vector_to_world_rot
        self._loco_state = HeadLocomotionState()
        self._loco_cfg = HeadLocomotionConfig(
            velocity_gain=float(os.environ.get("HEAD_LOCO_VELOCITY_GAIN", "1.0")),
            yaw_rate_gain=float(os.environ.get("HEAD_LOCO_YAW_GAIN", "0.9")),
            forward_scale=float(os.environ.get("HEAD_LOCO_FORWARD_SCALE", "1.0")),
            lateral_scale=float(os.environ.get("HEAD_LOCO_LATERAL_SCALE", "0.85")),
            sign_x=float(os.environ.get("HEAD_LOCO_SIGN_X", "1.0")),
            sign_y=float(os.environ.get("HEAD_LOCO_SIGN_Y", "1.0")),
            max_speed=float(os.environ.get("HEAD_LOCO_V_LIN", "0.45")),
            max_yaw_rate=float(os.environ.get("HEAD_LOCO_V_YAW", "0.7")),
            velocity_deadzone=float(os.environ.get("HEAD_LOCO_VEL_DEADZONE", "0.08")),
            yaw_rate_deadzone=float(os.environ.get("HEAD_LOCO_YAW_DEAD", "0.08")),
            smooth_alpha=float(os.environ.get("HEAD_LOCO_SMOOTH", "0.12")),
            output_deadzone=float(os.environ.get("HEAD_LOCO_OUTPUT_DEADZONE", "0.04")),
            idle_decay=float(os.environ.get("HEAD_LOCO_IDLE_DECAY", "0.85")),
        )
        self._calib_pos = None
        self._last_update_t = None

        self._pub = None
        self._shm = None
        self._ref_R = None
        self._ref_center = None
        self._ref_lr_plane = None
        self._hand_frame = "openxr"
        self._last_R = None
        self._stale_count = 0
        self._head_bad_streak = 0
        self._source = "head"
        self._announced_hands = False
        self._announced_no_stream = False
        self._frame = 0
        self._calib_until = None
        self._last_print = 0.0
        self._warned_stale = False
        self._seen_tracking = False
        self.mode = os.environ.get("HEAD_LOCO_MODE", "velocity").strip().lower()
        self.v_lin = self._loco_cfg.max_speed
        self.v_lat = float(os.environ.get("HEAD_LOCO_V_LAT", str(self._loco_cfg.max_speed)))
        self.v_yaw = self._loco_cfg.max_yaw_rate
        self.pitch_gain = float(os.environ.get("HEAD_LOCO_PITCH_GAIN", "1.2"))
        self.yaw_gain = float(os.environ.get("HEAD_LOCO_YAW_GAIN", "1.0"))
        self.hand_gain = float(os.environ.get("HAND_LOCO_GAIN", "2.5"))
        self.pitch_dead = float(os.environ.get("HEAD_LOCO_PITCH_DEAD", "0.08"))
        self.yaw_dead = float(os.environ.get("HEAD_LOCO_YAW_DEAD", "0.06"))
        self.pitch_sign = float(os.environ.get("HEAD_LOCO_PITCH_SIGN", "1.0"))
        self.yaw_sign = float(os.environ.get("HEAD_LOCO_YAW_SIGN", "1.0"))
        self.height = float(os.environ.get("HEAD_LOCO_HEIGHT", "0.8"))
        self.calib_sec = float(os.environ.get("HEAD_LOCO_CALIB_SEC", "2.5"))
        self.disp_scale = float(os.environ.get("HEAD_LOCO_DISP_SCALE", "0.6"))
        self.disp_kp = float(os.environ.get("HEAD_LOCO_DISP_KP", "0.7"))
        self.disp_kp_lat = float(os.environ.get("HEAD_LOCO_DISP_KP_LAT", "0.5"))
        self.disp_dead = float(os.environ.get("HEAD_LOCO_DISP_DEAD", "0.035"))
        self.disp_fwd_sign = float(os.environ.get("HEAD_LOCO_DISP_FWD_SIGN", "1.0"))
        self.disp_lat_sign = float(os.environ.get("HEAD_LOCO_DISP_LAT_SIGN", "1.0"))
        self._ref_t = None
        self.hand_fallback = os.environ.get("HEAD_LOCO_HAND_FALLBACK", "0") == "1"
        self.debug = os.environ.get("HEAD_LOCO_DEBUG", "1") == "1"
        native_hint = (
            "  Native Tracking Streamer sends head + hands to the PC (HEAD_LOCO works)."
            if XR_CLIENT_NATIVE
            else "  AVP Safari usually sends NO head matrix — use XR_CLIENT=native or Step 2."
        )
        mode_hint = {
            "velocity": "  Mode=velocity: head motion speed -> walk (work-master). Stop moving head to stop.",
            "displacement": "  Mode=displacement: legacy P-control on head offset.",
            "tilt": "  Mode=tilt: head rotation lean/turn (legacy).",
        }.get(self.mode, f"  Mode={self.mode}")
        print(
            "[xr_launcher] HEAD_LOCO enabled (Isaac ONNX legs, NOT SONIC).\n"
            f"{native_hint}\n"
            f"{mode_hint}\n"
            f"  limits: max_speed={self._loco_cfg.max_speed} max_yaw={self._loco_cfg.max_yaw_rate} "
            f"smooth={self._loco_cfg.smooth_alpha}\n"
            "  Move head to walk/turn; hold still to stop. Press [b] to reset sim after a fall.",
            flush=True,
        )

    def _reset_session(self) -> None:
        self._calib_until = None
        self._ref_R = None
        self._ref_center = None
        self._ref_lr_plane = None
        self._ref_t = None
        self._calib_pos = None
        self._last_update_t = None
        self._reset_head_locomotion_state(self._loco_state)
        self._stale_count = 0
        self._head_bad_streak = 0
        self._source = "head"
        self._announced_hands = False
        self._frame = 0
        print(f"[xr_launcher] HEAD_LOCO active (mode={self.mode})", flush=True)

    def _ensure_transport(self):
        if self._pub is not None and self._shm is not None:
            return
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        if self._pub is None:
            self._String_ = String_
            self._pub = ChannelPublisher(self.TOPIC, String_)
            self._pub.Init()
        if self._shm is None:
            sim_root = os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")
            if sim_root not in sys.path:
                sys.path.insert(0, sim_root)
            from dds.sharedmemorymanager import SharedMemoryManager

            self._shm = SharedMemoryManager(self.SHM_NAME, 512)
            print(f"[xr_launcher] HEAD_LOCO connected to {self.SHM_NAME}", flush=True)

    @staticmethod
    def _deadzone(x, dz):
        if abs(x) <= dz:
            return 0.0
        return x - dz if x > 0 else x + dz

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def _send(self, cmd_str: str) -> None:
        self._ensure_transport()
        if self._shm is not None:
            self._shm.write_data({"run_command": cmd_str})
        if self._pub is not None:
            self._pub.Write(self._String_(data=cmd_str))

    @staticmethod
    def _valid_rotation(R, np) -> bool:
        if R.shape != (3, 3):
            return False
        if not np.all(np.isfinite(R)):
            return False
        det = np.linalg.det(R)
        return np.isfinite(det) and not np.isclose(det, 0.0, atol=1e-5)

    def _mat4(self, arr):
        return self._np.asarray(arr, dtype=float).reshape(4, 4, order="F")

    def _head_valid(self, H) -> bool:
        np = self._np
        if np.allclose(H, 0.0):
            return False
        return self._valid_rotation(H[:3, :3], np)

    def _read_wrists(self, tvuer, tele_data=None):
        np = self._np
        # Same wrist poses used by arm IK (best signal when arms move in sim).
        if tele_data is not None:
            L = np.asarray(tele_data.left_wrist_pose, dtype=float)
            R = np.asarray(tele_data.right_wrist_pose, dtype=float)
            if L.shape == (4, 4) and R.shape == (4, 4):
                tl, tr = L[:3, 3], R[:3, 3]
                if np.linalg.norm(tl) > 0.05 or np.linalg.norm(tr) > 0.05:
                    return tl, tr, "robot"
        lp = np.asarray(tvuer.left_hand_positions[0], dtype=float)
        rp = np.asarray(tvuer.right_hand_positions[0], dtype=float)
        if np.all(np.isfinite(lp)) and np.all(np.isfinite(rp)):
            if np.linalg.norm(lp) > 0.02 or np.linalg.norm(rp) > 0.02:
                return lp, rp, "openxr"
        return None

    @staticmethod
    def _angle2(a, b) -> float:
        import numpy as np

        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na < 1e-4 or nb < 1e-4:
            return 0.0
        a /= na
        b /= nb
        cross = a[0] * b[1] - a[1] * b[0]
        dot = float(np.clip(a[0] * b[0] + a[1] * b[1], -1.0, 1.0))
        return float(np.arctan2(cross, dot))

    def update_from_tvuer(self, tvuer, tele_data=None) -> None:
        if not self._seen_tracking:
            self._seen_tracking = True
            self._reset_session()

        np = self._np
        self._frame += 1
        H = self._mat4(tvuer.head_pose)
        wrists = self._read_wrists(tvuer, tele_data)

        if self._head_valid(H):
            self._head_bad_streak = 0
            if self._source != "head":
                self._source = "head"
                self._ref_center = None
                self._ref_lr_plane = None
                self._ref_t = None
                self._reset_head_locomotion_state(self._loco_state)
                self._calib_pos = None
                self._last_update_t = None
            if self.mode == "velocity":
                self._update_head_velocity(H)
            elif self.mode == "displacement":
                self._update_head_displacement(H)
            else:
                self._update_head(H[:3, :3])
            return

        self._head_bad_streak += 1
        if wrists is None or not self.hand_fallback:
            if self._frame in (1, 60, 180) or (self.debug and self._frame % 120 == 0):
                lp = np.asarray(tvuer.left_hand_positions[0], dtype=float)
                print(
                    "[xr_launcher] loco: no head (det=0) and no wrist stream. "
                    f"hand0={lp.round(3).tolist()} — "
                    "Do Isaac arms move when you move VP hands?",
                    flush=True,
                )
            if self._frame > 180 and not self._announced_no_stream:
                self._announced_no_stream = True
                print(
                    "[xr_launcher] AVP Safari is NOT streaming pose data to the PC.\n"
                    "  VR hand lines can be local-only. Use Step 2 instead:\n"
                    "    Terminal 2: ./run_xr_teleop.sh\n"
                    "    Terminal 3: ./scripts/wholebody_walk_test.py",
                    flush=True,
                )
            return

        if self._head_bad_streak > 15 and self._source != "hands":
            self._source = "hands"
            self._calib_until = None
            self._ref_center = None
            self._ref_lr_plane = None
            if not self._announced_hands:
                print(
                    "[xr_launcher] Using HAND loco (same wrists as arm teleop):\n"
                    "  push both hands forward/back to walk, twist shoulders to turn.",
                    flush=True,
                )
                self._announced_hands = True

        left_p, right_p, frame = wrists
        self._hand_frame = frame
        self._update_hands(left_p, right_p, frame)

    def _update_head_velocity(self, H) -> None:
        """work-master: head motion derivative in robot frame -> walk velocity."""
        now = time.time()
        if self._last_update_t is None:
            self._last_update_t = now
            return
        dt = max(now - self._last_update_t, 1e-3)
        self._last_update_t = now

        head_pose = self._head_pose_from_openxr(H)
        if self._calib_pos is None:
            self._calib_pos = head_pose[:3, 3].copy()
            calib_rot = head_pose[:3, :3].copy()
            calib_yaw = self._horizontal_heading_from_calib_rot(calib_rot)
            self._reset_head_locomotion_state(self._loco_state, calib_rot=calib_rot)
            print(
                "[xr_launcher] HEAD_LOCO calibrated at "
                f"{self._np.round(self._calib_pos, 3).tolist()} "
                f"facing_yaw={calib_yaw:.3f} rad",
                flush=True,
            )

        vx, vy, vyaw = self._compute_head_locomotion_velocity(
            head_pose,
            self._loco_state,
            self._loco_cfg,
            dt,
            calib_pos=self._calib_pos,
        )
        calib_rot = self._loco_state.calib_rot
        vel_wh = self._np.asarray(
            self._loco_state.debug.get("vel_world_h") or [0.0, 0.0, 0.0],
            dtype=float,
        )
        wh_norm = float(self._np.linalg.norm(vel_wh[:2]))
        local_speed = float(self._np.hypot(vx, vy))
        if calib_rot is not None:
            if wh_norm > 1e-6 and local_speed > 1e-6:
                wx, wy = (vel_wh[:2] / wh_norm * local_speed).tolist()
            else:
                wx, wy = self._walk_vector_to_world_rot(calib_rot, vx, vy)
        else:
            wx, wy = vx, vy
        debug = self._loco_state.debug
        raw_vel = debug.get("raw_vel") or [0.0, 0.0]
        self._publish(
            wx,
            wy,
            vyaw,
            now,
            "head/velocity",
            float(raw_vel[0]),
            float(raw_vel[1]),
            float(debug.get("raw_vyaw", 0.0)),
        )

    def _update_head_displacement(self, H) -> None:
        """Head translation in space -> bounded walk velocity (not head tilt)."""
        np = self._np
        t = H[:3, 3]
        R = H[:3, :3]
        now = time.time()

        if self._calib_until is None:
            self._calib_until = now + self.calib_sec
            print(
                "[xr_launcher] HEAD_LOCO calibrating (displacement)... "
                "hold head still at neutral",
                flush=True,
            )

        if now < self._calib_until:
            self._ref_t = t.copy()
            self._ref_R = R.copy()
            self._publish(0.0, 0.0, 0.0, now, "head/disp", 0.0, 0.0, 0.0)
            return

        if self._ref_t is None or self._ref_R is None:
            self._ref_t = t.copy()
            self._ref_R = R.copy()
            return

        delta_w = t - self._ref_t
        delta_h = self._ref_R.T @ delta_w
        # OpenXR head frame at calib: x right, y up, z toward back of head.
        forward_m = self.disp_fwd_sign * (-float(delta_h[2]))
        lateral_m = self.disp_lat_sign * float(delta_h[0])
        forward_m = self._deadzone(forward_m, self.disp_dead)
        lateral_m = self._deadzone(lateral_m, self.disp_dead)

        vx = self._clamp(self.disp_kp * self.disp_scale * forward_m, self.v_lin)
        vy = self._clamp(self.disp_kp_lat * self.disp_scale * lateral_m, self.v_lat)
        self._publish(vx, vy, 0.0, now, "head/disp", forward_m, lateral_m, 0.0)

    def _update_head(self, R) -> None:
        np = self._np
        if self._last_R is not None and np.allclose(R, self._last_R, atol=1e-5, rtol=0):
            self._stale_count += 1
        else:
            self._stale_count = 0
        self._last_R = R.copy()

        now = time.time()
        if self._calib_until is None:
            self._calib_until = now + self.calib_sec
            print("[xr_launcher] HEAD_LOCO calibrating (head)... keep neutral", flush=True)

        if now < self._calib_until:
            self._ref_R = R.copy()
            self._publish(0.0, 0.0, 0.0, now, "head/tilt", 0.0, 0.0, 0.0)
            return

        if self._ref_R is None:
            self._ref_R = R.copy()
            return

        if self._stale_count > 90 and not self._warned_stale:
            self._warned_stale = True
            print(
                "[xr_launcher] WARNING: head pose stopped updating; will try hand fallback.",
                flush=True,
            )

        R_rel = self._ref_R.T @ R
        yaw = np.arctan2(R_rel[1, 0], R_rel[0, 0])
        pitch = np.arctan2(-R_rel[2, 0], np.hypot(R_rel[0, 0], R_rel[1, 0]))
        self._publish_from_angles(pitch, yaw, now, "head")

    def _update_hands(self, left_p, right_p, frame="openxr") -> None:
        np = self._np
        center = (left_p + right_p) * 0.5
        if frame == "robot":
            lr_plane = np.array([left_p[1] - right_p[1], left_p[2] - right_p[2]], dtype=float)
        else:
            lr_plane = np.array([left_p[0] - right_p[0], left_p[2] - right_p[2]], dtype=float)

        now = time.time()
        if self._calib_until is None:
            self._calib_until = now + self.calib_sec
            print(
                f"[xr_launcher] HEAD_LOCO calibrating ({frame})... hold neutral pose",
                flush=True,
            )

        if now < self._calib_until:
            self._ref_center = center.copy()
            self._ref_lr_plane = lr_plane.copy()
            self._publish(0.0, 0.0, 0.0, now, f"hands/{frame}", 0.0, 0.0, 0.0)
            return

        if self._ref_center is None or self._ref_lr_plane is None:
            self._ref_center = center.copy()
            self._ref_lr_plane = lr_plane.copy()
            return

        if frame == "robot":
            pitch = float(center[0] - self._ref_center[0]) * self.hand_gain
        else:
            pitch = -float(center[2] - self._ref_center[2]) * self.hand_gain
        yaw = self._angle2(self._ref_lr_plane, lr_plane)
        self._publish_from_angles(pitch, yaw, now, f"hands/{frame}")

    def _publish_from_angles(self, pitch, yaw, now, source) -> None:
        pitch = self._deadzone(pitch, self.pitch_dead)
        yaw = self._deadzone(yaw, self.yaw_dead)
        vx = self._clamp(self.pitch_sign * self.pitch_gain * pitch, self.v_lin)
        vyaw = self._clamp(self.yaw_sign * self.yaw_gain * yaw, self.v_yaw)
        self._publish(vx, 0.0, vyaw, now, "head/tilt", pitch, 0.0, yaw)

    def _publish(self, vx, vy, vyaw, now, source, fwd, lat, yaw) -> None:
        cmd = [
            round(float(vx), 4),
            round(float(-vy), 4),
            round(float(-vyaw), 4),
            self.height,
        ]
        cmd_str = str(cmd)
        try:
            self._send(cmd_str)
        except Exception as exc:
            print(f"[xr_launcher] HEAD_LOCO transport failed: {exc}", flush=True)
            return
        if self.debug and now - self._last_print > 0.5:
            tag = "MOVE" if (vx or vy or vyaw) else "idle"
            cmd_dbg = self._loco_state.debug.get("cmd") if self.mode == "velocity" else None
            extra = f" cmd={cmd_dbg}" if cmd_dbg else ""
            print(
                f"[xr_launcher] loco [{source}/{tag}] "
                f"fwd={fwd:+.3f} lat={lat:+.3f} yaw={yaw:+.3f}{extra} -> {cmd_str}",
                flush=True,
            )
            self._last_print = now

    def update(self, head_pose_4x4):
        """Legacy entry."""
        return

    def on_tracking_start(self, tvuer, tele_data=None) -> None:
        self._seen_tracking = False
        self.update_from_tvuer(tvuer, tele_data)


def _patch_tv_wrapper(module) -> None:
    global _TV_WRAPPER_PATCHED
    if _TV_WRAPPER_PATCHED or not hasattr(module, "TeleVuerWrapper"):
        return
    _TV_WRAPPER_PATCHED = True

    orig_init = module.TeleVuerWrapper.__init__

    def wrapper_init(self, *args, **kwargs):
        if XR_CLIENT_NATIVE:
            if not AVP_ENDPOINT:
                raise SystemExit(
                    "XR_CLIENT=native requires AVP_ENDPOINT (room code like MLBS-4109 "
                    "or Vision Pro IP on the same LAN)."
                )
            if XR_FORCE_ZMQ:
                kwargs["img_shape"] = [SIM_H, STEREO_W]
                kwargs["binocular"] = True
            else:
                kwargs["img_shape"] = [SIM_H, SIM_MONO_W]
                kwargs["binocular"] = False
            kwargs["webrtc"] = False
            kwargs["zmq"] = True
            self.use_hand_tracking = kwargs.get("use_hand_tracking", args[0] if args else True)
            self.return_hand_rot_data = kwargs.get("return_hand_rot_data", False)
            if args:
                self.use_hand_tracking = args[0]
            sys.path.insert(0, SCRIPT_DIR)
            from avp_native_televuer import AvpNativeTeleVuer

            webrtc_port = int(os.environ.get("AVP_WEBRTC_PORT", "9999"))
            self.tvuer = AvpNativeTeleVuer.create(
                endpoint=AVP_ENDPOINT,
                img_shape=tuple(kwargs["img_shape"]),
                display_mode=kwargs.get("display_mode", "immersive"),
                display_fps=float(kwargs.get("display_fps", 30.0)),
                stereo_video=bool(kwargs.get("binocular", False)),
                ht_backend=os.environ.get("AVP_HT_BACKEND", "grpc"),
                webrtc_port=webrtc_port,
            )
            print(
                "[xr_launcher] XR client=native (Tracking Streamer app, NOT Safari 8012)",
                flush=True,
            )
        elif XR_FORCE_ZMQ:
            kwargs["webrtc"] = False
            kwargs["zmq"] = True
            kwargs["img_shape"] = [SIM_H, STEREO_W]
            kwargs["binocular"] = True
            orig_init(self, *args, **kwargs)
        else:
            kwargs["img_shape"] = [SIM_H, SIM_MONO_W]
            kwargs["binocular"] = False
            orig_init(self, *args, **kwargs)

        if XR_CLIENT_NATIVE:
            mode = "avp_native_webrtc"
        else:
            mode = "zmq" if self.tvuer.zmq and not self.tvuer.webrtc else "webrtc"
        LOG.info(
            "TeleVuer video path: %s display_mode=%s img_shape=%s",
            mode,
            self.tvuer.display_mode,
            list(self.tvuer.img_shape[:2]),
        )
        print(
            f"[xr_launcher] video path={mode} "
            f"display_mode={self.tvuer.display_mode} img_shape={list(self.tvuer.img_shape[:2])}",
            flush=True,
        )

    module.TeleVuerWrapper.__init__ = wrapper_init

    if XR_HEAD_LOCO:
        global _HEAD_LOCO
        _HEAD_LOCO = _HeadLoco()
        orig_get_tele_data = module.TeleVuerWrapper.get_tele_data

        def get_tele_data(self):
            tele_data = orig_get_tele_data(self)
            try:
                if _HEAD_LOCO is not None:
                    _HEAD_LOCO.update_from_tvuer(self.tvuer, tele_data)
            except Exception as exc:
                print(f"[xr_launcher] HEAD_LOCO error: {exc}", flush=True)
            return tele_data

        module.TeleVuerWrapper.get_tele_data = get_tele_data


def _force_patch_televuer():
    """Ensure patches apply even when teleop uses `from televuer import TeleVuerWrapper`."""
    import importlib

    for mod_name in ("televuer.tv_wrapper", "televuer"):
        try:
            mod = importlib.import_module(mod_name)
        except ImportError:
            continue
        if mod_name == "televuer.tv_wrapper":
            _patch_tv_wrapper(mod)
        elif hasattr(mod, "TeleVuerWrapper"):
            tw = importlib.import_module("televuer.tv_wrapper")
            mod.TeleVuerWrapper = tw.TeleVuerWrapper


def _maybe_patch_loaded(name: str) -> None:
    module = sys.modules.get(name)
    if module is None:
        return
    if name.endswith("image_client"):
        _patch_image_client(module)
    elif name == "televuer.televuer":
        _patch_televuer(module)
    elif name == "televuer.tv_wrapper":
        _patch_tv_wrapper(module)
    elif name == "televuer":
        for sub in ("televuer.televuer", "televuer.tv_wrapper"):
            _maybe_patch_loaded(sub)


_orig_import = builtins.__import__


def _patched_import(name, globals=None, locals=None, fromlist=(), level=0):
    module = _orig_import(name, globals, locals, fromlist, level)
    _maybe_patch_loaded(name)
    return module


builtins.__import__ = _patched_import

_force_patch_televuer()

def _patch_sim_reset_key() -> None:
    """Add [b] in sim teleop to reset robot+scene without restarting Isaac."""
    if "--sim" not in sys.argv:
        return
    try:
        import sshkeyboard
    except ImportError:
        return

    orig_listen = sshkeyboard.listen_keyboard
    _reset_pub = {"pub": None}

    def _send_reset() -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        if _reset_pub["pub"] is None:
            domain = int(os.environ.get("SIM_DDS_DOMAIN", "1"))
            net_if = os.environ.get("SIM_DDS_IFACE", "")
            if net_if:
                ChannelFactoryInitialize(domain, net_if)
            else:
                ChannelFactoryInitialize(domain)
            pub = ChannelPublisher("rt/reset_pose/cmd", String_)
            pub.Init()
            _reset_pub["pub"] = pub
        _reset_pub["pub"].Write(String_(data="2"))
        stop_cmd = str([0.0, 0.0, 0.0, float(os.environ.get("HEAD_LOCO_HEIGHT", "0.8"))])
        if _HEAD_LOCO is not None:
            try:
                _HEAD_LOCO._send(stop_cmd)
            except Exception:
                pass
        print("[xr_launcher] [b] sent sim reset (category 2: robot + scene)", flush=True)

    def listen_keyboard(on_press=None, on_release=None, **kwargs):
        def wrapped_press(key):
            if key == "b":
                try:
                    _send_reset()
                except Exception as exc:
                    print(f"[xr_launcher] sim reset failed: {exc}", flush=True)
            if on_press is not None:
                on_press(key)

        return orig_listen(on_press=wrapped_press, on_release=on_release, **kwargs)

    sshkeyboard.listen_keyboard = listen_keyboard


_patch_sim_reset_key()

os.chdir(TELEOP_DIR)
sys.argv = ["teleop_hand_and_arm.py", *sys.argv[1:]]
runpy.run_path(os.path.join(TELEOP_DIR, "teleop_hand_and_arm.py"), run_name="__main__")
