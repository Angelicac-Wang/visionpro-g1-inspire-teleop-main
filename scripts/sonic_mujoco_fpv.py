"""Stream MuJoCo head_camera (ZMQ :5555) to Vision Pro via Tracking Streamer WebRTC."""

from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np

GR00T_ROOT = os.environ.get("GR00T_ROOT", "/mnt/newssd/GR00T-WholeBodyControl")
if GR00T_ROOT not in sys.path:
    sys.path.insert(0, GR00T_ROOT)


class MujocoFpvStreamer:
    """Pull ego_view from SONIC MuJoCo sim and push frames into VisionProStreamer."""

    def __init__(
        self,
        streamer,
        *,
        camera_host: str = "127.0.0.1",
        camera_port: int = 5555,
        camera_name: str = "ego_view",
        webrtc_port: int = 9999,
        video_size: str = "1280x720",
        fps: int = 30,
    ):
        from gear_sonic.camera.composed_camera import ComposedCameraClientSensor

        self._streamer = streamer
        self._camera_name = camera_name
        self._client = ComposedCameraClientSensor(server_ip=camera_host, port=camera_port)
        self._video_w, self._video_h = (int(x) for x in video_size.lower().split("x"))
        self._frames_sent = 0
        self._last_warn = 0.0
        self._started = False

        streamer.configure_video(
            device=None,
            size=video_size,
            fps=max(int(fps), 1),
            stereo=False,
        )
        streamer.start_webrtc(port=int(webrtc_port), blocking=False)
        self._started = True
        print(
            f"[mujoco_fpv] WebRTC video -> Vision Pro on port {webrtc_port} "
            f"({video_size}, source tcp://{camera_host}:{camera_port}/{camera_name})"
        )
        print(
            "[mujoco_fpv] On Vision Pro Tracking Streamer: enable video / immersive view "
            "if the app shows a separate video toggle."
        )

    def push_latest(self) -> bool:
        data = self._client.read(blocking=False)
        if not data or not data.get("images"):
            now = time.time()
            if now - self._last_warn > 5.0:
                print(
                    "[mujoco_fpv] Waiting for MuJoCo camera frames. "
                    "Start Terminal 1 with SONIC_ENABLE_FPV=1 ./run_sonic_sim_loop.sh"
                )
                self._last_warn = now
            return False

        images = data["images"]
        frame_rgb = images.get(self._camera_name)
        if frame_rgb is None:
            frame_rgb = next(iter(images.values()))

        frame = np.asarray(frame_rgb)
        if frame.ndim == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if frame.shape[1] != self._video_w or frame.shape[0] != self._video_h:
            frame = cv2.resize(frame, (self._video_w, self._video_h))

        self._streamer.update_frame(frame)
        self._frames_sent += 1
        if self._frames_sent == 1:
            print(f"[mujoco_fpv] First frame sent shape={frame.shape}")
        return True

    def close(self) -> None:
        try:
            self._client.close()
        except Exception:
            pass
