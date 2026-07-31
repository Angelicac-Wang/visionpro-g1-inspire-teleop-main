"""SONIC ZMQ packed-message publisher."""

from __future__ import annotations

import json
import struct
import time

import numpy as np
import zmq

from g1_teleop.bridge.constants import HEADER_SIZE


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
