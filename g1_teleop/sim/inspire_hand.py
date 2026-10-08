"""Receive the bridge's Inspire commands and drive MuJoCo's six active joints."""

from __future__ import annotations

import json
import time

import numpy as np
import zmq

from g1_teleop.bridge.constants import HEADER_SIZE

CHANNEL_JOINTS = ("little_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1")
OPEN_COMMAND = np.full(6, 1000.0)


def decode_command(message: bytes, topic: bytes = b"inspire_hand") -> dict[str, np.ndarray]:
    """Decode one packed message; malformed/NaN commands never reach physics."""
    if not message.startswith(topic) or len(message) < len(topic) + HEADER_SIZE:
        raise ValueError("Invalid Inspire topic/header")
    header = json.loads(message[len(topic):len(topic) + HEADER_SIZE].rstrip(b"\0"))
    if not isinstance(header, dict) or header.get("v") != 1 or header.get("count") != 1 or header.get("endian") != "le":
        raise ValueError("Unsupported Inspire packet version/layout")
    payload = message[len(topic) + HEADER_SIZE:]
    offset = 0
    hands = {}
    for field in header["fields"]:
        dtype = np.dtype({"f32": "<f4", "f64": "<f8"}[field["dtype"]])
        shape = field["shape"]
        if not isinstance(shape, list) or len(shape) != 1 or shape[0] not in (1, 6):
            raise ValueError("Invalid Inspire field shape")
        count = shape[0]
        values = np.frombuffer(payload, dtype=dtype, count=count, offset=offset)
        offset += count * dtype.itemsize
        if field["name"] in ("left", "right"):
            if count != 6 or not np.isfinite(values).all() or field["name"] in hands:
                raise ValueError("Invalid Inspire hand values")
            hands[field["name"]] = np.clip(values.astype(float), 0.0, 1000.0)
    if set(hands) != {"left", "right"} or offset != len(payload):
        raise ValueError("Incomplete Inspire command")
    return hands


class InspireHandController:
    def __init__(self, model, data, endpoint="tcp://127.0.0.1:5556", *, connect=True):
        self.model = model
        self.data = data
        self.indices = {}
        self.active = {}
        self.targets = {}
        self.commands = {side: OPEN_COMMAND.copy() for side in ("left", "right")}
        self.last_received = None
        self.received = 0
        self.rejected = 0
        self.timed_out = False
        self.context = None
        self.socket = None
        for side in self.commands:
            joints = [j for j in range(model.njnt) if model.joint(j).name.startswith(f"{side}_hand_")]
            if len(joints) != 12:
                raise ValueError(f"Expected 12 Inspire {side} joints, got {len(joints)}")
            active = [model.joint(f"{side}_hand_{name}_joint").id for name in CHANNEL_JOINTS]
            self.indices[side] = np.array(joints)
            self.active[side] = np.array(active)
            self.targets[side] = data.qpos[model.jnt_qposadr[active]].copy()
        if connect:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"inspire_hand")
            self.socket.connect(endpoint)
            print(f"[inspire_sim] Five-finger FTP hands listening on {endpoint}/inspire_hand", flush=True)

    def receive(self, message: bytes, now: float) -> bool:
        try:
            commands = decode_command(message)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.rejected += 1
            return False
        self.commands = commands
        self.last_received = now
        self.received += 1
        if self.received == 1 or self.timed_out:
            print("[inspire_sim] Hand commands received; finger tracking active.", flush=True)
        self.timed_out = False
        return True

    def torques(self, dt: float, now: float | None = None) -> np.ndarray:
        now = time.monotonic() if now is None else now
        if self.socket is not None:
            try:
                self.receive(self.socket.recv(zmq.NOBLOCK), now)
            except zmq.Again:
                pass
        if self.last_received is not None and now - self.last_received > 0.5:
            self.commands = {side: OPEN_COMMAND.copy() for side in self.commands}
            if not self.timed_out:
                print("[inspire_sim] Hand command timeout; opening fingers.", flush=True)
            self.timed_out = True
        result = []
        for side, joints in self.indices.items():
            active = self.active[side]
            limits = self.model.jnt_range[active]
            # Inspire hardware convention: 1000=open, 0=closed, all six channels.
            desired = limits[:, 0] + (1.0 - self.commands[side] / 1000.0) * (limits[:, 1] - limits[:, 0])
            self.targets[side] += np.clip(desired - self.targets[side], -3.0 * dt, 3.0 * dt)
            q = self.data.qpos[self.model.jnt_qposadr[active]]
            dq = self.data.qvel[self.model.jnt_dofadr[active]]
            tau = np.clip(2.0 * (self.targets[side] - q) - 0.08 * dq, -1.4, 1.4)
            hand_tau = np.zeros(len(joints))
            hand_tau[np.searchsorted(joints, active)] = tau
            result.append(hand_tau)
        return np.concatenate(result)

    def reset(self):
        self.commands = {side: OPEN_COMMAND.copy() for side in self.commands}
        self.last_received = None
        for side, active in self.active.items():
            self.targets[side] = self.data.qpos[self.model.jnt_qposadr[active]].copy()

    def close(self):
        if self.socket is not None:
            self.socket.close(0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None


def install_inspire_sim(endpoint="tcp://127.0.0.1:5556"):
    """Adapt only this launcher's MuJoCo instance; no physical hand DDS output."""
    import gear_sonic.utils.mujoco_sim.base_sim as base_sim
    from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

    original_env = base_sim.DefaultEnv
    original_create = SimulatorFactory._create_mujoco_simulator
    original_close = base_sim.BaseSimulator.close

    class InspireEnv(original_env):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.torque_limit = np.max(np.abs(self.mj_model.actuator_ctrlrange), axis=1)
            self.inspire = InspireHandController(self.mj_model, self.mj_data, endpoint)

        def compute_hand_torques(self):
            return self.inspire.torques(self.sim_dt)

        def reset(self):
            super().reset()
            if hasattr(self, "inspire"):
                self.inspire.reset()

    def create(config, env_name="default", **kwargs):
        config = dict(config)
        config.update(NUM_HAND_JOINTS=12, NUM_HAND_MOTORS=0)
        # DDS continues to carry the 29 body motors. Dex3 hand commands are
        # intentionally ignored; the Inspire subscriber owns finger torques.
        return original_create(config, env_name, **kwargs)

    def close(sim):
        env = getattr(sim, "sim_env", None)
        controller = getattr(env, "inspire", None)
        if controller is not None:
            controller.close()
        original_close(sim)
        # Free EGL resources before Python tears down the OpenGL modules.
        renderers = getattr(env, "renderers", {})
        for renderer in renderers.values():
            renderer.close()
        renderers.clear()

    base_sim.DefaultEnv = InspireEnv
    SimulatorFactory._create_mujoco_simulator = staticmethod(create)
    base_sim.BaseSimulator.close = close
