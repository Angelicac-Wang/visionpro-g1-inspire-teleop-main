#!/usr/bin/env python3
"""Keyboard driver for the G1 whole-body sim walking policy.

Uses the same command format as Unitree's official send_commands_keyboard.py:
  rt/run_command/cmd  ->  "[vx, -vy, -vyaw, height]"

Delivery path (most reliable first):
  1) shared memory  isaac_run_command_cmd  (same-machine, no DDS discovery issues)
  2) DDS topic      rt/run_command/cmd     (fallback)

Run AFTER ./run_sim_wholebody.sh is fully loaded and you clicked the Isaac window.
"""

from __future__ import annotations

import os
import sys
import threading
import time

_TV_PYTHON = os.environ.get("TV_PYTHON", "/mnt/newssd/conda_envs/tv/bin/python")
if os.path.exists(_TV_PYTHON) and os.path.realpath(sys.executable) != os.path.realpath(_TV_PYTHON):
    os.execv(_TV_PYTHON, [_TV_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])

UNITREE_SIM_ROOT = os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")
sys.path.insert(0, UNITREE_SIM_ROOT)

from sshkeyboard import listen_keyboard, stop_listening
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from dds.sharedmemorymanager import SharedMemoryManager

SHM_NAME = "isaac_run_command_cmd"
TOPIC = "rt/run_command/cmd"
DOMAIN = int(os.environ.get("SIM_DDS_DOMAIN", "1"))
NET_IF = os.environ.get("SIM_DDS_IFACE", "")
DEFAULT_HEIGHT = float(os.environ.get("WALK_HEIGHT", "0.8"))
INCREMENT = float(os.environ.get("WALK_INCREMENT", "0.05"))
PUB_HZ = 100.0

# internal state (same semantics as official keyboard controller)
_params = {"x_vel": 0.0, "y_vel": 0.0, "yaw_vel": 0.0, "height": 0.0}
_keys = {"w": False, "s": False, "a": False, "d": False, "z": False, "x": False}
_lock = threading.Lock()
_running = True
_last_cmd = None


def _patch_shared_memory_attach() -> None:
    """Allow attaching to a named segment created by the sim."""
    from multiprocessing import shared_memory as _shm

    _orig = SharedMemoryManager.__init__

    def _fixed_init(self, name=None, size=512):
        import threading

        self.size = size
        self.lock = threading.RLock()
        if name:
            try:
                self.shm = _shm.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
            except FileNotFoundError:
                raise FileNotFoundError(name)
        else:
            self.shm = _shm.SharedMemory(create=True, size=size)
            self.shm_name = self.shm.name
            self.created = True

    SharedMemoryManager.__init__ = _fixed_init


def _wait_for_sim_shm(timeout_s: float = 180.0) -> SharedMemoryManager | None:
    deadline = time.time() + timeout_s
    while time.time() < deadline and _running:
        try:
            shm = SharedMemoryManager(SHM_NAME, 512)
            print(f"[walk] Connected to sim shared memory: {SHM_NAME}", flush=True)
            return shm
        except FileNotFoundError:
            time.sleep(1.0)
    return None


def _update_params() -> None:
    with _lock:
        p = _params
        k = _keys

        if k["w"]:
            p["x_vel"] = min(p["x_vel"] + INCREMENT, 0.6)
        elif k["s"]:
            p["x_vel"] = max(p["x_vel"] - INCREMENT, -0.3)
        elif p["x_vel"] > 0:
            p["x_vel"] = max(0.0, p["x_vel"] - INCREMENT * 2)
        elif p["x_vel"] < 0:
            p["x_vel"] = min(0.0, p["x_vel"] + INCREMENT * 2)

        if k["a"]:
            p["y_vel"] = max(p["y_vel"] - INCREMENT, -0.3)
        elif k["d"]:
            p["y_vel"] = min(p["y_vel"] + INCREMENT, 0.3)
        elif p["y_vel"] > 0:
            p["y_vel"] = max(0.0, p["y_vel"] - INCREMENT * 2)
        elif p["y_vel"] < 0:
            p["y_vel"] = min(0.0, p["y_vel"] + INCREMENT * 2)

        if k["z"]:
            p["yaw_vel"] = max(p["yaw_vel"] - INCREMENT, -0.3)
        elif k["x"]:
            p["yaw_vel"] = min(p["yaw_vel"] + INCREMENT, 0.3)
        elif p["yaw_vel"] > 0:
            p["yaw_vel"] = max(0.0, p["yaw_vel"] - INCREMENT * 2)
        elif p["yaw_vel"] < 0:
            p["yaw_vel"] = min(0.0, p["yaw_vel"] + INCREMENT * 2)


def _build_command() -> list[float]:
    with _lock:
        height = DEFAULT_HEIGHT + _params["height"]
        # official sign convention from send_commands_keyboard.py
        return [
            round(_params["x_vel"], 4),
            round(-_params["y_vel"], 4),
            round(-_params["yaw_vel"], 4),
            round(height, 4),
        ]


def on_press(key: str) -> None:
    if key in _keys:
        with _lock:
            if not _keys[key]:
                _keys[key] = True
                print(f"[walk] key {key} down", flush=True)
    elif key == "space":
        with _lock:
            for k in _params:
                _params[k] = 0.0
        print("[walk] space -> stop", flush=True)
    elif key == "q":
        global _running
        _running = False
        stop_listening()


def on_release(key: str) -> None:
    if key in _keys:
        with _lock:
            _keys[key] = False


def _publisher_loop(shm: SharedMemoryManager | None, pub: ChannelPublisher | None) -> None:
    global _last_cmd
    period = 1.0 / PUB_HZ
    while _running:
        _update_params()
        cmd = _build_command()
        cmd_str = str(cmd)
        payload = {"run_command": cmd_str}

        if shm is not None:
            shm.write_data(payload)
        if pub is not None:
            pub.Write(String_(data=cmd_str))

        if cmd != _last_cmd and any(abs(v) > 1e-4 for v in cmd[:3]):
            print(f"[walk] sending cmd={cmd_str}", flush=True)
            _last_cmd = cmd.copy()
        time.sleep(period)


def main() -> None:
    _patch_shared_memory_attach()

    print("=" * 60)
    print("G1 whole-body WALK test (keyboard -> sim walking policy)")
    print("=" * 60)
    print("IMPORTANT: run this in its OWN terminal.")
    print("  Terminal 1: ./run_sim_wholebody.sh   (sim only)")
    print("  Terminal 2: ./scripts/wholebody_walk_test.py   (THIS script)")
    print("Do NOT press w/a/d in run_xr_teleop.sh — that program ignores them.")
    print()
    print("Click THIS terminal, then HOLD keys:")
    print("  w/s = forward/back   a/d = strafe   z/x = turn   space = stop   q = quit")
    print("=" * 60)

    print("[walk] Waiting for sim shared memory (restart sim if this times out)...", flush=True)
    shm = _wait_for_sim_shm()
    if shm is None:
        print(
            f"[walk] ERROR: sim shared memory '{SHM_NAME}' not found.\n"
            "  1) Stop and restart Terminal 1: ./run_sim_wholebody.sh\n"
            "  2) Wait for 'create dds success' and click the Isaac window\n"
            "  3) Run this script again",
            flush=True,
        )
        raise SystemExit(1)

    if NET_IF:
        ChannelFactoryInitialize(DOMAIN, NET_IF)
    else:
        ChannelFactoryInitialize(DOMAIN)
    pub = ChannelPublisher(TOPIC, String_)
    pub.Init()
    print(f"[walk] DDS publisher ready on {TOPIC} (domain {DOMAIN})", flush=True)

    pub_thread = threading.Thread(target=_publisher_loop, args=(shm, pub), daemon=True)
    pub_thread.start()

    try:
        listen_keyboard(on_press=on_press, on_release=on_release, sequential=False)
    finally:
        global _running
        _running = False
        time.sleep(0.15)
        stop_cmd = str([0.0, 0.0, 0.0, DEFAULT_HEIGHT])
        if shm is not None:
            shm.write_data({"run_command": stop_cmd})
        if pub is not None:
            pub.Write(String_(data=stop_cmd))
        print("\n[walk] Stopped. Sent stand-still command.", flush=True)


if __name__ == "__main__":
    main()
