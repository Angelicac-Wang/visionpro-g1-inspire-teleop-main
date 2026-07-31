"""Terminal keyboard input and planner state for manual walk testing."""

from __future__ import annotations

import select
import sys
import termios
import tty
from dataclasses import dataclass

import numpy as np


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
