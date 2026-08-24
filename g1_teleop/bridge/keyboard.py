"""Terminal keyboard input and planner state for manual walk testing."""

from __future__ import annotations

import select
import subprocess
import sys
import termios
import tty
from dataclasses import dataclass, field

import numpy as np

MOVEMENT_KEYS = frozenset({"w", "s", "a", "d", ",", "."})
TURN_KEYS = frozenset({"j", "l", "q", "e"})
HELD_KEYS = MOVEMENT_KEYS | TURN_KEYS


class RawKeyboard:
    def __init__(self):
        self._stty_saved: str | None = None

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        try:
            self._stty_saved = subprocess.check_output(
                ["stty", "-g"],
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip()
            # Fast repeat while held (~100 ms delay/interval) for hold-to-move.
            subprocess.run(["stty", "time", "1", "min", "1"], check=False)
        except (OSError, subprocess.SubprocessError):
            self._stty_saved = None
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._stty_saved:
            subprocess.run(f"stty {self._stty_saved}", shell=True, check=False)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_keys(self) -> list[str]:
        keys = []
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return keys
            keys.append(sys.stdin.read(1))


@dataclass
class KeyboardHoldTracker:
    """Track which movement/turn keys are currently held (via autorepeat refresh)."""

    hold_timeout_s: float = 0.16
    _last_seen: dict[str, float] = field(default_factory=dict)

    def refresh(self, keys: list[str], now: float) -> None:
        for key in keys:
            lowered = key.lower()
            if lowered in HELD_KEYS:
                self._last_seen[lowered] = now

    def held(self, now: float) -> set[str]:
        active = {
            key
            for key, seen_at in self._last_seen.items()
            if now - seen_at <= self.hold_timeout_s
        }
        self._last_seen = {key: self._last_seen[key] for key in active}
        return active

    def clear(self) -> None:
        self._last_seen.clear()


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


def handle_key(
    state: BridgeState,
    key: str,
    *,
    head_locomotion: bool = False,
    hybrid_locomotion: bool = False,
    keyboard_walk_speed: float = 0.42,
) -> bool:
    """Handle non-hold planner keys (calibration / stop bridge). Movement uses hold tracker when hybrid."""
    if key == "0":
        state.set_idle()
    elif key == "1":
        state.set_mode(1)
    elif key == "2":
        state.set_mode(2)
    elif key == "3":
        state.set_mode(3)
    elif key in (" ", "r", "R"):
        if not hybrid_locomotion:
            state.movement[:] = 0.0
            state.speed = -1.0
    elif head_locomotion and not hybrid_locomotion:
        turn_step = np.pi / 12.0
        if key in ("j", "J", "q", "Q", "a", "A"):
            state.facing_angle += turn_step
        elif key in ("l", "L", "e", "E", "d", "D"):
            state.facing_angle -= turn_step
        elif key in ("w", "W"):
            if state.mode == 0:
                state.set_mode(1)
            state.movement[:] = state.facing
            state.speed = keyboard_walk_speed
        elif key in ("s", "S"):
            if state.mode == 0:
                state.set_mode(1)
            state.movement[:] = -state.facing
            state.speed = keyboard_walk_speed
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
