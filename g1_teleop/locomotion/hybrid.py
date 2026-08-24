"""Merge head-driven and keyboard locomotion for SONIC planner."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from g1_teleop.locomotion.head import HeadLocomotionConfig, SonicPlannerCommand, wrap_to_pi


class KeyboardPhase(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    STOPPING = "stopping"
    TURNING = "turning"


class MoveIntent(str, Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    STRAFE_L = "strafe_l"
    STRAFE_R = "strafe_r"


def _unit_angle(angle: float) -> np.ndarray:
    return np.array([np.cos(angle), np.sin(angle), 0.0], dtype=np.float64)


def _lateral_left(forward: np.ndarray) -> np.ndarray:
    return np.array([-forward[1], forward[0], 0.0], dtype=np.float64)


def _lateral_right(forward: np.ndarray) -> np.ndarray:
    return np.array([forward[1], -forward[0], 0.0], dtype=np.float64)


def _is_walking(cmd: SonicPlannerCommand | None) -> bool:
    if cmd is None or cmd.mode == 0:
        return False
    if cmd.speed > 0.0 and float(np.linalg.norm(cmd.movement[:2])) > 1e-6:
        return True
    # In-place turn: positive speed with facing != movement (or zero movement).
    return cmd.speed > 0.0


@dataclass
class KeyboardLocomotionController:
    """Hold-to-move keyboard locomotion with stop-before-turn / stop-before-direction-change."""

    body_facing_angle: float = 0.0
    phase: KeyboardPhase = KeyboardPhase.IDLE
    move_intent: MoveIntent | None = None
    smooth_vx: float = 0.0
    smooth_vy: float = 0.0
    smooth_turn_speed: float = 0.0
    has_control: bool = False
    stop_requested: bool = False
    pending_imu_sync: bool = False
    turn_rate: float = 0.48
    turn_in_place_speed: float = 0.20
    turn_lead_angle: float = 1.5707963267948966  # pi/2 — tight spot turn

    def sync_body_facing(self, angle: float) -> None:
        self.body_facing_angle = wrap_to_pi(float(angle))

    def request_stop(self) -> None:
        self.stop_requested = True

    def reset_motion(self) -> None:
        self.phase = KeyboardPhase.IDLE
        self.move_intent = None
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self.smooth_turn_speed = 0.0
        self.stop_requested = False

    def output_facing(self) -> np.ndarray:
        return _unit_angle(self.body_facing_angle)

    def _speed(self) -> float:
        return float(np.hypot(self.smooth_vx, self.smooth_vy))

    def _is_stopped(self, cfg: HeadLocomotionConfig) -> bool:
        return (
            self._speed() < max(cfg.output_deadzone, 0.05)
            and self.smooth_turn_speed < max(cfg.output_deadzone, 0.05)
        )

    def _decay_velocity(self, cfg: HeadLocomotionConfig) -> None:
        decay = float(np.clip(cfg.idle_decay, 0.0, 1.0))
        self.smooth_vx *= decay
        self.smooth_vy *= decay
        self.smooth_turn_speed *= decay
        if self._speed() < cfg.output_deadzone:
            self.smooth_vx = 0.0
            self.smooth_vy = 0.0
        if self.smooth_turn_speed < cfg.output_deadzone:
            self.smooth_turn_speed = 0.0

    def _set_target_velocity(
        self,
        direction: np.ndarray,
        speed: float,
        cfg: HeadLocomotionConfig,
        alpha: float,
    ) -> None:
        direction = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(direction[:2]))
        if norm < 1e-6:
            return
        direction = direction.copy()
        direction[:2] /= norm
        direction[2] = 0.0
        target_vx = float(direction[0] * speed)
        target_vy = float(direction[1] * speed)
        self.smooth_vx += alpha * (target_vx - self.smooth_vx)
        self.smooth_vy += alpha * (target_vy - self.smooth_vy)

    def _move_key_intent(self, held: set[str]) -> MoveIntent | None:
        if "w" in held:
            return MoveIntent.FORWARD
        if "s" in held:
            return MoveIntent.BACKWARD
        if "," in held:
            return MoveIntent.STRAFE_L
        if "." in held:
            return MoveIntent.STRAFE_R
        return None

    def _turn_sign(self, held: set[str]) -> int:
        if any(key in held for key in ("a", "j", "q")):
            return 1
        if any(key in held for key in ("d", "l", "e")):
            return -1
        return 0

    def _movement_and_facing_for_intent(
        self,
        intent: MoveIntent,
    ) -> tuple[np.ndarray, np.ndarray]:
        """SONIC decoupled control: body facing stays forward unless turning."""
        body_facing = self.output_facing()
        if intent == MoveIntent.FORWARD:
            return body_facing.copy(), body_facing.copy()
        if intent == MoveIntent.BACKWARD:
            # Moonwalk: travel opposite to body heading, body keeps facing forward.
            return -body_facing, body_facing.copy()
        if intent == MoveIntent.STRAFE_L:
            return _lateral_left(body_facing), body_facing.copy()
        return _lateral_right(body_facing), body_facing.copy()

    def _command_for_intent(
        self,
        intent: MoveIntent,
        cfg: HeadLocomotionConfig,
        locomotion_mode: int,
        walk_speed: float,
    ) -> SonicPlannerCommand:
        speed = min(max(self._speed(), cfg.output_deadzone * 1.5), cfg.max_speed)
        movement, facing = self._movement_and_facing_for_intent(intent)
        return SonicPlannerCommand(
            mode=locomotion_mode,
            movement=movement,
            facing=facing,
            speed=speed,
        )

    def _command_for_turn(
        self,
        turn_sign: int,
        cfg: HeadLocomotionConfig,
        locomotion_mode: int,
        dt: float,
        alpha: float,
    ) -> SonicPlannerCommand:
        """Spot turn: facing leads body by 90° with minimal forward stepping."""
        loop_dt = max(float(dt), 1e-3)
        self.body_facing_angle = wrap_to_pi(
            self.body_facing_angle + turn_sign * self.turn_rate * loop_dt
        )
        body_facing = self.output_facing()
        target_angle = wrap_to_pi(self.body_facing_angle + turn_sign * self.turn_lead_angle)
        target_facing = _unit_angle(target_angle)

        target_turn_speed = min(self.turn_in_place_speed, cfg.max_speed)
        self.smooth_turn_speed += alpha * (target_turn_speed - self.smooth_turn_speed)
        turn_speed = min(max(self.smooth_turn_speed, cfg.output_deadzone * 1.5), cfg.max_speed)

        return SonicPlannerCommand(
            mode=locomotion_mode,
            movement=body_facing.copy(),
            facing=target_facing,
            speed=turn_speed,
        )

    def update(
        self,
        held: set[str],
        cfg: HeadLocomotionConfig,
        dt: float,
        *,
        locomotion_mode: int = 1,
        walk_speed: float = 0.42,
        smooth_alpha: float = 0.18,
    ) -> SonicPlannerCommand | None:
        alpha = float(np.clip(smooth_alpha, 0.05, 1.0))
        move_key = self._move_key_intent(held)
        turn_sign = self._turn_sign(held)

        if self.stop_requested:
            held = set()
            move_key = None
            turn_sign = 0

        if move_key is not None or turn_sign != 0:
            self.has_control = True

        if self.phase == KeyboardPhase.STOPPING:
            self._decay_velocity(cfg)
            if self._is_stopped(cfg):
                self.phase = KeyboardPhase.IDLE
                self.move_intent = None
                self.stop_requested = False
                self.pending_imu_sync = True
            return self._build_idle_command()

        if self.phase == KeyboardPhase.TURNING:
            if turn_sign == 0:
                self.phase = KeyboardPhase.STOPPING
                return self._build_idle_command()
            if not self._is_stopped(cfg) and self._speed() > cfg.output_deadzone:
                self.phase = KeyboardPhase.STOPPING
                return self._build_idle_command()
            self.smooth_vx = 0.0
            self.smooth_vy = 0.0
            return self._command_for_turn(turn_sign, cfg, locomotion_mode, dt, alpha)

        if self.phase == KeyboardPhase.MOVING:
            if move_key is None:
                self.phase = KeyboardPhase.STOPPING
                self.move_intent = None
            elif move_key != self.move_intent:
                self.phase = KeyboardPhase.STOPPING
                self.move_intent = None
            else:
                movement, _ = self._movement_and_facing_for_intent(self.move_intent)
                self._set_target_velocity(
                    movement,
                    min(walk_speed, cfg.max_speed),
                    cfg,
                    alpha,
                )
                self.smooth_turn_speed = 0.0
                return self._command_for_intent(self.move_intent, cfg, locomotion_mode, walk_speed)

            return self._build_idle_command()

        # IDLE
        if not self._is_stopped(cfg):
            self.phase = KeyboardPhase.STOPPING
            return self._build_idle_command()

        if move_key is not None:
            self.phase = KeyboardPhase.MOVING
            self.move_intent = move_key
            movement, _ = self._movement_and_facing_for_intent(move_key)
            self._set_target_velocity(movement, min(walk_speed, cfg.max_speed), cfg, alpha)
            self.smooth_turn_speed = 0.0
            return self._command_for_intent(move_key, cfg, locomotion_mode, walk_speed)

        if turn_sign != 0:
            self.phase = KeyboardPhase.TURNING
            self.smooth_vx = 0.0
            self.smooth_vy = 0.0
            return self._command_for_turn(turn_sign, cfg, locomotion_mode, dt, alpha)

        self.stop_requested = False
        return self._build_idle_command()

    def _build_idle_command(self) -> SonicPlannerCommand:
        return SonicPlannerCommand(
            mode=0,
            movement=np.zeros(3, dtype=np.float64),
            facing=self.output_facing(),
            speed=-1.0,
        )


def keyboard_planner_command(
    controller: KeyboardLocomotionController,
    held: set[str],
    cfg: HeadLocomotionConfig,
    dt: float,
    *,
    locomotion_mode: int = 1,
    default_speed: float = 0.42,
    smooth_alpha: float = 0.18,
) -> SonicPlannerCommand | None:
    return controller.update(
        held,
        cfg,
        dt,
        locomotion_mode=locomotion_mode,
        walk_speed=default_speed,
        smooth_alpha=smooth_alpha,
    )


def merge_hybrid_planner_commands(
    head_cmd: SonicPlannerCommand | None,
    kb_cmd: SonicPlannerCommand | None,
    cfg: HeadLocomotionConfig,
    *,
    locomotion_mode: int = 1,
    kb_has_control: bool = False,
    kb_facing: np.ndarray | None = None,
) -> SonicPlannerCommand | None:
    """Keyboard owns movement when active; preserve keyboard body facing when idle."""
    head_active = _is_walking(head_cmd)
    kb_active = _is_walking(kb_cmd)

    if kb_active and kb_cmd is not None:
        return SonicPlannerCommand(
            mode=locomotion_mode,
            movement=np.asarray(kb_cmd.movement, dtype=np.float64).reshape(3).copy(),
            facing=np.asarray(kb_cmd.facing, dtype=np.float64).reshape(3).copy(),
            speed=float(kb_cmd.speed),
            height=head_cmd.height if head_cmd is not None else -1.0,
        )

    if head_active and head_cmd is not None:
        return head_cmd

    if kb_has_control and kb_facing is not None:
        return SonicPlannerCommand(
            mode=0,
            movement=np.zeros(3, dtype=np.float64),
            facing=np.asarray(kb_facing, dtype=np.float64).reshape(3).copy(),
            speed=-1.0,
            height=head_cmd.height if head_cmd is not None else -1.0,
        )

    if kb_cmd is not None:
        return kb_cmd

    if head_cmd is not None:
        return SonicPlannerCommand(
            mode=0,
            movement=np.zeros(3, dtype=np.float64),
            facing=np.asarray(head_cmd.facing, dtype=np.float64).reshape(3).copy(),
            speed=-1.0,
            height=head_cmd.height,
        )
    return None
