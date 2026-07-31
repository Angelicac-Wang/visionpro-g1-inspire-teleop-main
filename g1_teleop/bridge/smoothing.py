"""VR target smoothing and quaternion utilities."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as SciRotation
from scipy.spatial.transform import Slerp


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def _slerp_quat_wxyz(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    t = float(np.clip(t, 0.0, 1.0))
    q0 = _normalize_quat_wxyz(q0)
    q1 = _normalize_quat_wxyz(q1)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    if t <= 0.0:
        return q0.copy()
    if t >= 1.0:
        return q1.copy()
    r0 = SciRotation.from_quat([q0[1], q0[2], q0[3], q0[0]])
    r1 = SciRotation.from_quat([q1[1], q1[2], q1[3], q1[0]])
    blended = Slerp([0.0, 1.0], SciRotation.concatenate([r0, r1]))([t])[0]
    out = blended.as_quat()
    return np.array([out[3], out[0], out[1], out[2]], dtype=np.float64)


def _quat_geodesic_rad(q0: np.ndarray, q1: np.ndarray) -> float:
    q0 = _normalize_quat_wxyz(q0)
    q1 = _normalize_quat_wxyz(q1)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    r0 = SciRotation.from_quat([q0[1], q0[2], q0[3], q0[0]])
    r1 = SciRotation.from_quat([q1[1], q1[2], q1[3], q1[0]])
    return float((r0.inv() * r1).magnitude())


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


class OrientationSmoother:
    """Low-pass + angular speed cap for left/right/head quaternions (wxyz x3)."""

    def __init__(self):
        self.orientation = None
        self.last_time = None

    def reset(self):
        self.orientation = None
        self.last_time = None

    def update(
        self,
        target: np.ndarray,
        now: float,
        tau: float,
        max_angular_speed: float,
    ) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64).reshape(12)
        if self.orientation is None or self.last_time is None:
            self.orientation = target.copy()
            self.last_time = now
            for i in range(0, 12, 4):
                self.orientation[i : i + 4] = _normalize_quat_wxyz(self.orientation[i : i + 4])
            return self.orientation.copy()

        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        alpha = 1.0 if tau <= 1e-6 else 1.0 - np.exp(-dt / tau)
        out = self.orientation.copy()
        for i in range(0, 12, 4):
            q_curr = self.orientation[i : i + 4]
            q_tgt = _normalize_quat_wxyz(target[i : i + 4])
            step_t = alpha
            if max_angular_speed > 0.0:
                full_angle = _quat_geodesic_rad(q_curr, q_tgt)
                if full_angle > 1e-8:
                    step_t = min(step_t, (max_angular_speed * dt) / full_angle)
            out[i : i + 4] = _slerp_quat_wxyz(q_curr, q_tgt, step_t)
        self.orientation = out
        return self.orientation.copy()
