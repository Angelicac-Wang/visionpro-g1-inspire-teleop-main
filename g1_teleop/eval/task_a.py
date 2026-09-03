"""REMS evaluation helpers (Task A corridor route + IMU metrics)."""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from g1_teleop.calibration.session import FeedbackSnapshot, ZmqFeedbackClient
from g1_teleop.locomotion.head import horizontal_yaw_from_quat_wxyz, wrap_to_pi


@dataclass(frozen=True)
class Waypoint:
    index: int
    x: float
    y: float
    label: str


# REMS Task A — 3.0 m × 2.2 m corridor, pelvis frame (x forward, y left).
TASK_A_WAYPOINTS: tuple[Waypoint, ...] = (
    Waypoint(1, 0.40, 0.00, "entry"),
    Waypoint(2, 0.90, 0.45, "left_branch"),
    Waypoint(3, 1.50, 0.00, "mid_corridor"),
    Waypoint(4, 2.10, -0.45, "right_branch"),
    Waypoint(5, 2.80, 0.00, "far_end"),
)

TASK_A_CORRIDOR_LENGTH_M = 3.0
TASK_A_CORRIDOR_WIDTH_M = 2.2


def _quat_wxyz(snapshot: FeedbackSnapshot | None) -> np.ndarray | None:
    if snapshot is None or snapshot.base_quat is None:
        return None
    return np.asarray(snapshot.base_quat, dtype=np.float64).reshape(4)


class RemsEvalLogger:
    """Append-only CSV logger for bridge-side locomotion + IMU telemetry."""

    FIELDNAMES = [
        "t_mono",
        "t_wall",
        "mode",
        "move_x",
        "move_y",
        "speed",
        "face_x",
        "face_y",
        "head_active",
        "kb_active",
        "imu_active",
        "robot_yaw_rel",
        "yaw_err",
        "facing_corr",
        "head_cmd_vx",
        "head_cmd_vy",
        "kb_cmd_vx",
        "kb_cmd_vy",
        "base_qw",
        "base_qx",
        "base_qy",
        "base_qz",
        "delta_heading",
        "waypoint_mark",
    ]

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()
        self._imu_zero_yaw: float | None = None
        self._waypoint_marks: list[tuple[int, float]] = []

    def close(self) -> None:
        self._file.close()

    def mark_waypoint(self, index: int) -> None:
        self._waypoint_marks.append((int(index), time.monotonic()))

    @property
    def waypoint_marks(self) -> list[tuple[int, float]]:
        return list(self._waypoint_marks)

    def write_row(
        self,
        *,
        mode: int,
        movement: np.ndarray,
        facing: np.ndarray,
        speed: float,
        head_active: bool,
        kb_active: bool,
        imu_debug: dict | None,
        head_cmd_debug: dict | None,
        kb_cmd_debug: dict | None,
        feedback: FeedbackSnapshot | None,
        waypoint_mark: int | None = None,
    ) -> None:
        imu = imu_debug or {}
        head_dbg = head_cmd_debug or {}
        kb_dbg = kb_cmd_debug or {}
        base_quat = _quat_wxyz(feedback)
        robot_yaw_rel = imu.get("robot_yaw_rel")
        if robot_yaw_rel is None and base_quat is not None:
            yaw = horizontal_yaw_from_quat_wxyz(base_quat)
            if self._imu_zero_yaw is None:
                self._imu_zero_yaw = yaw
            robot_yaw_rel = round(wrap_to_pi(yaw - self._imu_zero_yaw), 4)

        row = {
            "t_mono": round(time.monotonic(), 4),
            "t_wall": round(time.time(), 4),
            "mode": int(mode),
            "move_x": round(float(movement[0]), 4),
            "move_y": round(float(movement[1]), 4),
            "speed": round(float(speed), 4),
            "face_x": round(float(facing[0]), 4),
            "face_y": round(float(facing[1]), 4),
            "head_active": int(head_active),
            "kb_active": int(kb_active),
            "imu_active": int(bool(imu.get("imu_active"))),
            "robot_yaw_rel": robot_yaw_rel,
            "yaw_err": imu.get("yaw_err"),
            "facing_corr": imu.get("facing_corr"),
            "head_cmd_vx": head_dbg.get("cmd", [None, None])[0] if isinstance(head_dbg.get("cmd"), list) else None,
            "head_cmd_vy": head_dbg.get("cmd", [None, None])[1] if isinstance(head_dbg.get("cmd"), list) else None,
            "kb_cmd_vx": kb_dbg.get("vx"),
            "kb_cmd_vy": kb_dbg.get("vy"),
            "base_qw": round(float(base_quat[0]), 5) if base_quat is not None else None,
            "base_qx": round(float(base_quat[1]), 5) if base_quat is not None else None,
            "base_qy": round(float(base_quat[2]), 5) if base_quat is not None else None,
            "base_qz": round(float(base_quat[3]), 5) if base_quat is not None else None,
            "delta_heading": feedback.delta_heading if feedback is not None else None,
            "waypoint_mark": waypoint_mark,
        }
        self._writer.writerow(row)
        self._file.flush()


@dataclass
class TaskAMetrics:
    duration_s: float
    samples: int
    imu_samples: int
    yaw_err_mean_abs: float | None
    yaw_err_max_abs: float | None
    facing_corr_mean_abs: float | None
    waypoint_times_s: dict[int, float] = field(default_factory=dict)
    imu_condition: str = "unknown"


def _parse_float(value: object) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_eval_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def compute_task_a_metrics(
    rows: list[dict[str, str]],
    *,
    imu_condition: str = "unknown",
) -> TaskAMetrics:
    if not rows:
        return TaskAMetrics(
            duration_s=0.0,
            samples=0,
            imu_samples=0,
            yaw_err_mean_abs=None,
            yaw_err_max_abs=None,
            facing_corr_mean_abs=None,
            imu_condition=imu_condition,
        )

    t0 = _parse_float(rows[0].get("t_mono")) or 0.0
    t1 = _parse_float(rows[-1].get("t_mono")) or t0
    yaw_errs: list[float] = []
    facing_corrs: list[float] = []
    imu_samples = 0
    waypoint_times: dict[int, float] = {}

    for row in rows:
        if row.get("imu_active") in ("1", "True", "true"):
            imu_samples += 1
        yaw_err = _parse_float(row.get("yaw_err"))
        if yaw_err is not None:
            yaw_errs.append(abs(yaw_err))
        facing_corr = _parse_float(row.get("facing_corr"))
        if facing_corr is not None:
            facing_corrs.append(abs(facing_corr))
        mark = row.get("waypoint_mark")
        if mark not in (None, ""):
            idx = int(float(mark))
            t = (_parse_float(row.get("t_mono")) or 0.0) - t0
            waypoint_times.setdefault(idx, t)

    return TaskAMetrics(
        duration_s=max(t1 - t0, 0.0),
        samples=len(rows),
        imu_samples=imu_samples,
        yaw_err_mean_abs=(float(np.mean(yaw_errs)) if yaw_errs else None),
        yaw_err_max_abs=(float(np.max(yaw_errs)) if yaw_errs else None),
        facing_corr_mean_abs=(float(np.mean(facing_corrs)) if facing_corrs else None),
        waypoint_times_s=waypoint_times,
        imu_condition=imu_condition,
    )


def format_task_a_report(metrics: TaskAMetrics) -> str:
    lines = [
        f"Task A metrics ({metrics.imu_condition})",
        f"  duration_s: {metrics.duration_s:.2f}",
        f"  samples: {metrics.samples} (imu_active={metrics.imu_samples})",
    ]
    if metrics.yaw_err_mean_abs is not None:
        lines.append(f"  |yaw_err| mean: {np.degrees(metrics.yaw_err_mean_abs):.2f} deg")
        lines.append(f"  |yaw_err| max:  {np.degrees(metrics.yaw_err_max_abs or 0.0):.2f} deg")
    if metrics.facing_corr_mean_abs is not None:
        lines.append(f"  |facing_corr| mean: {np.degrees(metrics.facing_corr_mean_abs):.2f} deg")
    if metrics.waypoint_times_s:
        lines.append("  waypoint reach times (s from log start):")
        for idx in sorted(metrics.waypoint_times_s):
            lines.append(f"    WP{idx}: {metrics.waypoint_times_s[idx]:.2f}")
    else:
        lines.append("  waypoint marks: none (press 4-8 during teleop to mark WP1-5)")
    return "\n".join(lines)


class TaskAFeedbackLogger:
    """Standalone g1_debug subscriber for IMU-only trials."""

    def __init__(self, host: str, port: int, topic: str, path: str | Path):
        self.client = ZmqFeedbackClient(host, port, topic)
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=["t_mono", "base_qw", "base_qx", "base_qy", "base_qz", "delta_heading"],
        )
        self._writer.writeheader()
        self._file.flush()
        self._zero_yaw: float | None = None

    def close(self) -> None:
        self.client.close()
        self._file.close()

    def poll_and_write(self) -> bool:
        snap = self.client.poll()
        if snap is None or snap.base_quat is None:
            return False
        q = np.asarray(snap.base_quat, dtype=np.float64).reshape(4)
        yaw = horizontal_yaw_from_quat_wxyz(q)
        if self._zero_yaw is None:
            self._zero_yaw = yaw
        self._writer.writerow(
            {
                "t_mono": round(time.monotonic(), 4),
                "base_qw": round(float(q[0]), 5),
                "base_qx": round(float(q[1]), 5),
                "base_qy": round(float(q[2]), 5),
                "base_qz": round(float(q[3]), 5),
                "delta_heading": round(wrap_to_pi(yaw - self._zero_yaw), 5),
            }
        )
        self._file.flush()
        return True
