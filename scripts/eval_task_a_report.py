#!/usr/bin/env python3
"""Summarize REMS Task A eval CSV (IMU yaw metrics + waypoint times)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from g1_teleop.eval.task_a import compute_task_a_metrics, format_task_a_report, load_eval_csv


def main() -> None:
    parser = argparse.ArgumentParser(description="Report Task A metrics from bridge eval CSV.")
    parser.add_argument("csv", help="Eval log from --eval-log")
    parser.add_argument(
        "--imu-condition",
        default="unknown",
        help="Label for ablation runs, e.g. imu_on / imu_off",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional path to write metrics JSON",
    )
    args = parser.parse_args()

    rows = load_eval_csv(args.csv)
    metrics = compute_task_a_metrics(rows, imu_condition=args.imu_condition)
    report = format_task_a_report(metrics)
    print(report)

    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "csv": str(Path(args.csv).resolve()),
            "imu_condition": metrics.imu_condition,
            "duration_s": metrics.duration_s,
            "samples": metrics.samples,
            "imu_samples": metrics.imu_samples,
            "yaw_err_mean_abs_rad": metrics.yaw_err_mean_abs,
            "yaw_err_max_abs_rad": metrics.yaw_err_max_abs,
            "facing_corr_mean_abs_rad": metrics.facing_corr_mean_abs,
            "waypoint_times_s": metrics.waypoint_times_s,
        }
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
