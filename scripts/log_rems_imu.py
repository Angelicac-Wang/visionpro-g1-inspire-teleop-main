#!/usr/bin/env python3
"""Standalone IMU feedback logger for REMS Task A (runs beside teleop)."""

from __future__ import annotations

import argparse
import signal
import time

from g1_teleop.eval.task_a import TaskAFeedbackLogger


def main() -> None:
    parser = argparse.ArgumentParser(description="Log g1_debug base IMU to CSV for Task A.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5557)
    parser.add_argument("--topic", default="g1_debug")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--output", required=True, help="Output CSV path")
    args = parser.parse_args()

    logger = TaskAFeedbackLogger(args.host, args.port, args.topic, args.output)
    stop = False

    def _handle(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    period = 1.0 / max(args.rate, 1.0)
    print(f"Logging IMU feedback to {args.output} ({args.host}:{args.port}/{args.topic})")
    while not stop:
        logger.poll_and_write()
        time.sleep(period)

    logger.close()
    print("Done.")


if __name__ == "__main__":
    main()
