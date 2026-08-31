#!/usr/bin/env python3
"""Capture AVP open/close hand poses and save Inspire finger calibration JSON."""

from __future__ import annotations

import argparse
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from g1_teleop.bridge.cli import default_hand_calibration_file
from g1_teleop.hand.mapping import InspireHandMapper, run_hand_calibration
from g1_teleop.paths import ensure_scripts_on_path, visionpro_teleop_root

ensure_scripts_on_path()
vp_root = visionpro_teleop_root()
if vp_root not in sys.path:
    sys.path.insert(0, vp_root)

from avp_stream import VisionProStreamer  # noqa: E402


class _CalibArgs:
    """Minimal namespace for InspireHandMapper defaults during calibration."""

    thumb_rotation_metric = "angle"
    thumb_open_distance = 0.12
    thumb_close_distance = 0.035
    finger_mcp_weight = 0.45
    finger_pip_weight = 0.40
    finger_dip_weight = 0.15
    thumb_mcp_weight = 0.65
    thumb_ip_weight = 0.35
    flip_thumb_rotation = False
    finger_deadband = 0.015
    thumb_deadband = 0.02
    finger_smoothing = 0.45
    thumb_smoothing = 0.35
    finger_range_scale = 1.15
    thumb_bend_range_scale = 1.35
    thumb_rotation_range_scale = 1.25
    invert_thumb_rotation_command = True
    open_angle = 1000
    close_angle = 0
    thumb_bend_open_angle = 800
    thumb_bend_close_angle = 200
    thumb_rotation_open_angle = 200
    thumb_rotation_close_angle = 800


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate AVP left or right hand for Inspire mapping.")
    parser.add_argument("--avp-endpoint", required=True, help="Vision Pro IP or room code.")
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="Which AVP hand to sample during calibration.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Calibration JSON path. Default: scripts/visionpro_{side}_hand_calibration.json",
    )
    parser.add_argument(
        "--sample-seconds",
        type=float,
        default=2.0,
        help="Seconds to average for each open/close pose.",
    )
    args = parser.parse_args()
    output_path = args.output or default_hand_calibration_file(args.side)

    print(f"Connecting to Vision Pro at {args.avp_endpoint} ...")
    streamer = VisionProStreamer(ip=args.avp_endpoint, record=False, benchmark_quiet=True)
    mapper = InspireHandMapper(_CalibArgs())

    try:
        run_hand_calibration(
            streamer,
            mapper,
            args.sample_seconds,
            output_path,
            side=args.side,
        )
    finally:
        print("\nDone. Use in sim teleop, e.g.:")
        if args.side == "left":
            print(
                f"  ./run_sonic_avp_teleop_pick.sh <VP_IP> "
                f"--hand-calibration-file-left {output_path}"
            )
        else:
            print(
                f"  ./run_sonic_avp_teleop_pick.sh <VP_IP> "
                f"--hand-calibration-file-right {output_path}"
            )


if __name__ == "__main__":
    main()
