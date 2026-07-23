#!/usr/bin/env python3
"""AVP -> SONIC bridge with safe stand-hold before full teleop."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

BRIDGE = Path(__file__).resolve().parent / "avp_to_sonic_zmq.py"


def main() -> None:
    if not BRIDGE.exists():
        raise SystemExit(f"SONIC bridge not found: {BRIDGE}")

    argv = list(sys.argv)
    if "--active-hands" not in argv:
        argv.extend(["--active-hands", "both"])
    if "--auto-start" not in argv and "--no-auto-start" not in argv:
        argv.append("--no-auto-start")
    if "--head-locomotion" not in argv:
        argv.append("--head-locomotion")
    if "--head-height-squat" not in argv and "--no-head-height-squat" not in argv:
        argv.append("--head-height-squat")
    if "--enable-mujoco-fpv" not in argv and "--no-mujoco-fpv" not in argv:
        argv.append("--enable-mujoco-fpv")

    sys.argv = argv
    print(f"[g1_avp_sonic_teleop] Running {BRIDGE}")
    print("[g1_avp_sonic_teleop] Flow: F -> ] -> S -> T  (CALIB_FULL / ENGAGE / SYNC / TELEOP)")
    print("[g1_avp_sonic_teleop] MuJoCo FPV ON by default (needs SONIC_ENABLE_FPV sim). Duck/squat + P/H/S keys.")
    runpy.run_path(str(BRIDGE), run_name="__main__")


if __name__ == "__main__":
    main()
