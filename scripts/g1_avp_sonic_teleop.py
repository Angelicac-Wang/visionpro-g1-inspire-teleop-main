#!/usr/bin/env python3
"""AVP -> SONIC bridge with safe stand-hold before full teleop."""

from __future__ import annotations

import os
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
    if "--hybrid-smoothing-tau" not in argv:
        argv.extend(["--hybrid-smoothing-tau", "0.10"])
    if "--hybrid-max-speed" not in argv:
        argv.extend(["--hybrid-max-speed", "0.30"])
    if "--arm-orientation-smoothing-tau" not in argv:
        argv.extend(["--arm-orientation-smoothing-tau", "0.14"])
    if "--arm-max-angular-speed" not in argv:
        argv.extend(["--arm-max-angular-speed", "1.8"])
    if "--wrist-rotation-scale" not in argv:
        argv.extend(["--wrist-rotation-scale", "0.55"])
    if "--arm-transition-max-speed" not in argv:
        argv.extend(["--arm-transition-max-speed", "0.08"])
    if "--arm-transition-max-angular-speed" not in argv:
        argv.extend(["--arm-transition-max-angular-speed", "0.5"])
    if "--arm-transition-ramp-sec" not in argv:
        argv.extend(["--arm-transition-ramp-sec", "6.0"])
    if "--robot-init-pose" not in argv:
        init_pose = os.environ.get("SONIC_ROBOT_INIT_POSE", "arms-down")
        argv.extend(["--robot-init-pose", init_pose])

    sys.argv = argv
    print(f"[g1_avp_sonic_teleop] Running {BRIDGE}")
    print("[g1_avp_sonic_teleop] Flow: F -> ] -> S -> T  (CALIB_FULL / ENGAGE / SYNC / TELEOP)")
    print(
        "[g1_avp_sonic_teleop] Arm smoothing + safe S/T transition ramp "
        f"(transition max {0.08} m/s for 6s). Tune --arm-transition-max-speed."
    )
    runpy.run_path(str(BRIDGE), run_name="__main__")


if __name__ == "__main__":
    main()
