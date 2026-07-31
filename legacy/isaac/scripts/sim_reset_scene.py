#!/usr/bin/env python3
"""Reset a running Isaac sim without restarting run_sim_wholebody.sh.

Publishes to DDS topic rt/reset_pose/cmd (same as Unitree reset_pose_test.py).

Categories (with our launch_sim_main patch):
  1  reset cylinder/object only
  2  reset entire scene — robot back to spawn + object (use after fall)
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_TV_PYTHON = os.environ.get("TV_PYTHON", "/mnt/newssd/conda_envs/tv/bin/python")
if os.path.exists(_TV_PYTHON) and os.path.realpath(sys.executable) != os.path.realpath(_TV_PYTHON):
    os.execv(_TV_PYTHON, [_TV_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

DOMAIN = int(os.environ.get("SIM_DDS_DOMAIN", "1"))
NET_IF = os.environ.get("SIM_DDS_IFACE", "")
DEFAULT_HEIGHT = float(os.environ.get("WALK_HEIGHT", "0.8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Reset Isaac sim scene via DDS.")
    parser.add_argument(
        "--category",
        type=int,
        choices=(1, 2),
        default=2,
        help="1=object only, 2=full scene reset (default, use after robot fall)",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Also send stand-still walk command [0,0,0,height]",
    )
    args = parser.parse_args()

    if NET_IF:
        ChannelFactoryInitialize(DOMAIN, NET_IF)
    else:
        ChannelFactoryInitialize(DOMAIN)

    pub = ChannelPublisher("rt/reset_pose/cmd", String_)
    pub.Init()
    pub.Write(String_(data=str(args.category)))
    print(f"[reset] Published rt/reset_pose/cmd category={args.category}", flush=True)
    if args.category == 2:
        print("[reset] Terminal 1 should print: reset all", flush=True)
    else:
        print("[reset] Terminal 1 should print: reset object", flush=True)

    if args.stop:
        walk_pub = ChannelPublisher("rt/run_command/cmd", String_)
        walk_pub.Init()
        stop_cmd = str([0.0, 0.0, 0.0, DEFAULT_HEIGHT])
        walk_pub.Write(String_(data=stop_cmd))
        print(f"[reset] Sent stand-still walk cmd: {stop_cmd}", flush=True)

    time.sleep(0.2)


if __name__ == "__main__":
    main()
