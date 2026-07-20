#!/usr/bin/env python3
"""Quick check: sim camera config + ZMQ JPEG stream."""

import sys
import time

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.2.32"

sys.path.insert(0, "/mnt/newssd/unitree_sim_isaaclab/xr_teleoperate/teleop/teleimager/src")

from teleimager.image_client import ImageClient


def main() -> int:
    print(f"Checking camera server at {HOST}:60000 and ZMQ head port...")
    try:
        client = ImageClient(host=HOST, request_bgr=True)
    except Exception as exc:
        print(f"FAIL: cannot connect ImageClient: {exc}")
        print("Is Terminal 1 sim running and Isaac window clicked?")
        return 1

    cfg = client.get_cam_config()
    head = cfg["head_camera"]
    print("Head camera config:")
    print(f"  image_shape={head.get('image_shape')} binocular={head.get('binocular')}")
    print(f"  enable_zmq={head.get('enable_zmq')} zmq_port={head.get('zmq_port')}")
    print(f"  enable_webrtc={head.get('enable_webrtc')} webrtc_port={head.get('webrtc_port')}")

    ok_frames = 0
    null_frames = 0
    shape = None
    for i in range(60):
        frame = client.get_head_frame()
        if frame.bgr is not None:
            ok_frames += 1
            shape = frame.bgr.shape
            if ok_frames == 1:
                print(f"First frame: shape={shape} fps={frame.fps:.1f}")
        else:
            null_frames += 1
        time.sleep(0.05)

    client.close()
    print(f"Result: {ok_frames} frames with data, {null_frames} empty (2s window)")
    if ok_frames == 0:
        print("FAIL: no ZMQ frames. Sim image server not publishing on head port.")
        return 2
    print(f"OK: ZMQ stream alive, shape={shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
