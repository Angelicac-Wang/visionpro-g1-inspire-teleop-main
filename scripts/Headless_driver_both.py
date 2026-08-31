import argparse
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SINGLE_DRIVER = os.path.join(SCRIPT_DIR, "Headless_driver_r.py")


def _start_process(side: str, ip: str, device_id: int, dds_network: str | None) -> subprocess.Popen:
    cmd = [
        sys.executable,
        SINGLE_DRIVER,
        "--lr",
        side,
        "--ip",
        ip,
        "--device-id",
        str(device_id),
    ]
    if dds_network:
        cmd.extend(["--dds-network", dds_network])
    label = "left" if side == "l" else "right"
    topic = f"rt/inspire_hand/ctrl/{side}"
    print(f"Starting {label} hand driver: ip={ip} topic={topic} device_id={device_id}")
    return subprocess.Popen(cmd, cwd=SCRIPT_DIR)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run left/right Inspire hand drivers (topic l @ .210, topic r @ .211 by default)."
    )
    parser.add_argument("--left-ip", default=os.environ.get("INSPIRE_HAND_LEFT_IP", "192.168.123.210"))
    parser.add_argument("--right-ip", default=os.environ.get("INSPIRE_HAND_RIGHT_IP", "192.168.123.211"))
    parser.add_argument(
        "--dds-network",
        default=os.environ.get("INSPIRE_HAND_DDS_NETWORK") or os.environ.get("SONIC_NET_IF"),
        help="Optional DDS NIC, e.g. enp3s0.",
    )
    parser.add_argument("--left-device-id", type=int, default=1)
    parser.add_argument("--right-device-id", type=int, default=1)
    parser.add_argument("--sides", choices=("both", "l", "r"), default="both")
    args = parser.parse_args()

    processes: list[subprocess.Popen] = []
    try:
        if args.sides in ("both", "l"):
            processes.append(_start_process("l", args.left_ip, args.left_device_id, args.dds_network))
        if args.sides in ("both", "r"):
            processes.append(_start_process("r", args.right_ip, args.right_device_id, args.dds_network))
        if not processes:
            raise SystemExit("No hand side selected.")

        print("Dual Inspire hand drivers running. Press Ctrl+C to stop.")
        while True:
            for proc in processes:
                code = proc.poll()
                if code is not None:
                    raise SystemExit(f"Hand driver exited unexpectedly with code {code}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping Inspire hand drivers.")
    finally:
        for proc in processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
