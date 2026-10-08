"""Manual AVP tracking probe; importing this module does not connect."""
import argparse
import time


def main():
    from avp_stream import VisionProStreamer

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ip", nargs="?", default="192.168.2.5")
    args = parser.parse_args()
    streamer = VisionProStreamer(ip=args.ip)
    while True:
        frame = streamer.latest
        print(frame["head"][0][:3, 3], frame["right_wrist"][0][:3, 3])
        time.sleep(0.1)


if __name__ == "__main__":
    main()
