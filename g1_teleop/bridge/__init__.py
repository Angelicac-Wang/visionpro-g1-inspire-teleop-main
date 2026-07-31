"""AVP → SONIC ZMQ bridge package."""

__all__ = ["main"]


def main():
    from g1_teleop.bridge.runtime import main as _main

    return _main()
