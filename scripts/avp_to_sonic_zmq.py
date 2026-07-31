#!/usr/bin/env python3
"""Entry point: AVP tracking → SONIC ZMQ bridge."""

import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from g1_teleop.bridge.runtime import main

if __name__ == "__main__":
    main()
