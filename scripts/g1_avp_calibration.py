"""Backward-compatible shim — prefer `g1_teleop.calibration.session`."""

import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from g1_teleop.calibration.session import *  # noqa: F403
