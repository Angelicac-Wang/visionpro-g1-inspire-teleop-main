"""Repository and external SDK path helpers."""

from __future__ import annotations

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")


def ensure_repo_on_path() -> None:
    if REPO_ROOT not in sys.path:
        sys.path.insert(0, REPO_ROOT)


def ensure_scripts_on_path() -> None:
    ensure_repo_on_path()
    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)


def unitree_sim_root() -> str:
    return os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")


def visionpro_teleop_root() -> str:
    sim = unitree_sim_root()
    return os.environ.get(
        "VISIONPRO_TELEOP_ROOT",
        os.path.join(sim, "inspire_hand_ws", "VisionProTeleop"),
    )


def inspire_hand_sdk_root() -> str:
    sim = unitree_sim_root()
    return os.environ.get(
        "INSPIRE_HAND_SDK_ROOT",
        os.path.join(sim, "inspire_hand_ws", "inspire_hand_sdk"),
    )
