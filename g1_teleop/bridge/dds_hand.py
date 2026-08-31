"""Unitree DDS publisher for physical Inspire hand(s)."""

from __future__ import annotations

import os
import sys

import numpy as np

from g1_teleop.paths import inspire_hand_sdk_root, unitree_sim_root


def _init_dds(dds_network: str | None) -> None:
    unitree_sdk2_root = os.environ.get(
        "UNITREE_SDK2_ROOT",
        os.path.join(unitree_sim_root(), "unitree_sdk2_python"),
    )
    for path in (unitree_sdk2_root, inspire_hand_sdk_root()):
        if path not in sys.path:
            sys.path.insert(0, path)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if dds_network:
        ChannelFactoryInitialize(0, dds_network)
    else:
        ChannelFactoryInitialize(0)


def _normalize_sides(sides: str | tuple[str, ...]) -> tuple[str, ...]:
    if isinstance(sides, str):
        if sides == "both":
            return ("l", "r")
        return (sides,)
    return tuple(sides)


class DdsInspireHandPublisher:
    """Publish to a single rt/inspire_hand/ctrl/{l|r} topic."""

    def __init__(self, topic_side: str, dds_network: str | None):
        _init_dds(dds_network)

        from unitree_sdk2py.core.channel import ChannelPublisher
        from inspire_sdkpy import inspire_dds, inspire_hand_defaut

        self._hand_default = inspire_hand_defaut
        self._publisher = ChannelPublisher(
            f"rt/inspire_hand/ctrl/{topic_side}",
            inspire_dds.inspire_hand_ctrl,
        )
        self._publisher.Init()

    def send(self, values: np.ndarray):
        cmd = self._hand_default.get_inspire_hand_ctrl()
        cmd.angle_set = [int(v) for v in np.asarray(values, dtype=np.int16).tolist()]
        cmd.mode = 0b0001
        return self._publisher.Write(cmd)


class DdsInspireHandsPublisher:
    """Publish AVP left/right finger commands to one or both physical hands."""

    def __init__(self, sides: str | tuple[str, ...], dds_network: str | None):
        _init_dds(dds_network)

        from unitree_sdk2py.core.channel import ChannelPublisher
        from inspire_sdkpy import inspire_dds, inspire_hand_defaut

        self._hand_default = inspire_hand_defaut
        self._publishers: dict[str, object] = {}
        for side in _normalize_sides(sides):
            publisher = ChannelPublisher(
                f"rt/inspire_hand/ctrl/{side}",
                inspire_dds.inspire_hand_ctrl,
            )
            publisher.Init()
            self._publishers[side] = publisher

    @property
    def sides(self) -> tuple[str, ...]:
        return tuple(self._publishers.keys())

    def send_side(self, side: str, values: np.ndarray):
        publisher = self._publishers.get(side)
        if publisher is None:
            return None
        cmd = self._hand_default.get_inspire_hand_ctrl()
        cmd.angle_set = [int(v) for v in np.asarray(values, dtype=np.int16).tolist()]
        cmd.mode = 0b0001
        return publisher.Write(cmd)

    def send_physical_hands(
        self,
        *,
        left_command: np.ndarray,
        right_command: np.ndarray,
        mode: str = "both",
    ):
        if mode in ("both", "l"):
            self.send_side("l", left_command)
        if mode in ("both", "r"):
            self.send_side("r", right_command)

    def safe_open(self, open_command: np.ndarray):
        for side in self.sides:
            self.send_side(side, open_command)
