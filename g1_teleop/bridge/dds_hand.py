"""Unitree DDS publisher for physical Inspire hand."""

from __future__ import annotations

import os
import sys

import numpy as np

from g1_teleop.paths import inspire_hand_sdk_root, unitree_sim_root


class DdsInspireHandPublisher:
    def __init__(self, topic_side: str, dds_network: str | None):
        unitree_sdk2_root = os.environ.get(
            "UNITREE_SDK2_ROOT",
            os.path.join(unitree_sim_root(), "unitree_sdk2_python"),
        )
        for path in (unitree_sdk2_root, inspire_hand_sdk_root()):
            if path not in sys.path:
                sys.path.insert(0, path)

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from inspire_sdkpy import inspire_dds, inspire_hand_defaut

        if dds_network:
            ChannelFactoryInitialize(0, dds_network)
        else:
            ChannelFactoryInitialize(0)

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
