#!/usr/bin/env python3
"""Run GR00T MuJoCo sim loop without double-initializing CycloneDDS.

run_sim_loop.py calls init_channel() in the parent process and again inside
BaseSimulator. The second init fails on a Singleton domain and can destabilize
sim<->deploy DDS on some setups. BaseSimulator's init is sufficient.
"""

from __future__ import annotations

import os

import gear_sonic.scripts.run_sim_loop as run_sim_loop
import gear_sonic.utils.mujoco_sim.simulator_factory as simulator_factory
import tyro

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig


def _patch_robot_scene_loader() -> None:
    """Allow SONIC_ROBOT_SCENE env var to point at an absolute MJCF outside GR00T."""
    override = os.environ.get("SONIC_ROBOT_SCENE")
    if not override:
        return

    import gear_sonic.utils.mujoco_sim.base_sim as base_sim

    original = base_sim.DefaultEnv.init_scene

    def init_scene(self):
        saved_root = base_sim.GEAR_SONIC_ROOT
        saved_scene = self.config["ROBOT_SCENE"]
        try:
            base_sim.GEAR_SONIC_ROOT = "."
            self.config = dict(self.config)
            self.config["ROBOT_SCENE"] = override
            return original(self)
        finally:
            base_sim.GEAR_SONIC_ROOT = saved_root
            if isinstance(self.config, dict):
                self.config["ROBOT_SCENE"] = saved_scene

    base_sim.DefaultEnv.init_scene = init_scene


def main() -> None:
    _patch_robot_scene_loader()
    simulator_factory.init_channel = lambda config: None  # noqa: ARG005
    config = tyro.cli(SimLoopConfig)
    run_sim_loop.main(config)


if __name__ == "__main__":
    main()
