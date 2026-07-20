#!/usr/bin/env python3
"""Run GR00T MuJoCo sim loop without double-initializing CycloneDDS.

run_sim_loop.py calls init_channel() in the parent process and again inside
BaseSimulator. The second init fails on a Singleton domain and can destabilize
sim<->deploy DDS on some setups. BaseSimulator's init is sufficient.
"""

from __future__ import annotations

import gear_sonic.scripts.run_sim_loop as run_sim_loop
import gear_sonic.utils.mujoco_sim.simulator_factory as simulator_factory
import tyro

from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig


def main() -> None:
    simulator_factory.init_channel = lambda config: None  # noqa: ARG005
    config = tyro.cli(SimLoopConfig)
    run_sim_loop.main(config)


if __name__ == "__main__":
    main()
