# VisionPro G1 Inspire Teleop

Apple Vision Pro teleoperation scripts for controlling a Unitree G1 right arm and an Inspire dexterous hand.

This repository contains the control entrypoints, the improved AVP-to-Inspire hand mapping algorithm, requirements, and installation notes. It intentionally does not vendor the large Unitree, VisionProTeleop, or Inspire SDK trees; those are installed or referenced as external dependencies.

## What Is Included

Library code lives in the **`g1_teleop/`** Python package. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for module layout, data flow, and calibration workflow.

Install the package in editable mode (recommended for development):

```bash
pip install -e .
```

Entry scripts in `scripts/` are thin wrappers for shell launchers (`run_*.sh`).

- `g1_teleop/hand/mapping.py` (shim: `scripts/avp_inspire_hand_mapping.py`)  
  Improved hand mapping algorithm. It maps AVP hand skeleton joint geometry to six Inspire hand angle channels:

  ```text
  [little, ring, middle, index, thumb_bend, thumb_root_rotation]
  ```

- `scripts/visionpro_g1_right_arm_hand.py`  
  Main teleop script. It receives Vision Pro tracking data, computes the G1 right-arm IK target, and publishes Inspire hand DDS commands.

- `g1_teleop/bridge/` (entry: `scripts/avp_to_sonic_zmq.py`, `scripts/g1_avp_sonic_teleop.py`)  
  SONIC whole-body AVP bridge: staged calibration, head locomotion with IMU closed-loop yaw, MuJoCo FPV.

- `g1_teleop/locomotion/head.py`  
  Head-driven walk / facing / squat planner commands.

- `g1_teleop/calibration/session.py`  
  F/]/S/T calibration session and ZMQ deploy feedback.

- `scripts/Headless_driver_r.py`  
  Inspire hand Modbus driver. It subscribes to DDS control commands and writes them to the physical hand.

- `docs/VISIONPRO_G1_HAND_USAGE.md`  
  Field usage guide in Chinese, including startup order, calibration, thumb direction correction, and troubleshooting.

## Dependency Layout

The scripts expect these external SDK directories. Defaults match the current robot workstation:

```bash
export UNITREE_SIM_ROOT=/mnt/newssd/unitree_sim_isaaclab
export UNITREE_SDK2_ROOT=/mnt/newssd/unitree_sim_isaaclab/unitree_sdk2_python
export XR_TELEOP_ROOT=/mnt/newssd/unitree_sim_isaaclab/xr_teleoperate
export INSPIRE_HAND_SDK_ROOT=/mnt/newssd/unitree_sim_isaaclab/inspire_hand_ws/inspire_hand_sdk
export VISIONPRO_TELEOP_ROOT=/mnt/newssd/unitree_sim_isaaclab/inspire_hand_ws/VisionProTeleop
```

If your SDKs live elsewhere, copy `.env.example` to `.env` and update the paths.

Before running commands from a shell, load `.env` if you created one:

```bash
set -a
source .env
set +a
```

## Installation

Recommended Python environment:

```bash
conda create -n inspire_clean python=3.10 -y
conda activate inspire_clean
```

Install Pinocchio and core solver dependencies through conda-forge:

```bash
conda install -c conda-forge pinocchio casadi -y
```

Install Python requirements:

```bash
pip install -r requirements.txt
```

Install local SDK packages:

```bash
pip install -e "$VISIONPRO_TELEOP_ROOT"
pip install -e "$UNITREE_SDK2_ROOT"
pip install -e "$INSPIRE_HAND_SDK_ROOT"
pip install -r "$XR_TELEOP_ROOT/requirements.txt"
```

## Quick Start

Use two terminals.

Terminal 1, start the Inspire hand driver:

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/Headless_driver_r.py \
  --ip 192.168.123.211 \
  --lr l \
  --device-id 1
```

Terminal 2, tune only the hand:

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --print-debug
```

Control right arm and hand together:

```bash
/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --print-debug
```

## First-Time Hand Calibration

Run calibration once for your hand:

```bash
/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --calibrate-hand \
  --print-debug
```

The script samples two poses:

1. Fully open hand.
2. Closed grasp/fist with thumb opposition.

It saves:

```text
scripts/visionpro_right_hand_calibration.json
```

This file is ignored by git because it is machine/operator-specific.

## Thumb Direction Fix

If your thumb rotates inward while the robot thumb rotates outward, add:

```bash
--invert-thumb-rotation-command
```

Example:

```bash
/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --print-debug \
  --invert-thumb-rotation-command
```

## More Documentation

See [docs/VISIONPRO_G1_HAND_USAGE.md](docs/VISIONPRO_G1_HAND_USAGE.md) for the detailed Chinese usage guide.

## SONIC Whole-Body Sim (MuJoCo + Vision Pro)

Three-terminal sim2sim using [GR00T-WholeBodyControl](https://github.com/NVIDIA/GR00T-WholeBodyControl) GEAR-SONIC:

```bash
./scripts/setup_sonic_sim_venv.sh   # once
./run_sonic_sim_loop.sh             # Terminal 1 — MuJoCo (press 9 to drop elastic band)
./run_sonic_deploy.sh               # Terminal 2 — policy deploy
./run_sonic_avp_teleop.sh <VP_IP>   # Terminal 3 — AVP bridge
```

Operator flow: calibrate (`c`) → stand-hold (`]`) → teleop + head walk (`T`).  
See [command.txt](command.txt) for tuning flags (default `--loco-velocity-deadzone` is 0.07).
