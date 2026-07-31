# Architecture

This document describes how the repository is organized for open-source contributors.

## Overview

The project bridges **Apple Vision Pro (AVP)** tracking to a **Unitree G1** robot running **SONIC** (GR00T whole-body control). Data flows:

```text
Vision Pro  →  AVP streamer  →  g1_teleop bridge  →  ZMQ (planner + hands)
                                                      ↘  DDS (physical Inspire hand, optional)
SONIC deploy  →  ZMQ feedback (g1_debug)  →  calibration + IMU closed-loop locomotion
MuJoCo sim    →  camera ZMQ  →  Vision Pro FPV (optional)
```

Operator workflow (staged calibration):

```text
F  CALIB_FULL   — forearms-forward pose + walk/squat zero
]  ENGAGE       — start balance policy, stand hold
S  CALIB_SYNC   — align wrists to current robot pose (FK / feedback)
T  TELEOP       — hands + head walk + squat
H  HEAD zero    — re-sync facing / height only
P  PAUSE        — freeze mapping
```

## Python package: `g1_teleop/`

| Module | Role |
|--------|------|
| `g1_teleop/transforms/frames.py` | AVP / OpenXR → robot Z-up frame, yaw helpers |
| `g1_teleop/hand/mapping.py` | AVP finger skeleton → Inspire 6-DOF hand commands |
| `g1_teleop/calibration/session.py` | Staged F/]/S/T calibration, ZMQ feedback client, FK robot reference |
| `g1_teleop/locomotion/head.py` | Head-driven walk / facing / squat; **IMU yaw closed loop** |
| `g1_teleop/bridge/` | AVP → SONIC ZMQ bridge (split into focused files) |
| `g1_teleop/sim/mujoco_fpv.py` | MuJoCo head camera → Vision Pro WebRTC |

### Bridge submodules

| File | Responsibility |
|------|----------------|
| `bridge/runtime.py` | Main control loop, keyboard handling, publish loop |
| `bridge/cli.py` | All CLI flags |
| `bridge/zmq_pub.py` | SONIC packed ZMQ publisher |
| `bridge/dds_hand.py` | Physical Inspire hand DDS publisher |
| `bridge/vr_targets.py` | Map AVP poses → `vr_position` / `vr_orientation` |
| `bridge/smoothing.py` | Arm position / orientation smoothing |
| `bridge/rotation_utils.py` | Wrist calibration rotation math |
| `bridge/locomotion_io.py` | Read base IMU from deploy feedback for closed-loop facing |
| `bridge/keyboard.py` | Terminal keyboard (WASD fallback walk) |
| `bridge/constants.py` | Fixed coordinate transforms |

## Entry scripts (`scripts/`)

Shell launchers (`run_sonic_avp_teleop.sh`, etc.) call thin wrappers in `scripts/` for backward compatibility. Implementation lives under `g1_teleop/`.

| Script | Purpose |
|--------|---------|
| `scripts/avp_to_sonic_zmq.py` | SONIC ZMQ bridge entry |
| `scripts/g1_avp_sonic_teleop.py` | Opinionated defaults for real / sim teleop |
| `scripts/visionpro_g1_right_arm_hand.py` | Right arm + Inspire hand only (no SONIC) |
| `scripts/Headless_driver_r.py` | Inspire hand Modbus/DDS driver |

Shims (`scripts/g1_*.py`, `scripts/avp_inspire_hand_mapping.py`) re-export the package so older imports keep working.

## Head locomotion + IMU closed loop

`g1_teleop/locomotion/head.py` converts head motion into SONIC planner commands:

- **movement** — travel direction (can include backward motion)
- **facing** — body heading (separate from movement; avoids unwanted turn-around)

At calibration (`F` / `T` / `H`), the bridge records `robot_base_yaw_at_calib` from deploy feedback (`base_quat` in `g1_debug`). Each frame, `apply_imu_yaw_closed_loop()` compares commanded facing to measured pelvis yaw and applies a bounded correction so the robot reaches the intended direction **regardless of its initial body heading**.

CLI tuning: `--loco-imu-yaw-gain`, `--loco-imu-yaw-deadzone`, `--no-loco-imu-correction`.

## External dependencies (not vendored)

Set paths in `.env` (see `.env.example`):

- `VisionProTeleop` / `avp_stream`
- `unitree_sdk2_python`
- `inspire_hand_sdk`
- `GR00T-WholeBodyControl` (optional FK for CALIB_SYNC)

## Install for development

```bash
pip install -e .
# or export PYTHONPATH=$REPO_ROOT when running scripts directly
```

## Assets

- `assets/mujoco/` — MuJoCo scenes for sim2sim
- `recordings/` — optional debug traces (not required to run)
