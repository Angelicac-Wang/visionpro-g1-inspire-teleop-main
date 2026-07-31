# Architecture

Module layout for contributors. For operator steps see [OPERATIONS.md](OPERATIONS.md).

## Data flow

```text
Vision Pro  →  AVP streamer  →  g1_teleop bridge  →  ZMQ planner (:5556)
                                                      ↘  DDS Inspire hand (optional)
SONIC deploy  →  ZMQ feedback (:5557, g1_debug)  →  calibration + IMU yaw loop
MuJoCo sim    →  camera ZMQ (:5555)  →  Vision Pro FPV (optional)
```

## Repository map

| Path | Purpose |
|------|---------|
| `g1_teleop/` | Importable library — all core logic |
| `bin/` | Supported bash launchers |
| `scripts/` | Python CLI entry points (called by `bin/`) |
| `tools/` | One-off setup / scene / diagnostic scripts |
| `legacy/` | Unmaintained Isaac Sim stack |

## Python package

| Module | Role |
|--------|------|
| `g1_teleop/transforms/frames.py` | AVP / OpenXR → robot Z-up frame |
| `g1_teleop/hand/mapping.py` | Finger skeleton → Inspire 6-DOF commands |
| `g1_teleop/calibration/session.py` | F/]/S/T calibration, ZMQ feedback, optional FK |
| `g1_teleop/locomotion/head.py` | Head walk / facing / squat; IMU closed loop |
| `g1_teleop/bridge/` | AVP → SONIC runtime (split submodules) |
| `g1_teleop/sim/mujoco_fpv.py` | MuJoCo head camera → Vision Pro WebRTC |

### Bridge submodules

| File | Responsibility |
|------|----------------|
| `bridge/runtime.py` | Main loop |
| `bridge/cli.py` | Argument parser |
| `bridge/zmq_pub.py` | SONIC packed ZMQ publisher |
| `bridge/vr_targets.py` | AVP poses → `vr_position` / `vr_orientation` |
| `bridge/locomotion_io.py` | Base IMU from deploy feedback |
| `bridge/smoothing.py` | Arm pose smoothing |
| `bridge/rotation_utils.py` | Wrist calibration math |

## Entry points

| Script | Called by |
|--------|-----------|
| `scripts/g1_avp_sonic_teleop.py` | `bin/sonic-teleop.sh` |
| `scripts/g1_avp_sonic_teleop_pick.py` | `bin/sonic-teleop-pick.sh` |
| `scripts/avp_to_sonic_zmq.py` | Bridge with raw defaults |
| `scripts/visionpro_g1_right_arm_hand.py` | `bin/hand-teleop.sh` |
| `scripts/Headless_driver_r.py` | `bin/hand-driver.sh` |

## IMU closed-loop locomotion

At calibration, the bridge stores `robot_base_yaw_at_calib` from deploy `base_quat`. Each frame, `apply_imu_yaw_closed_loop()` corrects commanded facing so travel direction is consistent regardless of initial pelvis heading.

Flags: `--loco-imu-yaw-gain`, `--loco-imu-yaw-deadzone`, `--no-loco-imu-correction`.

## External dependencies

Configure in `.env` — not vendored:

- VisionProTeleop / `avp_stream`
- `unitree_sdk2_python`
- `inspire_hand_sdk`
- GR00T-WholeBodyControl (SONIC deploy + sim)

```bash
pip install -e .
```
