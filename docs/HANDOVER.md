# Vision Pro–G1 Teleoperation: Project Handover

This document explains the current system, its dependencies, design, known limitations, and the files a new maintainer should understand.

For copy-paste launch commands, calibration keys, real-robot preflight, and troubleshooting, use [`OPERATIONS.md`](OPERATIONS.md). That file is the canonical day-to-day runbook. Older files under `legacy/` are reference material and should not be used as the current operating procedure.

## 1. Project scope and current status

The project uses Apple Vision Pro to teleoperate a Unitree G1 humanoid through NVIDIA's SONIC whole-body policy. The current stack supports:

- MuJoCo whole-body teleoperation;
- head-driven forward, backward, lateral, and facing control;
- head-height-based squat/kneel control;
- Vision Pro wrist targets for both robot arms;
- simulated and physical Inspire Hand control;
- staged calibration and mid-session head re-zeroing;
- arm smoothing, rate limits, and tracking-loss hold;
- optional keyboard-assisted locomotion for constrained lab spaces.

Basic whole-body movement, bag pick-and-place, and light assisted two-hand manipulation have been demonstrated informally. The system is not yet suitable for heavy bimanual loads or long unattended operation.

## 2. System architecture

```text
Apple Vision Pro
  └─ head, wrist, and finger tracking
       ↓
g1_teleop bridge
  ├─ head pose → movement, facing, speed, pelvis height
  ├─ wrist poses → left/right arm targets
  ├─ finger tracking → Inspire Hand commands
  └─ SONIC planner + VR targets → ZMQ port 5556
       ↓
SONIC deploy
  ├─ whole-body policy
  ├─ locomotion, arm coordination, and balance
  └─ robot feedback → ZMQ port 5557
       ↓
MuJoCo simulation or physical Unitree G1
```

For physical Inspire Hands:

```text
Vision Pro fingers
  → bridge DDS command
  → left/right hand-driver process
  → Modbus TCP
  → physical Inspire Hands
```

Local ports used by the current stack:

- `5556`: bridge to SONIC planner and VR commands;
- `5557`: SONIC feedback to the bridge for calibration and IMU correction;
- `5555`: MuJoCo camera frames to the optional FPV bridge;
- `9999`: optional WebRTC video output toward Vision Pro.

## 3. Head-driven locomotion

Head-driven locomotion is the main functional extension developed during this internship. Vision Pro supplies a six-degree-of-freedom head pose. At calibration, the bridge records a neutral head position and orientation, then converts later motion as follows:

- horizontal head motion becomes walking direction and speed;
- head orientation becomes a separate body-facing direction;
- vertical head motion becomes pelvis-height control.

Separating movement from facing allows the robot to walk backward while continuing to face forward. Before commands reach SONIC, the bridge applies frame conversion, dead zones, smoothing, speed limits, and optional base-IMU heading correction.

The bridge sends high-level intent to SONIC. SONIC generates coordinated whole-body motion and balance; the bridge does not directly command individual leg joints.

## 4. Platform and dependencies

The runtime is intended for the Linux lab workstation and assumes:

- Python 3.10 or later;
- an NVIDIA GPU and TensorRT;
- `GR00T-WholeBodyControl`;
- `VisionProTeleop`;
- `unitree_sdk2_python`;
- `inspire_hand_sdk`;
- Apple Vision Pro and its Tracking Streamer app;
- the Unitree G1 and Inspire Hands on the `192.168.123.x` network.

Machine-specific paths belong in `.env`, which is created from `.env.example`.

Important variables:

```bash
GR00T_ROOT=/absolute/path/to/GR00T-WholeBodyControl
VISIONPRO_TELEOP_ROOT=/absolute/path/to/VisionProTeleop
UNITREE_SDK2_ROOT=/absolute/path/to/unitree_sdk2_python
INSPIRE_HAND_SDK_ROOT=/absolute/path/to/inspire_hand_sdk
SONIC_PYTHON=/absolute/path/to/python
TensorRT_ROOT=/absolute/path/to/TensorRT

# Physical robot and hands
SONIC_NET_IF=enp3s0
INSPIRE_HAND_LEFT_IP=192.168.123.210
INSPIRE_HAND_RIGHT_IP=192.168.123.211
INSPIRE_HAND_DDS_NETWORK=enp3s0
```

`SONIC_PYTHON` must point to an environment that can import `pyzmq`.

### One-time installation

```bash
cp .env.example .env
# Edit .env before continuing.

set -a
source .env
set +a

"$SONIC_PYTHON" -m pip install -r requirements.txt
"$SONIC_PYTHON" -m pip install -e .
"$SONIC_PYTHON" -m pip install -e "$VISIONPRO_TELEOP_ROOT"
"$SONIC_PYTHON" -m pip install -e "$UNITREE_SDK2_ROOT"
"$SONIC_PYTHON" -m pip install -e "$INSPIRE_HAND_SDK_ROOT"

./tools/setup_sonic_sim_venv.sh
```

The default simulation environment is:

```text
~/.config/visionpro-g1-inspire-teleop/venv_sonic_sim
```

The SONIC launcher expects:

```text
$GR00T_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref
```

If this executable is absent, `./run_sonic_deploy.sh` prints the build command. TensorRT engines may need to be regenerated for a different GPU.

## 5. Runtime behavior and defaults

The root `run_*.sh` files are the supported user entry points. They call scripts under `bin/`, which load `.env` and prepare the required Python paths.

The standard teleoperation wrapper currently enables:

- head locomotion;
- head-height squat control;
- staged calibration;
- arm tracking-loss hold;
- IMU yaw correction;
- simulated Inspire fingers in MuJoCo;
- debug printing;
- the tuned left-arm preset.

Keyboard hybrid locomotion is **off** unless `--hybrid-locomotion` is passed.

The tuned left-arm defaults are:

```text
left-hand-delta-remap: identity
left-wrist-orientation-mode: calibrated
left-wrist-axis-remap: avp-palm-left
left-wrist-rot-sign-y: 1.0
```

Calibration uses the current `F → ] → S → T` sequence. Do not mix this with older notes that use `c → ] → T`.

## 6. Code map

Main runtime:

- `g1_teleop/bridge/runtime.py` — central Vision Pro-to-SONIC loop;
- `g1_teleop/bridge/cli.py` — bridge arguments and defaults;
- `g1_teleop/bridge/zmq_pub.py` — SONIC ZMQ command publishing;
- `g1_teleop/bridge/vr_targets.py` — Vision Pro poses to arm targets;
- `g1_teleop/bridge/keyboard.py` — terminal controls;
- `g1_teleop/bridge/locomotion_io.py` — base IMU and SONIC feedback;
- `g1_teleop/bridge/smoothing.py` — arm pose smoothing;
- `g1_teleop/bridge/arm_tracking_hold.py` — lost-wrist hold behavior;
- `g1_teleop/bridge/dds_hand.py` — physical hand DDS publishing.

Locomotion and calibration:

- `g1_teleop/locomotion/head.py` — head movement, facing, squat, and IMU correction;
- `g1_teleop/locomotion/hybrid.py` — keyboard/head command merging;
- `g1_teleop/calibration/session.py` — staged calibration state and feedback.

Hands and simulation:

- `g1_teleop/hand/mapping.py` — finger tracking to Inspire Hand mapping;
- `g1_teleop/sim/mujoco_fpv.py` — experimental MuJoCo first-person video;
- `tools/sonic_run_sim_loop.py` — MuJoCo simulation entry;
- `tools/generate_pnp_scene.py` — pick-and-place scene generation.

Entry points:

- `scripts/g1_avp_sonic_teleop.py` — standard tuned teleoperation wrapper;
- `scripts/g1_avp_sonic_teleop_pick.py` — pick-and-place wrapper;
- `scripts/avp_to_sonic_zmq.py` — low-level bridge entry;
- `scripts/Headless_driver_r.py` — Inspire Hand Modbus/DDS driver;
- `bin/` — launcher implementations;
- `run_*.sh` — stable user-facing launcher names.

See `docs/ARCHITECTURE.md` for the package-level dependency structure.

## 7. Generated and machine-local files

The following are machine-specific or generated and may be gitignored:

- `.env`;
- `scripts/visionpro_left_hand_calibration.json`;
- `scripts/visionpro_right_hand_calibration.json`;
- `assets/mujoco/g1_runtime/*.xml`;
- `~/.config/visionpro-g1-inspire-teleop/venv_sonic_sim`.

Back up useful calibration JSON files before moving to a different workstation. Do not commit machine-specific absolute paths or credentials.

## 8. Known limitations

### Egocentric video

The MuJoCo camera, ZMQ forwarding, and WebRTC bridge exist, but the robot's first-person view is not reliably visible inside Vision Pro. Treat this path as experimental. Daily operation should use `--no-mujoco-fpv` when video is unnecessary.

### Left arm

The current left-specific preset supports basic tasks but remains less consistent than the right arm. Some poses can cause reduced reach or torso hunching. Test mapping changes in MuJoCo before hardware.

Useful diagnostic overrides:

```text
--active-hands right
--left-wrist-orientation-mode neutral
```

### Bimanual manipulation

Light assisted two-hand manipulation is possible. Heavy two-hand carrying remains unstable and should not be attempted without further controller work and supervised testing.

### Long-duration hardware operation

Shoulder and pelvis motors may overheat during extended sessions. Use short trials, monitor robot status, and allow cooldown periods.

### Tracking loss

Vision Pro can temporarily lose a wrist. Hold-last-valid-pose behavior is enabled by default, but prolonged or incorrect tracking still requires pausing or stopping the session.

## 9. Development and verification

The current unit tests cover core mapping and safety behavior:

- `tests/test_official_hand_delta.py`;
- `tests/test_wrist_rotation_signs.py`;
- `tests/test_arm_tracking_hold.py`.

These tests are unrelated to any unfinished experiment and should be retained.

Run:

```bash
pytest
```

Recommended verification order:

1. unit tests;
2. standard MuJoCo workflow;
3. MuJoCo pick-and-place;
4. hoisted real robot at reduced speed;
5. physical Inspire Hands;
6. unhoisted locomotion only after supervised safety review.

When a runtime default changes, update the wrapper, `OPERATIONS.md`, and this file together.

## 10. Handover checklist

- [ ] Configure `.env` on the target Linux workstation.
- [ ] Confirm the SONIC deploy binary and TensorRT engines work on its GPU.
- [ ] Confirm `"$SONIC_PYTHON" -c "import zmq"` succeeds.
- [ ] Connect Vision Pro Tracking Streamer.
- [ ] Complete the MuJoCo workflow through `F → ] → S → T`.
- [ ] Verify head walking, facing, arm motion, and simulated fingers.
- [ ] Back up hand-calibration JSON files.
- [ ] Record the correct robot and hand network interface.
- [ ] Verify emergency-stop behavior with the real robot hoisted.
- [ ] Transfer the demo videos and internship final report.

