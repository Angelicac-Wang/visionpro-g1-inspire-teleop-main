# Vision Pro → Unitree G1 Teleoperation

Apple Vision Pro teleoperation for **Unitree G1** + **Inspire hand**, using **SONIC** ([GR00T-WholeBodyControl](https://github.com/NVIDIA/GR00T-WholeBodyControl)).

Core logic lives in the Python package **`g1_teleop/`**.  
Shell entry points are the **`run_*.sh`** scripts in the repo root (same as `./bin/*.sh` — see [Which script to run?](#which-script-to-run) below).

---

## Install (once)

```bash
cp .env.example .env          # set GR00T_ROOT, SDK paths, SONIC_PYTHON
pip install -r requirements.txt
pip install -e .

# external SDKs (paths from .env)
pip install -e "$VISIONPRO_TELEOP_ROOT"
pip install -e "$UNITREE_SDK2_ROOT"
pip install -e "$INSPIRE_HAND_SDK_ROOT"

# MuJoCo sim only:
./tools/setup_sonic_sim_venv.sh
```

Vision Pro: open **Tracking Streamer → Start** on the same Wi‑Fi as the PC.

Operator keys (Terminal 3): **F** → **]** → **S** → **T** · **P** pause · **H** head zero · **o** stop  
Details: [docs/OPERATIONS.md](docs/OPERATIONS.md)

---

## Workflow A — MuJoCo sim (walk + head locomotion)

Same as before — three terminals:

```bash
# Terminal 1
./run_sonic_sim_loop.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop.sh <VISION_PRO_IP_OR_ROOM_CODE>
# example:
./run_sonic_avp_teleop.sh 192.168.2.14
```

MuJoCo tip: after **]**, press **9** in the sim window if the robot hangs in the air.  
MuJoCo FPV in the headset is **on by default** (sim camera → Vision Pro).

---

## Workflow B — MuJoCo sim pick-up (walk + hands + **simulated Inspire fingers**)

Table + cube scene; pinch in AVP to grasp in MuJoCo.

```bash
# Terminal 1
./run_sonic_sim_loop_pnp.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop_pick.sh <VISION_PRO_IP_OR_ROOM_CODE>
```

Same **F → ] → S → T** flow; fingers drive the MuJoCo Inspire hand (not physical hardware).

---

## Workflow C — Real robot (no MuJoCo, no sim FPV)

Two terminals only:

```bash
# Terminal 1 — on robot network (set SONIC_NET_IF in .env if needed)
./run_sonic_deploy.sh real

# Terminal 2
./run_sonic_avp_teleop.sh <VISION_PRO_IP> --no-mujoco-fpv
# example:
./run_sonic_avp_teleop.sh 192.168.2.14 --no-mujoco-fpv
```

`--no-mujoco-fpv` turns off sim camera streaming (there is no MuJoCo on real hardware).  
Physical Inspire hand on the robot uses a separate DDS driver — not covered in these three workflows.

---

## Which script to run?

| You used to run… | Same thing as… | What it does |
|------------------|----------------|--------------|
| `./run_sonic_sim_loop.sh` | `./bin/sonic-sim.sh` | MuJoCo sim (Terminal 1) |
| `./run_sonic_sim_loop_pnp.sh` | `./bin/sonic-sim-pick.sh` | MuJoCo pick-up scene |
| `./run_sonic_deploy.sh` | `./bin/sonic-deploy.sh` | SONIC policy (Terminal 2) |
| `./run_sonic_deploy.sh real` | `./bin/sonic-deploy.sh real` | Real robot deploy |
| `./run_sonic_avp_teleop.sh …` | `./bin/sonic-teleop.sh …` | AVP bridge (Terminal 3) |
| `./run_sonic_avp_teleop_pick.sh …` | `./bin/sonic-teleop-pick.sh …` | AVP bridge + sim fingers |

Root `run_*.sh` files are **one-line wrappers** that call `bin/`.  
**You can keep using `run_*.sh` exactly as in your old notes** — nothing changed in behavior.

Extra args (e.g. `--no-mujoco-fpv`, `--loco-max-speed 0.4`) go at the end of Terminal 3:

```bash
./run_sonic_avp_teleop.sh 192.168.2.14 --no-mujoco-fpv --print-debug
```

---

## Repo layout (for developers)

| Path | Purpose |
|------|---------|
| `g1_teleop/` | Library: transforms, hand map, calibration, head walk + IMU loop, SONIC bridge |
| `run_*.sh` | **Start here** (user-facing) |
| `bin/` | Implementation of `run_*.sh` |
| `scripts/` | Python entry (`g1_avp_sonic_teleop.py`, etc.) |
| `tools/` | `setup_sonic_sim_venv.sh`, scene generation |
| `docs/ARCHITECTURE.md` | Module design |
| `legacy/` | Old Isaac Sim stack — not used in the workflows above |

---

## Head locomotion + IMU

Head motion drives walk/turn; base IMU closes the yaw loop so direction is correct regardless of robot heading at calibration.  
Disable: `--no-loco-imu-correction` · Tune: `--loco-imu-yaw-gain`, `--loco-velocity-deadzone 0.07`

---

## More docs

- [docs/OPERATIONS.md](docs/OPERATIONS.md) — calibration steps, troubleshooting  
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — code structure  
- [docs/VISIONPRO_G1_HAND_USAGE.md](docs/VISIONPRO_G1_HAND_USAGE.md) — arm + physical Inspire hand only (separate from SONIC whole-body)

---

## External dependencies (not in this repo)

VisionProTeleop, `unitree_sdk2_python`, `inspire_hand_sdk`, GR00T-WholeBodyControl — configure paths in `.env`.
