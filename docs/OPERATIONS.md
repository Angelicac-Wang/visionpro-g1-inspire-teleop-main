# Operations guide

Copy-paste workflows for MuJoCo sim and real G1. All teleop sessions use the same calibration flow unless noted.

**Calibration flow (every session):** **F → ] → S → T**

| Key | Action |
|-----|--------|
| **F** | CALIB_FULL — forearms forward, hold ~2 s |
| **]** | ENGAGE policy (MuJoCo: press **9** in sim if robot floats) |
| **S** | CALIB_SYNC — match arms to robot, hold ~2 s |
| **T** | TELEOP — live control |
| **H** | Re-zero head facing / squat height |
| **P** | Pause teleop |
| **o** / **O** | Stop bridge / emergency stop deploy |

**Before you start**

```bash
cd /path/to/visionpro-g1-inspire-teleop-main
cp .env.example .env          # once; edit paths and NICs
pip install -e .                # once
```

Set in `.env` (see `.env.example`):

| Variable | When needed |
|----------|-------------|
| `SONIC_PYTHON` | Bridge Python with `pyzmq` |
| `GR00T_ROOT` | SONIC deploy + sim |
| `SONIC_NET_IF` | Real robot DDS NIC, e.g. `enp3s0` |
| `INSPIRE_HAND_LEFT_IP` | Real Inspire left (default `192.168.123.210`) |
| `INSPIRE_HAND_RIGHT_IP` | Real Inspire right (default `192.168.123.211`) |
| `INSPIRE_HAND_DDS_NETWORK` | Same NIC as robot/hands, e.g. `enp3s0` |

Deploy must publish `g1_debug` on port **5557** (sync + IMU locomotion).

`run_*.sh` and `bin/*.sh` are identical wrappers.

---

## Locomotion modes (pick one)

Both modes enable **head locomotion** by default (`--head-locomotion`). The only difference is whether keyboard walk keys are layered on top.

| Mode | Flag | How you walk |
|------|------|--------------|
| **Pure head** (default) | *(none — default)* | Head lean / turn only; keyboard walk keys ignored |
| **Hybrid** | `--hybrid-locomotion` | Head walk + keyboard overlay; keyboard wins while held |

**Hybrid rules**

- Click the **teleop terminal** so key presses reach the bridge.
- **Hold** a key to move; **release** to coast to a stop.
- **Fully stop** before reversing direction or spot-turning.
- **Space** stops immediately and **keeps current body facing**.

| Key (hybrid, hold) | Action |
|--------------------|--------|
| **W** | Walk forward (body facing direction) |
| **S** | Walk backward (moonwalk — facing stays forward) |
| **,** | Strafe left |
| **.** | Strafe right |
| **A** / **D** | Spot turn (only while fully stopped) |
| **space** / **r** | Stop — decelerate, keep facing |
| **H** | Re-zero head facing / squat height |

**Head-walk tips (both modes)**

- Press **H** before walking if facing feels wrong.
- At **F**, face the same direction you want the robot to face.
- When backing up, avoid turning your head to look behind — that drifts `facing`.

---

## A — MuJoCo sim (walk only)

### Terminal layout

| Terminal | Command |
|----------|---------|
| 1 — Sim | `./run_sonic_sim_loop.sh` |
| 2 — Deploy | `./run_sonic_deploy.sh` |
| 3 — Teleop | see **Pure head** or **Hybrid** below |

Replace `<VP_IP>` with Vision Pro IP or room code (e.g. `192.168.2.14`).

### A1 — Sim teleop (default: pure head + Inspire finger sim)

```bash
# Terminal 1
./run_sonic_sim_loop.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop.sh <VP_IP>
```

Built-in defaults (no extra flags needed):

- Pure head locomotion (`--no-hybrid-locomotion`)
- MuJoCo Inspire finger sim (`--enable-inspire-hand-sim`)
- Left arm: `--left-hand-delta-remap identity`, `--left-wrist-orientation-mode calibrated`, `--left-wrist-axis-remap avp-palm-left`, `--left-wrist-rot-sign-y 1.0`

### A2 — Hybrid head + keyboard (sim)

```bash
# Terminal 1
./run_sonic_sim_loop.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop.sh <VP_IP> --hybrid-locomotion
```

Focus Terminal 3 before using **W/A/S/D**.

---

## B — MuJoCo pick-up (walk + sim Inspire fingers)

Same locomotion choice as A; pick script adds `--enable-inspire-hand-sim` automatically. Pinch in AVP to close fingers in sim.

### B1 — Pure head (sim pick-up)

```bash
# Terminal 1
./run_sonic_sim_loop_pnp.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop_pick.sh <VP_IP> --no-hybrid-locomotion
```

### B2 — Hybrid (sim pick-up, default)

```bash
# Terminal 1
./run_sonic_sim_loop_pnp.sh

# Terminal 2
./run_sonic_deploy.sh

# Terminal 3
./run_sonic_avp_teleop_pick.sh <VP_IP>
```

---

## C — Real robot (full stack)

Real robot needs **no MuJoCo FPV**, robot network on `SONIC_NET_IF`, and (for fingers) Inspire hand drivers on a separate terminal.

### C0 — Pre-flight checks

**Before `./run_sonic_deploy.sh real`**, put the G1 in low-level-ready state:

1. **Hoist** the robot (feet off ground initially).
2. Power on → wait for **zero-torque mode** (joints free when pushed by hand).
3. On the **Unitree remote**: press **L2 + R2** (repeat if needed) → **debug mode** (yellow LED, joints in damping).
4. PC Ethernet to robot on `192.168.123.x` via your NIC (e.g. `enp3s0`).

If deploy prints **`Failed to switch to Release Mode`** in a loop, the built-in sport/motion service is still running. Re-do step 3, or **reboot the robot** and enter debug mode again before restarting deploy.

```bash
# Load env
source .env

# Robot NIC (adjust name)
ip -4 addr show enp3s0

# Inspire hands on 192.168.123.x — both must reply before starting drivers
ping -c 2 192.168.123.210    # left
ping -c 2 192.168.123.211    # right
```

Green LED on Inspire = power only, not network. Each hand needs its own Ethernet to `192.168.123.x`.

If ping fails, fix cabling/IP before `./run_both_hand_driver.sh`.

### C1 — Arms + walk only (no physical hands)

```bash
# Terminal 1 — deploy on robot network
./run_sonic_deploy.sh real

# Terminal 2 — teleop (default: pure head)
./run_sonic_avp_teleop.sh <VP_IP> --no-mujoco-fpv

# Terminal 2 — teleop (hybrid keyboard overlay)
./run_sonic_avp_teleop.sh <VP_IP> --no-mujoco-fpv --hybrid-locomotion
```

Use **one** Terminal 2 command, not both.

### C2 — Full teleop (walk + real Inspire hands)

Four terminals:

```bash
# Terminal 1 — SONIC policy on real G1
./run_sonic_deploy.sh real

# Terminal 2 — Inspire Modbus → DDS (both hands)
./run_both_hand_driver.sh --dds-network enp3s0

# Terminal 3 — AVP bridge, hybrid keyboard walk (recommended on real robot)
./run_sonic_avp_teleop.sh <VP_IP> \
  --no-mujoco-fpv \
  --hybrid-locomotion \
  --enable-inspire-hand-dds \
  --hand-dds-sides both \
  --hand-dds-network enp3s0

# Terminal 3 — pure head only (no keyboard walk)
./run_sonic_avp_teleop.sh <VP_IP> \
  --no-mujoco-fpv \
  --enable-inspire-hand-dds \
  --hand-dds-sides both \
  --hand-dds-network enp3s0
```

Use **one** Terminal 3 command. `--hand-dds-network` should match `INSPIRE_HAND_DDS_NETWORK` in `.env`.

**Single hand only**

```bash
./run_both_hand_driver.sh --sides l --dds-network enp3s0
# teleop: --hand-dds-sides l
```

**Custom hand IPs**

```bash
./run_both_hand_driver.sh \
  --left-ip 192.168.123.210 \
  --right-ip 192.168.123.211 \
  --dds-network enp3s0
```

Driver Hz logs mean Modbus is up; they do **not** prove DDS commands from teleop are arriving.

---

## Optional flags (append to Terminal 3)

Add flags **after** `<VP_IP>` on the same line. Examples below use hybrid sim; same flags work on real robot after `--no-mujoco-fpv`.

### Locomotion tuning

```bash
# Softer facing / less IMU fighting (if walk direction drifts)
--loco-facing-smooth 0.12 \
--loco-yaw-deadzone 0.20

# Disable IMU yaw closed-loop (debug / ablation)
--no-loco-imu-correction

# Speed and smoothing
--loco-max-speed 0.50 \
--loco-smooth 0.18 \
--loco-velocity-deadzone 0.06

# Hybrid keyboard walk speed only
--keyboard-loco-speed 0.42
```

### Arm tracking loss (default ON)

When a wrist leaves the AVP field of view, the bridge **holds the last valid arm target** instead of snapping back to the L init pose. Tracking recovery is smoothed by the existing arm ramp (`--vr-ramp-max-speed`, `--arm-max-angular-speed`).

```bash
# Default — no flag needed
--arm-tracking-hold

# Legacy snap-to-init when tracking is lost
--no-arm-tracking-hold
```

Debug: with `--print-debug`, look for `arm_hold= {'left_hold': True, ...}`.

### Left arm hunching (tracking active)

Left wrist **orientation** can pull the whole upper body when using `calibrated` mode. If the torso hunches while the left hand moves, try decoupling wrist rotation from the policy:

```bash
--left-wrist-orientation-mode neutral
```

Re-enable calibrated palm control once reach mapping is tuned:

```bash
--left-wrist-orientation-mode calibrated \
--left-wrist-axis-remap avp-palm-left \
--left-wrist-rot-sign-y 1.0
```

Track **right hand only** while debugging left mapping:

```bash
--active-hands right
```

### Squat / head height

Enabled by default in `g1_avp_sonic_teleop.py`. To disable:

```bash
--no-head-height-squat
```

### Debug logging

```bash
--print-debug    # already added by run_sonic_avp_teleop.sh; safe to repeat
```

### Left wrist (if left palm / reach needs tuning)

Defaults are set in code; override only when needed:

```bash
--left-wrist-orientation-mode calibrated \
--left-wrist-axis-remap avp-palm-left \
--left-wrist-rot-sign-y 1.0
```

### Eval / corridor (optional)

See [EXPERIMENT_TASK_A.md](EXPERIMENT_TASK_A.md).

Full flag list:

```bash
python scripts/g1_avp_sonic_teleop.py --help
```

---

## Example: one-liner reference

**Sim hybrid (keyboard overlay)**

```bash
./run_sonic_sim_loop.sh          # T1
./run_sonic_deploy.sh            # T2
./run_sonic_avp_teleop.sh 192.168.2.14 --hybrid-locomotion   # T3
```

**Sim default (pure head — same as your tuned preset)**

```bash
./run_sonic_avp_teleop.sh 192.168.2.14
```

**Real robot + hands + hybrid**

```bash
./run_sonic_deploy.sh real                                              # T1
./run_both_hand_driver.sh --dds-network enp3s0                          # T2
./run_sonic_avp_teleop.sh 192.168.2.14 --no-mujoco-fpv \
  --enable-inspire-hand-dds --hand-dds-sides both --hand-dds-network enp3s0   # T3
```

**Real robot facing drift workaround**

```bash
./run_sonic_avp_teleop.sh <VP_IP> --no-mujoco-fpv \
  --loco-facing-smooth 0.12 --loco-yaw-deadzone 0.20
```

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| CALIB_SYNC fails | Deploy running? `g1_debug` on `:5557`? |
| Walk direction wrong / facing drifts | Press **H**; recalibrate **F** with head forward; try `--loco-facing-smooth 0.12` |
| Back then forward goes sideways | Don't turn head while backing up; press **H** before next walk |
| Keyboard does nothing | Focus teleop terminal; finish **T** first; hybrid must be on (no `--no-hybrid-locomotion`) |
| **S** triggers sync during teleop | Update bridge — teleop **S** is backward walk only in hybrid mode |
| Robot arms snap down before **]** | Start teleop **before** **]** so init pose streams |
| Hand lost → body snaps / hunches | Default `--arm-tracking-hold` holds last pose; use `--no-hybrid-locomotion` for stable walk |
| Left arm pulls torso / hunch | Try `--left-wrist-orientation-mode neutral` or `--active-hands right` while tuning |
| Driver runs but fingers don't move | Teleop needs `--enable-inspire-hand-dds --hand-dds-sides both --hand-dds-network enp3s0` |
| Import / ZMQ errors | `pip install -e .`; set `SONIC_PYTHON` in `.env` |
| Deploy can't find robot | Set `SONIC_NET_IF=enp3s0` in `.env` |

Legacy Isaac Sim notes: [legacy/command.txt](../legacy/command.txt) (outdated — use workflows A/B/C above).
