# Operations guide

Three supported test workflows. All use the same **F → ] → S → T** calibration on Terminal 3 unless noted.

Terminal 3 must have **keyboard focus** for hybrid locomotion (click the bridge window before pressing keys).

---

## A — MuJoCo sim (walk)

```bash
./run_sonic_sim_loop.sh              # Terminal 1
./run_sonic_deploy.sh                # Terminal 2
./run_sonic_avp_teleop.sh <VP_IP>   # Terminal 3
```

Default: **head locomotion + hybrid keyboard overlay** (see below).

---

## B — MuJoCo pick-up (walk + sim Inspire fingers)

```bash
./run_sonic_sim_loop_pnp.sh              # Terminal 1
./run_sonic_deploy.sh                    # Terminal 2
./run_sonic_avp_teleop_pick.sh <VP_IP>   # Terminal 3
```

Pinch fingers in AVP to close the gripper in simulation.

---

## C — Real robot

```bash
./run_sonic_deploy.sh real                         # Terminal 1
./run_sonic_avp_teleop.sh <VP_IP> --no-mujoco-fpv  # Terminal 2
```

Use `--no-mujoco-fpv` on hardware (no sim camera).  
Set `SONIC_NET_IF=enp3s0` (or your NIC) in `.env` if deploy cannot find the robot network.

---

## Calibration keys (Terminal 3)

| Key | Action |
|-----|--------|
| **F** | CALIB_FULL — forearms-forward, hold ~2 s |
| **]** | ENGAGE policy (MuJoCo: press **9** if robot floats) |
| **S** | CALIB_SYNC — match arms to robot, hold ~2 s |
| **T** | TELEOP |
| **H** | Re-zero head facing / squat height |
| **P** | Pause |
| **o** / **O** | Stop bridge / emergency stop deploy |

Terminal 2 must publish `g1_debug` on port **5557** for sync + IMU locomotion.

---

## Hybrid keyboard locomotion (Terminal 3)

Enabled by default when using `./run_sonic_avp_teleop.sh` (`--head-locomotion` + `--hybrid-locomotion`).

**Hold** a key to move; **release** to coast to a stop. You must **fully stop** before changing direction or turning. **Space** stops immediately and **keeps the current body facing** (does not snap back to the calibration heading).

Click the **Terminal 3** window so key presses reach the bridge.

| Key | Action (hold) |
|-----|----------------|
| **W** | Walk forward (body facing direction) |
| **S** | Walk backward (moonwalk — body facing stays forward) |
| **,** | Strafe left |
| **.** | Strafe right |
| **A** / **D** | Spot turn (only while fully stopped) |
| **space** / **r** | Stop — decelerate, keep facing |
| **H** | Re-zero head facing / squat height (head loco) |

While a keyboard move is active, keyboard commands **override** head walk speed. After you release keys and the robot stops, head locomotion resumes.

### Pure head control (no keyboard overlay)

Disable hybrid and use head lean / turn only:

```bash
./run_sonic_avp_teleop.sh <VP_IP> --no-hybrid-locomotion
```

Same calibration flow (**F → ] → S → T**). Locomotion comes from head motion only; keyboard walk keys are ignored.

Optional tuning (append to Terminal 3):

```bash
--no-loco-imu-correction     # open-loop yaw (ablation)
--loco-max-speed 0.50
--loco-smooth 0.18
--loco-facing-smooth 0.28
```

### Hybrid-only tuning

```bash
--keyboard-loco-speed 0.42   # keyboard walk speed (m/s)
--no-hybrid-locomotion       # pure head control
```

Full list: `python scripts/g1_avp_sonic_teleop.py --help`

---

## Common flags (append to Terminal 3 command)

```bash
--no-mujoco-fpv              # real robot (required on hardware)
--no-hybrid-locomotion       # pure head locomotion (no keyboard overlay)
--keyboard-loco-speed 0.42   # hybrid keyboard walk speed
--loco-velocity-deadzone 0.06
--loco-max-speed 0.50
--no-loco-imu-correction
```

Full list: `python scripts/g1_avp_sonic_teleop.py --help`

---

## Task A eval (optional)

Corridor scene + eval logging: [EXPERIMENT_TASK_A.md](EXPERIMENT_TASK_A.md)

---

## `run_*.sh` vs `bin/*.sh`

Identical. Example:

```text
./run_sonic_avp_teleop.sh 192.168.2.14
  →  ./bin/sonic-teleop.sh 192.168.2.14
```

Keep using `run_*.sh` if that is what you already have in your notes.

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| CALIB_SYNC fails | Deploy running? `g1_debug` on :5557? |
| Wrong walk direction | Press **H**; check IMU feedback |
| Keyboard does nothing | Focus Terminal 3; complete **T** (teleop) first |
| **S** triggers sync during teleop | Update bridge — teleop **S** is backward only |
| Robot arms snap down before **]** | Start Terminal 3 before **]** (streams init pose) |
| Import errors | `pip install -e .` and `source .env` |

Legacy Isaac Sim notes: [legacy/command.txt](../legacy/command.txt) (outdated paths — use workflows A/B/C above).
