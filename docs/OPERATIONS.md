# Operations guide

Three supported test workflows. All use the same **F → ] → S → T** calibration on Terminal 3 unless noted.

---

## A — MuJoCo sim (walk)

```bash
./run_sonic_sim_loop.sh              # Terminal 1
./run_sonic_deploy.sh                # Terminal 2
./run_sonic_avp_teleop.sh <VP_IP>   # Terminal 3
```

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

## Common flags (append to Terminal 3 command)

```bash
--no-mujoco-fpv              # real robot (required on hardware)
--loco-velocity-deadzone 0.07
--loco-max-speed 0.45
--no-loco-imu-correction
```

Full list: `python scripts/g1_avp_sonic_teleop.py --help`

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
| Robot arms snap down before **]** | Start Terminal 3 before **]** (streams init pose) |
| Import errors | `pip install -e .` and `source .env` |

Legacy Isaac Sim notes: [legacy/command.txt](../legacy/command.txt) (outdated paths — use workflows A/B/C above).
