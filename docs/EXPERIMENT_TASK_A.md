# REMS Task A — Atmospheric Route Examination

MuJoCo corridor (**3.0 m × 2.2 m**) with **5 waypoints**. Measures route completion with **IMU closed-loop locomotion** (default ON) vs ablation (`--no-loco-imu-correction`).

## Terminals

```bash
# T1 — corridor scene
./run_sonic_sim_loop_corridor.sh

# T2 — deploy (must publish g1_debug :5557)
./run_sonic_deploy.sh

# T3 — teleop + eval log
./run_sonic_avp_teleop.sh <VP_IP> \
  --eval-log runs/task_a_imu_on_trial1.csv

# IMU ablation (same trial, new CSV)
./run_sonic_avp_teleop.sh <VP_IP> \
  --no-loco-imu-correction \
  --eval-log runs/task_a_imu_off_trial1.csv
```

Operator flow: **F → ] → S → T** (see [OPERATIONS.md](OPERATIONS.md)).

## Hybrid locomotion (small lab)

Default **ON** with head locomotion:

- **Head** — lean / turn to walk (primary REMS mode)
- **Hold W / S** — forward / backward (body faces forward on S; moonwalk back)
- **Hold , / .** — strafe left / right (body facing unchanged)
- **Hold A / D** or **j / l** — spot turn in place while fully stopped
- **space / r** — stop now; **keeps current body facing** (no snap to head zero)
- Must fully stop before changing direction or turning

Disable keyboard overlay: `--no-hybrid-locomotion`

## Waypoints

| Key (during teleop) | WP | Position (x, y) m | Label |
|---------------------|----|-------------------|-------|
| 4 | 1 | (0.40, 0.00) | entry |
| 5 | 2 | (0.90, 0.45) | left_branch |
| 6 | 3 | (1.50, 0.00) | mid_corridor |
| 7 | 4 | (2.10, -0.45) | right_branch |
| 8 | 5 | (2.80, 0.00) | far_end |

Press the key when the robot reaches each orange marker in sim.

## Protocol (one trial)

1. Calibrate (**F / ] / S / T**); confirm `Loco IMU zero` prints at **T**.
2. Start eval log (via `--eval-log`).
3. Walk the route using **head locomotion**; use **WASD** only if you need fine position in a tight lab.
4. Mark **WP1–WP5** with keys **4–8** at each waypoint.
5. Stop with **o** when WP5 is marked.

Optional: repeat with `--no-loco-imu-correction` for IMU ablation.

## Metrics report

```bash
python scripts/eval_task_a_report.py runs/task_a_imu_on_trial1.csv --imu-condition imu_on
python scripts/eval_task_a_report.py runs/task_a_imu_off_trial1.csv --imu-condition imu_off \
  --json-out runs/task_a_imu_off_trial1.json
```

Report includes:

- Trial duration
- `|yaw_err|` mean / max (from IMU closed loop debug)
- Waypoint reach times (from keys 4–8)

## Optional standalone IMU log (Terminal 4)

```bash
python scripts/log_rems_imu.py --output runs/task_a_imu_raw.csv
```

## Paper alignment

Task A supports **atmospheric route examination** along a confined corridor. Compare **imu_on** vs **imu_off** on yaw stability and completion time; full user study (N≥10) can be continued by your lab partner after handoff.
