# Vision Pro–G1 Teleoperation: Operations Runbook

This is the canonical copy-paste guide for daily operation. For installation, architecture, code ownership, and known limitations, see [`HANDOVER.md`](HANDOVER.md).

All commands are run from the repository root on the Linux lab workstation. Replace `<VP>` with the Vision Pro IP address or Tracking Streamer room code.

## 1. Every-session calibration

In the AVP bridge terminal:

```text
F → ] → S → T
```

- `F` — hold a forearms-forward L-shape for about two seconds; records head and wrist references.
- `]` — engage the SONIC policy; wait until the robot is stable.
- `S` — match the displayed robot arm pose and hold for about two seconds.
- `T` — begin live teleoperation.
- `H` — re-zero head facing and squat height.
- `P` — pause or resume pose mapping.
- `o` — stop and exit from the bridge terminal.
- `O` — emergency stop from the SONIC deploy terminal.

In MuJoCo, press `9` in the simulator window after `]` if the robot remains suspended by the elastic band.

## 2. MuJoCo whole-body teleoperation

Start three terminals in order.

Terminal 1:

```bash
./run_sonic_sim_loop.sh
```

Terminal 2:

```bash
./run_sonic_deploy.sh
```

Terminal 3:

```bash
./run_sonic_avp_teleop.sh <VP>
```

Complete `F → ] → S → T`.

The experimental MuJoCo first-person stream starts by default but is not reliably visible inside Vision Pro. Disable it when unnecessary:

```bash
./run_sonic_avp_teleop.sh <VP> --no-mujoco-fpv
```

## 3. MuJoCo pick-and-place

Terminal 1:

```bash
./run_sonic_sim_loop_pnp.sh
```

Terminal 2:

```bash
./run_sonic_deploy.sh
```

Terminal 3:

```bash
./run_sonic_avp_teleop_pick.sh <VP>
```

Complete `F → ] → S → T`. Pinching in Vision Pro closes the simulated Inspire fingers.

Keyboard hybrid locomotion is not enabled automatically. To use it:

```bash
./run_sonic_avp_teleop_pick.sh <VP> --hybrid-locomotion
```

## 4. Real G1 preflight

> Keep the robot hoisted for initial engagement. Clear people and equipment from its reachable area, and keep the Unitree remote ready.

Before running SONIC:

1. Connect the workstation Ethernet interface to the `192.168.123.x` robot network.
2. Confirm `.env` contains the correct `SONIC_NET_IF`, such as `enp3s0`.
3. Hoist the G1 with its feet initially off the ground.
4. Power on and wait for zero-torque mode; joints should move freely by hand.
5. Press `L2+R2` on the Unitree remote until the robot enters debug mode with the yellow LED and damping behavior.
6. Confirm no person is within arm or leg reach.

If deploy repeatedly reports `Failed to switch to Release Mode`, the sport controller is still active. Re-enter debug mode or reboot the robot and repeat the preflight.

## 5. Real G1 without physical finger control

Terminal 1:

```bash
./run_sonic_deploy.sh real
```

Terminal 2:

```bash
./run_sonic_avp_teleop.sh <VP> --no-mujoco-fpv
```

For keyboard-assisted positioning:

```bash
./run_sonic_avp_teleop.sh <VP> \
  --no-mujoco-fpv \
  --hybrid-locomotion
```

Complete `F → ] → S → T` slowly and verify stability after every stage.

## 6. Real G1 with both Inspire Hands

Check both hands before starting:

```bash
ping -c 2 192.168.123.210
ping -c 2 192.168.123.211
```

Terminal 1:

```bash
./run_sonic_deploy.sh real
```

Terminal 2:

```bash
./run_both_hand_driver.sh --dds-network enp3s0
```

Terminal 3:

```bash
./run_sonic_avp_teleop.sh <VP> \
  --no-mujoco-fpv \
  --enable-inspire-hand-dds \
  --hand-dds-sides both \
  --hand-dds-network enp3s0
```

Add `--hybrid-locomotion` to the Terminal 3 command when keyboard positioning is needed.

For one physical hand only:

```bash
# Left hand driver
./run_both_hand_driver.sh --sides l --dds-network enp3s0

# Add to the teleop command
--enable-inspire-hand-dds --hand-dds-sides l --hand-dds-network enp3s0
```

Hand-driver frequency output confirms Modbus communication with the hand. It does not prove that DDS commands are arriving from the bridge.

## 7. Hybrid keyboard locomotion

Enable:

```text
--hybrid-locomotion
```

After reaching `T`, focus the bridge terminal and hold:

- `W` — walk forward;
- `S` — walk backward while preserving facing;
- `,` / `.` — strafe left/right;
- `A` / `D` or `j` / `l` — turn in place;
- `space` or `r` — stop while preserving facing.

Come to a complete stop before reversing or turning in place. Before `T`, `S` means arm synchronization rather than backward walking.

## 8. Common bridge options

Append options after `<VP>`:

```bash
./run_sonic_avp_teleop.sh <VP> [OPTIONS]
```

- `--hybrid-locomotion` — enable keyboard/head locomotion together.
- `--no-mujoco-fpv` — disable the experimental first-person video path.
- `--loco-max-speed 0.4` — reduce maximum walking speed.
- `--no-loco-imu-correction` — disable base-IMU yaw correction for diagnosis.
- `--active-hands right` — ignore left-arm input.
- `--left-wrist-orientation-mode neutral` — diagnostic fallback for left-arm hunching.
- `--enable-inspire-hand-dds` — publish physical Inspire Hand commands.
- `--hand-dds-sides both` — select both hand DDS topics.
- `--hand-dds-network enp3s0` — select the hand DDS interface.
- `--print-debug` — print command and tracking state; standard wrappers already enable it.

Defaults in the standard wrapper:

- head locomotion and squat control: on;
- staged calibration: on;
- arm tracking-loss hold: on;
- IMU yaw correction: on;
- keyboard hybrid mode: off;
- command publish rate: 50 Hz.

## 9. Normal shutdown and emergency stop

Normal shutdown:

1. Press `o` in the bridge terminal and confirm it exits.
2. Stop the hand drivers with `Ctrl+C`.
3. Stop SONIC deploy.
4. Stop MuJoCo if running.

Unexpected real-robot movement:

1. Press `O` in the SONIC deploy terminal immediately.
2. Use the Unitree remote emergency control if required.
3. Do not restart until the cause is understood.

Do not rely on closing a terminal window as the primary emergency-stop method.

## 10. Troubleshooting

### CALIB_SYNC fails

- Confirm SONIC deploy is running.
- Confirm SONIC feedback is available on ZMQ port `5557`.
- Restart the bridge and repeat `F → ] → S → T`.

### Robot does not walk

- Confirm live teleoperation has reached `T`.
- Press `H` to re-zero facing.
- Check bridge debug output for nonzero movement and speed.
- For keyboard control, confirm `--hybrid-locomotion` was passed.

### Walking direction or facing drifts

- Face the intended neutral direction and press `H`.
- Repeat `F` if needed.
- Confirm base-IMU feedback is reaching the bridge.

### Arms move suddenly when the policy starts

- Start the bridge before pressing `]`.
- Complete `F` so the initialized arm targets are buffered.
- Do not skip `S`.

### A wrist disappears and the arm moves incorrectly

- Confirm arm tracking hold has not been disabled.
- Pause with `P` if tracking does not recover.

### Left arm pulls the torso or hunches

Try one diagnostic change at a time:

```text
--active-hands right
--left-wrist-orientation-mode neutral
```

Also recheck the synchronized pose at `S`.

### Physical fingers do not move

- Ping both hand IPs.
- Confirm the hand drivers are running.
- Confirm all Inspire DDS flags are present in the bridge command.
- Use the same network interface for the bridge and drivers.

### Python import or ZMQ error

- Confirm `SONIC_PYTHON` points to the intended environment.
- Run `"$SONIC_PYTHON" -m pip install -e .`.
- Verify `"$SONIC_PYTHON" -c "import zmq"` succeeds.

### SONIC cannot find the real robot

- Confirm the workstation is on `192.168.123.x`.
- Check `SONIC_NET_IF`.
- Confirm the G1 is in debug mode.

### MuJoCo waits for camera frames

Start the standard MuJoCo loop or disable video with:

```text
--no-mujoco-fpv
```
