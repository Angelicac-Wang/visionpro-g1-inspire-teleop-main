# Vision Pro–G1 Teleoperation: Operations Runbook

This is the canonical copy-paste guide for daily operation. For installation, architecture, code ownership, and known limitations, see [`HANDOVER.md`](HANDOVER.md).

All commands are run from the repository root on the Linux lab workstation. Replace `<VP>` with the Vision Pro IP address or Tracking Streamer room code.

## Jetson AGX Orin checkpoint — 2026-10-08

本次进度已保存，回来后按以下顺序启动。当前网络记录：Jetson
`192.168.2.22`，Vision Pro `192.168.2.5`；地址变化时替换命令中的 IP。
三个终端均先进入 `~/projects/visionpro-g1-inspire-teleop-main`。

1. 终端一：`SONIC_HAND_MODEL=inspire SONIC_ENABLE_FPV=1 ./run_sonic_sim_loop.sh`。
   等待五指仿真窗口和 `Camera publish: ON`；报错时先保留终端输出。
2. 终端二：`./run_sonic_deploy.sh`，等待 `Init Done`。
3. 头显打开 Tracking Streamer，终端三运行：

   ```bash
   bash solo_teleop.sh 192.168.2.5 --loco-velocity-gain 0 --loco-yaw-gain 0 --no-head-height-squat
   ```

脚本自动执行 `F → ]`。提示落地时点击 Jetson 的 MuJoCo 窗口按 `9`，
回到终端三按回车，保持校准姿势，脚本继续执行 `S → T`。这套命令先关闭
头部行走、转向和下蹲输入，方便检查手臂与手指。三个进程需要持续运行。
桥接在 tmux 中，关闭终端窗口不一定会停止它；重新运行脚本会替换 `tele`
会话。停止桥接：`tmux send-keys -t tele o`。

已完成：修复 SONIC 在 ENGAGE 切换 motion 后丢失 VR encoder mode 的问题，
已在 AGX 编译并替换部署程序，操作者确认手臂可以跟随；仿真入口改为
Inspire FTP 五指手，接入六通道手指命令。五指仿真的 10 项协议、动力学和
ZMQ 测试通过，也完成相机渲染检查。仍需操作者重新启动后验证完整头显链路。

待复测：最后一次现场检查中仿真和相机进程已退出，5555 端口在 4 秒内
收到 0 帧；只有视频服务 ready 不能证明头显已经收到图像，退出原因尚不明确。
更早的 20 秒采样虽持续收到桥接消息，但姿态和手指值完全不变，不能证明
头显产生了新追踪帧。启动后分别检查张手、握拳、手臂移动和头显画面。
左右手校准 JSON 尚未生成，拇指旋转命令曾固定为 800；需要单独做手指
张开/握拳校准，`F/S` 不会校准手指范围。实机 Inspire 驱动仍是独立链路，
本次新增适配器仅控制仿真。

### 保存的 SONIC 补丁与编译脚本

上游 GR00T 仓库不是个人仓库，因此其源码修复以
[`sonic-vr-encoder-mode.patch`](../tools/jetson/sonic-vr-encoder-mode.patch)
保存在这里。补丁基于 GR00T commit
`b042411fae38ee4d1af9aac82a37a1f8d14d6dd0`。当前机器已经应用并编译，
正常重启无需重新应用或编译。另一份兼容的干净源码可按以下方式恢复：

```bash
cd ~/projects/visionpro-g1-inspire-teleop-main
git -C ../GR00T-WholeBodyControl apply --check "$PWD/tools/jetson/sonic-vr-encoder-mode.patch"
git -C ../GR00T-WholeBodyControl apply "$PWD/tools/jetson/sonic-vr-encoder-mode.patch"
cp tools/jetson/build_sonic_deploy.sh tools/jetson/copy_sonic_libs.sh ../
cd ..
bash build_sonic_deploy.sh
```

脚本须放在 `~/projects`，与 GR00T、`sonic_libs`、`jetson-teleop` 同级。
它依赖现有 ONNX Runtime 1.16.3 和 TensorRT 10.7/CUDA 12.6 运行库；若未
复制运行库，先运行 `bash copy_sonic_libs.sh <旧Jetson用户@地址>`。
构建依赖解压到本地目录，不需要 sudo。实际编译、动态库检查和 usage
冒烟检查已通过；上游 FK 测试因缺少 `reference/bones_072925_test` 数据而跳过。
仓库保存源码、补丁与脚本；运行库、模型下载缓存、构建产物、`.env` 和运行日志
保留在本机。五指模型可由 `tools/setup_inspire_sim.py` 按固定上游版本重新生成。

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

This disables video only; simulated Inspire fingers remain enabled. Use
`--no-inspire-hand-sim` to disable simulated fingers independently.

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
./run_sonic_avp_teleop.sh <VP> --no-mujoco-fpv --no-inspire-hand-sim
```

For keyboard-assisted positioning:

```bash
./run_sonic_avp_teleop.sh <VP> \
  --no-mujoco-fpv \
  --no-inspire-hand-sim \
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
  --no-inspire-hand-sim \
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

### Simulated Inspire fingers do not move

The standard simulator launcher now loads Inspire FTP five-finger hands and
subscribes directly to `tcp://127.0.0.1:5556/inspire_hand`. The default upstream
`scene_43dof.xml` contains Dex3 three-finger hands and cannot consume that topic.
`SONIC_HAND_MODEL=dex3` explicitly selects the original upstream simulator.

After changing the hand model, stop the bridge, deploy, and simulator; restart
the simulator first, then deploy, then the existing `solo_teleop.sh` workflow.
No C++ rebuild is needed for the hand simulation adapter. The simulator prints:

```text
[inspire_sim] Five-finger FTP hands listening on .../inspire_hand
[inspire_sim] Hand commands received; finger tracking active.
```

The six commands are little, ring, middle, index, thumb bend, and thumb rotation.
Each hand has six driven joints and six coupled joints. Commands stop updating
for 0.5 seconds → the simulated fingers open. Body control stays on the existing
29-motor DDS interface. For a bridge on another host/port, set
`SONIC_HAND_ENDPOINT=tcp://HOST:PORT` on the simulator.

Assets are generated by `tools/setup_inspire_sim.py` from the official
[Unitree G1 Inspire FTP description](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf),
pinned to commit `5994d4faef0a9cadd3287f8de0199a67eeb2a259`. Downloaded URDFs,
meshes, and their BSD license are cached under `assets/mujoco/inspire_ftp/`;
the assembled model and scenes are under `assets/mujoco/g1_runtime/`.
The first standard simulator launch generates them if absent. The original
GR00T body model and its camera are retained; the Dex3 palms/fingers are replaced.

If fingers move but cannot fully close, run a separate open/fist calibration;
`F → ] → S → T` calibrates head/wrists, not finger ranges. The per-hand calibration
JSON files are ignored by Git and must be copied or regenerated on a new machine.

### Physical Inspire fingers do not move

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
