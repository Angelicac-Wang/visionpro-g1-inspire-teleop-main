# Vision Pro 控制 G1 右臂和 Inspire 灵巧手使用说明

本文档说明如何使用 `visionpro_g1_right_arm_hand.py` 通过 Apple Vision Pro 控制 G1 右臂和 Inspire 灵巧手，重点是新版手指映射算法的启动、标定和调参。

## 1. 程序结构

主要文件：

- `scripts/visionpro_g1_right_arm_hand.py`：主启动脚本，负责接收 Vision Pro 追踪数据，控制 G1 右臂，并发布 Inspire hand DDS 命令。
- `scripts/avp_inspire_hand_mapping.py`：新版手部映射模块，负责把 AVP 手指骨架转换成 Inspire hand 六路角度命令。
- `scripts/Headless_driver_r.py`：Inspire hand 硬件驱动，负责监听 DDS 命令并写入灵巧手硬件。

Inspire hand 命令顺序保持为：

```text
[小拇指, 无名指, 中指, 食指, 大拇指弯曲, 大拇指根部旋转]
```

## 2. 启动顺序

需要开两个终端。

### 终端 1：启动 Inspire hand 驱动

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/Headless_driver_r.py
```

注意：当前 `Headless_driver_r.py` 默认使用：

```bash
--lr l
```

所以主控制脚本默认发布到 `rt/inspire_hand/ctrl/l`，两者是匹配的。

### 终端 2：启动 Vision Pro 控制

只调手，不动 G1 手臂：

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --print-debug
```

同时控制右臂和手：

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --print-debug
```

## 3. 第一次使用：手部标定

新版算法支持用你的 AVP 手部数据做开合标定。建议第一次使用先只调手并完成标定：

```bash
cd /mnt/newssd/visionpro-g1-inspire-teleop

/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --calibrate-hand \
  --print-debug
```

程序会提示两次：

1. 张开手，五指自然伸直，然后按 Enter。
2. 做闭合/握拳/捏合动作，然后按 Enter。

完成后会保存：

```text
scripts/visionpro_right_hand_calibration.json
```

之后普通启动时会自动读取该文件，不需要每次都加 `--calibrate-hand`。

## 4. 新版手指映射算法

默认使用：

```bash
--hand-map-mode angle
```

它会用 AVP 骨架计算每根手指的关节角度：

- 四指：MCP/PIP/DIP 类似关节角度加权求和。
- 拇指弯曲：单独计算拇指近端和远端弯曲。
- 拇指根部旋转：计算拇指基节方向相对掌面的旋转。

如果需要临时切回旧算法，可以加：

```bash
--hand-map-mode legacy
```

## 5. 大拇指方向修正

如果出现这种情况：

- 你的大拇指向内旋转；
- 机器人拇指却向外旋转；

启动时加：

```bash
--invert-thumb-rotation-command
```

推荐测试命令：

```bash
/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/visionpro_g1_right_arm_hand.py \
  --dds-network enp3s0 \
  --avp-endpoint 192.168.2.45 \
  --disable-arm \
  --print-debug \
  --invert-thumb-rotation-command
```

如果只是 AVP 里计算出来的拇指旋转几何方向需要翻转，可以试：

```bash
--flip-thumb-rotation
```

通常你现在遇到的“机器人拇指方向相反”优先用：

```bash
--invert-thumb-rotation-command
```

## 6. 常用参数

### 只控制手

```bash
--disable-arm
```

用于安全调手，不发送 G1 手臂命令。

### 打印调试信息

```bash
--print-debug
```

会打印类似：

```text
hand_cmd=[1000, 1000, 1000, 1000, 800, 200] | raw little/ring/middle/index=[...] -> [...] | thumb_bend=... thumb_rot=...
```

其中：

- `hand_cmd` 是最终发给 Inspire hand 的六路角度。
- `raw` 是 AVP 几何测量值。
- `->` 后面是归一化后的 0 到 1 控制量。

### 手指运动幅度

四指：

```bash
--finger-range-scale 1.15
```

拇指弯曲：

```bash
--thumb-bend-range-scale 1.35
```

拇指旋转：

```bash
--thumb-rotation-range-scale 1.25
```

数值更大，机器人使用更大的运动范围；数值更小，动作更保守。

### 平滑程度

四指：

```bash
--finger-smoothing 0.45
```

拇指：

```bash
--thumb-smoothing 0.35
```

数值越大响应越快，数值越小越平滑。

### Inspire hand 开合角度

四指打开/闭合：

```bash
--open-angle 1000
--close-angle 0
```

拇指弯曲：

```bash
--thumb-bend-open-angle 800
--thumb-bend-close-angle 200
```

拇指根部旋转：

```bash
--thumb-rotation-open-angle 200
--thumb-rotation-close-angle 800
```

如果拇指初始平面位置不对，可以重点调 `thumb-rotation-open-angle` 和 `thumb-rotation-close-angle`。

## 7. 推荐调试流程

1. 启动 `Headless_driver_r.py`。
2. 用 `--disable-arm --print-debug` 只调手。
3. 第一次先运行 `--calibrate-hand`。
4. 观察 `hand_cmd` 是否随手指变化。
5. 如果四指方向正确，但拇指根部方向反，加 `--invert-thumb-rotation-command`。
6. 如果运动幅度不够，适当增大 `--finger-range-scale` 或 `--thumb-rotation-range-scale`。
7. 手部满意后，去掉 `--disable-arm`，再让右臂一起运动。

## 8. 常见问题

### AVP 控制脚本在打印 hand_cmd，但机器手不动

通常是 Inspire hand 驱动没启动。先确认另一个终端正在运行：

```bash
/mnt/newssd/conda_envs/inspire_clean/bin/python \
  scripts/Headless_driver_r.py
```

### 加了 --calibrate-hand 但机器手没动

标定阶段只采集 AVP 手部数据，不控制机器手。完成两次 Enter 并保存 calibration 文件后，才会进入正常控制循环。

### 大拇指向内/向外反了

加：

```bash
--invert-thumb-rotation-command
```

### 想回到旧算法对比

加：

```bash
--hand-map-mode legacy
```

### DDS topic 不匹配

当前默认发布到：

```text
rt/inspire_hand/ctrl/l
```

如果驱动监听右手 topic，则启动主脚本时加：

```bash
--hand-topic-side r
```
