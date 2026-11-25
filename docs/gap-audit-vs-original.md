# 原版 microduck vs 本仓缺口盘点

基准仓库：[`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck)（Radxa Linux + Dynamixel XL330 + `imu_to_dxl`）。  
本仓：`cpp2go/microduck_esp32`（ESP32-P4 INT8 + **ICM-42688 SPI** + **飞特 1910 / 宇树 S288** + C5 无线）。

刷新：2026-11-25（相对 PR #1–#6 后再移植一轮）。

## 已对齐 / 已移植

| 项 | 状态 |
|----|------|
| 61D obs / 14D act / 50 Hz 环序 | 已对齐 |
| IMU → gyro + projected gravity | ICM-42688 SPI + Mahony；**另有 200 Hz fast task**（SPI 等价 `imu_to_dxl@200`） |
| 命令块 + EMA | `cmd_bus` + C5 peer-data |
| 板端策略推理 | P4 INT8 `duck_policy` |
| 声音播放 + 麦克风 | ES8311 + NS4150B；**PCM 银行**（robotd emotion WAV→16 kHz）+ synth 回退 |
| Emotion JSON 关键帧 | `components/emotion`（robotd/emotions/*.json） |
| Theremin（ToF 远近→音高/嘴） | `components/theremin` |
| 视觉帧总线入口 | 最小 synthetic / detect stub |
| ToF 距离 + 避障门控 | `components/tof` + autod 式 STOP/REACT |
| VL53L8CX 完整 ULD | **已厂商化**（`vendor/vl53l8cx` + ESP-IDF I2C platform）；`MICRODUCK_TOF_ULD` 默认关（固件 ~550 KB），开启后真测距 |
| 语音意图 | ES8311 VAD+频带（非 ASR；原版也无 ASR） |
| **技能状态机** | `components/skills`：walk/stand/sit/rise/ground_pick/kick/roulade/roller → 切换 bake 网络 |
| **Fall / safety** | `components/safety`：FallPredictor + fallen + NaN/±π clamp |
| Posture 粗分类 | `components/posture`（tilt + 关节折叠启发式；无 MuJoCo FK） |
| 50 Hz 校验助手 | `loop_validate` + shell `validate` |

## 仍缺 / 刻意不做

| 原版能力 | 说明 |
|----------|------|
| mediad / WebRTC / 真 CSI | Linux/V4L2；P4 未接相机 pipeline |
| duck-detect / pet-detect / ball YOLO | 无 RKNN/YOLO 权重；detect API 仍占位 |
| chorale / wheee 全合成人格 | theremin 最小版已有；完整 MIDI/人格栈未搬 |
| pickup CNN（ORT） | 训练侧 ORT 模型；板端不跑 FP ORT |
| Dynamixel fast sync / 0x8A | 飞特 STS 无此指令 |
| idle_head / head_imu（头模 LSM6） | 需第二颗 IMU 硬件 |
| LEDs / laser | 需原版 I²C 扩展器 / 激光外设 |
| soccer 全栈 | 缺球检测；kick bake 已可经 `skill kick` 触发 |
| Linux 守护进程整机 | autod 完整、mapd、btd、nfc、padd、updater、gateway、configd… |
| 训练 / 仿真 / kinematics MuJoCo | PC-only |

## 硬件刻意差异（不是漏移植）

| 原版 | 本仓 |
|------|------|
| Dynamixel XL330 | 飞特 1910 / 宇树 S288 |
| `imu_to_dxl` @200 on DXL bus | ICM-42688 SPI @200 Hz task |
| Linux ONNX Runtime | P4 INT8 bake |

## Shell 速查（本轮新增）

`skill` · `emotion` · `theremin` · `posture` · `validate` · `imu fast` · `sound list`（`*`=银行）
