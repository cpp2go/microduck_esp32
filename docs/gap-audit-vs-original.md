# 原版 microduck vs 本仓缺口盘点

基准仓库：[`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck)（Radxa Linux + Dynamixel XL330 + `imu_to_dxl`）。  
本仓：`cpp2go/microduck_esp32`（ESP32-P4 INT8 + **ICM-42688 SPI** + **飞特 1910 STS** + C5 无线）。

## 已对齐 / 已移植

| 项 | 状态 |
|----|------|
| 61D obs / 14D act / 50 Hz 环序 | 已对齐（`obs` 布局 + Feetech SyncRead/Write） |
| IMU → gyro + projected gravity | ICM-42688 SPI + Mahony（契约同 `ImuData`） |
| 命令块 + EMA（body x/y/yaw=0） | `cmd_bus` + C5 peer-data |
| 板端策略推理 | P4 INT8 `duck_policy`（非 Linux ONNX） |
| 声音 tag 播放入口 | **ES8311 + NS4150B** 硬件通路 + tag 合成 cue（见 [vision-sound-port.md](vision-sound-port.md)） |
| 视觉帧总线入口 | **最小等价**：`components/vision` synthetic（同上） |

## 视觉 / 声音（优先项）— 仍缺

| 原版能力 | 缺口 |
|----------|------|
| mediad 相机 + pipeline + WebRTC | Linux/V4L2；P4 无真实 CSI 驱动接线 |
| duck-detect / pet-detect / ball | 无 YOLO/RKNN；detect API 占位 |
| WAV 语音库 + aplay | 无银行；板端合成短音经 ES8311/NS4150B |
| theremin / wheee / chorale | 未移植 |
| emotion JSON 驱动姿态/表情 | 未移植（仅 tag 名） |

## 控制 / 总线仍缺

| 原版 | 说明 |
|------|------|
| **技能状态机**（`robotd/src/control.rs`） | walk/stand/sit/rise、kick、roulade、ground_pick、可配置 skill 窗口与网络切换 — 本仓仅单策略 loop |
| **多网络调度**（stand / sitstand / skill） | 权重可 bake 并用 `policy use` 手动切换；**无** robotd 按意图自动选网 |
| **`imu_to_dxl` @200 Hz** | 原版 IMU 经舵机总线旁路融合；本板 ICM 直连 SPI，**无** 200 Hz `imu_to_dxl` 路径（有意不同硬件） |
| **Dynamixel fast sync / 0x8A** | XL330 Proto 2.0 特性；飞特 STS 用 Pos+Speed SyncRead + Goal SyncWrite，**无** 0x8A |
| pickup / fall / safety 完整逻辑 | duck-control 有；本仓仅连续读失败 torque-off 等基础安全 |
| standing gain / skill tuning（`robotd-params`） | 未移植参数表与运行时切换 |
| head idle / head IMU 行为 | `idle_head.rs` / `head_imu.rs` 未移植 |
| LEDs / laser | 未移植 |
| soccer / intents / posture 高层 | 未移植 |

## 原版其它守护进程 / 板级外设（整机能力，非本 PR 范围）

多数依赖 Linux 用户态或 Radxa 外设，**未** 移植到 P4：

- `autod`（自主/起身/障碍）、`mapd` / `maploc` / `odometry`、`tof`（VL53）
- `btd` / `duck-ble`、`nfc`、`padd` / `pad-imu` / `pad-map`、`netpadd`
- `configd`、`updater`、`duck-gateway`、`duck-ether`、`duckctl` / `robotctl`
- `kinematics`（MuJoCo FK 资产）、训练/仿真侧（本仓只消费 bake 权重）

## 硬件刻意差异（不是“漏移植”）

| 原版 | 本仓（约束） |
|------|----------------|
| Dynamixel XL330 | 飞特 Feetech 1910 STS |
| BMI/`imu_to_dxl` 链 | ICM-42688 SPI |
| Linux ONNX Runtime | **P4 板端 INT8** |

## 建议后续优先级

1. 真机 CSI（或外置 USB 相机经主机）接到 `vision_grab`，再决定板端轻量检测 vs 主机检测。  
2. 嵌入短 PCM/WAV 替代合成 cue（硬件通路已就位）。  
3. 技能状态机 + 多网络 bake（stand/sit），与原版 `control.rs` 调度对齐。  
4. 标定 ICM/舵机轴系与 trunk 映射。
