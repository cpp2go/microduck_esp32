# microduck_esp32

Microduck 在嵌入式侧拆成两颗芯片 + Linux 训练：

- **Linux** — [`microduck_rl`](../microduck_rl)：训练与 ONNX 导出 / bake（仅此）
- **ESP32-P4** — 本工程：**板端** 50 Hz INT8 策略、ICM-42688 SPI、飞特 1910 闭环
- **ESP32-C5** — [`c5_wireless/`](c5_wireless/)：无线与遥控命令（不写策略）

## 与原版 pollen-robotics/microduck 的对齐

对照真机路径 [`robotd` + `duck-control`](https://github.com/pollen-robotics/microduck)（**不是**其它 ESP 移植）。本仓库硬件选型与原版不同，观测/控制契约对齐：

| 项目 | 原版 | microduck_esp32 |
|------|------|-----------------|
| 控制频率 | 50 Hz | 50 Hz（core 1） |
| 观测 / 动作 | 61D / 14D（嘴跳过） | 同布局 |
| 舵机 | Dynamixel XL330 | **飞特 Feetech HD1910M / STS** |
| 关节读回 | SyncRead Present | SyncRead Pos+Speed @56 |
| 位置编码 | `(2π·raw/4096)−π` | 相同（4096/圈，中位 2048） |
| IMU | `imu_to_dxl` @200 | **ICM-42688 SPI** + Mahony |
| 命令块 / EMA | body x/y/yaw=0；alpha=0.2 | 同 |
| 推理 | 板载 Linux ONNX | **ESP32-P4 INT8 `duck_policy`** |
| 声音 / 视觉 | WAV+aplay / mediad | **ES8311+NS4150B** tag 音效 + synthetic 帧总线 |

详见 [docs/architecture.md](docs/architecture.md)、[docs/vision-sound-port.md](docs/vision-sound-port.md)、[docs/gap-audit-vs-original.md](docs/gap-audit-vs-original.md)。

## 快速开始

```bash
idf.py set-target esp32p4
idf.py menuconfig   # IMU SPI 引脚 + Feetech UART
idf.py build flash monitor
```

常用命令：`info` · `policy …` · `imu` · `servo stats|on|off` · `sound list|yes` · `vision status|grab|detect`

## 配置

- **Microduck IMU (ICM-42688 SPI)** — Host / SCLK / MOSI / MISO / CS / Hz  
  默认：SPI2，SCLK=9 MOSI=8 MISO=10 CS=11 @ 8 MHz
- **Microduck Feetech bus** — UART、半双工 DIR、P 增益（0..254）、action_scale / 低通

## 烘焙策略（主机工具 → 板端权重）

```bash
python3 tools/bake_policies.py --dir /path/to/onnx --out-dir .
idf.py build flash
sh host_test/run.sh   # golden + obs 布局（不替代板端推理）
```

## 目录

```
components/duck_policy/   板端 INT8 推理
components/imu/           ICM-42688 SPI
components/servo_bus/     Feetech STS/HD1910 SyncRead/Write
components/cmd_bus/       命令块 + EMA
components/sound/         ES8311 + NS4150B 情绪/cue tag
components/vision/        帧总线（synthetic；CSI 未接线）
main/main.cpp             50 Hz 环
```

