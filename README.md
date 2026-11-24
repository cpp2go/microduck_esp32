# microduck_esp32

Microduck 在嵌入式侧拆成两颗芯片 + Linux 训练：

- **Linux** — [`microduck_rl`](../microduck_rl)：训练与 ONNX 导出 / bake（仅此）
- **ESP32-P4** — 本工程：**板端** 50 Hz INT8 策略推理、IMU、XL330 闭环
- **ESP32-C5** — [`c5_wireless/`](c5_wireless/)：无线与遥控命令（不写策略）

## 与原版 pollen-robotics/microduck 的对齐

对照真机路径 [`robotd` + `duck-control`](https://github.com/pollen-robotics/microduck)（**不是**其它 ESP 移植）：

| 项目 | 原版 (robotd / duck-control) | microduck_esp32 (本仓库) |
|------|------------------------------|---------------------------|
| 控制频率 | 50 Hz | 50 Hz（core 1） |
| 观测 | 61D：`obs.rs` 布局 | 同布局（见 `common/include/microduck_model.h`） |
| 动作 | 14D，嘴关节跳过 | `scatter` / `gather` 同映射 |
| 关节读回 | SyncRead Present 块 @124（12B） | 同址同长一次 SyncRead |
| 位置编码 | `(2π·raw/4096)−π` | 相同 |
| 写目标 | `HOME + action_scale × scatter` | 默认 scale=0.9；head/leg 低通 0.5/0.7 |
| 命令块 | body x/y/yaw 恒 0；z,roll,pitch | `cmd_bus_apply_to_obs` |
| 命令平滑 | `cmd_alpha=0.2` / `head_alpha=0.2` | Kconfig 同默认 |
| 推理 | 板载 Linux ONNX Runtime | **ESP32-P4 上 INT8 `duck_policy`**（由 ONNX bake，推理不在主机） |
| IMU | `imu_to_dxl` @200 同总线 | 暂用 P4 本地 BMI270（后续可接总线 IMU） |

推理核心（归一化 → Gemm+ELU MLP → 输出、运行时热切换、golden 自检）保持与原版策略契约一致，并**实际跑在 P4**。

更细的差异与未决项见 [docs/architecture.md](docs/architecture.md)。

## 快速开始

```bash
cd microduck_esp32
# 需要 ESP-IDF v5.5+，且已安装 esp32p4 工具链
idf.py set-target esp32p4
idf.py build flash monitor
```

上电后串口（USB Serial/JTAG）出现 `microduck>` 提示符：

- `info` — 芯片、PSRAM、当前策略
- `policy test` / `policy bench 100` / `policy list` / `policy use <name>`
- `policy loop [off]` / `policy stats` — 50 Hz 板端推理环
- `imu` — BMI270 实时读数
- `servo stats|on|off` — XL330 读回 / 急停断扭矩

## 配置

`idf.py menuconfig`：

- **Microduck IMU (BMI270)** — I2C 引脚
- **Microduck Dynamixel bus** — UART / action_scale / 低通 / 故障断扭矩
- **Microduck P4 ← C5 command link** — SDIO，以及 cmd/head EMA

## 烘焙 / 更新策略（主机工具 → 板端权重）

在**主机**上把 ONNX bake 成 C++ 权重，**推理只在 P4**：

```bash
python3 tools/bake_policies.py --dir /path/to/onnx --out-dir .
idf.py build flash
```

ONNX 来自 `microduck_rl`。默认烘焙 9 个官方策略（`alpha_walking` 为开机默认）。

主机自检（不替代板端推理）：

```bash
sh host_test/run.sh   # golden + obs 布局对齐原版
```

## 观测布局（与训练 / 原版一致）

50 Hz 循环中组装的 61 维向量：

- `[0:3]` 陀螺（trunk，rad/s）
- `[3:6]` 投影重力
- `[6:48]` 关节相对位置 / 速度 / 上次动作（14 槽，嘴不占位）
- `[48:61]` 命令块（twist + head + body；x/y/yaw=0）

## 目录结构

```
components/duck_policy/   板端 INT8 推理 + 烘焙权重
components/servo_bus/     XL330 SyncRead/Write + proprio → obs
components/imu/           BMI270
components/cmd_bus/       命令块 → obs[48:61] + EMA
main/main.cpp             自检、50 Hz 环、shell
tools/                    bake_policies.py（主机侧）
```
