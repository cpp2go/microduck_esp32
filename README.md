# microduck_esp32

Microduck 在嵌入式侧拆成两颗芯片 + Linux 训练：

- **Linux** — [`microduck_rl`](../microduck_rl)：训练与 ONNX（仅此）
- **ESP32-P4** — 本工程：50 Hz INT8 策略、IMU、动作输出
- **ESP32-C5** — [`c5_wireless/`](c5_wireless/)：无线与遥控命令（不写策略）

从 [`esp-duck`](../esp-duck) 移植的推理栈与 **61 维观测** 约定不变。架构图见 [docs/architecture.md](docs/architecture.md)。

## 与 esp-duck (ESP32-S3) 的差异

| 项目 | esp-duck | microduck_esp32 |
|------|----------|-------------------|
| 目标芯片 | ESP32-S3 (Xtensa) | **ESP32-P4** (RISC-V 双核) |
| 权重放置 | `SPIRAM_RODATA`（CoreS3 8MB QUAD PSRAM） | `SPIRAM_XIP_FROM_PSRAM` |
| IMU I2C | 硬编码 GPIO 12/11（M5 CoreS3） | **Kconfig 可配**（默认 GPIO 2/3） |
| 策略 / 工具链 | 相同 `duck_policy` + `bake_policies.py` | 相同（已随仓库拷贝） |

推理核心（归一化 → Gemm+ELU MLP → 输出、运行时热切换、golden 自检）未改，便于与真机
`pollen-robotics/microduck` 运行时对齐。

## 快速开始

```bash
cd microduck_esp32
# 需要 ESP-IDF v5.5+，且已安装 esp32p4 工具链
idf.py set-target esp32p4
idf.py build flash monitor
```

上电后串口（USB Serial/JTAG）出现 `microduck>` 提示符。常用命令与 esp-duck 相同：

- `info` — 芯片、PSRAM、当前策略
- `policy test` / `policy bench 100` / `policy list` / `policy use <name>`
- `policy loop [off]` / `policy stats` — 50 Hz 推理环
- `imu` — BMI270 实时读数（若已接线且 `CONFIG_MICRODUCK_IMU_ENABLE=y`）

## 配置 IMU 引脚

`idf.py menuconfig` → **Microduck IMU (BMI270)**：

- `CONFIG_MICRODUCK_IMU_SDA_GPIO` / `CONFIG_MICRODUCK_IMU_SCL_GPIO` — 按你的 P4 载板修改
- 无 IMU 时可关闭 `CONFIG_MICRODUCK_IMU_ENABLE`，循环仍跑但陀螺/重力为合成数据

## 烘焙 / 更新策略

与 esp-duck 相同，在项目根目录：

```bash
python3 tools/bake_policies.py --dir /path/to/onnx --out-dir .
idf.py build
```

ONNX 来自 `microduck_rl`：

```bash
cd ../microduck_rl
uv run scripts/export.py Mjlab-Velocity-Flat-MicroDuck --wandb-run-path <entity/project/run>
```

默认烘焙 9 个官方策略（`alpha_walking` 为开机默认），见 `tools/bake_policies.py` 中的 `MODELS` 列表。

## 观测布局（与训练一致）

50 Hz 循环中 `main.cpp` 组装的 61 维向量与 `microduck_rl` / 真机运行时一致：

- `[0:3]` 陀螺（trunk，rad/s）
- `[3:6]` 投影重力（单位向量，直立 ≈ `0, 0, -1`）
- `[6:20]` 关节位置 / 速度 / 上次动作（当前无舵机总线时为 0）
- `[48:61]` 命令块（twist + head + body）

策略输出 14 维关节目标；**尚未接 Dynamixel 总线**时需由后续 `servo` 组件消费 `duck::Run()` 的结果。

## 目录结构

```
components/duck_policy/   INT8 推理 + 烘焙好的 duck_models.cpp
components/imu/             BMI270 + Mahony（Kconfig 引脚）
components/shell/           USB 交互 REPL
main/main.cpp               自检、50 Hz 环、命令注册
tools/                      extract_policy.py, bake_policies.py
```

## 后续集成建议

1. **舵机 / 状态机**：从 `imu::Poll()` + 编码器读数填充 obs 的关节段，将 `act` 写入 BAM/位置 PD。
2. **与 C6 协处理器**：若 Wi‑Fi 在外挂 ESP32-C6 上，P4 专注 50 Hz 控制环（参考 xiaozhi-esp32 P4 分区实践）。
3. **重新烘焙**：每次更换 Hub 上的 ONNX 后必须跑 `bake_policies.py`，否则 golden 自检会拒绝切换。