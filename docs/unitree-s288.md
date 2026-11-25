# Unitree S288 舵机支持

对照飞特 1910（FT-SCS）新增可选后端：**宇树 Unitree S288 / J288**。控制契约不变：61D obs / 14D act / 50 Hz / P4 INT8。IMU 与音频选型不动。

## 协议差异（相对飞特 STS）

| | 飞特 1910 STS | Unitree S288 |
|--|---------------|--------------|
| 物理层 | 半双工 TTL | 半双工 TTL（单总线） |
| 波特率 | 默认 1 Mbps | **固定 6 Mbps** |
| 帧 | `FF FF` + checksum | 控制 20 B / 反馈 26 B + **CRC-32** |
| 头 | — | 命令 `FE EE`，反馈 `FC EE` |
| 多机 | SyncRead `0x82` / SyncWrite `0x83` | **无 Sync***；逐 ID 请求–响应 |
| 位置 | 4096/圈编码器 | 转子 raw → 输出轴 rad（减速比 ≈288.35） |
| ID | 常用 10–34（本仓 alpha 表） | **0–14**（15=广播无应答） |

规范：[unitreerobotics/digital_servo specs/protocol.md](https://github.com/unitreerobotics/digital_servo/blob/main/specs/protocol.md)。

## menuconfig 切换

`Component config` → **Microduck servo bus**：

1. **Servo bus protocol** → `Unitree S288 / J288`（或保持 `Feetech 1910`）
2. 波特率默认随协议切换为 `6000000` / `1000000`
3. TX/RX/DIR GPIO 可配（默认仍 TX=20 RX=19 DIR=-1）
4. S288：`Kp`/`Kd`（×1000，输出轴 N·m 单位）

`sdkconfig.defaults` 默认仍为 **Feetech**，避免误连 6 Mbps。

## 接线与 ID

- 半双工 TTL；6 Mbps 需板级 UART 支持（ESP32-P4 一般可用）。
- 将 15 个关节按本仓 slot 顺序设为 ID **0..14**（左腿 → 头/嘴 → 右腿，与 `MICRODUCK_S288_JOINT_IDS` 一致）。
- 飞特 alpha ID 表（20/21/…）**不能**直接用于 S288。

## 控制环

`servo_bus_poll` / `write_goals_rel` 对上层透明：S288 后端在内部对每个 ID 发送 FOC 位置指令并解析反馈，填充同一 `pos_rel` / `vel` 观测布局。

## 验证

```bash
sh host_test/run.sh   # 含 ut_s288_codec
```

板端：`servo stats`；切换协议后重新 `idf.py build flash`。
