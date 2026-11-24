# Microduck 双芯片 + Linux 训练架构

```
Linux (microduck_rl)  训练 → ONNX → bake INT8（主机工具）
        │
        ▼ 权重烧进固件
┌─────────────────── ESP32-P4 (host) ───────────────────┐
│ 50 Hz 板端 INT8 策略 · BMI270 · XL330 闭环 · policy_out │
│ esp_hosted SDIO host ←──────────────────────────────┐   │
└────────────────────────────────────────────────────│───┘
                                                      │ SDIO 4-bit
┌─────────────────── ESP32-C5 (slave) ───────────────┘
│ esp_hosted coprocessor · WiFi/BLE · OTA
│ microduck_cmd_publisher → peer-data → P4 cmd_bus
└────────────────────────────────────────────────────
```

**推理必须在 P4 上执行**（`components/duck_policy`）。主机只负责训练、bake 与 `host_test` 布局/golden 校验，不承担真机控制推理。

## 与原版 pollen-robotics/microduck 对照

真机对照源：[`duck-control/src/obs.rs`](https://github.com/pollen-robotics/microduck/blob/main/duck-control/src/obs.rs)、[`bus.rs`](https://github.com/pollen-robotics/microduck/blob/main/duck-control/src/bus.rs)、[`robotd/src/control.rs`](https://github.com/pollen-robotics/microduck/blob/main/robotd/src/control.rs)。

| 能力 | 原版 | 本固件 |
|------|------|--------|
| 61D obs / 14D action | `Observation::build` | `main` + `servo_bus_write_obs_proprio` + `cmd_bus_apply_to_obs` |
| SyncRead | 12B @124（可含 imu@200） | 15× XL330 同块；IMU 暂用板载 BMI270 |
| 位置编码 | `(2π·raw/4096)−π` | 相同 |
| `action_scale` / 低通 | 0.9 / head0.5 / leg0.7 | Kconfig 同默认 |
| 命令 EMA | cmd_alpha / head_alpha = 0.2 | 同默认 |
| `return_delay_time` / `current_limit` | 0 / 1000 mA | 配置舵机时写入 |
| 推理运行时 | Linux ONNX Runtime | **P4 INT8 MLP（bake 自 ONNX）** |
| 技能/坐立网络切换等 | robotd 状态机 | 未移植（仍走当前激活策略） |

## SDIO + peer-data（默认）

| 方向 | 机制 | 载荷 |
|------|------|------|
| C5 → P4 遥控 | `esp_hosted_send_custom_data(MICRODUCK_MSG_ID_CMD, …)` | `microduck_link_frame_t` |
| P4 接收 | callback → `cmd_bus_set` → obs[48:61] | 同上 |

UART 仅 `menuconfig` 选 **UART (bench)** 时用于无共板台架。

## 真机闭环（P4）

```
50 Hz (core 1):
  BMI270 → obs[0:6]
  SyncRead Present@124 → joint_rel/vel → obs[6:34]；last_action → obs[34:48]
  C5 cmd（EMA 后；body x/y/yaw=0）→ obs[48:61]
  板端 INT8 policy → action[14]
  scatter → HOME + action×scale（head/leg 低通）→ SyncWrite Goal Position
```

连续读失败达阈值则真正 `torque off`。上电 SyncRead 成功时按 robotd `seed_from_pose` 初始化 `last_action`。

Protocol 2.0 @ 1 Mbps。关节表见 `common/include/microduck_model.h`。

## 固件路径

| 路径 | 目标 |
|------|------|
| 本仓库根 | P4 host（策略 + 控制） |
| `c5_wireless/esp_hosted_slave_patch/` | C5 SDIO slave 补丁 |
| `microduck_rl/`（外部） | 仅 Linux 训练 |

## 仍开放（需真机）

- 总线 `imu_to_dxl@200` 与舵机同事务读
- fast sync read `0x8A`
- robotd 技能/站立网络切换、电压补偿 PWM、impact guard
- 实机 50 Hz 抖动与 SyncRead 成功率测量
