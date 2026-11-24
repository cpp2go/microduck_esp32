# Microduck 双芯片 + Linux 训练架构

```
Linux (microduck_rl)  训练 → ONNX → bake INT8（主机工具）
        │
        ▼ 权重烧进固件
┌─────────────────── ESP32-P4 (host) ───────────────────┐
│ 50 Hz 板端 INT8 策略 · ICM-42688 SPI · Feetech 1910   │
│ esp_hosted SDIO host ←──────────────────────────────┐   │
└────────────────────────────────────────────────────│───┘
                                                      │ SDIO 4-bit
┌─────────────────── ESP32-C5 (slave) ───────────────┘
│ esp_hosted coprocessor · WiFi/BLE · OTA
│ microduck_cmd_publisher → peer-data → P4 cmd_bus
└────────────────────────────────────────────────────
```

**推理必须在 P4 上执行**（`components/duck_policy`）。主机只负责训练、bake 与 `host_test`。

## 与原版 pollen-robotics/microduck 对照

真机对照源：[`obs.rs`](https://github.com/pollen-robotics/microduck/blob/main/duck-control/src/obs.rs)、[`control.rs`](https://github.com/pollen-robotics/microduck/blob/main/robotd/src/control.rs)。硬件与原版 Radxa+XL330/`imu_to_dxl` 不同，但 **61D 观测 / 14D 动作 / 50 Hz 环序** 对齐。

| 能力 | 原版 | 本固件 |
|------|------|--------|
| 61D obs / 14D action | `Observation::build` | 同布局填充 |
| 关节总线 | Dynamixel XL330 Proto 2.0 | **飞特 Feetech HD1910M / STS（FT-SCS）** |
| 关节读回 | Present 块 SyncRead | SyncRead Pos+Speed @56（4B） |
| 位置编码 | `(2π·raw/4096)−π`，中位 2048 | 相同几何（1910 也是 4096/圈） |
| 写目标 | Goal Position | Goal Position @42 |
| IMU | `imu_to_dxl` SFLP @200 | **ICM-42688 SPI** + Mahony → 同 ImuData 契约 |
| `action_scale` / 低通 / 命令 EMA | robotd 默认 | Kconfig 同默认 |
| 推理 | Linux ONNX Runtime | **P4 INT8 MLP** |
| 声音 | robotd WAV + aplay | **最小**：tag → I2S/log 短音（[vision-sound-port.md](vision-sound-port.md)） |
| 视觉 | mediad V4L2 + detect | **最小**：synthetic 帧总线；detect 占位 |

完整缺口表见 [gap-audit-vs-original.md](gap-audit-vs-original.md)。

## 真机闭环（P4）

```
50 Hz (core 1):
  ICM-42688 SPI → gyro + projected gravity → obs[0:6]
  Feetech SyncRead → joint_rel/vel → obs[6:34]；last_action → obs[34:48]
  C5 cmd（EMA；body x/y/yaw=0）→ obs[48:61]
  板端 INT8 policy → action[14]
  scatter → HOME + action×scale（低通）→ SyncWrite Goal @42
```

连续读失败达阈值则真正 torque off。上电 SyncRead 成功时按 `seed_from_pose` 初始化 `last_action`。

Feetech STS @ 1 Mbps（默认）；半双工可用 `DIR` GPIO。

## SDIO + peer-data

| 方向 | 机制 | 载荷 |
|------|------|------|
| C5 → P4 | esp_hosted peer-data `MICRODUCK_MSG_ID_CMD` | `microduck_link_frame_t` |
| P4 接收 | → `cmd_bus_set` → obs[48:61] | 同上 |

## 仍开放（需真机）

- 安装姿态下 ICM / 舵机轴系标定（trunk 映射）
- robotd 技能/站立网络切换
- 实机 50 Hz 抖动与总线成功率
- CSI 真相机接线 + 检测；I2S DAC + WAV/PCM 资产（见 vision-sound-port.md）
