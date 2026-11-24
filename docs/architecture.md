# Microduck 双芯片 + Linux 训练架构

```
Linux (microduck_rl)  训练 → ONNX → bake INT8
        │
        ▼
┌─────────────────── ESP32-P4 (host) ───────────────────┐
│ 50 Hz INT8 策略 · IMU · policy_output                    │
│ esp_hosted SDIO host ←──────────────────────────────┐   │
└────────────────────────────────────────────────────│───┘
                                                      │ SDIO 4-bit
┌─────────────────── ESP32-C5 (slave) ───────────────┘
│ esp_hosted coprocessor · WiFi/BLE · OTA
│ microduck_cmd_publisher → peer-data → P4 cmd_bus
└────────────────────────────────────────────────────
```

## SDIO + peer-data（默认）

| 方向 | 机制 | 载荷 |
|------|------|------|
| C5 → P4 遥控 | `esp_hosted_send_custom_data(MICRODUCK_MSG_ID_CMD, frame, sizeof(frame))` | `microduck_link_frame_t` |
| P4 接收 | `esp_hosted_register_custom_callback` → `cmd_bus_set` → obs[48:61] | 同上 |

- P4：`components/cmd_link/cmd_link_sdio.c`，`sdkconfig` 选 **SDIO**。
- C5：烧录 [esp-hosted-mcu/slave](https://github.com/espressif/esp-hosted-mcu)（`esp32c5`）+ `c5_wireless/esp_hosted_slave_patch/`。
- 引脚：P4/C5 两边 **menuconfig → ESP-Hosted → SDIO** 必须一致；板级可用 `CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD`。

UART 仅 `menuconfig` 选 **UART (bench)** 时用于无共板台架；**产品不用 UART 传命令**。

## 真机闭环（P4）

布局与上游 [`duck-control/src/obs.rs`](https://github.com/pollen-robotics/microduck/blob/main/duck-control/src/obs.rs) 一致（61D obs / 14D action，嘴关节不占策略槽位）。

```
50 Hz (core 1):
  BMI270（板载）或将来 imu_to_dxl@200 → obs[0:6]
  15× XL330 SyncRead（ID 20–24,10–14,30–34）→ policy 槽位 joint_rel/vel/last_action → obs[6:48]
  C5 SDIO cmd（body 仅 z,roll,pitch；x/y/yaw 强制 0）→ obs[48:61]
  INT8 policy → action[14]
  scatter → HOME + action×scale（head/leg 低通同 robotd 默认）→ SyncWrite
```

上游 `robotd` 在 **同一条 1 Mbps 总线** 上与 `imu_to_dxl` 一次读完；本固件暂用 P4 本地 BMI270，关节表见 `common/include/microduck_model.h`。

Protocol 2.0 @ 1 Mbps；连续读失败则自动 `torque off`。

## 固件

| 路径 | 目标 |
|------|------|
| `microduck_esp32/` | P4 host |
| `c5_wireless/esp_hosted_slave_patch/` + esp-hosted 从工程 | C5 SDIO slave |
| `microduck_rl/` | 仅 Linux 训练 |

## 依赖（P4）

`main/idf_component.yml`：`espressif/esp_hosted` ≥ 3.0（peer-data）、`esp_wifi_remote`（在 P4 上调 WiFi API，协议栈在 C5）。

首次构建：`idf.py set-target esp32p4` → `idf.py build` 会拉取组件。