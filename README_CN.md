# microduck_esp32（P4 控制 + C5 无线，SDIO）

| 位置 | 职责 |
|------|------|
| Linux `microduck_rl` | 训练、ONNX、bake |
| **ESP32-P4** | 50 Hz 策略、IMU、动作 |
| **ESP32-C5** | esp_hosted **SDIO 从机**、WiFi、遥控 → P4 |

**P4↔C5 默认 SDIO**（esp_hosted peer-data），不用 UART。说明见 [docs/architecture.md](docs/architecture.md)、[c5_wireless/README.md](c5_wireless/README.md)。

## P4 固件

```bash
cd microduck_esp32
idf.py set-target esp32p4
idf.py menuconfig   # ESP-Hosted SDIO 引脚 + 开发板预设
idf.py build flash monitor
```

## C5 从机固件

按 `c5_wireless/README.md` 编译 **esp-hosted-mcu/slave**（`esp32c5`），并入 `esp_hosted_slave_patch/microduck_cmd_publisher.c`。

## 真机闭环（舵机 + 观测）

50 Hz 环顺序：**SyncRead 关节 → 组 61D 观测 → INT8 策略 → SyncWrite Goal Position**。

| 配置 | menuconfig → Microduck Dynamixel bus |
|------|--------------------------------------|
| UART 引脚 / 1 Mbps | `SERVO_TX/RX_GPIO` |
| 电机 ID | alpha 表 `20–24,10–14,30–34`（见 `common/include/microduck_model.h`） |
| 动作 | `scatter` → `HOME + action × scale`（默认 scale=0.9，head/leg 低通同 robotd） |

Shell：`servo stats` · `servo off`（急停断扭矩）· `servo on`。

**上电先** `policy test` **并在支架上确认** `servo off` 可用，再让脚着地。

## 调试

- `policy loop` / `policy stats` — P4 侧 50 Hz  
- P4 日志 `cmd_sdio: SDIO cmd link ready` 表示 C5 已连通  
- 仅台架可改 `menuconfig` → Command link → **UART**