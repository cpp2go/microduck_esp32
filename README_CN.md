# microduck_esp32（P4 控制 + C5 无线，SDIO）

| 位置 | 职责 |
|------|------|
| Linux `microduck_rl` | 训练、ONNX 导出、bake（主机工具） |
| **ESP32-P4** | **板端** 50 Hz INT8 策略、ICM-42688 SPI、飞特 1910 闭环 |
| **ESP32-C5** | esp_hosted **SDIO 从机**、WiFi、遥控 → P4 |

对齐基准是原版 [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) 的观测/控制契约；本板硬件为 **ICM-42688（SPI）+ 飞特 1910（STS）**，不是 XL330 / BMI270。说明见 [docs/architecture.md](docs/architecture.md)。

**P4↔C5 默认 SDIO**（esp_hosted peer-data）。

## P4 固件

```bash
idf.py set-target esp32p4
idf.py menuconfig   # ICM-42688 SPI 引脚 + Feetech UART
idf.py build flash monitor
```

## 真机闭环（舵机 + 观测）

50 Hz：**SyncRead 飞特关节 → 组 61D 观测 → INT8 策略 → SyncWrite Goal Position**。

| 配置 | menuconfig |
|------|------------|
| IMU | ICM-42688 SPI（SCLK/MOSI/MISO/CS） |
| 舵机 | Feetech UART / 1 Mbps；ID 表见 `microduck_model.h` |
| 动作 | `scatter` → `HOME + action × scale`（默认 0.9） |

Shell：`servo stats` · `servo off` · `servo on` · `imu` · `sound list|yes` · `vision status|grab|detect`。

声音/视觉为原版最小等价（非 Linux WAV/mediad 全栈），说明见 [docs/vision-sound-port.md](docs/vision-sound-port.md)；缺口盘点见 [docs/gap-audit-vs-original.md](docs/gap-audit-vs-original.md)。

**上电先** `policy test` **并在支架上确认** `servo off` 可用，再着地。
