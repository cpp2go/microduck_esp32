# 视觉 / 声音移植说明（ESP32-P4）

对照原版 [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)。

## 声音（ES8311 + NS4150B）

| 原版 | 本仓 |
|------|------|
| `robotd/src/sound.rs` + `sounds/`：WAV 库、`aplay`、theremin、wheee | `components/sound`：同名 **tag** → 板端合成短音，经 **ES8311 DAC** 与 **NS4150B** 功放播出 |
| `robotd/emotions/*.wav` | 无 WAV 资产；tag 列表对齐 emotion 名 |
| ALSA 独占 PCM | I2S TX → ES8311；打断语义为单次 `sound_play` 覆盖 |

### 硬件通路

```
ESP32-P4  --I2C-->  ES8311 (控制/时钟)
         --I2S-->  ES8311 DAC  --模拟-->  NS4150B  --喇叭
         --GPIO--> NS4150B PA_CTRL（高电平使能）
```

默认引脚对齐 **Waveshare ESP32-P4-Pico** 板载编解码（均可在 menuconfig 改）：

| 信号 | 默认 GPIO |
|------|-----------|
| I2C SDA / SCL | 7 / 8 |
| ES8311 地址 | `0x18` |
| I2S MCLK / BCLK / WS / DOUT | 13 / 12 / 10 / 9 |
| NS4150B `PA_CTRL` | 53（`-1` = 不控，常开） |

**注意：** 本仓 IMU 默认 SPI 脚（8–11）与上述音频默认有重叠。同板并用时请在 menuconfig 把 IMU 或音频改到互不冲突的脚。

Shell：`sound list` · `sound yes`（`list` 会显示 `hw=ready|log-only`）

codec 探测/I2S 失败时自动降级为 log-only，不阻断启动。

**仍未移植：** WAV 银行、chorale/MIDI、theremin、wheee start/loop/end、personality 音色。

## 视觉（已落地最小等价）

| 原版 | 本仓 |
|------|------|
| `mediad`：V4L2/IMX219、pipeline、WebRTC、snapshot、laser | `components/vision`：**gray8 帧总线** + **synthetic** 帧 |
| `duck-detect` / `pet-detect`（ONNX/RKNN） | detect API **恒为 present=false** |
| Linux 相机栈 | CSI/MIPI **未接线**；`MICRODUCK_VISION_CSI_HOOK` 仅占位 |

Shell：`vision status` · `vision grab` · `vision detect`

**阻塞：** 无板级相机驱动与链路时无法得到真实画面；YOLO/RKNN 无法原样搬到 P4。

## 设计原则

- 推理仍在 P4；视觉/声音 **不** 进入 61D obs。
- 用可调用 API + Shell 做 bring-up。
