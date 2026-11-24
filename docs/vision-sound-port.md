# 视觉 / 声音移植说明（ESP32-P4）

对照原版 [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)。

## 声音（ES8311 + NS4150B：播放 + 麦克风）

详见 **[es8311-audio.md](es8311-audio.md)**。

| 原版 | 本仓 |
|------|------|
| `robotd` WAV + `aplay` | tag → **ES8311 DAC** + **NS4150B** 合成短音 |
| （原版 mic 走 mediad/其它） | **ES8311 ADC** → I2S RX：`sound_record` / `sound_mic_level` |
| ALSA PCM | I2S 双工（TX DOUT + RX DIN） |

Shell：`sound list` · `sound yes` · `sound level 200` · `sound record 500`

**仍未移植：** WAV 银行、chorale/MIDI、theremin、wheee、personality 音色。

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
