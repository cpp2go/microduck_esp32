# 视觉 / 声音移植说明（ESP32-P4）

对照原版 [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)。**不**对照其它 ESP 移植。

## 声音（已落地最小等价）

| 原版 | 本仓 |
|------|------|
| `robotd/src/sound.rs` + `sounds/`：WAV 库、`aplay`、theremin、wheee 流式 | `components/sound`：按 **同名 tag** 播放短正弦（可选 I2S）或 log-only |
| `robotd/emotions/*.wav` | 无 WAV 资产；tag 列表对齐 emotion 名（yes/no/sad/…） |
| ALSA 独占 PCM、打断语义 | 单次 `sound_play`；有 I2S 时写一段缓冲，无硬件则只打日志 |

Shell：`sound list` · `sound yes`

menuconfig：`MICRODUCK_SOUND_*`；默认 **不** 开 I2S（避免未接线引脚空转）。接上 DAC/喇叭后打开 `MICRODUCK_SOUND_I2S_ENABLE` 并设 BCLK/WS/DOUT。

**阻塞 / 未移植：** WAV 银行生成、chorale/MIDI、theremin 合成流、wheee start/loop/end、personality 音色、扬声器 rolloff。

## 视觉（已落地最小等价）

| 原版 | 本仓 |
|------|------|
| `mediad`：V4L2/IMX219、pipeline、WebRTC、snapshot、laser | `components/vision`：**gray8 帧总线** + **synthetic** 帧（可抓取、均值亮度） |
| `duck-detect` / `pet-detect`（ONNX/RKNN） | `vision_detect_ball` / `vision_detect_pet` **恒为 present=false** |
| Linux 相机栈 | ESP32-P4 CSI/MIPI **未接线**；`MICRODUCK_VISION_CSI_HOOK` 仅编译占位 |

Shell：`vision status` · `vision grab` · `vision detect`

**阻塞：** 无板级相机驱动与链路时无法得到真实画面；YOLO/RKNN 权重与运行时也无法原样搬到 P4。后续若接 CSI，应填同一 `vision_grab` API，再考虑板端轻量检测或把检测放到主机。

## 设计原则

- 推理仍在 P4；视觉/声音 **不** 进入 61D obs（与原版主策略路径一致：球/宠物检测走 mediad/旁路意图）。
- 用可调用 API + Shell 做 bring-up，而不是假装已有 Linux 全栈。
