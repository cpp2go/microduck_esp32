# ES8311 + NS4150B 播放与语音输入

硬件：Everest **ES8311** 编解码 + Nsiway **NS4150B** 功放（默认脚对齐 Waveshare ESP32-P4-Pico，均可 Kconfig）。

## 通路

```
播放: ESP I2S TX (DOUT) → ES8311 DAC → 模拟 → NS4150B (PA_CTRL) → 喇叭
录音: 麦克风 → ES8311 ADC → I2S ASDOUT → ESP I2S RX (DIN)
控制: ESP I2C → ES8311 寄存器；GPIO → NS4150B 使能
```

| 信号 | 默认 GPIO |
|------|-----------|
| I2C SDA / SCL | 7 / 8 |
| 地址 | `0x18` |
| MCLK / BCLK / WS | 13 / 12 / 10 |
| DOUT（ESP→SDIN） | 9 |
| DIN（ASDOUT→ESP） | 11 |
| PA_CTRL | 53 |

采样率 16 kHz，MCLK = 256×fs。默认 IMU SPI 脚（8–11）与音频重叠，同板请改脚。

## API

- `sound_play(tag)` — 合成 cue（robotd emotion 同名）
- `sound_record(mono, n, timeout_ms)` — 左声道 mono int16
- `sound_mic_level(window_ms)` — 峰值电平
- `sound_hw_ready()` / `sound_mic_ready()`

录音时会短暂关 PA / mute DAC，减轻喇叭串音。

## Shell 验证

```
sound list          # play=ready mic=ready
sound yes           # 听喇叭
sound level 200     # 对麦说话，看 peak 上升
sound record 500    # 打印 frames / peak / mean_abs
```

## 仍非原版

无 WAV 银行、theremin、wheee；本路径提供可测板端 I/O，便于后续接 PCM/ASR。
