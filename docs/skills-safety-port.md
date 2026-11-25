# Skills / safety / emotion port

Faithful ESP32-P4 ports from [`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck):

| Module | Original | This repo |
|--------|----------|-----------|
| Net scheduler | `robotd/src/control.rs` | `components/skills` |
| Fall / clamp | `duck-control` fall.rs + safety.rs | `components/safety` |
| Emotion clips | `robotd/emotions/*.json` + WAV | `components/emotion` + sound PCM bank |
| Theremin | `robotd/src/theremin.rs` | `components/theremin` |
| imu@200 | `imu_to_dxl` on DXL bus | `imu::StartFastTask` @200 Hz SPI |
| VL53 ULD | `tof/vendor/vl53l8cx` | same + ESP-IDF I2C platform (`MICRODUCK_TOF_ULD`) |

Shell: `skill sit` · `emotion play yes` · `imu fast` · `validate` · `theremin beep`.

Re-bake assets (optional):

```bash
python3 tools/bake_emotions.py --src /path/to/microduck/robotd/emotions \
  --out-h components/emotion/include/emotion_data.h \
  --out-c components/emotion/src/emotion_data.c
python3 tools/bake_sound_bank.py --src /path/to/microduck/robotd/emotions \
  --out-h components/sound/include/sound_bank.h \
  --out-c components/sound/sound_bank_data.c
```
