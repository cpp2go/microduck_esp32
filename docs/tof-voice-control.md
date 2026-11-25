# 距离（ToF）与语音控制

对照原版 [`pollen-robotics/microduck`](https://github.com/pollen-robotics/microduck) 的 **`tof`/`tofd`** + **`autod` 障碍门控**，以及本仓基于 **ES8311 麦** 的最小语音意图。

## 距离 / ToF

| | 原版 | 本仓 |
|--|--|--|
| 传感器 | VL53L8CX，`tofd` 刷 ULD 固件，15 Hz `tof.frame` | **8×8 帧总线**；默认 **synthetic / `tof inject`** |
| 避障 | `autod` `obstacle.rs`：STOP 0.15 m / REACT 0.45 m | **`tof_obstacle`** 同阈值；50 Hz 环内缩放 `obs[48:51]` twist |
| 完整 ULD | Linux I²C + ~90 KB 固件 | **未移植**（可选 I2C probe，测距仍用 inject） |

### Shell

```
tof status
tof inject near    # ~0.10 m → stop
tof inject wall    # ~0.30 m → slow
tof inject clear
tof grab
```

### 验证

1. `sh host_test/run.sh` — `tof_obstacle_test` 覆盖 clear/stop/slow。
2. 板上：`tof inject near` → `tof status` 见 `stop=1 scale=0`；开环行走时 twist 应被门控。

## 语音控制

原版 **没有 ASR 口令控机**；「The voice」是发声银行 + `pet-detect` / 环境声。本仓在 ES8311 上提供 **可用最小方案**：

| 方案 | 说明 |
|------|------|
| **板上最小（已实现）** | VAD → 录音 → **8 带 Goertzel，主峰频带 → 意图**（音叉式关键词：300/800/1200/2500/5000 Hz） |
| 意图 | `stop`(300) / `sit`(800) / `go`(1200) / `stand`(2500) / `quack`(5000) |
| 动作 | `cmd_bus` 清/设 twist；`policy use alpha_sitstand|alpha_stand`；`sound_play` |
| **未做** | 自然语言 ASR / Whisper（板上过重；用 `voice inject` 或主机 ASR 接同一回调） |

### Shell

```
voice test              # 无麦：合成模板自检
voice inject stop|go|sit|stand|quack
voice listen            # 开麦监听
voice listen off
voice status
```

真机可靠路径：`voice inject …`，或对麦播放上述频点短音；自然语言请主机 ASR → 同一 `OnVoiceIntent` 回调。

### 验证

1. `sh host_test/run.sh` — `voice_match_test`。
2. 板上：`voice test`；`voice inject go` 后观测 `cmd` / 喇叭 cue；`voice listen` + 近麦大声再试模板。

## 组件

- `components/tof/` — `tof.h`、`tof_obstacle.h`
- `components/voice/` — `voice_control.h`、`voice_match.h`
