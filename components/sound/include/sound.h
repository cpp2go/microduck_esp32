// SPDX-License-Identifier: MIT
//
// ES8311 + NS4150B voice I/O for microduck_esp32:
//   play:  I2S TX → ES8311 DAC → NS4150B → speaker
//   mic:   mic → ES8311 ADC → I2S RX → short PCM buffer / level meter
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool sound_init(void);

bool sound_play(const char *tag);
/// Play raw mono tone (Hz) for ms via I2S DAC; falls back to log-only.
bool sound_play_tone(float hz, int ms);
const char *const *sound_tags(int *count_out);
/// True if tag has an embedded PCM bank clip (robotd emotion WAV).
bool sound_bank_has(const char *tag);

bool sound_available(void);
/// True when ES8311 I2S TX (+ PA) path is live.
bool sound_hw_ready(void);
/// True when ES8311 ADC + I2S RX mic path is live.
bool sound_mic_ready(void);

/// Sample rate used by play/record (Hz).
int sound_sample_rate(void);

/**
 * Capture mono int16 PCM from the mic into `mono` (max_samples frames).
 * Internally reads stereo I2S and keeps the left channel.
 * Returns frames written, or <0 on error. Blocks up to timeout_ms.
 */
int sound_record(int16_t *mono, int max_samples, int timeout_ms);

/**
 * Peak absolute sample over a short live window (ms). 0..32767, or -1 on error.
 * Mutes the PA briefly to reduce speaker feedback while metering.
 */
int sound_mic_level(int window_ms);

#ifdef __cplusplus
}
#endif
