// SPDX-License-Identifier: MIT
// PCM sound bank from pollen-robotics/microduck robotd/emotions/*.wav (16 kHz mono s16).
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    const char *tag;
    const int16_t *pcm;
    size_t samples; // mono frames @ 16 kHz
} sound_bank_clip_t;
extern const sound_bank_clip_t kSoundBank[];
extern const size_t kSoundBankCount;
const sound_bank_clip_t *sound_bank_find(const char *tag);
#ifdef __cplusplus
}
#endif
