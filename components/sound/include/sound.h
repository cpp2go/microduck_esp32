// SPDX-License-Identifier: MIT
//
// Minimal on-device voice equivalent of pollen-robotics/microduck robotd/src/sound.rs:
// play a named tag (emotion / cue). Original uses WAV bank + aplay; this board
// synthesizes short I2S tones (or log-only when disabled / no codec).
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool sound_init(void);

/// Play cue by tag (e.g. "yes", "no", "sad"). Non-blocking; cuts previous tone.
bool sound_play(const char *tag);

/// List known tags (null-terminated array of string pointers).
const char *const *sound_tags(int *count_out);

bool sound_available(void);

#ifdef __cplusplus
}
#endif
