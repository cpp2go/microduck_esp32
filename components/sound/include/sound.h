// SPDX-License-Identifier: MIT
//
// On-device voice cues aligned with pollen-robotics/microduck robotd tags.
// Hardware path: ES8311 (I2C + I2S DAC) → NS4150B (PA_CTRL GPIO) → speaker.
// Without HW / on probe failure, play() still logs the tag.
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

/// True when ES8311 + I2S path is live (not log-only).
bool sound_hw_ready(void);

#ifdef __cplusplus
}
#endif
