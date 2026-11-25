// SPDX-License-Identifier: MIT
//
// Minimal port of pollen-robotics/microduck robotd theremin:
// ToF closeness → mouth opening + continuous tone via sound path.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float mouth;       ///< 0..1
    float note_hz;     ///< 0 when silent
    float closeness;   ///< 0 far .. 1 near; -1 if no hand
    bool active;
} theremin_note_t;

bool theremin_init(void);
void theremin_set_active(bool up);
bool theremin_active(void);

/// Map nearest ahead distance (metres) into a note. ahead_m = +inf → silence.
theremin_note_t theremin_from_ahead_m(float ahead_m);

/// Play a short tone burst at note_hz (uses sound PCM path / synth).
bool theremin_beep(float note_hz, int ms);

#ifdef __cplusplus
}
#endif
