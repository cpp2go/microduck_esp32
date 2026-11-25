// SPDX-License-Identifier: MIT
//
// Host-testable energy-band keyword matcher (no FreeRTOS / no codec).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VOICE_BANDS 8
#define VOICE_FEAT_DIM VOICE_BANDS

typedef enum {
    VOICE_MATCH_NONE = 0,
    VOICE_MATCH_STOP,
    VOICE_MATCH_GO,
    VOICE_MATCH_SIT,
    VOICE_MATCH_STAND,
    VOICE_MATCH_QUACK,
} voice_match_id_t;

/// Fill `feat[VOICE_FEAT_DIM]` from mono int16 PCM (16 kHz). Returns false if too quiet.
bool voice_extract_feat(const int16_t *mono, int n, float feat[VOICE_FEAT_DIM]);

/// Nearest baked template; score in [0,1] (1=identical). Returns NONE if below threshold.
voice_match_id_t voice_match_feat(const float feat[VOICE_FEAT_DIM], float *score_out);

/// Build a synthetic utterance for a label (for host tests / inject calibration).
void voice_synth_utterance(voice_match_id_t id, int16_t *mono, int n, int sample_rate);

const char *voice_match_name(voice_match_id_t id);

#ifdef __cplusplus
}
#endif
