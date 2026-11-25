// SPDX-License-Identifier: MIT

#include "voice_match.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float kBandHz[VOICE_BANDS] = {300.f, 500.f, 800.f, 1200.f, 1800.f, 2500.f, 3500.f, 5000.f};

/* Dominant band index → intent (tone keywords / synth). */
static const voice_match_id_t kBandIntent[VOICE_BANDS] = {
    VOICE_MATCH_STOP,  /* 300 */
    VOICE_MATCH_NONE,  /* 500 unused */
    VOICE_MATCH_SIT,   /* 800 */
    VOICE_MATCH_GO,    /* 1200 */
    VOICE_MATCH_NONE,  /* 1800 */
    VOICE_MATCH_STAND, /* 2500 */
    VOICE_MATCH_NONE,  /* 3500 */
    VOICE_MATCH_QUACK, /* 5000 */
};

#define kPeakRatio 1.8f

static float goertzel_power(const int16_t *x, int n, float freq, float fs)
{
    const float w = 2.f * (float)M_PI * freq / fs;
    const float coeff = 2.f * cosf(w);
    float s0 = 0.f, s1 = 0.f, s2 = 0.f;
    for (int i = 0; i < n; ++i) {
        s0 = (float)x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    const float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    return power > 0.f ? power : 0.f;
}

bool voice_extract_feat(const int16_t *mono, int n, float feat[VOICE_FEAT_DIM])
{
    if (!mono || n < 160 || !feat) {
        return false;
    }
    long long energy = 0;
    for (int i = 0; i < n; ++i) {
        int v = mono[i];
        energy += (long long)v * v;
    }
    const float rms = sqrtf((float)energy / (float)n);
    if (rms < 200.f) {
        return false;
    }
    const float fs = 16000.f;
    float sum = 0.f;
    for (int b = 0; b < VOICE_BANDS; ++b) {
        feat[b] = goertzel_power(mono, n, kBandHz[b], fs);
        sum += feat[b];
    }
    if (sum < 1.f) {
        return false;
    }
    for (int b = 0; b < VOICE_BANDS; ++b) {
        feat[b] /= sum;
    }
    return true;
}

voice_match_id_t voice_match_feat(const float feat[VOICE_FEAT_DIM], float *score_out)
{
    if (!feat) {
        if (score_out) {
            *score_out = 0.f;
        }
        return VOICE_MATCH_NONE;
    }
    int best_i = 0;
    int second_i = 1;
    for (int b = 1; b < VOICE_BANDS; ++b) {
        if (feat[b] > feat[best_i]) {
            second_i = best_i;
            best_i = b;
        } else if (feat[b] > feat[second_i]) {
            second_i = b;
        }
    }
    const float ratio = feat[second_i] > 1e-9f ? (feat[best_i] / feat[second_i]) : 100.f;
    const float score = feat[best_i]; /* fraction of total band power */
    if (score_out) {
        *score_out = score;
    }
    if (ratio < kPeakRatio) {
        return VOICE_MATCH_NONE;
    }
    return kBandIntent[best_i];
}

void voice_synth_utterance(voice_match_id_t id, int16_t *mono, int n, int sample_rate)
{
    if (!mono || n <= 0 || sample_rate <= 0) {
        return;
    }
    memset(mono, 0, (size_t)n * sizeof(int16_t));
    int band = -1;
    for (int b = 0; b < VOICE_BANDS; ++b) {
        if (kBandIntent[b] == id) {
            band = b;
            break;
        }
    }
    if (band < 0) {
        return;
    }
    const float freq = kBandHz[band];
    for (int i = 0; i < n; ++i) {
        const float t = (float)i / (float)sample_rate;
        float env = 1.f;
        if (i < n / 10) {
            env = (float)i / (float)(n / 10);
        } else if (i > n * 9 / 10) {
            env = (float)(n - i) / (float)(n / 10);
        }
        const float sample = env * 12000.f * sinf(2.f * (float)M_PI * freq * t);
        if (sample > 32767.f) {
            mono[i] = 32767;
        } else if (sample < -32768.f) {
            mono[i] = -32768;
        } else {
            mono[i] = (int16_t)sample;
        }
    }
}

const char *voice_match_name(voice_match_id_t id)
{
    switch (id) {
    case VOICE_MATCH_STOP:
        return "stop";
    case VOICE_MATCH_GO:
        return "go";
    case VOICE_MATCH_SIT:
        return "sit";
    case VOICE_MATCH_STAND:
        return "stand";
    case VOICE_MATCH_QUACK:
        return "quack";
    default:
        return "none";
    }
}
