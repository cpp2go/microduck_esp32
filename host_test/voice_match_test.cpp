// SPDX-License-Identifier: MIT
// Host test: band-template voice matcher (no mic / no ESP-IDF).
#include <cstdio>

#include "voice_match.h"

static int g_fail;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL: %s\n", msg);                                     \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

int main()
{
    const int sr = 16000;
    const int n = sr / 2;
    int16_t buf[8000];
    const voice_match_id_t ids[] = {VOICE_MATCH_STOP, VOICE_MATCH_GO, VOICE_MATCH_SIT,
                                    VOICE_MATCH_STAND, VOICE_MATCH_QUACK};

    for (voice_match_id_t id : ids) {
        voice_synth_utterance(id, buf, n, sr);
        float feat[VOICE_FEAT_DIM];
        CHECK(voice_extract_feat(buf, n, feat), "extract");
        float score = 0.f;
        const voice_match_id_t got = voice_match_feat(feat, &score);
        if (got != id) {
            std::printf("FAIL: expected %s got %s score=%.3f\n", voice_match_name(id),
                        voice_match_name(got), score);
            ++g_fail;
        } else {
            std::printf("ok %s score=%.3f\n", voice_match_name(id), score);
        }
        CHECK(score >= 0.35f, "dominant-band fraction");
    }

    /* Silence must not match. */
    for (int i = 0; i < n; ++i) {
        buf[i] = 0;
    }
    float feat[VOICE_FEAT_DIM];
    CHECK(!voice_extract_feat(buf, n, feat), "silence rejected");

    std::printf("voice_match_test: %s (%d fails)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
