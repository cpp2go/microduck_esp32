// SPDX-License-Identifier: MIT
//
// Minimal on-device voice intents over ES8311 mic.
// Full ASR (Whisper etc.) is too heavy for P4 beside INT8 policy — this uses
// VAD + short log-energy band templates + shell inject.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VOICE_INTENT_NONE = 0,
    VOICE_INTENT_STOP,
    VOICE_INTENT_GO,
    VOICE_INTENT_SIT,
    VOICE_INTENT_STAND,
    VOICE_INTENT_QUACK,
    VOICE_INTENT_COUNT
} voice_intent_t;

/// Called from the listen task (not ISR) when an intent fires.
typedef void (*voice_intent_cb_t)(voice_intent_t intent, void *user);

bool voice_init(void);
bool voice_available(void);

/// Start/stop background listen (mic → VAD → template match).
bool voice_listen_start(void);
void voice_listen_stop(void);
bool voice_listening(void);

void voice_set_callback(voice_intent_cb_t cb, void *user);

/// Inject an intent by name ("stop","go","sit","stand","quack") — shell / tests.
bool voice_inject(const char *name);

const char *voice_intent_name(voice_intent_t intent);
voice_intent_t voice_intent_from_name(const char *name);

/// Last fired intent (or NONE).
voice_intent_t voice_last_intent(void);

#ifdef __cplusplus
}
#endif
