// SPDX-License-Identifier: MIT

#include "voice_control.h"
#include "voice_match.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sound.h"

static const char *kTag = "voice";

#if CONFIG_MICRODUCK_VOICE_ENABLE
static voice_intent_cb_t s_cb;
static void *s_cb_user;
static voice_intent_t s_last = VOICE_INTENT_NONE;
static TaskHandle_t s_task;
static volatile bool s_run;
static bool s_up;
static SemaphoreHandle_t s_mux;

static voice_intent_t from_match(voice_match_id_t m)
{
    switch (m) {
    case VOICE_MATCH_STOP:
        return VOICE_INTENT_STOP;
    case VOICE_MATCH_GO:
        return VOICE_INTENT_GO;
    case VOICE_MATCH_SIT:
        return VOICE_INTENT_SIT;
    case VOICE_MATCH_STAND:
        return VOICE_INTENT_STAND;
    case VOICE_MATCH_QUACK:
        return VOICE_INTENT_QUACK;
    default:
        return VOICE_INTENT_NONE;
    }
}

static void fire(voice_intent_t intent)
{
    if (intent == VOICE_INTENT_NONE) {
        return;
    }
    s_last = intent;
    ESP_LOGI(kTag, "intent=%s", voice_intent_name(intent));
    if (s_cb) {
        s_cb(intent, s_cb_user);
    }
}

static void listen_task(void *arg)
{
    (void)arg;
    const int sr = sound_sample_rate() > 0 ? sound_sample_rate() : 16000;
    const int win_ms = CONFIG_MICRODUCK_VOICE_WINDOW_MS;
    const int n = (sr * win_ms) / 1000;
    int16_t *buf = (int16_t *)malloc((size_t)n * sizeof(int16_t));
    if (!buf) {
        ESP_LOGE(kTag, "oom listen buffer");
        s_run = false;
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    const int vad_thresh = CONFIG_MICRODUCK_VOICE_VAD_PEAK;
    while (s_run) {
        if (!sound_mic_ready()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        /* Cheap peek: short level; if quiet, sleep. */
        const int peak = sound_mic_level(40);
        if (peak < vad_thresh) {
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        const int got = sound_record(buf, n, win_ms + 200);
        if (got < n / 2) {
            continue;
        }
        float feat[VOICE_FEAT_DIM];
        if (!voice_extract_feat(buf, got, feat)) {
            continue;
        }
        float score = 0.f;
        const voice_match_id_t mid = voice_match_feat(feat, &score);
        if (mid == VOICE_MATCH_NONE) {
            ESP_LOGD(kTag, "vad but no match (score=%.3f)", score);
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }
        fire(from_match(mid));
        /* Debounce after a hit. */
        vTaskDelay(pdMS_TO_TICKS(CONFIG_MICRODUCK_VOICE_DEBOUNCE_MS));
    }
    free(buf);
    s_task = NULL;
    vTaskDelete(NULL);
}
#endif

bool voice_init(void)
{
#if !CONFIG_MICRODUCK_VOICE_ENABLE
    ESP_LOGW(kTag, "voice control disabled");
    return false;
#else
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    s_up = true;
    ESP_LOGI(kTag,
             "voice intents ready (VAD+band templates; no on-device ASR) mic=%s",
             sound_mic_ready() ? "yes" : "later");
    return true;
#endif
}

bool voice_available(void)
{
#if CONFIG_MICRODUCK_VOICE_ENABLE
    return s_up;
#else
    return false;
#endif
}

bool voice_listen_start(void)
{
#if !CONFIG_MICRODUCK_VOICE_ENABLE
    return false;
#else
    if (!s_up) {
        return false;
    }
    if (s_run) {
        return true;
    }
    s_run = true;
    if (xTaskCreate(listen_task, "voice", 8192, NULL, 3, &s_task) != pdPASS) {
        s_run = false;
        s_task = NULL;
        return false;
    }
    return true;
#endif
}

void voice_listen_stop(void)
{
#if CONFIG_MICRODUCK_VOICE_ENABLE
    s_run = false;
    for (int i = 0; i < 50 && s_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
#endif
}

bool voice_listening(void)
{
#if CONFIG_MICRODUCK_VOICE_ENABLE
    return s_run;
#else
    return false;
#endif
}

void voice_set_callback(voice_intent_cb_t cb, void *user)
{
#if CONFIG_MICRODUCK_VOICE_ENABLE
    s_cb = cb;
    s_cb_user = user;
#else
    (void)cb;
    (void)user;
#endif
}

bool voice_inject(const char *name)
{
    const voice_intent_t intent = voice_intent_from_name(name);
    if (intent == VOICE_INTENT_NONE) {
        return false;
    }
#if CONFIG_MICRODUCK_VOICE_ENABLE
    fire(intent);
    return true;
#else
    (void)intent;
    return false;
#endif
}

const char *voice_intent_name(voice_intent_t intent)
{
    switch (intent) {
    case VOICE_INTENT_STOP:
        return "stop";
    case VOICE_INTENT_GO:
        return "go";
    case VOICE_INTENT_SIT:
        return "sit";
    case VOICE_INTENT_STAND:
        return "stand";
    case VOICE_INTENT_QUACK:
        return "quack";
    default:
        return "none";
    }
}

voice_intent_t voice_intent_from_name(const char *name)
{
    if (!name) {
        return VOICE_INTENT_NONE;
    }
    if (strcmp(name, "stop") == 0) {
        return VOICE_INTENT_STOP;
    }
    if (strcmp(name, "go") == 0) {
        return VOICE_INTENT_GO;
    }
    if (strcmp(name, "sit") == 0) {
        return VOICE_INTENT_SIT;
    }
    if (strcmp(name, "stand") == 0) {
        return VOICE_INTENT_STAND;
    }
    if (strcmp(name, "quack") == 0) {
        return VOICE_INTENT_QUACK;
    }
    return VOICE_INTENT_NONE;
}

voice_intent_t voice_last_intent(void)
{
#if CONFIG_MICRODUCK_VOICE_ENABLE
    return s_last;
#else
    return VOICE_INTENT_NONE;
#endif
}
