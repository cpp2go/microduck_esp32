// SPDX-License-Identifier: MIT

#include "theremin.h"

#include <math.h>

#include "esp_log.h"
#include "sound.h"

static const char *kTag = "theremin";

/* robotd sound theremin band: near ~0.05 m → high, far ~0.35 m → low */
static const float kNearM = 0.05f;
static const float kFarM = 0.35f;
static const float kHzLo = 220.f;
static const float kHzHi = 880.f;

static bool s_active;
static bool s_up;

bool theremin_init(void)
{
    s_active = false;
    s_up = true;
    ESP_LOGI(kTag, "theremin ready (ToF closeness → Hz/mouth)");
    return true;
}

void theremin_set_active(bool up) { s_active = up; }

bool theremin_active(void) { return s_active; }

theremin_note_t theremin_from_ahead_m(float ahead_m)
{
    theremin_note_t n = {0.f, 0.f, -1.f, s_active};
    if (!s_active || !isfinite(ahead_m) || ahead_m >= kFarM) {
        n.mouth = 0.f;
        return n;
    }
    float c = (kFarM - ahead_m) / (kFarM - kNearM);
    if (c < 0.f) {
        c = 0.f;
    }
    if (c > 1.f) {
        c = 1.f;
    }
    n.closeness = c;
    n.mouth = c;
    n.note_hz = kHzLo + (kHzHi - kHzLo) * c;
    return n;
}

bool theremin_beep(float note_hz, int ms)
{
    if (!s_up || note_hz <= 0.f || ms <= 0) {
        return false;
    }
    return sound_play_tone(note_hz, ms);
}
