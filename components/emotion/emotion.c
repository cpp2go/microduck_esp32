// SPDX-License-Identifier: MIT

#include "emotion.h"

#include <string.h>

#include "esp_log.h"

static const char *kTag = "emotion";

static const emotion_clip_t *s_clip;
static float s_t;
static bool s_up;

bool emotion_init(void)
{
    s_clip = NULL;
    s_t = 0.f;
    s_up = true;
    ESP_LOGI(kTag, "emotion ready (%u clips from robotd/emotions)", (unsigned)kEmotionClipCount);
    return true;
}

bool emotion_play(const char *name)
{
    if (!s_up || !name) {
        return false;
    }
    const emotion_clip_t *c = emotion_find(name);
    if (!c) {
        return false;
    }
    s_clip = c;
    s_t = 0.f;
    ESP_LOGI(kTag, "play emotion '%s' (%u keys)", name, (unsigned)c->count);
    return true;
}

void emotion_stop(void)
{
    s_clip = NULL;
    s_t = 0.f;
}

bool emotion_active(void) { return s_clip != NULL; }

const char *emotion_current_name(void) { return s_clip ? s_clip->name : NULL; }

const emotion_clip_t *emotion_clips(size_t *count_out)
{
    if (count_out) {
        *count_out = kEmotionClipCount;
    }
    return kEmotionClips;
}

static void lerp_keys(const emotion_keyframe_t *a, const emotion_keyframe_t *b, float u,
                      emotion_pose_t *out)
{
    const float t = u < 0.f ? 0.f : (u > 1.f ? 1.f : u);
    out->neck = a->neck + (b->neck - a->neck) * t;
    out->head_pitch = a->head_pitch + (b->head_pitch - a->head_pitch) * t;
    out->head_yaw = a->head_yaw + (b->head_yaw - a->head_yaw) * t;
    out->head_roll = a->head_roll + (b->head_roll - a->head_roll) * t;
    out->body_pitch = a->body_pitch + (b->body_pitch - a->body_pitch) * t;
    out->mouth = a->mouth + (b->mouth - a->mouth) * t;
    out->active = true;
}

void emotion_step(float dt_s, emotion_pose_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (!s_clip || s_clip->count == 0) {
        return;
    }
    if (dt_s > 0.f) {
        s_t += dt_s;
    }
    const emotion_keyframe_t *keys = s_clip->keys;
    const size_t n = s_clip->count;
    const float t_end = keys[n - 1].t;
    if (s_t >= t_end) {
        out->neck = keys[n - 1].neck;
        out->head_pitch = keys[n - 1].head_pitch;
        out->head_yaw = keys[n - 1].head_yaw;
        out->head_roll = keys[n - 1].head_roll;
        out->body_pitch = keys[n - 1].body_pitch;
        out->mouth = keys[n - 1].mouth;
        out->active = false;
        s_clip = NULL;
        s_t = 0.f;
        return;
    }
    if (s_t <= keys[0].t) {
        lerp_keys(&keys[0], &keys[0], 0.f, out);
        return;
    }
    for (size_t i = 0; i + 1 < n; ++i) {
        if (s_t >= keys[i].t && s_t <= keys[i + 1].t) {
            const float span = keys[i + 1].t - keys[i].t;
            const float u = span > 1e-6f ? (s_t - keys[i].t) / span : 0.f;
            lerp_keys(&keys[i], &keys[i + 1], u, out);
            return;
        }
    }
}
