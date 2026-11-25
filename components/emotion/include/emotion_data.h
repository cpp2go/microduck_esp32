// SPDX-License-Identifier: MIT
// Auto-generated from pollen-robotics/microduck robotd/emotions/*.json — do not edit.
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float t;
    float neck;
    float head_pitch;
    float head_yaw;
    float head_roll;
    float body_pitch;
    float mouth;
} emotion_keyframe_t;
typedef struct {
    const char *name;
    const emotion_keyframe_t *keys;
    size_t count;
} emotion_clip_t;
extern const emotion_clip_t kEmotionClips[];
extern const size_t kEmotionClipCount;
const emotion_clip_t *emotion_find(const char *name);
#ifdef __cplusplus
}
#endif
