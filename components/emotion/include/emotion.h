// SPDX-License-Identifier: MIT
//
// Emotion keyframe player from pollen-robotics/microduck robotd/emotions/*.json.
// Overlays neck/head/body/mouth onto the live command / joint targets while a clip runs.
#pragma once

#include "emotion_data.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float neck;
    float head_pitch;
    float head_yaw;
    float head_roll;
    float body_pitch;
    float mouth;
    bool active;
} emotion_pose_t;

bool emotion_init(void);

/// Start named clip (same tags as sound / robotd emotions). Returns false if unknown.
bool emotion_play(const char *name);

void emotion_stop(void);
bool emotion_active(void);
const char *emotion_current_name(void);

/// Advance clock by dt_s; fills pose (zeros + active=false when idle).
void emotion_step(float dt_s, emotion_pose_t *out);

/// List baked clips.
const emotion_clip_t *emotion_clips(size_t *count_out);

#ifdef __cplusplus
}
#endif
