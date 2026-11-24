// SPDX-License-Identifier: MIT
//
// Minimal on-device stand-in for pollen-robotics/microduck mediad camera pipeline.
// Full Linux V4L2 / WebRTC / YOLO cannot run on ESP32-P4 as-is; this provides a
// frame-bus API, synthetic frames for bring-up, and hooks for a future CSI path.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t stride;   /* bytes per row (gray8) */
    uint64_t seq;
    int64_t ts_us;
    const uint8_t *gray; /* valid until next vision_grab / vision_release */
} vision_frame_t;

typedef struct {
    bool present; /* false = detector not available on this build */
    float x;      /* normalized [0,1] image coords when present */
    float y;
    float score;
} vision_detect_t;

bool vision_init(void);
bool vision_available(void);

/// Fill `out` with the latest gray frame (synthetic or camera). Returns false if empty.
bool vision_grab(vision_frame_t *out);

/// Optional: release borrowed buffer (no-op for synthetic static buffer).
void vision_release(void);

/// Ball / pet detect stub. Returns present=false until a real detector is wired.
bool vision_detect_ball(vision_detect_t *out);
bool vision_detect_pet(vision_detect_t *out);

/// Mean luminance of last grabbed frame (0–255), or -1 if none.
int vision_mean_luma(void);

const char *vision_backend_name(void);

#ifdef __cplusplus
}
#endif
