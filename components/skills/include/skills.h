// SPDX-License-Identifier: MIT
//
// robotd-like skill / net scheduler for ESP32-P4.
// Mirrors pollen-robotics/microduck robotd/src/control.rs priority:
//   skill > ground_pick > sit/rise > stand-by-magnitude > walk
// Switches among baked duck_policy nets (alpha_walking / alpha_stand /
// alpha_sitstand / alpha_ground_pick). Configurable one-shot skills that need
// extra baked nets stay as named windows that pin twist + standing tuning.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SKILL_NET_WALK = 0,
    SKILL_NET_STAND,
    SKILL_NET_SITSTAND,
    SKILL_NET_GROUND_PICK,
    SKILL_NET_SKILL, ///< named one-shot; still uses standing/walk bake
} skill_net_t;

typedef enum {
    SKILL_SIT_UP = 0,
    SKILL_SIT_SITTING,
    SKILL_SIT_RISING,
} skill_sit_t;

typedef struct {
    float twist[3];
    bool body_active;
    float twist_stand_thresh; ///< |twist| below → stand (default 0.05)
} skill_command_t;

typedef struct {
    skill_net_t net;
    const char *label; ///< "walk" / "stand" / "sit" / "rise" / "ground_pick" / skill name
    float effective_twist[3];
    float action_scale;
    float gain_ratio;
    bool busy; ///< true while a one-shot / sit / pick is in flight
} skill_step_t;

bool skills_init(void);

/// Request sit toggle (sit if up, rise if seated) — robotd sit_toggle.
void skills_request_sit_toggle(void);

/// Request ground-pick phase script (uses alpha_ground_pick while active).
void skills_request_ground_pick(void);

/// Request a named one-shot window. Built-ins: kick, kick_right, roulade, roller
/// (mapped to baked ball_kick_* / roulade / roller nets).
bool skills_request_oneshot(const char *name);

/// Force walk / stand / sit / rise without waiting for twist magnitude.
bool skills_force(const char *mode);

skill_sit_t skills_sit_state(void);
const char *skills_driving_label(void);

/// Advance timers and pick the net for this 50 Hz tick. `dt_s` ≈ 0.02.
void skills_step(const skill_command_t *cmd, float dt_s, skill_step_t *out);

/// Apply the selected net: SetModel to the matching baked policy if present.
/// Returns false if the requested bake is missing (caller keeps current).
bool skills_apply_net(const skill_step_t *step);

#ifdef __cplusplus
}
#endif
