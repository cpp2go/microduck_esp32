// SPDX-License-Identifier: MIT
//
// Port of pollen-robotics/microduck duck-control fall.rs + safety.rs (math only):
//   - FallPredictor (early tip detection from gyro × gravity)
//   - fallen verdict (gravity z debounce)
//   - non-finite reject + actuator range clamp on joint targets
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAFETY_ACTUATOR_MIN (-3.14159265f)
#define SAFETY_ACTUATOR_MAX (3.14159265f)

typedef struct {
    float tilt_z;       ///< already tilted past this (default -0.90)
    float predicted_z;  ///< extrapolated past this (default -0.5)
    float lookahead_s;  ///< default 0.30
    float debounce_s;   ///< default 0.060
    float arm_s;        ///< must stand this long before armed (default 0.200)
} fall_predictor_config_t;

typedef struct {
    fall_predictor_config_t cfg;
    float falling_for_s;
    float upright_for_s;
    bool armed;
} fall_predictor_t;

typedef struct {
    float fall_gravity_z; ///< default -0.5
    float fall_debounce_s; ///< default 0.200
    float deadman_s;       ///< default 0.500 (reported; twist zero is caller's job)
} safety_config_t;

typedef struct {
    safety_config_t cfg;
    float falling_for_s;
    bool fallen;
} safety_state_t;

void fall_predictor_init(fall_predictor_t *p, const fall_predictor_config_t *cfg_or_null);
void fall_predictor_reset(fall_predictor_t *p);

/// ġz = −(ωx·gy − ωy·gx)
float fall_gravity_z_rate(const float gyro[3], const float gravity[3]);

/// Edge-triggered: true once per fall when debounce completes.
bool fall_predictor_observe(fall_predictor_t *p, const float gyro[3], const float gravity[3],
                            float dt_s);

void safety_init(safety_state_t *s, const safety_config_t *cfg_or_null);

/// Update fallen verdict from projected gravity z. Returns current fallen flag.
bool safety_observe_gravity(safety_state_t *s, float gravity_z, float dt_s);

bool safety_fallen(const safety_state_t *s);

/// Reject NaN/Inf (zero that joint) and clamp to ±π. Returns number of joints limited.
int safety_clamp_targets(float *targets, int n);

#ifdef __cplusplus
}
#endif
