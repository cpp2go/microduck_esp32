// SPDX-License-Identifier: MIT
//
// On-device / host helpers for validating the 50 Hz control period
// (pollen-robotics/microduck robotd tick budget).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t ticks;
    int64_t overruns; ///< passes that alone exceeded period_us
    int64_t min_us;
    int64_t max_us;
    int64_t sum_us;
    int period_us; ///< nominal period (20000 for 50 Hz)
} loop_validate_t;

void loop_validate_reset(loop_validate_t *s, int period_us);

/// Record one pass duration (µs).
void loop_validate_record(loop_validate_t *s, int64_t pass_us);

/// Average period in µs, or 0 if idle.
double loop_validate_avg_us(const loop_validate_t *s);

/// True when avg is within ±tol_frac of period and overrun rate ≤ max_overrun_frac.
bool loop_validate_ok(const loop_validate_t *s, double tol_frac, double max_overrun_frac);

#ifdef __cplusplus
}
#endif
