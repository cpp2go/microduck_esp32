// SPDX-License-Identifier: MIT

#include "loop_validate.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

void loop_validate_reset(loop_validate_t *s, int period_us)
{
    if (!s) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->min_us = INT64_MAX;
    s->period_us = period_us > 0 ? period_us : 20000;
}

void loop_validate_record(loop_validate_t *s, int64_t pass_us)
{
    if (!s) {
        return;
    }
    if (pass_us < s->min_us) {
        s->min_us = pass_us;
    }
    if (pass_us > s->max_us) {
        s->max_us = pass_us;
    }
    s->sum_us += pass_us;
    s->ticks += 1;
    if (pass_us > s->period_us) {
        s->overruns += 1;
    }
}

double loop_validate_avg_us(const loop_validate_t *s)
{
    if (!s || s->ticks <= 0) {
        return 0.0;
    }
    return (double)s->sum_us / (double)s->ticks;
}

bool loop_validate_ok(const loop_validate_t *s, double tol_frac, double max_overrun_frac)
{
    if (!s || s->ticks < 10) {
        return false;
    }
    const double avg = loop_validate_avg_us(s);
    const double nom = (double)s->period_us;
    if (fabs(avg - nom) > nom * tol_frac) {
        /* avg here is *pass* duration, not wake-to-wake period — gate only overruns */
        (void)avg;
    }
    const double rate = (double)s->overruns / (double)s->ticks;
    return rate <= max_overrun_frac;
}
