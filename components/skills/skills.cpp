// SPDX-License-Identifier: MIT

#include "skills.h"

#include <math.h>
#include <string.h>

#include "duck_policy.hpp"
#include "esp_log.h"

static const char *kTag = "skills";

enum { kMaxOneshots = 4 };

typedef struct {
    const char *name;
    float duration_s;
    float command[3];
    bool chain;
    const char *bake; ///< baked duck_policy name, or NULL → alpha_stand
} oneshot_def_t;

static const oneshot_def_t kOneshots[kMaxOneshots] = {
    {"kick", 0.8f, {0.f, 0.f, 0.f}, false, "ball_kick_left"},
    {"kick_right", 0.8f, {0.f, 0.f, 0.f}, false, "ball_kick_right"},
    {"roulade", 1.2f, {0.f, 0.f, 0.f}, false, "roulade"},
    {"roller", 1.2f, {0.f, 0.f, 0.f}, false, "roller"},
};

static const float kSitstandRiseS = 1.0f;
static const float kSitstandSettleS = 1.5f;
static const float kGroundPickEnd = 1.0f; /* phase 0..1 over this many seconds */
static const float kStandThresh = 0.05f;
static const float kStandingScale = 1.0f;
static const float kStandingGainRatio = 0.8f;
static const float kWalkScale = 0.5f; /* matches typical duck-control action_scale */

static skill_sit_t s_sit = SKILL_SIT_UP;
static float s_sit_remain = 0.f;
static float s_sit_settle = 0.f;
static bool s_auto_rise = false;

static int s_active_oneshot = -1; /* index into kOneshots, or -1 */
static float s_oneshot_remain = 0.f;

static bool s_ground_pick = false;
static float s_ground_phase = 0.f;

static const char *s_label = "walk";
static bool s_up;

bool skills_init(void)
{
    s_sit = SKILL_SIT_UP;
    s_sit_remain = 0.f;
    s_sit_settle = 0.f;
    s_auto_rise = false;
    s_active_oneshot = -1;
    s_oneshot_remain = 0.f;
    s_ground_pick = false;
    s_ground_phase = 0.f;
    s_label = "walk";
    s_up = true;
    ESP_LOGI(kTag, "skills ready (walk/stand/sit/rise/ground_pick/kick/roulade/roller)");
    return true;
}

void skills_request_sit_toggle(void)
{
    if (!s_up) {
        return;
    }
    if (s_sit == SKILL_SIT_UP) {
        s_sit = SKILL_SIT_SITTING;
        s_sit_settle = kSitstandSettleS;
        s_auto_rise = false;
        ESP_LOGI(kTag, "sit requested");
    } else if (s_sit == SKILL_SIT_SITTING) {
        s_sit = SKILL_SIT_RISING;
        s_sit_remain = kSitstandRiseS;
        s_sit_settle = 0.f;
        ESP_LOGI(kTag, "rise requested");
    }
}

void skills_request_ground_pick(void)
{
    if (!s_up || s_sit != SKILL_SIT_UP || s_active_oneshot >= 0) {
        return;
    }
    s_ground_pick = true;
    s_ground_phase = 0.f;
    ESP_LOGI(kTag, "ground_pick requested");
}

bool skills_request_oneshot(const char *name)
{
    if (!s_up || !name || s_sit != SKILL_SIT_UP) {
        return false;
    }
    for (int i = 0; i < kMaxOneshots; ++i) {
        if (kOneshots[i].name && strcmp(kOneshots[i].name, name) == 0) {
            if (s_active_oneshot >= 0 || s_ground_pick) {
                return false; /* refuse preempt like robotd */
            }
            s_active_oneshot = i;
            s_oneshot_remain = kOneshots[i].duration_s;
            ESP_LOGI(kTag, "oneshot '%s' for %.2fs", name, (double)s_oneshot_remain);
            return true;
        }
    }
    return false;
}

bool skills_force(const char *mode)
{
    if (!mode) {
        return false;
    }
    if (strcmp(mode, "walk") == 0) {
        s_sit = SKILL_SIT_UP;
        s_active_oneshot = -1;
        s_ground_pick = false;
        s_label = "walk";
        return true;
    }
    if (strcmp(mode, "stand") == 0) {
        s_sit = SKILL_SIT_UP;
        s_active_oneshot = -1;
        s_ground_pick = false;
        s_label = "stand";
        return true;
    }
    if (strcmp(mode, "sit") == 0) {
        s_sit = SKILL_SIT_SITTING;
        s_sit_settle = kSitstandSettleS;
        s_active_oneshot = -1;
        s_ground_pick = false;
        return true;
    }
    if (strcmp(mode, "rise") == 0) {
        s_sit = SKILL_SIT_RISING;
        s_sit_remain = kSitstandRiseS;
        s_active_oneshot = -1;
        s_ground_pick = false;
        return true;
    }
    if (strcmp(mode, "ground_pick") == 0) {
        skills_request_ground_pick();
        return s_ground_pick;
    }
    return skills_request_oneshot(mode);
}

skill_sit_t skills_sit_state(void) { return s_sit; }

const char *skills_driving_label(void) { return s_label ? s_label : "walk"; }

static float twist_mag(const float t[3])
{
    return sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
}

void skills_step(const skill_command_t *cmd, float dt_s, skill_step_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->action_scale = kWalkScale;
    out->gain_ratio = 1.f;
    out->effective_twist[0] = cmd ? cmd->twist[0] : 0.f;
    out->effective_twist[1] = cmd ? cmd->twist[1] : 0.f;
    out->effective_twist[2] = cmd ? cmd->twist[2] : 0.f;

    if (dt_s < 0.f) {
        dt_s = 0.f;
    }

    /* Expire windows first — same order as robotd Controller::step. */
    if (s_active_oneshot >= 0) {
        s_oneshot_remain -= dt_s;
        if (s_oneshot_remain <= 0.f) {
            s_active_oneshot = -1;
            s_oneshot_remain = 0.f;
        }
    }
    if (s_sit == SKILL_SIT_RISING) {
        s_sit_remain -= dt_s;
        if (s_sit_remain <= 0.f) {
            s_sit = SKILL_SIT_UP;
            s_sit_remain = 0.f;
        }
    }
    if (s_sit == SKILL_SIT_SITTING && s_sit_settle > 0.f) {
        s_sit_settle -= dt_s;
        if (s_sit_settle < 0.f) {
            s_sit_settle = 0.f;
        }
        if (s_auto_rise && s_sit_settle <= 0.f) {
            s_auto_rise = false;
            s_sit = SKILL_SIT_RISING;
            s_sit_remain = kSitstandRiseS;
        }
    }
    if (s_ground_pick) {
        s_ground_phase += dt_s / kGroundPickEnd;
        if (s_ground_phase >= 1.f) {
            s_ground_pick = false;
            s_ground_phase = 0.f;
        }
    }

    const float thresh = (cmd && cmd->twist_stand_thresh > 0.f) ? cmd->twist_stand_thresh : kStandThresh;
    const bool body_active = cmd && cmd->body_active;

    if (s_active_oneshot >= 0) {
        const oneshot_def_t *d = &kOneshots[s_active_oneshot];
        out->net = SKILL_NET_SKILL;
        out->label = d->name;
        out->effective_twist[0] = d->command[0];
        out->effective_twist[1] = d->command[1];
        out->effective_twist[2] = d->command[2];
        out->action_scale = kStandingScale;
        out->gain_ratio = kStandingGainRatio;
        out->busy = true;
    } else if (s_ground_pick) {
        const float angle = 6.2831853f * s_ground_phase;
        out->net = SKILL_NET_GROUND_PICK;
        out->label = "ground_pick";
        out->effective_twist[0] = cosf(angle);
        out->effective_twist[1] = sinf(angle);
        out->effective_twist[2] = 0.f;
        out->action_scale = 1.0f;
        out->gain_ratio = 1.0f;
        out->busy = true;
    } else if (s_sit == SKILL_SIT_SITTING) {
        out->net = SKILL_NET_SITSTAND;
        out->label = "sit";
        out->effective_twist[0] = 1.f;
        out->effective_twist[1] = 0.f;
        out->effective_twist[2] = 0.f;
        out->action_scale = 1.0f;
        out->gain_ratio = 1.0f;
        out->busy = true;
    } else if (s_sit == SKILL_SIT_RISING) {
        out->net = SKILL_NET_SITSTAND;
        out->label = "rise";
        out->effective_twist[0] = 0.f;
        out->effective_twist[1] = 0.f;
        out->effective_twist[2] = 0.f;
        out->action_scale = kStandingScale;
        out->gain_ratio = kStandingGainRatio;
        out->busy = true;
    } else {
        float tw[3] = {out->effective_twist[0], out->effective_twist[1], out->effective_twist[2]};
        if (body_active) {
            tw[0] = tw[1] = tw[2] = 0.f;
        }
        const float mag = twist_mag(tw);
        /* skills_force("stand") parks label without clearing sit — honour sticky stand. */
        const bool force_stand = (s_label && strcmp(s_label, "stand") == 0 && mag < thresh);
        if (mag < thresh || force_stand) {
            out->net = SKILL_NET_STAND;
            out->label = "stand";
            out->action_scale = kStandingScale;
            out->gain_ratio = kStandingGainRatio;
        } else {
            out->net = SKILL_NET_WALK;
            out->label = "walk";
            out->action_scale = kWalkScale;
            out->gain_ratio = 1.f;
        }
        out->effective_twist[0] = tw[0];
        out->effective_twist[1] = tw[1];
        out->effective_twist[2] = tw[2];
        out->busy = false;
    }

    s_label = out->label;
}

bool skills_apply_net(const skill_step_t *step)
{
    if (!step) {
        return false;
    }
    const char *bake = "alpha_walking";
    switch (step->net) {
    case SKILL_NET_WALK:
        bake = "alpha_walking";
        break;
    case SKILL_NET_STAND:
        bake = "alpha_stand";
        break;
    case SKILL_NET_SKILL:
        bake = "alpha_stand";
        if (s_active_oneshot >= 0 && kOneshots[s_active_oneshot].bake) {
            bake = kOneshots[s_active_oneshot].bake;
        }
        break;
    case SKILL_NET_SITSTAND:
        bake = "alpha_sitstand";
        break;
    case SKILL_NET_GROUND_PICK:
        bake = "alpha_ground_pick";
        break;
    }
    const duck::Model *m = duck::FindModel(bake);
    if (!m) {
        ESP_LOGW(kTag, "bake '%s' missing for net %s", bake, step->label ? step->label : "?");
        return false;
    }
    if (duck::CurrentModel() != m) {
        duck::SetModel(m);
        ESP_LOGI(kTag, "net -> %s (%s)", step->label, bake);
    }
    return true;
}
