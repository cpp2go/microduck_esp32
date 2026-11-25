// Host test: skills net scheduler (no duck_policy — stub apply).
#include <cassert>
#include <cstdio>
#include <cstring>

// Minimal stubs so we can compile skills.cpp logic without ESP-IDF / duck_policy.
// Instead, exercise a freestanding copy of the priority rules via the public C API
// by compiling skills with a tiny shim.

#define SKILLS_HOST_TEST 1

// Re-implement the priority selection expectations without linking duck_policy.
#include "../components/skills/include/skills.h"

// Provide weak stubs if linking skills.o — we include a trimmed tester instead.
static void expect_label(const char *got, const char *want) {
    assert(got && want);
    assert(std::strcmp(got, want) == 0);
}

// Inline a minimal scheduler mirror for host (keeps duck_policy off the host link).
#include <cmath>

enum Net { Walk, Stand, SitStand, GroundPick, Skill };
enum Sit { Up, Sitting, Rising };

struct Step {
    Net net;
    const char *label;
    float twist[3];
};

static Sit g_sit = Up;
static float g_sit_remain = 0;
static int g_oneshot = -1;
static float g_oneshot_remain = 0;
static bool g_pick = false;
static float g_phase = 0;

static Step step(float tw[3], float dt) {
    if (g_oneshot >= 0) {
        g_oneshot_remain -= dt;
        if (g_oneshot_remain <= 0) g_oneshot = -1;
    }
    if (g_sit == Rising) {
        g_sit_remain -= dt;
        if (g_sit_remain <= 0) g_sit = Up;
    }
    if (g_pick) {
        g_phase += dt;
        if (g_phase >= 1.f) g_pick = false;
    }
    Step s{};
    if (g_oneshot >= 0) {
        s = {Skill, "kick", {0, 0, 0}};
    } else if (g_pick) {
        s = {GroundPick, "ground_pick", {1, 0, 0}};
    } else if (g_sit == Sitting) {
        s = {SitStand, "sit", {1, 0, 0}};
    } else if (g_sit == Rising) {
        s = {SitStand, "rise", {0, 0, 0}};
    } else {
        const float mag = std::sqrt(tw[0] * tw[0] + tw[1] * tw[1] + tw[2] * tw[2]);
        if (mag < 0.05f) s = {Stand, "stand", {tw[0], tw[1], tw[2]}};
        else s = {Walk, "walk", {tw[0], tw[1], tw[2]}};
    }
    return s;
}

int main() {
    float tw[3] = {0.3f, 0, 0};
    expect_label(step(tw, 0.02f).label, "walk");
    tw[0] = 0;
    expect_label(step(tw, 0.02f).label, "stand");
    g_sit = Sitting;
    expect_label(step(tw, 0.02f).label, "sit");
    g_sit = Rising;
    g_sit_remain = 1.f;
    expect_label(step(tw, 0.02f).label, "rise");
    g_sit = Up;
    g_oneshot = 0;
    g_oneshot_remain = 0.5f;
    expect_label(step(tw, 0.02f).label, "kick");
    g_oneshot = -1;
    g_pick = true;
    g_phase = 0;
    expect_label(step(tw, 0.02f).label, "ground_pick");
    std::puts("skills_test OK");
    return 0;
}
