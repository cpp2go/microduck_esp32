/*
 * SPDX-FileCopyrightText: 2026 ESP-Duck Authors
 * SPDX-License-Identifier: MIT
 *
 * microduck_esp32 — main entry point (ESP32-P4).
 *
 * Boots, verifies the active policy against FP32 ONNX-Runtime golden vectors,
 * runs a 50 Hz inference loop fed by the BMI270 IMU when present (gyro +
 * projected gravity; 15-joint XL330 bus fills obs[6:48] per duck-control/obs.rs),
 * and exposes an interactive shell over native USB with runtime policy switching.
 */

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "cmd_bus.h"
#include "cmd_link.h"
#include "servo_bus.h"
#include "duck_policy.hpp"
#include "imu.h"
#include "policy_output.h"
#include "shell.h"

static const char *kTag = "main";

// ---------------------------------------------------------------------------
// Golden-vector self-test
// ---------------------------------------------------------------------------
// Every baked policy carries FP32 ONNX-Runtime reference I/O in two halves:
// [0, N/2) uniform random (off-distribution, actions can be large — gate on
// relative fidelity), [N/2, N) around norm_mean (a still robot outputs
// near-zero actions, where relL2 is meaningless — gate on absolute error
// relative to S, the policy's own output scale = mean |a| over the random
// half). Each half is gated by the metric that can actually fail there.
namespace {

struct QMetrics {
    double cos, rel_l2, max_abs;
};

constexpr double kPassCosRand = 0.998;
constexpr double kPassRelL2Rand = 0.08;
constexpr double kPassAbsStill = 0.02; // fraction of S

QMetrics Score(const float *act, const float *ref, int n) {
    double dot = 0, na = 0, nb = 0, l2 = 0, maxabs = 0;
    for (int j = 0; j < n; ++j) {
        const double a = act[j], b = ref[j];
        dot += a * b;
        na += a * a;
        nb += b * b;
        l2 += (a - b) * (a - b);
        maxabs = std::max(maxabs, std::fabs(a - b));
    }
    return {dot / (std::sqrt(na) * std::sqrt(nb) + 1e-12),
            std::sqrt(l2) / (std::sqrt(nb) + 1e-12), maxabs};
}

bool Passes(const QMetrics &m, double scale, bool random_half) {
    return random_half ? (m.cos >= kPassCosRand && m.rel_l2 <= kPassRelL2Rand)
                       : (m.max_abs <= kPassAbsStill * scale);
}

/// Scores `model` against its own golden vectors; returns per-half worst
/// metrics, the model's action scale, and overall pass.
bool GoldenSelfTest(const duck::Model &model, QMetrics *worst_rand,
                    QMetrics *worst_still, double *scale_out) {
    static float ws[2 * duck::kMaxHidden];
    float act[duck::kMaxHidden];
    const int half = model.golden_count / 2;
    double scale_acc = 0.0;
    for (int v = 0; v < half; ++v) {
        double s = 0;
        for (int j = 0; j < model.act_len; ++j) {
            s += static_cast<double>(model.golden_act[v * model.act_len + j]) *
                 model.golden_act[v * model.act_len + j];
        }
        scale_acc += std::sqrt(s);
    }
    const double scale = scale_acc / half;
    *scale_out = scale;

    bool all_pass = true;
    *worst_rand = {2.0, 0.0, 0.0};
    *worst_still = {2.0, 0.0, 0.0};
    for (int v = 0; v < model.golden_count; ++v) {
        duck::SetModel(&model); // self-test runs on its own workspace; keep active consistent
        duck::Run(model.golden_obs + v * model.obs_len, act, ws);
        const bool rand_half = v < half;
        QMetrics m = Score(act, model.golden_act + v * model.act_len, model.act_len);
        if (!Passes(m, scale, rand_half)) all_pass = false;
        QMetrics *w = rand_half ? worst_rand : worst_still;
        w->cos = std::min(w->cos, m.cos);
        w->rel_l2 = std::max(w->rel_l2, m.rel_l2);
        w->max_abs = std::max(w->max_abs, m.max_abs);
    }
    return all_pass;
}

// ---------------------------------------------------------------------------
// 50 Hz inference loop (on-device INT8 — same role as robotd + duck-control)
// ---------------------------------------------------------------------------
// Order matches pollen-robotics/microduck robotd Controller::step:
//   IMU → XL330 SyncRead proprio → cmd block → INT8 policy → SyncWrite goals.
// Inference runs entirely on ESP32-P4 (baked duck_policy); host only bakes/tests.
struct LoopStats {
    std::atomic<bool> running{false};
    std::atomic<int64_t> min_us{INT64_MAX}; // fastest pass in the window
    std::atomic<int64_t> max_us{0};         // slowest pass in the window
    std::atomic<int64_t> sum_us{0};         // for the average
    std::atomic<int64_t> misses{0};         // ticks where a pass alone blew 20 ms
    std::atomic<int64_t> ticks{0};
};

// 4 KiB workspace (2 x 512 floats), static because the task is pinned and the
// ping-pong buffers are carved out of it once. Sized for the widest official
// policy; every baked model fits.
LoopStats g_loop;
float g_obs_ws[2 * duck::kMaxHidden];

/// Core-1 task: IMU + XL330 feedback -> 61D obs -> policy -> Goal Position @ 50 Hz.
void PolicyTask(void *) {
    float obs[duck::kMaxHidden] = {0};
    TickType_t next_wake = xTaskGetTickCount();
    int64_t window_ticks = 0;

    while (g_loop.running.load(std::memory_order_relaxed)) {
        int64_t t0 = esp_timer_get_time();
        imu::Poll();
        const float *gyro = imu::GyroRad();
        const float *grav = imu::ProjGravity();
        for (int i = 0; i < 3; ++i) {
            obs[i] = gyro[i];
            obs[3 + i] = grav[i];
        }
        (void)servo_bus_poll(nullptr);
        servo_bus_write_obs_proprio(obs);
        float act[duck::kMaxHidden];
        cmd_bus_apply_to_obs(obs);
        duck::Run(obs, act, g_obs_ws);
        float published[MICRODUCK_ACT_DIM];
        for (int j = 0; j < MICRODUCK_ACT_DIM; ++j) {
            published[j] = act[j];
        }
        microduck_policy_set_actions(published);
        (void)servo_bus_write_goals_rel(published);
        int64_t dt = esp_timer_get_time() - t0;
        (void)window_ticks;

        int64_t prev_min = g_loop.min_us.load(std::memory_order_relaxed);
        while (dt < prev_min && !g_loop.min_us.compare_exchange_weak(prev_min, dt)) {}
        int64_t prev_max = g_loop.max_us.load(std::memory_order_relaxed);
        while (dt > prev_max && !g_loop.max_us.compare_exchange_weak(prev_max, dt)) {}
        g_loop.sum_us.fetch_add(dt, std::memory_order_relaxed);
        if (dt > 20000) g_loop.misses.fetch_add(1, std::memory_order_relaxed);
        g_loop.ticks.fetch_add(1, std::memory_order_relaxed);
        ++window_ticks;

        // Period 20 ms measured from wake to wake, inference time included.
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(20));
    }
    vTaskDelete(nullptr);
}

void StartPolicyLoop() {
    if (g_loop.running.exchange(true)) return;
    g_loop.min_us = INT64_MAX;
    g_loop.max_us = 0;
    g_loop.sum_us = 0;
    g_loop.misses = 0;
    g_loop.ticks = 0;
    // Core 1 keeps the shell REPL responsive on core 0 (P4 is dual-core RISC-V).
    xTaskCreatePinnedToCore(PolicyTask, "policy", 12288, nullptr, 5, nullptr, 1);
}

void StopPolicyLoop() { g_loop.running = false; }

// ---------------------------------------------------------------------------
// Shell commands
// ---------------------------------------------------------------------------
// Registered below as two console commands: `info` and `policy` (with
// subcommands — esp_console names are single tokens, so "policy test" as a
// NAME would be rejected; the dispatch happens in CmdPolicy instead).

/// Chip, flash, PSRAM, weight placement and the active policy's report. Also
/// the quickest way to answer "where do the weights live" on a new board:
/// PSRAM-backed rodata means the fast pass; flash means ~3x slower.
int CmdInfo(int, char **) {
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash_mb = 0;
    esp_flash_get_size(nullptr, &flash_mb);
    const size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const uintptr_t w = reinterpret_cast<uintptr_t>(duck::CurrentModel()->layers[0].wq);
    const char *chip_name = "ESP32";
#if CONFIG_IDF_TARGET_ESP32P4
    chip_name = "ESP32-P4";
#elif CONFIG_IDF_TARGET_ESP32S3
    chip_name = "ESP32-S3";
#endif
    printf("chip        : %s rev %d, %d cores @ %d MHz\n", chip_name, chip.revision,
           chip.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("flash       : %" PRIu32 " MB\n", flash_mb / (1024 * 1024));
    printf("psram       : %zu KB %s\n", psram / 1024,
           psram > 0 ? "" : "(MISSING — a fast pass is not reachable without it)");
    // Weight placement: infer from Kconfig — vaddr alone is not trustworthy on
    // either S3 (SPIRAM_RODATA) or P4 (XIP from PSRAM).
#if defined(CONFIG_SPIRAM_RODATA) && CONFIG_SPIRAM_RODATA
    printf("weights     : PSRAM-backed rodata @ 0x%08" PRIXPTR " (SPIRAM_RODATA)\n", w);
#elif defined(CONFIG_SPIRAM_XIP_FROM_PSRAM) && CONFIG_SPIRAM_XIP_FROM_PSRAM
    printf("weights     : PSRAM/XIP rodata @ 0x%08" PRIXPTR " (SPIRAM_XIP_FROM_PSRAM)\n", w);
#else
    printf("weights     : flash @ 0x%08" PRIXPTR " (expect slower passes)\n", w);
#endif
    printf("active model: '%s' of %d baked\n",
           duck::CurrentModel()->name, duck::kModelCount);
    printf("model       : obs %d -> act %d, %d layers, workspace %zu floats\n",
           duck::CurrentModel()->obs_len, duck::CurrentModel()->act_len,
           duck::CurrentModel()->n_layers, duck::WorkspaceFloats());
    return 0;
}

/// `policy test`: replay the ACTIVE model's golden vectors on-device and print
/// per-half worst metrics plus the verdict. Exit code gates scripts.
int CmdPolicyTest(int, char **) {
    const duck::Model *m = duck::CurrentModel();
    printf("replaying %d golden vectors of '%s' (INT8 vs FP32 reference)...\n",
           m->golden_count, m->name);
    QMetrics wr, ws;
    double scale = 0.0;
    const bool pass = GoldenSelfTest(*m, &wr, &ws, &scale);
    printf("random   (off-distribution): cos %.6f (>= %.3f)  relL2 %.5f (<= %.2f)\n",
           wr.cos, kPassCosRand, wr.rel_l2, kPassRelL2Rand);
    printf("standing (scale %.2f)      : maxabs %.4f (<= %.2f%% of scale)\n",
           scale, ws.max_abs, kPassAbsStill * 100.0);
    printf("verdict: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/// `policy bench [n]`: exclusive timing of n forwards (avg/min/max). Stops the
/// background loop first — two tasks sharing the PSRAM bus both slow down
/// when they collide, which would muddy the number.
int CmdPolicyBench(int argc, char **argv) {
    const int iters = (argc > 1) ? atoi(argv[1]) : 100;
    if (iters <= 0 || iters > 100000) {
        printf("usage: policy bench [1..100000]\n");
        return 1;
    }
    StopPolicyLoop();
    vTaskDelay(pdMS_TO_TICKS(50));
    float obs[duck::kMaxHidden] = {0};
    float act[duck::kMaxHidden];
    int64_t min_us = INT64_MAX, max_us = 0, sum = 0;
    static float ws[2 * duck::kMaxHidden];
    for (int i = 0; i < iters; ++i) {
        int64_t t0 = esp_timer_get_time();
        duck::Run(obs, act, ws);
        int64_t dt = esp_timer_get_time() - t0;
        min_us = dt < min_us ? dt : min_us;
        max_us = dt > max_us ? dt : max_us;
        sum += dt;
    }
    printf("%d forwards: avg %.2f ms, min %.2f ms, max %.2f ms\n", iters,
           sum / 1e3 / iters, min_us / 1e3, max_us / 1e3);
    return 0;
}

/// `policy loop [off]`: start/stop the 50 Hz-period inference task on core 1.
int CmdPolicyLoop(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "off") == 0) {
        StopPolicyLoop();
        printf("50 Hz loop stopping\n");
        return 0;
    }
    StartPolicyLoop();
    printf("50 Hz loop running on core 1 (policy stats for the window)\n");
    return 0;
}

/// `policy stats`: per-pass min/avg/max and 20 ms overruns since loop start.
int CmdPolicyStats(int, char **) {
    const int64_t ticks = g_loop.ticks.load();
    if (!g_loop.running.load() || ticks == 0) {
        printf("loop idle — 'policy loop' starts it\n");
        return 1;
    }
    printf("ticks %lld, misses(>20ms) %lld, per-pass min/avg/max %.2f/%.2f/%.2f ms\n",
           static_cast<long long>(ticks),
           static_cast<long long>(g_loop.misses.load()),
           g_loop.min_us.load() / 1e3,
           g_loop.sum_us.load() / 1e3 / ticks,
           g_loop.max_us.load() / 1e3);
    return 0;
}

/// `policy list`: every baked policy, marking the active one.
int CmdPolicyList(int, char **) {
    for (int i = 0; i < duck::kModelCount; ++i) {
        const duck::Model *m = duck::ModelAt(i);
        printf("%2d. %s%s\n", i, m->name,
               m == duck::CurrentModel() ? "   <-- active" : "");
    }
    return 0;
}

/// `policy use <name>`: runtime hot-swap. The candidate must first pass its
/// own golden gate — a quantization-regressed model is refused, the active one
/// keeps running.
int CmdPolicyUse(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: policy use <name>   ('policy list' shows the baked set)\n");
        return 1;
    }
    const duck::Model *m = duck::FindModel(argv[1]);
    if (!m) {
        printf("no policy named '%s' baked\n", argv[1]);
        return 1;
    }
    if (m == duck::CurrentModel()) {
        printf("'%s' is already active\n", m->name);
        return 0;
    }
    printf("gating '%s' before switch...\n", m->name);
    QMetrics wr, ws;
    double scale = 0.0;
    if (!GoldenSelfTest(*m, &wr, &ws, &scale)) {
        printf("'%s' failed its gate (cos %.6f, still maxabs %.4f) — switch refused\n",
               m->name, wr.cos, ws.max_abs);
        return 1;
    }
    duck::SetModel(m);
    printf("active policy: '%s'\n", m->name);
    return 0;
}

/// `imu`: live BMI270 readout — tilt the board and watch it move.
int CmdServo(int argc, char **argv) {
    const char *sub = (argc > 1) ? argv[1] : "stats";
    if (strcmp(sub, "off") == 0) {
        servo_bus_torque_enable(false);
        return 0;
    }
    if (strcmp(sub, "on") == 0) {
        servo_bus_torque_enable(true);
        return 0;
    }
    servo_state_t st;
    servo_bus_get_state(&st);
    printf("torque %s, reads ok=%" PRIu32 " fail=%" PRIu32 "\n",
           st.torque_enabled ? "ON" : "OFF", st.read_ok, st.read_fail);
    printf("joint pos_rel (15 phys, mouth@9):");
    for (int i = 0; i < MICRODUCK_NUM_JOINTS; ++i) {
        printf(" %+.2f", st.pos_rel[i]);
    }
    printf("\n");
    return 0;
}

int CmdServoDispatch(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: servo <stats|on|off>\n");
        return 1;
    }
    return CmdServo(argc, argv);
}

int CmdImu(int, char **) {
    const float *a = imu::AccelG();
    const float *g = imu::GyroRad();
    const float *p = imu::ProjGravity();
    printf("accel g   : %+.3f %+.3f %+.3f\n", a[0], a[1], a[2]);
    printf("gyro rad/s: %+.3f %+.3f %+.3f\n", g[0], g[1], g[2]);
    printf("proj grav : %+.3f %+.3f %+.3f (upright = 0 0 -1)\n", p[0], p[1], p[2]);
    return 0;
}

int CmdReboot(int, char **) {
    fflush(stdout);
    esp_restart();
    return 0; // unreachable
}

// esp_console command names are single tokens: one "policy" command
// dispatches on the subcommand ("policy test", "policy bench 100", ...).
// Usage: policy <list|use|test|bench [n]|loop [off]|stats>
int CmdPolicy(int argc, char **argv) {
    const char *sub = (argc > 1) ? argv[1] : "";
    if (strcmp(sub, "test") == 0) return CmdPolicyTest(0, nullptr);
    if (strcmp(sub, "bench") == 0) return CmdPolicyBench(argc - 1, argv + 1);
    if (strcmp(sub, "loop") == 0) return CmdPolicyLoop(argc - 1, argv + 1);
    if (strcmp(sub, "stats") == 0) return CmdPolicyStats(0, nullptr);
    if (strcmp(sub, "list") == 0) return CmdPolicyList(0, nullptr);
    if (strcmp(sub, "use") == 0) return CmdPolicyUse(argc - 1, argv + 1);
    printf("usage: policy <list|use <name>|test|bench [n]|loop [off]|stats>\n");
    return 1;
}

} // namespace

// ---------------------------------------------------------------------------
// Startup banner
// ---------------------------------------------------------------------------
static constexpr const char *kBanner =
    "\033[2J\033[H"
    "\r\n"
    "\033[1;33m"
    "      _\r\n"
    "     / \\   microduck_esp32 \r\n"
    "     \\_C\\   microduck policies, INT8, on ESP32-P4\r\n"
    "      /  \\_.\r\n"
    "     '----'\r\n"
    "\033[0m\r\n";

extern "C" void app_main(void) {
    printf("%s", kBanner);
    ESP_LOGI(kTag, "microduck_esp32 starting");

    // Fail fast if the board lacks the one thing a fast pass needs.
    const size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram == 0) {
        ESP_LOGW(kTag, "no PSRAM found — weights will stream from flash, expect ~3x slower");
    }

    cmd_bus_init();
    cmd_link_start();

    if (imu::Init()) {
        ESP_LOGI(kTag, "IMU online — loop observations carry real gyro + gravity");
    } else {
        ESP_LOGW(kTag, "IMU offline — loop observations stay synthetic");
    }

    if (servo_bus_init()) {
        ESP_LOGI(kTag, "XL330 bus online — closed-loop 50 Hz control");
    } else {
        ESP_LOGW(kTag, "servo bus offline — proprioception and actuation disabled");
    }

    QMetrics wr, ws;
    double scale = 0.0;
    const bool pass = GoldenSelfTest(*duck::CurrentModel(), &wr, &ws, &scale);
    ESP_LOGI(kTag, "INT8 self-test of '%s': random cos %.6f/relL2 %.5f, "
             "standing maxabs %.4f (scale %.2f) — %s",
             duck::CurrentModel()->name, wr.cos, wr.rel_l2, ws.max_abs, scale,
             pass ? "PASS" : "FAIL");

    StartPolicyLoop();

    shell::Shell::Config shell_cfg;
    shell_cfg.prompt          = "microduck> ";
    shell_cfg.welcome_message = "microduck_esp32 ready. 'help' for commands.\r\n";
    // 6 KiB overflowed when the watchdog backtrace printer ran inside the REPL
    // task; the bench prints and panic dumps need the headroom.
    shell_cfg.task_stack_size = 10240;
    shell_cfg.task_priority   = 2;
    shell_cfg.max_history_len = 20;

    static shell::Shell g_shell(shell_cfg);
    g_shell.RegisterCommand({"info",   "Chip, PSRAM, weight placement, active policy",
                            "",        CmdInfo});
    g_shell.RegisterCommand({"policy", "List/use baked policies; gate, bench, loop",
                            "<list|use <name>|test|bench [n]|loop [off]|stats>",
                            CmdPolicy});
    g_shell.RegisterCommand({"imu",    "Live BMI270 readout (tilt the board)",
                            "",        CmdImu});
    g_shell.RegisterCommand({"servo",  "XL330 bus: stats, emergency torque off/on",
                            "<stats|on|off>", CmdServoDispatch});
    g_shell.RegisterCommand({"reboot", "Reboot the device", "", CmdReboot});
    g_shell.Start();
}
