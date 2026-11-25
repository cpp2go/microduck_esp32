/*
 * SPDX-FileCopyrightText: 2026 ESP-Duck Authors
 * SPDX-License-Identifier: MIT
 *
 * microduck_esp32 — main entry point (ESP32-P4).
 *
 * Boots, verifies the active policy against FP32 ONNX-Runtime golden vectors,
 * runs a 50 Hz inference loop fed by the ICM-42688 (SPI) when present (gyro +
 * projected gravity; 15-joint servo bus fills obs[6:48] per duck-control/obs.rs),
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
#include "sound.h"
#include "tof.h"
#include "vision.h"
#include "voice_control.h"
#include "voice_match.h"

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
//   IMU → Feetech SyncRead proprio → cmd block → INT8 policy → SyncWrite goals.
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

/// Core-1 task: IMU + servo feedback -> 61D obs -> policy -> joint goals @ 50 Hz.
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
#if CONFIG_MICRODUCK_TOF_ENABLE && CONFIG_MICRODUCK_TOF_GATE_IN_LOOP
        {
            tof_obstacle_t gate{};
            tof_obstacle_update(&gate);
            /* Twist lives at obs[48:51]; gate scales/zeros like autod STOP/REACT. */
            float twist[3] = {obs[48], obs[49], obs[50]};
            tof_obstacle_apply_twist(&gate, twist);
            obs[48] = twist[0];
            obs[49] = twist[1];
            obs[50] = twist[2];
        }
#endif
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

/// `imu`: live ICM-42688 readout — tilt the board and watch it move.
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

/// `sound <list|tag|record [ms]|level [ms]>` — play cues or mic capture.
int CmdSound(int argc, char **argv) {
    if (argc < 2) {
        printf("usage: sound <list|tag|record [ms]|level [ms]>\n");
        return 1;
    }
    if (strcmp(argv[1], "list") == 0) {
        int n = 0;
        const char *const *tags = sound_tags(&n);
        printf("tags (%d):", n);
        for (int i = 0; i < n; ++i) {
            printf(" %s", tags[i]);
        }
        printf("\navailable=%s play=%s mic=%s rate=%d Hz (ES8311+NS4150B)\n",
               sound_available() ? "yes" : "no", sound_hw_ready() ? "ready" : "log-only",
               sound_mic_ready() ? "ready" : "off", sound_sample_rate());
        return 0;
    }
    if (strcmp(argv[1], "record") == 0) {
        const int ms = (argc > 2) ? atoi(argv[2]) : 500;
        if (ms <= 0 || ms > 5000) {
            printf("record ms must be 1..5000\n");
            return 1;
        }
        if (!sound_mic_ready()) {
            printf("mic not ready\n");
            return 1;
        }
        const int n = (sound_sample_rate() * ms) / 1000;
        int16_t *buf = static_cast<int16_t *>(malloc(static_cast<size_t>(n) * sizeof(int16_t)));
        if (!buf) {
            printf("oom\n");
            return 1;
        }
        const int got = sound_record(buf, n, ms + 300);
        long long sum = 0;
        int peak = 0;
        for (int i = 0; i < got; ++i) {
            int v = buf[i];
            if (v < 0) {
                v = -v;
            }
            sum += v;
            if (v > peak) {
                peak = v;
            }
        }
        const int mean = got > 0 ? static_cast<int>(sum / got) : 0;
        printf("record %d/%d frames @ %d Hz, peak=%d mean_abs=%d\n", got, n, sound_sample_rate(), peak,
               mean);
        free(buf);
        return got > 0 ? 0 : 1;
    }
    if (strcmp(argv[1], "level") == 0) {
        const int ms = (argc > 2) ? atoi(argv[2]) : 100;
        const int peak = sound_mic_level(ms);
        if (peak < 0) {
            printf("mic level failed (mic off?)\n");
            return 1;
        }
        printf("mic peak=%d (window %d ms)\n", peak, ms);
        return 0;
    }
    if (!sound_play(argv[1])) {
        printf("sound play failed (init off?)\n");
        return 1;
    }
    return 0;
}

/// `tof <status|grab|inject <scene>|gate>` — distance frame + obstacle gate.
int CmdTof(int argc, char **argv) {
    const char *sub = (argc > 1) ? argv[1] : "status";
    if (strcmp(sub, "status") == 0) {
        tof_obstacle_t g{};
        tof_obstacle_update(&g);
        printf("backend=%s available=%s stop_m=%.2f react_m=%.2f\n", tof_backend_name(),
               tof_available() ? "yes" : "no", tof_stop_m(), tof_react_m());
        printf("gate ahead=%.2fm away=%+.0f age=%.2fs stop=%d slow=%d scale=%.2f\n", g.ahead_m,
               g.away, g.age_s, g.stop ? 1 : 0, g.slow ? 1 : 0, g.speed_scale);
        return 0;
    }
    if (strcmp(sub, "grab") == 0) {
        tof_frame_t fr{};
        if (!tof_grab(&fr)) {
            printf("tof grab failed\n");
            return 1;
        }
        uint32_t min_mm = 65535, max_mm = 0;
        for (int i = 0; i < TOF_ZONE_COUNT; ++i) {
            if (fr.status[i] == 0 && fr.dist_mm[i] > 0) {
                if (fr.dist_mm[i] < min_mm) {
                    min_mm = fr.dist_mm[i];
                }
                if (fr.dist_mm[i] > max_mm) {
                    max_mm = fr.dist_mm[i];
                }
            }
        }
        printf("frame seq=%" PRIu64 " ts_us=%" PRId64 " min_mm=%u max_mm=%u (8x8)\n", fr.seq,
               fr.ts_us, (unsigned)(min_mm == 65535 ? 0 : min_mm), (unsigned)max_mm);
        return 0;
    }
    if (strcmp(sub, "inject") == 0) {
        if (argc < 3) {
            printf("usage: tof inject <clear|near|wall|left|right>\n");
            return 1;
        }
        if (!tof_inject(argv[2])) {
            printf("inject failed\n");
            return 1;
        }
        char *args[] = {argv[0], const_cast<char *>("status")};
        return CmdTof(2, args);
    }
    if (strcmp(sub, "gate") == 0) {
        char *args[] = {argv[0], const_cast<char *>("status")};
        return CmdTof(2, args);
    }
    printf("usage: tof <status|grab|inject <scene>|gate>\n");
    return 1;
}

/// `voice <status|listen [off]|inject <intent>|test>` — ES8311 keyword intents.
int CmdVoice(int argc, char **argv) {
    const char *sub = (argc > 1) ? argv[1] : "status";
    if (strcmp(sub, "status") == 0) {
        printf("available=%s listening=%s last=%s mic=%s\n", voice_available() ? "yes" : "no",
               voice_listening() ? "yes" : "no", voice_intent_name(voice_last_intent()),
               sound_mic_ready() ? "ready" : "off");
        return 0;
    }
    if (strcmp(sub, "listen") == 0) {
        if (argc > 2 && strcmp(argv[2], "off") == 0) {
            voice_listen_stop();
            printf("voice listen off\n");
            return 0;
        }
        if (!voice_listen_start()) {
            printf("voice listen start failed\n");
            return 1;
        }
        printf("voice listen on (speak near mic; or voice inject <intent>)\n");
        return 0;
    }
    if (strcmp(sub, "inject") == 0) {
        if (argc < 3) {
            printf("usage: voice inject <stop|go|sit|stand|quack>\n");
            return 1;
        }
        if (!voice_inject(argv[2])) {
            printf("inject failed\n");
            return 1;
        }
        return 0;
    }
    if (strcmp(sub, "test") == 0) {
        /* Synth each template through the matcher — no mic needed. */
        const int sr = 16000;
        const int n = sr / 2;
        int16_t *buf = static_cast<int16_t *>(malloc(static_cast<size_t>(n) * sizeof(int16_t)));
        if (!buf) {
            printf("oom\n");
            return 1;
        }
        int fails = 0;
        const voice_match_id_t ids[] = {VOICE_MATCH_STOP, VOICE_MATCH_GO, VOICE_MATCH_SIT,
                                        VOICE_MATCH_STAND, VOICE_MATCH_QUACK};
        for (voice_match_id_t id : ids) {
            voice_synth_utterance(id, buf, n, sr);
            float feat[VOICE_FEAT_DIM];
            float score = 0.f;
            if (!voice_extract_feat(buf, n, feat)) {
                printf("%s: extract FAIL\n", voice_match_name(id));
                ++fails;
                continue;
            }
            const voice_match_id_t got = voice_match_feat(feat, &score);
            const bool ok = got == id;
            printf("%s: match=%s score=%.3f %s\n", voice_match_name(id), voice_match_name(got),
                   score, ok ? "OK" : "FAIL");
            if (!ok) {
                ++fails;
            }
        }
        free(buf);
        return fails == 0 ? 0 : 1;
    }
    printf("usage: voice <status|listen [off]|inject <intent>|test>\n");
    return 1;
}

/// `vision <status|grab|detect>`: frame-bus bring-up (synthetic until CSI wired).
int CmdVision(int argc, char **argv) {
    const char *sub = (argc > 1) ? argv[1] : "status";
    if (strcmp(sub, "status") == 0) {
        printf("backend=%s available=%s mean_luma=%d\n", vision_backend_name(),
               vision_available() ? "yes" : "no", vision_mean_luma());
        return 0;
    }
    if (strcmp(sub, "grab") == 0) {
        vision_frame_t fr;
        if (!vision_grab(&fr)) {
            printf("vision grab failed\n");
            return 1;
        }
        printf("frame seq=%" PRIu64 " %ux%u stride=%u ts_us=%" PRId64 " mean=%d\n", fr.seq,
               (unsigned)fr.width, (unsigned)fr.height, (unsigned)fr.stride, fr.ts_us,
               vision_mean_luma());
        vision_release();
        return 0;
    }
    if (strcmp(sub, "detect") == 0) {
        vision_detect_t ball{}, pet{};
        (void)vision_detect_ball(&ball);
        (void)vision_detect_pet(&pet);
        printf("ball present=%d  pet present=%d  (YOLO/duck-detect not on P4)\n",
               ball.present ? 1 : 0, pet.present ? 1 : 0);
        return 0;
    }
    printf("usage: vision <status|grab|detect>\n");
    return 1;
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

void OnVoiceIntent(voice_intent_t intent, void *) {
    microduck_command_block_t cmd = MICRODUCK_COMMAND_IDLE;
    uint32_t seq = 0;
    cmd_bus_get(&cmd, &seq);
    switch (intent) {
    case VOICE_INTENT_STOP:
        cmd.twist[0] = 0.f;
        cmd.twist[1] = 0.f;
        cmd.twist[2] = 0.f;
        cmd_bus_set(&cmd, seq + 1);
        (void)sound_play("no");
        break;
    case VOICE_INTENT_GO: {
#if defined(CONFIG_MICRODUCK_VOICE_GO_VX)
        const float vx = CONFIG_MICRODUCK_VOICE_GO_VX / 1000.0f;
#else
        const float vx = 0.25f;
#endif
        cmd.twist[0] = vx;
        cmd.twist[1] = 0.f;
        cmd.twist[2] = 0.f;
        cmd_bus_set(&cmd, seq + 1);
        (void)sound_play("yes");
        break;
    }
    case VOICE_INTENT_SIT:
        (void)shell::Execute("policy use alpha_sitstand");
        (void)sound_play("curious");
        break;
    case VOICE_INTENT_STAND:
        (void)shell::Execute("policy use alpha_stand");
        (void)sound_play("yes");
        break;
    case VOICE_INTENT_QUACK:
        (void)sound_play("hello");
        break;
    default:
        break;
    }
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
        ESP_LOGI(kTag, "servo bus online — closed-loop 50 Hz control");
    } else {
        ESP_LOGW(kTag, "servo bus offline — proprioception and actuation disabled");
    }

    if (sound_init()) {
        ESP_LOGI(kTag, "sound cues ready (robotd tag names; WAV bank not on P4)");
    } else {
        ESP_LOGW(kTag, "sound offline");
    }

    if (vision_init()) {
        ESP_LOGI(kTag, "vision frame bus ready backend=%s", vision_backend_name());
    } else {
        ESP_LOGW(kTag, "vision offline");
    }

    if (tof_init()) {
        ESP_LOGI(kTag, "tof frame bus ready backend=%s", tof_backend_name());
    } else {
        ESP_LOGW(kTag, "tof offline");
    }

    if (voice_init()) {
        voice_set_callback(OnVoiceIntent, nullptr);
        ESP_LOGI(kTag, "voice control ready (VAD+templates; no Whisper ASR)");
#if CONFIG_MICRODUCK_VOICE_AUTOSTART
        if (voice_listen_start()) {
            ESP_LOGI(kTag, "voice listen autostart");
        }
#endif
    } else {
        ESP_LOGW(kTag, "voice control offline");
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
    g_shell.RegisterCommand({"imu",    "Live ICM-42688 SPI readout (tilt the board)",
                            "",        CmdImu});
    g_shell.RegisterCommand({"servo",  "Servo bus (Feetech/S288): stats, torque off/on",
                            "<stats|on|off>", CmdServoDispatch});
    g_shell.RegisterCommand({"sound",  "ES8311 play tags; mic record/level",
                            "<list|tag|record [ms]|level [ms]>", CmdSound});
    g_shell.RegisterCommand({"vision", "Frame bus: status, synthetic grab, detect stub",
                            "<status|grab|detect>", CmdVision});
    g_shell.RegisterCommand({"tof", "ToF 8x8 + obstacle gate (tofd/autod-style)",
                            "<status|grab|inject <scene>|gate>", CmdTof});
    g_shell.RegisterCommand({"voice", "Mic keyword intents (or inject)",
                            "<status|listen [off]|inject <intent>|test>", CmdVoice});
    g_shell.Start();
}
