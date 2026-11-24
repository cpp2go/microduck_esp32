// Host-side verification: replays EVERY baked policy's golden vectors through
// the C++ forward pass and scores them with the same scale-aware gates the
// on-device self-test uses. See run.sh.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "duck_policy.hpp"

namespace {

struct QMetrics {
    double cos, rel_l2, max_abs;
};

constexpr double kPassCosRand = 0.998;
constexpr double kPassRelL2Rand = 0.08;
constexpr double kPassAbsStill = 0.02;

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

} // namespace

int main() {
    auto *ws = static_cast<float *>(
        malloc(2 * duck::kMaxHidden * sizeof(float)));
    if (!ws) return 1;
    float act[duck::kMaxHidden];

    bool all_pass = true;
    for (int mi = 0; mi < duck::kModelCount; ++mi) {
        const duck::Model &m = *duck::ModelAt(mi);
        const int half = m.golden_count / 2;
        double scale_acc = 0.0;
        for (int v = 0; v < half; ++v) {
            double s = 0;
            for (int j = 0; j < m.act_len; ++j) {
                s += static_cast<double>(m.golden_act[v * m.act_len + j]) *
                     m.golden_act[v * m.act_len + j];
            }
            scale_acc += std::sqrt(s);
        }
        const double scale = scale_acc / half;

        double wc_rand = 2.0, wr_rand = 0.0, wa_rand = 0.0, wa_still = 0.0;
        bool pass = true;
        for (int v = 0; v < m.golden_count; ++v) {
            const bool rand_half = v < half;
            duck::SetModel(&m);
            duck::Run(m.golden_obs + v * m.obs_len, act, ws);
            QMetrics q = Score(act, m.golden_act + v * m.act_len, m.act_len);
            if (rand_half) {
                wc_rand = std::min(wc_rand, q.cos);
                wr_rand = std::max(wr_rand, q.rel_l2);
                wa_rand = std::max(wa_rand, q.max_abs);
            } else {
                wa_still = std::max(wa_still, q.max_abs);
            }
            const bool v_pass = rand_half
                ? (q.cos >= kPassCosRand && q.rel_l2 <= kPassRelL2Rand)
                : (q.max_abs <= kPassAbsStill * scale);
            pass = pass && v_pass;
        }
        printf("%-18s scale %8.2f  cos(rand) %.6f  relL2(rand) %.4f  "
               "abs(rand) %.4f  abs(stand) %.4f  %s\n",
               m.name, scale, wc_rand, wr_rand, wa_rand, wa_still, pass ? "PASS" : "FAIL");
        all_pass = all_pass && pass;
    }
    printf("%s (%d policies)\n", all_pass ? "ALL PASS" : "FAILURES", duck::kModelCount);

    // Informational only — this Mac is not the target; see README for the
    // on-device benchmark.
    float obs[duck::kMaxHidden] = {0};
    for (int j = 0; j < duck::CurrentModel()->obs_len; ++j) obs[j] = 0.01f * j;
    const int iters = 2000;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int i = 0; i < iters; ++i) {
        float act2[duck::kMaxHidden];
        duck::Run(obs, act2, ws);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double us = (t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) / 1e3;
    printf("host timing: %.1f us/forward (informational — production runs on ESP32-P4)\n",
           us / iters);

    free(ws);
    return all_pass ? 0 : 2;
}
