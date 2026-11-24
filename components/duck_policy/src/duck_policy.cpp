// Copyright 2026 ESP-Duck Authors
// SPDX-License-Identifier: MIT

#include "duck_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace duck {
namespace {

// Points at kModels[0] (constant-initialized: address constants don't care
// about translation-unit initialization order).
const Model *g_current = nullptr;

inline float Elu1(float x) { return x > 0.0f ? x : std::exp(x) - 1.0f; }

// y = dequant(W)·x + b over one row of W: accumulate int8·float in FP32, then
// apply the row's dequant scale once at the end. Sequential stride-1 reads so
// PSRAM streaming stays linear (this loop is what the MSPI prefetcher sees).
void GemmRow(const int8_t *wq, const float *scale, const float *b, const float *x,
             float *y, int in, int out) {
    for (int o = 0; o < out; ++o) {
        const int8_t *row = wq + static_cast<size_t>(o) * in;
        float acc = 0.0f;
        for (int i = 0; i < in; ++i) {
            acc += static_cast<float>(row[i]) * x[i];
        }
        y[o] = b[o] + scale[o] * acc;
    }
}

} // namespace

const Model *ModelAt(int i) {
    if (i < 0 || i >= kModelCount) return nullptr;
    return kModels[i];
}

const Model *FindModel(const char *name) {
    if (!name) return nullptr;
    for (int i = 0; i < kModelCount; ++i) {
        if (std::strcmp(kModels[i]->name, name) == 0) return kModels[i];
    }
    return nullptr;
}

void SetModel(const Model *m) {
    if (m) g_current = m;
}

const Model *CurrentModel() {
    if (!g_current) g_current = kModels[0];
    return g_current;
}

size_t WorkspaceFloats() {
    const Model *m = CurrentModel();
    int widest = 0;
    for (int i = 0; i < m->n_layers; ++i) {
        widest = std::max(widest, m->layers[i].in);
        widest = std::max(widest, m->layers[i].out);
    }
    return 2 * static_cast<size_t>(widest);
}

void Run(const float *obs, float *actions, float *ws) {
    const Model &model = *CurrentModel();
    const int n = model.n_layers;
    float *a = ws;                                    // normalized obs first
    float *b = ws + WorkspaceFloats() / 2;

    for (int i = 0; i < model.obs_len; ++i) {
        a[i] = (obs[i] - model.norm_mean[i]) / model.norm_std[i];
    }

    float *src = a;
    float *dst = b;
    for (int i = 0; i < n; ++i) {
        const Layer &layer = model.layers[i];
        float *out = (i == n - 1) ? actions : dst;
        GemmRow(layer.wq, layer.wscale, layer.b, src, out, layer.in, layer.out);
        if (layer.elu) { // only hidden layers carry elu (enforced by the extractor)
            for (int j = 0; j < layer.out; ++j) {
                dst[j] = Elu1(dst[j]);
            }
        }
        float *tmp = src;
        src = dst;
        dst = tmp;
    }
}

} // namespace duck
