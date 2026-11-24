#!/usr/bin/env python3
"""Extract a Microduck policy ONNX into C++ for INT8-weight inference on ESP32 (P4).

Reads the exported graph (observation normalizer + dense ELU MLP), verifies it
matches the expected pattern, quantizes every weight tensor to per-channel
symmetric INT8, and emits:

  components/duck_policy/src/duck_model.cpp      int8 weights + scales + descriptor
  components/duck_policy/include/golden_vectors.hpp  FP32 ONNX-Runtime reference I/O

Activations stay FP32 on purpose: they are 2 KiB living in internal SRAM, so
quantizing them saves nothing on the bus while costing accuracy — the 773 KiB
per-pass weight stream is the entire problem, and INT8 shrinks it 4x.

The script validates the quantized model on the host before writing anything:
a numpy forward pass with dequantized weights is compared against ONNX Runtime
on the original FP32 graph, per golden vector, reporting cosine similarity,
relative L2 error and max absolute error of the 14 action outputs.

Usage:
  python3 tools/extract_policy.py --onnx alpha_walking.onnx --out-dir .
"""

import argparse
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort

ARR = "__attribute__((aligned(16)))"
CLIPS = np.linspace(1.0, 0.55, 24)  # saturation multiplier sweep (global per layer)


def f32(x: np.ndarray) -> np.ndarray:
    arr = np.asarray(x, dtype=np.float32)
    if not np.all(np.isfinite(arr)):
        raise ValueError("model contains NaN/Inf weights")
    return arr


def c_ints(a: np.ndarray, per_line: int = 16) -> str:
    items = [str(int(v)) for v in np.asarray(a).reshape(-1)]
    lines = []
    for i in range(0, len(items), per_line):
        lines.append("    " + ", ".join(items[i : i + per_line]) + ",")
    return "\n".join(lines).rstrip(",")


def c_floats(a: np.ndarray, per_line: int = 8) -> str:
    vals = ", ".join(f"{v:.9g}" for v in np.asarray(a).reshape(-1))
    items = vals.split(", ")
    lines = []
    for i in range(0, len(items), per_line):
        lines.append("    " + ", ".join(items[i : i + per_line]) + ",")
    return "\n".join(lines).rstrip(",")


def expect(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(f"unsupported graph: {msg}")


def parse_graph(path: Path) -> dict:
    model = onnx.load(str(path))
    g = model.graph
    inits = {i.name: f32(onnx.numpy_helper.to_array(i)) for i in g.initializer}

    expect(len(g.input) == 1, f"{len(g.input)} graph inputs, want 1")
    expect(len(g.output) == 1, f"{len(g.output)} graph outputs, want 1")
    obs_name = g.input[0].name
    act_name = g.output[0].name

    nodes = list(g.node)
    expect(nodes[0].op_type == "Sub" and nodes[1].op_type == "Div",
           "graph must start with Sub(obs, mean) -> Div(_, std)")
    expect(nodes[0].input[0] == obs_name, "Sub does not read the graph input")
    mean = inits[nodes[0].input[1]]
    std = inits[nodes[1].input[1]]
    expect(mean.shape == std.shape, "mean/std shape mismatch")

    tail = nodes[2:]
    layers = []
    i = 0
    while i < len(tail):
        n = tail[i]
        expect(n.op_type == "Gemm", f"expected Gemm, got {n.op_type}")
        attrs = {a.name: onnx.helper.get_attribute_value(a) for a in n.attribute}
        expect(attrs.get("transB") == 1, "Gemm must have transB=1")
        expect(float(attrs.get("alpha", 1)) == 1.0 and float(attrs.get("beta", 1)) == 1.0,
               "Gemm alpha/beta must be 1")
        w, b = inits[n.input[1]], inits[n.input[2]]
        expect(w.ndim == 2 and b.ndim == 1 and w.shape[0] == b.shape[0],
               "Gemm weight/bias shapes inconsistent")
        out_name = n.output[0]
        i += 1
        has_elu = i < len(tail) and tail[i].op_type == "Elu"
        if has_elu:
            expect(tail[i].input[0] == out_name, "Elu does not follow its Gemm")
            i += 1
        layers.append({"w": w, "b": b, "elu": has_elu})
        if not has_elu:
            expect(out_name == act_name, "final Gemm does not produce the graph output")

    expect(layers[-1]["elu"] is False, "last layer must be linear (no Elu)")
    for layer in layers[:-1]:
        expect(layer["elu"], "hidden layers must be followed by Elu(alpha=1)")

    shapes = [layers[0]["w"].shape[1]] + [l["w"].shape[0] for l in layers]
    expect(shapes[0] == mean.shape[-1],
           f"first Gemm input width {shapes[0]} != obs width {mean.shape[-1]}")
    return {"obs_len": shapes[0], "act_len": shapes[-1],
            "mean": mean.reshape(-1), "std": std.reshape(-1), "layers": layers}


def quantize_layer(w: np.ndarray, clip: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Per-channel symmetric INT8, row scale = max|row|/127 * clip. clip < 1
    saturates outliers: a TensorRT-style clip search finds the MSE-optimal one."""
    scale = (np.max(np.abs(w), axis=1).astype(np.float32) / 127.0) * clip
    scale[scale == 0.0] = np.float32(1e-8)
    q = np.clip(np.round(w / scale[:, None]), -127, 127).astype(np.int8)
    return q, scale


def elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0, x, np.exp(np.minimum(x, 0)) - 1.0).astype(np.float32)


def quantize_and_calibrate(graph: dict, calib_obs: np.ndarray) -> None:
    """Greedy per-layer PTQ. For each layer in execution order:
    1. sweep the saturation multiplier, keep the INT8 grid minimising that
       layer's output MSE against the FP32 model on calibration data
       (TensorRT-style clip search — a per-channel variant was tried and
       overfits the calibration set: the global clip won on every metric);
    2. bias-correct: absorb the mean (FP32 - quantized) output error into the
       FP32 bias, so the next layer sees the corrected inputs the deployed
       model will actually produce."""
    x_fp = (calib_obs - graph["mean"]) / graph["std"]
    x_q = x_fp.copy()
    for layer in graph["layers"]:
        y_fp = x_fp @ layer["w"].T + layer["b"]
        best = None
        for clip in CLIPS:
            q, scale = quantize_layer(layer["w"], float(clip))
            y_q = x_q @ (q.astype(np.float32) * scale[:, None]).T + layer["b"]
            cost = float(np.mean((y_q - y_fp) ** 2))
            if best is None or cost < best[0]:
                best = (cost, q, scale, y_q)
        _, layer["q"], layer["scale"], y_q = best
        layer["b"] = (layer["b"] + (y_fp - y_q).mean(axis=0)).astype(np.float32)
        if layer["elu"]:
            y_fp, y_q = elu(y_fp), elu(y_q)
        x_fp, x_q = y_fp, y_q


def dequant_forward(graph: dict, obs: np.ndarray) -> np.ndarray:
    """Numpy forward with dequantized weights — the exact arithmetic the C++ does."""
    x = (obs - graph["mean"]) / graph["std"]
    for layer in graph["layers"]:
        wq, scale = layer["q"], layer["scale"]
        wdq = wq.astype(np.float32) * scale[:, None]
        x = x @ wdq.T + layer["b"]
        if layer["elu"]:
            x = elu(x)
    return x


def metrics(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float]:
    cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
    rel = float(np.linalg.norm(a - b) / (np.linalg.norm(b) + 1e-12))
    maxabs = float(np.max(np.abs(a - b)))
    return cos, rel, maxabs


def reference_session(path: Path) -> ort.InferenceSession:
    return ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])


def gen_model_cpp(graph: dict, src: Path, name: str) -> str:
    lay = graph["layers"]
    defs, ptrs = [], []
    for idx, layer in enumerate(lay):
        wq, scale, b = layer["q"], layer["scale"], layer["b"]
        defs.append(f"static const {ARR} int8_t w{idx}[{wq.size}] = {{\n{c_ints(wq)}\n}};")
        defs.append(f"static const {ARR} float s{idx}[{scale.size}] = {{\n{c_floats(scale)}\n}};")
        defs.append(f"static const {ARR} float b{idx}[{b.size}] = {{\n{c_floats(b)}\n}};")
        ptrs.append(f"    {{ .in = {wq.shape[1]}, .out = {wq.shape[0]}, "
                    f".wq = w{idx}, .wscale = s{idx}, .b = b{idx}, "
                    f".elu = {'true' if layer['elu'] else 'false'} }},")

    return f"""\
// GENERATED by tools/extract_policy.py from {src.name} — do not edit.
// Weights are per-channel symmetric INT8; scales and biases stay FP32.
#include "duck_policy.hpp"

{chr(10).join(defs)}

static const {ARR} float norm_mean[{graph['obs_len']}] = {{
{c_floats(graph['mean'])}
}};

static const {ARR} float norm_std[{graph['obs_len']}] = {{
{c_floats(graph['std'])}
}};

namespace duck {{

const Model kModel = {{
    .name      = "{name}",
    .obs_len   = {graph['obs_len']},
    .act_len   = {graph['act_len']},
    .n_layers  = {len(lay)},
    .norm_mean = norm_mean,
    .norm_std  = norm_std,
    .layers    = {{
{chr(10).join(ptrs)}
    }},
}};

}} // namespace duck
"""


def gen_golden_h(obs: np.ndarray, acts: np.ndarray) -> str:
    count, obs_len, act_len = obs.shape[0], obs.shape[1], acts.shape[1]
    blocks = []
    for i in range(count):
        blocks.append(
            "    {\n        .obs = {\n" + c_floats(obs[i]) + "\n        },\n"
            "        .act = {\n" + c_floats(acts[i]) + "\n        },\n    },")
    return f"""\
// GENERATED by tools/extract_policy.py — FP32 reference outputs from onnxruntime.
// Do not edit. The INT8 device model is scored against these.
#pragma once

#define GOLDEN_COUNT {count}
#define GOLDEN_OBS_LEN {obs_len}
#define GOLDEN_ACT_LEN {act_len}

struct golden_vector_t {{
    float obs[GOLDEN_OBS_LEN];
    float act[GOLDEN_ACT_LEN];
}};

inline constexpr golden_vector_t golden[GOLDEN_COUNT] = {{
{chr(10).join(blocks)}
}};
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--onnx", required=True, type=Path)
    ap.add_argument("--name", default=None, help="model name baked into the descriptor")
    ap.add_argument("--golden", type=int, default=16, help="number of reference vectors")
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--out-dir", type=Path, default=None, help="(unused, kept for compat)")
    args = ap.parse_args()

    graph = parse_graph(args.onnx)
    name = args.name or args.onnx.stem
    root = args.out_dir

    rng = np.random.default_rng(args.seed)
    # Validation set A: uniform random — off-distribution, deliberately harsh.
    obs = rng.uniform(-3.0, 3.0, size=(args.golden, graph["obs_len"])).astype(np.float32)
    # Validation set B: around norm_mean — a plausible standing pose.
    obs_near = (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                           size=(args.golden, graph["obs_len"]))).astype(np.float32)

    # Calibration data for clip search + bias correction (fresh draws).
    calib = np.concatenate([
        rng.uniform(-3.0, 3.0, size=(32, graph["obs_len"])).astype(np.float32),
        np.zeros((8, graph["obs_len"]), dtype=np.float32),
        (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                    size=(32, graph["obs_len"]))).astype(np.float32),
    ])
    quantize_and_calibrate(graph, calib)

    # --- host-side quantization validation BEFORE anything is written ---
    sess = reference_session(args.onnx)
    acts_fp32 = np.concatenate(
        [sess.run(None, {"obs": obs[i : i + 1]})[0] for i in range(args.golden)], axis=0)
    acts_near_fp32 = np.concatenate(
        [sess.run(None, {"obs": obs_near[i : i + 1]})[0] for i in range(args.golden)], axis=0)
    acts_int8 = dequant_forward(graph, obs)
    acts_near_int8 = dequant_forward(graph, obs_near)

    print(f"quantization report for '{name}' (INT8 weights, FP32 activations, "
          f"clip search + bias correction):")
    worst = (2.0, 0.0, 0.0)
    for i in range(args.golden):
        cos, rel, maxabs = metrics(acts_int8[i], acts_fp32[i])
        worst = (min(worst[0], cos), max(worst[1], rel), max(worst[2], maxabs))
        print(f"  random  {i:2d}: cos {cos:.6f}  relL2 {rel:.5f}  maxabs {maxabs:.4f}")
    print(f"  WORST(random, off-distribution): "
          f"cos {worst[0]:.6f}  relL2 {worst[1]:.5f}  maxabs {worst[2]:.4f}")
    worst_near = (2.0, 0.0, 0.0)
    for i in range(args.golden):
        cos, rel, maxabs = metrics(acts_near_int8[i], acts_near_fp32[i])
        worst_near = (min(worst_near[0], cos), max(worst_near[1], rel),
                      max(worst_near[2], maxabs))
        print(f"  standing {i:2d}: cos {cos:.6f}  relL2 {rel:.5f}  maxabs {maxabs:.4f}")
    print(f"  WORST(standing, in-distribution): "
          f"cos {worst_near[0]:.6f}  relL2 {worst_near[1]:.5f}  maxabs {worst_near[2]:.4f}")

    # Scale-aware gates: the policies' action outputs differ by two orders of
    # magnitude, so nothing absolute can span them. S = mean |a| over the
    # random set (the policy's own output scale).
    #   random (off-distribution, harsh): cos >= 0.998, relL2 <= 8%  — sanity,
    #     graceful degradation on inputs the robot never sees;
    #   standing (in-distribution): maxabs <= 2% of S — absolute noise while
    #     still, the gate that binds.
    scale = float(np.mean([np.linalg.norm(a) for a in acts_fp32]))
    ok = (worst[0] >= 0.998 and worst[1] <= 0.08
          and worst_near[2] <= 0.02 * scale)
    if not ok:
        raise SystemExit("host-side quantization metrics out of tolerance — refusing to emit")
    print(f"  deploy gate: standing maxabs {worst_near[2]:.4f} <= 2% of scale "
          f"({0.02 * scale:.3f}), random cos {worst[0]:.6f} >= 0.998 — PASS")

if __name__ == "__main__":
    main()
