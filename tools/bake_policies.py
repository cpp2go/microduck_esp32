#!/usr/bin/env python3
"""Bake the whole official Microduck policy set into one firmware model file.

For every ONNX in the list: parse, quantize (INT8 weights, clip search + bias
correction), validate against FP32 ONNX-Runtime references on two observation
distributions with scale-aware gates — and only then generate its C++ block.
One failing model aborts the bake; nothing is written partially.

Emits components/duck_policy/src/duck_models.cpp:
  per model — int8 weights, scales, biases, normalizer, golden vectors, and a
  duck::Model instance; plus the duck::kModels registry (alpha_walking first —
  it is the default policy the firmware boots with).

Usage: python3 tools/bake_policies.py --dir /path/to/onnx --out-dir .
"""

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort

from extract_policy import (ARR, parse_graph, quantize_and_calibrate,
                            dequant_forward, metrics, reference_session,
                            c_ints, c_floats)

# Default policy first — the firmware boots with kModels[0].
MODELS = ["alpha_walking", "alpha_stand", "alpha_sitstand", "alpha_ground_pick",
          "ball_kick_left", "ball_kick_right", "roller", "roller_crouch", "roulade"]
GOLDEN_PER_SET = 16


def validate_and_prepare(onnx_path: Path, seed: int = 1234) -> dict:
    graph = parse_graph(onnx_path)
    rng = np.random.default_rng(seed)
    obs = rng.uniform(-3.0, 3.0, size=(GOLDEN_PER_SET, graph["obs_len"])).astype(np.float32)
    obs_near = (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                           size=(GOLDEN_PER_SET, graph["obs_len"]))).astype(np.float32)
    calib = np.concatenate([
        rng.uniform(-3.0, 3.0, size=(32, graph["obs_len"])).astype(np.float32),
        np.zeros((8, graph["obs_len"]), dtype=np.float32),
        (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                    size=(32, graph["obs_len"]))).astype(np.float32),
    ])
    quantize_and_calibrate(graph, calib)

    sess = reference_session(onnx_path)
    ref = np.concatenate([sess.run(None, {"obs": x[i:i+1]})[0] for x in (obs, obs_near)
                          for i in range(GOLDEN_PER_SET)], axis=0)
    got = np.concatenate([dequant_forward(graph, obs), dequant_forward(graph, obs_near)])
    ms = [metrics(got[i], ref[i]) for i in range(2 * GOLDEN_PER_SET)]

    scale = float(np.mean([np.linalg.norm(r) for r in ref[:GOLDEN_PER_SET]]))
    cos_rand = min(m[0] for m in ms[:GOLDEN_PER_SET])
    rel_rand = max(m[1] for m in ms[:GOLDEN_PER_SET])
    abs_rand = max(m[2] for m in ms[:GOLDEN_PER_SET])
    abs_still = max(m[2] for m in ms[GOLDEN_PER_SET:])
    ok = (cos_rand >= 0.998 and rel_rand <= 0.08
          and abs_still <= 0.02 * scale and abs_rand <= 0.02 * scale * 5)
    return {"graph": graph, "ok": ok, "scale": scale, "cos": cos_rand,
            "rel": rel_rand, "abs_still": abs_still,
            "golden_obs": np.concatenate([obs, obs_near]),
            "golden_act": ref}


def c_floats_2d(a: np.ndarray) -> str:
    rows = ["    {" + ", ".join(f"{v:.9g}" for v in row) + "}," for row in a]
    return "\n".join(rows)


def gen_model_block(graph: dict, name: str, idx: int,
                    golden_obs: np.ndarray, golden_act: np.ndarray) -> str:
    lay = graph["layers"]
    defs, ptrs = [], []
    for li, layer in enumerate(lay):
        wq, scale, b = layer["q"], layer["scale"], layer["b"]
        defs.append(f"static const {ARR} int8_t w{idx}_{li}[{wq.size}] = {{\n{c_ints(wq)}\n}};")
        defs.append(f"static const {ARR} float s{idx}_{li}[{scale.size}] = {{\n{c_floats(scale)}\n}};")
        defs.append(f"static const {ARR} float b{idx}_{li}[{b.size}] = {{\n{c_floats(b)}\n}};")
        ptrs.append(f"      {{ .in = {wq.shape[1]}, .out = {wq.shape[0]}, "
                    f".wq = w{idx}_{li}, .wscale = s{idx}_{li}, .b = b{idx}_{li}, "
                    f".elu = {'true' if layer['elu'] else 'false'} }},")

    count = golden_obs.shape[0]
    obs_len, act_len = golden_obs.shape[1], golden_act.shape[1]
    return f"""\
// ------------------------------ {name} ------------------------------
{chr(10).join(defs)}

static const {ARR} float norm_mean_{idx}[{graph['obs_len']}] = {{
{c_floats(graph['mean'])}
}};

static const {ARR} float norm_std_{idx}[{graph['obs_len']}] = {{
{c_floats(graph['std'])}
}};

// FP32 ONNX-Runtime references: first half random, second half standing.
static const float golden_obs_{idx}[{count}][{obs_len}] = {{
{c_floats_2d(golden_obs)}
}};

static const float golden_act_{idx}[{count}][{act_len}] = {{
{c_floats_2d(golden_act)}
}};

static const duck::Model m{idx} = {{
    .name        = "{name}",
    .obs_len     = {graph['obs_len']},
    .act_len     = {graph['act_len']},
    .n_layers    = {len(lay)},
    .norm_mean   = norm_mean_{idx},
    .norm_std    = norm_std_{idx},
    .layers      = {{
{chr(10).join(ptrs)}
    }},
    .golden_count = {count},
    .golden_obs   = &golden_obs_{idx}[0][0],
    .golden_act   = &golden_act_{idx}[0][0],
}};
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default="/tmp/duckesp", type=Path,
                    help="directory holding <name>.onnx files")
    ap.add_argument("--models", nargs="*", default=MODELS)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--out-dir", type=Path, default=Path("."))
    args = ap.parse_args()

    blocks, table, instances = [], [], []
    failures = 0
    for idx, name in enumerate(args.models):
        path = args.dir / f"{name}.onnx"
        r = validate_and_prepare(path, args.seed)
        verdict = "PASS" if r["ok"] else "FAIL"
        table.append(f"{name:<18} scale {r['scale']:>8.2f}  cos {r['cos']:.6f}  "
                     f"relL2 {r['rel']:.4f}  abs(stand) {r['abs_still']:.4f}  {verdict}")
        print(table[-1])
        if not r["ok"]:
            failures += 1
            continue
        blocks.append(gen_model_block(r["graph"], name, idx,
                                      r["golden_obs"], r["golden_act"]))
        instances.append(f"&m{idx},")

    if failures:
        raise SystemExit(f"{failures} model(s) failed the quantization gates — "
                         f"nothing written")

    unit = f"""\
// GENERATED by tools/bake_policies.py — the baked official policy set.
// INT8 weights (per-channel symmetric), FP32 scales/biases/normalizer, and
// per-model FP32 golden vectors for the on-device accuracy gate. Do not edit.
#include "duck_policy.hpp"

{chr(10).join(blocks)}

namespace duck {{

const Model *const kModels[] = {{
{chr(10).join(instances)}
}};

const int kModelCount = {len(instances)};

}} // namespace duck
"""
    out = args.out_dir / "components/duck_policy/src/duck_models.cpp"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(unit)
    print(f"wrote {out} ({len(instances)} policies, default: {args.models[0]})")


if __name__ == "__main__":
    main()
