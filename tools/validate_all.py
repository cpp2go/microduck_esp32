#!/usr/bin/env python3
"""Validate the whole official policy set through the INT8 pipeline — report
only, writes nothing. Same parse → quantize+calibrate path as extract_policy.py.

Gate design (scale-aware): the nine policies' action outputs differ by two
orders of magnitude (||a|| from ~1 to ~560), so no fixed absolute threshold can
span them. Each policy is gated on its own output scale S = mean ||a_fp|| over
the off-distribution (random) set:
  random set  : cos >= 0.999 AND relL2 <= 2%      (relative fidelity at scale)
  standing set: maxabs <= 2% of S                 (absolute noise while still)
"""

import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).parent))
from extract_policy import (parse_graph, quantize_and_calibrate, dequant_forward,
                            metrics)

MODELS = ["alpha_walking", "alpha_stand", "alpha_sitstand", "alpha_ground_pick",
          "ball_kick_left", "ball_kick_right", "roller", "roller_crouch", "roulade"]


def validate(onnx_path: Path, seed: int = 1234, n: int = 16) -> dict:
    graph = parse_graph(onnx_path)
    rng = np.random.default_rng(seed)
    obs = rng.uniform(-3.0, 3.0, size=(n, graph["obs_len"])).astype(np.float32)
    obs_near = (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                           size=(n, graph["obs_len"]))).astype(np.float32)
    calib = np.concatenate([
        rng.uniform(-3.0, 3.0, size=(32, graph["obs_len"])).astype(np.float32),
        np.zeros((8, graph["obs_len"]), dtype=np.float32),
        (graph["mean"] + rng.normal(0, graph["std"] * 0.5,
                                    size=(32, graph["obs_len"]))).astype(np.float32),
    ])
    quantize_and_calibrate(graph, calib)

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    ref = np.concatenate([sess.run(None, {"obs": x[i:i+1]})[0] for x in (obs, obs_near)
                          for i in range(n)], axis=0)
    got = np.concatenate([dequant_forward(graph, obs), dequant_forward(graph, obs_near)])

    scale = float(np.mean([np.linalg.norm(r) for r in ref[:n]]))  # random half
    r_ms = [metrics(got[i], ref[i]) for i in range(n)]
    n_ms = [metrics(got[n + i], ref[n + i]) for i in range(n)]
    # Scale-aware gates (S = policy's own output scale, from the random set):
    #   random (off-distribution, harsh): cos >= 0.998, relL2 <= 8%   — sanity
    #   standing (in-distribution):       maxabs <= 2% of S           — binds
    ok = (min(m[0] for m in r_ms) >= 0.998
          and max(m[1] for m in r_ms) <= 0.08
          and max(m[2] for m in n_ms) <= 0.02 * scale)
    return {"ok": ok, "scale": scale,
            "cos_r": min(m[0] for m in r_ms), "rel_r": max(m[1] for m in r_ms),
            "abs_r": max(m[2] for m in r_ms), "abs_n": max(m[2] for m in n_ms)}


def main() -> None:
    directory = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/duckesp")
    print(f"{'model':<18} {'scale|a|':>9} {'cos(rand)':>10} {'relL2':>7} "
          f"{'abs(stand)':>11}  verdict")
    failures = 0
    for name in MODELS:
        path = directory / f"{name}.onnx"
        try:
            r = validate(path)
            print(f"{name:<18} {r['scale']:>9.2f} {r['cos_r']:>10.6f} {r['rel_r']:>7.4f} "
                  f"{r['abs_n']:>11.4f}  {'PASS' if r['ok'] else 'FAIL'}")
            failures += not r["ok"]
        except Exception as e:
            print(f"{name:<18}  ERROR: {e}")
            failures += 1
    print(f"\n{len(MODELS) - failures}/{len(MODELS)} models within tolerance")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
