#!/usr/bin/env python3
"""Bake robotd/emotions/*.wav into a 16 kHz mono PCM C bank (truncated)."""
from __future__ import annotations

import argparse
import array
import pathlib
import wave


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=pathlib.Path, required=True)
    ap.add_argument("--out-h", type=pathlib.Path, required=True)
    ap.add_argument("--out-c", type=pathlib.Path, required=True)
    ap.add_argument("--max-s", type=float, default=0.9)
    ap.add_argument("--sr", type=int, default=16000)
    args = ap.parse_args()

    entries = []
    for path in sorted(args.src.glob("*.wav")):
        with wave.open(str(path), "rb") as w:
            nch, sw, sr, nframes, *_ = w.getparams()
            raw = w.readframes(nframes)
        if sw != 2:
            raise SystemExit(f"bad sample width in {path}")
        samples = array.array("h")
        samples.frombytes(raw)
        mono = list(samples[::nch]) if nch > 1 else list(samples)
        if sr != args.sr:
            ratio = sr / args.sr
            out = []
            i = 0.0
            while int(i) < len(mono):
                out.append(mono[int(i)])
                i += ratio
            mono = out
        mono = mono[: int(args.sr * args.max_s)]
        fade = int(args.sr * 0.005)
        for i in range(fade):
            idx = len(mono) - fade + i
            if 0 <= idx < len(mono):
                mono[idx] = int(mono[idx] * (fade - i) / fade)
        entries.append((path.stem, mono))

    args.out_h.write_text(
        """// SPDX-License-Identifier: MIT
// PCM sound bank from pollen-robotics/microduck robotd/emotions/*.wav (16 kHz mono s16).
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    const char *tag;
    const int16_t *pcm;
    size_t samples;
} sound_bank_clip_t;
extern const sound_bank_clip_t kSoundBank[];
extern const size_t kSoundBankCount;
const sound_bank_clip_t *sound_bank_find(const char *tag);
#ifdef __cplusplus
}
#endif
"""
    )

    lines = [
        "// SPDX-License-Identifier: MIT",
        "// Auto-generated PCM bank — do not edit.",
        '#include "sound_bank.h"',
        "#include <string.h>",
        "",
    ]
    for name, mono in entries:
        lines.append(f"static const int16_t k_pcm_{name}[{len(mono)}] = {{")
        for i in range(0, len(mono), 12):
            chunk = mono[i : i + 12]
            lines.append("    " + ", ".join(str(x) for x in chunk) + ",")
        lines.append("};")
        lines.append("")
    lines.append("const sound_bank_clip_t kSoundBank[] = {")
    for name, mono in entries:
        lines.append(f'    {{"{name}", k_pcm_{name}, {len(mono)}}},')
    lines.append("};")
    lines.append(f"const size_t kSoundBankCount = {len(entries)};")
    lines.append(
        """
const sound_bank_clip_t *sound_bank_find(const char *tag)
{
    if (!tag) return NULL;
    for (size_t i = 0; i < kSoundBankCount; ++i) {
        if (strcmp(kSoundBank[i].tag, tag) == 0) return &kSoundBank[i];
    }
    return NULL;
}
"""
    )
    args.out_c.parent.mkdir(parents=True, exist_ok=True)
    args.out_c.write_text("\n".join(lines))
    print(f"baked {len(entries)} clips -> {args.out_c}")


if __name__ == "__main__":
    main()
