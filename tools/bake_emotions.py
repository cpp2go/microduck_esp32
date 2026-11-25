#!/usr/bin/env python3
"""Bake pollen-robotics/microduck robotd/emotions/*.json into emotion_data.c/h."""
from __future__ import annotations

import argparse
import json
import pathlib


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=pathlib.Path, required=True, help="robotd/emotions dir")
    ap.add_argument("--out-h", type=pathlib.Path, required=True)
    ap.add_argument("--out-c", type=pathlib.Path, required=True)
    args = ap.parse_args()

    emotions = []
    for path in sorted(args.src.glob("*.json")):
        data = json.loads(path.read_text())
        rows = []
        for k in data["keyframes"]:
            rows.append(
                (
                    float(k.get("t", 0)),
                    float(k.get("neck", 0)),
                    float(k.get("head_pitch", 0)),
                    float(k.get("head_yaw", 0)),
                    float(k.get("head_roll", 0)),
                    float(k.get("body_pitch", 0)),
                    float(k.get("mouth", 0) or 0),
                )
            )
        emotions.append((path.stem, rows))

    args.out_h.parent.mkdir(parents=True, exist_ok=True)
    args.out_h.write_text(
        """// SPDX-License-Identifier: MIT
// Auto-generated from pollen-robotics/microduck robotd/emotions/*.json — do not edit.
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float t;
    float neck;
    float head_pitch;
    float head_yaw;
    float head_roll;
    float body_pitch;
    float mouth;
} emotion_keyframe_t;
typedef struct {
    const char *name;
    const emotion_keyframe_t *keys;
    size_t count;
} emotion_clip_t;
extern const emotion_clip_t kEmotionClips[];
extern const size_t kEmotionClipCount;
const emotion_clip_t *emotion_find(const char *name);
#ifdef __cplusplus
}
#endif
"""
    )

    lines = [
        "// SPDX-License-Identifier: MIT",
        "// Auto-generated — do not edit.",
        '#include "emotion_data.h"',
        "#include <string.h>",
        "",
    ]
    for name, rows in emotions:
        lines.append(f"static const emotion_keyframe_t k_{name}[] = {{")
        for r in rows:
            lines.append(
                "    {%.3ff, %.3ff, %.3ff, %.3ff, %.3ff, %.3ff, %.3ff}," % r
            )
        lines.append("};")
        lines.append("")
    lines.append("const emotion_clip_t kEmotionClips[] = {")
    for name, rows in emotions:
        lines.append(f'    {{"{name}", k_{name}, {len(rows)}}},')
    lines.append("};")
    lines.append(f"const size_t kEmotionClipCount = {len(emotions)};")
    lines.append(
        """
const emotion_clip_t *emotion_find(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < kEmotionClipCount; ++i) {
        if (strcmp(kEmotionClips[i].name, name) == 0) return &kEmotionClips[i];
    }
    return NULL;
}
"""
    )
    args.out_c.write_text("\n".join(lines))
    print(f"baked {len(emotions)} emotions -> {args.out_c}")


if __name__ == "__main__":
    main()
