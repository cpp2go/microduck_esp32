#!/bin/sh
# Host verification only — production inference stays on ESP32-P4 (duck_policy INT8).
# 1) Golden INT8 vs baked FP32 references
# 2) Observation / joint layout vs pollen-robotics/microduck duck-control
# Run from the project root:  sh host_test/run.sh
set -e
cd "$(dirname "$0")/.."

CXX="${CXX:-g++}"

echo "== duck_policy golden =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    components/duck_policy/src/duck_policy.cpp \
    components/duck_policy/src/duck_models.cpp \
    host_test/main.cpp \
    -I components/duck_policy/include \
    -o /tmp/duck_policy_test_cpp
/tmp/duck_policy_test_cpp

echo "== obs layout (original microduck) =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/obs_layout_test.cpp \
    -I common/include \
    -o /tmp/microduck_obs_layout_test
/tmp/microduck_obs_layout_test

echo "== Feetech STS/HD1910 codec =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/ft_sts_codec_test.cpp \
    -o /tmp/microduck_ft_sts_codec_test
/tmp/microduck_ft_sts_codec_test
