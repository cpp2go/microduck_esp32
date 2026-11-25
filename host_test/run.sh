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

echo "== Unitree S288 codec =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    components/servo_bus/ut_s288_codec.c \
    host_test/ut_s288_codec_test.cpp \
    -I components/servo_bus/include \
    -o /tmp/microduck_ut_s288_codec_test
/tmp/microduck_ut_s288_codec_test

echo "== ToF obstacle gate =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    components/tof/tof_obstacle.c \
    host_test/tof_obstacle_test.cpp \
    -I components/tof/include \
    -o /tmp/microduck_tof_obstacle_test
/tmp/microduck_tof_obstacle_test

echo "== Voice band matcher =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    components/voice/voice_match.c \
    host_test/voice_match_test.cpp \
    -I components/voice/include \
    -lm \
    -o /tmp/microduck_voice_match_test
/tmp/microduck_voice_match_test

echo "== Safety / fall predictor =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/safety_fall_test.cpp \
    -I components/safety/include \
    -lm \
    -o /tmp/microduck_safety_fall_test
/tmp/microduck_safety_fall_test

echo "== Skills priority mirror =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/skills_test.cpp \
    -I components/skills/include \
    -lm \
    -o /tmp/microduck_skills_test
/tmp/microduck_skills_test

echo "== Loop validate helper =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/loop_validate_test.cpp \
    -I components/control/include \
    -lm \
    -o /tmp/microduck_loop_validate_test
/tmp/microduck_loop_validate_test

echo "== Posture classify =="
"$CXX" -std=c++20 -O2 -Wall -Wextra \
    host_test/posture_test.cpp \
    -I components/posture/include \
    -lm \
    -o /tmp/microduck_posture_test
/tmp/microduck_posture_test
