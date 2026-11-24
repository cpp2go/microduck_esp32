#!/bin/sh
# Verify the generated C++ model against ONNX Runtime reference outputs.
# Run from the project root:  sh host_test/run.sh
set -e
cd "$(dirname "$0")/.."
c++ -std=c++20 -O2 -Wall -Wextra \
    components/duck_policy/src/duck_policy.cpp \
    components/duck_policy/src/duck_models.cpp \
    host_test/main.cpp \
    -I components/duck_policy/include \
    -o /tmp/duck_policy_test_cpp
/tmp/duck_policy_test_cpp
