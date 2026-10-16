#!/usr/bin/env bash
#
# 主机端 IK 单元测试（不依赖 PROS）。
# 用法：bash scripts/test_native.sh
set -euo pipefail

cd "$(dirname "$0")/.."

CXX="${CXX:-g++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++17 -O2 -Wall -Wextra -Iinclude \
    tests/native/test_kinematics.cpp src/chassis/kinematics.cpp \
    -o "$OUT/test_kinematics"

"$OUT/test_kinematics"
