#!/usr/bin/env bash
#
# 构建并运行车道保持闭环仿真（主机端，不依赖 PROS）。
# 用法：bash sim/run_lane_keeping.sh [--trace] [--sigma=mm]
set -euo pipefail

cd "$(dirname "$0")/.."

CXX="${CXX:-g++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++17 -O2 -Wall -Wextra -UNDEBUG \
    -Iinclude -Itests/native \
    sim/lane_keeping_sim.cpp \
    src/modules/lane_keeper.cpp src/lane/pid.cpp src/lane/range_channel.cpp \
    src/lane/sensor_group_selector.cpp src/lane/lane_state_machine.cpp \
    src/lane/lane_config.cpp src/chassis/kinematics.cpp \
    -o "$OUT/lane_keeping_sim"

"$OUT/lane_keeping_sim" "$@"
