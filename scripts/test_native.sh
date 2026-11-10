#!/usr/bin/env bash
#
# 主机端单元测试（不依赖 PROS / V5 硬件）。
# 覆盖：逆运动学、航向控制器、Radio 摇杆映射、测距状态分类、10/15 cm 墙体
#       状态机、测距滤波、组选择、状态机、墙距 PID、LaneKeeper、真实迷宫射线、
#       遥测字段契约。
# 用法：bash scripts/test_native.sh
set -euo pipefail

cd "$(dirname "$0")/.."

CXX="${CXX:-g++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

# -UNDEBUG：确保 assert 永远生效，避免外部环境定义 NDEBUG 后测试变成空跑。
CXXFLAGS=(-std=c++17 -O2 -Wall -Wextra -UNDEBUG -Iinclude)

# 1) 逆运动学（纯数学）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_kinematics.cpp src/chassis/kinematics.cpp \
    -o "$OUT/test_kinematics"
"$OUT/test_kinematics"

# 2) 航向控制器（纯逻辑）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_heading_controller.cpp src/chassis/heading_controller.cpp \
    -o "$OUT/test_heading_controller"
"$OUT/test_heading_controller"

# 3) Radio 摇杆映射（纯函数）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_radio_mapping.cpp src/radio/input_mapping.cpp \
    -o "$OUT/test_radio_mapping"
"$OUT/test_radio_mapping"

# 4) 测距读数状态分类（PROS_ERR / 9999 / 过近 / 超量程）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_distance_status.cpp \
    -o "$OUT/test_distance_status"
"$OUT/test_distance_status"

# 5) 10/15 cm 滞回墙体状态机（严格 <100 / >150、无效样本、重复采样）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_wall_tracker.cpp src/lane/wall_state.cpp \
    -o "$OUT/test_wall_tracker"
"$OUT/test_wall_tracker"

# 6) 测距滤波（中值、低通、跳变确认、无效值不混入）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_range_filter.cpp src/lane/range_filter.cpp \
    -o "$OUT/test_range_filter"
"$OUT/test_range_filter"

# 7) 测距组选择（前后=A、左右=B、斜向/零速/转弯）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_sensor_group_selector.cpp \
    src/lane/sensor_group_selector.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_sensor_group_selector"
"$OUT/test_sensor_group_selector"

# 8) 车道状态机（ACQUIRING/TURNING/FAULT、积分权限、速度比例）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_state_machine.cpp \
    src/lane/lane_state_machine.cpp src/lane/wall_state.cpp \
    src/lane/lane_config.cpp \
    -o "$OUT/test_lane_state_machine"
"$OUT/test_lane_state_machine"

# 9) 墙距 PID（P/I/D、积分门控、限幅、抗饱和、D 低通、复位）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_pid.cpp src/lane/pid.cpp \
    -o "$OUT/test_lane_pid"
"$OUT/test_lane_pid"

# 10) LaneConfig 默认值与静态校验
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_config.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_lane_config"
"$OUT/test_lane_config"

# 11) LaneKeeper：组选择、双墙/单墙、状态机、滤波、限速、安全透传
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_keeper.cpp \
    src/modules/lane_keeper.cpp src/lane/pid.cpp src/lane/wall_state.cpp \
    src/lane/range_filter.cpp src/lane/sensor_group_selector.cpp \
    src/lane/lane_state_machine.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_lane_keeper"
"$OUT/test_lane_keeper"

# 12) 迷宫 fixture 必须与 test/data/maze_model.json 同步
python3 scripts/gen_maze_fixture.py --check

# 13) 用真实迷宫墙体做射线 + 45 cm 走廊模拟
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_maze_fixture.cpp \
    src/modules/lane_keeper.cpp src/lane/pid.cpp src/lane/wall_state.cpp \
    src/lane/range_filter.cpp src/lane/sensor_group_selector.cpp \
    src/lane/lane_state_machine.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_maze_fixture"
"$OUT/test_maze_fixture"

# 14) 遥测字段契约：固件 DIAG_TUNE_FIELDS ↔ test/pid_tune.py 的 FIELDS
python3 scripts/check_telemetry_schema.py
