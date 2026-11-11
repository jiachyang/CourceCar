#!/usr/bin/env bash
#
# 主机端单元测试（不依赖 PROS / V5 硬件）。
# 覆盖：逆运动学、航向控制器、Radio 摇杆映射、测距状态分类、近壁通道、
#       横向 PD、选对、模式机、LaneKeeper、独立安全层、真实迷宫射线、
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

# 5) 单路近壁通道（100/150 分类、MID 初始化、远距有效回波、滤波/丢帧）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_range_channel.cpp src/lane/range_channel.cpp \
    src/lane/lane_config.cpp \
    -o "$OUT/test_range_channel"
"$OUT/test_range_channel"

# 6) 横向 PD（P/D、限幅、D 低通、复位）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_pid.cpp src/lane/pid.cpp \
    -o "$OUT/test_lane_pid"
"$OUT/test_lane_pid"

# 7) 通道选对（前后=A、左右=B、斜向/换组）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_sensor_group_selector.cpp \
    src/lane/sensor_group_selector.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_sensor_group_selector"
"$OUT/test_sensor_group_selector"

# 8) 车道模式机（TRACKING/DEGRADED/TOO_CLOSE/NO_CORRIDOR）
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_state_machine.cpp \
    src/lane/lane_state_machine.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_lane_state_machine"
"$OUT/test_lane_state_machine"

# 9) LaneConfig 默认值与静态校验
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_config.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_lane_config"
"$OUT/test_lane_config"

# 10) LaneKeeper：近壁语义、选对、横向 PD、过近、丢墙缓释
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_lane_keeper.cpp \
    src/modules/lane_keeper.cpp src/lane/pid.cpp src/lane/range_channel.cpp \
    src/lane/sensor_group_selector.cpp \
    src/lane/lane_state_machine.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_lane_keeper"
"$OUT/test_lane_keeper"

# 11) 独立安全层：ToF 制动、航向门控、fail-closed
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_safety_supervisor.cpp \
    src/modules/safety_supervisor.cpp src/safety/safety_config.cpp \
    -o "$OUT/test_safety_supervisor"
"$OUT/test_safety_supervisor"

# 12) 迷宫 fixture 必须与 test/data/maze_model.json 同步
python3 scripts/gen_maze_fixture.py --check

# 13) 用真实迷宫墙体做射线 + 45 cm 走廊模拟
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_maze_fixture.cpp \
    src/modules/lane_keeper.cpp src/lane/pid.cpp src/lane/range_channel.cpp \
    src/lane/sensor_group_selector.cpp \
    src/lane/lane_state_machine.cpp src/lane/lane_config.cpp \
    -o "$OUT/test_maze_fixture"
"$OUT/test_maze_fixture"

# 14) 静止零偏、固定初始航向、延迟/失效/跨零
"$CXX" "${CXXFLAGS[@]}" \
    tests/native/test_startup_heading.cpp src/imu/startup_heading.cpp \
    -o "$OUT/test_startup_heading"
"$OUT/test_startup_heading"

# 15) 遥测字段契约
python3 scripts/check_telemetry_schema.py
