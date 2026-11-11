#!/usr/bin/env bash
#
# 架构规则静态检查：docs/development_rules.md 的可执行部分。
# 用法：bash scripts/check_rules.sh
set -uo pipefail
cd "$(dirname "$0")/.."

fail=0

fail_msg() {
    echo "RULE-FAIL: $1"
    fail=1
}

dump() {
    sed 's/^/    /'
}

echo "== 分层与依赖 =="

# R1.1 模块头文件不得互相包含
cross=0
for h in include/modules/*.hpp; do
    own=$(basename "$h")
    m=$(grep -Hn '#include "modules/' "$h" | grep -v "modules/$own" || true)
    if [ -n "$m" ]; then
        fail_msg "模块头文件互相包含（跨模块只允许 core 契约）"
        printf '%s\n' "$m" | dump
        cross=1
    fi
done
if [ "$cross" -eq 0 ]; then echo "PASS: 模块头文件无互相包含"; fi

# R1.2 模块内部头不得依赖 modules/
m=$(grep -rn '#include "modules/' include/chassis include/imu include/radio include/distance include/lane include/core || true)
if [ -n "$m" ]; then
    fail_msg "模块内部头文件包含了 modules/（依赖方向错误）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: 模块内部头文件不依赖 modules/"
fi

# R1.3 core 只能依赖标准库
m=$(grep -rn '#include "' include/core || true)
if [ -n "$m" ]; then
    fail_msg "include/core 只能依赖标准库（不得包含工程头文件）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: core 只依赖标准库"
fi

# R1.5 应用层不得被模块依赖
m=$(grep -rn '#include "\(diag\|bringup\)/' include/modules include/chassis include/imu include/radio include/distance include/lane src/modules src/chassis src/radio src/lane || true)
if [ -n "$m" ]; then
    fail_msg "模块依赖了应用层（diag/bringup 只能单向依赖模块）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: 模块不依赖 diag/bringup"
fi

echo "== main 的边界 =="

# R2.1 规模上限
max_main_lines=200
main_lines=$(wc -l < src/main.cpp)
if [ "$main_lines" -gt "$max_main_lines" ]; then
    fail_msg "src/main.cpp 超过 $max_main_lines 行（当前 $main_lines）：把功能搬进模块或 diag/bringup"
else
    echo "PASS: main.cpp 规模 $main_lines/$max_main_lines 行"
fi

# R2.2 不得直接使用硬件 API（时间/任务 API 允许）
m=$(grep -nE 'pros::(Device|Motor|MotorGroup|Imu|Controller|Distance|Optical|Vision|Gps|ADIPort|Serial|Link|adi)' src/main.cpp || true)
if [ -n "$m" ]; then
    fail_msg "src/main.cpp 直接使用硬件 API（应移入对应模块）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: main 不含硬件 API"
fi

# R2.3 不得包含遥测帧格式
m=$(grep -nE '"\$(P|D|S)' src/main.cpp || true)
if [ -n "$m" ]; then
    fail_msg "src/main.cpp 出现遥测帧格式串（应放 src/diag/telemetry.cpp）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: main 不含遥测格式串"
fi

# R2.4 不得做设备探测
m=$(grep -n 'get_plugged_type' src/main.cpp || true)
if [ -n "$m" ]; then
    fail_msg "src/main.cpp 做设备探测（应放 diag/telemetry.cpp）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: main 不做设备探测"
fi

echo "== 遥测与协议 =="

# R5.1 $P/$D/$S 帧只能在 diag 中定义
m=$(grep -rnE '"\$(P|D|S)' src --include='*.cpp' | grep -v '^src/diag/' || true)
if [ -n "$m" ]; then
    fail_msg "遥测帧格式出现在 diag 之外"
    printf '%s\n' "$m" | dump
else
    echo "PASS: 遥测帧格式只在 src/diag"
fi

# R5.2 schema 检查必须接在 test_native 上
if grep -q 'check_telemetry_schema.py' scripts/test_native.sh; then
    echo "PASS: 遥测字段契约检查已接入 test_native.sh"
else
    fail_msg "scripts/test_native.sh 未调用 check_telemetry_schema.py"
fi

echo "== 纯逻辑与平台解耦 =="

pure_files="include/bringup/heading_reference_config.hpp include/imu/startup_heading.hpp src/imu/startup_heading.cpp src/chassis/kinematics.cpp src/chassis/heading_controller.cpp src/radio/input_mapping.cpp include/core/types.hpp include/core/math_util.hpp include/core/range_types.hpp include/chassis/chassis_config.hpp include/chassis/kinematics.hpp include/chassis/heading_controller.hpp include/imu/imu_config.hpp include/distance/distance_config.hpp include/distance/range_status.hpp include/radio/input_mapping.hpp include/lane/*.hpp include/modules/lane_keeper.hpp src/lane/*.cpp src/modules/lane_keeper.cpp include/safety/*.hpp include/modules/safety_supervisor.hpp src/safety/*.cpp src/modules/safety_supervisor.cpp"
m=$(grep -nE 'pros::|#include "pros/' $pure_files | grep -vE '^[^:]+:[0-9]+:[[:space:]]*(\*|//)' || true)
if [ -n "$m" ]; then
    fail_msg "纯逻辑文件引入了 PROS（将无法主机端单测）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: 纯逻辑文件不依赖 PROS"
fi

echo "== 构建约束 =="

# R6.1 不得用 -Wno-* 压制项目警告
m=$(grep -n 'Wno-' Makefile || true)
if [ -n "$m" ]; then
    fail_msg "Makefile 中出现 -Wno-*（应修代码而不是压警告）"
    printf '%s\n' "$m" | dump
else
    echo "PASS: Makefile 未压制警告"
fi

if [ "$fail" -ne 0 ]; then
    echo
    echo "规则检查失败：见 docs/development_rules.md"
    exit 1
fi

echo
echo "layering rule checks: ALL PASS"
