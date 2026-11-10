/**
 * @file tests/native/test_distance_status.cpp
 * @brief Distance Sensor 原始读数分类测试（PROS_ERR / 9999 / 过近 / 超量程）。
 */
#include <cassert>
#include <cstdio>

#include "distance/range_status.hpp"

int main() {
    using core::RangeStatus;
    using distance::classifyDistanceReading;

    assert(classifyDistanceReading(100, false) == RangeStatus::kNotInstalled);
    assert(classifyDistanceReading(-1, true) == RangeStatus::kReadError);
    assert(classifyDistanceReading(9999, true) == RangeStatus::kNoTarget);
    assert(classifyDistanceReading(0, true) == RangeStatus::kTooClose);
    assert(classifyDistanceReading(19, true) == RangeStatus::kTooClose);
    assert(classifyDistanceReading(20, true) == RangeStatus::kOk);
    assert(classifyDistanceReading(100, true) == RangeStatus::kOk);
    assert(classifyDistanceReading(2000, true) == RangeStatus::kOk);
    assert(classifyDistanceReading(2001, true) == RangeStatus::kTooFar);
    assert(classifyDistanceReading(5000, true) == RangeStatus::kTooFar);

    // 未安装优先于数值：不能把端口没插误判成“读取失败”或“无目标”。
    assert(classifyDistanceReading(9999, false) == RangeStatus::kNotInstalled);
    assert(classifyDistanceReading(-1, false) == RangeStatus::kNotInstalled);

    std::printf("distance status tests: ALL PASS\n");
    return 0;
}
