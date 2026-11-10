/**
 * @file distance/range_status.hpp
 * @brief 把 V5 Distance Sensor 的原始读数分类为 core::RangeStatus。
 *
 * 为什么单独成纯函数：
 *   - 有效/无效判据是安全相关的纯逻辑，必须能在主机端单测；
 *   - `PROS_ERR`、9999、<20 mm、>2000 mm 必须区分处理：
 *     “读取失败”与“无目标”不能混为一谈，否则会把传感器故障误判成没有墙。
 *
 * 本文件不依赖 PROS，只依赖 `distance/distance_config.hpp` 的纯常量。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "distance/distance_config.hpp"

namespace distance {

/**
 * @brief 分类一条 Distance Sensor 读数。
 *
 * @param distanceMm 已把 PROS_ERR 归一到 -1 的原始距离 [mm]
 * @param installed  端口上是否识别到 Distance Sensor
 *
 * 注意：9999 在数值上大于 2000，必须先于“超量程”判断；
 * 否则“无目标”会被误写成“超量程”，虽然两者对近墙判断都算远，
 * 但诊断上必须可区分。
 */
inline core::RangeStatus classifyDistanceReading(std::int32_t distanceMm,
                                                 bool installed) {
    if (!installed) {
        return core::RangeStatus::kNotInstalled;
    }
    if (distanceMm < 0) {
        return core::RangeStatus::kReadError;
    }
    if (distanceMm == kNoTargetMm) {
        return core::RangeStatus::kNoTarget;
    }
    if (distanceMm < kRangeMinMm) {
        return core::RangeStatus::kTooClose;
    }
    if (distanceMm > kRangeMaxMm) {
        return core::RangeStatus::kTooFar;
    }
    return core::RangeStatus::kOk;
}

}  // namespace distance
