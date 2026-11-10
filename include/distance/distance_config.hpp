/**
 * @file distance/distance_config.hpp
 * @brief 四路 V5 Distance Sensor（激光测距）接线配置。
 *
 * 本文件只登记硬件真值（端口/量程）；读取由 DistanceArray 负责，
 * 近墙判断与横向纠偏由 LaneKeeper 负责。
 * 完整接线表见 docs/车辆配置.md，车道布局见 docs/lane_keeping.md。
 */
#pragma once

#include <cstdint>

namespace distance {

/// 测距传感器安装方位。全工程统一此顺序：前、后、左、右。
enum SideIndex { kFront = 0, kRear = 1, kLeft = 2, kRight = 3, kSideCount = 4 };

// Smart Port 接线（rev.5）：
//   7 = 正前方，8 = 正后方，9 = 左侧，10 = 右侧
inline constexpr std::int8_t kPortFront = 7;
inline constexpr std::int8_t kPortRear  = 8;
inline constexpr std::int8_t kPortLeft  = 9;
inline constexpr std::int8_t kPortRight = 10;

/// V5 Distance Sensor 有效量程 [mm]（见 docs/测距仪数据-P2.md）。
/// 低于 20 mm 会返回小值而不是 9999，必须显式判为无效。
inline constexpr std::int32_t kRangeMinMm = 20;
inline constexpr std::int32_t kRangeMaxMm = 2000;
/// 无目标时的原始读数。
inline constexpr std::int32_t kNoTargetMm = 9999;

/// 测距模块配置（端口顺序：前、后、左、右）。
struct DistanceConfig {
    std::int8_t ports[kSideCount] = {kPortFront, kPortRear, kPortLeft, kPortRight};
};

/// 默认测距配置（当前真值）。
inline DistanceConfig makeDefaultDistanceConfig() { return DistanceConfig{}; }

}  // namespace distance
