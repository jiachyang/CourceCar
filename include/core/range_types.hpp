/**
 * @file core/range_types.hpp
 * @brief 跨模块的测距快照契约（DistanceArray -> 任意消费者）。
 *
 * 设计原因（ARCH：模块之间只走 core 契约）：
 *   - LaneKeeper 需要四路测距，但**不允许** include modules/distance_array.hpp；
 *   - 因此 DistanceArray 把内部状态投影成本文件的 `RangeSnapshot`，
 *     main 转发给 LaneKeeper。换一个测距实现时，只要仍产出该快照即可。
 *
 * 约定：
 *   - 索引顺序固定为 前、后、左、右（与 DistanceArray 的 SideIndex 一致）；
 *   - 距离单位 mm；本文件只做“状态分类”，不包含量程常量（量程属于 distance 模块）。
 *   - 本文件只依赖标准库，可主机端编译。
 */
#pragma once

#include <cstdint>

namespace core {

/// 单次测距结果的状态。区分“读数失败”和“确实没有目标”是本系统的关键。
enum class RangeStatus : std::uint8_t {
    kNotInstalled = 0,  ///< 端口没有识别到 Distance Sensor（不是“无墙”）
    kReadError,         ///< PROS_ERR / 设备调用失败（不是“无墙”）
    kTooClose,          ///< 原始距离 < 量程下限（有物体但距离不可信）
    kTooFar,            ///< 原始距离 > 量程上限（远到不能用于近墙判断）
    kNoTarget,          ///< 原始值 9999：传感器工作正常，但量程内无目标
    kOk,                ///< 20..2000 mm 的可信测距
};

/// 四路测距在工程内的统一索引顺序。
enum RangeSide : std::uint8_t {
    kRangeFront = 0,
    kRangeRear = 1,
    kRangeLeft = 2,
    kRangeRight = 3,
    kRangeSideCount = 4,
};

/// 单路测距快照。
struct RangeSample {
    std::int32_t distanceMm = -1;                         ///< 原始距离 [mm]；-1 = 无读数
    std::int32_t confidence = -1;                         ///< 0..63；-1 = 无读数（仅诊断，不用于门控）
    RangeStatus status = RangeStatus::kNotInstalled;      ///< 有效性分类
    std::uint32_t timestampMs = 0;                        ///< 本路采样时刻 [ms]
};

/// 四路测距快照。
struct RangeSnapshot {
    RangeSample samples[kRangeSideCount]{};  ///< 前、后、左、右
    std::uint32_t timestampMs = 0;           ///< 本次快照时刻 [ms]
};

}  // namespace core
