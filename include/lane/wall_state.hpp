/**
 * @file lane/wall_state.hpp
 * @brief 单路 ToF 的 10/15 cm 滞回墙体状态机（纯逻辑）。
 *
 * 已确认参数（不得改成 5 cm）：
 *   WALL_ENTER_MM   = 100  -> 有效距离严格小于 100 mm 才累计“有墙”
 *   WALL_EXIT_MM    = 150  -> 有效距离严格大于 150 mm 才累计“无墙”
 *   WALL_CONFIRM_COUNT = 3 -> 连续有效确认样本数
 *
 * 关键安全语义（按完整任务书 §5.4）：
 *   - PROS_ERR / 9999 / <20 mm / >2000 mm / 未安装都属于**无效样本**；
 *   - 无效样本不参与进入/离开确认，**不直接等价于无墙**；
 *   - 100..150 mm（含端点）是滞回带，保持原状态并打断连续序列；
 *   - 连续无效超过 staleTimeoutMs 时状态转 Unknown（不是 Absent）；
 *   - 同一时刻/过短间隔的重复样本不重复计数，避免一个控制周期凑满 3 次。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "lane/lane_config.hpp"

namespace lane {

/// 单路墙体存在性。
enum class WallStatus : std::uint8_t {
    kUnknown = 0,  ///< 尚未确认 / 长时间丢数（不等于“没有墙”）
    kPresent,      ///< 已确认有墙
    kAbsent,       ///< 已确认无墙（有效远距样本）
};

/**
 * @brief 单路墙体跟踪器。
 *
 * 一个 ToF 一个实例，互不影响。所有计数只在“连续有效”样本上累计；
 * 任何无效样本都会打断连续性，但不会把 Present 直接改成 Absent。
 */
class WallTracker {
public:
    void init(const WallConfig& config);
    void reset();

    /// 用一条测距样本推进状态机。nowMs 为本次更新时刻（单调递增）。
    void update(const core::RangeSample& sample, std::uint32_t nowMs);

    WallStatus status() const { return status_; }
    bool present() const { return status_ == WallStatus::kPresent; }

    /// 最近一次有效距离是否足够新，可用于控制（不参与存在性确认）。
    bool usable(std::uint32_t nowMs, std::uint32_t holdMs) const;

    bool hasDistance() const { return hasDistance_; }
    std::int32_t lastDistanceMm() const { return lastDistanceMm_; }
    std::uint32_t lastValidMs() const { return lastValidMs_; }
    std::uint32_t lastEvidenceMs() const { return lastEvidenceMs_; }
    std::uint32_t lastUpdateMs() const { return lastUpdateMs_; }

private:
    WallConfig config_{};
    WallStatus status_ = WallStatus::kUnknown;

    std::int32_t nearCount_ = 0;
    std::int32_t farCount_ = 0;

    std::int32_t lastDistanceMm_ = -1;
    std::uint32_t lastValidMs_ = 0;
    std::uint32_t lastEvidenceMs_ = 0;
    std::uint32_t lastUpdateMs_ = 0;
    bool hasDistance_ = false;
    bool hasUpdate_ = false;
};

}  // namespace lane
