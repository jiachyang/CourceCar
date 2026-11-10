/**
 * @file lane/range_filter.hpp
 * @brief 单路测距的 3 点中值 + 一阶低通 + 跳变确认 + 无距离超时复位。
 *
 * 设计目的（完整任务书 §12）：
 *   - 墙体存在性仍用原始有效样本判定，滤波只服务 PID；
 *   - 不允许把 9999/无效值混入中值或均值；
 *   - 单点孤立异常被中值抑制，连续跳变才接受为新稳态；
 *   - 低通系数可配置；D 项自身还有一阶低通（在 Pid 内）。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "lane/lane_config.hpp"

namespace lane {

class RangeFilter {
public:
    void init(const RangeFilterConfig& config);
    void reset();

    /// 用一条原始样本推进滤波。仅 kOk 样本进入中值/低通。
    void update(const core::RangeSample& sample, std::uint32_t nowMs);

    bool hasFiltered() const { return hasFiltered_; }
    bool valid(std::uint32_t nowMs, std::uint32_t holdMs) const;
    bool rawValid(std::uint32_t nowMs, std::uint32_t holdMs) const;

    double filteredMm() const { return filtered_; }
    std::int32_t rawMm() const { return hasRaw_ ? raw_ : -1; }
    std::uint32_t lastValidMs() const { return lastValidMs_; }
    std::uint32_t lastRawMs() const { return lastRawMs_; }

private:
    void clearWindow();
    void pushSample(std::int32_t value);

    RangeFilterConfig config_{};
    std::int32_t window_[3] = {0, 0, 0};
    int windowCount_ = 0;
    std::int32_t outlierCount_ = 0;

    std::int32_t raw_ = -1;
    double filtered_ = 0.0;
    bool hasRaw_ = false;
    bool hasFiltered_ = false;
    std::uint32_t lastRawMs_ = 0;
    std::uint32_t lastValidMs_ = 0;
    std::uint32_t lastNoDistanceMs_ = 0;
    std::uint32_t lastUpdateStamp_ = 0;
    bool hasUpdateStamp_ = false;
};

}  // namespace lane
