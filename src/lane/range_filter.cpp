/**
 * @file lane/range_filter.cpp
 * @brief 测距滤波实现。纯逻辑，不依赖 PROS。
 */
#include "lane/range_filter.hpp"

#include <algorithm>
#include <cmath>

#include "core/math_util.hpp"

namespace lane {
namespace {

double medianOf3(const std::int32_t* values, int count) {
    if (count <= 0) {
        return 0.0;
    }
    std::int32_t sorted[3] = {values[0], values[0], values[0]};
    for (int i = 0; i < count; ++i) {
        sorted[i] = values[i];
    }
    if (count == 1) {
        return static_cast<double>(sorted[0]);
    }
    if (count == 2) {
        return 0.5 * static_cast<double>(sorted[0] + sorted[1]);
    }
    std::sort(sorted, sorted + 3);
    return static_cast<double>(sorted[1]);
}

}  // namespace

void RangeFilter::init(const RangeFilterConfig& config) {
    config_ = config;
    reset();
}

void RangeFilter::reset() {
    clearWindow();
    outlierCount_ = 0;
    raw_ = -1;
    filtered_ = 0.0;
    hasRaw_ = false;
    hasFiltered_ = false;
    lastRawMs_ = 0;
    lastValidMs_ = 0;
    lastNoDistanceMs_ = 0;
    lastUpdateStamp_ = 0;
    hasUpdateStamp_ = false;
}

void RangeFilter::clearWindow() {
    window_[0] = 0;
    window_[1] = 0;
    window_[2] = 0;
    windowCount_ = 0;
}

void RangeFilter::pushSample(std::int32_t value) {
    if (windowCount_ < 3) {
        window_[windowCount_++] = value;
    } else {
        window_[0] = window_[1];
        window_[1] = window_[2];
        window_[2] = value;
    }
}

void RangeFilter::update(const core::RangeSample& sample, std::uint32_t nowMs) {
    const std::uint32_t stamp =
        (sample.timestampMs != 0) ? sample.timestampMs : nowMs;

    // 同一时刻/乱序的重复样本不重复改变滤波历史。
    if (hasUpdateStamp_ && stamp <= lastUpdateStamp_) {
        return;
    }
    lastUpdateStamp_ = stamp;
    hasUpdateStamp_ = true;

    if (sample.status != core::RangeStatus::kOk) {
        // 无效/无目标/超量程：不进入滤波，记录无距离起点。
        if (lastNoDistanceMs_ == 0) {
            lastNoDistanceMs_ = stamp;
        } else if (stamp >= lastNoDistanceMs_ &&
                   stamp - lastNoDistanceMs_ >=
                       config_.resetAfterNoDistanceMs) {
            clearWindow();
            outlierCount_ = 0;
            hasFiltered_ = false;
            filtered_ = 0.0;
        }
        return;
    }

    // 从长时间无距离恢复到有效测距：丢弃旧窗口，避免旧墙和新墙混合。
    if (lastNoDistanceMs_ != 0 && stamp >= lastNoDistanceMs_ &&
        stamp - lastNoDistanceMs_ >= config_.resetAfterNoDistanceMs) {
        clearWindow();
        outlierCount_ = 0;
        hasFiltered_ = false;
        filtered_ = 0.0;
    }
    lastNoDistanceMs_ = 0;

    const std::int32_t d = sample.distanceMm;
    raw_ = d;
    hasRaw_ = true;
    lastRawMs_ = stamp;

    // 跳变确认：单点异常先不改变滤波稳态；连续跳变才接受。
    if (hasFiltered_ &&
        std::fabs(static_cast<double>(d) - filtered_) >
            config_.jumpThresholdMm) {
        ++outlierCount_;
        if (outlierCount_ < config_.jumpConfirmSamples) {
            return;
        }
        clearWindow();
        pushSample(d);
        filtered_ = static_cast<double>(d);
        hasFiltered_ = true;
        lastValidMs_ = stamp;
        outlierCount_ = 0;
        return;
    }

    outlierCount_ = 0;
    pushSample(d);
    const double median = medianOf3(window_, windowCount_);
    if (!hasFiltered_) {
        filtered_ = median;
        hasFiltered_ = true;
    } else {
        const double alpha =
            core::clampd(config_.lowPassAlpha, 0.0, 1.0);
        filtered_ = alpha * median + (1.0 - alpha) * filtered_;
    }
    lastValidMs_ = stamp;
}

bool RangeFilter::valid(std::uint32_t nowMs, std::uint32_t holdMs) const {
    if (!hasFiltered_) {
        return false;
    }
    return nowMs >= lastValidMs_ && (nowMs - lastValidMs_) <= holdMs;
}

bool RangeFilter::rawValid(std::uint32_t nowMs, std::uint32_t holdMs) const {
    if (!hasRaw_) {
        return false;
    }
    return nowMs >= lastRawMs_ && (nowMs - lastRawMs_) <= holdMs;
}

}  // namespace lane
