/**
 * @file lane/range_channel.cpp
 * @brief 单路近壁通道实现。纯逻辑，不依赖 PROS。
 */
#include "lane/range_channel.hpp"

#include <algorithm>
#include <cmath>

namespace lane {
namespace {

bool validSample(const core::RangeSample& s, const RangeChannelConfig& c) {
    if (s.status != core::RangeStatus::kOk &&
        s.status != core::RangeStatus::kTooClose) {
        return false;
    }
    const double mm = static_cast<double>(s.distanceMm);
    return std::isfinite(mm) && mm > c.minValidMm && mm <= c.maxValidMm;
}

}  // namespace

void RangeChannel::init(const RangeChannelConfig& config) {
    config_ = config;
    reset();
}

void RangeChannel::reset() {
    state_ = NearClass::kUnknown;
    nearCount_ = 0;
    farCount_ = 0;
    midCount_ = 0;
    jumpCount_ = 0;
    window_[0] = 0.0;
    window_[1] = 0.0;
    window_[2] = 0.0;
    windowCount_ = 0;
    filtered_ = 0.0;
    raw_ = -1;
    hasFiltered_ = false;
    hasRaw_ = false;
    observationValid_ = false;
    seen_ = false;
    lastGoodMs_ = 0;
    lastUpdateStamp_ = 0;
    hasUpdateStamp_ = false;
}

void RangeChannel::classify(double mm) {
    const int confirm = std::max(1, config_.confirmFrames);
    if (mm < config_.enterMm) {
        nearCount_ = std::min(confirm, nearCount_ + 1);
        farCount_ = 0;
        midCount_ = 0;
        if (nearCount_ >= confirm) {
            state_ = NearClass::kNearReturn;
        }
    } else if (mm > config_.exitMm) {
        farCount_ = std::min(confirm, farCount_ + 1);
        nearCount_ = 0;
        midCount_ = 0;
        if (farCount_ >= confirm) {
            state_ = NearClass::kFarReturn;
        }
    } else {
        // 100–150 mm（含端点）：保持原有近/远状态；首次观测连续确认后
        // 进入 MID_RETURN，避免“上电就在滞回带 -> 永远 Unknown”。
        nearCount_ = 0;
        farCount_ = 0;
        midCount_ = std::min(confirm, midCount_ + 1);
        if (state_ == NearClass::kUnknown && midCount_ >= confirm) {
            state_ = NearClass::kMidReturn;
        }
    }
}

void RangeChannel::pushSample(double mm) {
    if (windowCount_ < 3) {
        window_[windowCount_++] = mm;
    } else {
        window_[0] = window_[1];
        window_[1] = window_[2];
        window_[2] = mm;
    }
}

double RangeChannel::median() const {
    if (windowCount_ <= 0) {
        return 0.0;
    }
    double v[3] = {window_[0], window_[1], window_[2]};
    if (windowCount_ == 1) {
        return v[0];
    }
    if (windowCount_ == 2) {
        return 0.5 * (v[0] + v[1]);
    }
    std::sort(v, v + 3);
    return v[1];
}

void RangeChannel::update(const core::RangeSample& sample,
                          std::uint32_t nowMs) {
    const std::uint32_t stamp =
        (sample.timestampMs != 0) ? sample.timestampMs : nowMs;

    // 同一时刻/乱序旧样本不重复改变分类与滤波历史。
    if (hasUpdateStamp_ && stamp <= lastUpdateStamp_) {
        return;
    }
    lastUpdateStamp_ = stamp;
    hasUpdateStamp_ = true;

    if (!validSample(sample, config_)) {
        // 无回波/读取失败/超量程：不进入分类与滤波，也绝不确认“无墙”。
        // 短时保留旧估计；超过 maxAge 才释放为 UNKNOWN。
        observationValid_ = false;
        if (seen_ && nowMs >= lastGoodMs_ &&
            nowMs - lastGoodMs_ > config_.maxAgeMs) {
            state_ = NearClass::kUnknown;
        }
        return;
    }

    observationValid_ = true;
    seen_ = true;
    const double mm = static_cast<double>(sample.distanceMm);
    raw_ = sample.distanceMm;
    hasRaw_ = true;
    lastGoodMs_ = stamp;

    classify(mm);

    // 跳变确认：单点异常先不改变滤波稳态；连续跳变才接受新稳态。
    // 分类仍按原始有效回波更新，与滤波解耦。
    if (hasFiltered_ &&
        std::fabs(mm - filtered_) > config_.jumpMm) {
        ++jumpCount_;
        if (jumpCount_ < std::max(1, config_.jumpConfirm)) {
            return;
        }
        windowCount_ = 0;
        pushSample(mm);
        filtered_ = mm;
        hasFiltered_ = true;
        jumpCount_ = 0;
        return;
    }

    jumpCount_ = 0;
    pushSample(mm);
    const double med = median();
    if (!hasFiltered_) {
        filtered_ = med;
        hasFiltered_ = true;
    } else {
        const double alpha =
            std::max(0.0, std::min(1.0, config_.lowPassAlpha));
        filtered_ = alpha * med + (1.0 - alpha) * filtered_;
    }
}

RangeChannelValue RangeChannel::value(std::uint32_t nowMs) const {
    RangeChannelValue out{};
    out.state = state_;
    out.observationValid = observationValid_;
    if (!seen_ || !hasFiltered_) {
        return out;
    }
    if (nowMs < lastGoodMs_) {
        return out;
    }
    out.ageMs = nowMs - lastGoodMs_;
    out.valid = out.ageMs <= config_.maxAgeMs;
    out.fresh = out.valid && out.ageMs <= config_.freshWindowMs;
    out.mm = filtered_;
    out.rawMm = static_cast<double>(raw_);
    return out;
}

}  // namespace lane
