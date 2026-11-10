/**
 * @file lane/wall_state.cpp
 * @brief 10/15 cm 滞回墙体状态机实现。纯逻辑，不依赖 PROS。
 */
#include "lane/wall_state.hpp"

namespace lane {
namespace {

/// 记录一次“有效远距证据”后，是否达到确认次数。
void bumpFar(const WallConfig& config, WallStatus& status,
             std::int32_t& nearCount, std::int32_t& farCount) {
    nearCount = 0;
    if (status == WallStatus::kAbsent) {
        farCount = 0;  // 已确认无墙，不必让计数无限增长
        return;
    }
    ++farCount;
    if (farCount >= config.confirmCount) {
        status = WallStatus::kAbsent;
        farCount = 0;
        nearCount = 0;
    }
}

}  // namespace

void WallTracker::init(const WallConfig& config) {
    config_ = config;
    reset();
}

void WallTracker::reset() {
    status_ = WallStatus::kUnknown;
    nearCount_ = 0;
    farCount_ = 0;
    lastDistanceMm_ = -1;
    lastValidMs_ = 0;
    lastEvidenceMs_ = 0;
    lastUpdateMs_ = 0;
    hasDistance_ = false;
    hasUpdate_ = false;
}

void WallTracker::update(const core::RangeSample& sample, std::uint32_t nowMs) {
    const std::uint32_t stamp =
        (sample.timestampMs != 0) ? sample.timestampMs : nowMs;

    // 固定周期采样：同一时刻/过短间隔的重复调用不重复计数。
    if (hasUpdate_) {
        if (stamp < lastUpdateMs_) {
            return;  // 乱序旧样本
        }
        if (stamp - lastUpdateMs_ < config_.minSampleIntervalMs) {
            return;  // 重复/过快样本
        }
    }
    lastUpdateMs_ = stamp;
    hasUpdate_ = true;

    const core::RangeStatus s = sample.status;

    switch (s) {
        case core::RangeStatus::kOk: {
            const std::int32_t d = sample.distanceMm;
            lastEvidenceMs_ = stamp;

            if (d < config_.enterMm) {
                farCount_ = 0;
                if (nearCount_ < config_.confirmCount) {
                    ++nearCount_;
                }
                lastDistanceMm_ = d;
                lastValidMs_ = stamp;
                hasDistance_ = true;
                if (nearCount_ >= config_.confirmCount &&
                    status_ != WallStatus::kPresent) {
                    status_ = WallStatus::kPresent;
                    nearCount_ = 0;
                    farCount_ = 0;
                }
            } else if (d > config_.exitMm) {
                // 严格大于 150 才是有效远墙证据；150 本身留在滞回带。
                lastDistanceMm_ = d;
                lastValidMs_ = stamp;
                hasDistance_ = true;
                bumpFar(config_, status_, nearCount_, farCount_);
            } else {
                // 100..150 mm（含端点）：滞回保持，打断连续序列。
                nearCount_ = 0;
                farCount_ = 0;
                lastDistanceMm_ = d;
                lastValidMs_ = stamp;
                hasDistance_ = true;
            }
            break;
        }

        case core::RangeStatus::kNoTarget:
        case core::RangeStatus::kTooFar:
        case core::RangeStatus::kTooClose:
        case core::RangeStatus::kReadError:
        case core::RangeStatus::kNotInstalled:
        default:
            // 全部按无效处理（含 9999 / 超量程）：
            // 打断连续计数，不确认无墙，不刷新正常 PID 测量。
            nearCount_ = 0;
            farCount_ = 0;
            if (status_ != WallStatus::kUnknown && nowMs >= lastEvidenceMs_ &&
                nowMs - lastEvidenceMs_ >= config_.staleTimeoutMs) {
                status_ = WallStatus::kUnknown;
                hasDistance_ = false;
                lastDistanceMm_ = -1;
            }
            break;
    }
}

bool WallTracker::usable(std::uint32_t nowMs, std::uint32_t holdMs) const {
    if (status_ != WallStatus::kPresent || !hasDistance_) {
        return false;
    }
    return nowMs >= lastValidMs_ && (nowMs - lastValidMs_) <= holdMs;
}

}  // namespace lane
