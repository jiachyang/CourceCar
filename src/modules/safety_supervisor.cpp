/**
 * @file modules/safety_supervisor.cpp
 * @brief 前向安全监督器实现。纯逻辑，不依赖 PROS。
 */
#include "modules/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace safety {
namespace {

bool finiteCommand(const core::MotionCommand& c) {
    return std::isfinite(c.vx) && std::isfinite(c.vy) &&
           std::isfinite(c.omega);
}

/// 四个顶点 ToF 的车体出射轴，顺序与 core::RangeSnapshot 一致。
constexpr double kAxisX[4] = {1.0, -1.0, 0.0, 0.0};
constexpr double kAxisY[4] = {0.0, 0.0, 1.0, -1.0};

bool usableSample(const core::RangeSample& s, const SafetyConfig& c) {
    if (s.status != core::RangeStatus::kOk &&
        s.status != core::RangeStatus::kTooClose) {
        return false;
    }
    if (s.confidence < c.minConfidence) {
        return false;
    }
    return s.distanceMm > 0;
}

}  // namespace

void SafetySupervisor::init(const SafetyConfig& config) {
    config_ = config;
    configValid_ = isValidSafetyConfig(config_);
    initialized_ = true;
    reset();
}

void SafetySupervisor::reset() {
    state_ = SafetyState{};
    state_.enabled = enable;
    state_.configValid = configValid_;
}

void SafetySupervisor::update(const core::MotionCommand& command,
                              const core::RangeSnapshot& ranges,
                              const core::HeadingState& heading) {
    state_ = SafetyState{};
    state_.requested = command;
    state_.enabled = enable;
    state_.configValid = configValid_;
    state_.timestampMs = ranges.timestampMs;

    if (!initialized_ || !configValid_) {
        // 安全配置非法：fail-closed，不允许平移。
        state_.command = core::zeroMotion();
        state_.scale = 0.0;
        state_.limited = true;
        state_.stopped = true;
        return;
    }
    if (!finiteCommand(command)) {
        state_.command = core::zeroMotion();
        state_.scale = 0.0;
        state_.limited = true;
        state_.stopped = true;
        return;
    }
    if (!enable) {
        state_.command = command;
        return;
    }
    if (config_.requireHeading && !heading.valid) {
        state_.command = core::zeroMotion();
        state_.command.omega = command.omega;
        state_.scale = 0.0;
        state_.limited = true;
        state_.stopped = true;
        state_.headingBlocked = true;
        return;
    }

    const double speed = std::hypot(command.vx, command.vy);
    if (!(speed > 1e-9)) {
        state_.command = command;
        return;
    }

    const double dirX = command.vx / speed;
    const double dirY = command.vy / speed;
    const double brake = std::max(0.02, config_.brakeMps2);
    const double reaction = std::max(0.0, config_.reactionS);

    double scale = 1.0;
    double nearestMm = -1.0;
    for (int i = 0; i < core::kRangeSideCount; ++i) {
        const core::RangeSample& s = ranges.samples[i];
        if (!usableSample(s, config_)) {
            continue;
        }
        const double align = kAxisX[i] * dirX + kAxisY[i] * dirY;
        if (align < config_.minAxisAlignment) {
            continue;
        }
        const double distanceM = static_cast<double>(s.distanceMm) / 1000.0;
        const double freeM = std::max(0.0, distanceM - config_.tofReserveM);
        const double vmax =
            std::sqrt((brake * reaction) * (brake * reaction) +
                      2.0 * brake * freeM) -
            brake * reaction;
        const double factor =
            std::max(0.0, std::min(1.0, vmax / (speed * align)));
        scale = std::min(scale, factor);
        if (nearestMm < 0.0 || distanceM * 1000.0 < nearestMm) {
            nearestMm = distanceM * 1000.0;
        }
    }

    state_.scale = scale;
    state_.nearestMm = nearestMm;
    state_.limited = scale < 1.0 - 1e-9;
    state_.stopped = scale < 1e-5;
    state_.command = command;
    state_.command.vx = command.vx * scale;
    state_.command.vy = command.vy * scale;
    // omega 原样保留：航向由 Chassis 的航向环负责。
}

}  // namespace safety
