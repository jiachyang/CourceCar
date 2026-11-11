#include "imu/startup_heading.hpp"

#include "core/math_util.hpp"

#include <cmath>

namespace imu {
namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

void StartupHeading::clearWindow() {
    count_ = 0;
    gyroSumDps_ = 0.0;
    state_.stillMs = 0;
}

void StartupHeading::init(const ImuConfig& config, std::uint32_t nowMs) {
    config_ = config;
    state_ = StartupHeadingState{};
    bootMs_ = nowMs;
    clearWindow();
    initialized_ = true;
}

void StartupHeading::update(double rotationDeg, double gyroDps, bool valid,
                            std::uint32_t nowMs) {
    state_.relativeValid = false;
    if (!initialized_ || !valid || !std::isfinite(rotationDeg) ||
        !std::isfinite(gyroDps)) {
        clearWindow();
        return;
    }

    // 只确认旋转静止；平移不能由 Z 轴陀螺识别，实机初始化仍需整车静止。
    if (count_ && nowMs - lastMs_ > config_.startupMaxSampleGapMs) {
        clearWindow();
    }
    if (std::fabs(gyroDps) > config_.startupMaxGyroDps) {
        clearWindow();
    } else {
        if (count_ && std::fabs(rotationDeg - windowStartDeg_) >
                          config_.startupMaxAngleChangeDeg) {
            clearWindow();
        }
        if (!count_) {
            windowStartMs_ = nowMs;
            windowStartDeg_ = rotationDeg;
        }
        gyroSumDps_ += gyroDps;
        ++count_;
        state_.stillMs = nowMs - windowStartMs_;
    }
    lastMs_ = nowMs;

    if (!state_.biasMeasured) {
        if (config_.useManualGyroBias) {
            state_.gyroBiasDps = config_.gyroBiasDps;
            state_.gyroDriftDps = config_.gyroDriftDps;
            state_.biasMeasured = true;
        } else if (count_ && state_.stillMs >= config_.gyroBiasMeasureMs) {
            state_.gyroBiasDps = gyroSumDps_ / static_cast<double>(count_);
            const double drift = (rotationDeg - windowStartDeg_) /
                                 (state_.stillMs / 1000.0);
            // 角度漂移只作诊断；保持旧实现的噪声门限，不回扣航向角。
            state_.gyroDriftDps = std::fabs(drift) < config_.gyroDriftDeadbandDps
                                      ? 0.0 : drift;
            state_.biasMeasured = true;
        }
    }

    const double angle = rotationDeg * config_.yawSign * kDegToRad +
                         config_.mountingYawOffsetRad;
    if (!state_.referenceReady && state_.biasMeasured && count_ &&
        nowMs - bootMs_ >= config_.referenceDelayMs &&
        state_.stillMs >= config_.referenceStillMs) {
        state_.initialHeadingRad = angle;
        state_.referenceReady = true;
        state_.latchedMs = nowMs;
    }
    if (state_.referenceReady) {
        state_.relativeHeadingRad = core::wrapToPi(angle - state_.initialHeadingRad);
        state_.relativeValid = true;
    }
}

}  // namespace imu
