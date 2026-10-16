/**
 * @file heading_controller.cpp
 * @brief 航向 PID 实现。
 */
#include "chassis/heading_controller.hpp"

#include <cmath>

#include "chassis/math_util.hpp"

namespace chassis {

void HeadingController::reset() {
    hasTarget_ = false;
    integ_ = 0.0;
    prevErr_ = 0.0;
    wasActive_ = false;
}

void HeadingController::latchTarget(double yawRad) {
    targetRad_ = wrapToPi(yawRad);
    hasTarget_ = true;
    integ_ = 0.0;
    prevErr_ = 0.0;
    wasActive_ = false;
}

double HeadingController::update(double yawRad, double externalOmega, double dt) {
    if (!enabled_ || !hasTarget_) {
        return externalOmega;
    }

    // IMU-015：外部 omega ≠ 0 视为主动旋转，旁路航向保持
    const bool active = std::fabs(externalOmega) > 1e-6;
    if (active) {
        integ_ = 0.0;
        wasActive_ = true;
        return externalOmega;
    }

    // IMU-017：最短角差
    const double err = wrapToPi(targetRad_ - yawRad);
    if (wasActive_) {
        // 刚从主动旋转恢复到航向保持：清 D 项基准，避免速度突跳（IMU-016）
        prevErr_ = err;
        wasActive_ = false;
    }

    integ_ = clampd(integ_ + err * dt, -kHeadingIntegralLimit, kHeadingIntegralLimit);
    const double deriv = (dt > 1e-6) ? (err - prevErr_) / dt : 0.0;
    double corr = kHeadingKp * err + kHeadingKi * integ_ + kHeadingKd * deriv;
    corr = clampd(corr, -kMaxHeadingCorrectionRadPerSec, kMaxHeadingCorrectionRadPerSec);
    prevErr_ = err;

    // IMU-018：修正角速度叠加到 omega，再交给 IK
    return externalOmega + corr;
}

}  // namespace chassis
