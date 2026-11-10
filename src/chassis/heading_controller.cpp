/**
 * @file heading_controller.cpp
 * @brief 航向 PID 实现。
 */
#include "chassis/heading_controller.hpp"

#include <cmath>

#include "core/math_util.hpp"

namespace chassis {

void HeadingController::reset() {
    hasTarget_ = false;
    integ_ = 0.0;
    prevErr_ = 0.0;
    wasActive_ = false;
    targetRad_ = 0.0;
}

void HeadingController::setGains(double kp, double ki, double kd,
                                 double integralLimit, double maxCorrectionRadPerSec) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integralLimit_ = integralLimit;
    maxCorrection_ = maxCorrectionRadPerSec;
}

void HeadingController::latchTarget(double yawRad) {
    targetRad_ = core::wrapToPi(yawRad);
    hasTarget_ = true;
    integ_ = 0.0;
    prevErr_ = 0.0;
    wasActive_ = false;
}

double HeadingController::update(double yawRad, double externalOmega,
                                 double measuredOmega, double dt) {
    debug_ = Debug{};
    debug_.kp = kp_;
    debug_.ki = ki_;
    debug_.kd = kd_;
    debug_.integralLimit = integralLimit_;
    debug_.pLimit = pLimit_;
    debug_.iLimit = iLimit_;
    debug_.dLimit = dLimit_;
    debug_.maxCorr = maxCorrection_;

    if (!enabled_ || !hasTarget_) {
        debug_.extOmega = externalOmega;
        return externalOmega;
    }

    // IMU-015：外部 omega ≠ 0 视为主动旋转，旁路航向保持。
    // 同时让目标跟随当前朝向：这样松手（omega 归零）的瞬间目标已经等于
    // 松手时的朝向，不会再“转回开机锁存的旧目标”产生无输入自转。
    const bool active = std::fabs(externalOmega) > 1e-6;
    if (active) {
        targetRad_ = core::wrapToPi(yawRad);  // 主动旋转期间持续重锁目标
        integ_ = 0.0;
        prevErr_ = 0.0;
        wasActive_ = true;
        debug_.active = true;
        debug_.extOmega = externalOmega;
        return externalOmega;
    }

    // IMU-017：最短角差
    const double err = core::wrapToPi(targetRad_ - yawRad);
    if (wasActive_) {
        // 刚从主动旋转恢复到航向保持：清 D 项基准，避免速度突跳（IMU-016）
        prevErr_ = err;
        wasActive_ = false;
    }

    integ_ = core::clampd(integ_ + err * dt, -integralLimit_, integralLimit_);
    const double deriv = (dt > 1e-6) ? (err - prevErr_) / dt : 0.0;
    // 每一项各自限幅，再对总输出限幅（四项都有 clamp）。
    const double p = core::clampd(kp_ * err, -pLimit_, pLimit_);
    const double i = core::clampd(ki_ * integ_, -iLimit_, iLimit_);
    const double d = core::clampd(kd_ * deriv, -dLimit_, dLimit_);
    const double corr = core::clampd(p + i + d, -maxCorrection_, maxCorrection_);
    prevErr_ = err;

    // 静摩擦前馈：只在“接近静止 + 误差超过死区”时加入。
    // 方向由误差符号决定；正/负方向可使用不同的实测值。
    double ff = 0.0;
    const bool nearRest = std::fabs(measuredOmega) <= ffRateThreshold_;
    if (nearRest && std::fabs(err) > ffErrorDeadband_) {
        ff = (err > 0.0) ? ffPos_ : ffNeg_;
    }

    debug_.err = err;
    debug_.pTerm = p;
    debug_.iTerm = i;
    debug_.dTerm = d;
    debug_.corr = corr;
    debug_.ffTerm = ff;
    debug_.ffActive = (ff != 0.0);
    debug_.extOmega = externalOmega;

    // IMU-018：修正角速度与前馈叠加到 omega，再交给 IK
    return externalOmega + corr + ff;
}

}  // namespace chassis
