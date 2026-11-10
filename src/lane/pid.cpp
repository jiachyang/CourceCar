/**
 * @file lane/pid.cpp
 * @brief 墙距 PID 实现。纯逻辑，不依赖 PROS。
 */
#include "lane/pid.hpp"

#include <cmath>

#include "core/math_util.hpp"

namespace lane {

void Pid::init(const PidConfig& config) {
    config_ = config;
    reset();
}

void Pid::reset() {
    debug_ = PidDebug{};
    integral_ = 0.0;
    prevError_ = 0.0;
    prevDerivative_ = 0.0;
    hasPrev_ = false;
}

double Pid::update(double error, double dt, bool allowIntegral) {
    debug_ = PidDebug{};
    debug_.error = error;
    debug_.integralAllowed = allowIntegral;

    // 非有限输入 / 非法周期：不产生输出，并清掉历史。
    if (!std::isfinite(error) || !(dt > 0.0) || dt > 0.5) {
        reset();
        debug_.error = std::isfinite(error) ? error : 0.0;
        return 0.0;
    }

    // D 项：对误差求导，一阶低通。复位后的第一帧 D=0。
    if (!hasPrev_) {
        prevError_ = error;
        prevDerivative_ = 0.0;
        hasPrev_ = true;
    } else {
        const double rawDerivative = (error - prevError_) / dt;
        const double alpha =
            core::clampd(config_.derivativeFilterAlpha, 0.0, 1.0);
        debug_.derivative =
            alpha * rawDerivative + (1.0 - alpha) * prevDerivative_;
        prevDerivative_ = debug_.derivative;
        prevError_ = error;
    }

    const double previousIntegral = integral_;
    double integral = previousIntegral;

    // Ki=0 明确关闭积分；不允许积分时冻结当前积分（不清零）。
    if (config_.ki != 0.0) {
        if (allowIntegral) {
            integral += error * dt;
            integral = core::clampd(integral, -config_.integralLimit,
                                    config_.integralLimit);
        }
    } else {
        integral = 0.0;
    }

    const double p = core::clampd(config_.kp * error, -config_.pLimit,
                                  config_.pLimit);
    double i = core::clampd(config_.ki * integral, -config_.iLimit,
                            config_.iLimit);
    const double d = core::clampd(config_.kd * debug_.derivative,
                                  -config_.dLimit, config_.dLimit);
    double raw = p + i + d;
    bool saturated =
        (raw > config_.outputLimit || raw < -config_.outputLimit);

    // 抗饱和：输出顶到限幅且误差还在把积分往同方向推时，撤销本帧积分增量。
    if (allowIntegral && config_.ki != 0.0) {
        if ((raw > config_.outputLimit && error > 0.0) ||
            (raw < -config_.outputLimit && error < 0.0)) {
            integral = previousIntegral;
            i = core::clampd(config_.ki * integral, -config_.iLimit,
                            config_.iLimit);
            raw = p + i + d;
        }
    }

    integral_ = integral;
    debug_.integral = integral;
    debug_.pTerm = p;
    debug_.iTerm = i;
    debug_.dTerm = d;
    debug_.output =
        core::clampd(raw, -config_.outputLimit, config_.outputLimit);
    debug_.saturated = saturated || (raw != debug_.output);
    return debug_.output;
}

}  // namespace lane
