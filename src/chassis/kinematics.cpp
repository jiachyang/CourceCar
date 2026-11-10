/**
 * @file kinematics.cpp
 * @brief 逆运动学实现。纯数学，无 PROS 依赖（ARCH-002）。
 */
#include "chassis/kinematics.hpp"

#include <cmath>

namespace chassis {

void bodyVelocityToWheelRpm(const ChassisConfig& config,
                            double vx, double vy, double omega,
                            double outRpm[kWheelCount]) {
    constexpr double kSqrt2 = 1.41421356237309504880;
    constexpr double kTwoPi = 6.28318530717958647692;
    const double inv = 1.0 / kSqrt2;

    // 旋转项：d_i·(ω×r_i) = ωR，四轮相同（推导见设计文档 §7.2）
    const double rot = config.centerToWheelM * omega;

    // s_i = d_i·(v + ω×r_i)
    const double sFl = inv * (-vx + vy) + rot;
    const double sFr = inv * ( vx + vy) + rot;
    const double sRl = inv * (-vx - vy) + rot;
    const double sRr = inv * ( vx - vy) + rot;
    const double s[kWheelCount] = {sFl, sFr, sRl, sRr};

    // 轮缘线速度 → 电机 RPM：rpm = (s / r_wheel)·(60/2π)·gearRatio
    const double rpmPerMps =
        (60.0 / kTwoPi) / config.wheelRadiusM * config.gearRatioMotorPerWheel;

    for (int i = 0; i < kWheelCount; ++i) {
        outRpm[i] = s[i] * rpmPerMps * config.mixScale[i] + config.mixOffset[i];
    }
}

}  // namespace chassis
