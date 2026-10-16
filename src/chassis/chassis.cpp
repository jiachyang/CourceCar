/**
 * @file chassis.cpp
 * @brief 底盘门面实现：IMU 航向链路 + IK + 驱动。
 */
#include "chassis/chassis.hpp"

#include "chassis/kinematics.hpp"
#include "chassis/math_util.hpp"
#include "pros/rtos.hpp"

namespace chassis {

void Chassis::initialize() {
    bootMs_ = pros::millis();
    lastMs_ = 0;
    lastOmega_ = 0.0;
    heading_.reset();
    if (enableImuHeading) {
        imu_.startCalibration();  // IMU-004
    }
}

void Chassis::drive(double vx, double vy, double omega) {
    const std::uint32_t now = pros::millis();
    double dt = (lastMs_ == 0) ? 0.02 : (now - lastMs_) / 1000.0;
    if (!(dt > 0.0) || dt > 0.5) {
        dt = 0.02;  // 首周期或异常间隔兜底
    }
    lastMs_ = now;

    double omegaCmd = omega;

    // ---- IMU 航向链路（IMU-021：默认关；开启后 IMU 无效则自动不加修正）----
    if (enableImuHeading) {
        double yaw = 0.0;
        if (imu_.readHeadingRad(yaw)) {
            // IMU-011：上电后等待 kHeadingLatchDelayMs 再锁存目标航向
            if (!heading_.hasTarget() && (now - bootMs_) >= kHeadingLatchDelayMs) {
                heading_.latchTarget(yaw);
            }
            heading_.setEnabled(heading_.hasTarget());
            omegaCmd = heading_.update(yaw, omega, dt);  // IMU-015/018
        } else {
            heading_.setEnabled(false);  // IMU-009/T-10：无效则不修正
        }
    } else {
        heading_.setEnabled(false);
    }

    // ---- omega 平滑，避免主动旋转↔航向保持切换时跳变（IMU-016）----
    if (enableSlewLimit) {
        const double maxStep = kOmegaSlewRadPerSecPerSec * dt;
        omegaCmd = lastOmega_ + clampd(omegaCmd - lastOmega_, -maxStep, maxStep);
    }
    lastOmega_ = omegaCmd;

    // ---- 逆运动学 → 四轮 RPM ----
    double rpm[kWheelCount];
    bodyVelocityToWheelRpm(vx, vy, omegaCmd, rpm);

    if (!enableDrive) {
        return;  // ARCH-009：模块关闭
    }
    drive_.applyWheelRpm(rpm, enableSlewLimit);
}

void Chassis::stop() {
    lastOmega_ = 0.0;
    drive_.stop();  // DRV-010
}

}  // namespace chassis
