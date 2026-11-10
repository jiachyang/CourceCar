/**
 * @file modules/chassis.cpp
 * @brief Chassis 模块实现：航向修正 → omega 平滑 → IK → 电机下发。
 */
#include "modules/chassis.hpp"

#include <cmath>

#include "chassis/kinematics.hpp"
#include "core/math_util.hpp"
#include "pros/rtos.hpp"

namespace chassis {

void Chassis::init(const ChassisConfig& config) {
    config_ = config;
    drive_.emplace(config_);  // 唯一电机管理者在此建立

    heading_.reset();
    heading_.setGains(config_.headingKp, config_.headingKi, config_.headingKd,
                      config_.headingIntegralLimit,
                      config_.maxHeadingCorrectionRadPerSec);
    heading_.setTermLimits(config_.headingPLimit, config_.headingILimit,
                           config_.headingDLimit);
    heading_.setFrictionFeedforward(
        config_.headingFrictionFeedforwardPos,
        config_.headingFrictionFeedforwardNeg,
        config_.headingFrictionRateThreshold,
        config_.headingFrictionErrorDeadband);

    target_ = core::zeroMotion();
    headingState_ = core::HeadingState{};
    state_ = ChassisState{};
    state_.initialized = true;

    bootMs_ = pros::millis();
    lastOmega_ = 0.0;
    initialized_ = true;
}

void Chassis::setCommand(const core::MotionCommand& command) {
    target_ = command;
}

void Chassis::setHeadingState(const core::HeadingState& state) {
    headingState_ = state;
}

void Chassis::update(double dt) {
    if (!initialized_) {
        return;
    }
    if (!(dt > 0.0) || dt > 0.5) {
        dt = 0.02;  // 首周期或异常间隔兜底
    }
    const std::uint32_t now = pros::millis();

    double omegaCmd = target_.omega;
    bool headingActive = false;

    // ---- 航向链路（IMU-021：默认关；航向源无效则自动不加修正）----
    if (enableHeadingHold && headingState_.valid) {
        // IMU-011：上电后等待 kHeadingLatchDelayMs 再锁存目标航向
        if (!heading_.hasTarget() &&
            (now - bootMs_) >= config_.headingLatchDelayMs) {
            heading_.latchTarget(headingState_.rotationRad);
        }
        heading_.setEnabled(heading_.hasTarget());
        if (heading_.enabled()) {
            omegaCmd = heading_.update(headingState_.rotationRad, target_.omega,
                                       headingState_.angularVelocityRadPerSec, dt);
            headingActive = true;
        }
    } else {
        heading_.setEnabled(false);
    }

    // ---- omega 平滑，避免主动旋转↔航向保持切换时跳变（IMU-016）----
    const double omegaBeforeSlew = omegaCmd;
    if (enableSlewLimit) {
        const double maxStep = config_.omegaSlewRadPerSecPerSec * dt;
        omegaCmd = lastOmega_ + core::clampd(omegaCmd - lastOmega_, -maxStep, maxStep);
    }
    lastOmega_ = omegaCmd;

    // ---- 逆运动学 → 四轮目标 RPM ----
    double rpm[kWheelCount];
    bodyVelocityToWheelRpm(config_, target_.vx, target_.vy, omegaCmd, rpm);
    for (int i = 0; i < kWheelCount; ++i) {
        state_.wheelTargetRpm[i] = rpm[i];
    }

    // ---- 电机下发 ----
    if (enableDrive && drive_) {
        drive_->applyWheelRpm(rpm, enableSlewLimit, state_.wheelCommandRpm);
        drive_->readWheelRpm(state_.wheelActualRpm);
        state_.error = !drive_->allMotorsInstalled();
    } else {
        for (int i = 0; i < kWheelCount; ++i) {
            state_.wheelCommandRpm[i] = 0.0;
        }
        state_.error = false;
    }

    state_.target = target_;
    state_.applied = core::MotionCommand{target_.vx, target_.vy, omegaCmd};
    state_.driveEnabled = enableDrive;
    state_.headingActive = headingActive;
    state_.headingTargetRad = heading_.hasTarget() ? heading_.targetRad() : 0.0;
    state_.heading = heading_.debug();
    state_.omegaBeforeSlewRadPerSec = omegaBeforeSlew;
    state_.headingValid = headingState_.valid;
}

void Chassis::stop() {
    lastOmega_ = 0.0;
    target_ = core::zeroMotion();
    heading_.reset();
    if (drive_) {
        drive_->stop();  // DRV-010
    }
    for (int i = 0; i < kWheelCount; ++i) {
        state_.wheelCommandRpm[i] = 0.0;
    }
    state_.headingActive = false;
}

}  // namespace chassis
