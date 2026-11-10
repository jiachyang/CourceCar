/**
 * @file modules/radio.cpp
 * @brief Radio 模块（V5 Controller 遥控输入）实现。
 */
#include "modules/radio.hpp"

#include "pros/rtos.hpp"
#include "radio/input_mapping.hpp"

namespace radio {

namespace {

/// 读取一个按键，返回是否按下。
bool readButton(pros::Controller& c, pros::controller_digital_e_t b) {
    return c.get_digital(b) != 0;
}

}  // namespace

void Radio::init(const RadioConfig& config) {
    config_ = config;
    controller_.emplace(config_.masterId);
    state_ = RadioState{};
}

void Radio::update() {
    if (!controller_) {
        state_ = RadioState{};
        state_.driveCommand = core::zeroMotion();
        return;
    }

    state_.connected = controller_->is_connected();
    if (!state_.connected) {
        // 失联：彻底清零，绝不得沿用上一帧命令（安全要求）
        state_ = RadioState{};
        state_.driveCommand = core::zeroMotion();
        return;
    }

    // 归一化摇杆（含死区）
    state_.leftX  = normalizeStick(controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), config_.deadband);
    state_.leftY  = normalizeStick(controller_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), config_.deadband);
    state_.rightX = normalizeStick(controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), config_.deadband);
    state_.rightY = normalizeStick(controller_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), config_.deadband);

    // 按键
    state_.buttonA  = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_A);
    state_.buttonB  = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_B);
    state_.buttonX  = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_X);
    state_.buttonY  = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_Y);
    state_.buttonL1 = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_L1);
    state_.buttonL2 = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_L2);
    state_.buttonR1 = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_R1);
    state_.buttonR2 = readButton(*controller_, pros::E_CONTROLLER_DIGITAL_R2);

    // 摇杆 → 车体命令：映射规则集中在 radio::mapToMotionCommand（可单测）
    state_.driveCommand =
        mapToMotionCommand(state_.leftX, state_.leftY, state_.rightX,
                           config_.maxLinearSpeedMps,
                           config_.maxAngularSpeedRadPerSec,
                           config_.invertForward, config_.invertStrafe,
                           config_.invertTurn, config_.snapLeftStickToAxis);
    state_.valid = true;
    state_.timestampMs = pros::millis();
}

bool Radio::isCommandFresh() const {
    if (!state_.valid) {
        return false;
    }
    return (pros::millis() - state_.timestampMs) <= config_.commandTimeoutMs;
}

}  // namespace radio
