/**
 * @file chassis_drive.cpp
 * @brief 四电机速度驱动实现。
 */
#include "chassis/chassis_drive.hpp"

#include <algorithm>
#include <cmath>

namespace chassis {

ChassisDrive::ChassisDrive(const ChassisConfig& config)
    : config_(config),
      motors_{
          pros::Motor(config.motorPorts[kFl], pros::v5::MotorGears::green),
          pros::Motor(config.motorPorts[kFr], pros::v5::MotorGears::green),
          pros::Motor(config.motorPorts[kRl], pros::v5::MotorGears::green),
          pros::Motor(config.motorPorts[kRr], pros::v5::MotorGears::green)} {
    for (auto &m : motors_) {
        m.set_brake_mode(pros::v5::MotorBrake::brake);  // DRV-007
    }
    for (double &v : lastCmd_) {
        v = 0.0;
    }
}

void ChassisDrive::applyWheelRpm(const double rpm[kWheelCount], bool enableSlew,
                                 double outApplied[kWheelCount]) {
    double cmd[kWheelCount];

    // 1) 方向符号（KIN-008）：几何→电机方向的映射只发生在这里
    for (int i = 0; i < kWheelCount; ++i) {
        cmd[i] = rpm[i] * config_.commandSign[i];
    }

    // 2) 饱和与等比缩放（T-07）：超过满速则四轮同比例缩小，保留运动方向
    double maxAbs = 0.0;
    for (double v : cmd) {
        maxAbs = std::max(maxAbs, std::fabs(v));
    }
    if (maxAbs > config_.maxWheelRpm) {
        const double scale = config_.maxWheelRpm / maxAbs;
        for (double &v : cmd) {
            v *= scale;
        }
    }

    // 3) 加减速限制（DRV-006）
    if (enableSlew && config_.slewRpmPerCycle > 0.0) {
        for (int i = 0; i < kWheelCount; ++i) {
            const double delta = cmd[i] - lastCmd_[i];
            const double step = std::min(std::fabs(delta), config_.slewRpmPerCycle);
            cmd[i] = lastCmd_[i] + (delta < 0.0 ? -step : step);
        }
    }

    // 4) 下发；掉线电机跳过（DRV-008）
    //    move_velocity 只接受整数 rpm，小修正会被 lround 抹成 0。
    //    改用 move_voltage：rpm → mV（线性），分辨率 ~0.017 rpm，支持小数。
    const double rpmToMv = config_.motorRpmToMillivolt;
    for (int i = 0; i < kWheelCount; ++i) {
        double mv = cmd[i] * rpmToMv;
        mv = std::max(-12000.0, std::min(12000.0, mv));
        const std::int32_t send = static_cast<std::int32_t>(std::lround(mv));
        if (motors_[i].is_installed()) {
            motors_[i].move_voltage(send);
        }
        lastCmd_[i] = cmd[i];
        if (outApplied != nullptr) {
            outApplied[i] = cmd[i];
        }
    }
}

void ChassisDrive::readWheelRpm(double out[kWheelCount]) {
    for (int i = 0; i < kWheelCount; ++i) {
        out[i] = motors_[i].get_actual_velocity();
    }
}

void ChassisDrive::stop() {
    for (int i = 0; i < kWheelCount; ++i) {
        if (motors_[i].is_installed()) {
            motors_[i].move_voltage(0);
        }
        lastCmd_[i] = 0.0;
    }
}

bool ChassisDrive::allMotorsInstalled() {
    for (auto &m : motors_) {
        if (!m.is_installed()) {
            return false;
        }
    }
    return true;
}

}  // namespace chassis
