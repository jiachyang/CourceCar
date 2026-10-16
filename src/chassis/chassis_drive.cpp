/**
 * @file chassis_drive.cpp
 * @brief 四电机速度驱动实现。
 */
#include "chassis/chassis_drive.hpp"

#include <algorithm>
#include <cmath>

namespace chassis {

ChassisDrive::ChassisDrive()
    : motors_{
          pros::Motor(kMotorPorts[kFl], pros::v5::MotorGears::green),
          pros::Motor(kMotorPorts[kFr], pros::v5::MotorGears::green),
          pros::Motor(kMotorPorts[kRl], pros::v5::MotorGears::green),
          pros::Motor(kMotorPorts[kRr], pros::v5::MotorGears::green)} {
    for (auto &m : motors_) {
        m.set_brake_mode(pros::v5::MotorBrake::brake);  // DRV-007
    }
    for (double &v : lastCmd_) {
        v = 0.0;
    }
}

void ChassisDrive::applyWheelRpm(const double rpm[kWheelCount], bool enableSlew) {
    double cmd[kWheelCount];

    // 1) 方向符号（KIN-008）：几何→电机方向的映射只发生在这里
    for (int i = 0; i < kWheelCount; ++i) {
        cmd[i] = rpm[i] * kCommandSign[i];
    }

    // 2) 饱和与等比缩放（T-07）：超过满速则四轮同比例缩小，保留运动方向
    double maxAbs = 0.0;
    for (double v : cmd) {
        maxAbs = std::max(maxAbs, std::fabs(v));
    }
    if (maxAbs > kMaxWheelRpm) {
        const double scale = kMaxWheelRpm / maxAbs;
        for (double &v : cmd) {
            v *= scale;
        }
    }

    // 3) 加减速限制（DRV-006）
    if (enableSlew && kSlewRpmPerCycle > 0.0) {
        for (int i = 0; i < kWheelCount; ++i) {
            const double delta = cmd[i] - lastCmd_[i];
            const double step = std::min(std::fabs(delta), kSlewRpmPerCycle);
            cmd[i] = lastCmd_[i] + (delta < 0.0 ? -step : step);
        }
    }

    // 4) 下发；掉线电机跳过（DRV-008）
    for (int i = 0; i < kWheelCount; ++i) {
        if (motors_[i].is_installed()) {
            motors_[i].move_velocity(static_cast<std::int32_t>(std::lround(cmd[i])));
        }
        lastCmd_[i] = cmd[i];
    }
}

void ChassisDrive::stop() {
    for (int i = 0; i < kWheelCount; ++i) {
        if (motors_[i].is_installed()) {
            motors_[i].move_velocity(0);
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
