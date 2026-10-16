/**
 * @file chassis_drive.hpp
 * @brief 四电机速度驱动：方向符号、饱和缩放、加减速、刹车。唯一访问电机硬件的模块。
 */
#pragma once

#include "chassis/chassis_config.hpp"
#include "pros/motors.hpp"

namespace chassis {

class ChassisDrive {
public:
    ChassisDrive();

    /// 下发四轮目标 RPM（顺序 FL,FR,RL,RR）。内部处理方向符号、饱和、slew。
    void applyWheelRpm(const double rpm[kWheelCount], bool enableSlew);

    /// 刹车（DRV-010：独立函数，当前 brake 模式，日后可整体替换）。
    void stop();

    /// 四轮是否都在线（DRV-008 故障检测用）。
    bool allMotorsInstalled();

private:
    pros::Motor motors_[kWheelCount];  ///< 顺序 FL,FR,RL,RR
    double lastCmd_[kWheelCount];      ///< 上一周期指令，用于 slew
};

}  // namespace chassis
