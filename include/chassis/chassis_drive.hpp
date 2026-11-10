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
    /// 构造四个电机（端口来自 config），并设为 brake 模式（DRV-007）。
    explicit ChassisDrive(const ChassisConfig& config);

    /**
     * 下发四轮目标 RPM（顺序 FL,FR,RL,RR）。内部处理方向符号、饱和、slew。
     * @param rpm         IK 输出的目标转速
     * @param enableSlew  是否启用加减速限制
     * @param outApplied  输出最终下发的转速（用于状态上报），可为 nullptr
     */
    void applyWheelRpm(const double rpm[kWheelCount], bool enableSlew,
                       double outApplied[kWheelCount]);

    /// 读取四轮实测转速 [rpm]。
    void readWheelRpm(double out[kWheelCount]);

    /// 刹车（DRV-010：独立函数，当前 brake 模式，日后可整体替换）。
    void stop();

    /// 四轮是否都在线（DRV-008 故障检测用）。
    bool allMotorsInstalled();

private:
    ChassisConfig config_{};
    pros::Motor motors_[kWheelCount];  ///< 顺序 FL,FR,RL,RR
    double lastCmd_[kWheelCount];      ///< 上一周期指令，用于 slew
};

}  // namespace chassis
