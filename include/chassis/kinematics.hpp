/**
 * @file kinematics.hpp
 * @brief 逆运动学（IK）：车体速度 → 四轮目标转速。纯数学，不访问硬件。
 */
#pragma once

#include "chassis/chassis_config.hpp"

namespace chassis {

/**
 * 逆运动学：把车体三自由度速度分配到四个轮子。
 *
 * 公式（推导见 docs/chassis_design.md §7.2）：
 *   s_i = d_i · (v + ω × r_i)                    轮缘线速度 [m/s]
 *   rpm_i = (s_i / r_wheel)·(60/2π)·gearRatio    电机目标转速
 *
 * 几何参数取自 config（centerToWheelM = R，wheelRadiusM = r，gearRatio），
 * 每轮标定增益/偏置取自 config.mixScale / mixOffset。
 * 电机方向符号不在这里处理（KIN-008），由 ChassisDrive 负责。
 *
 * @param config 底盘配置
 * @param vx     前后速度 [m/s]，正方向 = 车头
 * @param vy     左右速度 [m/s]，正方向 = 车体左侧
 * @param omega  角速度 [rad/s]，正方向 = 俯视逆时针
 * @param outRpm 输出四轮 RPM，顺序 FL,FR,RL,RR（含每轮增益/偏置）
 */
void bodyVelocityToWheelRpm(const ChassisConfig& config,
                            double vx, double vy, double omega,
                            double outRpm[kWheelCount]);

}  // namespace chassis
