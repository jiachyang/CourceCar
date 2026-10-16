/**
 * @file chassis_config.hpp
 * @brief 底盘**模块内部**的真值参数（端口、几何、传动、方向、限幅）。
 *
 * 需求 ARCH-010：端口、传动比等属于模块内部，**不放进 main**；
 * main 只放整车级开关（见 src/main.cpp 的 VehicleConfig）。
 * 任何模块都不得再散落字面量端口号 / 轮径 / 阈值。
 */
#pragma once

#include <cstdint>

namespace chassis {

/// 四轮索引。全工程统一此顺序：FL, FR, RL, RR。
enum WheelIndex { kFl = 0, kFr = 1, kRl = 2, kRr = 3, kWheelCount = 4 };

// =============================================================================
// 1. 电机端口（HW-003：1=FL 左上、2=RL 左下、3=FR 右上、4=RR 右下）
// =============================================================================
/// 顺序 FL,FR,RL,RR ← 端口 1=FL, 3=FR, 2=RL, 4=RR
inline constexpr std::int8_t kMotorPorts[kWheelCount] = {1, 3, 2, 4};

// =============================================================================
// 2. 几何（ME-004/005/006/007/015/016/017）
// =============================================================================
inline constexpr double kFrameSideM   = 0.255;  ///< 正方形外缘边长 25.5 cm
inline constexpr double kWheelOffsetM = 0.02;   ///< 轮心相对框架边缘外伸 2 cm
/// 底盘中心 → 轮心距离 R = 25.5/2 + 2 = 14.75 cm
inline constexpr double kCenterToWheelM = kFrameSideM / 2.0 + kWheelOffsetM;
/// 车体坐标下轮心坐标绝对值 a = R/√2 ≈ 0.1043 m
inline constexpr double kAxialOffsetM = kCenterToWheelM / 1.41421356237309504880;
/// 车轮有效半径（ME-007：直径 10 cm）
inline constexpr double kWheelRadiusM = 0.05;

// =============================================================================
// 3. 传动（ME-008：直驱 1:1）
// =============================================================================
inline constexpr double kGearRatioMotorPerWheel = 1.0;

// =============================================================================
// 4. 方向符号（KIN-008：只在这里表达反转，禁止改 IK 公式）
// =============================================================================
/// 电机指令方向符号，顺序 FL,FR,RL,RR。T-11 实测哪个轮反向就改哪个。
inline constexpr double kCommandSign[kWheelCount] = {+1.0, +1.0, +1.0, +1.0};

/// 运动学每轮增益/偏置（KIN-009：安装误差标定用），默认 1 / 0。
inline constexpr double kMixScale[kWheelCount]  = {+1.0, +1.0, +1.0, +1.0};
inline constexpr double kMixOffset[kWheelCount] = {+0.0, +0.0, +0.0, +0.0};

// =============================================================================
// 5. 速度限幅与加减速（KIN-007 / DRV-005 / DRV-006）
// =============================================================================
inline constexpr double kMaxWheelRpm = 200.0;              ///< 满速上限（green 200 rpm）
inline constexpr double kSlewRpmPerCycle = 40.0;           ///< 每 20ms 周期最大变化；0=禁用
inline constexpr double kOmegaSlewRadPerSecPerSec = 20.0;  ///< omega 变化率上限

// =============================================================================
// 6. IMU（HW-005 / IMU-011 / IMU-019）
// =============================================================================
inline constexpr std::int8_t kImuPort = 5;
inline constexpr std::uint32_t kImuCalibrationTimeoutMs = 3000;
inline constexpr std::uint32_t kHeadingLatchDelayMs = 5000;  ///< 上电后 5s 锁存目标航向
inline constexpr double kMaxDeltaYawPerCycleRad = 0.35;      ///< 单周期跳变保护

/**
 * VEX IMU 的 heading 以**顺时针为正**（见 pros/imu.hpp 文档），
 * 而本工程 omega 以**逆时针为正**（CS-008）。故用 -1 把 IMU 读数转成工程航向。
 * TODO(CALIBRATE)：IMU 装好后若发现修正方向相反，只改这里的正负号。
 */
inline constexpr double kImuYawSign = -1.0;

// =============================================================================
// 7. 航向 PID（IMU-010，初值均为待整定占位，不是标定结果）
// =============================================================================
inline constexpr double kHeadingKp = 1.0;
inline constexpr double kHeadingKi = 0.0;
inline constexpr double kHeadingKd = 0.0;
inline constexpr double kHeadingIntegralLimit = 0.5;
inline constexpr double kMaxHeadingCorrectionRadPerSec = 2.0;

}  // namespace chassis
