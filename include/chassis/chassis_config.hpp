/**
 * @file chassis/chassis_config.hpp
 * @brief 底盘**模块配置**：端口、几何、传动、方向、限幅、航向 PID。
 *
 * 配置管理位置（ARCH-010）：本文件是底盘硬件参数与底盘级控制参数的唯一来源。
 * - 端口、几何、传动比、方向符号、限幅、航向 PID 均在此定义；
 * - `main` 只持有整车级开关（见 include/config/robot_config.hpp）。
 *
 * 本文件不依赖 PROS，便于主机端单元测试直接包含。
 *
 * 接线真值见 `docs/车辆配置.md`：
 *   1=左前(FL)、2=左后(RL)、3=右前(FR)、4=右后(RR)
 * 车头方向：车体 +X；在框架边长坐标中为 +x 向 +y 偏 45°。
 * 该 45° 不进入 IK：IK 始终以车体 +X=车头、+Y=左侧计算。
 */
#pragma once

#include <cstdint>

namespace chassis {

/// 四轮索引。全工程统一此顺序：FL, FR, RL, RR。
enum WheelIndex { kFl = 0, kFr = 1, kRl = 2, kRr = 3, kWheelCount = 4 };

// =============================================================================
// 1. 电机端口（HW-003：1=FL 左前、2=RL 左后、3=FR 右前、4=RR 右后）
// =============================================================================
inline constexpr std::int8_t kMotorPortFl = 1;  ///< FL
inline constexpr std::int8_t kMotorPortRl = 2;  ///< RL
inline constexpr std::int8_t kMotorPortFr = 3;  ///< FR
inline constexpr std::int8_t kMotorPortRr = 4;  ///< RR

// =============================================================================
// 2. 几何（ME-004/005/006/007/015/016/017）
// =============================================================================
inline constexpr double kFrameSideM   = 0.255;  ///< 正方形外缘边长 25.5 cm
inline constexpr double kWheelOffsetM = 0.02;   ///< 轮心相对框架边缘外伸 2 cm
/// 底盘中心 → 轮心距离 R = 25.5/2 + 2 = 14.75 cm
inline constexpr double kCenterToWheelM = kFrameSideM / 2.0 + kWheelOffsetM;
/// 车轮有效半径（ME-007：直径 10 cm）
inline constexpr double kWheelRadiusM = 0.05;

// =============================================================================
// 3. 传动（ME-008：直驱 1:1）
// =============================================================================
inline constexpr double kGearRatioMotorPerWheel = 1.0;

// =============================================================================
// 4. 速度限幅与加减速（KIN-007 / DRV-005 / DRV-006）
// =============================================================================
inline constexpr double kMaxWheelRpm         = 200.0;  ///< 满速上限（green 200 rpm）
/// 轮速→电压线性系数 [mV/rpm]：12000 mV / 200 rpm(green)。
/// 用 move_voltage 下发，分辨率 ~1/60 rpm，支持小数（move_velocity 只能整数 rpm）。
inline constexpr double kMotorRpmToMillivolt = 60.0;
inline constexpr double kSlewRpmPerCycle     = 40.0;   ///< 每 20ms 周期最大变化；0=禁用
inline constexpr double kOmegaSlewRadPerSecPerSec = 20.0;  ///< omega 变化率上限

// =============================================================================
// 5. 航向保持 PID（IMU-010/011；初值均为待整定占位，不是标定结果）
// =============================================================================
inline constexpr std::uint32_t kHeadingLatchDelayMs = 5000;  ///< 上电后 5s 锁存目标航向
inline constexpr double kHeadingKp = 5.5;
inline constexpr double kHeadingKi = 0.0;
inline constexpr double kHeadingKd = 1.0;
inline constexpr double kHeadingIntegralLimit = 5;
/// P/I/D 单项输出限幅 [rad/s]（对 Kp·err / Ki·∫err / Kd·d(err) 各自限幅）。
inline constexpr double kHeadingPLimit = 10.0;
inline constexpr double kHeadingILimit = 5.0;
inline constexpr double kHeadingDLimit = 5.0;
/// 总输出（P+I+D）限幅 [rad/s]。
inline constexpr double kMaxHeadingCorrectionRadPerSec = 2.0;

// =============================================================================
// 6. 静摩擦前馈（2026-10-24 实测）
// =============================================================================
// 车载地面上、慢速斜坡测得的“刚要转”的 omega 指令（rad/s）。
// 正/负方向不对称，因此分开配置；符号按误差方向使用。
inline constexpr double kHeadingFrictionFeedforwardPos = 0.361;
inline constexpr double kHeadingFrictionFeedforwardNeg = -0.221;
/// 只在接近静止时启用（|gyro z| <= 此值），避免运动中持续加力过冲。
inline constexpr double kHeadingFrictionRateThreshold = 0.05;  // rad/s
/// 误差小于此值不启用前馈，避免目标附近来回抖。
inline constexpr double kHeadingFrictionErrorDeadband = 0.0087;  // rad ≈ 0.5°

/**
 * @brief 底盘模块配置：由 main 在初始化时构造并交给 Chassis。
 *
 * 默认值即当前真值（`makeDefaultChassisConfig()`）。运行时若要换机器人，
 * 只需构造不同的 ChassisConfig，无需改动任何 .cpp。
 */
struct ChassisConfig {
    // 电机端口（顺序 FL,FR,RL,RR）
    std::int8_t motorPorts[kWheelCount] = {kMotorPortFl, kMotorPortFr, kMotorPortRl, kMotorPortRr};

    // 方向符号（KIN-008：只在这里表达反转，禁止改 IK 公式）。T-11 实测哪个轮反向就改哪个。
    double commandSign[kWheelCount] = {+1.0, +1.0, +1.0, +1.0};

    // 运动学每轮增益/偏置（KIN-009：安装误差标定用），默认 1 / 0。
    double mixScale[kWheelCount]  = {+1.0, +1.0, +1.0, +1.0};
    double mixOffset[kWheelCount] = {+0.0, +0.0, +0.0, +0.0};

    // 几何 / 传动
    double frameSideM           = kFrameSideM;
    double wheelOffsetM         = kWheelOffsetM;
    double centerToWheelM       = kCenterToWheelM;
    double wheelRadiusM         = kWheelRadiusM;
    double gearRatioMotorPerWheel = kGearRatioMotorPerWheel;

    // 限幅与加减速
    double maxWheelRpm              = kMaxWheelRpm;
    double motorRpmToMillivolt      = kMotorRpmToMillivolt;
    double slewRpmPerCycle          = kSlewRpmPerCycle;
    double omegaSlewRadPerSecPerSec = kOmegaSlewRadPerSecPerSec;

    // 航向保持
    std::uint32_t headingLatchDelayMs = kHeadingLatchDelayMs;
    double headingKp                  = kHeadingKp;
    double headingKi                  = kHeadingKi;
    double headingKd                  = kHeadingKd;
    double headingIntegralLimit       = kHeadingIntegralLimit;
    double headingPLimit              = kHeadingPLimit;
    double headingILimit              = kHeadingILimit;
    double headingDLimit              = kHeadingDLimit;
    double maxHeadingCorrectionRadPerSec = kMaxHeadingCorrectionRadPerSec;

    // 静摩擦前馈（方向相关；接近静止 + 误差超过死区才启用）
    double headingFrictionFeedforwardPos = kHeadingFrictionFeedforwardPos;
    double headingFrictionFeedforwardNeg = kHeadingFrictionFeedforwardNeg;
    double headingFrictionRateThreshold = kHeadingFrictionRateThreshold;
    double headingFrictionErrorDeadband = kHeadingFrictionErrorDeadband;
};

/// 默认底盘配置（当前真值参数）。
inline ChassisConfig makeDefaultChassisConfig() { return ChassisConfig{}; }

}  // namespace chassis
