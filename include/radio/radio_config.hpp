/**
 * @file radio/radio_config.hpp
 * @brief Radio 模块配置：遥控器摇杆映射、死区、速度缩放与失联保护。
 *
 * 本项目采用 **V5 Controller 遥控输入**模式（不是 VEXlink 双机通信），
 * 调研结论见 docs/radio_research.md。
 *
 * 摇杆正方向（VEX 约定）：左右推杆向右为正，上下推杆向上为正。
 * 摇杆 → 车体命令的映射与符号在此模块内完成，main 不参与。
 */
#pragma once

#include <cstdint>

#include "pros/misc.hpp"

namespace radio {

/// V5 Robot Radio 的 Smart Port（HW-010）。
/// 说明：本工程用 `pros::Controller`（模式 A），无线链路由 VEXos 接管，
/// 该端口号不参与 PROS 调用，仅登记硬件接线真值，便于与需求表核对。
inline constexpr std::int8_t kPort = 5;

inline constexpr double kDeadband                  = 0.05;  ///< 归一化死区 [0,1)
inline constexpr double kMaxLinearSpeedMps         = 1.0;   ///< 满杆平移速度 [m/s]
inline constexpr double kMaxAngularSpeedRadPerSec  = 3.0;   ///< 满杆自转角速度 [rad/s]
inline constexpr std::uint32_t kCommandTimeoutMs   = 100;   ///< 控制命令保鲜期 [ms]
/// 左摇杆正交优先（单轴锁定）：只保留绝对值较大的一个轴，抑制斜向混叠。
/// 目的：左摇杆直上/直下即以纯 vx 前进/后退，便于走直线测试。
inline constexpr bool kSnapLeftStickToAxis = true;

/// Radio 模块配置。
struct RadioConfig {
    std::int8_t port = kPort;  ///< V5 Robot Radio 所在 Smart Port（HW-010，登记用）
    pros::controller_id_e_t masterId = pros::E_CONTROLLER_MASTER;  ///< 主遥控器

    double deadband                 = kDeadband;  ///< 摇杆死区（归一化）
    double maxLinearSpeedMps        = kMaxLinearSpeedMps;      ///< 平移满速
    double maxAngularSpeedRadPerSec = kMaxAngularSpeedRadPerSec;  ///< 旋转满速

    // 实机方向（T-11/R-01~R-03）：前后、左右实测相反，故均置 true。
    bool invertForward = true;   ///< 实测：推杆向上后退 → 反转
    bool invertStrafe  = true;   ///< 实测：推杆左右相反 → 反转
    bool invertTurn    = true;   ///< 实测：右摇杆转向相反 → 反转

    /// 左摇杆正交优先（单轴锁定）：true=不做斜向平移，只走正前/正后/正左/正右。
    bool snapLeftStickToAxis = kSnapLeftStickToAxis;

    std::uint32_t commandTimeoutMs = kCommandTimeoutMs;  ///< 超过此时长视为失联
};

/// 默认 Radio 配置。
inline RadioConfig makeDefaultRadioConfig() { return RadioConfig{}; }

}  // namespace radio
