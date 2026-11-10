/**
 * @file radio/input_mapping.hpp
 * @brief 纯函数：遥控摇杆 → 车体运动命令。不依赖 PROS，可在主机端测试。
 *
 * 把映射从 pros::Controller 读取中分离，Radio 模块只负责“读硬件”，
 * 映射规则集中于此，便于单元测试与标定。
 */
#pragma once

#include <cstdint>

#include "core/types.hpp"

namespace radio {

/**
 * 归一化摇杆原始读数并施加带重标定的死区。
 * @param raw       原始读数 [-127,127]
 * @param deadband  死区（归一化，[0,1)）
 * @return [-1,1]，死区内为 0
 */
double normalizeStick(std::int32_t raw, double deadband);

/**
 * 把归一化摇杆值映射为车体运动命令。
 *
 * 约定（可在 docs/radio_research.md §6 核对）：
 *   vx    = +leftY * maxLinear        （向上为车头）
 *   vy    = -leftX * maxLinear        （向右推 → 车体右侧移，vy<0）
 *   omega = -rightX * maxAngular      （向右推 → 顺时针，omega<0）
 * invert* 分别翻转对应分量。
 *
 * @param snapLeftStickToAxis  true=左摇杆正交优先（单轴锁定）：只保留绝对值
 *                             较大的轴，用作纯 vx 或纯 vy，不做斜向平移。
 *                             右摇杆 omega 不受影响。
 */
core::MotionCommand mapToMotionCommand(double leftX, double leftY, double rightX,
                                       double maxLinearSpeedMps,
                                       double maxAngularSpeedRadPerSec,
                                       bool invertForward, bool invertStrafe,
                                       bool invertTurn,
                                       bool snapLeftStickToAxis = false);

}  // namespace radio
