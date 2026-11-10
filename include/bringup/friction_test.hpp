/**
 * @file bringup/friction_test.hpp
 * @brief 台架/架空车轮阶段的临时调试流程（不属于正常控制链路）。
 *
 * 本模块只放两类 bring-up 代码：
 *   - 静摩擦/前馈自动测定（输出 $F 帧，配套 test/friction_test.py）；
 *   - 底盘点动自检序列（T-01~T-06，autonomous 中使用）。
 *
 * 正常比赛/控制流程不调用本模块；main 只保留调用点与一个编译期开关。
 */
#pragma once

#include "modules/chassis.hpp"
#include "modules/imu.hpp"

namespace bringup {

/// 编译期开关：true → opcontrol 只跑静摩擦标定（架空车轮），
/// 测完改回 false 并重新烧录。
inline constexpr bool kFrictionTestEnabled = false;

/**
 * @brief 静摩擦/前馈自动测定（临时测试模式）。
 *
 * 原理：航向保持关闭，只给纯旋转 omega 指令，按很慢的斜率增加；
 * 用 IMU 的 rotation 变化 + gyro z 速率判断车体是否真正开始旋转。
 * 记录“刚要动”的 omega 指令，作为静态摩擦前馈 F（rad/s）。
 * 输出格式见 test/friction_test.py。
 */
void runFrictionTest(chassis::Chassis& chassis, imu::Imu& imu);

/**
 * @brief 底盘点动自检（T-01~T-06）：架空车轮时验证四轮方向。
 *
 * 逐步下发前/后/左/右/自转/混合命令，每步 800 ms，步间刹车 300 ms。
 */
void runStaticTestSequence(chassis::Chassis& chassis);

}  // namespace bringup
