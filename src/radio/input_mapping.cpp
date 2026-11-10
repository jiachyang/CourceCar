/**
 * @file input_mapping.cpp
 * @brief 遥控摇杆映射的纯函数实现（无 PROS 依赖）。
 */
#include "radio/input_mapping.hpp"

#include <cmath>

#include "core/math_util.hpp"

namespace radio {

double normalizeStick(std::int32_t raw, double deadband) {
    double v = core::clampd(static_cast<double>(raw) / 127.0, -1.0, 1.0);
    const double db = core::clampd(deadband, 0.0, 0.99);
    if (std::fabs(v) <= db) {
        return 0.0;
    }
    const double sign = (v > 0.0) ? 1.0 : -1.0;
    return (v - sign * db) / (1.0 - db);
}

core::MotionCommand mapToMotionCommand(double leftX, double leftY, double rightX,
                                       double maxLinearSpeedMps,
                                       double maxAngularSpeedRadPerSec,
                                       bool invertForward, bool invertStrafe,
                                       bool invertTurn,
                                       bool snapLeftStickToAxis) {
    // 左摇杆正交优先：只保留绝对值较大的一个轴，抑制斜向混叠。
    // 推杆直上/直下 → 纯 vx；直左/直右 → 纯 vy；平局（如 45°）取前后轴。
    double lx = leftX;
    double ly = leftY;
    if (snapLeftStickToAxis) {
        if (std::fabs(ly) >= std::fabs(lx)) {
            lx = 0.0;
        } else {
            ly = 0.0;
        }
    }

    const double forward = invertForward ? -ly : ly;
    const double strafe  = invertStrafe ? lx : -lx;
    const double turn    = invertTurn ? rightX : -rightX;

    core::MotionCommand cmd;
    cmd.vx = forward * maxLinearSpeedMps;
    cmd.vy = strafe * maxLinearSpeedMps;
    cmd.omega = turn * maxAngularSpeedRadPerSec;
    return cmd;
}

}  // namespace radio
