/**
 * @file test_radio_mapping.cpp
 * @brief Radio 摇杆映射与死区的纯函数验证（主机端，不依赖 PROS / 硬件）。
 *
 * 编译运行：bash scripts/test_native.sh
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "radio/input_mapping.hpp"

using radio::mapToMotionCommand;
using radio::normalizeStick;

static bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) < eps;
}

int main() {
    constexpr double kDeadband = 0.05;
    constexpr double kLin = 1.0;
    constexpr double kAng = 3.0;

    // --- 死区：死区内归零，端点仍为 ±1 ---
    assert(approx(normalizeStick(0, kDeadband), 0.0));
    assert(approx(normalizeStick(3, kDeadband), 0.0));   // 3/127≈0.024 < 0.05
    assert(approx(normalizeStick(-6, kDeadband), 0.0));
    assert(approx(normalizeStick(127, kDeadband), 1.0));
    assert(approx(normalizeStick(-127, kDeadband), -1.0));
    // 死区外重标定：raw=64 (≈0.504) → (0.504-0.05)/0.95
    const double expected = (64.0 / 127.0 - kDeadband) / (1.0 - kDeadband);
    assert(approx(normalizeStick(64, kDeadband), expected, 1e-9));

    // --- 映射：零输入 → 零命令 ---
    {
        const auto c = mapToMotionCommand(0, 0, 0, kLin, kAng, false, false, false);
        assert(c.isZero());
    }

    // --- 映射：默认符号 ---
    {
        const auto c = mapToMotionCommand(0.0, 1.0, 0.0, kLin, kAng, false, false, false);
        assert(approx(c.vx, kLin) && approx(c.vy, 0.0) && approx(c.omega, 0.0));
    }
    {
        const auto c = mapToMotionCommand(1.0, 0.0, 0.0, kLin, kAng, false, false, false);
        assert(approx(c.vy, -kLin));  // 右推 → 右移 → vy 为负
    }
    {
        const auto c = mapToMotionCommand(0.0, 0.0, 1.0, kLin, kAng, false, false, false);
        assert(approx(c.omega, -kAng));  // 右推 → 顺时针 → omega 为负
    }

    // --- 映射：invert 开关翻转对应分量 ---
    {
        const auto c = mapToMotionCommand(1.0, 1.0, 1.0, kLin, kAng, true, true, true);
        assert(approx(c.vx, -kLin));
        assert(approx(c.vy, kLin));
        assert(approx(c.omega, kAng));
    }

    // --- 映射：满杆缩放 ---
    {
        const auto c = mapToMotionCommand(0.5, -0.5, 0.5, 2.0, 6.0, false, false, false);
        assert(approx(c.vx, -1.0));   // -0.5 * 2
        assert(approx(c.vy, -1.0));   // -0.5 * 2
        assert(approx(c.omega, -3.0));  // -0.5 * 6
    }

    // --- 映射：左摇杆正交优先（单轴锁定，不做斜向平移）---
    {
        // 直上：纯前进
        const auto c = mapToMotionCommand(0.0, 1.0, 0.0, kLin, kAng, false, false, false, true);
        assert(approx(c.vx, kLin) && approx(c.vy, 0.0) && approx(c.omega, 0.0));
    }
    {
        // 直右：纯右移（vy < 0），无前后分量
        const auto c = mapToMotionCommand(1.0, 0.0, 0.0, kLin, kAng, false, false, false, true);
        assert(approx(c.vx, 0.0) && approx(c.vy, -kLin));
    }
    {
        // 45° 斜推（两轴相等）→ 取前后轴，抑制左右
        const auto c = mapToMotionCommand(1.0, 1.0, 0.0, kLin, kAng, false, false, false, true);
        assert(approx(c.vx, kLin) && approx(c.vy, 0.0));
    }
    {
        // 偏右斜推（|x| > |y|）→ 取左右轴，抑制前后
        const auto c = mapToMotionCommand(1.0, 0.5, 0.0, kLin, kAng, false, false, false, true);
        assert(approx(c.vx, 0.0) && approx(c.vy, -kLin));
    }
    {
        // 正交优先不影响自转轴
        const auto c = mapToMotionCommand(0.0, 1.0, 1.0, kLin, kAng, false, false, false, true);
        assert(approx(c.vx, kLin) && approx(c.vy, 0.0) && approx(c.omega, -kAng));
    }
    {
        // 关闭正交优先：斜推仍可走斜向
        const auto c = mapToMotionCommand(1.0, 1.0, 0.0, kLin, kAng, false, false, false, false);
        assert(approx(c.vx, kLin) && approx(c.vy, -kLin));
    }

    std::printf("radio mapping tests: ALL PASS\n");
    return 0;
}
