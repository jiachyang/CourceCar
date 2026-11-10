/**
 * @file test_heading_controller.cpp
 * @brief 航向 PID 的纯逻辑验证（主机端，不依赖 PROS / 硬件）。
 *
 * 覆盖：主动旋转旁路、航向保持、最短角差、切换不突跳。
 * 编译运行：bash scripts/test_native.sh
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "chassis/heading_controller.hpp"
#include "core/math_util.hpp"

using chassis::HeadingController;

static bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) < eps;
}

int main() {
    constexpr double kPi = 3.14159265358979323846;

    // --- wrapToPi：跨 0/360° 取最短角差 ---
    assert(approx(core::wrapToPi(0.0), 0.0));
    assert(approx(core::wrapToPi(2.0 * kPi + 0.1), 0.1));
    assert(approx(core::wrapToPi(kPi + 0.1), -kPi + 0.1));   // 刚过 π 回到负侧
    assert(approx(core::wrapToPi(-kPi - 0.1), kPi - 0.1));   // 刚过 -π 回到正侧

    // --- 主动旋转：外部 omega ≠ 0 时原样旁路 ---
    {
        HeadingController h;
        h.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(0.0);
        h.setEnabled(true);
        const double out = h.update(0.0, 1.5, 0.02);  // 主动旋转
        assert(approx(out, 1.5));
    }

    // --- 航向保持：目标 0，当前偏 +0.1，修正应为负 ---
    {
        HeadingController h;
        h.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(0.0);
        h.setEnabled(true);
        const double out = h.update(0.1, 0.0, 0.02);  // e = 0 - 0.1 = -0.1
        assert(out < 0.0);
        assert(approx(out, -0.1));
    }

    // --- 修正限幅：误差很大时不超过 maxCorrection ---
    {
        HeadingController h;
        h.setGains(10.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(0.0);
        h.setEnabled(true);
        const double out = h.update(1.0, 0.0, 0.02);
        assert(approx(out, -2.0));  // clamp 到 -maxCorrection
    }

    // --- 跨零：目标 π-0.05，当前 -π+0.05，最短角差应为 -0.1（小量级）---
    {
        HeadingController h;
        h.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(kPi - 0.05);
        h.setEnabled(true);
        const double out = h.update(-kPi + 0.05, 0.0, 0.02);
        assert(out < 0.0 && out > -0.2);
    }

    // --- 切换：主动旋转后立刻回到保持，不应产生巨大 D 项突跳 ---
    {
        HeadingController h;
        h.setGains(1.0, 0.0, 5.0, 0.5, 2.0);  // 大 D 增益，检验切换处理
        h.latchTarget(0.0);
        h.setEnabled(true);
        h.update(0.0, 2.0, 0.02);             // 主动旋转一帧
        const double out = h.update(0.01, 0.0, 0.02);  // 切回保持
        assert(std::fabs(out) <= 2.0);
    }

    // --- 主动旋转结束重锁目标：松手后不再“转回旧目标” ---
    {
        HeadingController h;
        h.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(0.0);   // 初始目标 0
        h.setEnabled(true);
        h.update(0.0, 2.0, 0.02);  // 主动旋转：朝向逐渐转到 1.0 rad
        h.update(0.5, 2.0, 0.02);
        h.update(1.0, 2.0, 0.02);  // 目标应跟随到 1.0
        // 松手、朝向保持在 1.0 → 目标已重锁为 1.0，修正应≈ 0
        const double out = h.update(1.0, 0.0, 0.02);
        assert(std::fabs(out) < 1e-6);
    }

    // --- P/I/D 单项限幅 + 总输出限幅 ---
    {
        HeadingController h;
        h.setGains(10.0, 0.0, 0.0, 0.5, 2.0);   // Kp=10
        h.setTermLimits(0.5, 0.5, 0.5);          // P 项限幅 0.5
        h.latchTarget(0.0);
        h.setEnabled(true);
        const double out = h.update(1.0, 0.0, 0.02);  // err=-1 → Kp·err=-10 → 限到 -0.5
        assert(approx(out, -0.5));
        assert(approx(h.debug().pTerm, -0.5));
        assert(approx(h.debug().pLimit, 0.5));
        assert(approx(h.debug().kp, 10.0));
    }

    // --- 静摩擦前馈：方向相关 + 接近静止 + 误差死区 ---
    {
        HeadingController h;
        h.setGains(0.0, 0.0, 0.0, 0.5, 2.0);
        h.setFrictionFeedforward(0.36, -0.22, 0.05, 0.01);
        h.latchTarget(0.0);
        h.setEnabled(true);

        // err = +0.2 rad，静止 → 正向前馈 +0.36
        const double pos = h.update(-0.2, 0.0, 0.0, 0.02);
        assert(approx(pos, 0.36));
        assert(approx(h.debug().ffTerm, 0.36));
        assert(h.debug().ffActive);

        // 已经有角速度（> 0.05）→ 前馈关闭
        const double moving = h.update(-0.2, 0.0, 0.1, 0.02);
        assert(approx(moving, 0.0));
        assert(approx(h.debug().ffTerm, 0.0));
        assert(!h.debug().ffActive);

        // err = -0.2 rad，静止 → 负向前馈 -0.22
        const double neg = h.update(0.2, 0.0, 0.0, 0.02);
        assert(approx(neg, -0.22));
        assert(approx(h.debug().ffTerm, -0.22));

        // 误差在死区内 → 不启用
        const double dead = h.update(-0.005, 0.0, 0.0, 0.02);
        assert(approx(dead, 0.0));
        assert(approx(h.debug().ffTerm, 0.0));

        // 主动旋转旁路：不叠加前馈
        const double manual = h.update(-0.2, 1.0, 0.0, 0.02);
        assert(approx(manual, 1.0));
        assert(approx(h.debug().ffTerm, 0.0));
    }

    // --- 359° -> 1° / 1° -> 359°：最短角差而不是 358° ---
    {
        const double deg = kPi / 180.0;
        HeadingController h;
        h.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h.latchTarget(1.0 * deg);           // 目标 1°
        h.setEnabled(true);
        // 当前 359°（= -1°）：误差应为 +2°，修正为正（逆时针）
        const double out = h.update(-1.0 * deg, 0.0, 0.02);
        assert(out > 0.0 && out < 0.1);

        HeadingController h2;
        h2.setGains(1.0, 0.0, 0.0, 0.5, 2.0);
        h2.latchTarget(-1.0 * deg);         // 目标 359°
        h2.setEnabled(true);
        const double out2 = h2.update(1.0 * deg, 0.0, 0.02);
        assert(out2 < 0.0 && out2 > -0.1);
    }

    // --- 未使能：原样返回外部 omega ---
    {
        HeadingController h;
        h.latchTarget(0.0);
        h.setEnabled(false);
        assert(approx(h.update(0.5, 0.7, 0.02), 0.7));
    }

    std::printf("heading controller tests: ALL PASS\n");
    return 0;
}
