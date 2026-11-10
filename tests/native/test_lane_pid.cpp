/**
 * @file tests/native/test_lane_pid.cpp
 * @brief 墙距 PID 测试：完整 P/I/D、积分门控、限幅、抗饱和、D 低通、复位。
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "lane/pid.hpp"

namespace {

bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

lane::PidConfig baseConfig() {
    lane::PidConfig c{};
    c.kp = 0.0;
    c.ki = 0.0;
    c.kd = 0.0;
    c.integralLimit = 10.0;
    c.pLimit = 100.0;
    c.iLimit = 100.0;
    c.dLimit = 100.0;
    c.outputLimit = 100.0;
    c.derivativeFilterAlpha = 1.0;
    return c;
}

}  // namespace

int main() {
    // --- P 项 ---
    {
        lane::PidConfig c = baseConfig();
        c.kp = 2.0;
        lane::Pid pid;
        pid.init(c);
        assert(approx(pid.update(3.0, 0.02, true), 6.0));
        assert(approx(pid.debug().pTerm, 6.0));
        assert(approx(pid.debug().dTerm, 0.0));
    }

    // --- P 单项限幅 ---
    {
        lane::PidConfig c = baseConfig();
        c.kp = 10.0;
        c.pLimit = 2.0;
        lane::Pid pid;
        pid.init(c);
        assert(approx(pid.update(3.0, 0.02, true), 2.0));
    }

    // --- I 项允许时累积；不允许时冻结 ---
    {
        lane::PidConfig c = baseConfig();
        c.ki = 2.0;
        lane::Pid pid;
        pid.init(c);
        assert(approx(pid.update(1.0, 0.1, true), 0.2, 1e-12));
        assert(approx(pid.debug().integral, 0.1, 1e-12));
        assert(approx(pid.update(1.0, 0.1, false), 0.2, 1e-12));
        assert(approx(pid.debug().integral, 0.1, 1e-12));  // 冻结
        assert(approx(pid.update(1.0, 0.1, true), 0.4, 1e-12));
        assert(approx(pid.debug().integral, 0.2, 1e-12));
    }

    // --- Ki=0：积分状态必须为 0，不允许偷偷累积 ---
    {
        lane::PidConfig c = baseConfig();
        c.ki = 0.0;
        lane::Pid pid;
        pid.init(c);
        for (int i = 0; i < 20; ++i) {
            pid.update(5.0, 0.1, true);
        }
        assert(approx(pid.debug().integral, 0.0));
        assert(approx(pid.debug().iTerm, 0.0));
    }

    // --- 积分状态限幅 ---
    {
        lane::PidConfig c = baseConfig();
        c.ki = 1.0;
        c.integralLimit = 0.5;
        lane::Pid pid;
        pid.init(c);
        for (int i = 0; i < 100; ++i) {
            pid.update(1.0, 0.1, true);
        }
        assert(approx(pid.debug().integral, 0.5, 1e-12));
        assert(approx(pid.debug().iTerm, 0.5, 1e-12));
    }

    // --- D 项：首帧为 0；alpha=1 时直接差分 ---
    {
        lane::PidConfig c = baseConfig();
        c.kd = 1.0;
        lane::Pid pid;
        pid.init(c);
        assert(approx(pid.update(1.0, 0.1, true), 0.0));
        assert(approx(pid.debug().dTerm, 0.0));
        assert(approx(pid.update(2.0, 0.1, true), 10.0, 1e-12));
        assert(approx(pid.debug().dTerm, 10.0, 1e-12));
    }

    // --- D 一阶低通 alpha=0.5 ---
    {
        lane::PidConfig c = baseConfig();
        c.kd = 1.0;
        c.derivativeFilterAlpha = 0.5;
        lane::Pid pid;
        pid.init(c);
        pid.update(1.0, 0.1, true);
        assert(approx(pid.update(2.0, 0.1, true), 5.0, 1e-12));
        assert(approx(pid.update(3.0, 0.1, true), 7.5, 1e-12));
    }

    // --- 抗饱和：顶到输出限幅后积分不再增长 ---
    {
        lane::PidConfig c = baseConfig();
        c.ki = 1.0;
        c.outputLimit = 1.0;
        c.integralLimit = 100.0;
        lane::Pid pid;
        pid.init(c);
        for (int i = 0; i < 20; ++i) {
            const double out = pid.update(10.0, 0.1, true);
            assert(approx(out, 1.0, 1e-12));
            if (i > 0) {
                assert(pid.debug().saturated);
            }
        }
        assert(approx(pid.debug().integral, 1.0, 1e-12));
    }

    // --- 复位：积分和 D 历史都清空 ---
    {
        lane::PidConfig c = baseConfig();
        c.ki = 1.0;
        c.kd = 1.0;
        lane::Pid pid;
        pid.init(c);
        pid.update(1.0, 0.1, true);
        pid.update(2.0, 0.1, true);
        assert(pid.debug().integral > 0.0);
        pid.reset();
        assert(approx(pid.debug().integral, 0.0));
        assert(approx(pid.debug().dTerm, 0.0));
        assert(approx(pid.update(1.0, 0.1, true), 0.1, 1e-12));
        assert(approx(pid.debug().dTerm, 0.0));
    }

    // --- 非法输入不产生输出，且清历史 ---
    {
        lane::PidConfig c = baseConfig();
        c.kp = 1.0;
        lane::Pid pid;
        pid.init(c);
        assert(approx(pid.update(1.0, 0.0, true), 0.0));
        assert(approx(pid.update(1.0, -0.1, true), 0.0));
        assert(approx(pid.update(1.0, 2.0, true), 0.0));
        assert(approx(pid.update(NAN, 0.02, true), 0.0));
        assert(approx(pid.update(1.0, 0.02, true), 1.0));
    }

    std::printf("lane pid tests: ALL PASS\n");
    return 0;
}
