/**
 * @file tests/native/test_safety_supervisor.cpp
 * @brief 独立安全层：前向/侧向 ToF 制动、航向门控、故障关闭、透传。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/range_types.hpp"
#include "modules/safety_supervisor.hpp"

namespace {

core::RangeSnapshot snapshot(std::int32_t front, std::int32_t rear,
                             std::int32_t left, std::int32_t right,
                             std::int32_t confidence = 10) {
    core::RangeSnapshot s{};
    s.timestampMs = 20;
    const std::int32_t d[4] = {front, rear, left, right};
    for (int i = 0; i < 4; ++i) {
        s.samples[i].status = (d[i] > 0) ? core::RangeStatus::kOk
                                         : core::RangeStatus::kNoTarget;
        s.samples[i].distanceMm = (d[i] > 0) ? d[i] : 9999;
        s.samples[i].confidence = confidence;
        s.samples[i].timestampMs = 20;
    }
    return s;
}

safety::SafetySupervisor makeSupervisor() {
    safety::SafetySupervisor s;
    s.init(safety::makeDefaultSafetyConfig());
    return s;
}

bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

}  // namespace

int main() {
    // --- 空旷路径不额外限速 ---
    {
        auto s = makeSupervisor();
        const core::MotionCommand cmd{0.55, 0.0, 0.2};
        s.update(cmd, snapshot(800, 800, 800, 800), core::HeadingState{});
        const auto& st = s.getState();
        assert(approx(st.scale, 1.0));
        assert(!st.limited);
        assert(approx(st.command.vx, 0.55));
        assert(approx(st.command.omega, 0.2));
    }

    // --- 正前方 60 mm：必须减速，且只改平移 ---
    {
        auto s = makeSupervisor();
        const core::MotionCommand cmd{0.55, 0.0, 0.2};
        s.update(cmd, snapshot(60, 800, 800, 800), core::HeadingState{});
        const auto& st = s.getState();
        assert(st.limited);
        assert(st.scale > 0.0 && st.scale < 0.7);
        assert(st.command.vx < 0.55);
        assert(st.command.vx > 0.0);
        assert(approx(st.command.omega, 0.2));
        assert(approx(st.nearestMm, 60.0, 1e-9));
    }

    // --- 前方 300 mm：在当前速度下不应额外限速 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.55, 0.0, 0.0},
                 snapshot(300, 800, 800, 800), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
    }

    // --- 侧向运动使用左右 ToF ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.0, -0.55, 0.0},
                 snapshot(800, 800, 800, 60), core::HeadingState{});
        const auto& st = s.getState();
        assert(st.limited);
        assert(st.scale < 0.7);
        assert(approx(st.command.vy, -0.55 * st.scale));
    }

    // --- 45° 斜向：没有足够对齐的传感器，不虚构限速 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.40, 0.40, 0.0},
                 snapshot(60, 800, 800, 60), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
    }

    // --- 低置信度样本不参与安全限速 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.55, 0.0, 0.0},
                 snapshot(60, 800, 800, 800, 3), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
    }

    // --- 无目标/超量程不当作障碍 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.55, 0.0, 0.0},
                 snapshot(-1, -1, -1, -1), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
    }

    // --- 航向门控可配置；默认不因 IMU 无效而停车 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.4, 0.0, 0.1},
                 snapshot(800, 800, 800, 800), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
        assert(!s.getState().headingBlocked);
    }
    {
        safety::SafetyConfig c = safety::makeDefaultSafetyConfig();
        c.requireHeading = true;
        safety::SafetySupervisor s;
        s.init(c);
        core::HeadingState heading{};
        heading.valid = false;
        s.update(core::MotionCommand{0.4, 0.0, 0.1},
                 snapshot(800, 800, 800, 800), heading);
        const auto& st = s.getState();
        assert(st.headingBlocked);
        assert(approx(st.scale, 0.0));
        assert(st.stopped);
        assert(approx(st.command.vx, 0.0));
        assert(approx(st.command.vy, 0.0));
        assert(approx(st.command.omega, 0.1));  // 航向仍由 Chassis 执行
    }

    // --- 零速不强停（避免无意义刹车） ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{0.0, 0.0, 0.2},
                 snapshot(30, 30, 30, 30), core::HeadingState{});
        const auto& st = s.getState();
        assert(approx(st.scale, 1.0));
        assert(approx(st.command.omega, 0.2));
    }

    // --- 关闭：原样透传 ---
    {
        auto s = makeSupervisor();
        s.enable = false;
        s.update(core::MotionCommand{0.55, 0.0, 0.1},
                 snapshot(60, 60, 60, 60), core::HeadingState{});
        assert(approx(s.getState().scale, 1.0));
        assert(approx(s.getState().command.vx, 0.55));
    }

    // --- 配置非法：fail-closed ---
    {
        safety::SafetyConfig c = safety::makeDefaultSafetyConfig();
        c.brakeMps2 = 0.0;
        safety::SafetySupervisor s;
        s.init(c);
        s.update(core::MotionCommand{0.4, 0.0, 0.0},
                 snapshot(800, 800, 800, 800), core::HeadingState{});
        const auto& st = s.getState();
        assert(!st.configValid);
        assert(st.stopped);
        assert(approx(st.command.vx, 0.0));
    }

    // --- 非法输入：停车 ---
    {
        auto s = makeSupervisor();
        s.update(core::MotionCommand{std::nan(""), 0.0, 0.0},
                 snapshot(800, 800, 800, 800), core::HeadingState{});
        assert(s.getState().stopped);
        assert(approx(s.getState().command.vx, 0.0));
    }

    std::printf("safety supervisor tests: ALL PASS\n");
    return 0;
}
