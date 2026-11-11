/**
 * @file tests/native/test_lane_state_machine.cpp
 * @brief v2.1 模式机：TRACKING/DEGRADED/TOO_CLOSE/NO_CORRIDOR/
 *        INSUFFICIENT_DATA/NO_PAIR，且不输出任何速度比例。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "lane/lane_state_machine.hpp"

namespace {

lane::LaneStateMachine makeMachine() {
    lane::LaneStateMachine m;
    m.init(lane::makeDefaultLaneConfig());
    return m;
}

lane::LaneStateInputs credibleInputs(std::uint32_t now) {
    lane::LaneStateInputs in{};
    in.enabled = true;
    in.configValid = true;
    in.inputValid = true;
    in.pairValid = true;
    in.leftValid = true;
    in.rightValid = true;
    in.credible = true;
    in.tooClose = false;
    in.bothFresh = true;
    in.pairChanged = false;
    in.nowMs = now;
    return in;
}

}  // namespace

int main() {
    // --- 关闭/输入非法/配置非法 ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(0);
        in.enabled = false;
        auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kDisabled);
        assert(out.reason == lane::LaneReason::kDisabled);
        assert(!out.allowCorrection);
    }
    {
        auto m = makeMachine();
        auto in = credibleInputs(0);
        in.inputValid = false;
        auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kDisabled);
        assert(out.reason == lane::LaneReason::kInputInvalid);
    }
    {
        auto m = makeMachine();
        auto in = credibleInputs(0);
        in.configValid = false;
        auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kDisabled);
        assert(out.reason == lane::LaneReason::kConfigInvalid);
    }

    // --- 无通道方向 -> NO_PAIR，不猜墙 ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(0);
        in.pairValid = false;
        const auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kNoPair);
        assert(!out.allowCorrection);
        assert(out.quality == 0.0);
    }

    // --- 双墙可信且新鲜 -> TRACKING ---
    {
        auto m = makeMachine();
        const auto out = m.update(credibleInputs(20));
        assert(out.mode == lane::LaneMode::kTracking);
        assert(out.reason == lane::LaneReason::kTracking);
        assert(out.allowCorrection);
        assert(!out.decay);
        assert(out.quality == 1.0);
        assert(out.stateChanged);
    }

    // --- 可信但一侧不新鲜 -> DEGRADED，但仍继续算修正 ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(20);
        in.bothFresh = false;
        const auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kDegraded);
        assert(out.reason == lane::LaneReason::kStaleMeasurement);
        assert(out.allowCorrection);
        assert(!out.decay);
        assert(out.degraded);
        assert(out.quality == 0.5);
    }

    // --- 过近 -> TOO_CLOSE，仍允许横向动作 ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(20);
        in.tooClose = true;
        in.bothFresh = false;
        const auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kTooClose);
        assert(out.reason == lane::LaneReason::kTooClose);
        assert(out.allowCorrection);
        assert(out.quality == 0.5);
    }

    // --- 双侧都有数据但几何/分类不可信 -> NO_CORRIDOR ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(20);
        in.credible = false;
        const auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kNoCorridor);
        assert(out.reason == lane::LaneReason::kNoCorridor);
        assert(!out.allowCorrection);
        assert(out.quality == 0.0);
    }

    // --- 一侧没有有效数据 -> INSUFFICIENT_DATA ---
    {
        auto m = makeMachine();
        auto in = credibleInputs(20);
        in.credible = false;
        in.rightValid = false;
        const auto out = m.update(in);
        assert(out.mode == lane::LaneMode::kInsufficientData);
        assert(out.reason == lane::LaneReason::kInsufficientData);
    }

    // --- 丢墙后 150 ms 内缓释旧纠偏，之后释放 ---
    {
        auto m = makeMachine();
        auto out = m.update(credibleInputs(20));
        assert(out.mode == lane::LaneMode::kTracking);

        auto lost = credibleInputs(95);
        lost.credible = false;
        lost.bothFresh = false;
        out = m.update(lost);
        assert(out.mode == lane::LaneMode::kDegraded);
        assert(out.reason == lane::LaneReason::kLostWallHold);
        assert(out.decay);
        assert(out.allowCorrection == false);
        assert(out.decayFactor > 0.49 && out.decayFactor < 0.51);

        lost.nowMs = 170;  // 距上次可信 150 ms：hold 结束
        out = m.update(lost);
        assert(out.mode == lane::LaneMode::kNoCorridor);
        assert(!out.decay);
    }

    // --- 换对后不允许沿用上一对的保持窗口 ---
    {
        auto m = makeMachine();
        m.update(credibleInputs(20));
        auto lost = credibleInputs(40);
        lost.credible = false;
        lost.leftValid = false;
        lost.pairChanged = true;
        const auto out = m.update(lost);
        assert(out.mode == lane::LaneMode::kInsufficientData);
        assert(!out.decay);
    }

    // --- 模式未变化时 stateChanged=false ---
    {
        auto m = makeMachine();
        auto out = m.update(credibleInputs(20));
        assert(out.stateChanged);
        out = m.update(credibleInputs(40));
        assert(!out.stateChanged);
    }

    // --- 重置 -> DISABLED ---
    {
        auto m = makeMachine();
        m.update(credibleInputs(20));
        m.reset();
        assert(m.mode() == lane::LaneMode::kDisabled);
    }

    std::printf("lane state machine tests: ALL PASS\n");
    return 0;
}
