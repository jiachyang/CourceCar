/**
 * @file tests/native/test_wall_tracker.cpp
 * @brief 10/15 cm 滞回墙体状态机测试（按完整任务书 §5 / §18.1）。
 *
 * 重点：
 *   - 进入严格 <100，退出严格 >150，100–150（含端点）保持；
 *   - 3 次连续有效确认；
 *   - 9999 / >2000 / <20 / PROS_ERR / 未安装 都是无效样本，不确认无墙；
 *   - 无效样本打断连续序列，超时转 Unknown 而非 Absent；
 *   - 同一时刻重复样本不重复计数。
 */
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "lane/wall_state.hpp"

namespace {

constexpr std::uint32_t kDtMs = 20;

lane::WallTracker makeTracker() {
    lane::WallConfig cfg{};
    cfg.enterMm = 100;
    cfg.exitMm = 150;
    cfg.confirmCount = 3;
    cfg.staleTimeoutMs = 200;
    cfg.minSampleIntervalMs = 15;
    lane::WallTracker t;
    t.init(cfg);
    return t;
}

core::RangeSample sample(core::RangeStatus status, std::int32_t mm,
                         std::uint32_t tMs) {
    core::RangeSample s{};
    s.status = status;
    s.distanceMm = mm;
    s.confidence = 10;
    s.timestampMs = tMs;
    return s;
}

void feed(lane::WallTracker& t, core::RangeStatus status, std::int32_t mm,
          int count, std::uint32_t* nowMs) {
    for (int i = 0; i < count; ++i) {
        *nowMs += kDtMs;
        t.update(sample(status, mm, *nowMs), *nowMs);
    }
}

}  // namespace

int main() {
    using core::RangeStatus;

    // --- 初始 Unknown，不虚构距离 ---
    {
        lane::WallTracker t = makeTracker();
        assert(t.status() == lane::WallStatus::kUnknown);
        assert(!t.hasDistance());
        assert(t.lastDistanceMm() == -1);
    }

    // --- 严格 <100：99 累计 3 次 -> Present；100 不确认 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 99, 2, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
        feed(t, RangeStatus::kOk, 99, 1, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        assert(t.lastDistanceMm() == 99);
    }
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 100, 10, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 101, 10, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }

    // --- 严格 >150：150 保持，151 才累计离开 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 90, 3, &now);
        assert(t.status() == lane::WallStatus::kPresent);

        feed(t, RangeStatus::kOk, 120, 10, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 149, 10, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 150, 10, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 151, 2, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 151, 1, &now);
        assert(t.status() == lane::WallStatus::kAbsent);
    }

    // --- 80 -> 160 -> 80：单次 160 不足以确认离开 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 160, 1, &now);
        feed(t, RangeStatus::kOk, 80, 1, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        // 160 的连续计数被 80 打断
        feed(t, RangeStatus::kOk, 160, 2, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        feed(t, RangeStatus::kOk, 80, 1, &now);
        assert(t.status() == lane::WallStatus::kPresent);
    }

    // --- 连续有效才累计：无效样本打断 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 2, &now);
        feed(t, RangeStatus::kReadError, -1, 1, &now);
        feed(t, RangeStatus::kOk, 80, 2, &now);
        feed(t, RangeStatus::kReadError, -1, 1, &now);
        feed(t, RangeStatus::kOk, 80, 1, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }

    // --- 9999 是无效样本：不确认 Absent，超时转 Unknown ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        const std::uint32_t lastValid = now;

        feed(t, RangeStatus::kNoTarget, 9999, 4, &now);
        assert(t.status() == lane::WallStatus::kPresent);
        assert(!t.usable(now, 20));
        assert(t.lastDistanceMm() == 80);
        assert(t.lastValidMs() == lastValid);

        // 继续无效到超过 stale：Unknown，绝不是 Absent
        feed(t, RangeStatus::kNoTarget, 9999, 10, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
        assert(!t.hasDistance());
    }

    // --- >2000 同样按无效处理，不是“远墙”证据 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        feed(t, RangeStatus::kTooFar, 2100, 20, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }

    // --- <20 / PROS_ERR / 未安装：不确认无墙 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        feed(t, RangeStatus::kTooClose, 5, 20, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        feed(t, RangeStatus::kReadError, -1, 20, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 80, 3, &now);
        feed(t, RangeStatus::kNotInstalled, -1, 20, &now);
        assert(t.status() == lane::WallStatus::kUnknown);
    }

    // --- 同一时刻重复样本不重复计数 ---
    {
        lane::WallTracker t = makeTracker();
        const std::uint32_t t0 = 20;
        t.update(sample(RangeStatus::kOk, 80, t0), t0);
        t.update(sample(RangeStatus::kOk, 80, t0), t0);
        t.update(sample(RangeStatus::kOk, 80, t0), t0);
        assert(t.status() == lane::WallStatus::kUnknown);  // 只算 1 次
        t.update(sample(RangeStatus::kOk, 80, t0 + 20), t0 + 20);
        t.update(sample(RangeStatus::kOk, 80, t0 + 40), t0 + 40);
        assert(t.status() == lane::WallStatus::kPresent);
    }

    // --- usable：只表示控制可用，受 hold 限制 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 100;
        feed(t, RangeStatus::kOk, 90, 3, &now);  // now=160
        assert(t.status() == lane::WallStatus::kPresent);
        assert(t.usable(160, 40));
        assert(t.usable(200, 40));
        assert(!t.usable(201, 40));
    }

    // --- reset 清状态 ---
    {
        lane::WallTracker t = makeTracker();
        std::uint32_t now = 0;
        feed(t, RangeStatus::kOk, 90, 3, &now);
        t.reset();
        assert(t.status() == lane::WallStatus::kUnknown);
        assert(!t.hasDistance());
        assert(t.lastDistanceMm() == -1);
    }

    std::printf("wall tracker tests: ALL PASS\n");
    return 0;
}
