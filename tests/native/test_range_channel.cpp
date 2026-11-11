/**
 * @file tests/native/test_range_channel.cpp
 * @brief v2.1 单路近壁通道：100/150 分类、MID 初始化、远距有效回波、
 *        丢帧保持与超时、跳变确认、滤波、时间戳去重。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "lane/range_channel.hpp"

namespace {

constexpr std::uint32_t kDt = 20;

lane::RangeChannelConfig testConfig() {
    lane::RangeChannelConfig c{};
    c.enterMm = 100.0;
    c.exitMm = 150.0;
    c.detectMm = 150.0;
    c.confirmFrames = 3;
    c.maxAgeMs = 180;
    c.freshWindowMs = 42;
    c.jumpMm = 60.0;
    c.jumpConfirm = 2;
    c.lowPassAlpha = 1.0;  // 便于验证中值本身
    return c;
}

lane::RangeChannel makeChannel() {
    lane::RangeChannel ch;
    ch.init(testConfig());
    return ch;
}

core::RangeSample sample(core::RangeStatus status, std::int32_t mm,
                         std::uint32_t t) {
    core::RangeSample s{};
    s.status = status;
    s.distanceMm = mm;
    s.confidence = 10;
    s.timestampMs = t;
    return s;
}

void feed(lane::RangeChannel& ch, core::RangeStatus status, std::int32_t mm,
          int count, std::uint32_t* now) {
    for (int i = 0; i < count; ++i) {
        *now += kDt;
        ch.update(sample(status, mm, *now), *now);
    }
}

}  // namespace

int main() {
    using core::RangeStatus;

    // --- 初始 Unknown，不虚构距离 ---
    {
        auto ch = makeChannel();
        const auto v = ch.value(0);
        assert(!v.valid);
        assert(!v.fresh);
        assert(v.state == lane::NearClass::kUnknown);
        assert(ch.rawMm() == -1);
    }

    // --- 严格 <100：99 连续 3 帧 -> NEAR ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 99, 2, &now);
        assert(ch.state() == lane::NearClass::kUnknown);
        feed(ch, RangeStatus::kOk, 99, 1, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);
        const auto v = ch.value(now);
        assert(v.valid && v.fresh);
        assert(std::fabs(v.mm - 99.0) < 1e-9);
        assert(std::fabs(v.rawMm - 99.0) < 1e-9);
    }

    // --- 100–150 mm 首次观测：连续 3 帧 -> MID，不再永久 Unknown ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 120, 2, &now);
        assert(ch.state() == lane::NearClass::kUnknown);
        feed(ch, RangeStatus::kOk, 120, 1, &now);
        assert(ch.state() == lane::NearClass::kMidReturn);
        assert(ch.value(now).valid);
    }

    // --- 150 属于 MID；151 连续 3 帧才 FAR ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 150, 5, &now);
        assert(ch.state() == lane::NearClass::kMidReturn);
        feed(ch, RangeStatus::kOk, 151, 2, &now);
        assert(ch.state() == lane::NearClass::kMidReturn);
        feed(ch, RangeStatus::kOk, 151, 1, &now);
        assert(ch.state() == lane::NearClass::kFarReturn);
    }

    // --- 远距 1.65 m 是有效回波，不是无目标/无墙 ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 1650, 3, &now);
        const auto v = ch.value(now);
        assert(v.valid && v.fresh);
        assert(v.state == lane::NearClass::kFarReturn);
        assert(std::fabs(v.rawMm - 1650.0) < 1e-9);
        assert(std::fabs(v.mm - 1650.0) < 1e-9);
    }

    // --- 近 -> 远单帧不确认；回到近仍保持 NEAR ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 80, 3, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);
        feed(ch, RangeStatus::kOk, 160, 1, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);
        feed(ch, RangeStatus::kOk, 80, 1, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);
        feed(ch, RangeStatus::kOk, 160, 2, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);
        feed(ch, RangeStatus::kOk, 160, 1, &now);
        assert(ch.state() == lane::NearClass::kFarReturn);
    }

    // --- 9999/超量程/读取失败不立刻清状态；超过 maxAge 才 UNKNOWN ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 80, 3, &now);
        const std::uint32_t lastGood = now;
        assert(ch.state() == lane::NearClass::kNearReturn);

        feed(ch, RangeStatus::kNoTarget, 9999, 1, &now);
        assert(ch.state() == lane::NearClass::kNearReturn);  // 保持旧估计
        assert(ch.value(now).valid);
        assert(!ch.value(now).observationValid);

        feed(ch, RangeStatus::kReadError, -1, 1, &now);
        feed(ch, RangeStatus::kTooFar, 2100, 1, &now);
        assert(now - lastGood <= 180);
        assert(ch.state() == lane::NearClass::kNearReturn);

        while (now - lastGood <= 180) {
            now += kDt;
            ch.update(sample(RangeStatus::kNoTarget, 9999, now), now);
        }
        assert(ch.state() == lane::NearClass::kUnknown);
        assert(!ch.value(now).valid);
    }

    // --- 过近样本是有效物理回波，可进入 NEAR ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kTooClose, 12, 3, &now);
        const auto v = ch.value(now);
        assert(v.valid);
        assert(v.observationValid);
        assert(v.state == lane::NearClass::kNearReturn);
        assert(std::fabs(v.rawMm - 12.0) < 1e-9);
    }

    // --- 重复/乱序时间戳不重复计数 ---
    {
        auto ch = makeChannel();
        const std::uint32_t t0 = 20;
        const auto s = sample(RangeStatus::kOk, 80, t0);
        ch.update(s, t0);
        ch.update(s, t0);
        ch.update(s, t0);
        assert(ch.state() == lane::NearClass::kUnknown);  // 只算 1 帧
        ch.update(sample(RangeStatus::kOk, 80, t0 + 20), t0 + 20);
        ch.update(sample(RangeStatus::kOk, 80, t0 + 40), t0 + 40);
        assert(ch.state() == lane::NearClass::kNearReturn);
        ch.update(sample(RangeStatus::kOk, 80, t0 - 20), t0 - 20);  // 乱序
        assert(ch.state() == lane::NearClass::kNearReturn);
    }

    // --- 跳变确认：第一帧大跳变不改变滤波稳态，第二帧才接受 ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 45, 5, &now);
        assert(std::fabs(ch.filteredMm() - 45.0) < 1e-9);
        feed(ch, RangeStatus::kOk, 200, 1, &now);
        assert(std::fabs(ch.filteredMm() - 45.0) < 1e-9);
        assert(ch.state() == lane::NearClass::kNearReturn);  // 分类仍按原始 200
        feed(ch, RangeStatus::kOk, 200, 1, &now);
        assert(std::fabs(ch.filteredMm() - 200.0) < 1e-9);
    }

    // --- 3 点中值 + 一阶低通 ---
    {
        lane::RangeChannelConfig c = testConfig();
        c.lowPassAlpha = 0.5;
        lane::RangeChannel ch;
        ch.init(c);
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 40, 1, &now);
        assert(std::fabs(ch.filteredMm() - 40.0) < 1e-9);
        feed(ch, RangeStatus::kOk, 50, 1, &now);  // 两点中值 45
        assert(std::fabs(ch.filteredMm() - 42.5) < 1e-9);
        feed(ch, RangeStatus::kOk, 60, 1, &now);  // 三点中值 50
        assert(std::fabs(ch.filteredMm() - 46.25) < 1e-9);
    }

    // --- 新鲜度窗口与最大保持时间 ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 45, 3, &now);
        assert(ch.value(now).fresh);
        assert(ch.value(now + 42).fresh);
        assert(!ch.value(now + 43).fresh);
        assert(ch.value(now + 180).valid);
        assert(!ch.value(now + 181).valid);
    }

    // --- reset 清历史 ---
    {
        auto ch = makeChannel();
        std::uint32_t now = 0;
        feed(ch, RangeStatus::kOk, 80, 3, &now);
        ch.reset();
        assert(ch.state() == lane::NearClass::kUnknown);
        assert(ch.rawMm() == -1);
        assert(!ch.value(now).valid);
    }

    std::printf("range channel tests: ALL PASS\n");
    return 0;
}
