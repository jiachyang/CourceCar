/**
 * @file tests/native/test_range_filter.cpp
 * @brief 测距滤波测试：中值、低通、跳变确认、无目标不混入、超时复位。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "lane/range_filter.hpp"

namespace {

core::RangeSample ok(std::int32_t mm, std::uint32_t t) {
    core::RangeSample s{};
    s.status = core::RangeStatus::kOk;
    s.distanceMm = mm;
    s.timestampMs = t;
    return s;
}

core::RangeSample noTarget(std::uint32_t t) {
    core::RangeSample s{};
    s.status = core::RangeStatus::kNoTarget;
    s.distanceMm = 9999;
    s.timestampMs = t;
    return s;
}

lane::RangeFilter makeFilter(double alpha = 1.0) {
    lane::RangeFilterConfig c{};
    c.lowPassAlpha = alpha;
    c.jumpThresholdMm = 60.0;
    c.jumpConfirmSamples = 2;
    c.resetAfterNoDistanceMs = 80;
    lane::RangeFilter f;
    f.init(c);
    return f;
}

bool near(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

}  // namespace

int main() {
    // --- 初始无有效值 ---
    {
        lane::RangeFilter f = makeFilter();
        assert(!f.hasFiltered());
        assert(!f.valid(20, 40));
        assert(f.rawMm() == -1);
    }

    // --- 中值 + 低通：alpha=1 时取中值 ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        f.update(ok(90, 20), 20);
        f.update(ok(90, 40), 40);
        f.update(ok(92, 60), 60);
        assert(near(f.filteredMm(), 90.0));
        assert(f.valid(60, 40));
        assert(f.rawMm() == 92);
    }

    // --- 单点孤立异常不改变稳态 ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        f.update(ok(90, 20), 20);
        f.update(ok(90, 40), 40);
        f.update(ok(90, 60), 60);
        f.update(ok(200, 80), 80);  // 单点 200 超出 60 mm 跳变阈值
        assert(!near(f.filteredMm(), 200.0));
        assert(f.rawMm() == 200);   // 原始值仍可见
        f.update(ok(91, 100), 100);
        assert(near(f.filteredMm(), 90.0));
    }

    // --- 连续跳变才接受新稳态 ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        f.update(ok(90, 20), 20);
        f.update(ok(90, 40), 40);
        f.update(ok(90, 60), 60);
        f.update(ok(200, 80), 80);
        f.update(ok(200, 100), 100);
        assert(near(f.filteredMm(), 200.0));
    }

    // --- 9999 不进入滤波，超时后无效 ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        f.update(ok(90, 20), 20);
        f.update(ok(90, 40), 40);
        assert(f.valid(40, 40));
        f.update(noTarget(60), 60);
        assert(near(f.filteredMm(), 90.0));
        assert(f.valid(60, 40));
        assert(!f.valid(101, 40));
        // 无目标累计超过 resetAfterNoDistanceMs -> 清滤波
        f.update(noTarget(120), 120);
        assert(f.hasFiltered());  // 60 ms < 80 ms
        f.update(noTarget(140), 140);
        assert(!f.hasFiltered());
    }

    // --- 相同数值连续出现是有效的（静止墙） ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        for (int i = 0; i < 5; ++i) {
            const std::uint32_t t = static_cast<std::uint32_t>(20 + 20 * i);
            f.update(ok(80, t), t);
        }
        assert(f.hasFiltered());
        assert(near(f.filteredMm(), 80.0));
    }

    // --- alpha=0：首值后保持不变 ---
    {
        lane::RangeFilter f = makeFilter(0.0);
        f.update(ok(100, 20), 20);
        f.update(ok(80, 40), 40);
        f.update(ok(80, 60), 60);
        assert(near(f.filteredMm(), 100.0));
    }

    // --- rawValid 与 filtered valid 分开 ---
    {
        lane::RangeFilter f = makeFilter(1.0);
        f.update(ok(90, 20), 20);
        assert(f.rawValid(20, 40));
        assert(!f.rawValid(61, 40));
        assert(f.valid(20, 40));
    }

    std::printf("range filter tests: ALL PASS\n");
    return 0;
}
