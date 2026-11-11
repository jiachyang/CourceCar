/**
 * @file tests/native/test_sensor_group_selector.cpp
 * @brief v2.1 通道选对：前后运动用左右组、横移用前后组、斜向不硬选。
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "lane/geometry.hpp"
#include "lane/sensor_group_selector.hpp"

namespace {

lane::SensorGroupSelector makeSelector() {
    lane::LaneConfig c = lane::makeDefaultLaneConfig();
    lane::SensorGroupSelector s;
    s.init(c);
    return s;
}

}  // namespace

int main() {
    // --- 前进：Group B，左=S1，右=S3，法向 +Y ---
    {
        auto s = makeSelector();
        const auto p = s.select(0.30, 0.0, true);
        assert(p.valid);
        assert(p.group == lane::kGroupB);
        assert(p.leftSensor == lane::kS1);
        assert(p.rightSensor == lane::kS3);
        assert(std::fabs(p.leftNormalX) < 1e-12);
        assert(std::fabs(p.leftNormalY - 1.0) < 1e-12);
    }

    // --- 后退：Group B，但左右对调 ---
    {
        auto s = makeSelector();
        const auto p = s.select(-0.30, 0.0, true);
        assert(p.group == lane::kGroupB);
        assert(p.leftSensor == lane::kS3);
        assert(p.rightSensor == lane::kS1);
    }

    // --- 左移：Group A，运动左=S2(后)，右=S0(前) ---
    {
        auto s = makeSelector();
        const auto p = s.select(0.0, 0.30, true);
        assert(p.group == lane::kGroupA);
        assert(p.leftSensor == lane::kS2);
        assert(p.rightSensor == lane::kS0);
        assert(std::fabs(p.leftNormalX + 1.0) < 1e-12);
    }

    // --- 右移：Group A，运动左=S0(前)，右=S2(后) ---
    {
        auto s = makeSelector();
        const auto p = s.select(0.0, -0.30, true);
        assert(p.group == lane::kGroupA);
        assert(p.leftSensor == lane::kS0);
        assert(p.rightSensor == lane::kS2);
        assert(std::fabs(p.leftNormalX - 1.0) < 1e-12);
    }

    // --- 45° 斜向：两组都不到 0.82 -> 不选 ---
    {
        auto s = makeSelector();
        const auto p = s.select(0.30, 0.30, true);
        assert(!p.valid);
        assert(p.group == -1);
        assert(p.leftSensor == -1);
    }

    // --- 30°：|t·左/右轴| ≈ 0.866 >= 0.82 -> 仍可选 B ---
    {
        auto s = makeSelector();
        const double a = 30.0 * lane::kPi / 180.0;
        const auto p = s.select(std::cos(a), std::sin(a), true);
        assert(p.valid);
        assert(p.group == lane::kGroupB);
        assert(p.score >= 0.82);
    }

    // --- 无方向/零向量：不选组 ---
    {
        auto s = makeSelector();
        auto p = s.select(0.30, 0.0, true);
        assert(p.valid);
        p = s.select(0.0, 0.0, false);
        assert(!p.valid);
        p = s.select(0.0, 0.0, false);
        assert(!p.valid);
        p = s.select(0.0, 0.0, true);
        assert(!p.valid);
    }

    // --- 同一方向重复选择结果稳定；换方向换组 ---
    {
        auto s = makeSelector();
        auto p1 = s.select(0.30, 0.0, true);
        auto p2 = s.select(0.30, 0.0, true);
        assert(p1.valid && p2.valid);
        assert(p1.group == p2.group);
        assert(p1.leftSensor == p2.leftSensor);
        auto p3 = s.select(0.0, 0.30, true);
        assert(p3.valid);
        assert(p3.group == lane::kGroupA);
    }

    // --- 世界方向先转车体：转 90° 后世界 +X 变成车体 -Y（右移） ---
    {
        const core::MotionCommand world{0.30, 0.0, 0.0};
        const core::MotionCommand body =
            lane::worldVelocityToBody(world, lane::kPi / 2.0);
        assert(std::fabs(body.vx) < 1e-12);
        assert(body.vy < 0.0);
        auto s = makeSelector();
        const auto p = s.select(body.vx, body.vy, true);
        assert(p.group == lane::kGroupA);
    }

    std::printf("sensor group selector tests: ALL PASS\n");
    return 0;
}
