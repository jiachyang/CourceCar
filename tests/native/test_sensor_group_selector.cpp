/**
 * @file tests/native/test_sensor_group_selector.cpp
 * @brief 测距组选择测试：前后=A、左右=B，前进/后退/横移/斜向/零速/转弯。
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
    // --- 前进：左右组 B，left=S1(左)，right=S3(右) ---
    {
        auto s = makeSelector();
        const auto g = s.select(0.30, 0.0, 20, false);
        assert(g.group == lane::kGroupB);
        assert(g.leftSensor == lane::kS1);
        assert(g.rightSensor == lane::kS3);
        assert(g.changed);
    }

    // --- 左移：前后组 A，运动左侧=后(S2)，右侧=前(S0) ---
    {
        auto s = makeSelector();
        const auto g = s.select(0.0, 0.30, 20, false);
        assert(g.group == lane::kGroupA);
        assert(g.leftSensor == lane::kS2);
        assert(g.rightSensor == lane::kS0);
    }

    // --- 右移：前后组 A，运动左侧=前(S0)，右侧=后(S2) ---
    {
        auto s = makeSelector();
        const auto g = s.select(0.0, -0.30, 20, false);
        assert(g.group == lane::kGroupA);
        assert(g.leftSensor == lane::kS0);
        assert(g.rightSensor == lane::kS2);
    }

    // --- 后退：左右组 B，但左右对调 ---
    {
        auto s = makeSelector();
        const auto g = s.select(-0.30, 0.0, 20, false);
        assert(g.group == lane::kGroupB);
        assert(g.leftSensor == lane::kS3);  // 车体右 = 倒车左侧
        assert(g.rightSensor == lane::kS1); // 车体左 = 倒车右侧
    }

    // --- 45° 斜向：两组都不到 0.90 -> 不选 ---
    {
        auto s = makeSelector();
        const auto g = s.select(0.30, 0.30, 20, false);
        assert(g.group == -1);
        assert(g.leftSensor == -1);
        assert(g.rightSensor == -1);
    }

    // --- 零速：不选组，且第二次不重复报 changed ---
    {
        auto s = makeSelector();
        auto g = s.select(0.30, 0.0, 20, false);
        assert(g.group == lane::kGroupB);
        g = s.select(0.0, 0.0, 40, false);
        assert(g.group == -1);
        assert(g.changed);
        g = s.select(0.0, 0.0, 60, false);
        assert(g.group == -1);
        assert(!g.changed);
        g = s.select(0.30, 0.0, 80, false);
        assert(g.group == lane::kGroupB);
        assert(g.changed);
    }

    // --- 转弯：不选组 ---
    {
        auto s = makeSelector();
        s.select(0.30, 0.0, 20, false);
        const auto g = s.select(0.30, 0.0, 40, true);
        assert(g.group == -1);
        assert(g.changed);
    }

    // --- 车体坐标 vs 世界坐标 ---
    {
        // 车体原地转 90° 后仍沿车头前进：车体命令不变 -> 仍选左右组 B
        auto s = makeSelector();
        const auto g = s.select(0.30, 0.0, 20, false);
        assert(g.group == lane::kGroupB);
    }
    {
        // 世界 +X 固定，车体转 90° -> 车体命令向右 -> 前后组 A
        auto s = makeSelector();
        const auto world = core::MotionCommand{0.30, 0.0, 0.0};
        const auto body = lane::worldVelocityToBody(
            world, 1.5707963267948966);
        assert(std::fabs(body.vx) < 1e-12);
        assert(body.vy < 0.0);
        const auto g = s.select(body.vx, body.vy, 20, false);
        assert(g.group == lane::kGroupA);
    }

    std::printf("sensor group selector tests: ALL PASS\n");
    return 0;
}
