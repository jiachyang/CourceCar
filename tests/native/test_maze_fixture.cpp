/**
 * @file tests/native/test_maze_fixture.cpp
 * @brief 用 maze_model.json 生成的 fixture 做地图/射线/车道保持模拟测试。
 *
 * 数据来源：test/data/maze_model.json（与 /home/jc/下载/maze_model.json 同一份）。
 * fixture 由 scripts/gen_maze_fixture.py 生成；test_native.sh 会 --check 防漂移。
 *
 * 本测试证明：
 *   - 固定墙体/活动隔板数量与坐标直接来自 JSON，而不是手绘；
 *   - S 点四方向射线与已知基准一致；
 *   - 45 cm 真实走廊中，LaneKeeper 能看到左右近墙并完成双墙居中；
 *   - M1/M2/M3 的安装/拆除会改变对应射线。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "core/types.hpp"
#include "fixtures/maze_model.hpp"
#include "lane/lane_config.hpp"
#include "maze_world.hpp"
#include "modules/lane_keeper.hpp"

namespace {

bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

double ray(double x, double y, double dx, double dy) {
    return maze_world::raycast(maze_world::fixedWalls(), x, y, dx, dy);
}

const maze_fixture::Site& site(const char* id) {
    for (int i = 0; i < maze_fixture::kSiteCount; ++i) {
        if (std::string(maze_fixture::kSites[i].id) == id) {
            return maze_fixture::kSites[i];
        }
    }
    assert(false);
    return maze_fixture::kSites[0];
}

}  // namespace

int main() {
    // --- 数量与基础参数：直接来自 JSON ---
    assert(maze_fixture::kCols == 8);
    assert(maze_fixture::kRows == 8);
    assert(approx(maze_fixture::kCellSizeM, 0.45));
    assert(approx(maze_fixture::kFieldSizeM, 3.6));
    assert(maze_fixture::kFixedVerticalCount == 16);
    assert(maze_fixture::kFixedHorizontalCount == 11);
    assert(maze_fixture::kFixedVerticalCount +
               maze_fixture::kFixedHorizontalCount ==
           27);  // 27 条内部固定墙；外框 4 条由 maze_world 另加
    assert(maze_fixture::kMovableCount == 3);
    assert(maze_fixture::kSiteCount == 5);

    // 27 条内部固定墙 + 4 条场地外框 = 31 条固定边界（与旧任务书的 31 一致）
    {
        auto all = maze_world::fixedWalls();
        assert(all.size() == 31);
        maze_world::appendMovable(&all, true, true, true);
        assert(all.size() == 34);
    }

    // --- 站点坐标 ---
    const auto& s = site("S");
    assert(s.cellX == 4 && s.cellY == 7);
    assert(approx(s.xM, 2.025) && approx(s.yM, 3.375));
    const auto& f = site("F");
    assert(f.cellX == 3 && f.cellY == 0);
    assert(approx(f.xM, 1.575) && approx(f.yM, 0.225));

    // --- S 点、车头朝北、四束从车体中心发出：已知基准 ---
    // front/right/back/left = 0.675 / 0.225 / 0.225 / 0.675 m
    {
        const auto& p = site("S");
        assert(approx(ray(p.xM, p.yM, 0.0, -1.0), 0.675, 1e-9));
        assert(approx(ray(p.xM, p.yM, 1.0, 0.0), 0.225, 1e-9));
        assert(approx(ray(p.xM, p.yM, 0.0, 1.0), 0.225, 1e-9));
        assert(approx(ray(p.xM, p.yM, -1.0, 0.0), 0.675, 1e-9));
    }

    // --- 活动隔板：安装/拆除改变射线 ---
    {
        auto walls = maze_world::fixedWalls();
        // M1 = x=0.45, y=1.35..1.80
        assert(approx(maze_world::raycast(walls, 0.225, 1.575, 1.0, 0.0),
                      1.125, 1e-9));
        auto withM1 = walls;
        maze_world::appendMovable(&withM1, true, false, false);
        assert(approx(maze_world::raycast(withM1, 0.225, 1.575, 1.0, 0.0),
                      0.225, 1e-9));

        // M2 = x=2.25, y=1.80..2.25
        assert(approx(maze_world::raycast(walls, 2.025, 2.025, 1.0, 0.0),
                      1.125, 1e-9));
        auto withM2 = walls;
        maze_world::appendMovable(&withM2, false, true, false);
        assert(approx(maze_world::raycast(withM2, 2.025, 2.025, 1.0, 0.0),
                      0.225, 1e-9));

        // M3 = y=2.25, x=2.70..3.15；拆除后命中 y=3.15 的横墙，距离 1.125 m
        assert(approx(maze_world::raycast(walls, 2.925, 2.025, 0.0, 1.0),
                      1.125, 1e-9));
        auto withM3 = walls;
        maze_world::appendMovable(&withM3, false, false, true);
        assert(approx(maze_world::raycast(withM3, 2.925, 2.025, 0.0, 1.0),
                      0.225, 1e-9));
    }

    // --- 真实 45 cm 走廊：cell (7,3)，车头朝北 ---
    // 传感器在四个顶点（半径 0.1803 m），左右墙原始距离约 45 mm。
    const double cx = 3.375;  // (7+0.5)*0.45
    const double cy = 1.575;  // (3+0.5)*0.45
    const auto walls = maze_world::fixedWalls();

    lane::LaneConfig cfg = lane::makeDefaultLaneConfig();
    cfg.layoutVerified = true;
    cfg.state.acquireTimeMs = 1;
    cfg.state.acquireSpeedScale = 1.0;
    cfg.state.maxSpeedScaleRatePerSec = 1000.0;
    cfg.state.maxLateralAccelMps2 = 1000.0;
    cfg.state.motionStopMm = 0.0;
    cfg.state.motionSlowMm = 1.0;
    cfg.state.maxLateralCorrectionMps = 0.5;
    cfg.wallPid.kp = 0.01;
    cfg.wallPid.outputLimit = 1.0;
    cfg.filter.lowPassAlpha = 1.0;

    const auto firstSnap = maze_world::sensorSnapshot(
        walls, cx, cy, 0.0, -1.0, cfg, 20);  // 车体 +X 朝北
    assert(firstSnap.samples[core::kRangeLeft].status ==
           core::RangeStatus::kOk);
    assert(firstSnap.samples[core::kRangeRight].status ==
           core::RangeStatus::kOk);
    assert(std::abs(firstSnap.samples[core::kRangeLeft].distanceMm - 45) <= 1);
    assert(std::abs(firstSnap.samples[core::kRangeRight].distanceMm - 45) <= 1);

    lane::LaneKeeper keeper;
    keeper.init(cfg);
    keeper.enable = true;

    std::uint32_t nowMs = 20;
    lane::LaneKeeperState st{};
    for (int i = 0; i < 3; ++i) {
        const auto snap = maze_world::sensorSnapshot(
            walls, cx, cy, 0.0, -1.0, cfg, nowMs);
        keeper.update(core::MotionCommand{0.30, 0.0, 0.0}, snap,
                      core::HeadingState{}, 0.02);
        st = keeper.getState();
        nowMs += 20;
    }
    assert(st.configValid);
    assert(st.group == lane::kGroupB);  // 前进用左右组
    assert(st.state == lane::LaneState::kDualWall);
    assert(st.leftObserved && st.rightObserved);
    assert(approx(st.leftDistanceMm, 45.0, 1.0));
    assert(approx(st.rightDistanceMm, 45.0, 1.0));
    assert(approx(st.corridorWidthM,
                  2.0 * lane::kDefaultSensorRadiusM + 0.090, 0.001));
    assert(approx(st.lateralErrorMm, 0.0, 0.5));

    // --- 传感器轴略偏离法向：参考垂距仍应还原约 0.225 m ---
    {
        lane::LaneConfig tilted = cfg;
        tilted.sensors[lane::kS1].axisRad = lane::kPi / 2.0 + 0.15;
        tilted.sensors[lane::kS3].axisRad = -lane::kPi / 2.0 + 0.15;
        assert(lane::isValidLaneConfig(tilted));
        lane::LaneKeeper k2;
        k2.init(tilted);
        k2.enable = true;
        lane::LaneKeeperState s2{};
        std::uint32_t t2 = 20;
        for (int i = 0; i < 3; ++i) {
            const auto snap = maze_world::sensorSnapshot(
                walls, cx, cy, 0.0, -1.0, tilted, t2);
            k2.update(core::MotionCommand{0.30, 0.0, 0.0}, snap,
                      core::HeadingState{}, 0.02);
            s2 = k2.getState();
            t2 += 20;
        }
        assert(s2.state == lane::LaneState::kDualWall);
        assert(std::fabs(s2.leftReferenceM - 0.225) <= 0.002);
        assert(std::fabs(s2.rightReferenceM - 0.225) <= 0.002);
    }

    // 车体向西偏 10 mm：左约 35 mm、右约 55 mm；右墙更远 -> 向右修正
    const double offsetCx = cx - 0.01;
    for (int i = 0; i < 2; ++i) {
        nowMs += 20;
        const auto fresh = maze_world::sensorSnapshot(
            walls, offsetCx, cy, 0.0, -1.0, cfg, nowMs);
        keeper.update(core::MotionCommand{0.30, 0.0, 0.0}, fresh,
                      core::HeadingState{}, 0.02);
        st = keeper.getState();
    }
    assert(st.state == lane::LaneState::kDualWall);
    assert(st.lateralErrorMm > 0.0);       // 右墙更远 -> 向右
    assert(st.lateralCorrectionMps > 0.0);
    assert(st.command.vy < 0.0);           // 车体 +Y 左，负为右
    assert(approx(st.command.vx, 0.30, 0.01));

    std::printf("maze fixture tests: ALL PASS\n");
    return 0;
}
