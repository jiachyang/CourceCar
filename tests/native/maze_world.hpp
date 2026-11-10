/**
 * @file tests/native/maze_world.hpp
 * @brief 主机端地图射线世界：用 maze_model.json 生成的 fixture 计算四路测距。
 *
 * 仅测试使用，不参与固件构建。世界坐标与 maze_model.json 一致：
 *   x 向右、y 向下、单位 m；墙段是中心线。
 */
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "distance/distance_config.hpp"
#include "fixtures/maze_model.hpp"
#include "lane/lane_config.hpp"

namespace maze_world {

struct Wall {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

inline std::vector<Wall> fixedWalls() {
    std::vector<Wall> walls;
    walls.reserve(40);
    const double c = maze_fixture::kCellSizeM;
    const double f = maze_fixture::kFieldSizeM;

    for (int i = 0; i < maze_fixture::kFixedVerticalCount; ++i) {
        const auto& s = maze_fixture::kFixedVertical[i];
        walls.push_back(Wall{s.x * c, s.yStart * c, s.x * c, s.yEnd * c});
    }
    for (int i = 0; i < maze_fixture::kFixedHorizontalCount; ++i) {
        const auto& s = maze_fixture::kFixedHorizontal[i];
        walls.push_back(Wall{s.xStart * c, s.y * c, s.xEnd * c, s.y * c});
    }

    // 场地外框：maze_model.json 只记录内部隔板，外框恒存在（3.6 x 3.6 m）。
    walls.push_back(Wall{0.0, 0.0, 0.0, f});
    walls.push_back(Wall{f, 0.0, f, f});
    walls.push_back(Wall{0.0, 0.0, f, 0.0});
    walls.push_back(Wall{0.0, f, f, f});
    return walls;
}

inline void appendMovable(std::vector<Wall>* walls, bool m1, bool m2, bool m3) {
    if (walls == nullptr) {
        return;
    }
    const double c = maze_fixture::kCellSizeM;
    const bool installed[maze_fixture::kMovableCount] = {m1, m2, m3};
    for (int i = 0; i < maze_fixture::kMovableCount; ++i) {
        if (!installed[i]) {
            continue;
        }
        const auto& s = maze_fixture::kMovable[i];
        if (s.orientation == 'V') {
            walls->push_back(Wall{s.position * c, s.start * c, s.position * c,
                                  s.end * c});
        } else {
            walls->push_back(Wall{s.start * c, s.position * c, s.end * c,
                                  s.position * c});
        }
    }
}

/// 最近正向交点距离；无交点返回 +inf。
inline double raycast(const std::vector<Wall>& walls, double x0, double y0,
                      double dx, double dy) {
    double best = std::numeric_limits<double>::infinity();
    for (const Wall& w : walls) {
        const double sx = w.x2 - w.x1;
        const double sy = w.y2 - w.y1;
        const double den = dx * sy - dy * sx;
        if (std::fabs(den) < 1e-12) {
            continue;
        }
        const double ax = w.x1 - x0;
        const double ay = w.y1 - y0;
        const double t = (ax * sy - ay * sx) / den;
        const double u = (ax * dy - ay * dx) / den;
        if (t >= 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9 && t < best) {
            best = t;
        }
    }
    return best;
}

/**
 * @brief 按逻辑传感器布局生成四路测距快照。
 *
 * @param bodyXx/bodyXy 车体 +X（车头）在世界坐标中的单位方向；车体 +Y
 *                      由左手法向 (bodyXy, -bodyXx) 得到（世界 y 向下）。
 */
inline core::RangeSnapshot sensorSnapshot(const std::vector<Wall>& walls,
                                          double centerX, double centerY,
                                          double bodyXx, double bodyXy,
                                          const lane::LaneConfig& config,
                                          std::uint32_t nowMs) {
    core::RangeSnapshot snap{};
    snap.timestampMs = nowMs;

    const double bodyYx = bodyXy;
    const double bodyYy = -bodyXx;

    for (int i = 0; i < lane::kSensorCount; ++i) {
        const lane::SensorConfig& s = config.sensors[i];
        const double ox = centerX + bodyXx * s.xM + bodyYx * s.yM;
        const double oy = centerY + bodyXy * s.xM + bodyYy * s.yM;
        const double c = std::cos(s.axisRad);
        const double sn = std::sin(s.axisRad);
        const double ax = bodyXx * c + bodyYx * sn;
        const double ay = bodyXy * c + bodyYy * sn;

        const double dist = raycast(walls, ox, oy, ax, ay);
        core::RangeSample& sample = snap.samples[s.rangeSource];
        sample.timestampMs = nowMs;
        sample.confidence = 10;  // 近距离实测常为 10，仅诊断用

        if (!std::isfinite(dist) || dist > 2.0) {
            sample.status = core::RangeStatus::kNoTarget;
            sample.distanceMm = distance::kNoTargetMm;
        } else if (dist < 0.02) {
            sample.status = core::RangeStatus::kTooClose;
            sample.distanceMm = static_cast<std::int32_t>(std::lround(dist * 1000.0));
        } else {
            sample.status = core::RangeStatus::kOk;
            sample.distanceMm = static_cast<std::int32_t>(std::lround(dist * 1000.0));
        }
    }
    return snap;
}

inline core::MotionCommand command(double vx, double vy, double omega = 0.0) {
    return core::MotionCommand{vx, vy, omega};
}

}  // namespace maze_world
