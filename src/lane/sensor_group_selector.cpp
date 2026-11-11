/**
 * @file lane/sensor_group_selector.cpp
 * @brief 通道测距对选择实现。纯逻辑，不依赖 PROS。
 */
#include "lane/sensor_group_selector.hpp"

#include <cmath>

namespace lane {

void SensorGroupSelector::init(const LaneConfig& config) {
    config_ = config;
}

void SensorGroupSelector::reset() {}

PairSelection SensorGroupSelector::select(double dirX, double dirY,
                                          bool directionValid) const {
    PairSelection out{};
    const double len = std::hypot(dirX, dirY);
    if (!directionValid || !(len > 1e-9) || !std::isfinite(len)) {
        return out;
    }

    const double dx = dirX / len;
    const double dy = dirY / len;
    const double tx = -dy;  // 运动左法向
    const double ty = dx;

    const double scoreFrontBack = std::fabs(tx);
    const double scoreLeftRight = std::fabs(ty);
    const bool frontBack = scoreFrontBack > scoreLeftRight;
    const int group = frontBack ? kGroupA : kGroupB;
    const double score = frontBack ? scoreFrontBack : scoreLeftRight;
    out.score = score;

    if (score < config_.state.minAxisAlignment) {
        return out;
    }

    const int i0 = config_.groupMembers[group][0];
    const int i1 = config_.groupMembers[group][1];
    if (i0 < 0 || i0 >= kSensorCount || i1 < 0 || i1 >= kSensorCount ||
        i0 == i1) {
        return out;
    }
    const SensorConfig& s0 = config_.sensors[i0];
    const SensorConfig& s1 = config_.sensors[i1];
    if (!s0.enabled || !s1.enabled) {
        return out;
    }

    // 传感器轴在运动左法向上的投影：正的一侧是“运动左侧”。
    const double p0 =
        std::cos(s0.axisRad) * tx + std::sin(s0.axisRad) * ty;
    const int left = (p0 > 0.0) ? i0 : i1;
    const int right = (p0 > 0.0) ? i1 : i0;

    out.valid = true;
    out.group = group;
    out.leftSensor = left;
    out.rightSensor = right;
    out.leftNormalX = tx;
    out.leftNormalY = ty;
    return out;
}

}  // namespace lane
