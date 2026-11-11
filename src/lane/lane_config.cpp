/**
 * @file lane/lane_config.cpp
 * @brief LaneKeeper v2.1 配置静态校验。纯逻辑，不依赖 PROS。
 */
#include "lane/lane_config.hpp"

#include <cmath>

namespace lane {
namespace {

bool finite(double v) { return std::isfinite(v); }

bool validPid(const PidConfig& c) {
    if (!finite(c.kp) || !finite(c.ki) || !finite(c.kd)) {
        return false;
    }
    if (!finite(c.integralLimit) || c.integralLimit < 0.0) {
        return false;
    }
    if (!finite(c.pLimit) || !finite(c.iLimit) || !finite(c.dLimit) ||
        c.pLimit < 0.0 || c.iLimit < 0.0 || c.dLimit < 0.0) {
        return false;
    }
    if (!finite(c.outputLimit) || c.outputLimit <= 0.0) {
        return false;
    }
    if (!finite(c.derivativeFilterAlpha) || c.derivativeFilterAlpha < 0.0 ||
        c.derivativeFilterAlpha > 1.0) {
        return false;
    }
    return true;
}

bool validChannel(const RangeChannelConfig& c) {
    if (!finite(c.enterMm) || c.enterMm <= 0.0) {
        return false;
    }
    if (!finite(c.exitMm) || c.exitMm <= c.enterMm) {
        return false;
    }
    if (!finite(c.detectMm) || c.detectMm < c.enterMm ||
        c.detectMm > c.maxValidMm) {
        return false;
    }
    if (c.confirmFrames < 1 || c.jumpConfirm < 1) {
        return false;
    }
    if (!finite(c.minValidMm) || c.minValidMm < 0.0 ||
        !finite(c.maxValidMm) || c.maxValidMm <= c.minValidMm) {
        return false;
    }
    if (c.maxAgeMs == 0 || c.freshWindowMs == 0 ||
        c.freshWindowMs > c.maxAgeMs) {
        return false;
    }
    if (!finite(c.jumpMm) || c.jumpMm < 0.0) {
        return false;
    }
    if (!finite(c.lowPassAlpha) || c.lowPassAlpha < 0.0 ||
        c.lowPassAlpha > 1.0) {
        return false;
    }
    return true;
}

bool validState(const LaneStateConfig& c) {
    const double values[] = {
        c.minCorridorWidthM, c.maxCorridorWidthM, c.tooCloseMm,
        c.repulseMps, c.maxCorrectionMps, c.maxLateralAccelMps2,
        c.minAxisAlignment, c.minDirectionSpeedMps,
        c.standstillMinCorrectionMps, c.standstillDeadbandMm,
    };
    for (double v : values) {
        if (!finite(v)) {
            return false;
        }
    }
    if (c.minCorridorWidthM <= 0.0 ||
        c.maxCorridorWidthM <= c.minCorridorWidthM) {
        return false;
    }
    if (c.tooCloseMm <= 0.0) {
        return false;
    }
    if (c.repulseMps <= 0.0 ||
        c.maxCorrectionMps <= 0.0 ||
        c.repulseMps > c.maxCorrectionMps) {
        return false;
    }
    if (c.maxLateralAccelMps2 <= 0.0) {
        return false;
    }
    if (c.minAxisAlignment <= 0.0 || c.minAxisAlignment > 1.0) {
        return false;
    }
    if (c.minDirectionSpeedMps < 0.0) {
        return false;
    }
    if (c.standstillMinCorrectionMps < 0.0 ||
        c.standstillMinCorrectionMps > c.maxCorrectionMps) {
        return false;
    }
    if (c.standstillDeadbandMm < 0.0) {
        return false;
    }
    return true;
}

bool validGroup(const LaneConfig& c, int group) {
    if (group < 0 || group >= kGroupCount) {
        return false;
    }
    const int a = c.groupMembers[group][0];
    const int b = c.groupMembers[group][1];
    return a >= 0 && a < kSensorCount && b >= 0 && b < kSensorCount && a != b;
}

}  // namespace

bool isValidLaneConfig(const LaneConfig& c) {
    if (!validChannel(c.channel) || !validPid(c.pd) ||
        !validState(c.state)) {
        return false;
    }

    // 过近阈值必须小于近壁进入阈值；否则 TOO_CLOSE 与 NEAR 重叠不可控。
    if (c.state.tooCloseMm >= c.channel.enterMm) {
        return false;
    }

    // 传感器映射：每个启用的逻辑通道必须有唯一、合法的快照索引。
    bool sourceUsed[4] = {false, false, false, false};
    for (int i = 0; i < kSensorCount; ++i) {
        const SensorConfig& s = c.sensors[i];
        if (s.rangeSource < 0 || s.rangeSource >= 4) {
            return false;
        }
        if (!finite(s.xM) || !finite(s.yM) || !finite(s.axisRad)) {
            return false;
        }
        if (!s.enabled) {
            continue;
        }
        if (sourceUsed[s.rangeSource]) {
            return false;  // 同一路硬件不能映射给两个逻辑通道
        }
        sourceUsed[s.rangeSource] = true;
    }

    if (!validGroup(c, kGroupA) || !validGroup(c, kGroupB)) {
        return false;
    }

    return true;
}

}  // namespace lane
