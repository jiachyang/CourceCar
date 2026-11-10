/**
 * @file lane/lane_config.cpp
 * @brief LaneConfig 静态校验。纯逻辑，不依赖 PROS。
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

bool validFilter(const RangeFilterConfig& c) {
    if (!finite(c.lowPassAlpha) || c.lowPassAlpha < 0.0 ||
        c.lowPassAlpha > 1.0) {
        return false;
    }
    if (!finite(c.jumpThresholdMm) || c.jumpThresholdMm < 0.0) {
        return false;
    }
    return c.jumpConfirmSamples >= 1;
}

bool validState(const LaneStateConfig& c) {
    if (c.acquireTimeMs == 0 || c.postTurnAcquireMs == 0 ||
        c.sensorFaultTimeoutMs == 0 || c.noWallTimeoutMs == 0) {
        return false;
    }
    const double scales[] = {c.acquireSpeedScale, c.turningSpeedScale,
                             c.degradedSpeedScale, c.faultSpeedScale,
                             c.tooCloseSpeedScale};
    for (double s : scales) {
        if (!finite(s) || s < 0.0 || s > 1.0) {
            return false;
        }
    }
    if (!finite(c.turningOmegaRadPerSec) || c.turningOmegaRadPerSec <= 0.0 ||
        !finite(c.headingStableOmegaRadPerSec) ||
        c.headingStableOmegaRadPerSec <= 0.0) {
        return false;
    }
    if (!finite(c.targetWallMm) || c.targetWallMm <= 0.0) {
        return false;
    }
    if (!finite(c.minCorridorWidthM) || !finite(c.maxCorridorWidthM) ||
        c.minCorridorWidthM <= 0.0 ||
        c.maxCorridorWidthM <= c.minCorridorWidthM) {
        return false;
    }
    if (!finite(c.maxLateralCorrectionMps) ||
        c.maxLateralCorrectionMps <= 0.0 ||
        !finite(c.maxLateralAccelMps2) || c.maxLateralAccelMps2 <= 0.0 ||
        !finite(c.maxSpeedScaleRatePerSec) ||
        c.maxSpeedScaleRatePerSec <= 0.0) {
        return false;
    }
    if (!finite(c.motionStopMm) || !finite(c.motionSlowMm) ||
        c.motionStopMm < 0.0 || c.motionSlowMm <= c.motionStopMm) {
        return false;
    }
    if (!finite(c.motionSafetyMinAxisAlignment) ||
        c.motionSafetyMinAxisAlignment <= 0.0 ||
        c.motionSafetyMinAxisAlignment > 1.0) {
        return false;
    }

    // 过近排斥：阈值必须在有效量程下限附近且不超过墙体进入阈值；
    // 最小/最大排斥速度和增益必须有限、非负、有序。
    if (!finite(c.tooCloseMm) || c.tooCloseMm <= 0.0 ||
        !finite(c.corridorEnableMm) || c.corridorEnableMm <= 0.0) {
        return false;
    }
    if (c.tooCloseConfirmMs == 0) {
        return false;
    }
    if (!finite(c.tooCloseGainMpsPerMm) ||
        c.tooCloseGainMpsPerMm <= 0.0 ||
        !finite(c.tooCloseMinCorrectionMps) ||
        !finite(c.tooCloseMaxCorrectionMps) ||
        c.tooCloseMinCorrectionMps < 0.0 ||
        c.tooCloseMaxCorrectionMps < c.tooCloseMinCorrectionMps ||
        c.tooCloseMaxCorrectionMps > c.maxLateralCorrectionMps) {
        return false;
    }
    if (!finite(c.standstillMaxCorrectionMps) ||
        c.standstillMaxCorrectionMps <= 0.0 ||
        c.standstillMaxCorrectionMps > c.maxLateralCorrectionMps ||
        !finite(c.standstillMinTooCloseMps) ||
        c.standstillMinTooCloseMps < 0.0 ||
        c.standstillMinTooCloseMps > c.standstillMaxCorrectionMps) {
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
    // 墙体状态机参数
    if (c.wall.enterMm <= 0 || c.wall.exitMm <= c.wall.enterMm ||
        c.wall.confirmCount < 1 || c.wall.staleTimeoutMs == 0 ||
        c.wall.minSampleIntervalMs == 0) {
        return false;
    }
    if (c.state.tooCloseMm > c.wall.enterMm ||
        (c.state.corridorOnly &&
         c.state.corridorEnableMm > c.wall.exitMm)) {
        return false;
    }

    // 单墙目标 / 启动阈值 / 几何阈值
    if (!finite(c.minMotionSpeedMps) || c.minMotionSpeedMps < 0.0) {
        return false;
    }
    if (!finite(c.minAxisAlignment) || c.minAxisAlignment <= 0.0 ||
        c.minAxisAlignment > 1.0) {
        return false;
    }
    if (!finite(c.maxPairOppositionErrorRad) ||
        c.maxPairOppositionErrorRad < 0.0 ||
        c.maxPairOppositionErrorRad >= kPi / 2.0) {
        return false;
    }
    if (!finite(c.groupSwitchScoreMargin) || c.groupSwitchScoreMargin < 0.0 ||
        c.groupSwitchScoreMargin > 1.0) {
        return false;
    }
    if (!finite(c.maxWallClearanceM) || c.maxWallClearanceM <= 0.0) {
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

    return validPid(c.wallPid) && validFilter(c.filter) &&
           validState(c.state);
}

}  // namespace lane
