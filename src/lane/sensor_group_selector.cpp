/**
 * @file lane/sensor_group_selector.cpp
 * @brief 测距组选择实现。纯逻辑，不依赖 PROS。
 */
#include "lane/sensor_group_selector.hpp"

#include <algorithm>
#include <cmath>

namespace lane {

void SensorGroupSelector::init(const LaneConfig& config) {
    config_ = config;
    currentGroup_ = -1;
    lastChangeMs_ = 0;
    hasChange_ = false;
}

void SensorGroupSelector::reset() {
    currentGroup_ = -1;
    lastChangeMs_ = 0;
    hasChange_ = false;
}

SensorGroupSelector::Choice SensorGroupSelector::evaluate(
    int group, double tX, double tY) const {
    Choice choice{};
    if (group < 0 || group >= kGroupCount) {
        return choice;
    }
    const int i0 = config_.groupMembers[group][0];
    const int i1 = config_.groupMembers[group][1];
    if (i0 < 0 || i0 >= kSensorCount || i1 < 0 || i1 >= kSensorCount ||
        i0 == i1) {
        return choice;
    }
    const SensorConfig& s0 = config_.sensors[i0];
    const SensorConfig& s1 = config_.sensors[i1];
    if (!s0.enabled || !s1.enabled) {
        return choice;
    }

    const double a0x = std::cos(s0.axisRad);
    const double a0y = std::sin(s0.axisRad);
    const double a1x = std::cos(s1.axisRad);
    const double a1y = std::sin(s1.axisRad);

    const double opposition = a0x * a1x + a0y * a1y;
    if (opposition > -std::cos(config_.maxPairOppositionErrorRad)) {
        return choice;
    }

    const double c0 = a0x * tX + a0y * tY;
    const double c1 = a1x * tX + a1y * tY;
    if (c0 * c1 >= 0.0) {
        return choice;
    }

    const double score = std::min(std::fabs(c0), std::fabs(c1));
    if (score < config_.minAxisAlignment) {
        return choice;
    }

    choice.valid = true;
    choice.group = group;
    choice.score = score;
    if (c0 > 0.0) {
        choice.leftSensor = i0;
        choice.rightSensor = i1;
    } else {
        choice.leftSensor = i1;
        choice.rightSensor = i0;
    }
    return choice;
}

GroupSelection SensorGroupSelector::select(double vx, double vy,
                                           std::uint32_t nowMs,
                                           bool turning) {
    GroupSelection out{};
    const double speed = std::hypot(vx, vy);

    // 零速/转弯：不选组，强制重新获取阶段，避免方向抖动导致频繁切换。
    if (turning || !(speed >= config_.minMotionSpeedMps)) {
        out.changed = (currentGroup_ != -1);
        currentGroup_ = -1;
        hasChange_ = false;
        return out;
    }

    const double dX = vx / speed;
    const double dY = vy / speed;
    const double tX = -dY;  // 运动左法向
    const double tY = dX;

    const Choice a = evaluate(kGroupA, tX, tY);
    const Choice b = evaluate(kGroupB, tX, tY);

    int bestGroup = -1;
    if (a.valid && b.valid) {
        bestGroup = (a.score >= b.score) ? kGroupA : kGroupB;
    } else if (a.valid) {
        bestGroup = kGroupA;
    } else if (b.valid) {
        bestGroup = kGroupB;
    }

    if (bestGroup < 0) {
        out.changed = (currentGroup_ != -1);
        currentGroup_ = -1;
        hasChange_ = false;
        return out;
    }

    const Choice& best = (bestGroup == kGroupA) ? a : b;
    int chosen = bestGroup;

    if (currentGroup_ >= 0) {
        const Choice& current =
            (currentGroup_ == kGroupA) ? a : b;
        if (current.valid) {
            const bool withinDwell =
                hasChange_ &&
                (nowMs - lastChangeMs_ < config_.groupSwitchDwellMs);
            if (bestGroup != currentGroup_) {
                const bool clearlyBetter =
                    best.score >
                    current.score + config_.groupSwitchScoreMargin;
                chosen = (!withinDwell && clearlyBetter) ? bestGroup
                                                         : currentGroup_;
            } else {
                chosen = currentGroup_;
            }
        } else {
            chosen = bestGroup;
        }
    }

    if (chosen != currentGroup_) {
        currentGroup_ = chosen;
        lastChangeMs_ = nowMs;
        hasChange_ = true;
        out.changed = true;
    }

    const Choice& selected = (chosen == kGroupA) ? a : b;
    out.group = chosen;
    out.leftSensor = selected.leftSensor;
    out.rightSensor = selected.rightSensor;
    out.score = selected.score;
    return out;
}

}  // namespace lane
