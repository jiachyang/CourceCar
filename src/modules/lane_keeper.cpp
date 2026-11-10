/**
 * @file modules/lane_keeper.cpp
 * @brief LaneKeeper 实现：滤波 -> 墙体状态 -> 组选择 -> 状态机 -> 墙距 PID
 *        -> 速度/横向平滑 -> 车体命令。
 *
 * 纯逻辑，不依赖 PROS，也不访问电机/传感器对象。
 */
#include "modules/lane_keeper.hpp"

#include <algorithm>
#include <cmath>

#include "core/math_util.hpp"

namespace lane {
namespace {

constexpr double kFallbackDt = 0.02;
constexpr double kMaxDt = 0.5;

bool finiteCommand(const core::MotionCommand& c) {
    return std::isfinite(c.vx) && std::isfinite(c.vy) &&
           std::isfinite(c.omega);
}

}  // namespace

void LaneKeeper::init(const LaneConfig& config) {
    config_ = config;
    configValid_ = isValidLaneConfig(config_);

    for (int i = 0; i < kSensorCount; ++i) {
        walls_[i].init(config_.wall);
        filters_[i].init(config_.filter);
    }
    wallPid_.init(config_.wallPid);
    selector_.init(config_);
    stateMachine_.init(config_);

    initialized_ = true;
    reset();
    state_.configValid = configValid_;
    state_.targetWallMm = config_.state.targetWallMm;
}

void LaneKeeper::resetControl() {
    wallPid_.reset();
    selector_.reset();
    stateMachine_.reset();
    leftSensor_ = -1;
    rightSensor_ = -1;
    lastGroup_ = -1;
    lastLateralCorrectionMps_ = 0.0;
    lastSpeedScale_ = 1.0;
}

void LaneKeeper::reset() {
    resetControl();
    for (int i = 0; i < kSensorCount; ++i) {
        walls_[i].reset();
        filters_[i].reset();
    }
    state_ = LaneKeeperState{};
    state_.configValid = configValid_;
    state_.targetWallMm = config_.state.targetWallMm;
    transitionCount_ = 0;
}

bool LaneKeeper::referenceClearanceM(int sensor, double nX, double nY,
                                     std::uint32_t nowMs,
                                     double* outM) const {
    if (sensor < 0 || sensor >= kSensorCount || outM == nullptr) {
        return false;
    }
    const SensorConfig& s = config_.sensors[sensor];
    if (walls_[sensor].status() != WallStatus::kPresent) {
        return false;
    }
    if (!filters_[sensor].valid(nowMs, config_.state.measurementHoldMs)) {
        return false;
    }

    const double aX = std::cos(s.axisRad);
    const double aY = std::sin(s.axisRad);
    const double cosAxisNormal = aX * nX + aY * nY;
    if (cosAxisNormal < config_.minAxisAlignment) {
        return false;
    }

    const double rawM = filters_[sensor].filteredMm() / 1000.0;
    if (!(rawM > 0.0)) {
        return false;
    }

    // 参考垂距：L = p·n + d·(a·n)
    const double L = s.xM * nX + s.yM * nY + rawM * cosAxisNormal;
    if (!std::isfinite(L) || L < 0.0 || L > config_.maxWallClearanceM) {
        return false;
    }
    *outM = L;
    return true;
}

double LaneKeeper::forwardSafetyScale(const core::MotionCommand& base,
                                      std::uint32_t nowMs,
                                      double* obstacleMm) const {
    if (obstacleMm != nullptr) {
        *obstacleMm = -1.0;
    }
    const double speed = std::hypot(base.vx, base.vy);
    if (!(speed >= config_.minMotionSpeedMps)) {
        return 1.0;
    }
    const double dX = base.vx / speed;
    const double dY = base.vy / speed;

    int bestSensor = -1;
    double bestDot = -2.0;
    for (int i = 0; i < kSensorCount; ++i) {
        const SensorConfig& s = config_.sensors[i];
        if (!s.enabled) {
            continue;
        }
        const double dot = std::cos(s.axisRad) * dX + std::sin(s.axisRad) * dY;
        if (dot > bestDot) {
            bestDot = dot;
            bestSensor = i;
        }
    }

    // 没有与运动方向足够对齐的传感器（例如 45° 斜向）：感知盲区，不虚构限速。
    if (bestSensor < 0 ||
        bestDot < config_.state.motionSafetyMinAxisAlignment) {
        return 1.0;
    }
    if (!filters_[bestSensor].rawValid(nowMs,
                                       config_.state.measurementHoldMs)) {
        return 1.0;
    }

    const double rawMm = static_cast<double>(filters_[bestSensor].rawMm());
    if (!(rawMm > 0.0)) {
        return 1.0;
    }
    if (obstacleMm != nullptr) {
        *obstacleMm = rawMm;
    }
    if (rawMm <= config_.state.motionStopMm) {
        return 0.0;
    }
    if (rawMm >= config_.state.motionSlowMm) {
        return 1.0;
    }
    return (rawMm - config_.state.motionStopMm) /
           (config_.state.motionSlowMm - config_.state.motionStopMm);
}

void LaneKeeper::update(const core::MotionCommand& bodyCommand,
                        const core::RangeSnapshot& ranges,
                        const core::HeadingState& heading, double dt) {
    state_ = LaneKeeperState{};
    state_.base = bodyCommand;
    const bool inputValid = finiteCommand(bodyCommand);
    state_.command = inputValid ? bodyCommand : core::zeroMotion();
    state_.enabled = enable;
    state_.configValid = configValid_;
    state_.timestampMs = ranges.timestampMs;
    state_.targetWallMm = config_.state.targetWallMm;
    state_.headingValid = heading.valid;

    if (!initialized_) {
        state_.state = LaneState::kDisabled;
        state_.reason = LaneReason::kDisabled;
        return;
    }

    const std::uint32_t nowMs = ranges.timestampMs;
    for (int i = 0; i < kSensorCount; ++i) {
        const SensorConfig& s = config_.sensors[i];
        const int src = s.rangeSource;
        if (s.enabled && src >= 0 && src < core::kRangeSideCount) {
            const core::RangeSample& sample = ranges.samples[src];
            filters_[i].update(sample, nowMs);
            walls_[i].update(sample, nowMs);
        } else {
            filters_[i].reset();
            walls_[i].reset();
        }
    }

    // 物理两侧（S1/S3）的最新原始距离：不依赖当前运动组，UI 可同时看到。
    auto latestSideRaw = [&](int logical) -> double {
        if (logical < 0 || logical >= kSensorCount) {
            return -1.0;
        }
        const SensorConfig& s = config_.sensors[logical];
        if (!s.enabled || s.rangeSource < 0 ||
            s.rangeSource >= core::kRangeSideCount) {
            return -1.0;
        }
        const core::RangeSample& sample = ranges.samples[s.rangeSource];
        if ((sample.status == core::RangeStatus::kOk ||
             sample.status == core::RangeStatus::kTooClose) &&
            sample.distanceMm > 0) {
            return static_cast<double>(sample.distanceMm);
        }
        return -1.0;
    };
    state_.sideLeftRawMm = latestSideRaw(kS1);
    state_.sideRightRawMm = latestSideRaw(kS3);

    for (int i = 0; i < kSensorCount; ++i) {
        WallDebug& d = state_.walls[i];
        d.status = walls_[i].status();
        d.rawMm = filters_[i].rawMm();
        d.filteredMm = filters_[i].filteredMm();
        d.rawValid = filters_[i].rawValid(nowMs,
                                         config_.state.measurementHoldMs);
        d.filterValid = filters_[i].valid(nowMs,
                                         config_.state.measurementHoldMs);
        d.usable = (d.status == WallStatus::kPresent) && d.filterValid;
        d.rawMs = filters_[i].lastRawMs();
        d.filteredMs = filters_[i].lastValidMs();
    }

    // 模块关闭：只更新传感器诊断，不产生控制；基础命令原样透传。
    if (!enable) {
        resetControl();
        state_.state = LaneState::kDisabled;
        state_.reason = LaneReason::kDisabled;
        state_.group = -1;
        state_.command = inputValid ? bodyCommand : core::zeroMotion();
        state_.valid = inputValid;
        return;
    }
    if (!configValid_) {
        resetControl();
        state_.state = LaneState::kDisabled;
        state_.reason = LaneReason::kConfigInvalid;
        state_.group = -1;
        state_.command = inputValid ? bodyCommand : core::zeroMotion();
        state_.valid = false;
        return;
    }
    if (!inputValid) {
        resetControl();
        state_.state = LaneState::kDisabled;
        state_.reason = LaneReason::kInputInvalid;
        state_.group = -1;
        state_.command = core::zeroMotion();
        state_.valid = false;
        return;
    }

    // 双墙走廊门控：只有物理左/右原始距离都 < corridorEnableMm 才启用
    // 墙距控制；单侧/无墙不纠偏。这样车道保持可以长时间开着，但不会在
    // 开放区域或单墙旁边乱修正。
    const bool corridorActive =
        config_.state.corridorOnly &&
        state_.sideLeftRawMm > 0.0 && state_.sideRightRawMm > 0.0 &&
        state_.sideLeftRawMm < config_.state.corridorEnableMm &&
        state_.sideRightRawMm < config_.state.corridorEnableMm;
    state_.corridorActive = corridorActive;

    if (config_.state.corridorOnly && !corridorActive) {
        resetControl();
        state_.state = LaneState::kAcquiring;
        state_.reason = LaneReason::kCorridorInactive;
        state_.group = -1;
        state_.active = false;
        state_.degraded = false;
        state_.fault = false;
        state_.leftTooClose =
            state_.sideLeftRawMm > 0.0 &&
            state_.sideLeftRawMm < config_.state.tooCloseMm;
        state_.rightTooClose =
            state_.sideRightRawMm > 0.0 &&
            state_.sideRightRawMm < config_.state.tooCloseMm;
        state_.stateSpeedScale = 1.0;
        state_.targetSpeedScale = 1.0;
        state_.speedScale = 1.0;
        state_.command = bodyCommand;
        state_.valid = true;
        return;
    }

    if (!(dt > 0.0) || dt > kMaxDt) {
        dt = kFallbackDt;
    }

    const double speed = std::hypot(bodyCommand.vx, bodyCommand.vy);
    const bool moving = speed >= config_.minMotionSpeedMps;
    const bool measuredTurning =
        heading.valid &&
        std::fabs(heading.angularVelocityRadPerSec) >
            config_.state.headingStableOmegaRadPerSec;
    const bool turning =
        std::fabs(bodyCommand.omega) >
            config_.state.turningOmegaRadPerSec ||
        measuredTurning;
    const bool headingStable =
        !heading.valid ||
        std::fabs(heading.angularVelocityRadPerSec) <=
            config_.state.headingStableOmegaRadPerSec;
    state_.turning = turning;
    state_.headingStable = headingStable;

    GroupSelection selection =
        selector_.select(bodyCommand.vx, bodyCommand.vy, nowMs, turning);
    if (corridorActive) {
        // 双墙走廊固定用物理 S1/S3；不因停车/倒车/横移方向改变墙对。
        selection.group = kGroupB;
        selection.leftSensor = kS1;
        selection.rightSensor = kS3;
        selection.changed = (lastGroup_ != kGroupB) || selection.changed;
        lastGroup_ = static_cast<std::int8_t>(kGroupB);
    } else {
        lastGroup_ = static_cast<std::int8_t>(selection.group);
    }
    state_.group = static_cast<std::int8_t>(selection.group);
    state_.groupChanged = selection.changed;
    leftSensor_ = selection.leftSensor;
    rightSensor_ = selection.rightSensor;

    double dX = 0.0;
    double dY = 0.0;
    if (moving) {
        dX = bodyCommand.vx / speed;
        dY = bodyCommand.vy / speed;
    }
    double tX = -dY;  // 运动左法向
    double tY = dX;
    if (corridorActive) {
        // 双墙走廊：墙体是车体左右两侧，参考法向固定为车体 +Y/-Y。
        tX = 0.0;
        tY = 1.0;
    }

    WallStatus leftStatus = WallStatus::kUnknown;
    WallStatus rightStatus = WallStatus::kUnknown;
    bool leftUsable = false;
    bool rightUsable = false;
    double leftDistanceMm = -1.0;
    double rightDistanceMm = -1.0;
    double leftRawMm = -1.0;
    double rightRawMm = -1.0;
    double leftReferenceM = -1.0;
    double rightReferenceM = -1.0;
    double corridorWidthM = -1.0;
    bool corridorWidthValid = false;
    bool leftNoTarget = false;
    bool rightNoTarget = false;
    bool leftSensorError = false;
    bool rightSensorError = false;
    bool leftTooClose = false;
    bool rightTooClose = false;

    if (selection.group >= 0 && leftSensor_ >= 0 && rightSensor_ >= 0) {
        leftStatus = walls_[leftSensor_].status();
        rightStatus = walls_[rightSensor_].status();
        if (filters_[leftSensor_].valid(nowMs,
                                        config_.state.measurementHoldMs)) {
            leftDistanceMm = filters_[leftSensor_].filteredMm();
        }
        if (filters_[rightSensor_].valid(nowMs,
                                         config_.state.measurementHoldMs)) {
            rightDistanceMm = filters_[rightSensor_].filteredMm();
        }
        leftUsable = (leftStatus == WallStatus::kPresent) &&
                     (leftDistanceMm > 0.0);
        rightUsable = (rightStatus == WallStatus::kPresent) &&
                      (rightDistanceMm > 0.0);
        const int leftSrc = config_.sensors[leftSensor_].rangeSource;
        const int rightSrc = config_.sensors[rightSensor_].rangeSource;
        if (leftSrc >= 0 && leftSrc < core::kRangeSideCount) {
            const core::RangeSample& sample = ranges.samples[leftSrc];
            leftNoTarget = (sample.status == core::RangeStatus::kNoTarget) ||
                           (sample.status == core::RangeStatus::kTooFar);
            leftTooClose = (sample.status == core::RangeStatus::kTooClose);
            leftSensorError = (sample.status == core::RangeStatus::kReadError) ||
                              (sample.status == core::RangeStatus::kNotInstalled);
            if (sample.status == core::RangeStatus::kOk ||
                sample.status == core::RangeStatus::kTooClose) {
                leftRawMm = static_cast<double>(sample.distanceMm);
            }
        }
        if (rightSrc >= 0 && rightSrc < core::kRangeSideCount) {
            const core::RangeSample& sample = ranges.samples[rightSrc];
            rightNoTarget = (sample.status == core::RangeStatus::kNoTarget) ||
                            (sample.status == core::RangeStatus::kTooFar);
            rightTooClose = (sample.status == core::RangeStatus::kTooClose);
            rightSensorError =
                (sample.status == core::RangeStatus::kReadError) ||
                (sample.status == core::RangeStatus::kNotInstalled);
            if (sample.status == core::RangeStatus::kOk ||
                sample.status == core::RangeStatus::kTooClose) {
                rightRawMm = static_cast<double>(sample.distanceMm);
            }
        }

        if (leftUsable && rightUsable) {
            const bool leftRefOk =
                referenceClearanceM(leftSensor_, tX, tY, nowMs,
                                    &leftReferenceM);
            const bool rightRefOk =
                referenceClearanceM(rightSensor_, -tX, -tY, nowMs,
                                    &rightReferenceM);
            if (leftRefOk && rightRefOk) {
                corridorWidthM = leftReferenceM + rightReferenceM;
                corridorWidthValid =
                    corridorActive ||
                    (corridorWidthM >= config_.state.minCorridorWidthM &&
                     corridorWidthM <= config_.state.maxCorridorWidthM);
            }
        }
    }

    state_.leftObserved = leftUsable;
    state_.rightObserved = rightUsable;
    state_.leftTooClose = leftTooClose;
    state_.rightTooClose = rightTooClose;
    state_.leftDistanceMm = leftUsable ? leftDistanceMm : -1.0;
    state_.rightDistanceMm = rightUsable ? rightDistanceMm : -1.0;
    state_.leftRawMm = leftRawMm;
    state_.rightRawMm = rightRawMm;
    state_.leftReferenceM = leftReferenceM;
    state_.rightReferenceM = rightReferenceM;
    state_.corridorWidthM = corridorWidthM;
    state_.corridorWidthValid = corridorWidthValid;

    LaneStateInputs inputs{};
    inputs.enabled = enable;
    inputs.configValid = configValid_;
    inputs.inputValid = inputValid;
    inputs.moving = moving;
    inputs.turning = turning;
    inputs.headingStable = headingStable;
    inputs.groupValid = selection.group >= 0;
    inputs.groupChanged = selection.changed;
    inputs.corridorActive = corridorActive;
    inputs.leftStatus = leftStatus;
    inputs.rightStatus = rightStatus;
    inputs.leftUsable = leftUsable;
    inputs.rightUsable = rightUsable;
    inputs.corridorWidthValid = corridorWidthValid;
    inputs.leftNoTarget = leftNoTarget;
    inputs.rightNoTarget = rightNoTarget;
    inputs.leftSensorError = leftSensorError;
    inputs.rightSensorError = rightSensorError;
    inputs.leftTooClose = leftTooClose;
    inputs.rightTooClose = rightTooClose;
    inputs.leftRawMm = leftRawMm;
    inputs.rightRawMm = rightRawMm;
    inputs.nowMs = nowMs;

    const LaneStateOutput stateOut = stateMachine_.update(inputs);
    state_.state = stateOut.state;
    state_.reason = stateOut.reason;
    state_.active = stateOut.allowLateral;
    state_.degraded = stateOut.degraded;
    state_.fault = stateOut.fault;
    state_.stateChanged = stateOut.stateChanged;
    state_.stateSpeedScale = stateOut.speedScale;
    if (stateOut.stateChanged) {
        ++transitionCount_;
    }
    state_.transitionCount = transitionCount_;

    if (stateOut.resetPid) {
        wallPid_.reset();
    }

    double errorMm = 0.0;
    double targetCorrectionMps = 0.0;
    if (stateOut.state == LaneState::kTooClose) {
        // 过近排斥：不依赖墙距 PID，直接向远离最近墙的方向给一个最小速度。
        // 单侧过近：远离该侧；双侧过近：远离更近的一侧；完全相等时沿用
        // 上一次纠偏方向，避免原地卡死。
        if (leftTooClose && !rightTooClose) {
            const double gapMm = std::max(
                0.0, config_.state.tooCloseMm - leftRawMm);
            errorMm = +gapMm;  // 左墙过近 -> 向右
            targetCorrectionMps =
                core::clampd(gapMm * config_.state.tooCloseGainMpsPerMm,
                             config_.state.tooCloseMinCorrectionMps,
                             config_.state.tooCloseMaxCorrectionMps);
        } else if (rightTooClose && !leftTooClose) {
            const double gapMm = std::max(
                0.0, config_.state.tooCloseMm - rightRawMm);
            errorMm = -gapMm;  // 右墙过近 -> 向左
            targetCorrectionMps =
                -core::clampd(gapMm * config_.state.tooCloseGainMpsPerMm,
                              config_.state.tooCloseMinCorrectionMps,
                              config_.state.tooCloseMaxCorrectionMps);
        } else if (leftTooClose && rightTooClose) {
            // 两侧都 <20：用两侧差做“居中”，不再按“远离更近侧”给最小
            // 排斥量，否则在窄通道中心会左右来回抖。
            errorMm = 0.5 * (rightRawMm - leftRawMm);
            if (std::fabs(errorMm) < 1.0) {
                errorMm = 0.0;
            }
            targetCorrectionMps =
                core::clampd(errorMm * config_.state.tooCloseGainMpsPerMm,
                             -config_.state.tooCloseMaxCorrectionMps,
                             config_.state.tooCloseMaxCorrectionMps);
        }
    } else if (stateOut.allowLateral) {
        if (stateOut.state == LaneState::kDualWall) {
            // 右墙更远 -> 车体偏左 -> 向右修正（right-positive）
            errorMm = 0.5 * (rightReferenceM - leftReferenceM) * 1000.0;
        } else if (stateOut.state == LaneState::kLeftWall) {
            // 左墙太近 -> 向右
            errorMm = config_.state.targetWallMm - leftDistanceMm;
        } else if (stateOut.state == LaneState::kRightWall) {
            // 右墙太近 -> 向左（负）
            errorMm = rightDistanceMm - config_.state.targetWallMm;
        }
        targetCorrectionMps =
            wallPid_.update(errorMm, dt, stateOut.allowIntegral);
        targetCorrectionMps =
            core::clampd(targetCorrectionMps,
                         -config_.state.maxLateralCorrectionMps,
                         config_.state.maxLateralCorrectionMps);
    }

    state_.lateralErrorMm = errorMm;
    state_.wallPid = wallPid_.debug();
    state_.lateralTargetMps = targetCorrectionMps;

    double appliedCorrectionMps = 0.0;
    if (stateOut.state == LaneState::kSensorFault) {
        appliedCorrectionMps = 0.0;
        lastLateralCorrectionMps_ = 0.0;
    } else {
        const double maxStep =
            config_.state.maxLateralAccelMps2 * dt;
        appliedCorrectionMps =
            lastLateralCorrectionMps_ +
            core::clampd(targetCorrectionMps -
                             lastLateralCorrectionMps_,
                         -maxStep, maxStep);
        lastLateralCorrectionMps_ = appliedCorrectionMps;
    }
    double obstacleMm = -1.0;
    const double forwardScale =
        forwardSafetyScale(bodyCommand, nowMs, &obstacleMm);
    state_.forwardSafetyScale = forwardScale;
    state_.motionObstacleMm = obstacleMm;

    const double targetSpeedScale =
        stateOut.speedScale * forwardScale;
    state_.targetSpeedScale = targetSpeedScale;

    double appliedSpeedScale = 1.0;
    if (stateOut.state == LaneState::kSensorFault) {
        // 故障：按配置的 faultSpeedScale（默认 0）立即生效。
        appliedSpeedScale = stateOut.speedScale;
        lastSpeedScale_ = appliedSpeedScale;
    } else if (forwardScale <= 0.0) {
        // 前向障碍：立即停止，不做速度斜坡。
        appliedSpeedScale = 0.0;
        lastSpeedScale_ = 0.0;
    } else {
        const double maxStep =
            config_.state.maxSpeedScaleRatePerSec * dt;
        appliedSpeedScale =
            lastSpeedScale_ +
            core::clampd(targetSpeedScale - lastSpeedScale_,
                         -maxStep, maxStep);
        appliedSpeedScale = core::clampd(appliedSpeedScale, 0.0, 1.0);
        lastSpeedScale_ = appliedSpeedScale;
    }
    state_.speedScale = appliedSpeedScale;

    core::MotionCommand output{};
    output.omega = bodyCommand.omega;  // 航向环属于 Chassis
    if (corridorActive) {
        // 双墙走廊固定以车体右法向 (0,-1) 纠偏；前进/倒车/横移都保持
        // “左墙-右墙”物理对，不会因倒车把方向翻反。
        if (moving) {
            output.vx = bodyCommand.vx * appliedSpeedScale;
            output.vy = (bodyCommand.vy - appliedCorrectionMps) *
                        appliedSpeedScale;
            state_.lateralCorrectionMps =
                appliedCorrectionMps * appliedSpeedScale;
        } else {
            // 走廊内停车：允许低速自动居中/远离，避免“目标非 0 但实际 0”
            // 的假动作。速度受 standstillMaxCorrectionMps 限制。
            output.vx = bodyCommand.vx * appliedSpeedScale;
            output.vy = bodyCommand.vy * appliedSpeedScale;
            double holdCorr = 0.0;
            if (config_.state.standstillCenter) {
                double holdTarget = appliedCorrectionMps;
                if (stateOut.state == LaneState::kTooClose &&
                    std::fabs(holdTarget) > 1e-4) {
                    // 静摩擦：过近自动远离至少给一个能动的低速。
                    const double minMag =
                        config_.state.standstillMinTooCloseMps;
                    holdTarget = std::copysign(
                        std::max(std::fabs(holdTarget), minMag), holdTarget);
                } else {
                    holdTarget *= appliedSpeedScale;
                }
                holdCorr = core::clampd(
                    holdTarget, -config_.state.standstillMaxCorrectionMps,
                    config_.state.standstillMaxCorrectionMps);
            }
            output.vy -= holdCorr;  // right-positive -> 车体 -Y
            state_.lateralCorrectionMps = holdCorr;
        }
    } else if (moving) {
        // 右法向 r = (dY, -dX)；横向纠偏向右为正。
        // 基础速度与横向纠偏统一乘安全速度比例：前向障碍为 0 时整车停。
        output.vx = (bodyCommand.vx + appliedCorrectionMps * dY) *
                    appliedSpeedScale;
        output.vy = (bodyCommand.vy - appliedCorrectionMps * dX) *
                    appliedSpeedScale;
        state_.lateralCorrectionMps =
            appliedCorrectionMps * appliedSpeedScale;
    } else {
        output.vx = bodyCommand.vx * appliedSpeedScale;
        output.vy = bodyCommand.vy * appliedSpeedScale;
        state_.lateralCorrectionMps = 0.0;
    }

    if (!finiteCommand(output)) {
        state_.command = core::zeroMotion();
        state_.valid = false;
        state_.active = false;
        state_.fault = true;
        return;
    }

    state_.command = output;
    state_.valid = true;
}

}  // namespace lane
