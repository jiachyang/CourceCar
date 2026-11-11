/**
 * @file modules/lane_keeper.cpp
 * @brief LaneKeeper v2.1 实现：
 *        四路近壁通道 -> 稳定方向选对 -> 双墙可信判定 -> PD/过近排斥
 *        -> 横向变化率限幅 -> 叠加到基础命令。
 *
 * 纯逻辑，不依赖 PROS，也不访问电机/传感器对象。
 */
#include "modules/lane_keeper.hpp"

#include <algorithm>
#include <cmath>

#include "core/math_util.hpp"

namespace lane {
namespace {

constexpr double kMaxDt = 0.2;

bool finiteCommand(const core::MotionCommand& c) {
    return std::isfinite(c.vx) && std::isfinite(c.vy) &&
           std::isfinite(c.omega);
}

}  // namespace

void LaneKeeper::init(const LaneConfig& config) {
    config_ = config;
    configValid_ = isValidLaneConfig(config_);

    for (int i = 0; i < kSensorCount; ++i) {
        channels_[i].init(config_.channel);
    }
    pd_.init(config_.pd);
    selector_.init(config_);
    stateMachine_.init(config_);

    initialized_ = true;
    reset();
    state_.configValid = configValid_;
}

void LaneKeeper::resetControl() {
    pd_.reset();
    selector_.reset();
    stateMachine_.reset();
    lastCorrectionMps_ = 0.0;
    trackDirX_ = 0.0;
    trackDirY_ = 0.0;
    hasTrackDir_ = false;
    wasCredible_ = false;
    lastMode_ = LaneMode::kDisabled;
    lastPairKey_ = -1;
}

void LaneKeeper::reset() {
    resetControl();
    for (int i = 0; i < kSensorCount; ++i) {
        channels_[i].reset();
    }
    state_ = LaneKeeperState{};
    state_.configValid = configValid_;
    transitionCount_ = 0;
}

void LaneKeeper::fillChannelDebug(std::uint32_t nowMs) {
    for (int i = 0; i < kSensorCount; ++i) {
        const RangeChannelValue v = channels_[i].value(nowMs);
        ChannelDebug& d = state_.channels[i];
        d.state = v.state;
        d.rawMm = (v.rawMm > 0.0)
                      ? static_cast<std::int32_t>(v.rawMm)
                      : -1;
        d.filteredMm = v.mm;
        d.observationValid = v.observationValid;
        d.valid = v.valid;
        d.fresh = v.fresh;
        d.ageMs = v.ageMs;
        d.lastGoodMs = channels_[i].lastGoodMs();
    }
}

double LaneKeeper::sideRawMm(int logicalSensor, std::uint32_t nowMs) const {
    if (logicalSensor < 0 || logicalSensor >= kSensorCount) {
        return -1.0;
    }
    const RangeChannelValue v = channels_[logicalSensor].value(nowMs);
    return v.valid ? v.rawMm : -1.0;
}

LaneKeeper::PairEvaluation LaneKeeper::evaluatePair(
    const PairSelection& pair, std::uint32_t nowMs) const {
    PairEvaluation out{};
    if (!pair.valid || pair.leftSensor < 0 ||
        pair.leftSensor >= kSensorCount || pair.rightSensor < 0 ||
        pair.rightSensor >= kSensorCount) {
        return out;
    }

    out.left = channels_[pair.leftSensor].value(nowMs);
    out.right = channels_[pair.rightSensor].value(nowMs);

    const SensorConfig& sl = config_.sensors[pair.leftSensor];
    const SensorConfig& sr = config_.sensors[pair.rightSensor];
    out.qLeft = sl.xM * pair.leftNormalX + sl.yM * pair.leftNormalY;
    out.qRight = sr.xM * (-pair.leftNormalX) +
                 sr.yM * (-pair.leftNormalY);

    const bool bothValid = out.left.valid && out.right.valid;
    const bool geometryValid = out.qLeft >= 0.0 && out.qRight >= 0.0;
    out.widthM = bothValid
                     ? (out.left.mm + out.right.mm) / 1000.0 +
                           out.qLeft + out.qRight
                     : -1.0;
    out.widthValid =
        bothValid && geometryValid &&
        out.widthM >= config_.state.minCorridorWidthM &&
        out.widthM <= config_.state.maxCorridorWidthM;
    out.credible =
        bothValid &&
        out.left.state != NearClass::kFarReturn &&
        out.right.state != NearClass::kFarReturn &&
        out.left.state != NearClass::kUnknown &&
        out.right.state != NearClass::kUnknown &&
        out.left.mm < config_.channel.detectMm &&
        out.right.mm < config_.channel.detectMm &&
        out.widthValid;
    return out;
}

PairSelection LaneKeeper::fallbackPair(std::uint32_t nowMs,
                                       bool* used) const {
    if (used != nullptr) {
        *used = false;
    }

    // 没有稳定通道方向时，按车体物理左右/前后墙各试一对。
    // 只有一对可信才使用；两对都可信时选“更近的一对”（max 距离更小），
    // 避免在十字路口把前后墙当成左右墙。
    PairSelection fb{};
    fb.valid = true;
    fb.group = kGroupB;
    fb.leftSensor = kS1;
    fb.rightSensor = kS3;
    fb.leftNormalX = 0.0;
    fb.leftNormalY = 1.0;
    fb.score = 1.0;

    PairSelection lr{};
    lr.valid = true;
    lr.group = kGroupA;
    lr.leftSensor = kS0;
    lr.rightSensor = kS2;
    lr.leftNormalX = 1.0;
    lr.leftNormalY = 0.0;
    lr.score = 1.0;

    const PairEvaluation efb = evaluatePair(fb, nowMs);
    const PairEvaluation elr = evaluatePair(lr, nowMs);

    if (efb.credible && elr.credible) {
        const double maxFb = std::max(efb.left.mm, efb.right.mm);
        const double maxLr = std::max(elr.left.mm, elr.right.mm);
        if (used != nullptr) {
            *used = true;
        }
        // 默认偏向车体左右（前进通道）；只有前后墙明显更近才选 A。
        return (maxFb <= maxLr + 5.0) ? fb : lr;
    }
    if (efb.credible) {
        if (used != nullptr) {
            *used = true;
        }
        return fb;
    }
    if (elr.credible) {
        if (used != nullptr) {
            *used = true;
        }
        return lr;
    }
    return PairSelection{};
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
    state_.headingValid = heading.valid;

    const std::uint32_t nowMs = ranges.timestampMs;

    // 测距通道每周期都推进，即使 LaneKeeper 关闭也保留诊断。
    for (int i = 0; i < kSensorCount; ++i) {
        const SensorConfig& s = config_.sensors[i];
        const int src = s.rangeSource;
        if (s.enabled && src >= 0 && src < core::kRangeSideCount) {
            channels_[i].update(ranges.samples[src], nowMs);
        } else {
            channels_[i].reset();
        }
    }
    fillChannelDebug(nowMs);

    // 物理左右两侧的原始距离：不依赖当前运动组，便于 UI 同时观察。
    state_.sideLeftRawMm = sideRawMm(kS1, nowMs);
    state_.sideRightRawMm = sideRawMm(kS3, nowMs);

    if (!initialized_) {
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kDisabled;
        state_.valid = inputValid;
        return;
    }
    if (!enable) {
        resetControl();
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kDisabled;
        state_.group = -1;
        state_.pairValid = false;
        state_.command = inputValid ? bodyCommand : core::zeroMotion();
        state_.valid = inputValid;
        return;
    }
    if (!configValid_) {
        resetControl();
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kConfigInvalid;
        state_.group = -1;
        state_.pairValid = false;
        state_.command = inputValid ? bodyCommand : core::zeroMotion();
        state_.valid = false;
        return;
    }
    if (!inputValid) {
        resetControl();
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kInputInvalid;
        state_.group = -1;
        state_.pairValid = false;
        state_.command = core::zeroMotion();
        state_.valid = false;
        return;
    }
    if (!(dt > 0.0) || dt > kMaxDt || !std::isfinite(dt)) {
        // 周期异常时不做任何纠偏，也不产生速度；基础命令原样透传。
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kInputInvalid;
        state_.command = bodyCommand;
        state_.valid = false;
        return;
    }

    // 通道方向：有平移速度时更新；停车时沿用最后一次方向，
    // 这样停车仍可按稳定通道轴居中，而不是因 |v|=0 退出。
    const double speed = std::hypot(bodyCommand.vx, bodyCommand.vy);
    if (std::isfinite(speed) &&
        speed >= config_.state.minDirectionSpeedMps) {
        trackDirX_ = bodyCommand.vx / speed;
        trackDirY_ = bodyCommand.vy / speed;
        hasTrackDir_ = true;
    }

    PairSelection pair =
        selector_.select(trackDirX_, trackDirY_, hasTrackDir_);
    bool fallback = false;
    if (!pair.valid && speed < config_.state.minDirectionSpeedMps) {
        pair = fallbackPair(nowMs, &fallback);
    }

    const int pairKey = pair.valid
                            ? pair.leftSensor * kSensorCount +
                                  pair.rightSensor
                            : -1;
    const bool pairChanged = (pairKey != lastPairKey_);
    if (pairChanged) {
        lastPairKey_ = pairKey;
        pd_.reset();
    }

    state_.pairValid = pair.valid;
    state_.group = pair.group;
    state_.groupChanged = pairChanged;
    state_.fallbackPair = fallback;
    state_.leftSensor = pair.leftSensor;
    state_.rightSensor = pair.rightSensor;
    state_.pairScore = pair.score;
    state_.normalX = pair.leftNormalX;
    state_.normalY = pair.leftNormalY;

    const PairEvaluation eval = evaluatePair(pair, nowMs);
    const RangeChannelValue& left = eval.left;
    const RangeChannelValue& right = eval.right;
    const double qLeft = eval.qLeft;
    const double qRight = eval.qRight;
    const double corridorWidth = eval.widthM;
    const bool widthValid = eval.widthValid;
    const bool credible = eval.credible;

    const bool leftTooClose = left.valid &&
                              left.rawMm < config_.state.tooCloseMm;
    const bool rightTooClose = right.valid &&
                               right.rawMm < config_.state.tooCloseMm;

    state_.corridorWidthM = corridorWidth;
    state_.corridorWidthValid = widthValid;
    state_.corridor = credible;
    state_.leftValid = left.valid;
    state_.rightValid = right.valid;
    state_.leftTooClose = leftTooClose;
    state_.rightTooClose = rightTooClose;
    state_.leftMm = left.valid ? left.mm : -1.0;
    state_.rightMm = right.valid ? right.mm : -1.0;
    state_.leftRawMm = left.valid ? left.rawMm : -1.0;
    state_.rightRawMm = right.valid ? right.rawMm : -1.0;

    LaneStateInputs inputs{};
    inputs.enabled = true;
    inputs.configValid = true;
    inputs.inputValid = true;
    inputs.pairValid = pair.valid;
    inputs.leftValid = left.valid;
    inputs.rightValid = right.valid;
    inputs.credible = credible;
    inputs.tooClose = leftTooClose || rightTooClose;
    inputs.bothFresh = left.fresh && right.fresh;
    inputs.pairChanged = pairChanged;
    inputs.nowMs = nowMs;

    const LaneStateOutput stateOut = stateMachine_.update(inputs);
    state_.mode = stateOut.mode;
    state_.reason = stateOut.reason;
    state_.modeChanged = stateOut.stateChanged;
    state_.degraded = stateOut.degraded;
    state_.decay = stateOut.decay;
    state_.quality = stateOut.quality;
    if (stateOut.stateChanged) {
        ++transitionCount_;
    }
    state_.transitionCount = transitionCount_;

    // 进入/离开 TOO_CLOSE 时清 PD 微分历史，避免恢复瞬间 D 突跳。
    if ((stateOut.mode == LaneMode::kTooClose &&
         lastMode_ != LaneMode::kTooClose) ||
        (lastMode_ == LaneMode::kTooClose &&
         stateOut.mode != LaneMode::kTooClose)) {
        pd_.reset();
    }
    if (credible && !wasCredible_) {
        pd_.reset();
    }
    wasCredible_ = credible;
    lastMode_ = stateOut.mode;

    double errorMm = 0.0;
    double wantedMps = 0.0;
    if (pair.valid && stateOut.mode == LaneMode::kTooClose) {
        // 过近排斥：单侧向远离该侧；双侧按差居中并限幅到排斥速度。
        errorMm = 0.5 * ((right.mm + qRight * 1000.0) -
                         (left.mm + qLeft * 1000.0));
        if (leftTooClose && !rightTooClose) {
            wantedMps = config_.state.repulseMps;
        } else if (rightTooClose && !leftTooClose) {
            wantedMps = -config_.state.repulseMps;
        } else {
            wantedMps = core::clampd(
                config_.pd.kp * errorMm,
                -config_.state.repulseMps,
                config_.state.repulseMps);
        }
    } else if (pair.valid && stateOut.allowCorrection) {
        // PD 误差直接来自两侧滤波距离；q 项用于兼容左右安装半径不一致。
        errorMm = 0.5 * ((right.mm + qRight * 1000.0) -
                         (left.mm + qLeft * 1000.0));
        wantedMps = pd_.update(errorMm, dt, true);
        wantedMps = core::clampd(wantedMps,
                                 -config_.state.maxCorrectionMps,
                                 config_.state.maxCorrectionMps);
    } else if (stateOut.decay) {
        // 丢墙保持：不重新算误差，只把上次纠偏按时间缓释到 0。
        wantedMps =
            lastCorrectionMps_ * stateOut.decayFactor;
    }

    // 实车停车居中：几毫米误差时 PD 只有几 mm/s，克服不了静摩擦。
    // 仅在基础平移速度近似为 0、仍处于可信双墙/过近模式时，给一个
    // 可配置的最小横向速度；误差在死区内不施加，避免目标附近抖动。
    const bool stationary =
        speed < config_.state.minDirectionSpeedMps;
    if (stationary && pair.valid &&
        (stateOut.mode == LaneMode::kTracking ||
         stateOut.mode == LaneMode::kTooClose) &&
        std::fabs(errorMm) > config_.state.standstillDeadbandMm) {
        const double minMag =
            config_.state.standstillMinCorrectionMps;
        if (std::fabs(wantedMps) < minMag) {
            wantedMps = std::copysign(minMag, errorMm);
        }
    }

    // 横向修正变化率限幅。即使目标为 0，也按 maxLateralAccel 平滑回 0。
    const double maxStep = config_.state.maxLateralAccelMps2 * dt;
    double appliedCorrection = lastCorrectionMps_ +
        core::clampd(wantedMps - lastCorrectionMps_, -maxStep, maxStep);
    appliedCorrection = core::clampd(
        appliedCorrection, -config_.state.maxCorrectionMps,
        config_.state.maxCorrectionMps);
    lastCorrectionMps_ = appliedCorrection;

    state_.centerErrorMm = errorMm;
    state_.pd = pd_.debug();
    state_.lateralTargetMps = wantedMps;
    state_.lateralCorrectionMps = appliedCorrection;
    state_.active = pair.valid &&
                    (stateOut.allowCorrection || stateOut.decay ||
                     std::fabs(appliedCorrection) > 1e-9);

    core::MotionCommand output{};
    output.omega = bodyCommand.omega;  // 航向环属于 Chassis
    if (pair.valid) {
        // 左法向 t=(tx,ty)，右法向为 -t；正修正 = 向右。
        output.vx = bodyCommand.vx - appliedCorrection * pair.leftNormalX;
        output.vy = bodyCommand.vy - appliedCorrection * pair.leftNormalY;
    } else {
        output.vx = bodyCommand.vx;
        output.vy = bodyCommand.vy;
    }

    if (!finiteCommand(output)) {
        state_.command = core::zeroMotion();
        state_.valid = false;
        state_.active = false;
        state_.mode = LaneMode::kDisabled;
        state_.reason = LaneReason::kInputInvalid;
        return;
    }

    state_.command = output;
    state_.valid = true;
}

}  // namespace lane
