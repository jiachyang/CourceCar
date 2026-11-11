/**
 * @file lane/lane_state_machine.cpp
 * @brief LaneKeeper v2.1 模式机实现。纯逻辑，不依赖 PROS。
 */
#include "lane/lane_state_machine.hpp"

namespace lane {

void LaneStateMachine::init(const LaneConfig& config) {
    config_ = config;
    reset();
}

void LaneStateMachine::reset() {
    mode_ = LaneMode::kDisabled;
    stateChanged_ = false;
    everTracked_ = false;
    lastTrackMs_ = 0;
}

void LaneStateMachine::enter(LaneMode mode) {
    stateChanged_ = (mode != mode_);
    mode_ = mode;
}

LaneStateOutput LaneStateMachine::build(LaneMode mode, LaneReason reason,
                                        const LaneStateInputs& inputs) const {
    (void)inputs;
    LaneStateOutput out{};
    out.mode = mode;
    out.reason = reason;
    out.stateChanged = stateChanged_;
    return out;
}

LaneStateOutput LaneStateMachine::update(const LaneStateInputs& inputs) {
    stateChanged_ = false;

    if (!inputs.inputValid) {
        everTracked_ = false;
        lastTrackMs_ = 0;
        enter(LaneMode::kDisabled);
        return build(LaneMode::kDisabled, LaneReason::kInputInvalid, inputs);
    }
    if (!inputs.enabled) {
        everTracked_ = false;
        lastTrackMs_ = 0;
        enter(LaneMode::kDisabled);
        return build(LaneMode::kDisabled, LaneReason::kDisabled, inputs);
    }
    if (!inputs.configValid) {
        everTracked_ = false;
        lastTrackMs_ = 0;
        enter(LaneMode::kDisabled);
        return build(LaneMode::kDisabled, LaneReason::kConfigInvalid, inputs);
    }

    // 没有稳定的通道方向：不猜左右墙，也不应用旧纠偏。
    if (!inputs.pairValid) {
        everTracked_ = false;
        lastTrackMs_ = 0;
        enter(LaneMode::kNoPair);
        return build(LaneMode::kNoPair, LaneReason::kNoPair, inputs);
    }

    // 换对后不允许把上一对的时间/微分历史带过来。
    if (inputs.pairChanged) {
        everTracked_ = false;
        lastTrackMs_ = 0;
    }

    if (inputs.credible) {
        everTracked_ = true;
        lastTrackMs_ = inputs.nowMs;
        const bool stale = !inputs.bothFresh;
        const LaneMode mode = inputs.tooClose
                                  ? LaneMode::kTooClose
                                  : (stale ? LaneMode::kDegraded
                                           : LaneMode::kTracking);
        const LaneReason reason = inputs.tooClose
                                      ? LaneReason::kTooClose
                                      : (stale ? LaneReason::kStaleMeasurement
                                               : LaneReason::kTracking);
        enter(mode);
        LaneStateOutput out = build(mode, reason, inputs);
        // 新鲜度不足但仍可信：继续用当前测量算 PD，不把控制权交回 0。
        out.allowCorrection = true;
        out.degraded = (mode == LaneMode::kDegraded);
        out.quality = (mode == LaneMode::kTracking) ? 1.0 : 0.5;
        return out;
    }

    // 失去可信双墙：在 hold 窗口内缓释上次纠偏，不做“丢一帧就重置”。
    const bool holding =
        everTracked_ && inputs.nowMs >= lastTrackMs_ &&
        (inputs.nowMs - lastTrackMs_) < config_.state.degradedHoldMs;
    if (holding) {
        enter(LaneMode::kDegraded);
        LaneStateOutput out =
            build(LaneMode::kDegraded, LaneReason::kLostWallHold, inputs);
        const double hold =
            static_cast<double>(config_.state.degradedHoldMs);
        const double elapsed =
            static_cast<double>(inputs.nowMs - lastTrackMs_);
        out.decay = true;
        out.decayFactor = (hold > 0.0) ? (1.0 - elapsed / hold) : 0.0;
        out.degraded = true;
        out.quality = 0.5;
        return out;
    }

    everTracked_ = false;
    if (inputs.leftValid && inputs.rightValid) {
        enter(LaneMode::kNoCorridor);
        return build(LaneMode::kNoCorridor, LaneReason::kNoCorridor, inputs);
    }
    enter(LaneMode::kInsufficientData);
    return build(LaneMode::kInsufficientData,
                 LaneReason::kInsufficientData, inputs);
}

}  // namespace lane
