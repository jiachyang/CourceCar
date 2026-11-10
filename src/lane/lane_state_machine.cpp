/**
 * @file lane/lane_state_machine.cpp
 * @brief 车道状态机实现。纯逻辑，不依赖 PROS。
 */
#include "lane/lane_state_machine.hpp"

namespace lane {
namespace {

bool isWallState(LaneState s) {
    return s == LaneState::kDualWall || s == LaneState::kLeftWall ||
           s == LaneState::kRightWall;
}

bool allowsLateral(LaneState s) {
    return isWallState(s) || s == LaneState::kTooClose;
}

}  // namespace

void LaneStateMachine::init(const LaneConfig& config) {
    config_ = config;
    reset();
}

void LaneStateMachine::reset() {
    state_ = LaneState::kDisabled;
    started_ = false;
    hasProblemSince_ = false;
    problemSinceMs_ = 0;
    acquireUntilMs_ = 0;
    hasLeftNoTarget_ = false;
    hasRightNoTarget_ = false;
    leftNoTargetSinceMs_ = 0;
    rightNoTargetSinceMs_ = 0;
    hasLeftTooClose_ = false;
    hasRightTooClose_ = false;
    leftTooCloseSinceMs_ = 0;
    rightTooCloseSinceMs_ = 0;
}

LaneStateOutput LaneStateMachine::build(LaneState state, LaneReason reason,
                                        const LaneStateInputs& inputs) const {
    LaneStateOutput out{};
    out.state = state;
    out.reason = reason;
    out.stateChanged = stateChanged_;
    out.resetPid = out.stateChanged || inputs.groupChanged;
    out.degraded = (state == LaneState::kAcquiring) ||
                   (state == LaneState::kSensorFault) ||
                   (state == LaneState::kTooClose);
    out.fault = (state == LaneState::kSensorFault);
    out.allowLateral = allowsLateral(state);

    switch (state) {
        case LaneState::kDisabled:
            out.speedScale = inputs.inputValid ? 1.0 : 0.0;
            break;
        case LaneState::kAcquiring:
            out.speedScale = config_.state.acquireSpeedScale;
            break;
        case LaneState::kTurning:
            out.speedScale = config_.state.turningSpeedScale;
            break;
        case LaneState::kSensorFault:
            out.speedScale = config_.state.faultSpeedScale;
            break;
        case LaneState::kTooClose:
            out.speedScale = config_.state.tooCloseSpeedScale;
            break;
        case LaneState::kNoWall:
        case LaneState::kDualWall:
        case LaneState::kLeftWall:
        case LaneState::kRightWall:
            out.speedScale = 1.0;
            break;
    }

    // 积分门控：状态正常 + 当前测量可用 + 对应墙距严格 <100 mm。
    switch (state) {
        case LaneState::kDualWall:
            out.allowIntegral = inputs.leftUsable && inputs.rightUsable &&
                                !inputs.leftTooClose &&
                                !inputs.rightTooClose &&
                                inputs.leftRawMm > 0.0 &&
                                inputs.rightRawMm > 0.0 &&
                                inputs.leftRawMm <
                                    static_cast<double>(config_.wall.enterMm) &&
                                inputs.rightRawMm <
                                    static_cast<double>(config_.wall.enterMm);
            break;
        case LaneState::kLeftWall:
            out.allowIntegral = inputs.leftUsable && !inputs.leftTooClose &&
                                inputs.leftRawMm > 0.0 &&
                                inputs.leftRawMm <
                                    static_cast<double>(config_.wall.enterMm);
            break;
        case LaneState::kRightWall:
            out.allowIntegral = inputs.rightUsable && !inputs.rightTooClose &&
                                inputs.rightRawMm > 0.0 &&
                                inputs.rightRawMm <
                                    static_cast<double>(config_.wall.enterMm);
            break;
        default:
            out.allowIntegral = false;
            break;
    }
    return out;
}

void LaneStateMachine::enter(LaneState state,
                             const LaneStateInputs& inputs) {
    stateChanged_ = (state_ != state);
    if (stateChanged_) {
        state_ = state;
    }
    if (state == LaneState::kAcquiring) {
        // 首次/转弯后/组切换后的重新识别窗口；普通“状态不足”不反复重置。
        if (!started_ || inputs.groupChanged) {
            acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        }
    }
    started_ = true;
}

LaneStateOutput LaneStateMachine::update(const LaneStateInputs& inputs) {
    stateChanged_ = false;

    // 1) 输入/开关/配置：DISABLED
    if (!inputs.inputValid) {
        enter(LaneState::kDisabled, inputs);
        started_ = false;
        LaneStateOutput out =
            build(LaneState::kDisabled, LaneReason::kInputInvalid, inputs);
        out.speedScale = 0.0;
        return out;
    }
    if (!inputs.enabled) {
        enter(LaneState::kDisabled, inputs);
        started_ = false;
        return build(LaneState::kDisabled, LaneReason::kDisabled, inputs);
    }
    if (!inputs.configValid) {
        enter(LaneState::kDisabled, inputs);
        started_ = false;
        return build(LaneState::kDisabled, LaneReason::kConfigInvalid, inputs);
    }

    // 2) 主动转弯：暂停墙距 PID，清积分；航向稳定且不再转弯后进入重新识别。
    const bool turningActive =
        inputs.turning ||
        (state_ == LaneState::kTurning && !inputs.headingStable);
    if (turningActive) {
        hasProblemSince_ = false;
        hasLeftNoTarget_ = false;
        hasRightNoTarget_ = false;
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        enter(LaneState::kTurning, inputs);
        return build(LaneState::kTurning, LaneReason::kTurning, inputs);
    }
    if (state_ == LaneState::kTurning) {
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        acquireUntilMs_ =
            inputs.nowMs + config_.state.postTurnAcquireMs;
        enter(LaneState::kAcquiring, inputs);
        acquireUntilMs_ =
            inputs.nowMs + config_.state.postTurnAcquireMs;
        return build(LaneState::kAcquiring, LaneReason::kAfterTurn, inputs);
    }

    // 3) 首次启动 / 组切换：进入短重新识别窗口。
    if (!started_) {
        hasProblemSince_ = false;
        hasLeftNoTarget_ = false;
        hasRightNoTarget_ = false;
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        enter(LaneState::kAcquiring, inputs);
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        return build(LaneState::kAcquiring, LaneReason::kStartup, inputs);
    }
    if (inputs.groupChanged) {
        hasProblemSince_ = false;
        hasLeftNoTarget_ = false;
        hasRightNoTarget_ = false;
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        enter(LaneState::kAcquiring, inputs);
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        return build(LaneState::kAcquiring, LaneReason::kGroupChanged, inputs);
    }

    // 4) 无运动/组不可用：重新获取，限制速度。
    // 双墙走廊门控激活时，即使基础速度为 0 也保持模式/PID（但 LaneKeeper
    // 不会在没有运动命令时输出轮速；见模块说明）。
    if ((!inputs.moving && !inputs.corridorActive) || !inputs.groupValid) {
        hasProblemSince_ = false;
        hasLeftNoTarget_ = false;
        hasRightNoTarget_ = false;
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        if (state_ != LaneState::kAcquiring) {
            enter(LaneState::kAcquiring, inputs);
            acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        }
        return build(LaneState::kAcquiring,
                     inputs.moving ? LaneReason::kGroupUnknown
                                   : LaneReason::kStopped,
                     inputs);
    }

    // 5) “无目标/超量程”独立计时：墙体状态仍是 Unknown，不伪造成 Absent；
    //    但持续无目标可作为“无可观测近墙”的证据，超时后允许 NO_WALL/单墙。
    if (inputs.leftNoTarget) {
        if (!hasLeftNoTarget_) {
            hasLeftNoTarget_ = true;
            leftNoTargetSinceMs_ = inputs.nowMs;
        }
    } else {
        hasLeftNoTarget_ = false;
    }
    if (inputs.rightNoTarget) {
        if (!hasRightNoTarget_) {
            hasRightNoTarget_ = true;
            rightNoTargetSinceMs_ = inputs.nowMs;
        }
    } else {
        hasRightNoTarget_ = false;
    }
    const bool leftNoTargetStable =
        hasLeftNoTarget_ && inputs.nowMs >= leftNoTargetSinceMs_ &&
        (inputs.nowMs - leftNoTargetSinceMs_) >=
            config_.state.noWallTimeoutMs;
    const bool rightNoTargetStable =
        hasRightNoTarget_ && inputs.nowMs >= rightNoTargetSinceMs_ &&
        (inputs.nowMs - rightNoTargetSinceMs_) >=
            config_.state.noWallTimeoutMs;

    // 5b) 过近连续确认：<20 mm 不再直接判故障，而是进入 TOO_CLOSE 主动远离。
    if (inputs.leftTooClose) {
        if (!hasLeftTooClose_) {
            hasLeftTooClose_ = true;
            leftTooCloseSinceMs_ = inputs.nowMs;
        }
    } else {
        hasLeftTooClose_ = false;
    }
    if (inputs.rightTooClose) {
        if (!hasRightTooClose_) {
            hasRightTooClose_ = true;
            rightTooCloseSinceMs_ = inputs.nowMs;
        }
    } else {
        hasRightTooClose_ = false;
    }
    const bool leftTooCloseStable =
        hasLeftTooClose_ && inputs.nowMs >= leftTooCloseSinceMs_ &&
        (inputs.nowMs - leftTooCloseSinceMs_) >=
            config_.state.tooCloseConfirmMs;
    const bool rightTooCloseStable =
        hasRightTooClose_ && inputs.nowMs >= rightTooCloseSinceMs_ &&
        (inputs.nowMs - rightTooCloseSinceMs_) >=
            config_.state.tooCloseConfirmMs;

    // 过近优先于普通“未知/故障”：只要还能判断哪一侧贴墙，就先远离，不卡住。
    if (leftTooCloseStable || rightTooCloseStable) {
        hasProblemSince_ = false;
        enter(LaneState::kTooClose, inputs);
        return build(LaneState::kTooClose, LaneReason::kTooClose, inputs);
    }

    // 真正异常：未知且不是“无目标/过近”、Present 但滤波不可用、读取失败/未安装。
    const bool leftUnknownWithoutEvidence =
        inputs.leftStatus == WallStatus::kUnknown && !inputs.leftNoTarget &&
        !inputs.leftTooClose;
    const bool rightUnknownWithoutEvidence =
        inputs.rightStatus == WallStatus::kUnknown && !inputs.rightNoTarget &&
        !inputs.rightTooClose;
    const bool leftBad =
        inputs.leftSensorError || leftUnknownWithoutEvidence ||
        (inputs.leftStatus == WallStatus::kPresent && !inputs.leftUsable);
    const bool rightBad =
        inputs.rightSensorError || rightUnknownWithoutEvidence ||
        (inputs.rightStatus == WallStatus::kPresent && !inputs.rightUsable);

    // 已进入故障：问题未消失前保持故障；恢复后重新识别。
    if (state_ == LaneState::kSensorFault) {
        if (leftBad || rightBad) {
            return build(LaneState::kSensorFault, LaneReason::kSensorTimeout,
                         inputs);
        }
        hasProblemSince_ = false;
        hasLeftNoTarget_ = false;
        hasRightNoTarget_ = false;
        hasLeftTooClose_ = false;
        hasRightTooClose_ = false;
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        enter(LaneState::kAcquiring, inputs);
        acquireUntilMs_ = inputs.nowMs + config_.state.acquireTimeMs;
        return build(LaneState::kAcquiring, LaneReason::kWallAcquiring,
                     inputs);
    }

    if (leftBad || rightBad) {
        if (!hasProblemSince_) {
            hasProblemSince_ = true;
            problemSinceMs_ = inputs.nowMs;
        }
        const std::uint32_t elapsed = inputs.nowMs - problemSinceMs_;
        if (elapsed >= config_.state.sensorFaultTimeoutMs) {
            enter(LaneState::kSensorFault, inputs);
            return build(LaneState::kSensorFault, LaneReason::kSensorTimeout,
                         inputs);
        }
        if (state_ != LaneState::kAcquiring) {
            enter(LaneState::kAcquiring, inputs);
        }
        return build(LaneState::kAcquiring, LaneReason::kWallAcquiring,
                     inputs);
    }
    hasProblemSince_ = false;

    // 6) 重新识别窗口未结束：先维持 ACQUIRING。
    if (state_ == LaneState::kAcquiring &&
        inputs.nowMs < acquireUntilMs_) {
        return build(LaneState::kAcquiring, LaneReason::kWallAcquiring,
                     inputs);
    }

    // 7) 正常墙体模式判定
    const bool bothPresent = inputs.leftStatus == WallStatus::kPresent &&
                             inputs.rightStatus == WallStatus::kPresent;
    const bool leftWall =
        inputs.leftStatus == WallStatus::kPresent && inputs.leftUsable;
    const bool rightWall =
        inputs.rightStatus == WallStatus::kPresent && inputs.rightUsable;
    const bool leftNoNearWall =
        inputs.leftStatus == WallStatus::kAbsent ||
        (inputs.leftStatus == WallStatus::kUnknown && leftNoTargetStable);
    const bool rightNoNearWall =
        inputs.rightStatus == WallStatus::kAbsent ||
        (inputs.rightStatus == WallStatus::kUnknown && rightNoTargetStable);

    if (bothPresent && inputs.leftUsable && inputs.rightUsable &&
        inputs.corridorWidthValid) {
        enter(LaneState::kDualWall, inputs);
        return build(LaneState::kDualWall, LaneReason::kDualWalls, inputs);
    }
    if (bothPresent && inputs.leftUsable && inputs.rightUsable &&
        !inputs.corridorWidthValid) {
        if (state_ != LaneState::kAcquiring) {
            enter(LaneState::kAcquiring, inputs);
        }
        return build(LaneState::kAcquiring, LaneReason::kCorridorInconsistent,
                     inputs);
    }
    if (leftWall && rightNoNearWall) {
        enter(LaneState::kLeftWall, inputs);
        return build(LaneState::kLeftWall, LaneReason::kLeftOnly, inputs);
    }
    if (rightWall && leftNoNearWall) {
        enter(LaneState::kRightWall, inputs);
        return build(LaneState::kRightWall, LaneReason::kRightOnly, inputs);
    }
    if (!leftWall && !rightWall && leftNoNearWall && rightNoNearWall) {
        enter(LaneState::kNoWall, inputs);
        return build(LaneState::kNoWall, LaneReason::kNoWalls, inputs);
    }

    // 8) 其余情况：测量状态还不足以判断。
    if (state_ != LaneState::kAcquiring) {
        enter(LaneState::kAcquiring, inputs);
    }
    return build(LaneState::kAcquiring, LaneReason::kWallAcquiring, inputs);
}

}  // namespace lane
