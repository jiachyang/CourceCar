/**
 * @file lane/lane_state_machine.hpp
 * @brief 单一墙距/车道状态机：DISABLED / ACQUIRING / DUAL_WALL / LEFT_WALL /
 *        RIGHT_WALL / NO_WALL / TURNING / SENSOR_FAULT / TOO_CLOSE。
 *
 * 状态机只做“模式 + 权限 + 速度比例 + 复位事件”，不直接算 PID。
 * 墙距 PID 的积分权限、转弯暂停、无墙/故障降级都由这里统一给出。
 */
#pragma once

#include <cstdint>

#include "lane/lane_config.hpp"
#include "lane/wall_state.hpp"

namespace lane {

enum class LaneState : std::uint8_t {
    kDisabled = 0,
    kAcquiring,
    kDualWall,
    kLeftWall,
    kRightWall,
    kNoWall,
    kTurning,
    kSensorFault,
    kTooClose,  ///< 单侧/双侧过近：不卡故障，主动向远离近墙方向连续修正
};

enum class LaneReason : std::uint8_t {
    kNone = 0,
    kDisabled,
    kConfigInvalid,
    kInputInvalid,
    kStartup,
    kStopped,
    kGroupUnknown,
    kGroupChanged,
    kAfterTurn,
    kWallAcquiring,
    kCorridorInconsistent,
    kSensorTimeout,
    kDualWalls,
    kLeftOnly,
    kRightOnly,
    kNoWalls,
    kTurning,
    kTooClose,
    kCorridorInactive,
};

struct LaneStateInputs {
    bool enabled = false;
    bool configValid = false;
    bool inputValid = false;
    bool moving = false;
    bool turning = false;
    bool headingStable = true;
    bool groupValid = false;
    bool groupChanged = false;
    /// 双墙走廊门控：两侧物理传感器都 <150 mm 且强制 B 组时为 true。
    bool corridorActive = false;
    WallStatus leftStatus = WallStatus::kUnknown;
    WallStatus rightStatus = WallStatus::kUnknown;
    bool leftUsable = false;
    bool rightUsable = false;
    bool corridorWidthValid = false;
    /// 最新样本是“无目标/超量程”（9999/>2000）：不是 wall_present=false，
    /// 但可作为“无可观测近墙”的独立证据（超时后允许 NO_WALL/单墙）。
    bool leftNoTarget = false;
    bool rightNoTarget = false;
    /// 最新样本是读取失败/未安装：硬件或遮挡异常，不能当作无墙。
    /// 过近（kTooClose）单独由 leftTooClose/rightTooClose 表示，不再直接算故障。
    bool leftSensorError = false;
    bool rightSensorError = false;
    /// 最新原始距离 <20 mm（过近）；状态机连续确认后进入 TOO_CLOSE 主动远离。
    bool leftTooClose = false;
    bool rightTooClose = false;
    /// 积分门控使用**原始**有效墙距（严格 <100 mm），不是滤波值。
    double leftRawMm = -1.0;
    double rightRawMm = -1.0;
    std::uint32_t nowMs = 0;
};

struct LaneStateOutput {
    LaneState state = LaneState::kDisabled;
    LaneReason reason = LaneReason::kNone;
    bool stateChanged = false;
    bool allowLateral = false;
    bool allowIntegral = false;
    bool resetPid = false;
    bool degraded = false;
    bool fault = false;
    double speedScale = 1.0;
};

class LaneStateMachine {
public:
    void init(const LaneConfig& config);
    void reset();

    LaneStateOutput update(const LaneStateInputs& inputs);

    LaneState state() const { return state_; }

private:
    LaneStateOutput build(LaneState state, LaneReason reason,
                          const LaneStateInputs& inputs) const;
    void enter(LaneState state, const LaneStateInputs& inputs);

    LaneConfig config_{};
    LaneState state_ = LaneState::kDisabled;
    bool started_ = false;
    bool stateChanged_ = false;
    bool hasProblemSince_ = false;
    std::uint32_t problemSinceMs_ = 0;
    std::uint32_t acquireUntilMs_ = 0;
    bool hasLeftNoTarget_ = false;
    bool hasRightNoTarget_ = false;
    std::uint32_t leftNoTargetSinceMs_ = 0;
    std::uint32_t rightNoTargetSinceMs_ = 0;
    bool hasLeftTooClose_ = false;
    bool hasRightTooClose_ = false;
    std::uint32_t leftTooCloseSinceMs_ = 0;
    std::uint32_t rightTooCloseSinceMs_ = 0;
};

}  // namespace lane
