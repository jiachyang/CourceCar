/**
 * @file lane/lane_state_machine.hpp
 * @brief LaneKeeper v2.1 模式机：只决定“是否居中/是否衰减”，不提供速度比例。
 *
 * 模式：
 *   DISABLED / NO_PAIR / INSUFFICIENT_DATA / NO_CORRIDOR /
 *   TRACKING / DEGRADED / TOO_CLOSE
 *
 * 与旧版的区别：
 *   - 没有 TURNING / SENSOR_FAULT / LEFT_WALL / RIGHT_WALL / NO_WALL；
 *     转向与安全停车属于上游路径层和安全层；
 *   - 不输出 speedScale，LaneKeeper 不改变巡航速度；
 *   - DEGRADED 有两种：新鲜度不足但仍可信（继续算 PD），以及丢墙后的
 *     旧纠偏衰减（由 LaneKeeper 按 decayFactor 缓释）。
 */
#pragma once

#include <cstdint>

#include "lane/lane_config.hpp"

namespace lane {

enum class LaneMode : std::uint8_t {
    kDisabled = 0,
    kNoPair,
    kInsufficientData,
    kNoCorridor,
    kTracking,
    kDegraded,
    kTooClose,
};

enum class LaneReason : std::uint8_t {
    kNone = 0,
    kDisabled,
    kConfigInvalid,
    kInputInvalid,
    kNoPair,
    kInsufficientData,
    kNoCorridor,
    kTracking,
    kTooClose,
    kStaleMeasurement,
    kLostWallHold,
};

struct LaneStateInputs {
    bool enabled = false;
    bool configValid = false;
    bool inputValid = false;
    bool pairValid = false;
    bool leftValid = false;
    bool rightValid = false;
    bool credible = false;
    bool tooClose = false;
    bool bothFresh = false;
    bool pairChanged = false;
    std::uint32_t nowMs = 0;
};

struct LaneStateOutput {
    LaneMode mode = LaneMode::kDisabled;
    LaneReason reason = LaneReason::kNone;
    bool stateChanged = false;
    bool allowCorrection = false;  ///< 允许用新误差计算 PD/过近排斥
    bool decay = false;            ///< 丢墙保持：按 decayFactor 缓释旧纠偏
    bool degraded = false;
    double decayFactor = 0.0;      ///< 1 -> 0
    double quality = 0.0;          ///< 1=TRACKING, 0.5=DEGRADED/TOO_CLOSE
};

class LaneStateMachine {
public:
    void init(const LaneConfig& config);
    void reset();

    LaneStateOutput update(const LaneStateInputs& inputs);

    LaneMode mode() const { return mode_; }

private:
    LaneStateOutput build(LaneMode mode, LaneReason reason,
                          const LaneStateInputs& inputs) const;
    void enter(LaneMode mode);

    LaneConfig config_{};
    LaneMode mode_ = LaneMode::kDisabled;
    bool stateChanged_ = false;
    bool everTracked_ = false;
    std::uint32_t lastTrackMs_ = 0;
};

}  // namespace lane
