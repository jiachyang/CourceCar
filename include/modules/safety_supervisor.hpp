/**
 * @file modules/safety_supervisor.hpp
 * @brief 唯一安全限速出口：根据 ToF 与制动模型限制平移速度。
 *
 * 定位：应用在“基础命令 + LaneKeeper 横向 delta”之后、Chassis 之前。
 * 只缩小 vx/vy；不改变 omega（航向仍由 Chassis 航向环执行）。
 *
 * 当前实现：
 *   - 对与运动方向轴对齐的 ToF 做制动距离约束；
 *   - 可选航向可信门控（requireHeading）；
 *   - **不含地图扫掠、未知挡板置信边界、实车制动标定**。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "safety/safety_config.hpp"

namespace safety {

struct SafetyState {
    core::MotionCommand requested{};  ///< 进入安全层前的合成命令
    core::MotionCommand command{};    ///< 限速后的命令
    bool enabled = false;
    bool configValid = false;
    bool limited = false;
    bool stopped = false;
    bool headingBlocked = false;
    double scale = 1.0;               ///< 平移速度比例
    double nearestMm = -1.0;          ///< 运动方向最近有效障碍 [mm]
    std::uint32_t timestampMs = 0;
};

class SafetySupervisor {
public:
    void init(const SafetyConfig& config);

    /**
     * @param command 已合成的车体命令（含 LaneKeeper 横向 delta）
     * @param ranges  四路测距快照（前/后/左/右）
     * @param heading 航向状态；仅 requireHeading=true 时用于停车门控
     */
    void update(const core::MotionCommand& command,
                const core::RangeSnapshot& ranges,
                const core::HeadingState& heading);

    void reset();

    const SafetyState& getState() const { return state_; }
    const SafetyConfig& config() const { return config_; }

    // ---- 整车级运行时开关（ARCH-009）----
    bool enable = true;

private:
    SafetyConfig config_{};
    SafetyState state_{};
    bool configValid_ = false;
    bool initialized_ = false;
};

}  // namespace safety
