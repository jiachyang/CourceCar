/**
 * @file modules/radio.hpp
 * @brief Radio 模块：V5 Controller 遥控输入 → core::MotionCommand。
 *
 * 职责边界（3.2）：
 *   - 唯一读取遥控器摇杆 / 按键的模块；
 *   - 在本模块内完成死区、方向映射与速度缩放；
 *   - 输出公共 core::MotionCommand，**不知道底盘有几个轮子**；
 *   - 不访问任何电机 API。
 *
 * 安全机制：
 *   - 遥控器未连接 / 读取失败 → valid=false、命令清零；
 *   - 命令超过 commandTimeoutMs 未刷新 → isCommandFresh()==false，
 *     main 据此下发零命令（不得无限期沿用上一帧）。
 *
 * 调研结论（模式选择、API 依据）见 docs/radio_research.md。
 */
#pragma once

#include <cstdint>
#include <optional>

#include "core/types.hpp"
#include "pros/misc.hpp"
#include "radio/radio_config.hpp"

namespace radio {

/// Radio 状态快照：原始摇杆 + 归一化摇杆 + 映射后的运动命令。
struct RadioState {
    bool connected = false;      ///< 遥控器是否连接

    double leftX = 0.0;          ///< 归一化摇杆 [-1,1]
    double leftY = 0.0;
    double rightX = 0.0;
    double rightY = 0.0;

    bool buttonA = false;        ///< 主要按键状态
    bool buttonB = false;
    bool buttonX = false;
    bool buttonY = false;
    bool buttonL1 = false;
    bool buttonL2 = false;
    bool buttonR1 = false;
    bool buttonR2 = false;

    core::MotionCommand driveCommand{};  ///< 映射后的底盘运动命令
    bool valid = false;                  ///< 当前命令是否有效（连接正常且刚读取）
    std::uint32_t timestampMs = 0;       ///< 最近有效读取时间 [ms]
};

/**
 * @brief Radio 模块（遥控输入模式）。
 */
class Radio {
public:
    /// 初始化：保存配置并构建 Controller 对象。
    void init(const RadioConfig& config);

    /// 读取一次遥控器并刷新状态；失联时自动清零命令。
    void update();

    /// 只读状态快照。
    const RadioState& getState() const { return state_; }

    /// 映射后的底盘运动命令。
    core::MotionCommand getDriveCommand() const { return state_.driveCommand; }

    /// 遥控器是否连接。
    bool isConnected() const { return state_.connected; }

    /// 当前命令是否新鲜（有效且在 commandTimeoutMs 内）。
    bool isCommandFresh() const;

private:
    RadioConfig config_{};
    std::optional<pros::Controller> controller_;
    RadioState state_{};
};

}  // namespace radio
