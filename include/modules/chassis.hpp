/**
 * @file modules/chassis.hpp
 * @brief Chassis 模块：底盘配置、逆运动学、电机驱动、航向保持、状态快照。
 *
 * 职责（4.1）：
 *   - 保存并校验底盘配置；
 *   - 管理四个驱动电机；
 *   - 把 core::MotionCommand 逆运动学分解为四轮目标转速；
 *   - 在模块内完成方向符号、饱和限幅、加减速、刹车；
 *   - 可选地消费 core::HeadingState 做航向保持（航向源只提供状态，不碰电机）。
 *
 * 依赖方向：Chassis 只依赖 core 的公共契约（MotionCommand / HeadingState），
 * 不依赖 imu / radio 任何一个模块的头文件——换航向源或换命令源都不用改这里。
 *
 * 对外只接受“运动目标”，不暴露电机 / IK 细节；main 不需要知道轮速分配。
 */
#pragma once

#include <cstdint>
#include <optional>

#include "chassis/chassis_config.hpp"
#include "chassis/chassis_drive.hpp"
#include "chassis/heading_controller.hpp"
#include "core/types.hpp"

namespace chassis {

/// 底盘状态快照：目标命令、经过航向/平滑后的实际命令、四轮各级输出、健康标志。
struct ChassisState {
    core::MotionCommand target{};   ///< 最近一次 setCommand 的目标
    core::MotionCommand applied{};  ///< 经航向修正 + omega 平滑后实际用于 IK 的命令

    double wheelTargetRpm[kWheelCount] = {0.0, 0.0, 0.0, 0.0};   ///< IK 输出（饱和前）
    double wheelCommandRpm[kWheelCount] = {0.0, 0.0, 0.0, 0.0};  ///< 最终下发（含方向/饱和/slew）
    double wheelActualRpm[kWheelCount] = {0.0, 0.0, 0.0, 0.0};   ///< 电机实测转速

    bool initialized = false;    ///< 是否已 init
    bool driveEnabled = false;   ///< 本帧驱动是否使能（enableDrive）
    bool headingActive = false;  ///< 本帧航向保持是否生效
    double headingTargetRad = 0.0;  ///< 当前航向目标 [rad]（诊断用）
    HeadingController::Debug heading;  ///< 航向 PID 内部量：err/P/I/D/corr/参数/限幅
    double omegaBeforeSlewRadPerSec = 0.0;  ///< omega 斜坡限幅前 [rad/s]
    bool headingValid = false;   ///< 本帧航向数据是否有效（航向源无关）
    bool error = false;          ///< 任一驱动电机不在线（DRV-008）
};

/**
 * @brief Chassis 模块。
 *
 * 生命周期：init() 一次；setCommand() 可任意时刻提交目标；
 * update(dt) 每控制周期一次；stop() 立即安全停机。
 */
class Chassis {
public:
    /// 初始化：保存配置、构建电机、复位航向控制器。
    void init(const ChassisConfig& config);

    /// 提交运动目标（不立即下发，由 update() 消费）。
    void setCommand(const core::MotionCommand& command);

    /// 提交最新航向状态快照；航向保持关闭时被忽略。航向源与底盘无关。
    void setHeadingState(const core::HeadingState& state);

    /// 运行一次控制周期：航向修正 → omega 平滑 → IK → 电机下发 → 刷新状态。
    void update(double dt);

    /// 只读状态快照。
    const ChassisState& getState() const { return state_; }

    /// 只读配置（离线标定/诊断换算用；避免外部另抄一份几何常量）。
    const ChassisConfig& config() const { return config_; }

    /// 安全停止：清零命令、刹车、复位 omega 平滑基准。
    void stop();

    // ---- 整车级运行时开关（ARCH-009，由 main 的 VehicleConfig 同步）----
    bool enableDrive = true;        ///< 底盘驱动模块
    bool enableHeadingHold = false; ///< 航向保持（航向源无关）
    bool enableSlewLimit = true;    ///< 加减速限制

private:
    ChassisConfig config_{};
    std::optional<ChassisDrive> drive_;  ///< 唯一电机管理者
    HeadingController heading_;

    core::MotionCommand target_{};
    core::HeadingState headingState_{};
    ChassisState state_{};

    std::uint32_t bootMs_ = 0;   ///< 初始化时刻（航向锁存计时）
    double lastOmega_ = 0.0;     ///< 上周期实际 omega（平滑基准）
    bool initialized_ = false;
};

}  // namespace chassis
