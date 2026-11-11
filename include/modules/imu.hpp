/**
 * @file modules/imu.hpp
 * @brief IMU 模块：惯性传感器初始化、校准、读取与状态快照。
 *
 * 职责边界（ARCH-002/003，IMU-020）：
 *   - 唯一访问 pros::Imu 硬件的模块；
 *   - 只提供航向 / 角速度状态，**不做 EKF，不驱动底盘**；
 *   - 单位统一为工程约定：逆时针为正、rad、rad/s。
 *
 * 使用方式（见 main 的调度）：
 *   init(config) → startCalibration() → 每周期 update() → getState()
 */
#pragma once

#include <cstdint>
#include <optional>

#include "core/types.hpp"
#include "imu/imu_config.hpp"
#include "imu/startup_heading.hpp"
#include "pros/imu.hpp"

namespace imu {

/**
 * @brief IMU 状态快照（单位：rad / rad/s，逆时针为正）。
 *
 * heading_rad 是归一化到 (-π, π] 的朝向；已包含 IMU 安装偏航角
 * （`ImuConfig::mountingYawOffsetRad`）。
 * rotation_rad 是连续累计旋转角（可超出 ±π），同样包含安装偏航角；
 * 航向控制器使用它以避免跨零跳变。
 */
struct ImuState {
    double headingRad = 0.0;                ///< 归一化朝向 (-π, π]，已含安装偏航
    double rotationRad = 0.0;               ///< 连续累计旋转角，已含安装偏航
    double angularVelocityRadPerSec = 0.0;  ///< 绕 Z 轴角速度
    bool calibrating = false;               ///< 是否正在校准
    bool valid = false;                     ///< 本帧数据是否有效（未校准 / 读失败 / 跳变为 false）
    double gyroBiasDps = 0.0;               ///< 当前使用的 Z 轴角速度零偏 [deg/s]（诊断）
    double gyroDriftDps = 0.0;              ///< 当前使用的静置角度漂移率 [deg/s]（诊断）
    bool biasMeasured = false;              ///< 零偏/漂移是否已确定（手动常量或测量完成）
    std::int32_t statusRaw = 0;             ///< 原始 ImuStatus（诊断用：0=ready,19=calibrating,255=error）
    std::uint32_t timestampMs = 0;          ///< 数据更新时间 [ms]
    StartupHeadingState startup{};         ///< 固定初始航向，与底盘临时保持目标独立
};

/**
 * @brief IMU 模块。
 *
 * 生命周期：init() 一次；startCalibration() 在初始化或需要时调用；
 * update() 每控制周期一次；getState() 只读。
 */
class Imu {
public:
    /// 初始化：保存配置并按端口构建设备对象。可重复调用（重建设备）。
    void init(const ImuConfig& config);

    /// 非阻塞启动校准（IMU-004）：未安装时立即返回，不卡死。
    void startCalibration();

    /// 读取一次传感器并刷新状态。未就绪 / 读取失败 / 跳变时 valid=false。
    void update();

    /// 只读状态快照。
    const ImuState& getState() const { return state_; }

    /**
     * @brief 投影为跨模块公共航向契约（core::HeadingState）。
     *
     * Chassis 只依赖 core::HeadingState，不依赖本模块的头文件，
     * 因此换航向源（里程计 / GPS）时不需要改 Chassis。
     */
    core::HeadingState toHeadingState() const {
        return core::HeadingState{state_.rotationRad,
                                  state_.angularVelocityRadPerSec,
                                  state_.valid};
    }

    /// 传感器是否校准完成且无错误。
    bool isReady() const;

private:
    void updateSensor();
    ImuConfig config_{};
    std::optional<pros::Imu> device_;
    ImuState state_{};

    double lastRotationRad_ = 0.0;  ///< 上一有效连续角，用于跳变保护
    bool hasLastRotation_ = false;
    bool dataRateSet_ = false;      ///< 就绪后是否已下发刷新率（只做一次）
    std::uint32_t calibrationStartMs_ = 0;  ///< 校准起始时刻（超时判据）
    bool calibrationStarted_ = false;
    bool initialized_ = false;

    StartupHeading startup_;
    double rawRotationDeg_ = 0.0;
    double rawGyroDps_ = 0.0;
    std::uint32_t initMs_ = 0;
};

}  // namespace imu
