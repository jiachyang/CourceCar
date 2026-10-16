/**
 * @file chassis.hpp
 * @brief 底盘门面：对外统一的 drive(vx,vy,omega) 接口，串起 IK / 驱动 / IMU / 航向。
 */
#pragma once

#include <cstdint>

#include "chassis/chassis_config.hpp"
#include "chassis/chassis_drive.hpp"
#include "chassis/heading_controller.hpp"
#include "chassis/imu_sensor.hpp"

namespace chassis {

class Chassis {
public:
    /// 初始化：记录开机时刻、复位航向控制器、按需启动 IMU 校准。
    void initialize();

    /// 对外统一接口（CTRL-001）：vx,vy [m/s]，omega [rad/s]。
    void drive(double vx, double vy, double omega);

    /// 刹车（经 ChassisDrive::stop）。
    void stop();

    // ---- 整车级运行时开关（由 main 的真值配置表同步，ARCH-009）----
    bool enableDrive = true;        ///< 底盘驱动模块
    bool enableImuHeading = false;  ///< IMU 航向模块；IMU-021 默认关
    bool enableSlewLimit = true;    ///< 加减速限制（DRV-006）

private:
    ChassisDrive drive_;
    ImuSensor imu_;
    HeadingController heading_;

    std::uint32_t bootMs_ = 0;   ///< 开机时刻
    std::uint32_t lastMs_ = 0;   ///< 上周期时刻（算 dt）
    double lastOmega_ = 0.0;     ///< 上周期实际 omega（omega 平滑用）
};

}  // namespace chassis
