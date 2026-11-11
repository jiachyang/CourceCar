#pragma once

#include "bringup/heading_reference_config.hpp"
#include "diag/telemetry.hpp"
#include "modules/chassis.hpp"
#include "modules/imu.hpp"

namespace bringup {
/// IMU-011 初始化验收采样；无 IMU/仍在动时超时退出，后续测试循环继续等待。
void prepareHeadingReference(imu::Imu& imu, diag::Telemetry& telemetry,
                             const HeadingReferenceTestConfig& config = {});
/// driver/autonomous 共用的纯传感器测试循环，进入时立即停止电机。
void runHeadingReferenceTest(chassis::Chassis& chassis, imu::Imu& imu,
                             diag::Telemetry& telemetry,
                             const HeadingReferenceTestConfig& config = {});
}  // namespace bringup
