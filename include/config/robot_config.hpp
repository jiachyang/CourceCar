/**
 * @file config/robot_config.hpp
 * @brief 整车配置入口：汇聚各模块配置 + 整车级编译期默认开关。
 *
 * 分层（ARCH-010）：
 *   - 模块级硬件/控制真值  → 各模块的 *_config.hpp（chassis / imu / radio）
 *   - 整车级开关与行为     → 本文件的 VehicleConfig
 *
 * main / Application 只 include 本文件，即可拿到全部配置类型。
 */
#pragma once

#include "chassis/chassis_config.hpp"
#include "distance/distance_config.hpp"
#include "imu/imu_config.hpp"
#include "radio/radio_config.hpp"

namespace config {

/**
 * @brief 整车级编译期默认配置（ARCH-009/010）。
 *
 * main 在初始化时读取，并在每个周期同步给对应模块；当前没有外部修改入口，
 * 用户修改这些默认值后须重新编译上传，不能称为已实现运行时配置。
 */
struct VehicleConfig {
    /// 本轮只验收固定初始航向。true：driver/autonomous 均只采集 IMU；
    /// false：恢复原有控制流程，initialize 不执行航向测试的等待循环。
    bool enableHeadingReferenceTest = true;
    bool enableChassisDrive = true;   ///< 底盘驱动模块（关=不发任何电机指令）
    bool enableImu          = true;   ///< IMU 模块：已装（端口 6），开启校准与读取
    bool enableImuHeading   = true;   ///< IMU 航向保持：开启（Kp 初值 1.0，待整定）
    bool enableSlewLimit    = true;   ///< 加减速限制（DRV-006）
    bool enableRadioControl = true;   ///< 遥控输入模块（opcontrol 中生效）
    bool enableSafety       = true;   ///< 前向 ToF 安全限速（唯一安全出口）

    /// 车道保持（LaneKeeper）：**本次实车测试临时打开**。
    /// 传感器安装位置/轴向/PID 仍未标定（LaneConfig::layoutVerified=false）；
    /// 测试必须低速，先确认四路测距与左右映射；异常时立即改回 false 重新烧录。
    bool enableLaneAssist   = true;
};

}  // namespace config
