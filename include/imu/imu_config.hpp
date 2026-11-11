/**
 * @file imu/imu_config.hpp
 * @brief IMU 模块配置：端口、安装朝向、采样与异常判据。
 *
 * 注意：PROS 文档写 IMU 的 heading / rotation 以顺时针为正，
 * 但实际符号会受安装面/传感器朝向影响。本工程用 `yawSign` 统一换算到
 * “逆时针为正”（CS-008）；当前实机标定为 **+1.0**（见 YAW_SIGN 调试记录）。
 *
 * 安装朝向：IMU 绕竖直轴旋转安装时，航向零点会差一个固定角度。
 * 用 `mountingYawOffsetRad` 把“IMU +x 轴”换算到“车头方向”。
 * 该偏置只影响航向角度，不影响 Z 轴角速度和航向闭环的相对行为。
 * TODO(CALIBRATE)：IMU 装好后若发现修正方向相反，只改 `yawSign`；
 * 若航向零点差 45°，改 `mountingYawOffsetRad` 的符号/数值。
 */
#pragma once

#include <cstdint>

namespace imu {

inline constexpr std::int8_t kPort = 6;  ///< IMU 端口（HW-005：Smart Port 6）
inline constexpr std::uint32_t kCalibrationTimeoutMs = 3000;
/// 请求的 IMU 刷新率 [ms]。参考工程实测：设备最小 5ms，但 VEXos 共享缓存
/// 仍每 10ms 更新一次，设 <10ms 只保证数据尽量新、不会更快。20ms 控制周期够用。
inline constexpr std::uint32_t kDataRateMs = 5;
inline constexpr double kYawSign = 1.0;             ///< 原始航向→工程航向的符号；实测 +1.0（逆时针为正）
inline constexpr double kMaxDeltaYawPerCycleRad = 0.35;  ///< 单周期跳变保护

/// 车头相对 IMU +x 轴的安装偏航角 [rad]，工程约定逆时针为正。
/// 当前安装：**车头 = IMU +x 向 +y 偏 45°** → +π/4。
/// 说明：
///   - 偏置加到 heading / rotation 上，角速度 gyro.z 不受影响；
///   - 航向保持是相对闭环，常值偏置会在“目标-当前”中抵消，不改变保持行为；
///   - 若车头实际在 +x 的顺时针侧，改为 **−π/4**；
///   - 若启动时以“车头”对准场地参考（而不是以 IMU +x 对准），可设为 0。
inline constexpr double kMountingYawOffsetRad = 0.7853981633974483;  // +π/4

// -----------------------------------------------------------------------------
// 静置漂移补偿
// -----------------------------------------------------------------------------
// 分两部分（都在 IMU 原始帧，顺时针为正，未乘 yawSign）：
//   - 角速度零偏 kGyroBiasDps [deg/s]：用于校正上报的 angularVelocity；
//   - 角度漂移率 kGyroDriftDps [deg/s]：用于扣除 get_rotation() 的静置漂移。
// 为什么分开：VEXos 的 get_rotation() 已对零偏做过积分补偿，直接用角速度
// 零偏去扣角度会“二次补偿”反而制造漂移（实测验证）；所以角度用**实测漂移率**。
//
// - `kUseManualGyroBias=false`：校准后静置 `kGyroBiasMeasureMs` 自动测一次，
//   作为本次运行的常量（遥测 `bias=` / `drift=`，单位 mdps）。
// - `kUseManualGyroBias=true`：直接用下列常量，跳过自动测量。
// 建议：先自动测一次 → 读遥测 → 填到这里并改为 true 固定下来。
inline constexpr double kGyroBiasDps = 0.0;   ///< Z 轴角速度零偏 [deg/s]
inline constexpr double kGyroDriftDps = 0.0;  ///< 静置角度漂移率 [deg/s]
inline constexpr bool kUseManualGyroBias = false;
/// 自动测量窗口 [ms]：期间请保持车体静止。
inline constexpr std::uint32_t kGyroBiasMeasureMs = 2000;

/// IMU 模块配置。
struct ImuConfig {
    std::int8_t port                        = kPort;
    std::uint32_t calibrationTimeoutMs      = kCalibrationTimeoutMs;
    std::uint32_t dataRateMs                = kDataRateMs;
    double yawSign                          = kYawSign;
    double mountingYawOffsetRad             = kMountingYawOffsetRad;
    double maxDeltaYawPerCycleRad           = kMaxDeltaYawPerCycleRad;
    double gyroBiasDps                      = kGyroBiasDps;
    double gyroDriftDps                     = kGyroDriftDps;
    bool useManualGyroBias                  = kUseManualGyroBias;
    std::uint32_t gyroBiasMeasureMs         = kGyroBiasMeasureMs;
    // 初始化航向基准：程序启动至少 5s、零偏已测、旋转静止才锁存。
    std::uint32_t referenceDelayMs = 5000;
    std::uint32_t referenceStillMs = 500;
    std::uint32_t startupMaxSampleGapMs = 100;
    double startupMaxGyroDps = 2.0;
    double startupMaxAngleChangeDeg = 0.5;
    double gyroDriftDeadbandDps = 0.02;  ///< 静置漂移率诊断噪声门限 [deg/s]
};

/// 默认 IMU 配置。
inline ImuConfig makeDefaultImuConfig() { return ImuConfig{}; }

}  // namespace imu
