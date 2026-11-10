/**
 * @file diag/telemetry.hpp
 * @brief 遥测/诊断输出：把模块状态快照格式化为串口帧（$P / $D / $S）。
 *
 * 职责边界：
 *   - 只读模块状态快照，不参与控制、不修改任何模块；
 *   - 所有帧格式与字段顺序集中在本模块（main 不再拼格式串）；
 *   - 端口扫描是本模块唯一的硬件访问，仅用于接线排查（诊断用途）。
 *
 * 帧一览（字段含义见 docs/debug_log/07_调参工具.md）：
 *   $P  调参帧    默认 20 Hz  目标/实测/PID 内部量/参数
 *   $D  四路测距  默认 20 Hz  距离/置信度/大小/速度/设备类型 + 车道保持状态
 *   $S  端口扫描  默认 1 Hz   1~21 号 Smart Port 的 VEXos 设备类型
 *
 * 主机侧解析脚本：test/pid_tune.py、test/distance_monitor.py。
 * 字段顺序是固件与脚本之间的隐式契约，变更时必须同步脚本；
 * src/diag/telemetry.cpp 用 static_assert 守住 $P 的字段数。
 */
#pragma once

#include <cstdint>

#include "modules/chassis.hpp"
#include "modules/distance_array.hpp"
#include "modules/imu.hpp"
#include "modules/lane_keeper.hpp"
#include "modules/radio.hpp"

namespace diag {

/// 各帧的启停与周期（改这里即可，不用改 main）。
struct TelemetryConfig {
    bool enableTune = true;                  ///< $P 调参帧
    std::uint32_t tunePeriodMs = 20;         ///< 目标 50 Hz（受串口带宽限制约 25 Hz）
    bool enableDistance = true;              ///< $D 四路测距
    std::uint32_t distancePeriodMs = 50;     ///< 20 Hz
    bool enablePortScan = true;              ///< $S 端口扫描
    std::uint32_t portScanPeriodMs = 1000;   ///< 1 Hz
};

/// 默认遥测配置。
inline TelemetryConfig makeDefaultTelemetryConfig() { return TelemetryConfig{}; }

/**
 * @brief 遥测数据源：各模块状态快照的只读指针。
 *
 * 模块实例的生命周期必须长于 Telemetry（本工程中均为文件作用域全局对象）。
 * 允许为 nullptr：对应帧自动跳过，便于只装部分硬件时调试。
 */
struct TelemetrySources {
    const chassis::ChassisState* chassis = nullptr;
    const imu::ImuState* imu = nullptr;
    const radio::RadioState* radio = nullptr;
    const distance::DistanceArrayState* distance = nullptr;
    const lane::LaneKeeperState* lane = nullptr;  ///< 可为 nullptr：$D 车道字段填 -1
};

/**
 * @brief 遥测输出器。
 *
 * 生命周期：init() 一次；update() 每控制周期调用（内部按各自周期限频）。
 * 时间基准为 pros::millis()，与调度同源。
 */
class Telemetry {
public:
    void init(const TelemetryConfig& config, const TelemetrySources& sources);

    /// 按各帧周期限频输出；不访问除端口扫描外的任何硬件。
    void update();

private:
    TelemetryConfig config_{};
    TelemetrySources sources_{};
    std::uint32_t lastTuneMs_ = 0;
    std::uint32_t lastDistanceMs_ = 0;
    std::uint32_t lastPortScanMs_ = 0;
};

}  // namespace diag
