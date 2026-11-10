/**
 * @file modules/distance_array.hpp
 * @brief 四路 V5 Distance Sensor（前/后/左/右）读取模块。
 *
 * 职责边界：
 *   - 唯一访问四路 pros::Distance 的模块；
 *   - 只提供原始距离/置信度/大小/速度与有效性状态；
 *   - 通过 `toRangeSnapshot()` 把快照交给 LaneKeeper / diag；
 *   - 不做滤波/EKF，也不直接控制底盘。
 */
#pragma once

#include <cstdint>
#include <optional>

#include "core/range_types.hpp"
#include "distance/distance_config.hpp"
#include "pros/distance.hpp"

namespace distance {

/// 单路测距读数。
struct DistanceReading {
    std::int32_t distanceMm = -1;    ///< 原始距离 [mm]；-1 = 读取失败
    std::int32_t confidence = -1;    ///< 0–63；-1 = 读取失败
    std::int32_t objectSize = -1;    ///< 0–400；-1 = 无效
    std::int32_t velocityMmS = 0;    ///< 物体速度 [mm/s]（原始 m/s ×1000）
    std::int32_t pluggedType = 0;    ///< VEXos 识别的设备类型（7=Distance，0=无设备）
    bool installed = false;          ///< 端口上是否识别到 Distance Sensor
    bool valid = false;              ///< installed && 20 ≤ d ≤ 2000
    std::uint32_t timestampMs = 0;   ///< 本路读取时刻 [ms]
};

/// 四路测距状态快照。
struct DistanceArrayState {
    DistanceReading readings[kSideCount];  ///< 顺序：前、后、左、右
    std::uint8_t installedMask = 0;        ///< bit0 前、bit1 后、bit2 左、bit3 右
    std::uint8_t validMask = 0;            ///< 同上；1 = 本路有效
    std::uint32_t timestampMs = 0;         ///< 最近一次 update() 时刻
};

/**
 * @brief 四路距离传感器数组。
 *
 * init() 一次；update() 每控制周期调用；getState() 只读。
 * 未安装/读数失败不会卡住，只把对应状态置为 invalid。
 */
class DistanceArray {
public:
    void init(const DistanceConfig& config);
    void update();
    const DistanceArrayState& getState() const { return state_; }

    /**
     * @brief 投影为跨模块测距契约（core::RangeSnapshot）。
     *
     * LaneKeeper 只认 `core::RangeSnapshot`，不 include 本模块头文件。
     * 索引顺序保持 前、后、左、右；有效性由 `distance::classifyDistanceReading`
     * 统一分类（PROS_ERR / 9999 / 过近 / 超量程 各自独立）。
     */
    core::RangeSnapshot toRangeSnapshot() const;

private:
    DistanceConfig config_{};
    DistanceArrayState state_{};
    std::optional<pros::Distance> sensors_[kSideCount];
    bool initialized_ = false;
};

}  // namespace distance
