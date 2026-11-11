/**
 * @file lane/range_channel.hpp
 * @brief 单路 ToF 近壁通道：原始回波有效性、近壁分类、滤波与新鲜度。
 *
 * v2.1 语义：
 *   - `kFarReturn` 表示“有有效远距离回波”（>150 mm），**不是无墙**；
 *   - `kUnknown` 表示本控制器近窗内没有可用数据，**不是空间畅通**；
 *   - `kMidReturn` 允许 100–150 mm 首次观测在连续确认后进入稳定可见状态；
 *   - 原始距离是共享测量，本类只读入参，不修改、不截断、不反写定位。
 *
 * 墙体存在性用原始有效样本判定；滤波值只用于车道控制。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "lane/lane_config.hpp"

namespace lane {

/// 车道近壁分类（控制器局部语义，不是地图占据状态）。
enum class NearClass : std::uint8_t {
    kUnknown = 0,     ///< 近窗内没有可用回波；不等于无墙
    kNearReturn,      ///< 有效近壁回波 <100 mm
    kMidReturn,       ///< 有效中间回波 100–150 mm（可初始化的滞回带）
    kFarReturn,       ///< 有效远距离回波 >150 mm；不是无墙
};

/// 单路通道对外快照。
struct RangeChannelValue {
    bool observationValid = false;  ///< 本帧原始样本是否有效
    bool valid = false;             ///< 最近有效回波是否仍在 maxAge 内
    bool fresh = false;             ///< 是否在新鲜窗口内（TRACKING 判据）
    double mm = 0.0;                ///< 滤波后距离 [mm]
    double rawMm = 0.0;             ///< 最近原始有效距离 [mm]
    NearClass state = NearClass::kUnknown;
    std::uint32_t ageMs = 0;        ///< 距最近有效回波的时间 [ms]
};

/**
 * @brief 单路近壁通道。
 *
 * 一个逻辑传感器一个实例；所有计数只在连续有效回波上累计。
 * 无效样本不会把 kNearReturn/kFarReturn 立刻改成 kUnknown，
 * 超过 `maxAgeMs` 才释放；短时丢帧由 LaneKeeper 的 DEGRADED 处理。
 */
class RangeChannel {
public:
    void init(const RangeChannelConfig& config);
    void reset();

    /// 用一条测距样本推进通道。nowMs 为快照时刻（单调递增）。
    void update(const core::RangeSample& sample, std::uint32_t nowMs);

    /// 取当前快照；不修改内部状态。
    RangeChannelValue value(std::uint32_t nowMs) const;

    NearClass state() const { return state_; }
    bool observationValid() const { return observationValid_; }
    std::int32_t rawMm() const { return hasRaw_ ? raw_ : -1; }
    double filteredMm() const { return filtered_; }
    std::uint32_t lastGoodMs() const { return lastGoodMs_; }

private:
    void classify(double mm);
    void pushSample(double mm);
    double median() const;

    RangeChannelConfig config_{};
    NearClass state_ = NearClass::kUnknown;

    int nearCount_ = 0;
    int farCount_ = 0;
    int midCount_ = 0;
    int jumpCount_ = 0;

    double window_[3] = {0.0, 0.0, 0.0};
    int windowCount_ = 0;

    double filtered_ = 0.0;
    std::int32_t raw_ = -1;
    bool hasFiltered_ = false;
    bool hasRaw_ = false;
    bool observationValid_ = false;
    bool seen_ = false;

    std::uint32_t lastGoodMs_ = 0;
    std::uint32_t lastUpdateStamp_ = 0;
    bool hasUpdateStamp_ = false;
};

}  // namespace lane
