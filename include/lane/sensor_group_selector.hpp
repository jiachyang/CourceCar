/**
 * @file lane/sensor_group_selector.hpp
 * @brief 根据车体运动方向与已标定传感器轴，选择垂直测距组并映射左右传感器。
 *
 * 规则（完整任务书 §14）：
 *   - 组轴必须与运动方向左/右法向近似平行；
 *   - 同组两轴必须近似反向；
 *   - 切换有分数回差 + 最短驻留；
 *   - 零速/主动转弯不选组（避免抖动，转弯后台重新获取）。
 */
#pragma once

#include <cstdint>

#include "lane/lane_config.hpp"

namespace lane {

struct GroupSelection {
    int group = -1;       ///< 0=A,1=B,-1=未选
    int leftSensor = -1;  ///< 沿运动左法向的传感器
    int rightSensor = -1; ///< 沿运动右法向的传感器
    double score = 0.0;   ///< min(|a_i·t|)
    bool changed = false; ///< 本周期是否发生组变化
};

class SensorGroupSelector {
public:
    void init(const LaneConfig& config);
    void reset();

    /**
     * @param vx,vy   车体坐标目标平移速度 [m/s]
     * @param nowMs   当前时刻 [ms]
     * @param turning 是否主动转弯（转弯时不选组）
     */
    GroupSelection select(double vx, double vy, std::uint32_t nowMs,
                          bool turning);

    int currentGroup() const { return currentGroup_; }

private:
    struct Choice {
        bool valid = false;
        int group = -1;
        int leftSensor = -1;
        int rightSensor = -1;
        double score = 0.0;
    };

    Choice evaluate(int group, double tX, double tY) const;

    LaneConfig config_{};
    int currentGroup_ = -1;
    std::uint32_t lastChangeMs_ = 0;
    bool hasChange_ = false;
};

}  // namespace lane
