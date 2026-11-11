/**
 * @file lane/sensor_group_selector.hpp
 * @brief 按稳定通道方向选择前后/左右测距对，并映射运动左/右传感器。
 *
 * v2.1 规则：
 *   - 输入是**通道方向**（来自 RouteSegment 或 LaneKeeper 记住的最近平移方向），
 *     不是每帧修正后的瞬时速度；避免纠偏改变法向、进而反复换组；
 *   - 前后运动用左右传感器（Group B），横向运动用前后传感器（Group A）；
 *   - 45° 斜向且法向匹配不足（默认 <0.82）不输出常规居中修正；
 *   - 左右只由传感器轴在运动左法向 `t=(-dy,dx)` 上的投影符号决定。
 */
#pragma once

#include <cstdint>

#include "lane/lane_config.hpp"

namespace lane {

struct PairSelection {
    bool valid = false;
    int group = -1;        ///< 0=A(前后), 1=B(左右), -1=未选
    int leftSensor = -1;   ///< 运动左法向的传感器
    int rightSensor = -1;  ///< 运动右法向的传感器
    double leftNormalX = 0.0;   ///< 运动左法向 t=(-dy,dx)
    double leftNormalY = 0.0;
    double score = 0.0;    ///< max(|t·传感器轴|)
};

class SensorGroupSelector {
public:
    void init(const LaneConfig& config);
    void reset();

    /**
     * @param dirX,dirY 车体坐标通道方向（无需归一化）
     * @param directionValid 是否已有可信通道方向
     */
    PairSelection select(double dirX, double dirY, bool directionValid) const;

private:
    LaneConfig config_{};
};

}  // namespace lane
