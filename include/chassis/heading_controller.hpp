/**
 * @file heading_controller.hpp
 * @brief 航向 PID 闭环：目标航向锁存、主动旋转旁路、修正角速度融合。
 */
#pragma once

#include "chassis/chassis_config.hpp"

namespace chassis {

class HeadingController {
public:
    void reset();

    void setEnabled(bool en) { enabled_ = en; }
    bool enabled() const { return enabled_; }

    /// 锁存目标航向（IMU-011）。
    void latchTarget(double yawRad);
    bool hasTarget() const { return hasTarget_; }

    /**
     * 融合外部 omega 与航向修正，返回最终 omega。
     * - 外部 omega ≠ 0 → 主动旋转，旁路航向保持（IMU-015）
     * - 外部 omega = 0 → 保持航向，叠加 PID 修正（IMU-018）
     * @param yawRad        当前航向 [rad]
     * @param externalOmega 外部 omega 指令 [rad/s]
     * @param dt            周期 [s]
     */
    double update(double yawRad, double externalOmega, double dt);

private:
    bool enabled_ = false;
    bool hasTarget_ = false;
    double targetRad_ = 0.0;
    double integ_ = 0.0;
    double prevErr_ = 0.0;
    bool wasActive_ = false;  ///< 上一周期是否为主动旋转，用于抑制切换突跳
};

}  // namespace chassis
