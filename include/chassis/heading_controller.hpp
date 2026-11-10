/**
 * @file heading_controller.hpp
 * @brief 航向 PID 闭环：目标航向锁存、主动旋转旁路、修正角速度融合。
 *
 * 只消费 IMU 提供的航向数值，不访问硬件。
 */
#pragma once

#include "chassis/chassis_config.hpp"

namespace chassis {

class HeadingController {
public:
    /// 上一周期的内部量快照（诊断用）。
    struct Debug {
        double err = 0.0;       ///< 航向误差 err = wrapToPi(target - yaw) [rad]
        double pTerm = 0.0;     ///< P 项 Kp·err（限幅后）[rad/s]
        double iTerm = 0.0;     ///< I 项 Ki·integ（限幅后）[rad/s]
        double dTerm = 0.0;     ///< D 项 Kd·deriv（限幅后）[rad/s]
        double corr = 0.0;      ///< 总输出 P+I+D（限幅后）[rad/s]
        double ffTerm = 0.0;    ///< 静摩擦前馈（本周期生效值）[rad/s]
        double extOmega = 0.0;  ///< 外部 omega [rad/s]
        bool active = false;    ///< 本周期是否处于主动旋转
        bool ffActive = false;  ///< 本周期静摩擦前馈是否启用
        // 参数与限幅（常量，一并回传便于对账）
        double kp = 0.0, ki = 0.0, kd = 0.0;
        double integralLimit = 0.0;                 ///< 积分状态限幅 [rad·s]
        double pLimit = 0.0, iLimit = 0.0, dLimit = 0.0;  ///< 单项限幅 [rad/s]
        double maxCorr = 0.0;                       ///< 总输出限幅 [rad/s]
    };

    void reset();

    /// 由 Chassis 在初始化时从 ChassisConfig 注入 PID 参数。
    void setGains(double kp, double ki, double kd,
                  double integralLimit, double maxCorrectionRadPerSec);

    /// 注入 P/I/D 单项输出限幅 [rad/s]。
    void setTermLimits(double pLimit, double iLimit, double dLimit) {
        pLimit_ = pLimit;
        iLimit_ = iLimit;
        dLimit_ = dLimit;
    }

    /// 注入方向相关的静摩擦前馈 [rad/s] 与启用条件。
    /// @param pos           err > 0（需要逆时针）时的前馈，正值
    /// @param neg           err < 0（需要顺时针）时的前馈，负值
    /// @param rateThreshold 仅 |实测 gyro z| <= 此值时启用 [rad/s]
    /// @param errorDeadband |err| <= 此值时不启用 [rad]
    void setFrictionFeedforward(double pos, double neg,
                                double rateThreshold, double errorDeadband) {
        ffPos_ = pos;
        ffNeg_ = neg;
        ffRateThreshold_ = rateThreshold;
        ffErrorDeadband_ = errorDeadband;
    }

    void setEnabled(bool en) { enabled_ = en; }
    bool enabled() const { return enabled_; }

    /// 锁存目标航向（IMU-011）。
    void latchTarget(double yawRad);
    bool hasTarget() const { return hasTarget_; }
    /// 当前目标航向 [rad]（诊断用）。
    double targetRad() const { return targetRad_; }
    /// 上一周期内部量（诊断用）。
    const Debug& debug() const { return debug_; }

    /**
     * 融合外部 omega 与航向修正，返回最终 omega。
     * - 外部 omega ≠ 0 → 主动旋转：旁路航向保持，并让目标跟随当前朝向
     *   （松手即重锁目标，避免转回旧目标的自转）（IMU-015）
     * - 外部 omega = 0 → 保持航向，叠加 PID 修正（IMU-018）
     * @param yawRad        当前航向 [rad]
     * @param externalOmega 外部 omega 指令 [rad/s]
     * @param measuredOmega 实测车体角速度（gyro z）[rad/s]
     * @param dt            周期 [s]
     */
    double update(double yawRad, double externalOmega, double measuredOmega,
                  double dt);

    /// 兼容旧调用：不提供实测角速度时按“静止”处理。
    double update(double yawRad, double externalOmega, double dt) {
        return update(yawRad, externalOmega, 0.0, dt);
    }

private:
    bool enabled_ = false;
    bool hasTarget_ = false;
    double targetRad_ = 0.0;
    double integ_ = 0.0;
    double prevErr_ = 0.0;
    bool wasActive_ = false;  ///< 上一周期是否为主动旋转，用于抑制切换突跳

    double kp_ = 1.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double integralLimit_ = 0.5;
    double pLimit_ = 1e9;  ///< P 项输出限幅 [rad/s]（默认极大=不限）
    double iLimit_ = 1e9;  ///< I 项输出限幅 [rad/s]
    double dLimit_ = 1e9;  ///< D 项输出限幅 [rad/s]
    double maxCorrection_ = 2.0;

    // 静摩擦前馈（默认 0，不影响纯数学单测）
    double ffPos_ = 0.0;
    double ffNeg_ = 0.0;
    double ffRateThreshold_ = 0.05;
    double ffErrorDeadband_ = 0.0;

    Debug debug_{};
};

}  // namespace chassis
