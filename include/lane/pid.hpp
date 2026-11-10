/**
 * @file lane/pid.hpp
 * @brief 墙距 PID：完整 P/I/D、积分门控、积分/输出限幅、抗饱和、D 一阶低通。
 *
 * 与完整任务书 §9 对齐：
 *   - 积分只有在 `allowIntegral=true` 且 `Ki != 0` 时才更新；
 *   - `allowIntegral=false` 时**冻结**积分（保留原值）；
 *   - `Ki == 0` 时积分状态清零，不偷偷累计；
 *   - 模式/组切换由调用者 reset()，同时清积分与 D 历史。
 *
 * 输入误差 mm，输出 m/s（横向，向右为正）。
 */
#pragma once

#include "lane/lane_config.hpp"

namespace lane {

/// 上一周期内部量（诊断用）。
struct PidDebug {
    double error = 0.0;       ///< 输入误差 [mm]
    double pTerm = 0.0;       ///< P 项（已限幅，输出单位）
    double iTerm = 0.0;       ///< I 项（已限幅，输出单位）
    double dTerm = 0.0;       ///< D 项（已限幅，输出单位）
    double output = 0.0;      ///< 总输出（已限幅）
    double integral = 0.0;    ///< 积分状态 [mm·s]
    double derivative = 0.0;  ///< 低通后的误差变化率 [mm/s]
    bool integralAllowed = false;  ///< 本周期是否允许积分
    bool saturated = false;        ///< 本周期输出是否顶到限幅
};

class Pid {
public:
    void init(const PidConfig& config);
    void reset();

    /**
     * @brief 推进一周期。
     * @param error         误差 [mm]
     * @param dt            周期 [s]
     * @param allowIntegral 是否允许累计积分；false 时冻结
     */
    double update(double error, double dt, bool allowIntegral);

    const PidDebug& debug() const { return debug_; }
    const PidConfig& config() const { return config_; }

private:
    PidConfig config_{};
    PidDebug debug_{};
    double integral_ = 0.0;   ///< 积分状态（不能放在会被清零的 debug_ 里）
    double prevError_ = 0.0;
    double prevDerivative_ = 0.0;
    bool hasPrev_ = false;
};

}  // namespace lane
