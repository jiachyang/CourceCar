#pragma once

#include "imu/imu_config.hpp"

#include <cstdint>

namespace imu {

/// 初始化测量与固定航向零点。纯逻辑，不读设备，也不控制电机。
struct StartupHeadingState {
    bool biasMeasured = false;
    double gyroBiasDps = 0.0;
    double gyroDriftDps = 0.0;
    bool referenceReady = false;  ///< 锁存后只由 init() 清除
    bool relativeValid = false;   ///< 本帧有效且已有零点
    double initialHeadingRad = 0.0;
    double relativeHeadingRad = 0.0;
    std::uint32_t stillMs = 0;
    std::uint32_t latchedMs = 0;
};

class StartupHeading {
public:
    void init(const ImuConfig& config, std::uint32_t nowMs);
    void update(double rotationDeg, double gyroDps, bool valid,
                std::uint32_t nowMs);
    const StartupHeadingState& getState() const { return state_; }

private:
    void clearWindow();
    ImuConfig config_{};
    StartupHeadingState state_{};
    std::uint32_t bootMs_ = 0;
    std::uint32_t windowStartMs_ = 0;
    std::uint32_t lastMs_ = 0;
    double windowStartDeg_ = 0.0;
    double gyroSumDps_ = 0.0;
    std::uint32_t count_ = 0;
    bool initialized_ = false;
};

}  // namespace imu
