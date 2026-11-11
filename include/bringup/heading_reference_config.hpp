#pragma once

#include <cstdint>

namespace bringup {

/// IMU-011 验收流程的时间参数；只属于 bringup，不改变正常控制周期。
struct HeadingReferenceTestConfig {
    std::uint32_t prepareTimeoutMs = 10000;  ///< 初始化阶段等待上限 [ms]
    std::uint32_t samplePeriodMs = 20;       ///< 验收采样周期 [ms]
};

}  // namespace bringup
