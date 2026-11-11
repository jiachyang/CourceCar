/**
 * @file safety/safety_config.cpp
 * @brief 前向安全配置校验。纯逻辑，不依赖 PROS。
 */
#include "safety/safety_config.hpp"

#include <cmath>

namespace safety {

bool isValidSafetyConfig(const SafetyConfig& c) {
    if (!std::isfinite(c.reactionS) || c.reactionS < 0.0) {
        return false;
    }
    if (!std::isfinite(c.brakeMps2) || c.brakeMps2 <= 0.0) {
        return false;
    }
    if (!std::isfinite(c.tofReserveM) || c.tofReserveM < 0.0) {
        return false;
    }
    if (!std::isfinite(c.minAxisAlignment) ||
        c.minAxisAlignment <= 0.0 || c.minAxisAlignment > 1.0) {
        return false;
    }
    if (c.minConfidence < 0) {
        return false;
    }
    return true;
}

}  // namespace safety
