/**
 * @file math_util.hpp
 * @brief 通用小工具：角度归一化、限幅。纯内联，不依赖 PROS。
 */
#pragma once

#include <cmath>

namespace chassis {

/// 把角度归一化到 (-π, π]，用于最短角差（IMU-017）。
inline double wrapToPi(double rad) {
    constexpr double kPi = 3.14159265358979323846;
    return std::remainder(rad, 2.0 * kPi);
}

/// 简单限幅。
inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace chassis
