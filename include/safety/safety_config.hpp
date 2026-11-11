/**
 * @file safety/safety_config.hpp
 * @brief 前向安全监督器配置（唯一安全限速出口）。
 *
 * v2.1 职责划分：LaneKeeper 只输出横向修正，不改变巡航速度；
 * 前向障碍/停车由本模块根据 ToF 与制动模型统一限速。
 *
 * 注意：当前实现只是 ToF 轴向约束，不含地图扫掠/未知挡板置信边界，
 * 不能单独作为实机零碰撞保证；上车前必须实测制动距离和传感器延迟。
 */
#pragma once

#include <cstdint>

namespace safety {

struct SafetyConfig {
    /// true：航向不可信时禁止平移。默认 false，避免 IMU 未校准/未装时
    /// 整车完全不能动；需要更高安全等级时由整车配置打开。
    bool requireHeading = false;

    double reactionS = 0.12;      ///< 反应时间 [s]
    double brakeMps2 = 1.2;       ///< 规划制动减速度 [m/s²]
    double tofReserveM = 0.022;   ///< ToF 停车余量 [m]
    double minAxisAlignment = 0.85;  ///< 传感器轴与运动方向最小余弦
    std::int32_t minConfidence = 8;  ///< V5 Distance 置信度下限
};

inline SafetyConfig makeDefaultSafetyConfig() { return SafetyConfig{}; }

/// 配置静态校验；非法时安全层应采取停车（fail-closed）。
bool isValidSafetyConfig(const SafetyConfig& config);

}  // namespace safety
