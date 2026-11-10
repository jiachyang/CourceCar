/**
 * @file modules/distance_array.cpp
 * @brief 四路 V5 Distance Sensor 读取实现。
 */
#include "modules/distance_array.hpp"

#include <cmath>

#include "distance/range_status.hpp"
#include "pros/device.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"

namespace distance {

namespace {
/// 读取失败时 PROS 返回 INT32_MAX；统一转成 -1 便于遥测。
constexpr std::int32_t kReadError = -1;

std::int32_t sanitizeInt(std::int32_t value) {
    return (value == PROS_ERR) ? kReadError : value;
}
}  // namespace

void DistanceArray::init(const DistanceConfig& config) {
    config_ = config;
    for (int i = 0; i < kSideCount; ++i) {
        sensors_[i].emplace(static_cast<std::uint8_t>(config_.ports[i]));
    }
    state_ = DistanceArrayState{};
    initialized_ = true;
}

void DistanceArray::update() {
    if (!initialized_) {
        return;
    }

    const std::uint32_t now = pros::millis();
    DistanceArrayState next{};
    next.timestampMs = now;

    for (int i = 0; i < kSideCount; ++i) {
        DistanceReading& r = next.readings[i];
        r.timestampMs = now;

        if (!sensors_[i].has_value()) {
            continue;
        }

        // 直接查 VEXos 设备注册表（与独立 P2 测距工程的端口自检方式一致），
        // 并把实际类型回传，便于区分“没插”“插错设备”“类型正确”。
        // 这样不依赖 Device::is_installed() 的实现差异/热插拔时机。
        const auto port = static_cast<std::uint8_t>(config_.ports[i]);
        const pros::DeviceType type = pros::Device::get_plugged_type(port);
        r.pluggedType = static_cast<std::int32_t>(type);
        r.installed = (type == pros::DeviceType::distance);
        if (!r.installed) {
            continue;  // 距离保持 -1
        }
        next.installedMask |= static_cast<std::uint8_t>(1u << i);

        r.distanceMm = sanitizeInt(sensors_[i]->get_distance());
        r.confidence = sanitizeInt(sensors_[i]->get_confidence());
        r.objectSize = sanitizeInt(sensors_[i]->get_object_size());

        const double velocityMps = sensors_[i]->get_object_velocity();
        if (std::isfinite(velocityMps)) {
            r.velocityMmS = static_cast<std::int32_t>(std::lround(velocityMps * 1000.0));
        }

        r.valid = (r.distanceMm >= kRangeMinMm) && (r.distanceMm <= kRangeMaxMm);
        if (r.valid) {
            next.validMask |= static_cast<std::uint8_t>(1u << i);
        }
    }

    state_ = next;
}

core::RangeSnapshot DistanceArray::toRangeSnapshot() const {
    core::RangeSnapshot snapshot{};
    snapshot.timestampMs = state_.timestampMs;

    for (int i = 0; i < kSideCount; ++i) {
        const DistanceReading& reading = state_.readings[i];
        core::RangeSample& sample = snapshot.samples[i];
        sample.distanceMm = reading.distanceMm;
        sample.confidence = reading.confidence;
        sample.timestampMs = reading.timestampMs;
        sample.status =
            classifyDistanceReading(reading.distanceMm, reading.installed);
    }
    return snapshot;
}

}  // namespace distance
