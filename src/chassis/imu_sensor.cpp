/**
 * @file imu_sensor.cpp
 * @brief IMU 校准与航向读取实现。
 */
#include "chassis/imu_sensor.hpp"

#include <cmath>

namespace chassis {

ImuSensor::ImuSensor() : imu_(kImuPort) {}

void ImuSensor::startCalibration() {
    // 非阻塞：IMU 缺失/异常时立即返回，不卡住初始化（IMU-004）
    imu_.reset(false);
}

bool ImuSensor::isReady() {
    return imu_.get_status() == pros::ImuStatus::ready;
}

bool ImuSensor::hasError() {
    return imu_.get_status() == pros::ImuStatus::error;
}

bool ImuSensor::readHeadingRad(double &outRad) {
    if (!isReady()) {
        return false;  // IMU-009/T-10：未就绪不使用航向
    }
    const double deg = imu_.get_rotation();  // 连续角度（度）
    if (!std::isfinite(deg)) {
        return false;
    }
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    // IMU 顺时针为正 → 乘 kImuYawSign 转成工程约定（逆时针为正）
    outRad = deg * kDegToRad * kImuYawSign;
    return true;
}

}  // namespace chassis
