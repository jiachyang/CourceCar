/**
 * @file imu_sensor.hpp
 * @brief IMU 初始化、校准状态与航向读取。唯一访问 IMU 硬件的模块。
 */
#pragma once

#include "chassis/chassis_config.hpp"
#include "pros/imu.hpp"

namespace chassis {

class ImuSensor {
public:
    ImuSensor();

    /// 非阻塞启动校准（IMU-004）。IMU 缺失时不会卡死。
    void startCalibration();

    /// 校准完成且无错误（T-10：未就绪时不得使用航向）。
    bool isReady();

    /// 传感器是否处于错误状态（IMU-009）。
    bool hasError();

    /**
     * 读取航向，单位为弧度，工程约定：逆时针为正，连续可超出 ±π。
     * @return 有效返回 true 并写入 outRad；未就绪/无效返回 false。
     */
    bool readHeadingRad(double &outRad);

private:
    pros::Imu imu_;
};

}  // namespace chassis
