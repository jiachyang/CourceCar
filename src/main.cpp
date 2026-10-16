/**
 * @file src/main.cpp
 * @brief X-Drive 底盘入口：整车真值配置表 + 最小调用示例。
 *
 * ARCH-010：main 只放**整车级**配置（模块开关）；端口、传动比等在 chassis 模块内部。
 * 需求清单见 docs/chassis_requirements.md（rev.3），设计见 docs/chassis_design.md。
 */
#include "main.h"

#include <cstdio>

#include "chassis/chassis.hpp"

namespace {

/// 整车真值配置表（ARCH-009 / ARCH-010）：运行时可改，无需重编译。
struct VehicleConfig {
    bool enableChassisDrive = true;   ///< 底盘驱动模块
    bool enableImuHeading   = false;  ///< IMU 航向模块：未装未测，默认关（IMU-021）
    bool enableSlewLimit    = true;   ///< 加减速限制（DRV-006）
};

VehicleConfig g_vehicle;
chassis::Chassis g_chassis;

/// 把整车开关同步到各模块（运行时开关，ARCH-009）。
void syncConfig() {
    g_chassis.enableDrive     = g_vehicle.enableChassisDrive;
    g_chassis.enableImuHeading = g_vehicle.enableImuHeading;
    g_chassis.enableSlewLimit  = g_vehicle.enableSlewLimit;
}

/// 最小调用示例：低速依次演示 T-01～T-06（真实测试前请把驱动轮架空）。
void runDemoSequence() {
    struct Step {
        double vx, vy, omega;
        std::uint32_t ms;
        const char *name;
    };
    const Step plan[] = {
        { 0.20,  0.00,  0.00, 800, "T-01 fwd"},
        {-0.20,  0.00,  0.00, 800, "T-02 back"},
        { 0.00,  0.20,  0.00, 800, "T-03 left"},
        { 0.00, -0.20,  0.00, 800, "T-04 right"},
        { 0.00,  0.00,  0.50, 800, "T-05 ccw"},
        { 0.20,  0.20,  0.50, 800, "T-06 mix"},
    };

    for (const auto &s : plan) {
        std::printf("[chassis] %s\n", s.name);
        const std::uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < s.ms) {
            syncConfig();
            g_chassis.drive(s.vx, s.vy, s.omega);
            pros::delay(20);  // 控制周期 20 ms（CTRL-007）
        }
        g_chassis.stop();
        pros::delay(300);
    }
    std::printf("[chassis] demo done\n");
}

}  // namespace

void initialize() {
    std::printf("[chassis] boot\n");
    g_chassis.initialize();
    syncConfig();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runDemoSequence();
}

void opcontrol() {
    // 本阶段暂无手柄映射；直接复用演示序列，方便手动触发观察。
    runDemoSequence();
    while (true) {
        pros::delay(20);
    }
}
