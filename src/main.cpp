/**
 * @file src/main.cpp
 * @brief 应用调度层（Application）：初始化模块、组织数据流、管理 PROS 生命周期。
 *
 * 架构（详见 docs/architecture.md）：
 *   main 只负责“调用哪个模块、传入什么命令、读取什么状态、按什么顺序执行”，
 *   不包含逆运动学、电机写值、IMU 读取、摇杆映射、遥测格式或标定流程。
 *
 * 应用层分工：
 *   - diag::Telemetry  —— 只读状态快照，输出 $P / $D / $S 串口帧；
 *   - bringup          —— 台架调试（静摩擦标定、点动自检），不进入正常控制。
 *
 * 数据流：
 *   Radio ──► core::MotionCommand ──► LaneKeeper(横向 delta) ──► Safety ──► Chassis
 *   IMU   ──► core::HeadingState  ──► Chassis（航向 PID，ω 修正仍由 Chassis 完成）
 *   DistanceArray ──► core::RangeSnapshot ──► LaneKeeper / SafetySupervisor
 *   *State ──► diag::Telemetry    ──► 串口遥测
 */
#include "main.h"

#include <cstdio>

#include "bringup/friction_test.hpp"
#include "config/robot_config.hpp"
#include "core/types.hpp"
#include "diag/telemetry.hpp"
#include "modules/chassis.hpp"
#include "modules/distance_array.hpp"
#include "modules/imu.hpp"
#include "modules/lane_keeper.hpp"
#include "modules/radio.hpp"
#include "modules/safety_supervisor.hpp"
#include "pros/rtos.hpp"

namespace {

/// 整车级真值配置（ARCH-009/010）：只放模块开关，硬件参数在模块内部。
config::VehicleConfig g_vehicle;

// 各模块实例：唯一硬件管理者，生命周期贯穿整个程序。
chassis::Chassis g_chassis;
imu::Imu g_imu;
radio::Radio g_radio;
distance::DistanceArray g_distance;
lane::LaneKeeper g_lane;
safety::SafetySupervisor g_safety;

// 遥测输出器：只读上面各模块的状态快照。
diag::Telemetry g_telemetry;

/// 把整车开关同步到模块（运行时开关，改 g_vehicle 即生效）。
void syncVehicleConfig() {
    g_chassis.enableDrive       = g_vehicle.enableChassisDrive;
    g_chassis.enableHeadingHold = g_vehicle.enableImuHeading;
    g_chassis.enableSlewLimit   = g_vehicle.enableSlewLimit;
    g_lane.enable               = g_vehicle.enableLaneAssist;
    g_safety.enable             = g_vehicle.enableSafety;
}

/// 一个控制周期的调度：传感器 → 遥控 → 数据传递 → 底盘执行。
void runControlCycle(double dt) {
    // 1) 更新传感器（IMU 默认关，未装未测时不影响底盘）
    if (g_vehicle.enableImu) {
        g_imu.update();
    }
    g_distance.update();  // 四路测距：供 LaneKeeper / 安全层 / 遥测使用

    // 2) 采集遥控输入（失联/超时时 state.valid=false）
    g_radio.update();

    // 3) 数据传递：航向状态 → 底盘；遥控命令 → LaneKeeper → 安全层 → 底盘
    //    底盘只认 core::HeadingState，不知道航向来自 IMU 还是别的源。
    g_chassis.setHeadingState(g_imu.toHeadingState());

    core::MotionCommand base = core::zeroMotion();
    if (g_vehicle.enableRadioControl && g_radio.getState().valid &&
        g_radio.isCommandFresh()) {
        base = g_radio.getDriveCommand();
    }

    // 车道保持只叠加横向 delta；前向安全由独立安全层统一限速。
    const core::RangeSnapshot ranges = g_distance.toRangeSnapshot();
    g_lane.update(base, ranges, g_imu.toHeadingState(), dt);
    g_safety.update(g_lane.getState().command, ranges,
                    g_imu.toHeadingState());
    g_chassis.setCommand(g_safety.getState().command);

    // 4) 底盘执行（航向修正 + IK + 电机下发全在模块内部）
    g_chassis.update(dt);
}

}  // namespace

void initialize() {
    std::printf("[app] boot\n");

    // 模块初始化：每个模块只 init 一次，配置只来自统一配置入口。
    g_chassis.init(chassis::makeDefaultChassisConfig());
    g_imu.init(imu::makeDefaultImuConfig());
    g_radio.init(radio::makeDefaultRadioConfig());
    g_distance.init(distance::makeDefaultDistanceConfig());
    g_lane.init(lane::makeDefaultLaneConfig());
    g_safety.init(safety::makeDefaultSafetyConfig());

    if (g_vehicle.enableImu) {
        g_imu.startCalibration();
    }
    syncVehicleConfig();

    // 遥测只读上述状态快照；指针在模块生命周期内保持稳定。
    diag::TelemetrySources sources;
    sources.chassis  = &g_chassis.getState();
    sources.imu      = &g_imu.getState();
    sources.radio    = &g_radio.getState();
    sources.distance = &g_distance.getState();
    sources.lane     = &g_lane.getState();
    sources.safety   = &g_safety.getState();
    g_telemetry.init(diag::makeDefaultTelemetryConfig(), sources);
}

void disabled() {
    // 禁用期间必须确保底盘停止，不得保留上一帧动作。
    g_chassis.stop();
    g_lane.reset();    // 清车道 PD/选对/近壁分类历史
    g_safety.reset();  // 清安全限速状态
}

void competition_initialize() {}

void autonomous() {
    // 暂无自主任务：保留点动自检，方便架空车轮验证方向（T-01~T-06）。
    bringup::runStaticTestSequence(g_chassis);
}

void opcontrol() {
    // 台架模式：只跑静摩擦标定，不进入遥控循环（开关见 bringup/friction_test.hpp）。
    if (bringup::kFrictionTestEnabled) {
        bringup::runFrictionTest(g_chassis, g_imu);
        g_chassis.stop();
        return;
    }

    std::uint32_t tick = pros::millis();
    std::uint32_t lastMs = 0;
    while (true) {
        const std::uint32_t now = pros::millis();
        const double dt = (lastMs == 0) ? 0.02 : (now - lastMs) / 1000.0;
        lastMs = now;

        syncVehicleConfig();
        runControlCycle(dt);
        g_telemetry.update();

        // 固定 20ms 周期（CTRL-007）：delay_until 避免 delay 累加误差。
        pros::Task::delay_until(&tick, 20);
    }
}
