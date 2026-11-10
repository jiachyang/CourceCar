/**
 * @file modules/lane_keeper.hpp
 * @brief LaneKeeper 模块：四路近墙测距 + 组选择 + 双墙/单墙 PID + 状态机。
 *
 * 定位：**局部运动纠偏**，不替代全局路径规划，也不直接控制电机。
 * 输出是一个 `core::MotionCommand`（基础速度按状态限速 + 横向纠偏，
 * omega 原样透传），交给 Chassis 的既有链路：
 *
 *   基础命令(车体) --> LaneKeeper --(vx,vy,omega)--> Chassis
 *                                                      ├ IMU 航向 PID（既有 HeadingController）
 *                                                      └ X-Drive IK -> 四电机
 *
 * 因此 **IMU 航向 PID 没有在本模块重写**；LaneKeeper 只改 vx/vy。
 *
 * 输入是车体坐标 `core::MotionCommand` + `core::RangeSnapshot` +
 * `core::HeadingState`。若上游给世界坐标方向，先用
 * `lane::worldVelocityToBody()` 转换。
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "lane/lane_config.hpp"
#include "lane/lane_state_machine.hpp"
#include "lane/pid.hpp"
#include "lane/range_filter.hpp"
#include "lane/sensor_group_selector.hpp"
#include "lane/wall_state.hpp"

namespace lane {

/// 单路墙体/滤波诊断快照。
struct WallDebug {
    WallStatus status = WallStatus::kUnknown;
    std::int32_t rawMm = -1;        ///< 最近原始有效距离 [mm]
    double filteredMm = 0.0;        ///< 滤波后距离 [mm]；无历史时无意义
    bool rawValid = false;          ///< 原始距离是否在 hold 窗口内
    bool filterValid = false;       ///< 滤波距离是否在 hold 窗口内
    bool usable = false;            ///< Present && filterValid，可用于 PID
    std::uint32_t rawMs = 0;        ///< 最近原始样本时刻
    std::uint32_t filteredMs = 0;   ///< 最近滤波有效时刻
};

/// 车道保持状态快照（只读）。
struct LaneKeeperState {
    core::MotionCommand base{};     ///< 调用者输入的基础命令（车体坐标）
    /// 输出命令：基础 vx/vy 按状态限速后叠加横向纠偏；omega 原样透传。
    /// 最终航向修正后的 omega 由 Chassis 的 IMU 航向 PID 产生。
    core::MotionCommand command{};

    LaneState state = LaneState::kDisabled;
    LaneReason reason = LaneReason::kNone;
    bool enabled = false;
    bool configValid = false;
    bool active = false;            ///< 本周期是否允许墙距纠偏
    bool valid = false;             ///< 输入/输出有限且配置合法
    bool turning = false;
    bool degraded = false;
    bool fault = false;
    bool headingValid = false;
    bool headingStable = true;

    std::int8_t group = -1;         ///< 0=A,1=B,-1=未选
    bool groupChanged = false;
    bool stateChanged = false;
    std::uint32_t transitionCount = 0;

    WallDebug walls[kSensorCount]{};  ///< 逻辑 S0..S3

    bool leftObserved = false;      ///< 运动左法向是否有可用墙距
    bool rightObserved = false;
    /// 运动左/右法向的原始距离 <20 mm：连续确认后进入 TOO_CLOSE 主动远离。
    bool leftTooClose = false;
    bool rightTooClose = false;
    bool corridorActive = false;   ///< 双墙走廊门控：两侧原始距离都 <150 mm
    /// 物理左侧/右侧（逻辑 S1/S3）的最新原始距离，独立于当前运动组；
    /// 用于 UI 同时显示两侧距离（过近时仍保留原始值）。
    double sideLeftRawMm = -1.0;
    double sideRightRawMm = -1.0;
    double leftDistanceMm = -1.0;   ///< 左传感器滤波距离 [mm]（PID 控制量）
    double rightDistanceMm = -1.0;
    double leftRawMm = -1.0;        ///< 左传感器最近原始有效距离 [mm]（积分门控）
    double rightRawMm = -1.0;
    double leftReferenceM = -1.0;   ///< 车体中心->左墙垂距 [m]（双墙/宽度检查）
    double rightReferenceM = -1.0;
    double corridorWidthM = -1.0;   ///< 左+右垂距 [m]，-1=不可用
    bool corridorWidthValid = false;///< 宽度是否在配置的合理通道范围内

    double targetWallMm = 0.0;      ///< 单墙目标（原始测距）[mm]
    double lateralErrorMm = 0.0;    ///< 墙距 PID 误差，向右为正 [mm]
    PidDebug wallPid{};             ///< 墙距 PID 内部量
    double lateralTargetMps = 0.0;  ///< 限幅后、平滑前的目标纠偏 [m/s]，向右为正
    double lateralCorrectionMps = 0.0;  ///< 实际施加纠偏 [m/s]，向右为正

    double stateSpeedScale = 1.0;   ///< 状态机给出的基础速度比例
    double targetSpeedScale = 1.0;  ///< 状态*前向安全后的目标比例
    double speedScale = 1.0;        ///< 实际施加比例（带变化率限幅）
    double forwardSafetyScale = 1.0;
    double motionObstacleMm = -1.0; ///< 运动方向最近障碍 [mm]，-1=未测/不适用

    std::uint32_t timestampMs = 0;
};

/**
 * @brief 车道保持模块。
 *
 * 生命周期：init() 一次；update() 每控制周期一次；reset() 在 disabled 时调用。
 */
class LaneKeeper {
public:
    void init(const LaneConfig& config);

    /**
     * @param bodyCommand 基础运动命令（车体坐标，omega 原样透传）
     * @param ranges      四路测距快照（前/后/左/右）
     * @param heading     当前航向/角速度（IMU 契约；可 invalid）
     * @param dt          控制周期 [s]
     */
    void update(const core::MotionCommand& bodyCommand,
                const core::RangeSnapshot& ranges,
                const core::HeadingState& heading, double dt);

    /// 安全复位：清滤波/墙体确认/PID/组/状态机历史（disabled 时调用）。
    void reset();

    const LaneKeeperState& getState() const { return state_; }
    const LaneConfig& config() const { return config_; }

    // ---- 整车级运行时开关（ARCH-009，由 main 的 VehicleConfig 同步）----
    bool enable = false;

private:
    bool referenceClearanceM(int sensor, double nX, double nY,
                             std::uint32_t nowMs, double* outM) const;
    double forwardSafetyScale(const core::MotionCommand& base,
                              std::uint32_t nowMs,
                              double* obstacleMm) const;
    void resetControl();

    LaneConfig config_{};
    LaneKeeperState state_{};

    WallTracker walls_[kSensorCount]{};
    RangeFilter filters_[kSensorCount]{};
    Pid wallPid_{};
    SensorGroupSelector selector_{};
    LaneStateMachine stateMachine_{};

    int leftSensor_ = -1;
    int rightSensor_ = -1;
    std::int8_t lastGroup_ = -1;
    double lastLateralCorrectionMps_ = 0.0;
    double lastSpeedScale_ = 1.0;

    bool initialized_ = false;
    bool configValid_ = false;
    std::uint32_t transitionCount_ = 0;
};

}  // namespace lane
