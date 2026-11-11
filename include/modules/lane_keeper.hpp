/**
 * @file modules/lane_keeper.hpp
 * @brief LaneKeeper v2.1：四路 ToF 近壁通道 + 稳定方向选对 + 横向 PD。
 *
 * 职责边界（对齐 LaneKeeping_V2_1_RangeAware）：
 *   - 只输出**横向修正** `(dvx,dvy)`，叠加到上游基础命令；
 *   - **不修改**巡航速度，不输出 `speedScale`，不做前向安全急停；
 *     唯一安全限速出口见 `modules/safety_supervisor.hpp`；
 *   - `omega` 原样透传，航向 PID 仍在 Chassis 内；
 *   - 原始 `RangeSnapshot` 只读，LaneKeeper 的近壁分类仅供诊断，
 *     定位/地图必须直接读取原始测量，不能读 `NearClass`。
 *
 * 数据流：
 *
 *   基础命令(车体) ──► LaneKeeper ──► command = base + laneDelta ──► Safety ──► Chassis
 *   RangeSnapshot ────┘
 */
#pragma once

#include <cstdint>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "lane/lane_config.hpp"
#include "lane/lane_state_machine.hpp"
#include "lane/pid.hpp"
#include "lane/range_channel.hpp"
#include "lane/sensor_group_selector.hpp"

namespace lane {

/// 单路近壁/滤波诊断快照。
struct ChannelDebug {
    NearClass state = NearClass::kUnknown;
    std::int32_t rawMm = -1;      ///< 最近原始有效距离 [mm]
    double filteredMm = 0.0;      ///< 滤波后距离 [mm]
    bool observationValid = false;///< 本帧原始回波是否有效
    bool valid = false;           ///< 最近有效回波是否仍在 maxAge 内
    bool fresh = false;           ///< 是否在新鲜窗口内
    std::uint32_t ageMs = 0;      ///< 距最近有效回波的时间 [ms]
    std::uint32_t lastGoodMs = 0;
};

/// 车道保持状态快照（只读）。
struct LaneKeeperState {
    core::MotionCommand base{};     ///< 上游基础命令（车体坐标）
    core::MotionCommand command{};  ///< base + 横向 delta；omega 透传

    LaneMode mode = LaneMode::kDisabled;
    LaneReason reason = LaneReason::kNone;
    bool enabled = false;
    bool configValid = false;
    bool active = false;       ///< 本周期是否在输出横向修正/缓释
    bool valid = false;        ///< 输入/输出有限且配置合法
    bool pairValid = false;
    bool corridor = false;     ///< 双墙通道可信
    bool corridorWidthValid = false;
    bool degraded = false;
    bool decay = false;        ///< 丢墙保持：正在缓释旧纠偏
    bool fallbackPair = false; ///< 无通道方向时按物理左右/前后墙自动选对
    bool headingValid = false;

    int group = -1;            ///< 0=A,1=B,-1=未选
    bool groupChanged = false;
    bool modeChanged = false;
    std::uint32_t transitionCount = 0;

    ChannelDebug channels[kSensorCount]{};  ///< 逻辑 S0..S3

    int leftSensor = -1;
    int rightSensor = -1;
    double pairScore = 0.0;
    double normalX = 0.0;      ///< 运动左法向
    double normalY = 0.0;

    bool leftValid = false;
    bool rightValid = false;
    bool leftTooClose = false;
    bool rightTooClose = false;
    double leftMm = -1.0;      ///< 控制侧滤波距离 [mm]
    double rightMm = -1.0;
    double leftRawMm = -1.0;   ///< 控制侧原始有效距离 [mm]
    double rightRawMm = -1.0;

    /// 物理左/右（逻辑 S1/S3）最新原始距离，独立于当前运动组；
    /// 用于 UI 同时显示两侧距离，过近时仍保留原始值。
    double sideLeftRawMm = -1.0;
    double sideRightRawMm = -1.0;

    double corridorWidthM = -1.0;  ///< 左+右几何宽度 [m]，-1=不可用
    double centerErrorMm = 0.0;    ///< (右墙距-左墙距)/2，向右为正 [mm]
    PidDebug pd{};                 ///< 横向 PD 内部量
    double lateralTargetMps = 0.0; ///< 限幅后、平滑前目标 [m/s]，向右为正
    double lateralCorrectionMps = 0.0;  ///< 实际施加修正 [m/s]，向右为正
    double quality = 0.0;          ///< 1=TRACKING, 0.5=DEGRADED/TOO_CLOSE

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
     * @param heading     航向状态（仅诊断；v2.1 LaneKeeper 不用航向做速度门控）
     * @param dt          控制周期 [s]
     */
    void update(const core::MotionCommand& bodyCommand,
                const core::RangeSnapshot& ranges,
                const core::HeadingState& heading, double dt);

    /// 安全复位：清滤波/近壁分类/PD/组/模式历史。
    void reset();

    const LaneKeeperState& getState() const { return state_; }
    const LaneConfig& config() const { return config_; }

    // ---- 整车级运行时开关（ARCH-009，由 main 的 VehicleConfig 同步）----
    bool enable = false;

private:
    struct PairEvaluation {
        RangeChannelValue left{};
        RangeChannelValue right{};
        double qLeft = 0.0;
        double qRight = 0.0;
        double widthM = -1.0;
        bool widthValid = false;
        bool credible = false;
    };

    void resetControl();
    void fillChannelDebug(std::uint32_t nowMs);
    double sideRawMm(int logicalSensor, std::uint32_t nowMs) const;
    PairEvaluation evaluatePair(const PairSelection& pair,
                                std::uint32_t nowMs) const;
    PairSelection fallbackPair(std::uint32_t nowMs, bool* used) const;

    LaneConfig config_{};
    LaneKeeperState state_{};

    RangeChannel channels_[kSensorCount]{};
    Pid pd_{};
    SensorGroupSelector selector_{};
    LaneStateMachine stateMachine_{};

    double lastCorrectionMps_ = 0.0;
    double trackDirX_ = 0.0;
    double trackDirY_ = 0.0;
    bool hasTrackDir_ = false;
    bool wasCredible_ = false;
    LaneMode lastMode_ = LaneMode::kDisabled;
    int lastPairKey_ = -1;
    std::uint32_t transitionCount_ = 0;

    bool initialized_ = false;
    bool configValid_ = false;
};

}  // namespace lane
