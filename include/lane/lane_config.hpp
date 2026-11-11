/**
 * @file lane/lane_config.hpp
 * @brief LaneKeeper v2.1 配置：近壁分类、滤波、PD、走廊与过近排斥。
 *
 * v2.1 关键语义（参考 LaneKeeping_V2_1_RangeAware）：
 *   - 原始 ToF 是完整物理测量：1.5 m / 1.9 m 的回波仍然有效，
 *     不能因为超过 150 mm 就写成“无墙”或“无目标”；
 *   - 150 mm 只是**车道近壁分类**阈值，分类结果只服务 LaneKeeper，
 *     不得反写定位/地图；
 *   - 100–150 mm 首次观测连续 3 帧后进入 MID_RETURN，可正常参与双墙判定，
 *     不会永久停在 UNKNOWN；
 *   - LaneKeeper 只输出横向修正，不修改巡航速度，也不做前向急停；
 *     前向安全由独立 SafetySupervisor 负责。
 *
 * 车体坐标沿用工程既有约定：
 *   - +X = 车头，+Y = 车体左侧，+Z 向上；角度逆时针为正。
 *
 * 传感器几何（2026-10 更正）：
 *   - 4 个测距在正方形车架的四个顶点：车头、车尾、左侧、右侧；
 *   - 出射轴沿车体 ±X / ±Y，不是轮子所在的 45° 边中点；
 *   - 顶点半径 = 框架边长/√2 ≈ 0.255/1.414 = 0.1803 m；
 *   - 实际安装点/支架外伸量必须实测后修改 `sensors[]`。
 *
 * 逻辑分组：
 *   - Group A = S0 + S2 = 前 + 后；
 *   - Group B = S1 + S3 = 左 + 右。
 */
#pragma once

#include <cstdint>

namespace lane {

/// 逻辑传感器编号。分组固定为 A={S0,S2}、B={S1,S3}。
enum SensorIndex {
    kS0 = 0,
    kS1 = 1,
    kS2 = 2,
    kS3 = 3,
    kSensorCount = 4,
};

/// 逻辑测距组编号。
enum GroupIndex {
    kGroupA = 0,
    kGroupB = 1,
    kGroupCount = 2,
};

inline constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 默认传感器“出射点”到车体中心的距离 [m]。
 *
 * 25.5 cm 正方形车架，顶点到中心 = 0.255/√2 ≈ 0.1803 m。
 * TODO(CALIBRATE)：若传感器装在支架上外伸/内缩，必须实测修改每个
 * `SensorConfig::xM/yM`，不能把 0.1803 当成已验证值。
 */
inline constexpr double kDefaultSensorRadiusM = 0.1803120669;

/// 已确认的近壁分类参数（题目约定，禁止改成 5 cm）。
inline constexpr double kNearEnterMm = 100.0;
inline constexpr double kNearExitMm = 150.0;
inline constexpr double kNearDetectMm = 150.0;
inline constexpr int kNearConfirmFrames = 3;

/**
 * @brief 单个逻辑传感器的安装与映射。
 *
 * `xM/yM` 是**测距出射点**在车体坐标中的位置；
 * `axisRad` 是出射轴方向（车体坐标，逆时针为正）。
 */
struct SensorConfig {
    std::int8_t rangeSource = 0;  ///< core::RangeSnapshot 索引：0=前,1=后,2=左,3=右
    double xM = 0.0;              ///< 出射点车体 x [m]，+X 车头
    double yM = 0.0;              ///< 出射点车体 y [m]，+Y 左侧
    double axisRad = 0.0;         ///< 出射轴方向 [rad]，车体坐标逆时针为正
    bool enabled = true;          ///< 该逻辑通道是否参与车道保持
};

/**
 * @brief 单路近壁通道：有效性、近壁分类 + 中值/低通滤波。
 *
 * 只有 RangeStatus::kOk 和 kTooClose 是有效物理回波；
 * kNoTarget / kTooFar / kReadError / kNotInstalled 都是“本帧无有效回波”，
 * 既不是近墙，也不是地图级“无墙”。短时丢帧只保留本控制器内部估计。
 */
struct RangeChannelConfig {
    double enterMm = kNearEnterMm;    ///< 严格 < 该值累计“近壁”
    double exitMm = kNearExitMm;      ///< 严格 > 该值累计“远距有效回波”
    double detectMm = kNearDetectMm;  ///< 双墙判定时两侧都必须 < 该值
    int confirmFrames = kNearConfirmFrames;  ///< 连续有效确认帧数
    double minValidMm = 0.5;          ///< 小于该值视为无效距离
    double maxValidMm = 2000.0;       ///< V5 Distance 有效量程上限
    std::uint32_t maxAgeMs = 180;     ///< 最近有效回波的最大可用时间
    std::uint32_t freshWindowMs = 42; ///< TRACKING 要求的新鲜窗口（约 2 周期）
    double jumpMm = 60.0;             ///< 单帧大跳变阈值
    int jumpConfirm = 2;              ///< 连续跳变确认帧数
    double lowPassAlpha = 0.45;       ///< 一阶低通系数，1=不滤波
};

/**
 * @brief 墙距 PD 参数。
 *
 * 输入误差 mm，输出 m/s（横向，向右为正）。
 * v2.1 使用 PD（Ki=0）；保留 I 项结构只为兼容既有 PID 实现与遥测。
 * 初值不是整定结果。
 */
struct PidConfig {
    double kp = 0.0010;
    double ki = 0.0;
    double kd = 0.000025;
    double integralLimit = 200.0;   ///< 积分状态限幅（mm·s）
    double pLimit = 0.14;           ///< P 项输出限幅 [m/s]
    double iLimit = 0.14;           ///< I 项输出限幅 [m/s]
    double dLimit = 0.14;           ///< D 项输出限幅 [m/s]
    double outputLimit = 0.14;      ///< 总输出限幅 [m/s]
    double derivativeFilterAlpha = 0.12;  ///< D 一阶低通：1=不滤波
};

/**
 * @brief 车道控制参数。
 *
 * 所有速度/时间/宽度都是保守占位；实车必须重新标定。
 */
struct LaneStateConfig {
    std::uint32_t degradedHoldMs = 150;  ///< 丢墙后按旧纠偏衰减的最长时间

    /// 顶点传感器几何下的通道宽度合理范围 [m]。
    /// 45 cm 通道：45+45+360.6 ≈ 450 mm；默认范围覆盖 37–70 cm。
    double minCorridorWidthM = 0.37;
    double maxCorridorWidthM = 0.70;

    double tooCloseMm = 20.0;       ///< 原始有效距离 < 该值视为过近
    double repulseMps = 0.10;       ///< 过近排斥速度 [m/s]

    double maxCorrectionMps = 0.14;      ///< 横向修正总限幅 [m/s]
    double maxLateralAccelMps2 = 0.70;   ///< 横向修正变化率限幅 [m/s²]
    double minAxisAlignment = 0.82;      ///< 通道法向与传感器轴的最小余弦

    /// 低于该平移速度不更新通道方向；LaneKeeper 会沿用上次方向，
    /// 这样停车时仍可按稳定通道轴居中，而不是因为速度为 0 退出。
    double minDirectionSpeedMps = 0.02;

    /// 停车自动居中：PD 输出太小、克服不了底盘静摩擦时，按方向给一个
    /// 最小横向速度。0=关闭；只在基础平移速度 < minDirectionSpeedMps 时生效。
    double standstillMinCorrectionMps = 0.08;
    /// 停车自动居中的误差死区；小于该误差不施加最小速度，避免目标附近抖动。
    double standstillDeadbandMm = 3.0;
};

/**
 * @brief 车道保持配置。
 *
 * 默认逻辑布局（**安装几何未实测**）：
 *   S0 = 前方 (source=前, 轴 +X, 顶点 (+r,0))
 *   S1 = 左侧 (source=左, 轴 +Y, 顶点 (0,+r))
 *   S2 = 后方 (source=后, 轴 π, 顶点 (-r,0))
 *   S3 = 右侧 (source=右, 轴 -Y, 顶点 (0,-r))
 */
struct LaneConfig {
    // 顺序：rangeSource, xM, yM, axisRad, enabled
    SensorConfig sensors[kSensorCount] = {
        SensorConfig{0, +kDefaultSensorRadiusM, 0.0, 0.0, true},          // S0 前
        SensorConfig{2, 0.0, +kDefaultSensorRadiusM, +kPi / 2.0, true},   // S1 左
        SensorConfig{1, -kDefaultSensorRadiusM, 0.0, +kPi, true},         // S2 后
        SensorConfig{3, 0.0, -kDefaultSensorRadiusM, -kPi / 2.0, true},   // S3 右
    };

    /// 逻辑分组成员（A=前后, B=左右）。固定逻辑定义，物理朝向由 sensors[] 决定。
    std::int8_t groupMembers[kGroupCount][2] = {{kS0, kS2}, {kS1, kS3}};

    RangeChannelConfig channel{};
    PidConfig pd{};
    LaneStateConfig state{};

    /// 传感器位置/轴向是否已实测。false 时必须先按 docs/车辆配置.md 标定。
    bool layoutVerified = false;
};

/// 默认车道保持配置。
inline LaneConfig makeDefaultLaneConfig() { return LaneConfig{}; }

/**
 * @brief 配置静态校验（几何/阈值/滤波/PD/状态参数自洽性）。
 *
 * 返回 false 时 LaneKeeper 只透传基础命令，不产生横向纠偏。
 */
bool isValidLaneConfig(const LaneConfig& config);

}  // namespace lane
