/**
 * @file lane/lane_config.hpp
 * @brief 车道保持（LaneKeeper）配置：顶点传感器布局、墙体阈值、滤波、PID、
 *        组选择与状态机参数。
 *
 * 分层（ARCH-010）：
 *   - **端口**仍由 `distance/distance_config.hpp` 唯一管理；
 *   - 本文件登记“逻辑传感器 S0..S3 ↔ 测距快照索引、车体安装位置、发射轴方向”；
 *   - 所有**未实测**的安装位置/轴向/PID 初值都标了 TODO(CALIBRATE)。
 *
 * 车体坐标沿用工程既有约定（docs/architecture.md §6）：
 *   - +X = 车头，+Y = 车体左侧，+Z 向上；角度逆时针为正。
 *
 * 传感器几何（2026-10 更正，按用户提供的俯视图）：
 *   - 4 个测距在正方形车架的**四个顶点**：车头、车尾、左侧、右侧；
 *   - 出射轴沿车体 ±X / ±Y，**不是**轮子所在的 45° 边中点；
 *   - 顶点半径 = 框架边长/√2 ≈ 0.255/1.414 = 0.1803 m；
 *   - 实际安装点/支架外伸量必须实测后修改 `sensors[]`。
 *
 * 逻辑分组：
 *   - Group A = S0 + S2 = 前 + 后；
 *   - Group B = S1 + S3 = 左 + 右。
 *   这对应图上“A 组为前后（蓝色），左右为另一组（绿色）”。
 *   物理朝向若不同，只改 `sensors[]` 与 `groupMembers`，算法不改。
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

/// 已确认的墙体识别参数（题目约定，禁止改成 5 cm）。
inline constexpr std::int32_t kWallEnterMm = 100;
inline constexpr std::int32_t kWallExitMm = 150;
inline constexpr std::int32_t kWallConfirmCount = 3;
inline constexpr std::uint32_t kWallStaleTimeoutMs = 200;
/// 两次确认样本的最小间隔 [ms]：防止同一控制周期重复读取快速凑满 3 次。
inline constexpr std::uint32_t kWallMinSampleIntervalMs = 15;

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

/// 墙体状态机参数（单位统一 mm / 控制周期数）。
struct WallConfig {
    std::int32_t enterMm = kWallEnterMm;   ///< 有效距离**严格小于**该值才累计“有墙”
    std::int32_t exitMm = kWallExitMm;     ///< 有效距离**严格大于**该值才累计“无墙”
    std::int32_t confirmCount = kWallConfirmCount;  ///< 连续有效确认样本数
    std::uint32_t staleTimeoutMs = kWallStaleTimeoutMs;  ///< 连续无效超过该时间 -> Unknown
    std::uint32_t minSampleIntervalMs = kWallMinSampleIntervalMs;  ///< 最小确认间隔 [ms]
};

/**
 * @brief 单路测距滤波参数。
 *
 * 墙体存在性用**原始有效样本**判定；PID 使用**滤波后**的可信距离。
 * 中值窗口固定 3（抗单点孤立异常），再加一阶低通；跳变需连续确认。
 */
struct RangeFilterConfig {
    double lowPassAlpha = 0.50;       ///< 0..1，1=不低通
    double jumpThresholdMm = 60.0;    ///< 单次跳变超过该值先不作为新稳态
    std::int32_t jumpConfirmSamples = 2;  ///< 连续跳变样本数达到后接受新值
    std::uint32_t resetAfterNoDistanceMs = 80;  ///< 长时间无有效距离后清滤波历史
};

/**
 * @brief 通用 PID 参数（墙距环）。
 *
 * 输入误差 mm，输出 m/s（横向，向右为正）。
 * 初值全部是保守占位，**不是整定结果**；Ki 初始 0。
 */
struct PidConfig {
    double kp = 0.0015;
    double ki = 0.0;
    double kd = 0.0003;
    double integralLimit = 200.0;   ///< 积分状态限幅（mm·s）
    double pLimit = 0.30;           ///< P 项输出限幅 [m/s]
    double iLimit = 0.10;           ///< I 项输出限幅 [m/s]
    double dLimit = 0.10;           ///< D 项输出限幅 [m/s]
    double outputLimit = 0.30;      ///< 总输出限幅 [m/s]
    double derivativeFilterAlpha = 0.30;  ///< D 一阶低通：1=不滤波
};

/**
 * @brief 状态机、速度与安全限制参数。
 *
 * 所有速度比例/时间都是保守占位；转弯检测阈值待实车标定。
 */
struct LaneStateConfig {
    std::uint32_t acquireTimeMs = 150;       ///< 启动/组切换后的重新识别时间 [ms]
    std::uint32_t postTurnAcquireMs = 150;   ///< 转弯结束后的重新识别时间 [ms]
    double acquireSpeedScale = 0.50;         ///< ACQUIRING 时基础速度比例
    double turningSpeedScale = 1.00;         ///< TURNING 时基础速度比例
    double degradedSpeedScale = 0.50;        ///< 单侧盲区/降级时的速度比例（备用）
    double faultSpeedScale = 0.00;           ///< SENSOR_FAULT 时速度比例（默认停车）

    double turningOmegaRadPerSec = 0.10;     ///< |omega| 超过该值视为主动转弯 [rad/s]
    double headingStableOmegaRadPerSec = 0.15;  ///< 实测 gz 低于该值才认为航向稳定
    std::uint32_t sensorFaultTimeoutMs = 500;  ///< 持续无法获得可信侧墙 -> 故障 [ms]

    // ---- 过近“连续远离”策略 ------------------------------------------------
    // 单侧/双侧 <20 mm 时不再直接进 ACQUIRING/FAULT 卡住，而是进入 TOO_CLOSE，
    // 用独立于墙距 PID 的排斥项主动向远离近墙的一侧修正。
    double tooCloseMm = 20.0;                 ///< 低于该原始距离视为过近 [mm]
    std::uint32_t tooCloseConfirmMs = 20;     ///< 连续过近确认时间 [ms]
    double tooCloseSpeedScale = 0.35;         ///< TOO_CLOSE 时基础速度比例
    double tooCloseGainMpsPerMm = 0.012;      ///< 过近排斥增益 [m/s per mm]
    double tooCloseMinCorrectionMps = 0.06;   ///< 最小排斥速度 [m/s]
    double tooCloseMaxCorrectionMps = 0.20;   ///< 最大排斥速度 [m/s]

    // ---- 双墙走廊门控 ------------------------------------------------------
    /// true：只有物理左侧/右侧原始距离都 < corridorEnableMm 才启用墙距控制；
    /// 单墙/无墙不纠偏。false：保留旧的单墙/无墙回退（主机测试用）。
    bool corridorOnly = true;
    /// 双侧都在该原始距离以内才认为进入“双墙走廊” [mm]。
    double corridorEnableMm = 150.0;
    /// 走廊内停车时也允许低速自动居中/远离（没有摇杆也会动）。
    bool standstillCenter = true;
    /// 停车自动纠偏的绝对速度上限 [m/s]；仍受 maxLateralAccel 平滑。
    double standstillMaxCorrectionMps = 0.15;
    /// TOO_CLOSE 停车自动远离的最小速度；要能克服底盘静摩擦。
    double standstillMinTooCloseMps = 0.10;
    /// 单侧持续“无目标/超量程”（9999）多久后，可把该侧视为“无可观测近墙”并
    /// 进入 NO_WALL/单墙。注意：这不等于 wall_present=false，墙体状态仍是 Unknown。
    std::uint32_t noWallTimeoutMs = 120;

    /// 单墙目标：**原始传感器距离** [mm]，不是车体中心到墙距离。
    /// 45 cm 通道、顶点半径 0.180 m 时，原始读数约 45 mm。
    /// TODO(CALIBRATE)：按实际通道/安全间隙标定。
    double targetWallMm = 45.0;

    /// 双墙通道宽度一致性（车体中心到左右墙的垂距之和）[m]。
    /// 45 cm 格子通道的合理带宽；不同场地必须重标。TODO(CALIBRATE)
    double minCorridorWidthM = 0.30;
    double maxCorridorWidthM = 0.60;

    double maxLateralCorrectionMps = 0.30;   ///< 横向纠偏总限幅 [m/s]
    double maxLateralAccelMps2 = 1.50;       ///< 横向纠偏变化率限幅 [m/s²]
    double maxSpeedScaleRatePerSec = 3.00;   ///< 速度比例变化率限幅 [1/s]

    /// 前向安全：用与运动方向最对齐的传感器做“前方障碍”限速。
    /// 仅当轴对齐度 >= motionSafetyMinAxisAlignment 时启用；斜向运动是感知盲区。
    double motionStopMm = 120.0;
    double motionSlowMm = 300.0;
    double motionSafetyMinAxisAlignment = 0.80;

    std::uint32_t measurementHoldMs = 40;    ///< 控制可复用最近有效距离的时间 [ms]
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

    WallConfig wall{};
    RangeFilterConfig filter{};
    PidConfig wallPid{};
    LaneStateConfig state{};

    /// 组轴与“运动左/右法向”的最小余弦。0.90 ≈ 25.8°。
    /// 45° 斜向时两组都约 0.707 < 0.90 -> 不选组。
    double minAxisAlignment = 0.90;

    /// 同组两个轴必须近似反向：夹角误差上限 [rad]，默认 15°。
    double maxPairOppositionErrorRad = 0.26;

    /// 组切换回差与最短驻留，防止零速附近/方向抖动频繁切换。
    double groupSwitchScoreMargin = 0.10;
    std::uint32_t groupSwitchDwellMs = 100;

    /// 低于该平移速度不选组、不纠偏（方向无定义）。[m/s]
    double minMotionSpeedMps = 0.02;

    /// 换算后的“车体中心到墙”最大合理垂距 [m]，超过则视为不可用。
    double maxWallClearanceM = 0.80;

    /// 传感器位置/轴向是否已实测。false 时必须先按 docs/车辆配置.md 标定。
    bool layoutVerified = false;
};

/// 默认车道保持配置。
inline LaneConfig makeDefaultLaneConfig() { return LaneConfig{}; }

/**
 * @brief 配置静态校验（几何/阈值/滤波/PID/状态机自洽性）。
 *
 * 返回 false 时 LaneKeeper 只透传基础命令，不产生横向纠偏。
 */
bool isValidLaneConfig(const LaneConfig& config);

}  // namespace lane
