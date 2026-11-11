/**
 * @file sim/lane_keeping_sim.cpp
 * @brief 车道保持闭环主机仿真：真实迷宫地图 + 真实底盘参数 + 固件 LaneKeeper。
 *
 * 与 src/sim/maze_nav_sim.html 的区别：
 *   - 这里直接编译 `src/modules/lane_keeper.cpp`，不是重写一份 JS；
 *   - 只用 45 cm 真实走廊做“车道保持”切片，不跑路径规划；
 *   - 车辆模型用 chassis_config.hpp + kinematics.cpp 的参数与公式，
 *     ToF 用 maze_model.json 的真实墙段做射线。
 *
 * 仅主机端工具，不参与 PROS 构建，也不在 scripts/test_native.sh 里。
 *
 * 构建/运行：bash sim/run_lane_keeping.sh
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "chassis/chassis_config.hpp"
#include "chassis/kinematics.hpp"
#include "core/range_types.hpp"
#include "core/types.hpp"
#include "fixtures/maze_model.hpp"
#include "lane/lane_config.hpp"
#include "maze_world.hpp"
#include "modules/lane_keeper.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.02;  ///< 控制周期 50 Hz（CTRL-007）

// 真实地图中的直走廊：cell (7, y)，x 中心 3.375 m，左右墙 x=3.15/3.60。
// 左墙在 y=0.45..2.25 连续，长度 1.8 m。
constexpr double kCorridorCenterX = 3.375;
constexpr double kCorridorNorth = -kPi / 2.0;  ///< 车头朝北（世界 y 向上）

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double h = kCorridorNorth;  ///< 车体 +X（车头）的世界角，世界 y 向下
};

struct PlantConfig {
    chassis::ChassisConfig chassis = chassis::makeDefaultChassisConfig();
    double motorTauS = 0.09;      ///< 电机一阶惯性
    double stictionRpm = 1.2;     ///< 低于该转速直接吸附（近似静摩擦）
    double slip = 0.0;            ///< 乘性打滑（只影响位移，不影响传感器）
};

struct Plant {
    PlantConfig cfg{};
    double wheelCmd[4] = {0.0, 0.0, 0.0, 0.0};
    double wheelAct[4] = {0.0, 0.0, 0.0, 0.0};

    void reset() {
        for (int i = 0; i < 4; ++i) {
            wheelCmd[i] = 0.0;
            wheelAct[i] = 0.0;
        }
    }

    /// 一步：IK -> slew -> 一阶惯性 -> 正运动学 -> 位姿积分。
    core::MotionCommand step(const core::MotionCommand& command, Pose* pose,
                             double dt, std::mt19937* rng) {
        double target[4] = {0.0, 0.0, 0.0, 0.0};
        chassis::bodyVelocityToWheelRpm(cfg.chassis, command.vx, command.vy,
                                        command.omega, target);

        const double maxStep = cfg.chassis.slewRpmPerCycle * (dt / 0.02);
        const double alpha = 1.0 - std::exp(-dt / cfg.motorTauS);
        double rim[4] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 4; ++i) {
            double t = std::clamp(target[i], -cfg.chassis.maxWheelRpm,
                                  cfg.chassis.maxWheelRpm);
            wheelCmd[i] += std::clamp(t - wheelCmd[i], -maxStep, maxStep);
            wheelAct[i] += (wheelCmd[i] - wheelAct[i]) * alpha;
            if (std::fabs(wheelCmd[i]) < cfg.stictionRpm &&
                std::fabs(wheelAct[i]) < cfg.stictionRpm) {
                wheelAct[i] = 0.0;
            }
            rim[i] = wheelAct[i] * (2.0 * kPi / 60.0) *
                     cfg.chassis.wheelRadiusM;
        }

        // 逆运动学的反解（与 src/sim/maze_nav_sim.html 的 wheelToBody 一致）。
        constexpr double kSqrt2 = 1.41421356237309504880;
        const double q = kSqrt2 / 4.0;
        const double vx = q * (rim[chassis::kFr] + rim[chassis::kRr] -
                               rim[chassis::kFl] - rim[chassis::kRl]);
        const double vy = q * (rim[chassis::kFl] + rim[chassis::kFr] -
                               rim[chassis::kRl] - rim[chassis::kRr]);
        const double omega =
            (rim[chassis::kFl] + rim[chassis::kFr] + rim[chassis::kRl] +
             rim[chassis::kRr]) /
            (4.0 * cfg.chassis.centerToWheelM);

        double slipScale = 1.0;
        if (cfg.slip > 0.0) {
            std::normal_distribution<double> n(0.0, cfg.slip);
            slipScale = std::max(0.0, 1.0 + n(*rng));
        }

        const double mid = pose->h - omega * dt * 0.5;
        const double fx = std::cos(mid);
        const double fy = std::sin(mid);
        const double lx = std::sin(mid);
        const double ly = -std::cos(mid);
        pose->x += (vx * fx + vy * lx) * dt * slipScale;
        pose->y += (vx * fy + vy * ly) * dt * slipScale;
        pose->h -= omega * dt;  // 固件 +omega = 俯视逆时针；世界 y 向下
        return core::MotionCommand{vx, vy, omega};
    }
};

struct ToFConfig {
    double sigmaM = 0.001;  ///< 单次测距标准差（实测重复性 ~0.3 mm，取保守值）
    double beamJitterRad = 0.009;
    double dropoutProb = 0.0015;
    double spuriousProb = 0.0;   ///< 单帧 <20 mm 假回波概率（实测会返回 1/10/11 mm）
    double spuriousMm = 5.0;     ///< 假回波读数
    double biasMm[4] = {0.0, 0.0, 0.0, 0.0};  ///< 逻辑顺序 前/左/后/右 的固定偏置
};

/// 从真实墙段生成四路 ToF 快照；逻辑顺序与 LaneConfig::sensors 一致。
core::RangeSnapshot sampleToF(const std::vector<maze_world::Wall>& walls,
                              const Pose& pose, const lane::LaneConfig& cfg,
                              const ToFConfig& tof, std::uint32_t nowMs,
                              std::mt19937* rng, double rawOut[4]) {
    core::RangeSnapshot snap{};
    snap.timestampMs = nowMs;

    const double fx = std::cos(pose.h);
    const double fy = std::sin(pose.h);
    const double lx = std::sin(pose.h);
    const double ly = -std::cos(pose.h);

    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int i = 0; i < lane::kSensorCount; ++i) {
        const lane::SensorConfig& s = cfg.sensors[i];
        const double ox = pose.x + fx * s.xM + lx * s.yM;
        const double oy = pose.y + fy * s.xM + ly * s.yM;
        // body 向量 (cosθ,sinθ) 映射到世界角 h-θ（body +Y = h-90°，世界 y 向下）。
        const double a = pose.h - s.axisRad + gauss(*rng) * tof.beamJitterRad;
        const double dx = std::cos(a);
        const double dy = std::sin(a);

        const double dist = maze_world::raycast(walls, ox, oy, dx, dy);
        rawOut[i] = std::isfinite(dist) ? dist : -1.0;

        core::RangeSample& out = snap.samples[s.rangeSource];
        out.timestampMs = nowMs;
        out.confidence = 10;

        const bool dropped = uni(*rng) < tof.dropoutProb;
        if (dropped || !std::isfinite(dist) || dist > 2.0) {
            out.status = core::RangeStatus::kNoTarget;
            out.distanceMm = 9999;
            continue;
        }

        double d = dist + gauss(*rng) * tof.sigmaM + tof.biasMm[i] / 1000.0;
        if (tof.spuriousProb > 0.0 && uni(*rng) < tof.spuriousProb) {
            d = tof.spuriousMm / 1000.0;
        }
        d = std::max(0.001, d);
        if (d < 0.02) {
            out.status = core::RangeStatus::kTooClose;
        } else {
            out.status = core::RangeStatus::kOk;
        }
        out.distanceMm = static_cast<std::int32_t>(std::lround(d * 1000.0));
    }
    return snap;
}

enum class Axis { kNorthSouth, kEastWest };

struct RunSpec {
    std::string name;
    Axis axis = Axis::kNorthSouth;
    double startOffsetM = 0.0;   ///< 相对走廊中心，向右为正
    double startY = 2.10;        ///< 南北走廊北端起点
    double startX = 1.55;        ///< 东西走廊西端起点
    double rowCenterY = 2.475;   ///< 东西走廊中心线（真实地图 row 5: y=2.25..2.70）
    double headingErrorDeg = 0.0;  ///< 车头相对走廊的固定偏航（航向环保持该值）
    double driveMps = 0.40;      ///< 基础前进速度
    double driveS = 4.0;         ///< 巡航时长
    double strafeMps = 0.0;      ///< 之后横向平移速度（+向左，-向右）
    double strafeS = 0.0;        ///< 横向平移时长
    double stopS = 0.0;          ///< 之后停车时长（基础命令 0）
    double settleS = 1.5;        ///< 指标统计窗口起点
    double sigmaMm = 1.0;
    double biasLeftMm = 0.0;
    double biasRightMm = 0.0;
    double biasFrontMm = 0.0;
    double biasRearMm = 0.0;
    double dropoutProb = 0.0015;
    double spuriousProb = 0.0;
    double slip = 0.0;
    unsigned seed = 1;

    // 参数覆盖：用于评估“如果改这些参数会怎样”，-1 表示用默认值。
    double kp = -1.0;
    double kd = -1.0;
    double standstillMinMps = -1.0;
    double standstillDeadbandMm = -1.0;
    double maxCorrMps = -1.0;
    double maxLateralAccel = -1.0;
};

struct Metrics {
    double endOffsetMm = 0.0;
    double rmsMm = 0.0;
    double p2pMm = 0.0;        ///< 统计窗口内峰峰值
    double maxAbsMm = 0.0;
    int signFlips = 0;
    int tracking = 0;
    int degraded = 0;
    int tooClose = 0;
    int noCorridor = 0;
    int noPair = 0;
    int insufficient = 0;
    int saturated = 0;
    double endMeasuredErrMm = 0.0;
    int group = -1;  ///< 最后一帧选对：0=A(前后), 1=B(左右)
    double minCornerClearanceMm = 1e9;  ///< 车架角点到墙面的最小间隙（负=侵入）
    double maxAbsCorrMps = 0.0;
    bool configValid = false;
    bool leftTooCloseEver = false;
};

Metrics simulate(const RunSpec& spec, bool printTrace) {
    lane::LaneConfig cfg = lane::makeDefaultLaneConfig();
    cfg.layoutVerified = true;  // 本仿真使用的就是文档里的顶点几何
    if (spec.kp >= 0.0) cfg.pd.kp = spec.kp;
    if (spec.kd >= 0.0) cfg.pd.kd = spec.kd;
    if (spec.standstillMinMps >= 0.0) {
        cfg.state.standstillMinCorrectionMps = spec.standstillMinMps;
    }
    if (spec.standstillDeadbandMm >= 0.0) {
        cfg.state.standstillDeadbandMm = spec.standstillDeadbandMm;
    }
    if (spec.maxCorrMps >= 0.0) {
        cfg.state.maxCorrectionMps = spec.maxCorrMps;
        cfg.pd.outputLimit = spec.maxCorrMps;
        cfg.pd.pLimit = spec.maxCorrMps;
        cfg.pd.dLimit = spec.maxCorrMps;
    }
    if (spec.maxLateralAccel >= 0.0) {
        cfg.state.maxLateralAccelMps2 = spec.maxLateralAccel;
    }

    const std::vector<maze_world::Wall> walls = maze_world::fixedWalls();

    lane::LaneKeeper keeper;
    keeper.init(cfg);
    keeper.enable = true;

    const bool eastWest = (spec.axis == Axis::kEastWest);

    Plant plant;
    Pose pose;
    pose.x = eastWest ? spec.startX
                      : kCorridorCenterX + spec.startOffsetM;
    pose.y = eastWest ? spec.rowCenterY + spec.startOffsetM : spec.startY;
    pose.h = kCorridorNorth + spec.headingErrorDeg * kPi / 180.0;

    ToFConfig tof;
    tof.sigmaM = spec.sigmaMm / 1000.0;
    tof.dropoutProb = spec.dropoutProb;
    tof.spuriousProb = spec.spuriousProb;
    tof.biasMm[lane::kS0] = spec.biasFrontMm;
    tof.biasMm[lane::kS1] = spec.biasLeftMm;
    tof.biasMm[lane::kS2] = spec.biasRearMm;
    tof.biasMm[lane::kS3] = spec.biasRightMm;

    std::mt19937 rng(spec.seed);

    Metrics m;
    std::vector<double> offsets;
    std::vector<double> tArr;
    std::vector<double> corrArr;
    std::vector<double> errArr;
    std::vector<int> modeArr;

    const double totalS = spec.driveS + spec.strafeS + spec.stopS;
    const int steps = static_cast<int>(std::lround(totalS / kDt));
    std::uint32_t nowMs = 20;
    double lastSignOffset = 0.0;
    bool hasLastSign = false;

    for (int k = 0; k < steps; ++k) {
        const double t = k * kDt;

        double rawDist[4] = {0.0, 0.0, 0.0, 0.0};
        const core::RangeSnapshot snap =
            sampleToF(walls, pose, cfg, tof, nowMs, &rng, rawDist);

        core::MotionCommand base{0.0, 0.0, 0.0};
        if (t < spec.driveS - 1e-9) {
            if (eastWest) {
                base.vy = -spec.driveMps;  // 车头朝北时，负 vy = 向东平移
            } else {
                base.vx = spec.driveMps;
            }
        } else if (t < spec.driveS + spec.strafeS - 1e-9) {
            base.vy = spec.strafeMps;
        }
        keeper.update(base, snap, core::HeadingState{}, kDt);
        const lane::LaneKeeperState st = keeper.getState();

        const double offset = eastWest ? (pose.y - spec.rowCenterY)
                                       : (pose.x - kCorridorCenterX);
        offsets.push_back(offset);
        tArr.push_back(t);
        corrArr.push_back(st.lateralCorrectionMps);
        errArr.push_back(st.centerErrorMm);
        modeArr.push_back(static_cast<int>(st.mode));

        if (st.mode == lane::LaneMode::kTracking) ++m.tracking;
        if (st.mode == lane::LaneMode::kDegraded) ++m.degraded;
        if (st.mode == lane::LaneMode::kTooClose) ++m.tooClose;
        if (st.mode == lane::LaneMode::kNoCorridor) ++m.noCorridor;
        if (st.mode == lane::LaneMode::kNoPair) ++m.noPair;
        if (st.mode == lane::LaneMode::kInsufficientData) ++m.insufficient;
        if (std::fabs(st.lateralCorrectionMps) >=
            cfg.state.maxCorrectionMps - 1e-9) {
            ++m.saturated;
        }
        m.maxAbsCorrMps =
            std::max(m.maxAbsCorrMps, std::fabs(st.lateralCorrectionMps));
        if (st.leftTooClose) m.leftTooCloseEver = true;

        // 车架在走廊法向上的投影：正方形车架（顶点半径 R）转 θ 后为
        // R(|cosθ|+|sinθ|)。用 6 mm 半墙厚折算到墙面，负值代表理论侵入。
        const double theta = pose.h - kCorridorNorth;
        const double extent =
            lane::kDefaultSensorRadiusM *
            (std::fabs(std::cos(theta)) + std::fabs(std::sin(theta)));
        const double clearance =
            (0.225 - 0.006) - (std::fabs(offset) + extent);
        m.minCornerClearanceMm =
            std::min(m.minCornerClearanceMm, clearance * 1000.0);

        m.configValid = st.configValid;
        m.endMeasuredErrMm = st.centerErrorMm;
        m.group = st.group;

        if (hasLastSign && t >= spec.settleS) {
            if ((offset > 0.0) != (lastSignOffset > 0.0)) ++m.signFlips;
        }
        if (t >= spec.settleS) lastSignOffset = offset;
        hasLastSign = true;

        if (printTrace) {
            std::printf(
                "TRACE,%s,t=%.2f,x=%.4f,off=%.1f,Lraw=%.1f,Rraw=%.1f,"
                "L=%.1f,R=%.1f,err=%.1f,corr=%.3f,mode=%d,lc=%d,rc=%d\n",
                spec.name.c_str(), t, pose.x, offset * 1000.0,
                rawDist[lane::kS1] * 1000.0, rawDist[lane::kS3] * 1000.0,
                st.leftMm, st.rightMm, st.centerErrorMm,
                st.lateralCorrectionMps, static_cast<int>(st.mode),
                st.leftTooClose ? 1 : 0, st.rightTooClose ? 1 : 0);
        }

        plant.step(st.command, &pose, kDt, &rng);
        nowMs += 20;
    }

    // 稳态指标：只看 settleS 之后；车位姿有惯性，不能只看最后一帧。
    const int startIdx =
        std::min(steps - 1, static_cast<int>(std::lround(spec.settleS / kDt)));
    double sum = 0.0;
    double minV = 1e9;
    double maxV = -1e9;
    double maxAbs = 0.0;
    for (int k = startIdx; k < steps; ++k) {
        sum += offsets[k] * offsets[k];
        minV = std::min(minV, offsets[k]);
        maxV = std::max(maxV, offsets[k]);
        maxAbs = std::max(maxAbs, std::fabs(offsets[k]));
    }
    const int n = std::max(1, steps - startIdx);
    m.rmsMm = std::sqrt(sum / n) * 1000.0;
    m.p2pMm = (maxV - minV) * 1000.0;
    m.maxAbsMm = maxAbs * 1000.0;
    m.endOffsetMm = offsets.empty() ? 0.0 : offsets.back() * 1000.0;
    return m;
}

void printRow(const RunSpec& spec, const Metrics& m) {
    std::printf(
        "%-26s end=%7.1f rms=%6.1f p2p=%6.1f max=%6.1f flips=%2d "
        "mode[T/D/C/N/I/P]=%4d/%3d/%3d/%3d/%3d/%3d corrMax=%.3f sat=%3d "
        "errMeas=%6.1f clrMin=%6.1f grp=%d\n",
        spec.name.c_str(), m.endOffsetMm, m.rmsMm, m.p2pMm, m.maxAbsMm,
        m.signFlips, m.tracking, m.degraded, m.tooClose, m.noCorridor,
        m.insufficient, m.noPair, m.maxAbsCorrMps, m.saturated,
        m.endMeasuredErrMm, m.minCornerClearanceMm, m.group);
}

/// 用两条平行墙扫描“配置允许的走廊宽度”和“实际能进入 TRACKING 的宽度”。
void probeCorridorGates() {
    std::printf("\n== 走廊宽度门控扫描（固定在中心、车头朝北、无噪声）==\n");
    const lane::LaneConfig base = lane::makeDefaultLaneConfig();
    bool prevTracking = false;
    double firstTracking = -1.0;
    double lastTracking = -1.0;
    for (int i = 0; i <= 80; ++i) {
        const double widthM = 0.35 + 0.005 * i;
        std::vector<maze_world::Wall> walls;
        walls.push_back(maze_world::Wall{kCorridorCenterX - widthM / 2.0, 0.0,
                                         kCorridorCenterX - widthM / 2.0, 3.0});
        walls.push_back(maze_world::Wall{kCorridorCenterX + widthM / 2.0, 0.0,
                                         kCorridorCenterX + widthM / 2.0, 3.0});

        lane::LaneConfig cfg = base;
        cfg.layoutVerified = true;
        lane::LaneKeeper keeper;
        keeper.init(cfg);
        keeper.enable = true;

        Pose pose;
        pose.x = kCorridorCenterX;
        pose.y = 1.50;
        ToFConfig tof;
        tof.sigmaM = 0.0;
        tof.beamJitterRad = 0.0;
        tof.dropoutProb = 0.0;
        std::mt19937 rng(1);
        lane::LaneKeeperState st{};
        std::uint32_t nowMs = 20;
        for (int k = 0; k < 12; ++k) {
            double raw[4] = {0.0, 0.0, 0.0, 0.0};
            const core::RangeSnapshot snap =
                sampleToF(walls, pose, cfg, tof, nowMs, &rng, raw);
            keeper.update(core::MotionCommand{0.30, 0.0, 0.0}, snap,
                          core::HeadingState{}, 0.02);
            st = keeper.getState();
            nowMs += 20;
        }
        const bool tracking = st.mode == lane::LaneMode::kTracking;
        if (i == 0 || tracking != prevTracking || (i % 2 == 0)) {
            std::printf(
                "W=%.3f  L=%6.1f R=%6.1f width=%.3f valid=%d "
                "mode=%d tooClose=%d\n",
                widthM, st.leftMm, st.rightMm, st.corridorWidthM,
                st.corridorWidthValid ? 1 : 0, static_cast<int>(st.mode),
                (st.leftTooClose || st.rightTooClose) ? 1 : 0);
        }
        if (tracking) {
            if (firstTracking < 0.0) firstTracking = widthM;
            lastTracking = widthM;
        }
        prevTracking = tracking;
    }
    if (firstTracking > 0.0) {
        std::printf("实际 TRACKING 宽度范围（本次扫描）：%.3f–%.3f m\n",
                    firstTracking, lastTracking);
    }
}

/// 走廊先宽（>0.66）再收窄到 MID 区：FAR 状态能否重新变成 MID？
void probeMidHysteresis() {
    std::printf("\n== 100–150 mm（MID）迟滞探针 ==\n");
    const double widths[] = {0.70, 0.70, 0.60, 0.60, 0.55, 0.60};
    const char* labels[] = {"宽 0.70 (FAR)", "宽 0.70 (FAR)",
                            "收窄 0.60 (MID?)", "收窄 0.60 (MID?)",
                            "再窄 0.55 (NEAR)", "回 0.60 (MID)"};
    lane::LaneConfig cfg = lane::makeDefaultLaneConfig();
    cfg.layoutVerified = true;
    lane::LaneKeeper keeper;
    keeper.init(cfg);
    keeper.enable = true;
    std::uint32_t nowMs = 20;
    std::mt19937 rng(1);
    for (int phase = 0; phase < 6; ++phase) {
        std::vector<maze_world::Wall> walls;
        const double w = widths[phase];
        walls.push_back(maze_world::Wall{kCorridorCenterX - w / 2.0, 0.0,
                                         kCorridorCenterX - w / 2.0, 3.0});
        walls.push_back(maze_world::Wall{kCorridorCenterX + w / 2.0, 0.0,
                                         kCorridorCenterX + w / 2.0, 3.0});
        Pose pose;
        pose.x = kCorridorCenterX;
        pose.y = 1.50;
        ToFConfig tof;
        tof.sigmaM = 0.0;
        tof.beamJitterRad = 0.0;
        tof.dropoutProb = 0.0;
        lane::LaneKeeperState st{};
        for (int k = 0; k < 20; ++k) {
            double raw[4] = {0.0, 0.0, 0.0, 0.0};
            const core::RangeSnapshot snap =
                sampleToF(walls, pose, cfg, tof, nowMs, &rng, raw);
            keeper.update(core::MotionCommand{0.30, 0.0, 0.0}, snap,
                          core::HeadingState{}, 0.02);
            st = keeper.getState();
            nowMs += 20;
        }
        const char* modeName = "?";
        switch (st.mode) {
            case lane::LaneMode::kTracking: modeName = "TRACKING"; break;
            case lane::LaneMode::kDegraded: modeName = "DEGRADED"; break;
            case lane::LaneMode::kTooClose: modeName = "TOO_CLOSE"; break;
            case lane::LaneMode::kNoCorridor: modeName = "NO_CORRIDOR"; break;
            case lane::LaneMode::kInsufficientData: modeName = "INSUFF"; break;
            case lane::LaneMode::kNoPair: modeName = "NO_PAIR"; break;
            default: modeName = "DISABLED"; break;
        }
        std::printf(
            "%-20s W=%.2f L=%6.1f R=%6.1f class=%d/%d mode=%s\n",
            labels[phase], w, st.leftMm, st.rightMm,
            static_cast<int>(st.channels[lane::kS1].state),
            static_cast<int>(st.channels[lane::kS3].state), modeName);
    }
}

RunSpec makeSpec(const std::string& name) {
    RunSpec s;
    s.name = name;
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    bool trace = false;
    bool gates = false;
    double sigmaOverride = -1.0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--trace") == 0) {
            trace = true;
        } else if (std::strcmp(argv[i], "--gates") == 0) {
            gates = true;
        } else if (std::strncmp(argv[i], "--sigma=", 8) == 0) {
            sigmaOverride = std::atof(argv[i] + 8);
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("usage: %s [--trace] [--gates] [--sigma=mm]\n", argv[0]);
            return 0;
        }
    }

    std::printf(
        "== 车道保持闭环仿真：真实迷宫走廊 + v2.1 LaneKeeper ==\n"
        "走廊 x=%.3f, y=2.10->0.70, 45 cm, ToF 50 Hz, 控制 50 Hz, dt=20 ms\n",
        kCorridorCenterX);

    if (gates) {
        probeCorridorGates();
        probeMidHysteresis();
        return 0;
    }

    std::vector<RunSpec> specs;

    // 巡航：走廊长 1.35 m，按速度缩短时长，避免撞到走廊尽头污染指标。
    const double kRunM = 1.30;
    auto driveSpec = [&](const std::string& name, double offset,
                         double headingDeg, double speed) {
        RunSpec s = makeSpec(name);
        s.startOffsetM = offset;
        s.headingErrorDeg = headingDeg;
        s.driveMps = speed;
        s.driveS = kRunM / speed;
        return s;
    };

    specs.push_back(driveSpec("drive_offset+20", 0.020, 0.0, 0.40));
    specs.push_back(driveSpec("drive_offset-20", -0.020, 0.0, 0.40));
    {
        auto s = driveSpec("drive_offset+20_kp0.003", 0.020, 0.0, 0.40);
        s.kp = 0.003;
        specs.push_back(s);
    }

    // 航向固定偏 2/4/6/8°：航向环只会保持这个偏航，车道层只能靠横向平移抵消。
    for (double deg : {2.0, 4.0, 6.0, 8.0}) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "drive_heading+%.0fdeg",
                      static_cast<double>(deg));
        specs.push_back(driveSpec(buf, 0.0, deg, 0.40));
    }

    // 速度越大，同样的航向偏角产生越大的横向漂移。
    for (double v : {0.20, 0.60}) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "drive_v%.1f_heading+4deg",
                      static_cast<double>(v));
        specs.push_back(driveSpec(buf, 0.0, 4.0, v));
    }

    // 停车居中：基础命令 0，只有 LaneKeeper 的停车补偿在动。
    auto standSpec = [&](const std::string& name, double offset,
                         double sigmaMm) {
        RunSpec s = makeSpec(name);
        s.startOffsetM = offset;
        s.driveS = 0.0;
        s.stopS = 4.0;
        s.settleS = 1.0;
        s.sigmaMm = sigmaMm;
        return s;
    };

    specs.push_back(standSpec("standstill+20", 0.020, 1.0));
    specs.push_back(standSpec("standstill-20", -0.020, 1.0));
    specs.push_back(standSpec("standstill+10", 0.010, 1.0));
    specs.push_back(standSpec("standstill+2(deadband)", 0.002, 1.0));
    specs.push_back(standSpec("standstill+20_noise0.3", 0.020, 0.3));
    specs.push_back(standSpec("standstill+20_noise3", 0.020, 3.0));

    // 20 mm 过近阈值附近：没有迟滞，噪声会把模式来回切。
    specs.push_back(standSpec("standstill+22(threshold)", 0.022, 2.0));
    specs.push_back(standSpec("standstill+28(tooClose)", 0.028, 1.0));

    // 左右传感器固定偏置（规格 ±15 mm 的一半）。
    {
        auto s = standSpec("standstill_bias+10/-10", 0.0, 0.5);
        s.biasLeftMm = +10.0;
        s.biasRightMm = -10.0;
        specs.push_back(s);
    }
    {
        auto s = driveSpec("drive_bias+10/-10", 0.0, 0.0, 0.40);
        s.sigmaMm = 0.5;
        s.biasLeftMm = +10.0;
        s.biasRightMm = -10.0;
        specs.push_back(s);
    }
    {
        auto s = driveSpec("drive_bias+10/+10(common)", 0.0, 0.0, 0.40);
        s.sigmaMm = 0.5;
        s.biasLeftMm = +10.0;
        s.biasRightMm = +10.0;
        specs.push_back(s);
    }

    // 参数覆盖：用 D 阻尼停车时的极限环。
    {
        auto s = standSpec("ss+20_kd0.0005", 0.020, 1.0);
        s.kd = 0.0005;
        specs.push_back(s);
    }
    {
        auto s = standSpec("ss+20_kd0.001", 0.020, 1.0);
        s.kd = 0.001;
        specs.push_back(s);
    }
    {
        auto s = standSpec("ss+20_kd0.002", 0.020, 1.0);
        s.kd = 0.002;
        specs.push_back(s);
    }

    // 死区/最小速度扫描：找“不来回抖”的最小组合。
    for (double db : {5.0, 6.0, 7.0, 8.0, 9.0, 10.0}) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "ss+20_db%.0f", static_cast<double>(db));
        auto s = standSpec(buf, 0.020, 1.0);
        s.standstillDeadbandMm = db;
        specs.push_back(s);
    }
    for (double minV : {0.050, 0.060, 0.070}) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "ss+20_min%.3f_db6",
                      static_cast<double>(minV));
        auto s = standSpec(buf, 0.020, 1.0);
        s.standstillMinMps = minV;
        s.standstillDeadbandMm = 6.0;
        specs.push_back(s);
    }
    {
        auto s = driveSpec("drive_offset+20_kp0.003_noise3", 0.020, 0.0, 0.40);
        s.kp = 0.003;
        s.sigmaMm = 3.0;
        specs.push_back(s);
    }

    // 参数覆盖：评估“死区/最小速度/增益”改动的效果。
    {
        auto s = standSpec("ss+20_min0.04_db6", 0.020, 1.0);
        s.standstillMinMps = 0.04;
        s.standstillDeadbandMm = 6.0;
        specs.push_back(s);
    }
    {
        auto s = standSpec("ss+20_min0(PDonly)", 0.020, 1.0);
        s.standstillMinMps = 0.0;
        specs.push_back(s);
    }

    // 横向平移后停车：最后一次平移方向是“左右”，选对变成前后墙，
    // 停车时是否还能对左右墙居中？
    {
        auto s = makeSpec("strafe_right_then_stop");
        s.driveS = 0.0;
        s.strafeMps = -0.15;  // 向右
        s.strafeS = 0.15;
        s.stopS = 3.0;
        s.settleS = 1.0;
        specs.push_back(s);
    }
    {
        auto s = makeSpec("fwd_strafe_right_stop");
        s.driveMps = 0.30;
        s.driveS = 0.60;
        s.strafeMps = -0.15;
        s.strafeS = 0.15;
        s.stopS = 3.0;
        s.settleS = 1.5;
        specs.push_back(s);
    }

    // 传感器偏置 ±15 mm（规格上限）在 3 mm / 10 mm 死区下的结果。
    {
        auto s = standSpec("ss_bias15_db3", 0.0, 0.5);
        s.biasLeftMm = +15.0;
        s.biasRightMm = -15.0;
        specs.push_back(s);
    }
    {
        auto s = standSpec("ss_bias15_db10", 0.0, 0.5);
        s.biasLeftMm = +15.0;
        s.biasRightMm = -15.0;
        s.standstillDeadbandMm = 10.0;
        specs.push_back(s);
    }

    // 提高 Kp 是否能抵消航向偏角造成的稳态偏移。
    {
        auto s = driveSpec("h+4deg_kp0.002", 0.0, 4.0, 0.40);
        s.kp = 0.002;
        specs.push_back(s);
    }
    {
        auto s = driveSpec("h+6deg_kp0.003", 0.0, 6.0, 0.40);
        s.kp = 0.003;
        specs.push_back(s);
    }

    // 单帧 <20 mm 假回波：RangeChannel 接受它并直接触发 TOO_CLOSE 排斥。
    {
        auto s = driveSpec("drive_spurious5mm", 0.0, 0.0, 0.40);
        s.spuriousProb = 0.005;
        specs.push_back(s);
    }
    {
        auto s = standSpec("standstill_spurious5mm", 0.020, 1.0);
        s.spuriousProb = 0.005;
        specs.push_back(s);
    }

    // 丢包：20% 的 ToF 帧直接丢失，检查 DEGRADED/丢墙保持。
    {
        auto s = driveSpec("drive_dropout20pct", 0.0, 0.0, 0.40);
        s.dropoutProb = 0.20;
        specs.push_back(s);
    }

    // 带横向偏置驶出左墙：旧墙测量最多会保留 180 ms。
    {
        auto s = makeSpec("drive_exit_offset-20");
        s.startOffsetM = -0.020;
        s.driveS = 1.20;
        s.startY = 1.55;
        specs.push_back(s);
    }

    // ===== A 组：平移转弯（车头保持朝北，靠 vy 沿东西走廊平移）=====
    // 真实地图 row 5（y=2.25..2.70，x=1.35..2.25，0.9 m 长）：
    // 前后墙 S0/S2 各 45 mm，左右墙 S1/S3 是开口 -> 正好验证 A 组。
    auto ewSpec = [&](const std::string& name, double offset,
                      double headingDeg, double speed) {
        RunSpec s = makeSpec(name);
        s.axis = Axis::kEastWest;
        s.startOffsetM = offset;
        s.headingErrorDeg = headingDeg;
        s.driveMps = speed;
        s.driveS = 0.50 / speed;  // 沿 x 平移 0.50 m，仍在 0.9 m 走廊内
        return s;
    };

    specs.push_back(ewSpec("ew_drive+20", 0.020, 0.0, 0.30));
    {
        // 更慢、更长：看 A 组是否真正收敛，而不是只在 1.67 s 内衰减。
        auto s = ewSpec("ew_drive+20_slow", 0.020, 0.0, 0.20);
        s.startX = 1.45;
        s.driveS = 3.00;
        s.settleS = 1.50;
        specs.push_back(s);
    }
    specs.push_back(ewSpec("ew_drive-20", -0.020, 0.0, 0.30));
    specs.push_back(ewSpec("ew_drive_heading+2deg", 0.0, 2.0, 0.30));
    specs.push_back(ewSpec("ew_drive_heading+4deg", 0.0, 4.0, 0.30));
    {
        auto s = makeSpec("ew_drive_heading+4deg_kp0.002");
        s.axis = Axis::kEastWest;
        s.headingErrorDeg = 4.0;
        s.driveMps = 0.30;
        s.driveS = 0.50 / 0.30;
        s.kp = 0.002;
        specs.push_back(s);
    }
    {
        // 平移转弯后停车：最后方向是东西（A 组），与走廊轴一致。
        auto s = ewSpec("ew_drive_then_stop", 0.020, 0.0, 0.30);
        s.driveS = 1.20;
        s.stopS = 3.0;
        s.settleS = 1.8;
        specs.push_back(s);
    }
    {
        auto s = ewSpec("ew_drive_then_stop_db10", 0.020, 0.0, 0.30);
        s.driveS = 1.20;
        s.stopS = 3.0;
        s.settleS = 1.8;
        s.standstillDeadbandMm = 10.0;
        specs.push_back(s);
    }
    {
        auto s = ewSpec("ew_drive_then_stop_min.04_db6", 0.020, 0.0, 0.30);
        s.driveS = 1.20;
        s.stopS = 3.0;
        s.settleS = 1.8;
        s.standstillMinMps = 0.040;
        s.standstillDeadbandMm = 6.0;
        specs.push_back(s);
    }
    {
        // 0.4 m/s + 4°：稳态 v·tanθ/Kp ≈ 28 mm，超过 45 cm 走廊的 ±25 mm。
        auto s = ewSpec("ew_drive_heading+4deg_v0.4", 0.0, 4.0, 0.40);
        s.driveS = 1.50;
        specs.push_back(s);
    }
    {
        auto s = makeSpec("ew_standstill+20");
        s.axis = Axis::kEastWest;
        s.startOffsetM = 0.020;
        s.driveS = 0.0;
        s.stopS = 4.0;
        s.settleS = 1.0;
        specs.push_back(s);
    }
    {
        auto s = makeSpec("ew_standstill+20_db10");
        s.axis = Axis::kEastWest;
        s.startOffsetM = 0.020;
        s.driveS = 0.0;
        s.stopS = 4.0;
        s.settleS = 1.0;
        s.standstillDeadbandMm = 10.0;
        specs.push_back(s);
    }
    {
        auto s = ewSpec("ew_drive_biasF+10/R-10", 0.0, 0.0, 0.30);
        s.sigmaMm = 0.5;
        s.biasFrontMm = +10.0;
        s.biasRearMm = -10.0;
        specs.push_back(s);
    }
    {
        auto s = makeSpec("ew_stop_biasF+10/R-10");
        s.axis = Axis::kEastWest;
        s.driveS = 0.0;
        s.stopS = 4.0;
        s.settleS = 1.0;
        s.sigmaMm = 0.5;
        s.biasFrontMm = +10.0;
        s.biasRearMm = -10.0;
        specs.push_back(s);
    }
    {
        // 平移出混凝土走廊东端：A 组墙消失，应安全退出。
        auto s = makeSpec("ew_exit_corridor");
        s.axis = Axis::kEastWest;
        s.startOffsetM = -0.015;
        s.driveMps = 0.30;
        s.driveS = 0.80 / 0.30;
        specs.push_back(s);
    }

    // 驶出左墙端（走廊变成单边开阔）：观察纠偏是否安全退出。
    {
        auto s = makeSpec("drive_exit_left_wall");
        s.startOffsetM = -0.015;
        s.driveS = 5.0;
        s.startY = 2.10;
        specs.push_back(s);
    }

    for (RunSpec& s : specs) {
        if (sigmaOverride >= 0.0) s.sigmaMm = sigmaOverride;
        const Metrics m = simulate(s, trace);
        printRow(s, m);
        if (!m.configValid) {
            std::printf("  !! config invalid\n");
        }
    }

    std::printf(
        "\n说明：end/rms/p2p/max 单位 mm；正 offset = 车在走廊中心右侧；\n"
        "mode 顺序 T=TRACKING D=DEGRADED C=TOO_CLOSE N=NO_CORRIDOR "
        "I=INSUFFICIENT P=NO_PAIR。\n");
    return 0;
}
