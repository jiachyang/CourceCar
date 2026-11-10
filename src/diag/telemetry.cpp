/**
 * @file diag/telemetry.cpp
 * @brief 遥测帧格式化实现（$P / $D / $S）。
 */
#include "diag/telemetry.hpp"

#include <cstddef>
#include <cstdio>

#include "pros/device.hpp"
#include "pros/rtos.hpp"

namespace diag {
namespace {

// =============================================================================
// $P 调参帧字段表（唯一真源）
// =============================================================================
// 顺序即输出顺序；与 test/pid_tune.py 的 FIELDS（去掉开头的 ms）一一对应。
// 增删字段必须同步 test/pid_tune.py 与 docs/debug_log/07_调参工具.md。
// 单位：速度 mm/s、角速度 mrad/s、角度 mrad、轮速 milli-rpm、PID 参数 ×1000。
#define DIAG_TUNE_FIELDS(X)                                                     \
    X(tvx, static_cast<int>(c.applied.vx * 1000.0))                             \
    X(tvy, static_cast<int>(c.applied.vy * 1000.0))                             \
    X(tw, static_cast<int>(c.applied.omega * 1000.0))                           \
    X(wraw, static_cast<int>(c.omegaBeforeSlewRadPerSec * 1000.0))              \
    X(rot, static_cast<int>(is.rotationRad * 1000.0))                           \
    X(gz, static_cast<int>(is.angularVelocityRadPerSec * 1000.0))               \
    X(ht, static_cast<int>(c.headingTargetRad * 1000.0))                        \
    X(herr, static_cast<int>(c.heading.err * 1000.0))                           \
    X(hP, static_cast<int>(c.heading.pTerm * 1000.0))                           \
    X(hI, static_cast<int>(c.heading.iTerm * 1000.0))                           \
    X(hD, static_cast<int>(c.heading.dTerm * 1000.0))                           \
    X(hcorr, static_cast<int>(c.heading.corr * 1000.0))                         \
    X(hext, static_cast<int>(c.heading.extOmega * 1000.0))                      \
    X(hrot, c.heading.active ? 1 : 0)                                           \
    X(aFL, static_cast<int>(c.wheelActualRpm[0] * 1000.0))                      \
    X(aFR, static_cast<int>(c.wheelActualRpm[1] * 1000.0))                      \
    X(aRL, static_cast<int>(c.wheelActualRpm[2] * 1000.0))                      \
    X(aRR, static_cast<int>(c.wheelActualRpm[3] * 1000.0))                      \
    X(cFL, static_cast<int>(c.wheelCommandRpm[0] * 1000.0))                     \
    X(cFR, static_cast<int>(c.wheelCommandRpm[1] * 1000.0))                     \
    X(cRL, static_cast<int>(c.wheelCommandRpm[2] * 1000.0))                     \
    X(cRR, static_cast<int>(c.wheelCommandRpm[3] * 1000.0))                     \
    X(bias, static_cast<int>(is.gyroBiasDps * 1000.0))                          \
    X(drift, static_cast<int>(is.gyroDriftDps * 1000.0))                        \
    X(st, static_cast<int>(is.statusRaw))                                       \
    X(bm, is.biasMeasured ? 1 : 0)                                              \
    X(imu, c.headingValid ? 1 : 0)                                              \
    X(hdg, c.headingActive ? 1 : 0)                                             \
    X(rc, r.valid ? 1 : 0)                                                      \
    X(kp, static_cast<int>(c.heading.kp * 1000.0))                              \
    X(ki, static_cast<int>(c.heading.ki * 1000.0))                              \
    X(kd, static_cast<int>(c.heading.kd * 1000.0))                              \
    X(intLim, static_cast<int>(c.heading.integralLimit * 1000.0))               \
    X(pLim, static_cast<int>(c.heading.pLimit * 1000.0))                        \
    X(iLim, static_cast<int>(c.heading.iLimit * 1000.0))                        \
    X(dLim, static_cast<int>(c.heading.dLimit * 1000.0))                        \
    X(outLim, static_cast<int>(c.heading.maxCorr * 1000.0))                     \
    X(ff, static_cast<int>(c.heading.ffTerm * 1000.0))

/// $P 的字段数（不含帧头的 ms）。Python 侧 FIELDS 长度 = 该值 + 1。
constexpr int kTuneFieldCount = 38;

/// 端口扫描范围：1~21 号 Smart Port。
constexpr int kPortScanFirst = 1;
constexpr int kPortScanLast = 21;

/// 串口帧缓冲：整行拼好后一次写出，避免每个字段一次 printf 的系统调用。
class FrameBuffer {
public:
    FrameBuffer(const char *head, std::uint32_t nowMs)
        : len_(std::snprintf(data_, sizeof(data_), "%s,%u", head,
                             static_cast<unsigned>(nowMs))) {}

    void add(int value) {
        if (len_ < 0 || len_ > kCapacity - kReserve) {
            return;
        }
        const int n = std::snprintf(data_ + len_,
                                    static_cast<std::size_t>(kCapacity - len_),
                                    ",%d", value);
        if (n > 0) {
            len_ += n;
            if (len_ > kCapacity - 2) {
                len_ = kCapacity - 2;
            }
        }
    }

    void flush() {
        if (len_ < 0) {
            return;
        }
        data_[len_++] = '\n';
        data_[len_] = '\0';
        std::fputs(data_, stdout);
    }

private:
    static constexpr int kCapacity = 512;
    static constexpr int kReserve = 16;  ///< 留给最后一个字段与换行的余量
    char data_[kCapacity];
    int len_;
};

/// 限频判据：到周期返回 true 并更新 last。
bool due(std::uint32_t now, std::uint32_t &last, std::uint32_t periodMs) {
    if (now - last < periodMs) {
        return false;
    }
    last = now;
    return true;
}

// -----------------------------------------------------------------------------
// 各帧
// -----------------------------------------------------------------------------

void printTuneFrame(const chassis::ChassisState &c, const imu::ImuState &is,
                    const radio::RadioState &r, std::uint32_t nowMs) {
#define X(name, expr) expr,
    const int fields[] = {DIAG_TUNE_FIELDS(X)};
#undef X

    static_assert(sizeof(fields) / sizeof(fields[0]) == kTuneFieldCount,
                  "$P 字段数变化：必须同步 test/pid_tune.py 的 FIELDS 与文档");

    FrameBuffer frame("$P", nowMs);
    for (const int value : fields) {
        frame.add(value);
    }
    frame.flush();
}

constexpr int kLaneFieldCount = 30;

void addLaneFields(FrameBuffer &frame, const lane::LaneKeeperState *l) {
    // $D 追加字段（顺序固定，主机端 test/distance_monitor.py 按同一顺序解析）：
    //   1 laneMode/state
    //   2 laneGroup
    //   3 wallMask        墙存在位（逻辑 S0..S3）
    //   4 usableMask      可控制位（Present && filter fresh）
    //   5 leftMm          运动左传感器滤波距离（PID 原始量）
    //   6 rightMm
    //   7 errMm           墙距 PID 误差，向右为正
    //   8 pMms  9 iMms  10 dMms  11 corrMms
    //  12 outVxMms 13 outVyMms 14 outOmegaMradS（LaneKeeper 输出；最终 omega 见 $P.tw）
    //  15..18 filterS0..S3 滤波后距离（逻辑 S0..S3 = 前/左/后/右）
    //  19 filterValidMask
    //  20 targetWallMm
    //  21 corridorWidthMm
    //  22 laneReason
    //  23 speedScaleMilli
    //  24 forwardScaleMilli
    //  25 motionObstacleMm
    //  26 laneFlags
    //  27 sideLeftRawMm   物理左侧原始距离（过近时仍有效，便于 UI）
    //  28 sideRightRawMm  物理右侧原始距离
    //  29 pidTargetMmS    墙距/过近控制的目标横向速度（限幅后、限速前）
    //  30 tooCloseMask    bit0=左过近，bit1=右过近
    if (l == nullptr) {
        for (int i = 0; i < kLaneFieldCount; ++i) {
            frame.add(-1);
        }
        return;
    }

    int wallMask = 0;
    int usableMask = 0;
    int filterValidMask = 0;
    for (int i = 0; i < lane::kSensorCount; ++i) {
        if (l->walls[i].status == lane::WallStatus::kPresent) {
            wallMask |= (1 << i);
        }
        if (l->walls[i].usable) {
            usableMask |= (1 << i);
        }
        if (l->walls[i].filterValid) {
            filterValidMask |= (1 << i);
        }
    }

    int flags = 0;
    if (l->active) flags |= 1 << 0;
    if (l->turning) flags |= 1 << 1;
    if (l->degraded) flags |= 1 << 2;
    if (l->fault) flags |= 1 << 3;
    if (l->stateChanged) flags |= 1 << 4;
    if (l->groupChanged) flags |= 1 << 5;
    if (l->headingValid) flags |= 1 << 6;
    if (l->headingStable) flags |= 1 << 7;
    if (l->corridorWidthValid) flags |= 1 << 8;
    if (l->corridorActive) flags |= 1 << 9;

    frame.add(static_cast<int>(l->state));
    frame.add(static_cast<int>(l->group));
    frame.add(wallMask);
    frame.add(usableMask);
    frame.add(l->leftObserved
                  ? static_cast<int>(l->leftDistanceMm)
                  : -1);
    frame.add(l->rightObserved
                  ? static_cast<int>(l->rightDistanceMm)
                  : -1);
    frame.add(static_cast<int>(l->lateralErrorMm));
    frame.add(static_cast<int>(l->wallPid.pTerm * 1000.0));
    frame.add(static_cast<int>(l->wallPid.iTerm * 1000.0));
    frame.add(static_cast<int>(l->wallPid.dTerm * 1000.0));
    frame.add(static_cast<int>(l->lateralCorrectionMps * 1000.0));
    frame.add(static_cast<int>(l->command.vx * 1000.0));
    frame.add(static_cast<int>(l->command.vy * 1000.0));
    frame.add(static_cast<int>(l->command.omega * 1000.0));
    for (int i = 0; i < lane::kSensorCount; ++i) {
        frame.add(l->walls[i].filterValid
                      ? static_cast<int>(l->walls[i].filteredMm)
                      : -1);
    }
    frame.add(filterValidMask);
    frame.add(static_cast<int>(l->targetWallMm));
    frame.add(l->corridorWidthValid
                  ? static_cast<int>(l->corridorWidthM * 1000.0)
                  : -1);
    frame.add(static_cast<int>(l->reason));
    frame.add(static_cast<int>(l->speedScale * 1000.0));
    frame.add(static_cast<int>(l->forwardSafetyScale * 1000.0));
    frame.add(l->motionObstacleMm >= 0.0
                  ? static_cast<int>(l->motionObstacleMm)
                  : -1);
    frame.add(flags);

    frame.add(static_cast<int>(l->sideLeftRawMm));
    frame.add(static_cast<int>(l->sideRightRawMm));
    frame.add(static_cast<int>(l->lateralTargetMps * 1000.0));
    int tooCloseMask = 0;
    if (l->leftTooClose) tooCloseMask |= 1 << 0;
    if (l->rightTooClose) tooCloseMask |= 1 << 1;
    frame.add(tooCloseMask);
}

void printDistanceFrame(const distance::DistanceArrayState &d,
                        const lane::LaneKeeperState *l,
                        std::uint32_t nowMs) {
    FrameBuffer frame("$D", nowMs);
    frame.add(d.installedMask);
    frame.add(d.validMask);
    for (int i = 0; i < distance::kSideCount; ++i) {
        const distance::DistanceReading &r = d.readings[i];
        frame.add(r.distanceMm);
        frame.add(r.confidence);
        frame.add(r.objectSize);
        frame.add(r.velocityMmS);
        frame.add(r.pluggedType);
    }
    addLaneFields(frame, l);
    frame.flush();
}

void printPortScanFrame(std::uint32_t nowMs) {
    FrameBuffer frame("$S", nowMs);
    for (int port = kPortScanFirst; port <= kPortScanLast; ++port) {
        const auto type =
            pros::Device::get_plugged_type(static_cast<std::uint8_t>(port));
        frame.add(static_cast<int>(type));
    }
    frame.flush();
}

}  // namespace

void Telemetry::init(const TelemetryConfig &config,
                     const TelemetrySources &sources) {
    config_ = config;
    sources_ = sources;
    lastTuneMs_ = 0;
    lastDistanceMs_ = 0;
    lastPortScanMs_ = 0;
}

void Telemetry::update() {
    const std::uint32_t now = pros::millis();

    if (config_.enableTune && sources_.chassis != nullptr &&
        sources_.imu != nullptr && sources_.radio != nullptr &&
        due(now, lastTuneMs_, config_.tunePeriodMs)) {
        printTuneFrame(*sources_.chassis, *sources_.imu, *sources_.radio, now);
    }

    if (config_.enableDistance && sources_.distance != nullptr &&
        due(now, lastDistanceMs_, config_.distancePeriodMs)) {
        printDistanceFrame(*sources_.distance, sources_.lane, now);
    }

    if (config_.enablePortScan &&
        due(now, lastPortScanMs_, config_.portScanPeriodMs)) {
        printPortScanFrame(now);
    }
}

}  // namespace diag
