/**
 * @file tests/native/test_lane_keeper.cpp
 * @brief LaneKeeper v2.1 主机测试：
 *        近壁语义、通道选对、横向 PD、过近排斥、丢墙缓释、
 *        方向记忆、原始测量不被修改、无速度倍率。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "lane/lane_config.hpp"
#include "modules/lane_keeper.hpp"

namespace {

using core::MotionCommand;
using core::RangeSnapshot;
using core::RangeStatus;
using lane::LaneConfig;
using lane::LaneKeeper;
using lane::LaneKeeperState;
using lane::LaneMode;
using lane::LaneReason;
using lane::NearClass;

constexpr double kR = 0.1803120669;

bool approx(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

struct SideSpec {
    RangeStatus status = RangeStatus::kNoTarget;
    std::int32_t mm = 9999;
};

SideSpec ok(std::int32_t mm) { return SideSpec{RangeStatus::kOk, mm}; }
SideSpec none() { return SideSpec{RangeStatus::kNoTarget, 9999}; }
SideSpec err() { return SideSpec{RangeStatus::kReadError, -1}; }
SideSpec tooClose(std::int32_t mm) {
    return SideSpec{RangeStatus::kTooClose, mm};
}

RangeSnapshot snapshot(SideSpec front, SideSpec rear, SideSpec left,
                       SideSpec right, std::uint32_t nowMs) {
    RangeSnapshot s{};
    s.timestampMs = nowMs;
    const SideSpec sides[4] = {front, rear, left, right};
    for (int i = 0; i < core::kRangeSideCount; ++i) {
        s.samples[i].status = sides[i].status;
        s.samples[i].distanceMm = sides[i].mm;
        s.samples[i].confidence = 10;
        s.samples[i].timestampMs = nowMs;
    }
    return s;
}

LaneConfig testConfig() {
    LaneConfig c = lane::makeDefaultLaneConfig();
    c.layoutVerified = true;
    c.channel.lowPassAlpha = 1.0;   // 控制测试可预测
    c.channel.jumpConfirm = 2;
    c.channel.maxAgeMs = 180;
    c.channel.freshWindowMs = 42;
    c.pd.kp = 0.01;
    c.pd.ki = 0.0;
    c.pd.kd = 0.0;
    c.pd.pLimit = 1.0;
    c.pd.iLimit = 1.0;
    c.pd.dLimit = 1.0;
    c.pd.outputLimit = 1.0;
    c.state.maxCorrectionMps = 0.50;
    c.state.maxLateralAccelMps2 = 1000.0;  // 基本用例关闭斜坡
    return c;
}

class Sim {
public:
    explicit Sim(const LaneConfig& c) : config_(c) {
        keeper_.init(config_);
        keeper_.enable = true;
    }

    LaneKeeperState step(const MotionCommand& base, const RangeSnapshot& snap,
                         const core::HeadingState& heading = {}) {
        keeper_.update(base, snap, heading, 0.02);
        return keeper_.getState();
    }

    LaneKeeperState run(const MotionCommand& base, SideSpec front, SideSpec rear,
                        SideSpec left, SideSpec right, int n,
                        const core::HeadingState& heading = {}) {
        LaneKeeperState st{};
        for (int i = 0; i < n; ++i) {
            nowMs_ += 20;
            st = step(base, snapshot(front, rear, left, right, nowMs_),
                      heading);
        }
        return st;
    }

    std::uint32_t now() const { return nowMs_; }

private:
    LaneConfig config_{};
    LaneKeeper keeper_;
    std::uint32_t nowMs_ = 0;
};

}  // namespace

int main() {
    // =========================================================================
    // 1. 接口契约：只叠加横向 delta，不改巡航速度/omega
    // =========================================================================
    {
        Sim s(testConfig());
        const MotionCommand base{0.40, 0.0, 0.15};
        const auto st = s.run(base, none(), none(), ok(45), ok(45), 6);
        assert(st.mode == LaneMode::kTracking);
        assert(st.group == lane::kGroupB);
        assert(st.pairValid && st.corridor);
        assert(approx(st.command.vx, base.vx, 1e-9));
        assert(approx(st.command.vy, base.vy, 1e-9));
        assert(approx(st.command.omega, base.omega, 1e-9));
        assert(approx(st.lateralCorrectionMps, 0.0, 1e-9));
        assert(approx(st.centerErrorMm, 0.0, 1e-9));
        assert(approx(st.corridorWidthM, 2.0 * kR + 0.090, 1e-6));
    }

    // =========================================================================
    // 2. 左右符号：左近 -> 向右(-Y)，右近 -> 向左(+Y)
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(30), ok(60), 6);
        assert(st.mode == LaneMode::kTracking);
        assert(st.centerErrorMm > 0.0);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy < 0.0);
        assert(approx(st.command.vx, 0.30, 1e-9));
    }
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(60), ok(30), 6);
        assert(st.centerErrorMm < 0.0);
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);
    }

    // =========================================================================
    // 3. 100–150 mm 首次观测：可进入 TRACKING，而不是永久 UNKNOWN/FAULT
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(120), ok(120), 5);
        assert(st.mode == LaneMode::kTracking);
        assert(st.channels[lane::kS1].state == NearClass::kMidReturn);
        assert(st.channels[lane::kS3].state == NearClass::kMidReturn);
        assert(approx(st.corridorWidthM, 2.0 * kR + 0.240, 1e-6));
        assert(st.corridorWidthValid);
    }

    // =========================================================================
    // 4. 横移 / 倒车：选对与修正分量
    // =========================================================================
    {
        // 左移：A 组；运动左=后(S2)=30，右=前(S0)=60 -> 向右(+X)
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.30, 0.0}, ok(60), ok(30),
                              none(), none(), 6);
        assert(st.group == lane::kGroupA);
        assert(st.leftSensor == lane::kS2);
        assert(st.rightSensor == lane::kS0);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vx > 0.0);
        assert(approx(st.command.vy, 0.30, 1e-9));
    }
    {
        // 右移：A 组；运动左=前(S0)=30，右=后(S2)=60 -> 向右(-X)
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, -0.30, 0.0}, ok(30), ok(60),
                              none(), none(), 6);
        assert(st.group == lane::kGroupA);
        assert(st.leftSensor == lane::kS0);
        assert(st.rightSensor == lane::kS2);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vx < 0.0);
    }
    {
        // 倒车：B 组，左右对调；运动左=右(S3)=30，右=左(S1)=60 -> 向右(+Y)
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{-0.30, 0.0, 0.0}, none(), none(),
                              ok(60), ok(30), 6);
        assert(st.group == lane::kGroupB);
        assert(st.leftSensor == lane::kS3);
        assert(st.rightSensor == lane::kS1);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy > -0.30);
    }

    // =========================================================================
    // 5. 45° 斜向：不硬选组，不纠偏，基础命令原样
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.20, 0.20, 0.0}, none(), none(),
                              ok(45), ok(45), 6);
        assert(st.mode == LaneMode::kNoPair);
        assert(!st.pairValid);
        assert(st.group == -1);
        assert(approx(st.command.vx, 0.20, 1e-9));
        assert(approx(st.command.vy, 0.20, 1e-9));
        assert(approx(st.lateralCorrectionMps, 0.0, 1e-9));
    }

    // =========================================================================
    // 6. 远距回波不是“无墙”：侧墙 45，前/后 1.845/0.98 m 仍可跟踪
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.40, 0.0, 0.0}, ok(1845),
                              ok(980), ok(45), ok(46), 6);
        assert(st.mode == LaneMode::kTracking);
        assert(st.channels[lane::kS0].state == NearClass::kFarReturn);
        assert(st.channels[lane::kS2].state == NearClass::kFarReturn);
        assert(st.channels[lane::kS0].valid);
        assert(std::fabs(st.channels[lane::kS0].rawMm - 1845) <= 1);
        assert(std::fabs(st.channels[lane::kS2].rawMm - 980) <= 1);
    }

    // =========================================================================
    // 7. 单侧远回波不构成双墙，但原始量测仍然有效
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.40, 0.0, 0.0}, none(), none(),
                              ok(45), ok(850), 8);
        assert(st.channels[lane::kS3].state == NearClass::kFarReturn);
        assert(st.channels[lane::kS3].valid);
        assert(std::fabs(st.channels[lane::kS3].rawMm - 850) <= 1);
        assert(st.mode == LaneMode::kNoCorridor);
        assert(!st.corridor);
        assert(approx(st.command.vy, 0.0, 1e-9));
    }

    // =========================================================================
    // 8. 通道宽度不合理 -> NO_CORRIDOR
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(200), ok(200), 6);
        assert(!st.corridorWidthValid);
        assert(st.mode == LaneMode::kNoCorridor);
        assert(approx(st.command.vy, 0.0, 1e-9));
    }

    // =========================================================================
    // 9. 方向记忆：停车后仍按上次通道轴居中
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        ok(45), 6);
        assert(st.mode == LaneMode::kTracking);
        st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(), ok(45),
                   ok(55), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy < 0.0);
        assert(approx(st.command.vx, 0.0, 1e-9));
    }
    {
        // 从未移动：没有方向时按物理左右墙自动选 B 并停车居中
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              ok(45), ok(55), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(st.group == lane::kGroupB);
        assert(st.fallbackPair);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy < 0.0);
        assert(approx(st.command.vx, 0.0, 1e-9));
    }
    {
        // 前后墙：无方向时自动选 A，修正落在 vx
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, ok(45), ok(55),
                              none(), none(), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(st.group == lane::kGroupA);
        assert(st.fallbackPair);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vx < 0.0);  // 前墙更近 -> 向后
    }
    {
        // 两对都近：默认偏向左右对 B
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, ok(45), ok(45),
                              ok(45), ok(45), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(st.group == lane::kGroupB);
        assert(st.fallbackPair);
    }
    {
        // 对角移动中没有稳定通道方向：不 fallback、不纠偏
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.20, 0.20, 0.0}, none(), none(),
                              ok(45), ok(45), 8);
        assert(st.mode == LaneMode::kNoPair);
        assert(!st.pairValid);
    }

    // =========================================================================
    // 9b. 停车静摩擦补偿：误差超过死区时给最小横向速度
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              ok(45), ok(55), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(approx(st.lateralTargetMps, 0.08, 1e-9));
    }
    {
        // 误差 0.5 mm 在 3 mm 死区内：不施加最小速度
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              ok(45), ok(46), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(st.lateralTargetMps < 0.03);
    }
    {
        // 有平移速度时仍用 PD，不叠加停车最小速度
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(45), ok(55), 8);
        assert(st.mode == LaneMode::kTracking);
        assert(approx(st.lateralTargetMps, 0.05, 1e-9));
    }

    // =========================================================================
    // 10. 过近排斥：单侧远离、双侧按差居中，且不改纵向速度
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(58), tooClose(11), 20);
        assert(st.mode == LaneMode::kTooClose);
        assert(st.leftTooClose == false);
        assert(st.rightTooClose);
        assert(st.lateralTargetMps < 0.0);  // 向左
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);        // 前进时负修正 -> +Y
        assert(approx(st.command.vx, 0.30, 1e-9));
    }
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              tooClose(8), tooClose(11), 20);
        assert(st.mode == LaneMode::kTooClose);
        assert(st.leftTooClose && st.rightTooClose);
        assert(st.centerErrorMm > 0.0);     // 左更近 -> 向右
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.lateralCorrectionMps <= 0.10 + 1e-9);
    }
    {
        // 一侧读取失败：不进入 TOO_CLOSE，也不虚构双墙
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              tooClose(5), err(), 20);
        assert(st.mode == LaneMode::kInsufficientData);
        assert(!st.corridor);
        assert(approx(st.command.vy, 0.0, 1e-9));
    }

    // =========================================================================
    // 11. 丢墙：短时用旧纠偏衰减，不瞬跳；长时间释放
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                        ok(60), 8);
        assert(st.mode == LaneMode::kTracking);
        const double before = st.lateralCorrectionMps;
        assert(before > 0.0);

        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                   none(), 1);
        assert(st.mode == LaneMode::kTracking ||
               st.mode == LaneMode::kDegraded);
        assert(std::fabs(st.lateralCorrectionMps - before) < 0.0141);

        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                   none(), 10);
        assert(st.mode == LaneMode::kDegraded);
        assert(st.degraded);

        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                   none(), 12);
        assert(st.mode == LaneMode::kInsufficientData);
        assert(approx(st.lateralCorrectionMps, 0.0, 1e-9));
    }

    // =========================================================================
    // 12. 过近 -> 正常：模式恢复，PD 重新起步
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                        tooClose(8), tooClose(11), 20);
        assert(st.mode == LaneMode::kTooClose);
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                   ok(60), 3);
        assert(st.mode == LaneMode::kTracking);
        assert(st.lateralCorrectionMps > 0.0);
    }

    // =========================================================================
    // 13. 横向变化率限幅
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        cfg.state.maxLateralAccelMps2 = 0.30;
        Sim s(cfg);
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(30), ok(60), 3);
        assert(st.mode == LaneMode::kTracking);
        assert(st.lateralTargetMps > 0.01);
        assert(approx(st.lateralCorrectionMps, 0.006, 1e-9));
    }

    // =========================================================================
    // 14. 关闭 / 非法配置 / 非有限输入 / reset
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        LaneKeeper keeper;
        keeper.init(cfg);
        keeper.enable = false;
        const MotionCommand base{0.2, 0.1, 0.3};
        keeper.update(base, snapshot(none(), none(), ok(45), none(), 20),
                      core::HeadingState{}, 0.02);
        const auto& st = keeper.getState();
        assert(st.mode == LaneMode::kDisabled);
        assert(approx(st.command.vx, base.vx, 1e-9));
        assert(approx(st.command.vy, base.vy, 1e-9));
        assert(approx(st.command.omega, base.omega, 1e-9));
        assert(st.valid);
    }
    {
        LaneConfig cfg = testConfig();
        cfg.sensors[lane::kS0].rangeSource = cfg.sensors[lane::kS1].rangeSource;
        LaneKeeper keeper;
        keeper.init(cfg);
        keeper.enable = true;
        const MotionCommand base{0.2, 0.0, 0.0};
        keeper.update(base, snapshot(none(), none(), ok(45), ok(45), 20),
                      core::HeadingState{}, 0.02);
        const auto& st = keeper.getState();
        assert(!st.configValid);
        assert(st.mode == LaneMode::kDisabled);
        assert(approx(st.command.vx, base.vx, 1e-9));
    }
    {
        Sim s(testConfig());
        const double nan = std::nan("");
        const auto st = s.run(MotionCommand{nan, 0.0, 0.0}, none(), none(),
                              none(), none(), 1);
        assert(st.mode == LaneMode::kDisabled);
        assert(st.reason == LaneReason::kInputInvalid);
        assert(approx(st.command.vx, 0.0, 1e-9));
        assert(!st.valid);
    }
    {
        LaneKeeper keeper;
        keeper.init(testConfig());
        keeper.enable = true;
        for (int i = 0; i < 4; ++i) {
            keeper.update(MotionCommand{0.30, 0.0, 0.0},
                          snapshot(none(), none(), ok(45), ok(45),
                                   static_cast<std::uint32_t>(20 + i * 20)),
                          core::HeadingState{}, 0.02);
        }
        assert(keeper.getState().mode == LaneMode::kTracking);
        keeper.reset();
        const auto& rst = keeper.getState();
        assert(rst.mode == LaneMode::kDisabled);
        assert(approx(rst.command.vx, 0.0, 1e-9));
        assert(rst.channels[lane::kS1].state == NearClass::kUnknown);
    }

    // =========================================================================
    // 15. 航向无效不影响 LaneKeeper（安全门控属于 SafetySupervisor）
    // =========================================================================
    {
        Sim s(testConfig());
        core::HeadingState heading{};
        heading.valid = false;
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(30), ok(60), 6, heading);
        assert(st.mode == LaneMode::kTracking);
        assert(!st.headingValid);
        assert(st.lateralCorrectionMps > 0.0);
    }

    // =========================================================================
    // 16. 原始快照不可被 LaneKeeper 修改
    // =========================================================================
    {
        Sim s(testConfig());
        const auto snap = snapshot(ok(1650), ok(980), ok(45), ok(46), 20);
        const std::int32_t frontBefore = snap.samples[0].distanceMm;
        const std::int32_t rearBefore = snap.samples[1].distanceMm;
        s.step(MotionCommand{0.40, 0.0, 0.0}, snap);
        assert(snap.samples[0].distanceMm == frontBefore);
        assert(snap.samples[1].distanceMm == rearBefore);
        assert(snap.samples[0].status == RangeStatus::kOk);
    }

    std::printf("lane keeper tests: ALL PASS\n");
    return 0;
}
