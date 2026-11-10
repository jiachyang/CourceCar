/**
 * @file tests/native/test_lane_keeper.cpp
 * @brief LaneKeeper 主机端模拟测试（无 PROS / 无硬件）。
 *
 * 覆盖：顶点传感器布局、前后=A/左右=B、双墙/单墙符号、组切换、状态机、
 *       滤波、积分门控、速度比例、横向平滑、前向安全、异常降级。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "core/range_types.hpp"
#include "core/types.hpp"
#include "lane/geometry.hpp"
#include "modules/lane_keeper.hpp"

namespace {

using core::MotionCommand;
using core::RangeSnapshot;
using core::RangeStatus;
using lane::LaneConfig;
using lane::LaneKeeper;
using lane::LaneKeeperState;
using lane::LaneReason;
using lane::LaneState;

constexpr double kR = 0.1803120669;  // 顶点半径

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
    c.state.corridorOnly = false;  // 旧单墙/无墙用例显式关闭走廊门控
    c.layoutVerified = true;
    c.state.acquireTimeMs = 1;
    c.state.postTurnAcquireMs = 1;
    c.state.acquireSpeedScale = 1.0;
    c.state.turningSpeedScale = 1.0;
    c.state.faultSpeedScale = 0.0;
    c.state.maxSpeedScaleRatePerSec = 1000.0;
    c.state.maxLateralAccelMps2 = 1000.0;
    c.state.motionStopMm = 0.0;
    c.state.motionSlowMm = 1.0;  // 基本用例关闭前向限速
    c.state.maxLateralCorrectionMps = 0.50;
    c.state.maxCorridorWidthM = 0.70;  // 覆盖 120/120 滞回测试的宽度
    c.wallPid.kp = 0.01;
    c.wallPid.ki = 0.0;
    c.wallPid.kd = 0.0;
    c.wallPid.pLimit = 1.0;
    c.wallPid.iLimit = 1.0;
    c.wallPid.dLimit = 1.0;
    c.wallPid.outputLimit = 1.0;
    c.wallPid.integralLimit = 100.0;
    c.filter.lowPassAlpha = 1.0;  // 让控制测试可预测
    return c;
}

LaneConfig corridorConfig() {
    LaneConfig c = testConfig();
    c.state.corridorOnly = true;
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

    LaneKeeperState runHeading(const MotionCommand& base, SideSpec front,
                               SideSpec rear, SideSpec left, SideSpec right,
                               int n, const core::HeadingState& heading) {
        return run(base, front, rear, left, right, n, heading);
    }

    uint32_t now() const { return nowMs_; }

private:
    LaneConfig config_{};
    LaneKeeper keeper_;
    std::uint32_t nowMs_ = 0;
};

}  // namespace

int main() {
    // =========================================================================
    // 1. 组选择：前进/后退=左右组 B，横移=前后组 A，斜向不选
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), none(),
                        none(), 1);
        assert(st.group == lane::kGroupB);
        assert(st.state == LaneState::kAcquiring);

        Sim s2(testConfig());
        st = s2.run(MotionCommand{-0.30, 0.0, 0.0}, none(), none(), none(),
                    none(), 1);
        assert(st.group == lane::kGroupB);

        Sim s3(testConfig());
        st = s3.run(MotionCommand{0.0, 0.30, 0.0}, none(), none(), none(),
                    none(), 1);
        assert(st.group == lane::kGroupA);

        Sim s4(testConfig());
        st = s4.run(MotionCommand{0.0, -0.30, 0.0}, none(), none(), none(),
                    none(), 1);
        assert(st.group == lane::kGroupA);

        Sim s5(testConfig());
        st = s5.run(MotionCommand{0.30, 0.30, 0.0}, none(), none(), none(),
                    none(), 1);
        assert(st.group == -1);
        assert(st.state == LaneState::kAcquiring);
    }

    // =========================================================================
    // 2. 双墙居中：左右 B 组，居中无误差；偏左向右；偏右向左
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        ok(45), 3);
        assert(st.state == LaneState::kDualWall);
        assert(st.group == lane::kGroupB);
        assert(st.leftObserved && st.rightObserved);
        assert(approx(st.leftDistanceMm, 45.0));
        assert(approx(st.rightDistanceMm, 45.0));
        assert(approx(st.corridorWidthM, 2.0 * kR + 0.090, 1e-6));
        assert(st.corridorWidthValid);
        assert(approx(st.lateralErrorMm, 0.0, 1e-9));
        assert(approx(st.command.vx, 0.30, 1e-9));
        assert(approx(st.command.vy, 0.0, 1e-9));
    }
    {
        // 左 30 / 右 60：车体偏左（右墙更远）-> 向右（车体 -Y）修正
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                        ok(60), 3);
        assert(st.state == LaneState::kDualWall);
        assert(st.lateralErrorMm > 0.0);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy < 0.0);
        assert(st.command.vx > 0.0);
    }
    {
        // 左 60 / 右 30：车体偏右 -> 向左（+Y）修正
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(60),
                        ok(30), 3);
        assert(st.lateralErrorMm < 0.0);
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);
    }

    // =========================================================================
    // 3. 单墙：左/右目标 45 mm 原始距离，符号正确
    // =========================================================================
    {
        // 左 30（太近），右 200（确认 Absent）-> 向右
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                        ok(200), 3);
        assert(st.state == LaneState::kLeftWall);
        assert(st.leftObserved && !st.rightObserved);
        assert(approx(st.leftDistanceMm, 30.0));
        assert(st.lateralErrorMm > 0.0);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy < 0.0);
    }
    {
        // 右 30（太近），左 200（Absent）-> 向左
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(200),
                        ok(30), 3);
        assert(st.state == LaneState::kRightWall);
        assert(!st.leftObserved && st.rightObserved);
        assert(st.lateralErrorMm < 0.0);
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);
    }

    // =========================================================================
    // 4. 横移/倒车时左右映射与纠偏分量
    // =========================================================================
    {
        // 左移：A 组；左=后(S2) 30，右=前(S0) 200 -> 右侧修正 -> +X
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.0, 0.30, 0.0}, ok(200), ok(30), none(),
                        none(), 3);
        assert(st.group == lane::kGroupA);
        assert(st.state == LaneState::kLeftWall);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vx > 0.0);
        assert(approx(st.command.vy, 0.30));
    }
    {
        // 右移：A 组；左=前(S0) 30，右=后(S2) 200 -> 右侧修正 -> -X
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.0, -0.30, 0.0}, ok(30), ok(200), none(),
                        none(), 3);
        assert(st.group == lane::kGroupA);
        assert(st.state == LaneState::kLeftWall);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vx < 0.0);
        assert(approx(st.command.vy, -0.30));
    }
    {
        // 倒车：B 组；左=右(S3) 30，右=左(S1) 200 -> 右侧修正 -> +Y
        Sim s(testConfig());
        auto st = s.run(MotionCommand{-0.30, 0.0, 0.0}, none(), none(), ok(200),
                        ok(30), 3);
        assert(st.group == lane::kGroupB);
        assert(st.state == LaneState::kLeftWall);
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.command.vy > 0.0);
    }

    // =========================================================================
    // 5. 无墙：确认 Absent -> NO_WALL，原速透传、不虚构
    // =========================================================================
    {
        Sim s(testConfig());
        const auto st = s.run(MotionCommand{0.25, -0.10, 0.0}, none(), none(),
                              ok(200), ok(200), 3);
        assert(st.state == LaneState::kNoWall);
        assert(!st.active);
        assert(approx(st.command.vx, 0.25));
        assert(approx(st.command.vy, -0.10));
    }

    // =========================================================================
    // 6. 9999/超量程：不确认 Absent，但持续无目标 -> NO_WALL/单墙
    //    读取失败/过近/未安装：真正的传感器异常 -> SENSOR_FAULT
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), none(),
                        none(), 3);
        assert(st.state == LaneState::kAcquiring);
        assert(!st.fault);

        // 持续 9999 超过 noWallTimeoutMs：NO_WALL，不虚构墙、不停止
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), none(),
                   none(), 10);
        assert(st.state == LaneState::kNoWall);
        assert(!st.fault);
        assert(st.walls[lane::kS1].status == lane::WallStatus::kUnknown);
        assert(st.walls[lane::kS3].status == lane::WallStatus::kUnknown);
        assert(approx(st.command.vx, 0.30));
        assert(approx(st.command.vy, 0.0));
    }
    {
        // 一侧有墙、另一侧持续 9999：超时后单墙跟随
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        none(), 3);
        assert(st.state == LaneState::kAcquiring);

        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   none(), 10);
        assert(st.state == LaneState::kLeftWall);
        assert(st.walls[lane::kS3].status == lane::WallStatus::kUnknown);
        assert(st.leftObserved);
    }
    {
        // 右路过近：不判故障/不卡住，TOO_CLOSE 主动向左远离。
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                        ok(58), tooClose(11), 6);
        assert(st.state == LaneState::kTooClose);
        assert(!st.fault);
        assert(st.active);
        assert(st.rightTooClose);
        assert(st.leftTooClose == false);
        assert(approx(st.sideLeftRawMm, 58.0));
        assert(approx(st.sideRightRawMm, 11.0));
        assert(st.speedScale > 0.0);
        assert(st.lateralTargetMps < 0.0);  // 向左
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);        // 前进时向右法向 -Y，负纠偏 -> +Y
    }
    {
        // 双侧过近：按更近的一侧远离，仍输出非零横向速度。
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                        tooClose(8), tooClose(11), 6);
        assert(st.state == LaneState::kTooClose);
        assert(!st.fault);
        assert(st.leftTooClose && st.rightTooClose);
        assert(st.lateralTargetMps > 0.0);  // 左更近 -> 向右
        assert(st.lateralCorrectionMps > 0.0);
        assert(st.speedScale > 0.0);
    }
    {
        // 过近侧 + 另一侧读取失败：仍先远离，不因另一路错误卡住。
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                        tooClose(5), err(), 6);
        assert(st.state == LaneState::kTooClose);
        assert(!st.fault);
        assert(st.lateralTargetMps > 0.0);
        assert(st.speedScale > 0.0);
    }
    {
        // 只有读取失败（无过近）：才是真正的 SENSOR_FAULT。
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                        ok(45), err(), 30);
        assert(st.state == LaneState::kSensorFault);
        assert(st.fault);
        assert(st.walls[lane::kS1].status != lane::WallStatus::kAbsent);
        assert(st.walls[lane::kS3].status != lane::WallStatus::kAbsent);
    }

    // =========================================================================
    // 6b. 短墙出现/消失：无墙 -> 左墙 -> 无墙
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(200),
                        ok(200), 3);
        assert(st.state == LaneState::kNoWall);

        // 左侧突然出现近墙（45），右侧仍为已确认无墙
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(200), 3);
        assert(st.state == LaneState::kLeftWall);
        assert(st.leftObserved);

        // 左墙结束（有效远墙 200，3 次确认）-> NO_WALL
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(200),
                   ok(200), 3);
        assert(st.state == LaneState::kNoWall);
    }

    // =========================================================================
    // 7. 积分门控：<100 允许；滞回带/单侧 >=100 冻结；模式切换清零
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        cfg.wallPid.kp = 0.0;
        cfg.wallPid.ki = 1.0;
        cfg.wallPid.outputLimit = 1.0;
        Sim s(cfg);
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                        ok(60), 6);
        assert(st.state == LaneState::kDualWall);
        assert(st.wallPid.integralAllowed);
        assert(st.wallPid.integral > 0.0);
        const double integralBefore = st.wallPid.integral;

        // 120/120 在 100–150 滞回带：状态仍双墙，但冻结积分
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(120),
                   ok(120), 2);
        assert(st.state == LaneState::kDualWall);
        assert(!st.wallPid.integralAllowed);
        assert(approx(st.wallPid.integral, integralBefore));

        // 一侧 120、一侧 30：仍冻结
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(120),
                   ok(30), 2);
        assert(st.state == LaneState::kDualWall);
        assert(!st.wallPid.integralAllowed);

        // 右墙确认 Absent -> 左墙模式，状态切换应清零积分
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(200), 3);
        assert(st.state == LaneState::kLeftWall);
        assert(approx(st.wallPid.integral, 0.0));
    }

    // =========================================================================
    // 8. 状态机：启动/组切换进入 ACQUIRING，速度受限
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        cfg.state.acquireSpeedScale = 0.5;
        Sim s(cfg);
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        ok(45), 1);
        assert(st.state == LaneState::kAcquiring);
        assert(approx(st.stateSpeedScale, 0.5));
        assert(approx(st.command.vx, 0.15));
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(45), 2);
        assert(st.state == LaneState::kDualWall);
        assert(approx(st.speedScale, 1.0));
        assert(approx(st.command.vx, 0.30));
    }

    // =========================================================================
    // 9. 主动转弯：暂停墙距 PID；转弯后 ACQUIRING 再恢复
    // =========================================================================
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.20}, none(), none(), ok(45),
                        ok(45), 1);
        assert(st.state == LaneState::kTurning);
        assert(!st.active);
        assert(approx(st.command.vx, 0.30));
        assert(approx(st.command.omega, 0.20));

        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(45), 1);
        assert(st.state == LaneState::kAcquiring);
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(45), 2);
        assert(st.state == LaneState::kDualWall);
    }

    // =========================================================================
    // 9b. 实测陀螺角速度过大也视为转弯（即使基础 omega=0）
    // =========================================================================
    {
        Sim s(testConfig());
        core::HeadingState heading{};
        heading.valid = true;
        heading.angularVelocityRadPerSec = 0.30;
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        ok(45), 1, heading);
        assert(st.state == LaneState::kTurning);
        assert(!st.active);

        heading.angularVelocityRadPerSec = 0.0;
        st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                   ok(45), 1, heading);
        assert(st.state != LaneState::kTurning);
    }

    // =========================================================================
    // 10. 横向纠偏变化率限幅
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        cfg.state.maxLateralAccelMps2 = 0.3;
        cfg.wallPid.kp = 0.01;
        Sim s(cfg);
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(30),
                        ok(60), 3);
        assert(st.state == LaneState::kDualWall);
        // kp*15mm = 0.15 m/s；每周期最大 0.3*0.02=0.006
        assert(st.lateralTargetMps > 0.01);
        assert(approx(st.lateralCorrectionMps, 0.006, 1e-9));
    }

    // =========================================================================
    // 11. 前向安全：轴对齐时按最近障碍限速；斜向为盲区
    // =========================================================================
    {
        LaneConfig cfg = testConfig();
        cfg.state.motionStopMm = 100.0;
        cfg.state.motionSlowMm = 300.0;
        Sim s(cfg);
        auto st = s.runHeading(MotionCommand{0.30, 0.0, 0.0}, ok(50), none(),
                               ok(45), ok(45), 3, core::HeadingState{});
        assert(st.state == LaneState::kDualWall);
        assert(approx(st.motionObstacleMm, 50.0));
        assert(approx(st.forwardSafetyScale, 0.0));
        assert(approx(st.command.vx, 0.0));

        st = s.runHeading(MotionCommand{0.30, 0.0, 0.0}, ok(200), none(),
                          ok(45), ok(45), 1, core::HeadingState{});
        assert(approx(st.motionObstacleMm, 200.0));
        assert(st.forwardSafetyScale > 0.0);
        assert(st.forwardSafetyScale < 1.0);
    }
    {
        // 斜向：没有足够对齐的传感器 -> 不虚构前向限速
        LaneConfig cfg = testConfig();
        cfg.state.motionStopMm = 100.0;
        cfg.state.motionSlowMm = 300.0;
        Sim s(cfg);
        const auto st = s.run(MotionCommand{0.20, 0.20, 0.0}, ok(50), none(),
                              none(), none(), 3);
        assert(approx(st.forwardSafetyScale, 1.0));
        assert(approx(st.motionObstacleMm, -1.0));
    }

    // =========================================================================
    // 11b. 双墙走廊门控：两侧都 <150 mm 才启用；停车也保持模式
    // =========================================================================
    {
        // 两侧都有近墙、基础速度为 0：保持双墙，并低速自动向右居中
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              ok(45), ok(55), 6);
        assert(st.corridorActive);
        assert(st.state == LaneState::kDualWall);
        assert(st.group == lane::kGroupB);
        assert(st.active);
        assert(approx(st.command.vx, 0.0));
        assert(st.command.vy < 0.0);  // 左墙更远 -> 向右（车体 -Y）
        assert(st.lateralCorrectionMps > 0.0);
        assert(std::fabs(st.lateralCorrectionMps) <= 0.08 + 1e-9);
        assert(st.lateralTargetMps > 0.0);
    }
    {
        // 双侧过近、基础速度为 0：仍自动远离最近墙，不是只显示目标
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              tooClose(7), tooClose(18), 6);
        assert(st.corridorActive);
        assert(st.state == LaneState::kTooClose);
        assert(st.active);
        assert(!st.fault);
        assert(st.lateralTargetMps > 0.0);  // 左更近 -> 向右
        assert(st.lateralCorrectionMps >= 0.10 - 1e-9);
        assert(st.command.vy < 0.0);
        assert(std::fabs(st.lateralCorrectionMps) <= 0.15 + 1e-9);
    }
    {
        // 双侧对称：没有自动横移
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              ok(45), ok(45), 6);
        assert(st.state == LaneState::kDualWall);
        assert(approx(st.lateralTargetMps, 0.0));
        assert(approx(st.lateralCorrectionMps, 0.0));
        assert(approx(st.command.vx, 0.0));
        assert(approx(st.command.vy, 0.0));
    }
    {
        // 双侧都过近但对称：居中，不来回抖
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.0, 0.0, 0.0}, none(), none(),
                              tooClose(10), tooClose(10), 6);
        assert(st.state == LaneState::kTooClose);
        assert(st.active);
        assert(approx(st.lateralTargetMps, 0.0));
        assert(approx(st.lateralCorrectionMps, 0.0));
        assert(approx(st.command.vx, 0.0));
        assert(approx(st.command.vy, 0.0));
    }
    {
        // 只有一侧 <150 mm：走廊门控不通过，不纠偏/不慢速
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(45), ok(600), 6);
        assert(!st.corridorActive);
        assert(st.state == LaneState::kAcquiring);
        assert(st.reason == LaneReason::kCorridorInactive);
        assert(!st.active);
        assert(st.group == -1);
        assert(approx(st.command.vx, 0.30));
        assert(approx(st.command.vy, 0.0));
    }
    {
        // 两侧都 <150 mm 且右侧过近：TOO_CLOSE 主动向左，不停车
        Sim s(corridorConfig());
        const auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(),
                              ok(58), tooClose(11), 6);
        assert(st.corridorActive);
        assert(st.state == LaneState::kTooClose);
        assert(st.active);
        assert(st.lateralTargetMps < 0.0);
        assert(st.lateralCorrectionMps < 0.0);
        assert(st.command.vy > 0.0);
        assert(st.speedScale > 0.0);
        assert(!st.fault);
    }

    // =========================================================================
    // 12. 关闭 / 非法配置 / 非有限输入 / reset
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
        assert(st.state == LaneState::kDisabled);
        assert(approx(st.command.vx, base.vx));
        assert(approx(st.command.vy, base.vy));
        assert(approx(st.command.omega, base.omega));
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
        assert(st.state == LaneState::kDisabled);
        assert(approx(st.command.vx, base.vx));
    }
    {
        Sim s(testConfig());
        const double nan = std::nan("");
        const auto st = s.run(MotionCommand{nan, 0.0, 0.0}, none(), none(),
                              none(), none(), 1);
        assert(st.state == LaneState::kDisabled);
        assert(approx(st.command.vx, 0.0));
        assert(!st.valid);
    }
    {
        Sim s(testConfig());
        auto st = s.run(MotionCommand{0.30, 0.0, 0.0}, none(), none(), ok(45),
                        ok(45), 3);
        assert(st.state == LaneState::kDualWall);
        LaneKeeper keeper;
        keeper.init(testConfig());
        keeper.enable = true;
        keeper.update(MotionCommand{0.30, 0.0, 0.0},
                      snapshot(none(), none(), ok(45), ok(45), 20),
                      core::HeadingState{}, 0.02);
        keeper.update(MotionCommand{0.30, 0.0, 0.0},
                      snapshot(none(), none(), ok(45), ok(45), 40),
                      core::HeadingState{}, 0.02);
        keeper.update(MotionCommand{0.30, 0.0, 0.0},
                      snapshot(none(), none(), ok(45), ok(45), 60),
                      core::HeadingState{}, 0.02);
        keeper.reset();
        const auto& rst = keeper.getState();
        assert(rst.state == LaneState::kDisabled);
        assert(approx(rst.command.vx, 0.0));
        assert(rst.walls[lane::kS1].status == lane::WallStatus::kUnknown);
    }

    std::printf("lane keeper tests: ALL PASS\n");
    return 0;
}
