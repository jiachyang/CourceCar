/**
 * @file tests/native/test_lane_state_machine.cpp
 * @brief 车道状态机测试：模式切换、重新识别、转弯暂停、传感器故障、
 *        积分权限与速度比例。
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "lane/lane_state_machine.hpp"

namespace {

lane::LaneStateMachine makeMachine() {
    lane::LaneConfig c = lane::makeDefaultLaneConfig();
    lane::LaneStateMachine m;
    m.init(c);
    return m;
}

lane::LaneStateInputs baseInputs(std::uint32_t now) {
    lane::LaneStateInputs in{};
    in.enabled = true;
    in.configValid = true;
    in.inputValid = true;
    in.moving = true;
    in.turning = false;
    in.headingStable = true;
    in.groupValid = true;
    in.groupChanged = false;
    in.leftStatus = lane::WallStatus::kPresent;
    in.rightStatus = lane::WallStatus::kPresent;
    in.leftUsable = true;
    in.rightUsable = true;
    in.corridorWidthValid = true;
    in.leftRawMm = 45.0;
    in.rightRawMm = 45.0;
    in.nowMs = now;
    return in;
}

lane::LaneStateOutput firstUpdate(lane::LaneStateMachine& m) {
    const auto out = m.update(baseInputs(0));
    assert(out.state == lane::LaneState::kAcquiring);
    return out;
}

}  // namespace

int main() {
    // --- 关闭 / 输入非法 / 配置非法 ---
    {
        auto m = makeMachine();
        auto in = baseInputs(0);
        in.enabled = false;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kDisabled);
        assert(out.speedScale == 1.0);  // 关闭时透传基础命令
        assert(!out.allowLateral);
    }
    {
        auto m = makeMachine();
        auto in = baseInputs(0);
        in.inputValid = false;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kDisabled);
        assert(out.speedScale == 0.0);
    }
    {
        auto m = makeMachine();
        auto in = baseInputs(0);
        in.configValid = false;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kDisabled);
        assert(out.speedScale == 1.0);
    }

    // --- 启动 -> ACQUIRING -> DUAL_WALL ---
    {
        auto m = makeMachine();
        auto out = firstUpdate(m);
        assert(out.stateChanged);
        assert(out.resetPid);
        assert(out.speedScale == 0.5);
        assert(!out.allowLateral);

        out = m.update(baseInputs(200));
        assert(out.state == lane::LaneState::kDualWall);
        assert(out.allowLateral);
        assert(out.allowIntegral);  // 两者均 45 < 100
        assert(out.speedScale == 1.0);
        assert(out.resetPid);
    }

    // --- 双墙但墙距在 100–150 滞回带：保持模式但冻结积分 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.leftRawMm = 120.0;
        in.rightRawMm = 120.0;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kDualWall);
        assert(!out.allowIntegral);
    }

    // --- 双墙中仅一侧 <100：冻结积分 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.leftRawMm = 45.0;
        in.rightRawMm = 120.0;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kDualWall);
        assert(!out.allowIntegral);
    }

    // --- 双墙 -> 左墙、右墙 -> 无墙；每次切换 resetPid ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto out = m.update(baseInputs(200));
        assert(out.state == lane::LaneState::kDualWall);

        auto in = baseInputs(220);
        in.rightStatus = lane::WallStatus::kAbsent;
        in.rightUsable = false;
        out = m.update(in);
        assert(out.state == lane::LaneState::kLeftWall);
        assert(out.allowLateral);
        assert(out.allowIntegral);  // 左 45 < 100
        assert(out.resetPid);

        in = baseInputs(240);
        in.leftStatus = lane::WallStatus::kAbsent;
        in.leftUsable = false;
        in.rightStatus = lane::WallStatus::kPresent;
        in.rightUsable = true;
        out = m.update(in);
        assert(out.state == lane::LaneState::kRightWall);
        assert(out.allowIntegral);

        in = baseInputs(260);
        in.leftStatus = lane::WallStatus::kAbsent;
        in.rightStatus = lane::WallStatus::kAbsent;
        in.leftUsable = false;
        in.rightUsable = false;
        out = m.update(in);
        assert(out.state == lane::LaneState::kNoWall);
        assert(!out.allowLateral);
        assert(!out.allowIntegral);
    }

    // --- 单墙 -> 双墙：重新获取后恢复双墙并复位 PID ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        m.update(baseInputs(200));
        auto in = baseInputs(220);
        in.rightStatus = lane::WallStatus::kAbsent;
        in.rightUsable = false;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kLeftWall);

        in = baseInputs(240);
        in.groupChanged = true;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        out = m.update(baseInputs(400));
        assert(out.state == lane::LaneState::kDualWall);
    }

    // --- 组不可用 -> ACQUIRING ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.groupValid = false;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(!out.allowLateral);
    }

    // --- 组切换 -> ACQUIRING 并复位 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        m.update(baseInputs(200));
        auto in = baseInputs(220);
        in.groupChanged = true;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(out.resetPid);
        assert(out.speedScale == 0.5);
    }

    // --- 主动转弯 -> TURNING；航向稳定后 ACQUIRING；重新识别后双墙 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        m.update(baseInputs(200));

        auto in = baseInputs(220);
        in.turning = true;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kTurning);
        assert(!out.allowLateral);
        assert(out.speedScale == 1.0);
        assert(out.resetPid);

        in = baseInputs(240);
        in.turning = false;
        in.headingStable = false;
        out = m.update(in);
        assert(out.state == lane::LaneState::kTurning);

        in = baseInputs(260);
        in.turning = false;
        in.headingStable = true;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(out.resetPid);

        out = m.update(baseInputs(460));
        assert(out.state == lane::LaneState::kDualWall);
    }

    // --- 双方 9999 持续超时 -> NO_WALL（不是 Absent，也不停止） ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.leftStatus = lane::WallStatus::kUnknown;
        in.rightStatus = lane::WallStatus::kUnknown;
        in.leftUsable = false;
        in.rightUsable = false;
        in.leftNoTarget = true;
        in.rightNoTarget = true;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);

        in.nowMs = 320;
        out = m.update(in);
        assert(out.state == lane::LaneState::kNoWall);
        assert(!out.fault);
        assert(out.speedScale == 1.0);
    }

    // --- 一侧有墙 + 一侧 9999 超时 -> 单墙 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.rightStatus = lane::WallStatus::kUnknown;
        in.rightUsable = false;
        in.rightNoTarget = true;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);

        in.nowMs = 320;
        out = m.update(in);
        assert(out.state == lane::LaneState::kLeftWall);
        assert(out.allowLateral);
    }

    // --- 单侧过近 -> TOO_CLOSE（连续远离），不判故障 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto out = m.update(baseInputs(200));
        assert(out.state == lane::LaneState::kDualWall);

        auto in = baseInputs(400);
        in.leftStatus = lane::WallStatus::kUnknown;
        in.leftUsable = false;
        in.leftTooClose = true;
        in.leftRawMm = 11.0;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);

        in.nowMs = 420;
        out = m.update(in);
        assert(out.state == lane::LaneState::kTooClose);
        assert(out.allowLateral);
        assert(!out.allowIntegral);
        assert(!out.fault);
        assert(std::abs(out.speedScale - 0.35) < 1e-12);

        // 过近解除、两侧恢复可用 -> 直接双墙，不额外停住
        auto ok = baseInputs(440);
        ok.leftRawMm = 30.0;
        out = m.update(ok);
        assert(out.state == lane::LaneState::kDualWall);
        assert(out.allowLateral);
    }

    // --- 双侧过近 -> TOO_CLOSE，仍允许横向远离 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto out = m.update(baseInputs(200));
        assert(out.state == lane::LaneState::kDualWall);

        auto in = baseInputs(400);
        in.leftStatus = lane::WallStatus::kUnknown;
        in.rightStatus = lane::WallStatus::kUnknown;
        in.leftUsable = false;
        in.rightUsable = false;
        in.leftTooClose = true;
        in.rightTooClose = true;
        in.leftRawMm = 10.0;
        in.rightRawMm = 12.0;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        in.nowMs = 420;
        out = m.update(in);
        assert(out.state == lane::LaneState::kTooClose);
        assert(out.allowLateral);
        assert(!out.fault);
    }

    // --- 传感器未知超时 -> SENSOR_FAULT；恢复后 ACQUIRING ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.leftStatus = lane::WallStatus::kUnknown;
        in.leftUsable = false;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);

        in.nowMs = 200 + 500;
        out = m.update(in);
        assert(out.state == lane::LaneState::kSensorFault);
        assert(out.fault);
        assert(out.speedScale == 0.0);
        assert(!out.allowLateral);

        // 仍在故障且问题未消失：保持
        in.nowMs = 200 + 600;
        out = m.update(in);
        assert(out.state == lane::LaneState::kSensorFault);

        // 恢复
        in.nowMs = 200 + 700;
        in.leftStatus = lane::WallStatus::kPresent;
        in.leftUsable = true;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(!out.fault);
    }

    // --- 双墙宽度不一致 -> ACQUIRING（不危险纠偏） ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.corridorWidthValid = false;
        const auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(out.reason == lane::LaneReason::kCorridorInconsistent);
        assert(!out.allowLateral);
    }

    // --- 单侧有墙但另一侧未知：先 ACQUIRING，超时故障 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.leftStatus = lane::WallStatus::kPresent;
        in.leftUsable = true;
        in.rightStatus = lane::WallStatus::kUnknown;
        in.rightUsable = false;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        in.nowMs = 200 + 500;
        out = m.update(in);
        assert(out.state == lane::LaneState::kSensorFault);
    }

    // --- 零速：ACQUIRING，不累积故障 ---
    {
        auto m = makeMachine();
        firstUpdate(m);
        auto in = baseInputs(200);
        in.moving = false;
        in.leftStatus = lane::WallStatus::kUnknown;
        in.rightStatus = lane::WallStatus::kUnknown;
        in.leftUsable = false;
        in.rightUsable = false;
        auto out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        in.nowMs = 200 + 1000;
        out = m.update(in);
        assert(out.state == lane::LaneState::kAcquiring);
        assert(!out.fault);
    }

    std::printf("lane state machine tests: ALL PASS\n");
    return 0;
}
