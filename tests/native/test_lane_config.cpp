/**
 * @file tests/native/test_lane_config.cpp
 * @brief LaneConfig 默认值与静态校验测试（顶点传感器 / 前后=A / 左右=B）。
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "lane/lane_config.hpp"

namespace {
lane::LaneConfig validConfig() { return lane::makeDefaultLaneConfig(); }
}  // namespace

int main() {
    // --- 默认值：顶点传感器、10/15/3、目标 45 mm、Ki=0 ---
    {
        const lane::LaneConfig c = validConfig();
        assert(c.wall.enterMm == 100);
        assert(c.wall.exitMm == 150);
        assert(c.wall.confirmCount == 3);
        assert(c.wall.staleTimeoutMs == 200);
        assert(c.wall.minSampleIntervalMs == 15);
        assert(c.state.noWallTimeoutMs == 120);
        assert(c.state.sensorFaultTimeoutMs == 500);
        assert(c.state.tooCloseMm == 20.0);
        assert(c.state.tooCloseConfirmMs == 20);
        assert(c.state.tooCloseSpeedScale == 0.35);
        assert(c.state.tooCloseMinCorrectionMps > 0.0);
        assert(c.state.tooCloseMaxCorrectionMps >
               c.state.tooCloseMinCorrectionMps);
        assert(c.state.corridorOnly);
        assert(c.state.corridorEnableMm == 150.0);
        assert(c.state.standstillCenter);
        assert(c.state.standstillMaxCorrectionMps == 0.15);
        assert(c.state.standstillMinTooCloseMps == 0.10);
        assert(c.wallPid.ki == 0.0);
        assert(std::fabs(c.state.targetWallMm - 45.0) < 1e-12);
        assert(!c.layoutVerified);
        assert(lane::isValidLaneConfig(c));
    }

    // --- 默认映射：S0=前、S1=左、S2=后、S3=右；A=前后、B=左右 ---
    {
        const lane::LaneConfig c = validConfig();
        assert(c.sensors[lane::kS0].rangeSource == 0);
        assert(c.sensors[lane::kS1].rangeSource == 2);
        assert(c.sensors[lane::kS2].rangeSource == 1);
        assert(c.sensors[lane::kS3].rangeSource == 3);

        assert(c.sensors[lane::kS0].yM == 0.0);
        assert(c.sensors[lane::kS0].xM > 0.0);
        assert(c.sensors[lane::kS1].xM == 0.0);
        assert(c.sensors[lane::kS1].yM > 0.0);
        assert(c.sensors[lane::kS2].xM < 0.0);
        assert(c.sensors[lane::kS3].yM < 0.0);

        // 顶点半径 ≈ 0.1803 m，不是轮子边中点的 0.1275 m
        assert(std::fabs(c.sensors[lane::kS0].xM -
                         lane::kDefaultSensorRadiusM) < 1e-12);
        assert(c.sensors[lane::kS0].xM > 0.17);

        assert(c.groupMembers[lane::kGroupA][0] == lane::kS0);
        assert(c.groupMembers[lane::kGroupA][1] == lane::kS2);
        assert(c.groupMembers[lane::kGroupB][0] == lane::kS1);
        assert(c.groupMembers[lane::kGroupB][1] == lane::kS3);
    }

    // --- 阈值非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.wall.exitMm = c.wall.enterMm;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.wall.confirmCount = 0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.wall.minSampleIntervalMs = 0;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 几何/启动/速度阈值非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.minAxisAlignment = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.maxPairOppositionErrorRad = 1.6;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.targetWallMm = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.maxCorridorWidthM = c.state.minCorridorWidthM;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.faultSpeedScale = 1.5;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.noWallTimeoutMs = 0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseMm = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseMm = c.wall.enterMm + 1.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseConfirmMs = 0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.corridorEnableMm = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.corridorEnableMm = c.wall.exitMm + 1.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillMaxCorrectionMps = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillMinTooCloseMps =
            c.state.standstillMaxCorrectionMps + 0.01;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillMaxCorrectionMps =
            c.state.maxLateralCorrectionMps + 0.01;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseGainMpsPerMm = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseMinCorrectionMps = 0.20;
        c.state.tooCloseMaxCorrectionMps = 0.10;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.motionSlowMm = c.state.motionStopMm;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.sensors[lane::kS0].xM = std::nan("");
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 硬件映射重复 / 越界 ---
    {
        lane::LaneConfig c = validConfig();
        c.sensors[lane::kS0].rangeSource = c.sensors[lane::kS1].rangeSource;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.sensors[lane::kS0].rangeSource = 4;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 分组索引非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.groupMembers[lane::kGroupA][1] = lane::kS0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.groupMembers[lane::kGroupB][0] = 4;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 滤波/PID 非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.filter.lowPassAlpha = 1.5;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.wallPid.outputLimit = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.wallPid.kp = std::nan("");
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 合法的手工布局 ---
    {
        lane::LaneConfig c = validConfig();
        c.layoutVerified = true;
        c.sensors[lane::kS0] = lane::SensorConfig{0, 0.19, 0.0, 0.0, true};
        c.sensors[lane::kS2] = lane::SensorConfig{1, -0.19, 0.0, lane::kPi, true};
        c.state.targetWallMm = 50.0;
        assert(lane::isValidLaneConfig(c));
    }

    std::printf("lane config tests: ALL PASS\n");
    return 0;
}
