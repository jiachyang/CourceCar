/**
 * @file tests/native/test_lane_config.cpp
 * @brief v2.1 LaneConfig 默认值与静态校验。
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "lane/lane_config.hpp"

namespace {
lane::LaneConfig validConfig() { return lane::makeDefaultLaneConfig(); }
}  // namespace

int main() {
    // --- 默认值：近壁 100/150、MID 可初始化、PD、走廊宽度、过近 ---
    {
        const lane::LaneConfig c = validConfig();
        assert(std::fabs(c.channel.enterMm - 100.0) < 1e-12);
        assert(std::fabs(c.channel.exitMm - 150.0) < 1e-12);
        assert(std::fabs(c.channel.detectMm - 150.0) < 1e-12);
        assert(c.channel.confirmFrames == 3);
        assert(c.channel.maxAgeMs == 180);
        assert(c.channel.freshWindowMs == 42);
        assert(std::fabs(c.channel.jumpMm - 60.0) < 1e-12);
        assert(c.channel.jumpConfirm == 2);
        assert(std::fabs(c.channel.lowPassAlpha - 0.45) < 1e-12);

        assert(std::fabs(c.pd.kp - 0.0010) < 1e-12);
        assert(c.pd.ki == 0.0);
        assert(std::fabs(c.pd.kd - 0.000025) < 1e-12);
        assert(std::fabs(c.pd.outputLimit - 0.14) < 1e-12);
        assert(std::fabs(c.pd.derivativeFilterAlpha - 0.12) < 1e-12);

        assert(c.state.degradedHoldMs == 150);
        assert(std::fabs(c.state.minCorridorWidthM - 0.37) < 1e-12);
        assert(std::fabs(c.state.maxCorridorWidthM - 0.70) < 1e-12);
        assert(std::fabs(c.state.tooCloseMm - 20.0) < 1e-12);
        assert(std::fabs(c.state.repulseMps - 0.10) < 1e-12);
        assert(std::fabs(c.state.maxCorrectionMps - 0.14) < 1e-12);
        assert(std::fabs(c.state.maxLateralAccelMps2 - 0.70) < 1e-12);
        assert(std::fabs(c.state.minAxisAlignment - 0.82) < 1e-12);
        assert(std::fabs(c.state.standstillMinCorrectionMps - 0.08) < 1e-12);
        assert(std::fabs(c.state.standstillDeadbandMm - 3.0) < 1e-12);

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

        assert(std::fabs(c.sensors[lane::kS0].xM -
                         lane::kDefaultSensorRadiusM) < 1e-12);
        assert(c.sensors[lane::kS0].xM > 0.17);

        assert(c.groupMembers[lane::kGroupA][0] == lane::kS0);
        assert(c.groupMembers[lane::kGroupA][1] == lane::kS2);
        assert(c.groupMembers[lane::kGroupB][0] == lane::kS1);
        assert(c.groupMembers[lane::kGroupB][1] == lane::kS3);
    }

    // --- 近壁/滤波阈值非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.channel.exitMm = c.channel.enterMm;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.channel.confirmFrames = 0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.channel.freshWindowMs = c.channel.maxAgeMs + 1;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.channel.lowPassAlpha = 1.5;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.channel.detectMm = c.channel.maxValidMm + 1.0;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- PD 非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.pd.outputLimit = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.pd.kp = std::nan("");
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.pd.derivativeFilterAlpha = 1.5;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 走廊/过近/限幅非法 ---
    {
        lane::LaneConfig c = validConfig();
        c.state.maxCorridorWidthM = c.state.minCorridorWidthM;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.tooCloseMm = c.channel.enterMm;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.repulseMps = c.state.maxCorrectionMps + 0.01;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.maxCorrectionMps = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.maxLateralAccelMps2 = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.minAxisAlignment = 0.0;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.minDirectionSpeedMps = -0.1;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillMinCorrectionMps = -0.01;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillMinCorrectionMps =
            c.state.maxCorrectionMps + 0.01;
        assert(!lane::isValidLaneConfig(c));
    }
    {
        lane::LaneConfig c = validConfig();
        c.state.standstillDeadbandMm = -1.0;
        assert(!lane::isValidLaneConfig(c));
    }

    // --- 传感器映射重复 / 越界 / 非有限 ---
    {
        lane::LaneConfig c = validConfig();
        c.sensors[lane::kS0].xM = std::nan("");
        assert(!lane::isValidLaneConfig(c));
    }
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

    // --- 合法的手工布局 ---
    {
        lane::LaneConfig c = validConfig();
        c.layoutVerified = true;
        c.sensors[lane::kS0] = lane::SensorConfig{0, 0.19, 0.0, 0.0, true};
        c.sensors[lane::kS2] = lane::SensorConfig{1, -0.19, 0.0, lane::kPi, true};
        c.sensors[lane::kS1] = lane::SensorConfig{2, 0.0, 0.19, lane::kPi / 2.0, true};
        c.sensors[lane::kS3] = lane::SensorConfig{3, 0.0, -0.19, -lane::kPi / 2.0, true};
        assert(lane::isValidLaneConfig(c));
    }

    std::printf("lane config tests: ALL PASS\n");
    return 0;
}
