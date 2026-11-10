/**
 * @file bringup/friction_test.cpp
 * @brief 台架调试流程实现：静摩擦标定 + 点动自检。
 */
#include "bringup/friction_test.hpp"

#include <cmath>
#include <cstdio>

#include "core/types.hpp"
#include "pros/rtos.hpp"

namespace bringup {
namespace {

constexpr int kTrialsPerDir = 3;
constexpr double kRampRate = 0.05;          ///< omega 上升斜率 [rad/s^2]
constexpr double kMaxOmega = 0.8;           ///< 斜坡上限 [rad/s]
constexpr double kRateThreshold = 0.02;     ///< 判定“已在转”的角速度门限 [rad/s]
constexpr double kAngleThreshold = 0.005;   ///< 判定“确实转过去”的角度门限 [rad]
constexpr int kConfirmSamples = 3;          ///< 连续命中帧数（60 ms）
constexpr std::uint32_t kSettleMs = 1200;   ///< 每次试验前的静置时间
constexpr std::uint32_t kTrialTimeoutMs = 22000;
constexpr std::uint32_t kSamplePeriodMs = 20;
constexpr double kSamplePeriodSec = kSamplePeriodMs / 1000.0;
constexpr double kSettledRateRadPerSec = 0.01;  ///< 判定“已静止”的角速度门限 [rad/s]

constexpr double kPi = 3.14159265358979323846;

/// 中位数（最多 8 个样本；插入排序，样本极少无需更复杂算法）。
double median(const double *values, int n) {
    if (n <= 0) {
        return 0.0;
    }
    if (n > 8) {
        n = 8;
    }
    double tmp[8];
    for (int i = 0; i < n; ++i) {
        tmp[i] = values[i];
    }
    for (int i = 1; i < n; ++i) {
        const double v = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            --j;
        }
        tmp[j + 1] = v;
    }
    return tmp[n / 2];
}

/// 等 IMU 就绪（校准完成 + 零偏已确定），超时返回 false。
bool waitImu(imu::Imu &imu) {
    const std::uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 15000) {
        imu.update();
        const imu::ImuState &s = imu.getState();
        if (s.valid && s.biasMeasured) {
            return true;
        }
        pros::delay(kSamplePeriodMs);
    }
    return false;
}

/// 静置到车体不再转动（或超时），然后刹车。
void settle(chassis::Chassis &chassis, imu::Imu &imu) {
    const std::uint32_t t0 = pros::millis();
    int still = 0;
    while (pros::millis() - t0 < kSettleMs) {
        chassis.setCommand(core::zeroMotion());
        chassis.update(kSamplePeriodSec);
        imu.update();
        const imu::ImuState &s = imu.getState();
        if (s.valid && std::fabs(s.angularVelocityRadPerSec) <
                          kSettledRateRadPerSec) {
            if (++still >= 5) {
                break;
            }
        } else {
            still = 0;
        }
        pros::delay(kSamplePeriodMs);
    }
    chassis.stop();
}

/// 单方向斜坡试验；返回 breakaway omega 指令，未检测到返回 NAN。
double trial(chassis::Chassis &chassis, imu::Imu &imu, int dir) {
    settle(chassis, imu);
    imu.update();
    const double startRot = imu.getState().rotationRad;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t lastLog = 0;
    int confirm = 0;
    double cmd = 0.0;

    while (pros::millis() - t0 < kTrialTimeoutMs) {
        const double t = (pros::millis() - t0) / 1000.0;
        const double target = dir * kRampRate * t;
        cmd = (std::fabs(target) > kMaxOmega) ? dir * kMaxOmega : target;

        chassis.setCommand(core::MotionCommand{0.0, 0.0, cmd});
        chassis.update(kSamplePeriodSec);
        imu.update();

        const imu::ImuState &s = imu.getState();
        const double dtheta = s.rotationRad - startRot;
        const bool rate =
            std::fabs(s.angularVelocityRadPerSec) > kRateThreshold;
        const bool moved = (dtheta * dir) > kAngleThreshold;

        if (s.valid && rate && moved) {
            ++confirm;
        } else {
            confirm = 0;
        }

        const std::uint32_t now = pros::millis();
        if (now - lastLog >= 100) {
            lastLog = now;
            std::printf("$F,ramp,%u,%d,%.5f,%.5f,%.5f\n",
                        static_cast<unsigned>(now), dir, cmd,
                        s.angularVelocityRadPerSec, dtheta);
        }

        if (confirm >= kConfirmSamples) {
            const double onset =
                cmd - dir * (kConfirmSamples - 1) * kSamplePeriodSec * kRampRate;
            std::printf("$F,break,%u,%d,%.5f,%.5f,%.5f\n",
                        static_cast<unsigned>(now), dir, onset,
                        s.angularVelocityRadPerSec, dtheta);
            chassis.stop();
            return onset;
        }
        pros::delay(kSamplePeriodMs);
    }

    std::printf("$F,timeout,%u,%d,%.5f,%.5f,%.5f\n",
                static_cast<unsigned>(pros::millis()), dir, cmd,
                imu.getState().angularVelocityRadPerSec,
                imu.getState().rotationRad - startRot);
    chassis.stop();
    return NAN;
}

}  // namespace

void runFrictionTest(chassis::Chassis &chassis, imu::Imu &imu) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // 测试帧低速率，必须无缓冲
    std::printf("[FRICTION] static friction test\n");
    chassis.enableDrive = true;
    chassis.enableHeadingHold = false;
    chassis.enableSlewLimit = false;
    chassis.stop();

    if (!waitImu(imu)) {
        std::printf("[FRICTION] ERROR: IMU not ready\n");
        std::printf("[FRICTION] DONE\n");
        return;
    }

    std::printf("[FRICTION] start in 3s; keep area clear\n");
    pros::delay(3000);

    double pos[8] = {0};
    double neg[8] = {0};
    int np = 0;
    int nn = 0;

    for (int i = 0; i < kTrialsPerDir; ++i) {
        const double p = trial(chassis, imu, +1);
        if (p == p) {
            if (np < 8) {
                pos[np++] = p;
            }
        }
        const double n = trial(chassis, imu, -1);
        if (n == n) {
            if (nn < 8) {
                neg[nn++] = n;
            }
        }
    }

    const double mp = median(pos, np);
    const double mn = median(neg, nn);
    const double sym = std::fmax(std::fabs(mp), std::fabs(mn));

    // 几何/传动取自底盘配置本身，避免外部另抄一份常量（单一真值来源）。
    const chassis::ChassisConfig &cfg = chassis.config();
    const double wheelRpmPerOmega = cfg.centerToWheelM *
                                    (60.0 / (2.0 * kPi * cfg.wheelRadiusM)) *
                                    cfg.gearRatioMotorPerWheel;
    const double equivalentMv = sym * wheelRpmPerOmega * cfg.motorRpmToMillivolt;

    std::printf("[FRICTION] POS_MEDIAN=%.5f rad/s (%.2f deg/s)\n", mp,
                mp * 180.0 / kPi);
    std::printf("[FRICTION] NEG_MEDIAN=%.5f rad/s (%.2f deg/s)\n", mn,
                mn * 180.0 / kPi);
    std::printf("[FRICTION] FF_SYMMETRIC=%.5f rad/s (%.2f deg/s, %.0f mV eq)\n",
                sym, sym * 180.0 / kPi, equivalentMv);
    std::printf("[FRICTION] DONE\n");
    chassis.stop();
}

void runStaticTestSequence(chassis::Chassis &chassis) {
    struct Step {
        double vx;
        double vy;
        double omega;
        std::uint32_t ms;
        const char *name;
    };
    const Step plan[] = {
        {0.20, 0.00, 0.00, 800, "T-01 fwd"},
        {-0.20, 0.00, 0.00, 800, "T-02 back"},
        {0.00, 0.20, 0.00, 800, "T-03 left"},
        {0.00, -0.20, 0.00, 800, "T-04 right"},
        {0.00, 0.00, 0.50, 800, "T-05 ccw"},
        {0.20, 0.20, 0.50, 800, "T-06 mix"},
    };

    for (const auto &s : plan) {
        std::printf("[chassis] %s\n", s.name);
        const std::uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < s.ms) {
            chassis.setCommand(core::MotionCommand{s.vx, s.vy, s.omega});
            chassis.update(0.02);
            pros::delay(20);
        }
        chassis.stop();
        pros::delay(300);
    }
    std::printf("[chassis] demo done\n");
}

}  // namespace bringup
