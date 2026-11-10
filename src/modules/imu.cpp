/**
 * @file modules/imu.cpp
 * @brief IMU 模块实现。
 */
#include "modules/imu.hpp"

#include <cmath>

#include "core/math_util.hpp"
#include "pros/rtos.hpp"

namespace imu {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// 状态字里的错误码。就绪 / 校准**不能**用 `get_status()==ready` 判断：
// 本内核(4.2.2) 实测校准后返回 18(0x12)，与文档枚举(ready=0/calibrating=19)
// 不符；就绪判据以 IMU 的 `is_calibrating()` 为准。
// 参考：/home/jc/VEX/moudleTest/test-gyro-port3（同一内核实测结论）。
constexpr std::int32_t kImuStatusError =
    static_cast<std::int32_t>(pros::ImuStatus::error);

// 角度漂移率测量噪声门限 [deg/s]。VEXos 的 get_rotation() 实测几乎不漂，
// 短窗口（数秒）测出的漂移率主要是量化/噪声（实测 ~±7 mdps）。若直接拿来补偿，
// 反而往角度里注入噪声。因此 |实测| 低于此门限一律视为 0（不补偿）。
constexpr double kDriftDeadbandDps = 0.02;  // 0.02 deg/s ≈ 0.4°/20s

// 是否把测得的漂移率扣到角度上。
// 默认**关闭**：VEXos 的 get_rotation() 实测本身无漂移；而一旦测量窗口内
// 车体在动，测得的“漂移率”其实是车在转，扣到 rot 上会让 rot 出现假恒定漂移，
// 航向环为“追”它会持续下旋转命令 → 车无输入自转。仅确有需要时手动打开。
constexpr bool kApplyRotationDriftComp = false;
}  // namespace

void Imu::init(const ImuConfig& config) {
    config_ = config;
    device_.emplace(config_.port);
    state_ = ImuState{};
    lastRotationRad_ = 0.0;
    hasLastRotation_ = false;
    dataRateSet_ = false;
    calibrationStartMs_ = 0;
    calibrationStarted_ = false;
    gyroBiasDps_ = 0.0;
    gyroDriftDps_ = 0.0;
    biasMeasured_ = false;
    biasSumDps_ = 0.0;
    biasRotStartDeg_ = 0.0;
    biasSampleCount_ = 0;
    biasStartMs_ = 0;
    biasRefMs_ = 0;
    initialized_ = true;
}

void Imu::startCalibration() {
    if (!device_) {
        return;
    }
    // 非阻塞：IMU 缺失 / 异常时立即返回，不卡住初始化（IMU-004）
    device_->reset(false);
    calibrationStartMs_ = pros::millis();
    calibrationStarted_ = true;
    hasLastRotation_ = false;  // 校准会重置朝向，丢弃旧基线
    dataRateSet_ = false;      // reset 后需重新下发刷新率
    biasMeasured_ = false;     // 重新测零偏
    biasSumDps_ = 0.0;
    biasRotStartDeg_ = 0.0;
    biasSampleCount_ = 0;
    biasStartMs_ = 0;
    biasRefMs_ = 0;
}

bool Imu::isReady() const {
    return device_ && !device_->is_calibrating();
}

void Imu::update() {
    if (!initialized_ || !device_) {
        state_.valid = false;
        state_.calibrating = false;
        hasLastRotation_ = false;  // 无法读取时丢弃基线，恢复后重新基准化
        return;
    }

    const std::int32_t raw = static_cast<std::int32_t>(device_->get_status());
    state_.statusRaw = raw;
    state_.calibrating = device_->is_calibrating();

    // 校准超时保护：不让 calibrating 无限持续（IMU-004）。
    if (state_.calibrating && calibrationStarted_ &&
        (pros::millis() - calibrationStartMs_) > config_.calibrationTimeoutMs) {
        state_.calibrating = false;
        state_.valid = false;
        hasLastRotation_ = false;
        return;
    }

    // 未就绪（未插 / 校准中 / 错误）→ 数据无效，不得返回 0 冒充有效（IMU-009 / T-10）
    if (!device_->is_installed() || raw == kImuStatusError || state_.calibrating) {
        state_.valid = false;
        hasLastRotation_ = false;  // 校准/错误期间朝向会重置，丢弃旧基线
        return;
    }

    // 首次就绪后下发一次刷新率（设备侧最小 5ms；共享缓存仍 ~10ms）。
    if (!dataRateSet_) {
        static_cast<void>(device_->set_data_rate(config_.dataRateMs));
        dataRateSet_ = true;
    }

    const pros::imu_gyro_s_t gyro = device_->get_gyro_rate();
    const double rotationDeg = device_->get_rotation();
    const double headingDeg = device_->get_heading();
    if (!std::isfinite(gyro.z) || !std::isfinite(rotationDeg) ||
        !std::isfinite(headingDeg)) {
        state_.valid = false;
        hasLastRotation_ = false;
        return;
    }

    // ---- 静置漂移补偿：确定角速度零偏 + 角度漂移率（手动常量 或 启动测一次）----
    if (!biasMeasured_) {
        if (config_.useManualGyroBias) {
            gyroBiasDps_ = config_.gyroBiasDps;
            gyroDriftDps_ = config_.gyroDriftDps;
            biasMeasured_ = true;
            biasRefMs_ = pros::millis();
        } else {
            if (biasSampleCount_ == 0) {
                biasStartMs_ = pros::millis();
                biasSumDps_ = 0.0;
                biasRotStartDeg_ = rotationDeg;
            }
            biasSumDps_ += gyro.z;
            ++biasSampleCount_;
            const std::uint32_t elapsed = pros::millis() - biasStartMs_;
            if (elapsed >= config_.gyroBiasMeasureMs) {
                gyroBiasDps_ = biasSumDps_ / static_cast<double>(biasSampleCount_);
                // 角度漂移率必须实测 get_rotation() 本身（VEXos 已做过零偏补偿）；
                // 低于噪声门限则视为无漂移，避免把噪声当补偿注入。
                const double measuredDrift =
                    (rotationDeg - biasRotStartDeg_) /
                    (static_cast<double>(elapsed) / 1000.0);
                gyroDriftDps_ = (std::fabs(measuredDrift) < kDriftDeadbandDps)
                                    ? 0.0
                                    : measuredDrift;
                biasMeasured_ = true;
                biasRefMs_ = pros::millis();
            }
        }
    }

    // 按实测符号把原始航向换算到工程约定（当前 yawSign=+1.0，逆时针为正）；
    // 再加上安装偏航角：车头相对 IMU +x 的固定夹角（当前 +45°）。
    // 角速度 gyro.z 是绕同一竖直轴的速率，不需要加这个常值偏置。
    const double yawMountRad = config_.mountingYawOffsetRad;
    double rotationRad = rotationDeg * config_.yawSign * kDegToRad + yawMountRad;
    const double headingRad =
        core::wrapToPi(headingDeg * config_.yawSign * kDegToRad + yawMountRad);

    // 静置漂移补偿（默认关闭，见 kApplyRotationDriftComp）：
    // 若把测得的漂移率扣到 rot 上，测量偏差会让 rot 假漂移，导致持续自转。
    if (kApplyRotationDriftComp && biasMeasured_ && biasRefMs_ != 0) {
        const double dtSec = (pros::millis() - biasRefMs_) / 1000.0;
        rotationRad -= gyroDriftDps_ * kDegToRad * config_.yawSign * dtSec;
    }

    // 单周期跳变保护：本帧丢弃异常跳变，并把基准移到新值，
    // 避免校准复位 / 短暂尖峰后基准过期而永久卡在无效（IMU-009）。
    if (hasLastRotation_ &&
        std::fabs(rotationRad - lastRotationRad_) > config_.maxDeltaYawPerCycleRad) {
        state_.valid = false;
        lastRotationRad_ = rotationRad;
        return;
    }

    state_.rotationRad = rotationRad;
    state_.headingRad = headingRad;
    state_.angularVelocityRadPerSec =
        (gyro.z - gyroBiasDps_) * config_.yawSign * kDegToRad;
    state_.gyroBiasDps = gyroBiasDps_;
    state_.gyroDriftDps = gyroDriftDps_;
    state_.biasMeasured = biasMeasured_;
    state_.valid = true;
    state_.timestampMs = pros::millis();

    lastRotationRad_ = rotationRad;
    hasLastRotation_ = true;
}

}  // namespace imu
