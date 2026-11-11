#include "imu/startup_heading.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
void feed(imu::StartupHeading& h, unsigned from, unsigned to,
          double angle = 0.0, double gyro = 0.01) {
    for (unsigned t = from; t <= to; t += 20) h.update(angle, gyro, true, t);
}
bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }
constexpr double kPi = 3.14159265358979323846;
}

int main() {
    auto config = imu::makeDefaultImuConfig();
    imu::StartupHeading h;
    h.init(config, 0);
    feed(h, 0, 1980);
    assert(!h.getState().biasMeasured);
    feed(h, 2000, 4980);
    assert(h.getState().biasMeasured);
    assert(near(h.getState().gyroBiasDps, 0.01));
    assert(!h.getState().referenceReady);
    feed(h, 5000, 5000);
    assert(h.getState().referenceReady);
    assert(h.getState().latchedMs == 5000);
    const double original = h.getState().initialHeadingRad;
    h.update(90, 10, true, 5020);
    assert(near(h.getState().initialHeadingRad, original));
    assert(near(h.getState().relativeHeadingRad, kPi / 2));
    // 失效不能输出有效相对航向，也不能偷偷重设固定基准。
    h.update(90, 0, false, 5040);
    assert(h.getState().referenceReady && !h.getState().relativeValid);
    h.update(0, 0, true, 5060);
    assert(h.getState().relativeValid);
    assert(near(h.getState().relativeHeadingRad, 0));
    assert(near(h.getState().initialHeadingRad, original));

    // 零偏已完成但恰在 5s 时转动：仍需重新等满静止窗口。
    h.init(config, 0);
    feed(h, 0, 4980);
    h.update(20, 10, true, 5000);
    feed(h, 5020, 5500, 20);
    assert(!h.getState().referenceReady);
    feed(h, 5520, 5520, 20);
    assert(h.getState().referenceReady && h.getState().latchedMs == 5520);

    // 晚就绪必须积满连续 2s，不能只看开机已过 5s。
    h.init(config, 0);
    h.update(0, 0, false, 5000);
    feed(h, 6000, 7980);
    assert(!h.getState().referenceReady);
    feed(h, 8000, 8000);
    assert(h.getState().referenceReady && h.getState().latchedMs == 8000);

    // 旋转会重启采样，避免把真实旋转当零偏。
    h.init(config, 0);
    feed(h, 0, 1500);
    h.update(10, 15, true, 1520);
    feed(h, 1540, 3520, 10);
    assert(!h.getState().biasMeasured);
    feed(h, 3540, 5000, 10);
    assert(h.getState().referenceReady);
    assert(near(h.getState().gyroBiasDps, 0.01));

    // 小角速度但角度持续移动，同样不能冒充静止。
    h.init(config, 0);
    for (unsigned t = 0; t <= 6000; t += 20) h.update(t * 0.001, 0.01, true, t);
    assert(!h.getState().biasMeasured && !h.getState().referenceReady);

    // 丢样不能把时间间隙计作连续采样。
    h.init(config, 0);
    feed(h, 0, 1000);
    feed(h, 6000, 7980);
    assert(!h.getState().referenceReady);
    feed(h, 8000, 8000);
    assert(h.getState().referenceReady);

    // 漂移诊断遵循配置门限，不能偷偷使用流程中的固定值。
    config.gyroDriftDeadbandDps = 0.04;
    h.init(config, 0);
    for (unsigned t = 0; t <= 2000; t += 20) h.update(t * 0.00003, 0, true, t);
    assert(h.getState().biasMeasured);
    assert(near(h.getState().gyroDriftDps, 0));
    config.gyroDriftDeadbandDps = 0.02;
    h.init(config, 0);
    for (unsigned t = 0; t <= 2000; t += 20) h.update(t * 0.00003, 0, true, t);
    assert(near(h.getState().gyroDriftDps, 0.03));

    // 人工零偏仍需静止窗口；跨零后相对角度连续。
    config.useManualGyroBias = true;
    h.init(config, 0);
    feed(h, 5000, 5480, 179);
    assert(!h.getState().referenceReady);
    feed(h, 5500, 5500, 179);
    assert(h.getState().referenceReady);
    h.update(181, 0, true, 5520);
    assert(near(h.getState().relativeHeadingRad, 2 * kPi / 180));
    h.update(std::numeric_limits<double>::quiet_NaN(), 0, true, 5540);
    assert(!h.getState().relativeValid);
    h.init(config, 5560);
    assert(!h.getState().referenceReady);
    std::puts("startup heading tests: ALL PASS");
}
