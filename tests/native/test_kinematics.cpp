/**
 * @file test_kinematics.cpp
 * @brief IK 理论验证（主机端，不依赖 PROS）。对应验收 T-01~T-05 的理论部分。
 *
 * 编译运行：bash scripts/test_native.sh
 */
#include <cassert>
#include <cmath>
#include <cstdio>

#include "chassis/kinematics.hpp"

using namespace chassis;

static bool approx(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) < eps;
}

int main() {
    constexpr double kPi = 3.14159265358979323846;
    const double inv = 1.0 / std::sqrt(2.0);
    double rpm[kWheelCount];

    // T-01/T-02：前进时 FL,RL 与 FR,RR 反向且大小相等
    bodyVelocityToWheelRpm(1.0, 0.0, 0.0, rpm);
    assert(rpm[kFl] < 0.0 && rpm[kFr] > 0.0 && rpm[kRl] < 0.0 && rpm[kRr] > 0.0);
    assert(approx(rpm[kFl], -rpm[kFr]) && approx(rpm[kRl], -rpm[kRr]));

    // T-03/T-04：左移时前排同号、后排同号、前后相反
    bodyVelocityToWheelRpm(0.0, 1.0, 0.0, rpm);
    assert(rpm[kFl] > 0.0 && rpm[kFr] > 0.0 && rpm[kRl] < 0.0 && rpm[kRr] < 0.0);

    // T-05：原地逆时针旋转时四轮同向同速
    bodyVelocityToWheelRpm(0.0, 0.0, 1.0, rpm);
    assert(rpm[kFl] > 0.0);
    assert(approx(rpm[kFl], rpm[kFr]));
    assert(approx(rpm[kFl], rpm[kRl]));
    assert(approx(rpm[kFl], rpm[kRr]));

    // 量级检查：前进 1 m/s 时单轮转速 = (1/√2)·(60/2π)/r
    bodyVelocityToWheelRpm(1.0, 0.0, 0.0, rpm);
    const double expect = inv * (60.0 / (2.0 * kPi)) / kWheelRadiusM;
    assert(approx(rpm[kFr], expect, 1e-6));

    // T-06：三输入叠加 = 三个单项之和（线性性）
    double a[kWheelCount], b[kWheelCount], c[kWheelCount], s[kWheelCount];
    bodyVelocityToWheelRpm(0.5, 0.0, 0.0, a);
    bodyVelocityToWheelRpm(0.0, 0.2, 0.0, b);
    bodyVelocityToWheelRpm(0.0, 0.0, 0.5, c);
    bodyVelocityToWheelRpm(0.5, 0.2, 0.5, s);
    for (int i = 0; i < kWheelCount; ++i) {
        assert(approx(s[i], a[i] + b[i] + c[i], 1e-6));
    }

    std::printf("kinematics tests: ALL PASS\n");
    return 0;
}
