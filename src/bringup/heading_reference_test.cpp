#include "bringup/heading_reference_test.hpp"

#include "pros/rtos.hpp"

namespace bringup {
void prepareHeadingReference(imu::Imu& imu, diag::Telemetry& telemetry,
                             const HeadingReferenceTestConfig& config) {
    const std::uint32_t start = pros::millis();
    while (!imu.getState().startup.referenceReady &&
           pros::millis() - start < config.prepareTimeoutMs) {
        imu.update();
        telemetry.update();
        pros::delay(config.samplePeriodMs);
    }
}

void runHeadingReferenceTest(chassis::Chassis& chassis, imu::Imu& imu,
                             diag::Telemetry& telemetry,
                             const HeadingReferenceTestConfig& config) {
    chassis.stop();
    std::uint32_t tick = pros::millis();
    while (true) {
        imu.update();
        telemetry.update();
        pros::Task::delay_until(&tick, config.samplePeriodMs);
    }
}
}  // namespace bringup
