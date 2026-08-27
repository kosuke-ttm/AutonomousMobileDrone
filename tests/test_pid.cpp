#include "pid_controller.hpp"

#include <gtest/gtest.h>

using evtol::PidController;

TEST(PidController, CalculateError) {
    PidController pid(1.0, 0.0, 0.0, -100.0, 100.0);
    EXPECT_DOUBLE_EQ(pid.calculateError(100.0, 40.0), 60.0);
    EXPECT_DOUBLE_EQ(pid.calculateError(50.0, 80.0), -30.0);
}

TEST(PidController, ProportionalOnly) {
    PidController pid(2.0, 0.0, 0.0, -1000.0, 1000.0);
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.01), 20.0);
}

TEST(PidController, IntegralAccumulates) {
    PidController pid(0.0, 4.0, 0.0, -1000.0, 1000.0);
    const double first = pid.update(5.0, 0.1);
    const double second = pid.update(5.0, 0.1);
    EXPECT_NEAR(first, 2.0, 1e-9);
    EXPECT_NEAR(second, 4.0, 1e-9);
}

TEST(PidController, ThrustUpperLimit) {
    // TEST-004
    PidController pid(1000.0, 0.0, 0.0, 0.0, 250.0);
    EXPECT_DOUBLE_EQ(pid.update(100.0, 0.01), 250.0);
}

TEST(PidController, ThrustLowerLimit) {
    // TEST-005
    PidController pid(1000.0, 0.0, 0.0, 0.0, 250.0);
    EXPECT_DOUBLE_EQ(pid.update(-100.0, 0.01), 0.0);
}

TEST(PidController, ResetClearsIntegrator) {
    PidController pid(0.0, 10.0, 0.0, -1000.0, 1000.0);
    pid.update(2.0, 1.0);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(2.0, 1.0), 20.0);
}
