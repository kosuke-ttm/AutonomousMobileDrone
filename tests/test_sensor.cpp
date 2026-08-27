#include "altitude_sensor.hpp"

#include <gtest/gtest.h>

using evtol::AltitudeSensor;

TEST(AltitudeSensor, ZeroNoiseReturnsTrueAltitude) {
    AltitudeSensor sensor(0.0, 1);
    EXPECT_DOUBLE_EQ(sensor.measure(12.5), 12.5);
    EXPECT_DOUBLE_EQ(sensor.measure(0.0), 0.0);
}

TEST(AltitudeSensor, NoiseIsDeterministic) {
    AltitudeSensor a(0.5, 42);
    AltitudeSensor b(0.5, 42);
    for (int i = 0; i < 20; ++i) {
        EXPECT_DOUBLE_EQ(a.measure(10.0), b.measure(10.0));
    }
}

TEST(AltitudeSensor, DifferentSeedsDiffer) {
    AltitudeSensor a(1.0, 1);
    AltitudeSensor b(1.0, 2);
    EXPECT_NE(a.measure(0.0), b.measure(0.0));
}
