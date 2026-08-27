#include "flight_state.hpp"

#include <gtest/gtest.h>

using evtol::FlightState;
using evtol::FlightStateEstimator;
using evtol::flightStateName;

TEST(FlightState, NamesMatchSpec) {
    EXPECT_STREQ(flightStateName(FlightState::Ground), "GROUND");
    EXPECT_STREQ(flightStateName(FlightState::Takeoff), "TAKEOFF");
    EXPECT_STREQ(flightStateName(FlightState::Climb), "CLIMB");
    EXPECT_STREQ(flightStateName(FlightState::Hover), "HOVER");
    EXPECT_STREQ(flightStateName(FlightState::Descend), "DESCEND");
    EXPECT_STREQ(flightStateName(FlightState::Landing), "LANDING");
}

TEST(FlightState, GroundWhenOnPad) {
    FlightStateEstimator est;
    EXPECT_EQ(est.update(0.0, 0.0, 0.0, 0.0), FlightState::Ground);
}

TEST(FlightState, TakeoffAfterLeavingPad) {
    FlightStateEstimator est;
    EXPECT_EQ(est.update(0.05, 0.0, 100.0, 100.0), FlightState::Ground);
    EXPECT_EQ(est.update(1.2, 1.5, 100.0, 98.8), FlightState::Takeoff);
}

TEST(FlightState, ClimbWithPositiveVelocity) {
    FlightStateEstimator est;
    est.update(0.05, 0.0, 100.0, 100.0);
    EXPECT_EQ(est.update(20.0, 4.0, 100.0, 80.0), FlightState::Climb);
}

TEST(FlightState, HoverNearTarget) {
    FlightStateEstimator est;
    EXPECT_EQ(est.update(100.0, 0.1, 100.0, 0.1), FlightState::Hover);
}

TEST(FlightState, DescendWithNegativeVelocity) {
    FlightStateEstimator est;
    est.update(100.0, 0.0, 100.0, 0.0);
    EXPECT_EQ(est.update(80.0, -2.0, 50.0, -30.0), FlightState::Descend);
}

TEST(FlightState, LandingNearGround) {
    FlightStateEstimator est;
    EXPECT_EQ(est.update(2.0, -1.5, 0.0, -2.0), FlightState::Landing);
}
