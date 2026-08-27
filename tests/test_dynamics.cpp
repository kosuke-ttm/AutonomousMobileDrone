#include "evtol_model.hpp"

#include <gtest/gtest.h>

using evtol::EvtolModel;
using evtol::VehicleParams;

TEST(EvtolModel, HoverThrustHoldsAltitude) {
    EvtolModel model;
    model.reset(50.0, 0.0);
    const double hover = model.hoverThrust();
    for (int i = 0; i < 200; ++i) {
        model.step(hover, 0.0, 0.01);
    }
    EXPECT_NEAR(model.altitude(), 50.0, 1e-6);
    EXPECT_NEAR(model.velocity(), 0.0, 1e-6);
}

TEST(EvtolModel, ZeroThrustFallsToGround) {
    EvtolModel model;
    model.reset(5.0, 0.0);
    for (int i = 0; i < 500; ++i) {
        model.step(0.0, 0.0, 0.01);
    }
    EXPECT_DOUBLE_EQ(model.altitude(), 0.0);
    EXPECT_GE(model.velocity(), 0.0);
}

TEST(EvtolModel, ExcessThrustAcceleratesUp) {
    EvtolModel model;
    model.reset(10.0, 0.0);
    model.step(model.hoverThrust() + 50.0, 0.0, 0.01);
    EXPECT_GT(model.acceleration(), 0.0);
    EXPECT_GT(model.velocity(), 0.0);
}

TEST(EvtolModel, DisturbanceChangesAcceleration) {
    EvtolModel model;
    model.reset(20.0, 0.0);
    model.step(model.hoverThrust(), 100.0, 0.01);
    EXPECT_LT(model.acceleration(), 0.0);
}

TEST(EvtolModel, ThrustIsClampedToMaximum) {
    VehicleParams params;
    params.max_thrust_n = 200.0;
    EvtolModel model(params);
    model.reset(10.0, 0.0);
    model.step(1.0e6, 0.0, 0.01);
    const double expected_a =
        (params.max_thrust_n - params.mass_kg * params.gravity_mps2) /
        params.mass_kg;
    EXPECT_NEAR(model.acceleration(), expected_a, 1e-9);
}
