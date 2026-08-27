#include "flight_state.hpp"

#include <cmath>

namespace evtol {

namespace {
constexpr double kGroundAltM = 0.2;
constexpr double kTakeoffAltM = 8.0;
constexpr double kLandingAltM = 5.0;
constexpr double kClimbVelMps = 0.6;
constexpr double kDescendVelMps = -0.6;
constexpr double kHoverVelMps = 0.5;
constexpr double kHoverErrorM = 3.0;
}  // namespace

const char* flightStateName(FlightState state) {
    switch (state) {
        case FlightState::Ground:
            return "GROUND";
        case FlightState::Takeoff:
            return "TAKEOFF";
        case FlightState::Climb:
            return "CLIMB";
        case FlightState::Hover:
            return "HOVER";
        case FlightState::Descend:
            return "DESCEND";
        case FlightState::Landing:
            return "LANDING";
    }
    return "GROUND";
}

FlightStateEstimator::FlightStateEstimator() : state_(FlightState::Ground) {}

FlightState FlightStateEstimator::update(double altitude_m, double velocity_mps,
                                         double target_altitude_m,
                                         double altitude_error_m) {
    const double abs_error = std::abs(altitude_error_m);
    const bool near_target = abs_error <= kHoverErrorM;
    const bool slow = std::abs(velocity_mps) <= kHoverVelMps;

    if (altitude_m <= kGroundAltM && std::abs(velocity_mps) <= 0.15) {
        state_ = FlightState::Ground;
        return state_;
    }

    if (altitude_m <= kLandingAltM && velocity_mps < kDescendVelMps &&
        target_altitude_m <= kLandingAltM) {
        state_ = FlightState::Landing;
        return state_;
    }

    if (near_target && slow) {
        state_ = FlightState::Hover;
        return state_;
    }

    if (altitude_m < kTakeoffAltM && velocity_mps > 0.2 &&
        (state_ == FlightState::Ground || state_ == FlightState::Takeoff)) {
        state_ = FlightState::Takeoff;
        return state_;
    }

    if (velocity_mps <= kDescendVelMps) {
        state_ = FlightState::Descend;
        return state_;
    }

    if (velocity_mps >= kClimbVelMps) {
        state_ = FlightState::Climb;
        return state_;
    }

    if (state_ == FlightState::Ground && altitude_m > kGroundAltM) {
        state_ = FlightState::Takeoff;
    }

    return state_;
}

FlightState FlightStateEstimator::current() const { return state_; }

void FlightStateEstimator::reset() { state_ = FlightState::Ground; }

}  // namespace evtol
